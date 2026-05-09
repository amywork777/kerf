//! Bill of Materials (BOM): flatten an assembly tree into a per-part table.
//!
//! An [`BomInput`] is a lightweight structure describing placed parts or
//! sub-assemblies. [`assembly_bom`] walks the tree and returns one
//! [`BomEntry`] per unique source model, with quantities multiplied through
//! the hierarchy.
//!
//! Volume is computed by evaluating the Model's last feature via
//! `Model::evaluate`. Mass = volume_mm3 * density_g_per_mm3 (default 1.0
//! g/cm³ = 0.001 g/mm³).

use std::collections::HashMap;

use kerf_brep::measure::solid_volume;
use serde::{Deserialize, Serialize};

use crate::model::Model;

// ---------------------------------------------------------------------------
// Public data types
// ---------------------------------------------------------------------------

/// One row in the BOM table.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct BomEntry {
    /// Part name — from the Model's `name` field if present, else the
    /// Model's id (its last feature id or a fallback string).
    pub name: String,
    /// Total instance count across the entire assembly tree (post-flattening:
    /// sub-assembly multiplier applied).
    pub quantity: u32,
    /// Material name, from the Model's `material` field or "—" if absent.
    pub material: String,
    /// Per-instance volume in mm³ (computed via solid_volume).
    pub volume_each: f64,
    /// Total volume = quantity * volume_each.
    pub volume_total: f64,
    /// Per-instance mass in grams = volume_each * density_g_per_mm3.
    pub mass_each: f64,
    /// Total mass = quantity * mass_each.
    pub mass_total: f64,
    /// Hierarchical depth (root assembly instances = 0, sub-assembly
    /// instances = 1, …).
    pub depth: u32,
}

/// One placed part inside an assembly.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct BomInstance {
    /// Unique id for this instance within the assembly.
    pub id: String,
    /// Source model. Could be a leaf part or a sub-assembly.
    pub component: BomComponent,
    /// How many times this instance is repeated (e.g., 4 bolts of the same
    /// part — leave at 1 if each bolt is listed separately).
    #[serde(default = "default_count")]
    pub count: u32,
}

fn default_count() -> u32 {
    1
}

/// Either a leaf part (a Model) or a nested sub-assembly.
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "lowercase")]
pub enum BomComponent {
    /// A leaf part: a self-contained Model whose last feature is evaluated
    /// to produce the solid.
    Part {
        /// Logical model id for deduplication. Two instances with the same
        /// `model_id` share a BOM row (quantities are summed).
        model_id: String,
        /// The actual model. For JSON round-trips this is inline.
        model: Box<Model>,
    },
    /// A sub-assembly — recursed into with an increased depth.
    SubAssembly {
        assembly: Box<BomInput>,
    },
}

/// An assembly: a list of placed instances (parts or sub-assemblies).
#[derive(Clone, Debug, Serialize, Deserialize, Default)]
pub struct BomInput {
    /// Optional display name for this assembly.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub name: Option<String>,
    /// Placed parts / sub-assemblies.
    #[serde(default)]
    pub instances: Vec<BomInstance>,
}

impl BomInput {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn with_name(mut self, name: impl Into<String>) -> Self {
        self.name = Some(name.into());
        self
    }

    /// Add a leaf part instance.
    pub fn add_part(
        mut self,
        instance_id: impl Into<String>,
        model_id: impl Into<String>,
        model: Model,
        count: u32,
    ) -> Self {
        self.instances.push(BomInstance {
            id: instance_id.into(),
            component: BomComponent::Part {
                model_id: model_id.into(),
                model: Box::new(model),
            },
            count,
        });
        self
    }

    /// Add a sub-assembly instance.
    pub fn add_sub_assembly(
        mut self,
        instance_id: impl Into<String>,
        assembly: BomInput,
        count: u32,
    ) -> Self {
        self.instances.push(BomInstance {
            id: instance_id.into(),
            component: BomComponent::SubAssembly {
                assembly: Box::new(assembly),
            },
            count,
        });
        self
    }
}

// ---------------------------------------------------------------------------
// BOM algorithm
// ---------------------------------------------------------------------------

/// Compute the BOM for an assembly tree.
///
/// Returns one [`BomEntry`] per unique `model_id`, sorted by appearance
/// order. Quantities are totalled across all instances (with sub-assembly
/// multipliers applied). Volume/mass are per-instance values (identical for
/// all copies of the same model).
pub fn assembly_bom(assembly: &BomInput) -> Vec<BomEntry> {
    // Volume cache: model_id -> volume_mm3
    let mut vol_cache: HashMap<String, f64> = HashMap::new();
    // Accumulate rows: model_id -> (name, material, density, volume_each, quantity, depth)
    let mut rows: Vec<(String, BomRowAccum)> = Vec::new();
    let mut row_index: HashMap<String, usize> = HashMap::new();

    collect_bom(
        assembly,
        1,      // multiplier
        0,      // depth
        &mut vol_cache,
        &mut rows,
        &mut row_index,
    );

    rows.into_iter()
        .map(|(_, accum)| {
            let volume_total = accum.volume_each * accum.quantity as f64;
            let mass_each = accum.volume_each * accum.density_g_per_mm3;
            let mass_total = mass_each * accum.quantity as f64;
            BomEntry {
                name: accum.name,
                quantity: accum.quantity,
                material: accum.material,
                volume_each: accum.volume_each,
                volume_total,
                mass_each,
                mass_total,
                depth: accum.depth,
            }
        })
        .collect()
}

struct BomRowAccum {
    name: String,
    material: String,
    density_g_per_mm3: f64,
    volume_each: f64,
    quantity: u32,
    depth: u32,
}

fn collect_bom(
    assembly: &BomInput,
    multiplier: u32,
    depth: u32,
    vol_cache: &mut HashMap<String, f64>,
    rows: &mut Vec<(String, BomRowAccum)>,
    row_index: &mut HashMap<String, usize>,
) {
    for inst in &assembly.instances {
        let count = inst.count.max(1) * multiplier;
        match &inst.component {
            BomComponent::Part { model_id, model } => {
                let vol = *vol_cache.entry(model_id.clone()).or_insert_with(|| {
                    compute_model_volume(model)
                });

                let name = model
                    .name
                    .clone()
                    .unwrap_or_else(|| {
                        // Fall back to last feature id, then model_id
                        model
                            .ids()
                            .last()
                            .map(|s| s.to_string())
                            .unwrap_or_else(|| model_id.clone())
                    });
                let material = model
                    .material
                    .clone()
                    .unwrap_or_else(|| "\u{2014}".to_string()); // em-dash
                let density = model.density_g_per_cm3.unwrap_or(1.0) * 0.001; // cm3 -> mm3

                if let Some(&idx) = row_index.get(model_id) {
                    rows[idx].1.quantity += count;
                } else {
                    let idx = rows.len();
                    row_index.insert(model_id.clone(), idx);
                    rows.push((
                        model_id.clone(),
                        BomRowAccum {
                            name,
                            material,
                            density_g_per_mm3: density,
                            volume_each: vol,
                            quantity: count,
                            depth,
                        },
                    ));
                }
            }
            BomComponent::SubAssembly { assembly: sub } => {
                collect_bom(sub, count, depth + 1, vol_cache, rows, row_index);
            }
        }
    }
}

/// Evaluate the model's last feature to get volume in mm³.
/// Returns 0.0 on failure.
fn compute_model_volume(model: &Model) -> f64 {
    let target = match model.ids().last() {
        Some(id) => id.to_string(),
        None => return 0.0,
    };
    match model.evaluate(&target) {
        Ok(solid) => solid_volume(&solid),
        Err(_) => 0.0,
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::feature::Feature;
    use crate::scalar::Scalar;

    fn box_model(id: &str) -> Model {
        Model::new().add(Feature::Box {
            id: id.to_string(),
            extents: [Scalar::Lit(10.0), Scalar::Lit(10.0), Scalar::Lit(10.0)],
        })
    }

    fn box_model_named(
        model_name: &str,
        feat_id: &str,
        material: Option<&str>,
        density: Option<f64>,
    ) -> Model {
        let mut m = Model::new().add(Feature::Box {
            id: feat_id.to_string(),
            extents: [Scalar::Lit(10.0), Scalar::Lit(10.0), Scalar::Lit(10.0)],
        });
        m.name = Some(model_name.to_string());
        m.material = material.map(|s| s.to_string());
        m.density_g_per_cm3 = density;
        m
    }

    #[test]
    fn single_part_assembly_has_one_entry_qty_one() {
        let assy = BomInput::new().add_part("inst1", "part_a", box_model("body"), 1);
        let bom = assembly_bom(&assy);
        assert_eq!(bom.len(), 1);
        assert_eq!(bom[0].quantity, 1);
        assert_eq!(bom[0].depth, 0);
    }

    #[test]
    fn two_instances_of_same_part_dedup_to_qty_two() {
        let assy = BomInput::new()
            .add_part("inst1", "part_a", box_model("body"), 1)
            .add_part("inst2", "part_a", box_model("body"), 1);
        let bom = assembly_bom(&assy);
        assert_eq!(bom.len(), 1);
        assert_eq!(bom[0].quantity, 2);
    }

    #[test]
    fn sub_assembly_multiplies_quantities() {
        // Sub-assembly has 3 instances of part_x; it's placed twice in root.
        let sub = BomInput::new()
            .add_part("s1", "part_x", box_model("b"), 1)
            .add_part("s2", "part_x", box_model("b"), 1)
            .add_part("s3", "part_x", box_model("b"), 1);
        let root = BomInput::new()
            .add_sub_assembly("sub_inst1", sub.clone(), 1)
            .add_sub_assembly("sub_inst2", sub, 1);
        let bom = assembly_bom(&root);
        assert_eq!(bom.len(), 1);
        assert_eq!(bom[0].quantity, 6); // 3 * 2
        assert_eq!(bom[0].depth, 1);
    }

    #[test]
    fn material_and_name_and_density_appear_in_bom() {
        let model = box_model_named("Steel Bracket", "body", Some("Steel"), Some(7.85));
        let assy = BomInput::new().add_part("i1", "bracket", model, 1);
        let bom = assembly_bom(&assy);
        assert_eq!(bom[0].name, "Steel Bracket");
        assert_eq!(bom[0].material, "Steel");
        // volume = 10^3 = 1000 mm³; density = 7.85 g/cm³ = 0.00785 g/mm³
        let expected_mass = 1000.0 * 7.85 * 0.001;
        assert!((bom[0].mass_each - expected_mass).abs() < 1e-6);
    }

    #[test]
    fn missing_material_shows_dash_placeholder() {
        let model = box_model("body");
        let assy = BomInput::new().add_part("i1", "part_b", model, 1);
        let bom = assembly_bom(&assy);
        assert_eq!(bom[0].material, "\u{2014}"); // em-dash
    }

    #[test]
    fn volume_total_equals_quantity_times_volume_each() {
        let assy = BomInput::new().add_part("i1", "p", box_model("b"), 3);
        let bom = assembly_bom(&assy);
        assert_eq!(bom[0].quantity, 3);
        let expected_vol = bom[0].volume_each * 3.0;
        assert!((bom[0].volume_total - expected_vol).abs() < 1e-9);
    }
}
