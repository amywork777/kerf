//! Model: a DAG of features keyed by id.

use std::collections::{BTreeMap, HashMap};
use std::fs::File;
use std::io::{BufReader, BufWriter, Read, Write};
use std::path::Path;

use serde::{Deserialize, Serialize};
use thiserror::Error;

use crate::feature::Feature;
use crate::geometric::GeometricConstraint;

#[derive(Debug, Error)]
pub enum ModelError {
    #[error("duplicate feature id: {0}")]
    DuplicateId(String),
    #[error("unknown feature id: {0}")]
    UnknownId(String),
    #[error("cycle detected involving id: {0}")]
    Cycle(String),
    #[error("equation '{0}' evaluation error: {1}")]
    ExprError(String, String),
    #[error("json error: {0}")]
    Json(#[from] serde_json::Error),
    #[error("io error: {0}")]
    Io(#[from] std::io::Error),
}

/// A declarative CAD model. Insertion order is preserved (so JSON round-trips
/// keep their authored order) but lookup is by id.
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct Model {
    /// Named numeric parameters referenced by `Scalar::Param("$name")` in
    /// feature fields. Empty by default.
    #[serde(default, skip_serializing_if = "HashMap::is_empty")]
    pub parameters: HashMap<String, f64>,
    /// Part-level geometric constraints between feature outputs. The solver
    /// runs after parameter resolution and adjusts top-level parameters to
    /// satisfy. Empty by default.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub geometric_constraints: Vec<GeometricConstraint>,
    /// Features keyed by id, kept in insertion order so the JSON/file form
    /// matches what the user wrote.
    pub(crate) features: Vec<Feature>,
    /// Cached id -> index lookup. Recomputed after deserialize.
    #[serde(skip)]
    index: HashMap<String, usize>,
}

impl Model {
    pub fn new() -> Self {
        Self::default()
    }

    /// Add a feature. Panics if the id is already present (use `try_add` for
    /// the non-panicking variant). Returns self for fluent chaining.
    pub fn add(mut self, f: Feature) -> Self {
        self = self.try_add(f).expect("duplicate feature id");
        self
    }

    pub fn with_parameter(mut self, name: impl Into<String>, value: f64) -> Self {
        self.parameters.insert(name.into(), value);
        self
    }

    /// Add or replace an equation. Returns self for fluent chaining.
    pub fn with_equation(mut self, name: impl Into<String>, expr: impl Into<String>) -> Self {
        self.equations.insert(name.into(), expr.into());
        self
    }

    /// Resolve all equations and return the effective parameter map.
    /// If `equations` is empty, returns a reference-counted clone of `parameters`.
    /// Otherwise performs dependency analysis + topological evaluation.
    pub fn resolve_params(&self) -> Result<std::collections::HashMap<String, f64>, ModelError> {
        crate::equations::resolve_equations(&self.equations, &self.parameters)
    }

    pub fn try_add(mut self, f: Feature) -> Result<Self, ModelError> {
        let id = f.id().to_string();
        if self.index.contains_key(&id) {
            return Err(ModelError::DuplicateId(id));
        }
        self.index.insert(id, self.features.len());
        self.features.push(f);
        Ok(self)
    }

    pub fn feature(&self, id: &str) -> Option<&Feature> {
        self.index.get(id).map(|i| &self.features[*i])
    }

    pub fn ids(&self) -> impl Iterator<Item = &str> {
        self.features.iter().map(|f| f.id())
    }

    pub fn len(&self) -> usize {
        self.features.len()
    }

    pub fn is_empty(&self) -> bool {
        self.features.is_empty()
    }

    /// Rebuild the id -> index map. Called after deserialize and any time the
    /// caller mutates `features` directly.
    pub fn reindex(&mut self) -> Result<(), ModelError> {
        self.index.clear();
        for (i, f) in self.features.iter().enumerate() {
            let id = f.id().to_string();
            if self.index.insert(id.clone(), i).is_some() {
                return Err(ModelError::DuplicateId(id));
            }
        }
        Ok(())
    }

    pub fn to_json_string(&self) -> serde_json::Result<String> {
        serde_json::to_string_pretty(self)
    }

    pub fn from_json_str(s: &str) -> Result<Self, ModelError> {
        let mut m: Model = serde_json::from_str(s).map_err(ModelError::Json)?;
        m.reindex()?;
        Ok(m)
    }

    pub fn write_json(&self, w: &mut impl Write) -> Result<(), ModelError> {
        serde_json::to_writer_pretty(w, self).map_err(ModelError::Json)
    }

    pub fn read_json(r: &mut impl Read) -> Result<Self, ModelError> {
        let mut m: Model = serde_json::from_reader(r).map_err(ModelError::Json)?;
        m.reindex()?;
        Ok(m)
    }

    pub fn write_json_path(&self, path: impl AsRef<Path>) -> Result<(), ModelError> {
        let f = File::create(path.as_ref()).map_err(ModelError::Io)?;
        let mut w = BufWriter::new(f);
        self.write_json(&mut w)
    }

    pub fn read_json_path(path: impl AsRef<Path>) -> Result<Self, ModelError> {
        let f = File::open(path.as_ref()).map_err(ModelError::Io)?;
        let mut r = BufReader::new(f);
        Self::read_json(&mut r)
    }
}

// ---------------------------------------------------------------------------
// Equation integration tests (model-level)
// ---------------------------------------------------------------------------

#[cfg(test)]
mod equation_tests {
    use super::*;
    use crate::feature::Feature;
    use crate::scalar::Scalar;

    // Helper: build a trivial model with a box whose x extent is $radius.
    fn radius_box_model() -> Model {
        Model::new()
            .with_parameter("diameter", 10.0)
            .with_equation("radius", "$diameter / 2")
            .add(Feature::Box {
                id: "body".into(),
                extents: [Scalar::param("radius"), Scalar::lit(1.0), Scalar::lit(1.0)],
            })
    }

    // --- test 6: JSON round-trip preserves equations ---
    #[test]
    fn equations_json_round_trip() {
        let m = Model::new()
            .with_parameter("diameter", 10.0)
            .with_equation("radius", "$diameter / 2");
        let json = m.to_json_string().unwrap();
        let m2 = Model::from_json_str(&json).unwrap();
        assert_eq!(m2.equations.get("radius").map(String::as_str), Some("$diameter / 2"));
        assert!((m2.parameters["diameter"] - 10.0).abs() < 1e-12);
    }

    // --- test 7: legacy JSON without equations still loads ---
    #[test]
    fn legacy_json_without_equations_loads() {
        let json = r#"{"parameters": {"diameter": 10.0}, "features": []}"#;
        let m = Model::from_json_str(json).unwrap();
        assert!(m.equations.is_empty());
        assert!((m.parameters["diameter"] - 10.0).abs() < 1e-12);
    }

    // --- test 8: evaluate with equations produces correct geometry ---
    #[test]
    fn evaluate_with_equations_resolves_radius() {
        let m = radius_box_model();
        // resolve_params should give radius=5 from diameter=10
        let p = m.resolve_params().unwrap();
        assert!((p["radius"] - 5.0).abs() < 1e-12);
        // Full evaluation should succeed and produce a non-degenerate solid
        let solid = m.evaluate("body").expect("evaluate");
        assert!(solid.vertex_count() > 0);
    }

    // --- test 9: chained equations depth-first ---
    #[test]
    fn chained_equations_depth_first() {
        let m = Model::new()
            .with_parameter("depth", 3.0)
            .with_equation("width", "$depth * 2")
            .with_equation("height", "$width + 1");
        let p = m.resolve_params().unwrap();
        assert!((p["width"] - 6.0).abs() < 1e-12);
        assert!((p["height"] - 7.0).abs() < 1e-12);
    }

    // --- test 10: cycle in equations surfaces as ModelError::Cycle ---
    #[test]
    fn equations_cycle_errors() {
        let m = Model::new()
            .with_equation("a", "$b")
            .with_equation("b", "$a");
        match m.resolve_params() {
            Err(ModelError::Cycle(_)) => {}
            other => panic!("expected Cycle, got {other:?}"),
        }
    }

    // --- test 11: unknown $ref surfaces as ModelError::UnknownId ---
    #[test]
    fn equations_unknown_ref_errors() {
        let m = Model::new().with_equation("x", "$ghost + 1");
        match m.resolve_params() {
            Err(ModelError::UnknownId(name)) => assert_eq!(name, "ghost"),
            other => panic!("expected UnknownId, got {other:?}"),
        }
    }

    // --- test 12: empty equations, model with no equations fields in JSON ---
    #[test]
    fn no_equations_field_skipped_in_json() {
        // A model without equations must not emit an "equations" key at all.
        let m = Model::new().with_parameter("x", 1.0);
        let json = m.to_json_string().unwrap();
        assert!(!json.contains("equations"), "equations should be omitted when empty");
    }

    // --- test 13: equations compose with base params correctly ---
    #[test]
    fn equations_overwrite_base_params() {
        // If an equation has the same name as a base param, the equation wins.
        let m = Model::new()
            .with_parameter("r", 1.0) // base value
            .with_parameter("d", 10.0)
            .with_equation("r", "$d / 2"); // equation overrides base
        let p = m.resolve_params().unwrap();
        assert!((p["r"] - 5.0).abs() < 1e-12, "equation should win over base param");
    }
}
