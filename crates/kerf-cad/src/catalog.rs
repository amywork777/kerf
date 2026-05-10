//! Catalog: parse `feature.rs` to surface every `Feature` variant + its doc
//! comment + a default JSON example. Reused by `bin/extract_catalog.rs` (to
//! emit `docs/FEATURE_CATALOG.md`) and by `tests/catalog_examples.rs` (to
//! verify every example evaluates).
//!
//! The parser is intentionally regex-light (line-based) and lives next to
//! the source it scans — `kerf-cad` has no `syn` dep and we don't want to
//! add one. The format depends on the conventions used in `feature.rs`:
//!   - one field per line,
//!   - doc comments (`///`) immediately above each variant,
//!   - struct-style variants only (`Name { ... }`).
//!
//! When `feature.rs` deviates from that shape this module's tests catch the
//! regression rather than silently emitting the wrong doc.

use std::path::PathBuf;

// ---------------------------------------------------------------------------
// Public types.
// ---------------------------------------------------------------------------

/// One parsed `Feature` variant: its name, doc comment lines, and field list.
#[derive(Debug, Clone)]
pub struct Variant {
    pub name: String,
    pub doc: String,
    pub fields: Vec<Field>,
}

/// One field of a variant: name + type-as-written.
#[derive(Debug, Clone)]
pub struct Field {
    pub name: String,
    pub ty: String,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Category {
    Primitives,
    ExtrusionsAndProfiles,
    Booleans,
    TransformsAndPatterns,
    Edges,
    Holes,
    Fasteners,
    Joinery,
    StructuralShapes,
    Vessels,
    Furniture,
    Tools,
    DecorativeAndMisc,
    Reference,
    Sweeps,
}

impl Category {
    pub fn title(self) -> &'static str {
        match self {
            Category::Primitives => "Primitives",
            Category::ExtrusionsAndProfiles => "Extrusions & profiles",
            Category::Booleans => "Booleans",
            Category::TransformsAndPatterns => "Transforms & patterns",
            Category::Edges => "Edge treatments (fillet/chamfer/cut)",
            Category::Holes => "Holes & pockets",
            Category::Fasteners => "Fasteners (bolts, screws, rivets, etc.)",
            Category::Joinery => "Joinery (mortise/tenon/dovetail)",
            Category::StructuralShapes => "Structural shapes (beams, channels, brackets)",
            Category::Vessels => "Vessels & containers",
            Category::Furniture => "Furniture",
            Category::Tools => "Tools & hardware",
            Category::DecorativeAndMisc => "Decorative & miscellaneous shapes",
            Category::Reference => "Reference geometry & markers",
            Category::Sweeps => "Sweeps & paths",
        }
    }

    pub fn order() -> &'static [Category] {
        &[
            Category::Primitives,
            Category::ExtrusionsAndProfiles,
            Category::Sweeps,
            Category::Booleans,
            Category::TransformsAndPatterns,
            Category::Edges,
            Category::Holes,
            Category::Fasteners,
            Category::Joinery,
            Category::StructuralShapes,
            Category::Vessels,
            Category::Furniture,
            Category::Tools,
            Category::Reference,
            Category::DecorativeAndMisc,
        ]
    }
}

// ---------------------------------------------------------------------------
// Locating + reading feature.rs.
// ---------------------------------------------------------------------------

/// Path to feature.rs at compile time of this crate.
pub fn feature_rs_path() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("src")
        .join("feature.rs")
}

/// Read feature.rs and return its source as a string.
pub fn read_feature_rs() -> std::io::Result<String> {
    std::fs::read_to_string(feature_rs_path())
}

// ---------------------------------------------------------------------------
// Parser.
// ---------------------------------------------------------------------------

pub fn parse_variants(src: &str) -> Result<Vec<Variant>, String> {
    let enum_start = src
        .find("pub enum Feature {")
        .ok_or_else(|| "could not find `pub enum Feature {` in feature.rs".to_string())?;
    let after = &src[enum_start..];
    let body_start = after
        .find('{')
        .ok_or_else(|| "no opening brace for Feature enum".to_string())?
        + 1;

    let bytes = after.as_bytes();
    let mut depth = 1usize;
    let mut i = body_start;
    while i < bytes.len() && depth > 0 {
        match bytes[i] {
            b'{' => depth += 1,
            b'}' => depth -= 1,
            _ => {}
        }
        if depth == 0 {
            break;
        }
        i += 1;
    }
    if depth != 0 {
        return Err("unterminated Feature enum body".into());
    }
    let body = &after[body_start..i];

    parse_enum_body(body)
}

fn parse_enum_body(body: &str) -> Result<Vec<Variant>, String> {
    let lines: Vec<&str> = body.lines().collect();
    let mut variants = Vec::new();

    let mut i = 0usize;
    let mut pending_doc: Vec<String> = Vec::new();
    while i < lines.len() {
        let raw = lines[i];
        let trimmed = raw.trim();
        if trimmed.is_empty() {
            i += 1;
            continue;
        }
        if let Some(d) = trimmed.strip_prefix("///") {
            let d = d.strip_prefix(' ').unwrap_or(d);
            pending_doc.push(d.to_string());
            i += 1;
            continue;
        }
        if let Some(name) = parse_variant_header(trimmed) {
            let mut fields = Vec::new();
            i += 1;
            while i < lines.len() {
                let l = lines[i].trim();
                if l == "}," || l == "}" {
                    i += 1;
                    break;
                }
                if l.is_empty() || l.starts_with("///") || l.starts_with("//") {
                    i += 1;
                    continue;
                }
                if let Some((fname, fty)) = parse_field_line(l) {
                    fields.push(Field { name: fname, ty: fty });
                }
                i += 1;
            }
            // Drop the implicit `id: String` field — every variant has it
            // and surfacing it in every example clutters the docs.
            let fields: Vec<Field> = fields
                .into_iter()
                .filter(|f| !(f.name == "id" && f.ty == "String"))
                .collect();
            let doc = pending_doc.join("\n").trim().to_string();
            variants.push(Variant {
                name: name.to_string(),
                doc,
                fields,
            });
            pending_doc.clear();
            continue;
        }
        pending_doc.clear();
        i += 1;
    }
    Ok(variants)
}

fn parse_variant_header(line: &str) -> Option<&str> {
    let line = line.trim_end_matches(|c: char| c.is_whitespace() || c == ',');
    let line = line.trim_end();
    let line = line.strip_suffix('{')?.trim_end();
    if line.is_empty() {
        return None;
    }
    let first = line.chars().next()?;
    if !first.is_ascii_uppercase() {
        return None;
    }
    if line.contains(' ') || line.contains('(') {
        return None;
    }
    Some(line)
}

fn parse_field_line(line: &str) -> Option<(String, String)> {
    let line = line.trim_end_matches(',').trim();
    let (name, ty) = line.split_once(':')?;
    let name = name.trim().to_string();
    let ty = ty.trim().to_string();
    if name.is_empty() || ty.is_empty() {
        return None;
    }
    if !name
        .chars()
        .all(|c| c.is_ascii_alphanumeric() || c == '_')
    {
        return None;
    }
    Some((name, ty))
}

// ---------------------------------------------------------------------------
// Categorisation.
// ---------------------------------------------------------------------------

pub fn categorize(name: &str) -> Category {
    if matches!(name, "Union" | "Intersection" | "Difference") {
        return Category::Booleans;
    }
    if matches!(
        name,
        "Translate"
            | "Scale"
            | "ScaleXYZ"
            | "Rotate"
            | "Mirror"
            | "LinearPattern"
            | "PolarPattern"
    ) {
        return Category::TransformsAndPatterns;
    }
    if matches!(name, "Fillet" | "Fillets" | "Chamfer" | "CornerCut") {
        return Category::Edges;
    }
    if matches!(
        name,
        "HoleArray"
            | "BoltCircle"
            | "HexHole"
            | "SquareHole"
            | "Countersink"
            | "Counterbore"
            | "PrismHole"
            | "Mortise"
            | "Keyway"
            | "Slot"
            | "Slot3D"
            | "FilletedSlot"
            | "DrawerSlot"
            | "DovetailSlot"
            | "VeeGroove"
            | "TSlot"
            | "ChamferedHole"
            | "ThreadedHoleMarker"
            | "BoltPattern"
            | "SquareDrive"
            | "RaisedBoss"
            | "ShaftOilHole"
            | "WoodruffKey"
            | "DraftedHole"
            | "Heatset"
    ) {
        return Category::Holes;
    }
    if matches!(name, "HexFlange") {
        return Category::Fasteners;
    }
    if matches!(
        name,
        "ExtrudePolygon" | "Loft" | "TaperedExtrude" | "Revolve"
    ) {
        return Category::ExtrusionsAndProfiles;
    }
    if matches!(
        name,
        "PipeRun"
            | "SweepPath"
            | "Coil"
            | "Spring"
            | "Helix"
            | "HelicalSweep"
            | "AxisTaperedTube"
            | "AxisTwistExtrude"
            | "PolarRevolveLoft"
    ) {
        return Category::Sweeps;
    }
    if matches!(
        name,
        "RefPoint"
            | "RefAxis"
            | "RefPlane"
            | "BoundingBoxRef"
            | "CentroidPoint"
            | "DistanceRod"
            | "AngleArc"
            | "AnchorPoint"
            | "Marker3D"
            | "VectorArrow"
            | "Arrow"
            | "MidPlaneRef"
            | "PerpRefPlane"
            | "OffsetRefPlane"
            | "CoordinateAxes"
            | "OriginPoint"
            | "CenterMarker"
            | "AxisLabel"
            | "DistanceMarker"
    ) {
        return Category::Reference;
    }
    if matches!(
        name,
        "Bolt"
            | "CapBolt"
            | "CapScrew"
            | "FlangedBolt"
            | "FlatHeadScrew"
            | "ShoulderBolt"
            | "SocketHeadCapScrew"
            | "EyeBolt"
            | "WingedScrew"
            | "Rivet"
            | "ThreadInsert"
            | "Nut"
            | "Washer"
            | "FlatWasher"
    ) {
        return Category::Fasteners;
    }
    if matches!(
        name,
        "Tenon" | "FingerJoint" | "DovetailRail" | "SquareKey" | "CrossKey"
    ) {
        return Category::Joinery;
    }
    if matches!(
        name,
        "LBracket"
            | "UChannel"
            | "TBeam"
            | "IBeam"
            | "CChannel"
            | "ZBeam"
            | "AngleIron"
            | "BasePlate"
            | "BeamWithHoles"
            | "ShelfBracket"
            | "GussetPlate"
            | "ChamferedPlate"
            | "PerforatedPlate"
            | "RibbedPlate"
            | "OvalPlate"
            | "TriangularPlate"
            | "SpiralPlate"
            | "ScrollPlate"
            | "SquareTube"
            | "Trapezoid"
            | "TrussMember"
            | "AsymmetricBracket"
            | "CornerBracket"
            | "Cleat"
            | "ParapetWall"
            | "RoundedRect"
            | "Lattice"
            | "WireMesh"
            | "Stair"
            | "SheetBend"
            | "CrossBrace"
            | "TriPrism"
            | "Hinge"
            | "Yoke"
            | "Lever"
            | "ChainLink"
            | "BeltLoop"
            | "Stake"
            | "WindowFrame"
            | "WindowLouver"
            | "Pediment"
            | "Vault"
            | "ArchedDoorway"
            | "GearBlank"
            | "HoleyPlate"
            | "ScrewBoss"
            | "RoundBoss"
            | "RectBoss"
            | "MountingFlange"
            | "EndCap"
            | "FerruleEnd"
            | "ReducerCone"
            | "Tee"
            | "Cross"
            | "Elbow90"
            | "UBendPipe"
            | "SBend"
            | "Pipe"
            | "Tube"
            | "TubeAt"
            | "TaperedTube"
            | "FunnelTube"
            | "AxleShaft"
            | "PinShaft"
            | "SteppedShaft"
            | "CylinderShellAt"
            | "RatchetTooth"
    ) {
        return Category::StructuralShapes;
    }
    if matches!(
        name,
        "Cup"
            | "Bowl"
            | "Bottle"
            | "PlanterBox"
            | "Trough"
            | "Hopper"
            | "Funnel"
            | "HollowBox"
            | "HollowBrick"
            | "HollowSphere"
            | "HollowCone"
            | "HollowCylinder"
            | "Lampshade"
            | "CableSaddle"
            | "Spool"
            | "FishingFloat"
    ) {
        return Category::Vessels;
    }
    if matches!(
        name,
        "TableLeg"
            | "ChairLeg"
            | "Bookshelf"
            | "TableTop"
            | "Bench"
            | "Chair"
            | "Stand"
            | "Plinth"
            | "Column"
            | "Obelisk"
    ) {
        return Category::Furniture;
    }
    if matches!(
        name,
        "Hammer"
            | "ScrewDriver"
            | "Wrench"
            | "Knob"
            | "ToroidalKnob"
            | "Handle"
            | "HookHandle"
            | "KneadHandle"
            | "KnurledGrip"
            | "Bushing"
            | "Pulley"
            | "Sprocket"
            | "Crank"
            | "Cam"
            | "CamLobe"
            | "Plug"
            | "Spike"
            | "Brick"
            | "CorrugatedPanel"
            | "Bipyramid"
            | "Antiprism"
            | "Tetrahedron"
            | "SerratedDisk"
    ) {
        return Category::Tools;
    }
    if matches!(
        name,
        "Box"
            | "BoxAt"
            | "Cylinder"
            | "Sphere"
            | "Torus"
            | "Donut"
            | "Cone"
            | "Frustum"
            | "Wedge"
            | "RegularPrism"
            | "Pyramid"
            | "TruncatedPyramid"
            | "Capsule"
            | "CapsuleAt"
            | "SphereFaceted"
            | "Dome"
            | "Hemisphere"
            | "SphericalCap"
            | "CylinderAt"
            | "Diamond"
            | "TruncatedSphere"
            | "Lens2"
            | "Capsule2"
            | "OvoidShell"
            | "Bagel"
            | "Pringle"
            | "Cone2"
            | "Lozenge"
    ) {
        return Category::Primitives;
    }
    Category::DecorativeAndMisc
}

// ---------------------------------------------------------------------------
// Default-value heuristics + curated overrides.
// ---------------------------------------------------------------------------

pub fn snake_id(name: &str) -> String {
    let mut out = String::new();
    for (i, c) in name.chars().enumerate() {
        if c.is_ascii_uppercase() {
            if i > 0 {
                out.push('_');
            }
            out.push(c.to_ascii_lowercase());
        } else {
            out.push(c);
        }
    }
    out
}

/// Build the default JSON example for a variant. Returns a JSON object with
/// the `kind` set, an `id` derived from the variant name, and reasonable
/// defaults for every field. For variants with `input`/`inputs` references,
/// the example assumes a sibling `body` Feature::Box (and `tool` for booleans)
/// exists in the model.
pub fn default_example_json(v: &Variant) -> serde_json::Value {
    use serde_json::Value;

    if let Some(custom) = curated_override(&v.name) {
        return Value::Object(custom);
    }

    let mut obj = serde_json::Map::new();
    obj.insert("kind".into(), Value::String(v.name.clone()));
    obj.insert("id".into(), Value::String(snake_id(&v.name)));
    for f in &v.fields {
        let val = default_for_field(&v.name, &f.name, &f.ty);
        obj.insert(f.name.clone(), val);
    }
    Value::Object(obj)
}

fn default_for_field(variant: &str, name: &str, ty: &str) -> serde_json::Value {
    use serde_json::{json, Value};

    if ty == "String" {
        return Value::String("z".into());
    }
    if ty == "Scalar" {
        return num(pick_scalar(variant, name));
    }
    if ty == "usize" {
        return Value::Number(serde_json::Number::from(usize_default(variant, name)));
    }
    if ty == "[Scalar; 3]" {
        let v = vec3_default(variant, name);
        return Value::Array(v.into_iter().map(num).collect());
    }
    if ty == "[Scalar; 2]" {
        let v = vec2_default(variant, name);
        return Value::Array(v.into_iter().map(num).collect());
    }
    if ty == "Profile2D" {
        return profile2d_default();
    }
    if ty == "Vec<[Scalar; 3]>" {
        return json!([[0.0, 0.0, 0.0], [10.0, 0.0, 0.0], [10.0, 10.0, 0.0]]);
    }
    if ty == "Vec<Scalar>" {
        if name.contains("radi") {
            return json!([5.0, 4.0, 3.0]);
        }
        return json!([10.0, 8.0, 6.0]);
    }
    if ty == "Vec<FilletEdge>" {
        return json!([]);
    }
    if ty == "Vec<String>" {
        return json!(["body", "tool"]);
    }
    if ty == "bool" {
        return Value::Bool(false);
    }
    Value::String(format!("<{ty}>"))
}

fn num(x: f64) -> serde_json::Value {
    serde_json::Number::from_f64(x)
        .map(serde_json::Value::Number)
        .unwrap_or(serde_json::Value::Null)
}

fn pick_scalar(_variant: &str, name: &str) -> f64 {
    match name {
        "outer_radius" => 8.0,
        "inner_radius" => 4.0,
        "bore_radius" => 2.0,
        "shaft_radius" | "rod_radius" | "pin_radius" => 2.0,
        "head_radius" => 4.0,
        "tip_radius" => 1.0,
        "wire_radius" => 0.5,
        "marker_radius" | "marker_size" | "wire_thickness" => 0.3,
        "minor_radius" | "torus_minor_radius" => 1.0,
        "major_radius" | "torus_major_radius" => 5.0,
        "bend_radius" => 6.0,
        "pipe_radius" => 1.0,
        "ring_minor_radius" => 1.0,
        "ring_major_radius" => 4.0,
        "corner_radius" | "cap_radius" => 2.0,
        "root_radius" => 6.5,
        "outer_top" => 6.0,
        "outer_bottom" => 8.0,
        "inner_top" => 3.0,
        "inner_bottom" => 5.0,
        "csink_radius" => 5.0,
        "drill_radius" => 2.5,
        "cbore_radius" => 5.0,
        "hole_radius" => 1.5,
        "bolt_radius" => 1.0,
        "bolt_circle_radius" => 8.0,
        "disk_radius" => 12.0,
        "plate_radius" => 12.0,
        "small_radius" => 3.0,
        "large_radius" => 6.0,
        "thread_radius" => 1.5,
        "shoulder_radius" => 2.5,
        "body_radius" => 5.0,
        "flange_radius" => 7.0,
        "pivot_radius" | "wrist_radius" => 1.5,
        "groove_radius" => 1.0,
        "neck_radius" => 2.0,
        "base_radius" => 8.0,
        "capital_radius" => 8.0,
        "groove_depth" => 1.5,
        "groove_width" => 2.0,
        "tip_length" => 3.0,
        "ridge_height" => 0.5,
        "shaft_length" => 10.0,
        "head_thickness" => 3.0,
        "head_height" => 3.0,
        "neck_z" => 6.0,
        "spout_length" => 4.0,
        "csink_depth" => 1.5,
        "cbore_depth" => 2.0,
        "total_depth" => 8.0,
        "depth_into" => 3.0,
        "radius" => 5.0,
        "bottom_radius" => 6.0,
        "top_radius" => 3.0,
        "wall_thickness"
        | "thickness"
        | "leaf_thickness"
        | "plate_thickness"
        | "side_thickness"
        | "shelf_thickness"
        | "flange_thickness"
        | "back_thickness"
        | "leg_thickness"
        | "bar_thickness"
        | "rim_thickness"
        | "end_thickness"
        | "marker_thickness"
        | "arm_thickness"
        | "web_thickness" => 1.0,
        "height"
        | "shaft_height"
        | "body_height"
        | "leg_height"
        | "arm_height"
        | "back_height"
        | "tab_height"
        | "neck_height"
        | "funnel_height"
        | "support_height"
        | "leaf_height"
        | "base_height"
        | "capital_height"
        | "clear_height"
        | "step_height" => 10.0,
        "depth"
        | "leg_depth"
        | "leg_length"
        | "length"
        | "neck_length"
        | "body_length"
        | "rod_length"
        | "shoulder_length"
        | "thread_length"
        | "arm_length"
        | "bar_length"
        | "length_a"
        | "length_b" => 10.0,
        "edge_length" => 8.0,
        "shaft_thickness" | "shaft_width" | "support_width" => 2.0,
        "width"
        | "outer_width"
        | "plate_width"
        | "leaf_width"
        | "frame_size"
        | "step_width"
        | "slot_width"
        | "base_width"
        | "flange_width"
        | "outer_length"
        | "outer_height"
        | "plate_height"
        | "total_height"
        | "side"
        | "top_width"
        | "bottom_width"
        | "bottom_side"
        | "top_side" => 10.0,
        "inscribed_radius" => 4.0,
        "leg_x" | "leg_y" | "leg_z" => 6.0,
        "tread" | "riser" => 5.0,
        "cell_size" => 4.0,
        "chamfer" => 1.0,
        "setback" => 1.5,
        "slot_height" => 4.0,
        "cap_height" => 2.0,
        "factor" => 1.0,
        "top_scale" => 0.7,
        "aspect_z" => 1.5,
        "angle_deg" => 0.0,
        "start_deg" => 0.0,
        "sweep_deg" => 90.0,
        "total_angle_deg" => 360.0,
        "eccentricity" => 1.0,
        "pitch" => 4.0,
        "turns" => 4.0,
        "dx" | "dy" => 4.0,
        "margin" => 2.0,
        "flute_depth" => 1.5,
        _ => 1.0,
    }
}

fn usize_default(_variant: &str, name: &str) -> u64 {
    match name {
        "segments"
        | "stacks"
        | "slices"
        | "wire_segments"
        | "shaft_segments"
        | "ring_segs"
        | "minor_segs"
        | "major_segs"
        | "body_segs"
        | "torus_segs" => 16,
        "segments_along" | "segments_around" => 12,
        "segments_per_turn" => 12,
        "segments_per_tooth" => 4,
        "count" | "bolt_count" | "tooth_count" | "ridge_count" | "flute_count" | "step_count"
        | "shelves" => 4,
        "points" => 6,
        "nx" | "ny" => 3,
        _ => 4,
    }
}

fn vec3_default(_variant: &str, name: &str) -> [f64; 3] {
    match name {
        "extents" => [10.0, 10.0, 10.0],
        "origin" => [0.0, 0.0, 0.0],
        "corner" => [3.0, 3.0, 3.0],
        "offset" => [12.0, 0.0, 0.0],
        "axis" => [0.0, 0.0, 1.0],
        "center" => [0.0, 0.0, 0.0],
        "position" => [0.0, 0.0, 0.0],
        "edge_min" => [0.0, 0.0, 0.0],
        "from" => [0.0, 0.0, 0.0],
        "to" => [10.0, 0.0, 0.0],
        "start" => [0.0, 0.0, 0.0],
        "top_center" => [5.0, 5.0, 10.0],
        "base" => [0.0, 0.0, 0.0],
        "plane_origin" => [0.0, 0.0, 0.0],
        "plane_normal" => [1.0, 0.0, 0.0],
        "direction" => [0.0, 0.0, 5.0],
        _ => [0.0, 0.0, 0.0],
    }
}

fn vec2_default(_variant: &str, name: &str) -> [f64; 2] {
    match name {
        "center" => [0.0, 0.0],
        "extents" => [10.0, 10.0],
        _ => [0.0, 0.0],
    }
}

fn profile2d_default() -> serde_json::Value {
    serde_json::json!({
        "points": [
            [0.0, 0.0],
            [10.0, 0.0],
            [10.0, 10.0],
            [0.0, 10.0]
        ]
    })
}

/// Hand-crafted defaults for variants where the heuristic would either miss a
/// constraint or need to reference another feature. Returning `Some` here
/// completely replaces the heuristic-built object.
fn curated_override(variant: &str) -> Option<serde_json::Map<String, serde_json::Value>> {
    use serde_json::{json, Value};
    let mut m = serde_json::Map::new();
    m.insert("kind".into(), Value::String(variant.into()));
    m.insert("id".into(), Value::String(snake_id(variant)));

    match variant {
        "Union" | "Intersection" | "Difference" => {
            m.insert("inputs".into(), json!(["body", "tool"]));
            Some(m)
        }
        "Translate" => {
            m.insert("input".into(), Value::String("body".into()));
            m.insert("offset".into(), json!([5.0, 0.0, 0.0]));
            Some(m)
        }
        "Scale" => {
            m.insert("input".into(), Value::String("body".into()));
            m.insert("factor".into(), num(1.5));
            Some(m)
        }
        "ScaleXYZ" => {
            m.insert("input".into(), Value::String("body".into()));
            m.insert("factors".into(), json!([1.5, 1.0, 0.8]));
            Some(m)
        }
        "Rotate" => {
            m.insert("input".into(), Value::String("body".into()));
            m.insert("axis".into(), json!([0.0, 0.0, 1.0]));
            m.insert("angle_deg".into(), num(45.0));
            m.insert("center".into(), json!([0.0, 0.0, 0.0]));
            Some(m)
        }
        "Mirror" => {
            m.insert("input".into(), Value::String("body".into()));
            m.insert("plane_origin".into(), json!([0.0, 0.0, 0.0]));
            m.insert("plane_normal".into(), json!([1.0, 0.0, 0.0]));
            Some(m)
        }
        "LinearPattern" => {
            m.insert("input".into(), Value::String("body".into()));
            m.insert("count".into(), json!(3));
            m.insert("offset".into(), json!([12.0, 0.0, 0.0]));
            Some(m)
        }
        "PolarPattern" => {
            m.insert("input".into(), Value::String("body".into()));
            m.insert("count".into(), json!(4));
            m.insert("axis".into(), json!([0.0, 0.0, 1.0]));
            m.insert("center".into(), json!([20.0, 0.0, 0.0]));
            m.insert("total_angle_deg".into(), num(360.0));
            Some(m)
        }
        "CornerCut" => {
            m.insert("input".into(), Value::String("body".into()));
            m.insert("corner".into(), json!([7.0, 7.0, 7.0]));
            m.insert("extents".into(), json!([5.0, 5.0, 5.0]));
            Some(m)
        }
        "Fillet" => {
            m.insert("input".into(), Value::String("body".into()));
            m.insert("axis".into(), Value::String("z".into()));
            m.insert("edge_min".into(), json!([10.0, 10.0, 0.0]));
            m.insert("edge_length".into(), num(10.0));
            m.insert("radius".into(), num(2.0));
            m.insert("quadrant".into(), Value::String("nn".into()));
            m.insert("segments".into(), json!(8));
            Some(m)
        }
        "Fillets" => {
            m.insert("input".into(), Value::String("body".into()));
            m.insert(
                "edges".into(),
                json!([
                    {
                        "axis": "z",
                        "edge_min": [10.0, 10.0, 0.0],
                        "edge_length": 10.0,
                        "radius": 1.5,
                        "quadrant": "nn",
                        "segments": 8
                    }
                ]),
            );
            Some(m)
        }
        "Chamfer" => {
            m.insert("input".into(), Value::String("body".into()));
            m.insert("axis".into(), Value::String("z".into()));
            m.insert("edge_min".into(), json!([10.0, 10.0, 0.0]));
            m.insert("edge_length".into(), num(10.0));
            m.insert("setback".into(), num(1.0));
            m.insert("quadrant".into(), Value::String("nn".into()));
            Some(m)
        }
        "HoleArray" => {
            m.insert("input".into(), Value::String("body".into()));
            m.insert("axis".into(), Value::String("z".into()));
            m.insert("start".into(), json!([3.0, 5.0, 10.0]));
            m.insert("offset".into(), json!([2.5, 0.0, 0.0]));
            m.insert("count".into(), json!(2));
            m.insert("radius".into(), num(0.6));
            m.insert("depth".into(), num(8.0));
            m.insert("segments".into(), json!(12));
            Some(m)
        }
        "BoltCircle" => {
            m.insert("input".into(), Value::String("body".into()));
            m.insert("axis".into(), Value::String("z".into()));
            m.insert("center".into(), json!([5.0, 5.0, 10.0]));
            m.insert("bolt_circle_radius".into(), num(3.0));
            m.insert("count".into(), json!(4));
            m.insert("radius".into(), num(0.5));
            m.insert("depth".into(), num(8.0));
            m.insert("segments".into(), json!(12));
            Some(m)
        }
        "HexHole" => {
            m.insert("input".into(), Value::String("body".into()));
            m.insert("axis".into(), Value::String("z".into()));
            m.insert("top_center".into(), json!([5.0, 5.0, 10.0]));
            m.insert("inscribed_radius".into(), num(2.0));
            m.insert("depth".into(), num(8.0));
            Some(m)
        }
        "SquareHole" => {
            m.insert("input".into(), Value::String("body".into()));
            m.insert("axis".into(), Value::String("z".into()));
            m.insert("top_center".into(), json!([5.0, 5.0, 10.0]));
            m.insert("side".into(), num(3.0));
            m.insert("depth".into(), num(8.0));
            Some(m)
        }
        "Countersink" => {
            m.insert("input".into(), Value::String("body".into()));
            m.insert("axis".into(), Value::String("z".into()));
            m.insert("top_center".into(), json!([5.0, 5.0, 10.0]));
            m.insert("drill_radius".into(), num(1.5));
            m.insert("csink_radius".into(), num(3.0));
            m.insert("csink_depth".into(), num(1.5));
            m.insert("total_depth".into(), num(8.0));
            m.insert("segments".into(), json!(16));
            Some(m)
        }
        "Counterbore" => {
            m.insert("input".into(), Value::String("body".into()));
            m.insert("axis".into(), Value::String("z".into()));
            m.insert("top_center".into(), json!([5.0, 5.0, 10.0]));
            m.insert("drill_radius".into(), num(1.5));
            m.insert("cbore_radius".into(), num(3.0));
            m.insert("cbore_depth".into(), num(2.0));
            m.insert("total_depth".into(), num(8.0));
            m.insert("segments".into(), json!(16));
            Some(m)
        }
        "BoundingBoxRef" => {
            m.insert("input".into(), Value::String("body".into()));
            m.insert("wire_thickness".into(), num(0.3));
            Some(m)
        }
        "CentroidPoint" => {
            m.insert("input".into(), Value::String("body".into()));
            m.insert("marker_size".into(), num(0.5));
            Some(m)
        }
        "Sphere" => {
            m.insert("radius".into(), num(5.0));
            Some(m)
        }
        "Donut" => {
            m.insert("major_radius".into(), num(5.0));
            m.insert("minor_radius".into(), num(1.0));
            m.insert("major_segs".into(), json!(16));
            m.insert("minor_segs".into(), json!(8));
            Some(m)
        }
        "Donut2" => {
            m.insert("major_radius".into(), num(5.0));
            m.insert("minor_radius".into(), num(1.0));
            m.insert("major_segs".into(), json!(24));
            m.insert("minor_segs".into(), json!(12));
            Some(m)
        }
        "ToroidalCap" => {
            m.insert("major_radius".into(), num(5.0));
            m.insert("minor_radius".into(), num(1.0));
            m.insert("sweep_degrees".into(), num(180.0));
            m.insert("major_segs".into(), json!(12));
            m.insert("minor_segs".into(), json!(8));
            Some(m)
        }
        "EllipticTube" => {
            m.insert("semi_major".into(), num(4.0));
            m.insert("semi_minor".into(), num(2.0));
            m.insert("length".into(), num(10.0));
            m.insert("axis".into(), Value::String("z".into()));
            m.insert("segments".into(), json!(16));
            Some(m)
        }
        "Goblet2" => {
            m.insert("foot_radius".into(), num(4.0));
            m.insert("stem_radius".into(), num(1.0));
            m.insert("stem_height".into(), num(6.0));
            m.insert("cup_radius".into(), num(3.0));
            m.insert("cup_height".into(), num(4.0));
            m.insert("segments".into(), json!(16));
            Some(m)
        }
        "Frustum" => {
            m.insert("top_radius".into(), num(3.0));
            m.insert("bottom_radius".into(), num(5.0));
            m.insert("height".into(), num(8.0));
            Some(m)
        }
        "TruncatedPyramid" => {
            m.insert("bottom_radius".into(), num(8.0));
            m.insert("top_radius".into(), num(4.0));
            m.insert("height".into(), num(10.0));
            m.insert("segments".into(), json!(6));
            Some(m)
        }
        "Pyramid" => {
            m.insert("radius".into(), num(8.0));
            m.insert("height".into(), num(10.0));
            m.insert("segments".into(), json!(6));
            Some(m)
        }
        "Plug" => {
            m.insert("top_radius".into(), num(3.0));
            m.insert("bottom_radius".into(), num(5.0));
            m.insert("length".into(), num(10.0));
            m.insert("segments".into(), json!(16));
            Some(m)
        }
        "TableLeg" => {
            m.insert("bottom_radius".into(), num(5.0));
            m.insert("top_radius".into(), num(3.0));
            m.insert("height".into(), num(40.0));
            m.insert("segments".into(), json!(16));
            Some(m)
        }
        "Obelisk" => {
            m.insert("bottom_side".into(), num(8.0));
            m.insert("top_side".into(), num(3.0));
            m.insert("height".into(), num(20.0));
            Some(m)
        }
        "FerruleEnd" => {
            m.insert("small_radius".into(), num(3.0));
            m.insert("large_radius".into(), num(5.0));
            m.insert("wall_thickness".into(), num(0.5));
            m.insert("length".into(), num(8.0));
            m.insert("segments".into(), json!(16));
            Some(m)
        }
        "ReducerCone" => {
            m.insert("outer_bottom".into(), num(6.0));
            m.insert("outer_top".into(), num(4.0));
            m.insert("inner_bottom".into(), num(5.0));
            m.insert("inner_top".into(), num(3.0));
            m.insert("height".into(), num(8.0));
            m.insert("segments".into(), json!(16));
            Some(m)
        }
        "TaperedTube" => {
            m.insert("bottom_outer_radius".into(), num(6.0));
            m.insert("top_outer_radius".into(), num(4.0));
            m.insert("wall_thickness".into(), num(0.7));
            m.insert("height".into(), num(10.0));
            m.insert("segments".into(), json!(16));
            Some(m)
        }
        "Hopper" => {
            m.insert("top_radius".into(), num(8.0));
            m.insert("neck_radius".into(), num(3.0));
            m.insert("funnel_height".into(), num(8.0));
            m.insert("neck_height".into(), num(4.0));
            m.insert("segments".into(), json!(16));
            Some(m)
        }
        "Funnel" => {
            m.insert("top_radius".into(), num(8.0));
            m.insert("neck_radius".into(), num(2.0));
            m.insert("neck_z".into(), num(6.0));
            m.insert("spout_length".into(), num(4.0));
            m.insert("segments".into(), json!(16));
            Some(m)
        }
        "FunnelTube" => {
            m.insert("top_outer_radius".into(), num(6.0));
            m.insert("bottom_outer_radius".into(), num(8.0));
            m.insert("wall_thickness".into(), num(0.5));
            m.insert("height".into(), num(8.0));
            m.insert("segments".into(), json!(16));
            Some(m)
        }
        "HollowCylinder" => {
            m.insert("outer_radius".into(), num(5.0));
            m.insert("inner_radius".into(), num(3.0));
            m.insert("height".into(), num(10.0));
            m.insert("end_thickness".into(), num(0.5));
            m.insert("segments".into(), json!(16));
            Some(m)
        }
        "HollowSphere" => {
            m.insert("outer_radius".into(), num(5.0));
            m.insert("inner_radius".into(), num(3.0));
            m.insert("stacks".into(), json!(4));
            m.insert("slices".into(), json!(8));
            Some(m)
        }
        "HollowCone" => {
            m.insert("radius".into(), num(5.0));
            m.insert("height".into(), num(10.0));
            m.insert("bore_radius".into(), num(3.0));
            m.insert("segments".into(), json!(16));
            Some(m)
        }
        "HollowBox" => {
            m.insert("extents".into(), json!([10.0, 10.0, 10.0]));
            m.insert("wall_thickness".into(), num(1.0));
            Some(m)
        }
        "HollowBrick" => {
            m.insert("length".into(), num(10.0));
            m.insert("width".into(), num(6.0));
            m.insert("height".into(), num(4.0));
            m.insert("wall_thickness".into(), num(0.7));
            Some(m)
        }
        "Tube" => {
            m.insert("outer_radius".into(), num(5.0));
            m.insert("inner_radius".into(), num(3.0));
            m.insert("height".into(), num(10.0));
            m.insert("segments".into(), json!(16));
            Some(m)
        }
        "TubeAt" => {
            m.insert("base".into(), json!([0.0, 0.0, 0.0]));
            m.insert("axis".into(), Value::String("z".into()));
            m.insert("outer_radius".into(), num(5.0));
            m.insert("inner_radius".into(), num(3.0));
            m.insert("height".into(), num(10.0));
            m.insert("segments".into(), json!(16));
            Some(m)
        }
        "Pipe" => {
            m.insert("base".into(), json!([0.0, 0.0, 0.0]));
            m.insert("axis".into(), Value::String("z".into()));
            m.insert("outer_radius".into(), num(5.0));
            m.insert("inner_radius".into(), num(3.0));
            m.insert("length".into(), num(10.0));
            m.insert("segments".into(), json!(16));
            Some(m)
        }
        "Cup" => {
            m.insert("outer_radius".into(), num(5.0));
            m.insert("height".into(), num(8.0));
            m.insert("wall_thickness".into(), num(0.7));
            m.insert("segments".into(), json!(16));
            Some(m)
        }
        "Bowl" => {
            m.insert("outer_radius".into(), num(5.0));
            m.insert("inner_radius".into(), num(4.0));
            m.insert("stacks".into(), json!(4));
            m.insert("slices".into(), json!(8));
            Some(m)
        }
        "Bottle" => {
            m.insert("body_radius".into(), num(4.0));
            m.insert("body_height".into(), num(10.0));
            m.insert("neck_radius".into(), num(1.5));
            m.insert("neck_height".into(), num(3.0));
            m.insert("segments".into(), json!(16));
            Some(m)
        }
        "Bushing" => {
            m.insert("inner_radius".into(), num(2.0));
            m.insert("outer_radius".into(), num(3.0));
            m.insert("flange_radius".into(), num(4.0));
            m.insert("flange_thickness".into(), num(0.7));
            m.insert("body_length".into(), num(6.0));
            m.insert("segments".into(), json!(16));
            Some(m)
        }
        "Washer" | "FlatWasher" => {
            m.insert("outer_radius".into(), num(4.0));
            m.insert("inner_radius".into(), num(2.0));
            m.insert("thickness".into(), num(0.5));
            m.insert("segments".into(), json!(16));
            Some(m)
        }
        "Nut" => {
            m.insert("inscribed_radius".into(), num(3.0));
            m.insert("bore_radius".into(), num(1.5));
            m.insert("thickness".into(), num(2.0));
            m.insert("segments".into(), json!(6));
            Some(m)
        }
        "ThreadInsert" => {
            m.insert("outer_radius".into(), num(2.5));
            m.insert("inner_radius".into(), num(1.5));
            m.insert("length".into(), num(6.0));
            m.insert("segments".into(), json!(16));
            Some(m)
        }
        "GearBlank" => {
            m.insert("outer_radius".into(), num(8.0));
            m.insert("root_radius".into(), num(7.0));
            m.insert("tooth_count".into(), json!(12));
            m.insert("thickness".into(), num(2.0));
            m.insert("segments_per_tooth".into(), json!(4));
            Some(m)
        }
        "Sprocket" | "SerratedDisk" => {
            m.insert("outer_radius".into(), num(8.0));
            m.insert("root_radius".into(), num(7.0));
            m.insert("tooth_count".into(), json!(12));
            m.insert("thickness".into(), num(2.0));
            Some(m)
        }
        "Lattice" => {
            m.insert("nx".into(), json!(3));
            m.insert("ny".into(), json!(3));
            m.insert("cell_size".into(), num(4.0));
            m.insert("bar_thickness".into(), num(0.6));
            m.insert("depth".into(), num(2.0));
            Some(m)
        }
        "WireMesh" => {
            m.insert("nx".into(), json!(3));
            m.insert("ny".into(), json!(3));
            m.insert("cell_size".into(), num(4.0));
            m.insert("wire_radius".into(), num(0.3));
            m.insert("segments".into(), json!(8));
            Some(m)
        }
        "PerforatedPlate" => {
            m.insert("plate_width".into(), num(20.0));
            m.insert("plate_height".into(), num(20.0));
            m.insert("plate_thickness".into(), num(1.0));
            m.insert("margin".into(), num(2.0));
            m.insert("hole_radius".into(), num(0.6));
            m.insert("nx".into(), json!(3));
            m.insert("ny".into(), json!(3));
            m.insert("dx".into(), num(5.0));
            m.insert("dy".into(), num(5.0));
            m.insert("segments".into(), json!(12));
            Some(m)
        }
        "MountingFlange" => {
            m.insert("disk_radius".into(), num(8.0));
            m.insert("disk_thickness".into(), num(1.0));
            m.insert("bolt_circle_radius".into(), num(6.0));
            m.insert("bolt_count".into(), json!(4));
            m.insert("bolt_radius".into(), num(0.5));
            m.insert("segments".into(), json!(16));
            Some(m)
        }
        "Stair" => {
            m.insert("step_count".into(), json!(4));
            m.insert("tread".into(), num(5.0));
            m.insert("riser".into(), num(3.0));
            m.insert("step_width".into(), num(10.0));
            Some(m)
        }
        "FingerJoint" => {
            m.insert("count".into(), json!(3));
            m.insert("finger_width".into(), num(2.0));
            m.insert("finger_height".into(), num(3.0));
            m.insert("finger_depth".into(), num(2.0));
            m.insert("gap_width".into(), num(2.0));
            Some(m)
        }
        "Bookshelf" => {
            m.insert("width".into(), num(30.0));
            m.insert("depth".into(), num(10.0));
            m.insert("shelves".into(), json!(3));
            m.insert("shelf_thickness".into(), num(1.0));
            m.insert("clear_height".into(), num(8.0));
            m.insert("side_thickness".into(), num(1.0));
            Some(m)
        }
        "PlanterBox" => {
            m.insert("outer_width".into(), num(15.0));
            m.insert("outer_length".into(), num(10.0));
            m.insert("outer_height".into(), num(8.0));
            m.insert("wall_thickness".into(), num(0.8));
            Some(m)
        }
        "Trough" => {
            m.insert("outer_width".into(), num(10.0));
            m.insert("outer_height".into(), num(6.0));
            m.insert("wall_thickness".into(), num(0.8));
            m.insert("length".into(), num(20.0));
            Some(m)
        }
        "DrawerSlot" => {
            m.insert("width".into(), num(10.0));
            m.insert("depth".into(), num(8.0));
            m.insert("height".into(), num(4.0));
            m.insert("wall_thickness".into(), num(0.8));
            Some(m)
        }
        "Spring" | "Coil" => {
            m.insert("coil_radius".into(), num(5.0));
            m.insert("wire_radius".into(), num(0.5));
            m.insert("pitch".into(), num(2.0));
            m.insert("turns".into(), num(4.0));
            m.insert("segments_per_turn".into(), json!(12));
            m.insert("wire_segments".into(), json!(8));
            Some(m)
        }
        "Helix" => {
            m.insert("axis_radius".into(), num(5.0));
            m.insert("pitch".into(), num(2.0));
            m.insert("turns".into(), num(2.0));
            m.insert("wire_radius".into(), num(0.5));
            m.insert("axis".into(), json!("z"));
            m.insert("segments".into(), json!(16));
            Some(m)
        }
        "Bolt" => {
            m.insert("head_inscribed_radius".into(), num(3.0));
            m.insert("head_thickness".into(), num(2.0));
            m.insert("shaft_radius".into(), num(1.5));
            m.insert("shaft_length".into(), num(8.0));
            m.insert("segments".into(), json!(16));
            Some(m)
        }
        // Curated overrides for variants whose evaluator validates parameter
        // relationships the heuristic doesn't know about.
        "Rivet" => {
            // Requires body_radius < head_radius.
            m.insert("body_radius".into(), num(2.0));
            m.insert("body_length".into(), num(8.0));
            m.insert("head_radius".into(), num(3.0));
            m.insert("head_height".into(), num(1.0));
            m.insert("segments".into(), json!(16));
            Some(m)
        }
        "Crank" => {
            // Requires arm_width >= 2 * max(pivot, wrist).
            m.insert("pivot_radius".into(), num(1.5));
            m.insert("wrist_radius".into(), num(1.0));
            m.insert("arm_length".into(), num(8.0));
            m.insert("arm_width".into(), num(4.0));
            m.insert("arm_thickness".into(), num(1.0));
            m.insert("segments".into(), json!(16));
            Some(m)
        }
        "Revolve" => {
            // Endpoints on z-axis (x=0); interior x>0.
            m.insert(
                "profile".into(),
                json!({
                    "points": [
                        [0.0, 0.0],
                        [4.0, 0.0],
                        [4.0, 8.0],
                        [0.0, 8.0],
                    ]
                }),
            );
            Some(m)
        }
        "ZBeam" => {
            // Requires thickness < flange and thickness < web.
            m.insert("flange".into(), num(8.0));
            m.insert("web".into(), num(10.0));
            m.insert("thickness".into(), num(1.0));
            m.insert("depth".into(), num(20.0));
            Some(m)
        }
        "TSlot" => {
            m.insert("slot_width".into(), num(3.0));
            m.insert("slot_height".into(), num(4.0));
            m.insert("base_width".into(), num(8.0));
            m.insert("base_height".into(), num(2.0));
            m.insert("depth".into(), num(10.0));
            Some(m)
        }
        "KnurledGrip" => {
            m.insert("radius".into(), num(5.0));
            m.insert("ridge_height".into(), num(0.5));
            m.insert("height".into(), num(10.0));
            m.insert("ridge_count".into(), json!(12));
            Some(m)
        }
        "DovetailRail" => {
            // Requires top_width < width.
            m.insert("width".into(), num(10.0));
            m.insert("top_width".into(), num(6.0));
            m.insert("height".into(), num(4.0));
            m.insert("length".into(), num(20.0));
            Some(m)
        }
        "AsymmetricBracket" => {
            m.insert("leg_a_length".into(), num(10.0));
            m.insert("leg_b_length".into(), num(8.0));
            m.insert("thickness".into(), num(1.0));
            m.insert("depth".into(), num(6.0));
            Some(m)
        }
        "RibbedPlate" => {
            // rib_width * n_ribs < plate_length.
            m.insert("plate_length".into(), num(30.0));
            m.insert("plate_width".into(), num(10.0));
            m.insert("plate_thickness".into(), num(1.0));
            m.insert("n_ribs".into(), json!(4));
            m.insert("rib_width".into(), num(2.0));
            m.insert("rib_height".into(), num(1.5));
            Some(m)
        }
        "ShelfBracket" => {
            // thickness < horizontal & thickness < vertical.
            m.insert("horizontal".into(), num(8.0));
            m.insert("vertical".into(), num(10.0));
            m.insert("thickness".into(), num(1.0));
            m.insert("depth".into(), num(6.0));
            Some(m)
        }
        "NameTag" => {
            m.insert("width".into(), num(20.0));
            m.insert("height".into(), num(10.0));
            m.insert("thickness".into(), num(1.0));
            m.insert("corner_radius".into(), num(1.5));
            m.insert("hole_radius".into(), num(0.8));
            m.insert("hole_offset_from_top".into(), num(2.0));
            m.insert("segments".into(), json!(12));
            Some(m)
        }
        "BoneShape" => {
            // shaft_radius < end_radius.
            m.insert("end_radius".into(), num(3.0));
            m.insert("shaft_radius".into(), num(1.5));
            m.insert("shaft_length".into(), num(10.0));
            m.insert("stacks".into(), json!(4));
            m.insert("slices".into(), json!(8));
            Some(m)
        }
        "Bishop" => {
            // Eval requires bod_r < br, hr >= bod_r, sw < 2*hr, segments>=6, stacks>=2.
            m.insert("base_radius".into(), num(6.0));
            m.insert("base_height".into(), num(2.0));
            m.insert("body_radius".into(), num(3.0));
            m.insert("body_height".into(), num(10.0));
            m.insert("head_radius".into(), num(3.5));
            m.insert("slot_width".into(), num(1.0));
            m.insert("segments".into(), json!(16));
            m.insert("stacks".into(), json!(4));
            Some(m)
        }
        "CamLobe" => {
            // hole < rx and hole < ry.
            m.insert("rx".into(), num(5.0));
            m.insert("ry".into(), num(3.0));
            m.insert("hole_radius".into(), num(1.0));
            m.insert("thickness".into(), num(2.0));
            m.insert("segments".into(), json!(16));
            Some(m)
        }
        "TriangularPlate" => {
            m.insert("a".into(), json!([0.0, 0.0]));
            m.insert("b".into(), json!([10.0, 0.0]));
            m.insert("c".into(), json!([5.0, 8.0]));
            m.insert("thickness".into(), num(1.0));
            Some(m)
        }
        "SpiralPlate" => {
            m.insert("max_radius".into(), num(8.0));
            m.insert("revolutions".into(), num(2.0));
            m.insert("rod_radius".into(), num(0.4));
            m.insert("z".into(), num(1.0));
            m.insert("segments_per_revolution".into(), json!(12));
            Some(m)
        }
        "Crescent" => {
            // Inner partially overlaps outer: 0 < offset < outer + inner,
            // and inner < outer.
            m.insert("outer_radius".into(), num(8.0));
            m.insert("inner_radius".into(), num(6.0));
            m.insert("offset".into(), num(3.0));
            m.insert("thickness".into(), num(1.0));
            m.insert("segments".into(), json!(16));
            Some(m)
        }
        "Hourglass" => {
            // waist < end.
            m.insert("end_radius".into(), num(5.0));
            m.insert("waist_radius".into(), num(2.0));
            m.insert("half_height".into(), num(5.0));
            m.insert("segments".into(), json!(16));
            Some(m)
        }
        "Diabolo" => {
            // waist > end.
            m.insert("end_radius".into(), num(2.0));
            m.insert("waist_radius".into(), num(5.0));
            m.insert("half_height".into(), num(5.0));
            m.insert("segments".into(), json!(16));
            Some(m)
        }
        "TripleStep" => {
            // top < middle < bottom radii.
            m.insert("bottom_radius".into(), num(8.0));
            m.insert("bottom_height".into(), num(2.0));
            m.insert("middle_radius".into(), num(5.0));
            m.insert("middle_height".into(), num(2.0));
            m.insert("top_radius".into(), num(3.0));
            m.insert("top_height".into(), num(2.0));
            m.insert("segments".into(), json!(16));
            Some(m)
        }
        "KeyholeShape" => {
            // circle radius needs to fit inside plate; slot inside plate too.
            m.insert("circle_radius".into(), num(2.0));
            m.insert("slot_width".into(), num(1.5));
            m.insert("slot_height".into(), num(4.0));
            m.insert("plate_width".into(), num(8.0));
            m.insert("plate_height".into(), num(12.0));
            m.insert("plate_thickness".into(), num(1.0));
            m.insert("segments".into(), json!(16));
            Some(m)
        }
        "Volute" => {
            // rod < center_disk < max_radius, revolutions > 0.
            m.insert("max_radius".into(), num(8.0));
            m.insert("revolutions".into(), num(2.0));
            m.insert("rod_radius".into(), num(0.4));
            m.insert("center_disk_radius".into(), num(1.5));
            m.insert("thickness".into(), num(1.0));
            m.insert("segments_per_revolution".into(), json!(12));
            m.insert("center_segments".into(), json!(12));
            Some(m)
        }
        "TableTop" => {
            // legs fit in slab: leg_inset+leg_radius < top dimensions/2.
            m.insert("top_width".into(), num(40.0));
            m.insert("top_depth".into(), num(20.0));
            m.insert("top_thickness".into(), num(1.5));
            m.insert("leg_radius".into(), num(1.0));
            m.insert("leg_height".into(), num(20.0));
            m.insert("leg_inset".into(), num(2.0));
            m.insert("segments".into(), json!(16));
            Some(m)
        }
        "Hammer" => {
            // handle fits in head: handle_radius < min(head_width, head_height)/2.
            m.insert("head_length".into(), num(8.0));
            m.insert("head_width".into(), num(3.0));
            m.insert("head_height".into(), num(3.0));
            m.insert("handle_radius".into(), num(0.6));
            m.insert("handle_length".into(), num(20.0));
            m.insert("segments".into(), json!(16));
            Some(m)
        }
        "ScrewDriver" => {
            // shaft_radius < handle_radius.
            m.insert("handle_radius".into(), num(2.5));
            m.insert("handle_length".into(), num(8.0));
            m.insert("shaft_radius".into(), num(0.5));
            m.insert("shaft_length".into(), num(15.0));
            m.insert("tip_width".into(), num(1.5));
            m.insert("tip_thickness".into(), num(0.3));
            m.insert("tip_length".into(), num(1.0));
            m.insert("segments".into(), json!(16));
            Some(m)
        }
        "Cross3D" => {
            // arm_span > width; arm_z + arm_height <= height.
            m.insert("width".into(), num(2.0));
            m.insert("depth".into(), num(2.0));
            m.insert("height".into(), num(10.0));
            m.insert("arm_span".into(), num(8.0));
            m.insert("arm_height".into(), num(2.0));
            m.insert("arm_z".into(), num(6.0));
            Some(m)
        }
        "Chair" => {
            // legs+back fit in seat.
            m.insert("seat_size".into(), num(10.0));
            m.insert("seat_thickness".into(), num(1.0));
            m.insert("leg_height".into(), num(12.0));
            m.insert("leg_thickness".into(), num(1.0));
            m.insert("leg_inset".into(), num(1.0));
            m.insert("back_thickness".into(), num(1.0));
            m.insert("back_height".into(), num(12.0));
            Some(m)
        }
        // --- Manufacturing batch 5 (2026-05-10) ---
        "ChamferedHole" => {
            m.insert("input".into(), Value::String("body".into()));
            m.insert("center".into(), json!([5.0, 5.0, 10.0]));
            m.insert("axis".into(), Value::String("z".into()));
            m.insert("hole_radius".into(), num(1.5));
            m.insert("hole_depth".into(), num(8.0));
            m.insert("chamfer_radius".into(), num(2.5));
            m.insert("chamfer_depth".into(), num(1.0));
            Some(m)
        }
        "ThreadedHoleMarker" => {
            m.insert("input".into(), Value::String("body".into()));
            m.insert("center".into(), json!([5.0, 5.0, 10.0]));
            m.insert("axis".into(), Value::String("z".into()));
            m.insert("thread_diameter".into(), num(3.0));
            m.insert("depth".into(), num(8.0));
            m.insert("thread_pitch".into(), num(0.5));
            Some(m)
        }
        "BoltPattern" => {
            m.insert("input".into(), Value::String("body".into()));
            m.insert("center".into(), json!([5.0, 5.0, 10.0]));
            m.insert("axis".into(), Value::String("z".into()));
            m.insert("pattern_radius".into(), num(3.5));
            m.insert("hole_radius".into(), num(0.6));
            m.insert("hole_depth".into(), num(8.0));
            m.insert("count".into(), json!(4));
            m.insert("phase".into(), num(0.0));
            Some(m)
        }
        "SquareDrive" => {
            m.insert("input".into(), Value::String("body".into()));
            m.insert("center".into(), json!([5.0, 5.0, 10.0]));
            m.insert("axis".into(), Value::String("z".into()));
            m.insert("side_length".into(), num(3.0));
            m.insert("depth".into(), num(6.0));
            Some(m)
        }
        "RaisedBoss" => {
            m.insert("input".into(), Value::String("body".into()));
            m.insert("center".into(), json!([5.0, 5.0, 10.0]));
            m.insert("axis".into(), Value::String("z".into()));
            m.insert("boss_radius".into(), num(2.0));
            m.insert("boss_height".into(), num(4.0));
            m.insert("hole_radius".into(), num(0.8));
            m.insert("hole_depth".into(), num(3.0));
            Some(m)
        }
        "TruncatedSphere" => {
            m.insert("radius".into(), num(5.0));
            m.insert("clip_z".into(), num(0.0)); // hemisphere by default
            m.insert("segments".into(), json!(12));
            Some(m)
        }
        "Lens2" => {
            m.insert("radius".into(), num(5.0));
            m.insert("thickness".into(), num(4.0));
            m.insert("segments".into(), json!(12));
            Some(m)
        }
        "Capsule2" => {
            m.insert("radius".into(), num(3.0));
            m.insert("length".into(), num(8.0));
            m.insert("axis".into(), Value::String("z".into()));
            m.insert("segments".into(), json!(12));
            Some(m)
        }
        "OvoidShell" => {
            m.insert("radius_min".into(), num(4.0));
            m.insert("radius_max".into(), num(4.0));
            m.insert("length".into(), num(8.0));
            m.insert("thickness".into(), num(0.5));
            m.insert("segments".into(), json!(12));
            Some(m)
        }
        "DishCap" => {
            // depth must be <= radius.
            m.insert("radius".into(), num(5.0));
            m.insert("depth".into(), num(2.0));
            m.insert("rim_width".into(), num(1.0));
            m.insert("segments".into(), json!(16));
            Some(m)
        }
        "PaperLanternStrips" => {
            // strip_count >= 1, swd in (0, 360).
            m.insert("axis_radius".into(), num(3.0));
            m.insert("minor_radius".into(), num(0.5));
            m.insert("strip_count".into(), json!(6_u64));
            m.insert("strip_width_deg".into(), num(30.0));
            m.insert("segments".into(), json!(12_u64));
            Some(m)
        }
        "Trefoil" => {
            // segments_along >= 6, segments_around >= 4.
            m.insert("scale".into(), num(2.0));
            m.insert("tube_radius".into(), num(0.3));
            m.insert("segments_along".into(), json!(18_u64));
            m.insert("segments_around".into(), json!(8_u64));
            Some(m)
        }
        "AcornShapeDome" => {
            // positive dims.
            m.insert("base_radius".into(), num(3.0));
            m.insert("height".into(), num(3.0));
            m.insert("point_height".into(), num(2.0));
            m.insert("segments".into(), json!(16_u64));
            Some(m)
        }
        "Bagel" => {
            // major > minor, dome_height <= minor.
            m.insert("major_radius".into(), num(5.0));
            m.insert("minor_radius".into(), num(1.2));
            m.insert("dome_height".into(), num(0.6));
            m.insert("segments".into(), json!(12));
            Some(m)
        }
        "Pringle" => {
            m.insert("side".into(), num(8.0));
            m.insert("dome_height".into(), num(2.0));
            m.insert("segments".into(), json!(4));
            Some(m)
        }
        "Cone2" => {
            // base_radius > tip_radius > 0 for frustum+cap variant.
            m.insert("base_radius".into(), num(5.0));
            m.insert("tip_radius".into(), num(1.5));
            m.insert("height".into(), num(8.0));
            m.insert("segments".into(), json!(12));
            Some(m)
        }
        "Lozenge" => {
            // corner_radius < min(size[0], size[1]) / 2.
            m.insert("size".into(), json!([8.0, 6.0, 4.0]));
            m.insert("corner_radius".into(), num(1.5));
            m.insert("segments".into(), json!(4));
            Some(m)
        }
        "PetalCluster" => {
            // petal_count>=2, segments>=3, petal_width<petal_length.
            m.insert("petal_count".into(), json!(5_usize));
            m.insert("petal_length".into(), num(3.0));
            m.insert("petal_width".into(), num(1.0));
            m.insert("segments".into(), json!(8_usize));
            Some(m)
        }
        "HeartSolid" => {
            // 2*lobe_radius <= total_height.
            m.insert("lobe_radius".into(), num(1.0));
            m.insert("total_height".into(), num(3.0));
            m.insert("segments".into(), json!(12_usize));
            Some(m)
        }
        "Whisker" => {
            // length>0, wire_radius>0, segments>=3.
            m.insert("length".into(), num(5.0));
            m.insert("amplitude".into(), num(1.0));
            m.insert("wire_radius".into(), num(0.15));
            m.insert("segments".into(), json!(8_usize));
            Some(m)
        }
        "CrossShape" => {
            // arm_thickness < arm_length.
            m.insert("arm_length".into(), num(6.0));
            m.insert("arm_thickness".into(), num(1.5));
            Some(m)
        }
        "AxisTaperedTube" => {
            m.insert("start_radius".into(), num(6.0));
            m.insert("end_radius".into(), num(4.0));
            m.insert("length".into(), num(10.0));
            m.insert("axis".into(), serde_json::Value::String("z".into()));
            m.insert("segments".into(), serde_json::json!(16));
            m.insert("wall_thickness".into(), num(1.0));
            Some(m)
        }
        "PolarRevolveLoft" => {
            // Vec<Profile2D> field needs a hand-crafted example.
            // Four identical triangular sections arranged around a circle.
            let tri = serde_json::json!({
                "points": [[0.0, -1.0], [2.0, -1.0], [1.0, 1.0]]
            });
            m.insert(
                "sections".into(),
                serde_json::json!([tri, tri, tri, tri]),
            );
            m.insert("axis_radius".into(), num(6.0));
            m.insert("axis".into(), serde_json::Value::String("z".into()));
            m.insert("segments_around".into(), serde_json::json!(4));
            Some(m)
        }
        // -------------------------------------------------------------------
        // Manufacturing batch 5 curated overrides.
        // -------------------------------------------------------------------
        "ShaftOilHole" => {
            m.insert("input".into(), Value::String("body".into()));
            m.insert("center".into(), json!([5.0, 5.0, 5.0]));
            m.insert("axis".into(), Value::String("z".into()));
            m.insert("radius".into(), num(0.5));
            m.insert("depth".into(), num(6.0));
            m.insert("segments".into(), json!(12));
            Some(m)
        }
        "WoodruffKey" => {
            m.insert("input".into(), Value::String("body".into()));
            m.insert("center".into(), json!([5.0, 5.0, 5.0]));
            m.insert("axis".into(), Value::String("z".into()));
            m.insert("radius".into(), num(2.0));
            m.insert("width".into(), num(2.0));
            m.insert("depth".into(), num(1.5));
            m.insert("segments".into(), json!(12));
            Some(m)
        }
        "DraftedHole" => {
            m.insert("input".into(), Value::String("body".into()));
            m.insert("center".into(), json!([5.0, 5.0, 10.0]));
            m.insert("axis".into(), Value::String("z".into()));
            m.insert("top_radius".into(), num(2.5));
            m.insert("bottom_radius".into(), num(1.5));
            m.insert("depth".into(), num(8.0));
            m.insert("segments".into(), json!(12));
            Some(m)
        }
        "HexFlange" => {
            m.insert("center".into(), json!([0.0, 0.0, 0.0]));
            m.insert("axis".into(), Value::String("z".into()));
            m.insert("across_flats".into(), num(8.0));
            m.insert("height".into(), num(5.0));
            Some(m)
        }
        "Heatset" => {
            m.insert("input".into(), Value::String("body".into()));
            m.insert("center".into(), json!([5.0, 5.0, 10.0]));
            m.insert("axis".into(), Value::String("z".into()));
            m.insert("insert_radius".into(), num(1.5));
            m.insert("insert_depth".into(), num(6.0));
            m.insert("lead_in_radius".into(), num(2.0));
            m.insert("segments".into(), json!(12));
            Some(m)
        }
        _ => None,
    }
}

// ---------------------------------------------------------------------------
// Markdown rendering.
// ---------------------------------------------------------------------------

pub fn render_markdown(variants: &[Variant]) -> String {
    let mut out = String::new();
    out.push_str("# kerf-cad Feature Catalog\n\n");
    out.push_str("_Auto-generated by `cargo run -p kerf-cad --bin extract_catalog`. Do not edit by hand — change `crates/kerf-cad/src/feature.rs` doc comments instead._\n\n");
    out.push_str(&format!(
        "This catalog covers **{n}** `Feature` variants. Each entry lists its parameters and a default JSON skeleton you can paste into a `Model`. Examples that take an `input` (transforms, edge ops, holes) assume a sibling feature with id `\"body\"` (a 10x10x10 `Box`) exists in the model; booleans assume both `\"body\"` and `\"tool\"` exist.\n\n",
        n = variants.len()
    ));

    out.push_str("## Categories\n\n");
    let mut by_cat: std::collections::BTreeMap<usize, Vec<&Variant>> =
        std::collections::BTreeMap::new();
    for v in variants {
        let cat = categorize(&v.name);
        let pos = Category::order().iter().position(|c| *c == cat).unwrap_or(0);
        by_cat.entry(pos).or_default().push(v);
    }
    for (&pos, vs) in &by_cat {
        let cat = Category::order()[pos];
        let anchor = anchor_of(cat.title());
        out.push_str(&format!(
            "- [{title}](#{anchor}) ({n})\n",
            title = cat.title(),
            anchor = anchor,
            n = vs.len()
        ));
    }
    out.push('\n');

    for (&pos, vs) in &by_cat {
        let cat = Category::order()[pos];
        out.push_str(&format!("## {}\n\n", cat.title()));
        let mut vs_sorted = vs.clone();
        vs_sorted.sort_by(|a, b| a.name.cmp(&b.name));
        for v in vs_sorted {
            render_variant(&mut out, v);
        }
    }

    out
}

fn anchor_of(title: &str) -> String {
    title
        .to_lowercase()
        .chars()
        .filter_map(|c| match c {
            'a'..='z' | '0'..='9' => Some(c),
            ' ' | '-' => Some('-'),
            _ => None,
        })
        .collect()
}

fn render_variant(out: &mut String, v: &Variant) {
    out.push_str(&format!("### `{}`\n\n", v.name));
    let summary = first_sentence(&v.doc);
    if !summary.is_empty() {
        out.push_str(&format!("{}\n\n", summary));
    } else {
        out.push_str("_(undocumented - add a doc comment to the variant in `feature.rs`.)_\n\n");
    }
    if !v.fields.is_empty() {
        out.push_str("**Parameters**\n\n");
        out.push_str("| field | type |\n|---|---|\n");
        for f in &v.fields {
            out.push_str(&format!("| `{}` | `{}` |\n", f.name, f.ty));
        }
        out.push('\n');
    }
    let example = default_example_json(v);
    let pretty = serde_json::to_string_pretty(&example).unwrap_or_else(|_| "{}".into());
    out.push_str("**Example**\n\n```json\n");
    out.push_str(&pretty);
    out.push_str("\n```\n\n");
}

fn first_sentence(doc: &str) -> String {
    let trimmed = doc.trim();
    if trimmed.is_empty() {
        return String::new();
    }
    let para = trimmed.split("\n\n").next().unwrap_or(trimmed);
    para.replace('\n', " ").trim().to_string()
}
