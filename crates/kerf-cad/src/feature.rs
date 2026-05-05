//! Feature: one node in the model DAG. Carries its id, inputs, and parameters.

use serde::{Deserialize, Serialize};

use crate::scalar::Scalar;

/// A 2D polygonal profile, used by `ExtrudePolygon`. Points are in the XY
/// plane and consumed in order. The polygon should be simple (non-self-
/// intersecting) and CCW for the extrusion to come out outward-facing.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct Profile2D {
    pub points: Vec<[Scalar; 2]>,
}

/// One edge entry in a `Fillets` (plural) feature.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct FilletEdge {
    pub axis: String,
    pub edge_min: [Scalar; 3],
    pub edge_length: Scalar,
    pub radius: Scalar,
    pub quadrant: String,
    pub segments: usize,
}

/// One operation in the model DAG.
///
/// Every variant has an `id` (its key in the model) and either parameters
/// (for primitives) or `input`/`inputs` referencing other feature ids
/// (for transforms and booleans).
///
/// Booleans take 2+ inputs and fold left: `Union { inputs: [a,b,c] }`
/// evaluates as `(a ∪ b) ∪ c`.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(tag = "kind")]
pub enum Feature {
    Box {
        id: String,
        extents: [Scalar; 3],
    },
    BoxAt {
        id: String,
        extents: [Scalar; 3],
        origin: [Scalar; 3],
    },
    /// Faceted (polyhedral) cylinder. `segments` is the number of sides;
    /// it is structural, not a measurement, so it stays a literal `usize`.
    Cylinder {
        id: String,
        radius: Scalar,
        height: Scalar,
        segments: usize,
    },
    Sphere {
        id: String,
        radius: Scalar,
    },
    Torus {
        id: String,
        major_radius: Scalar,
        minor_radius: Scalar,
    },
    Cone {
        id: String,
        radius: Scalar,
        height: Scalar,
    },
    Frustum {
        id: String,
        top_radius: Scalar,
        bottom_radius: Scalar,
        height: Scalar,
    },
    ExtrudePolygon {
        id: String,
        profile: Profile2D,
        direction: [Scalar; 3],
    },
    /// Loft (skin) between two parallel polygons of the same vertex count.
    /// `bottom` lies in the xy plane (z=0); `top` is the same number of
    /// (x, y) points lifted to z = `height`. Both must have the same
    /// length ≥ 3, both CCW from +z. Side faces are quads (will be
    /// non-planar if the two polygons differ in shape — the kernel
    /// stores those as "best-fit planar" with reduced boolean
    /// robustness).
    Loft {
        id: String,
        bottom: Profile2D,
        top: Profile2D,
        height: Scalar,
    },
    /// Tapered extrude: extrude `profile` along +z by `height`, scaling
    /// the top profile by `top_scale` around the centroid. `top_scale`
    /// in (0, ∞); 1.0 is a normal prism, < 1 tapers inward (draft), > 1
    /// tapers outward.
    TaperedExtrude {
        id: String,
        profile: Profile2D,
        height: Scalar,
        top_scale: Scalar,
    },
    /// Revolve an open polyline in the xz-plane around the z-axis. The first
    /// and last points must lie on the z-axis (x = 0); all interior points
    /// must have x > 0. Produces a closed axisymmetric solid.
    Revolve {
        id: String,
        profile: Profile2D,
    },

    /// Subtract an axis-aligned box (a "corner cutter") from `input`. The
    /// cutter has its `min` corner at `corner` and the given `extents`. Useful
    /// as quick-and-dirty chamfer-by-cut or "flatten this corner of a
    /// bounding box". Composes as `Difference(input, BoxAt(extents, corner))`.
    CornerCut {
        id: String,
        input: String,
        corner: [Scalar; 3],
        extents: [Scalar; 3],
    },

    /// Round an axis-aligned 90° edge of `input` with a quarter-circle of
    /// radius `radius`.
    ///
    /// `axis` is the edge direction ("x" | "y" | "z"). `edge_min` is one
    /// endpoint of the edge (the start in +axis direction); `edge_length` is
    /// the distance to the other endpoint. `quadrant` is two characters,
    /// each "p" or "n", giving the body's direction relative to the edge in
    /// the two perpendicular axes (in canonical (a, b) order — for axis "z"
    /// that's (x, y); for axis "x" that's (y, z); for axis "y" that's (z, x)).
    /// `segments` is the polygonal approximation count for the rounded
    /// arc (≥ 3).
    Fillet {
        id: String,
        input: String,
        axis: String,
        edge_min: [Scalar; 3],
        edge_length: Scalar,
        radius: Scalar,
        quadrant: String,
        segments: usize,
    },

    /// Subtract a linear array of through-holes from `input`. The
    /// first hole is at `start` (center of the +axis-facing opening);
    /// successive holes are spaced by `offset` (3D vector). `count` ≥ 1.
    /// Each hole is a cylinder of `radius`, axis along `axis`, length
    /// equal to `depth` extending in -axis direction (with overhang).
    HoleArray {
        id: String,
        input: String,
        axis: String,
        start: [Scalar; 3],
        offset: [Scalar; 3],
        count: usize,
        radius: Scalar,
        depth: Scalar,
        segments: usize,
    },

    /// Subtract a polar array of through-holes around `center`. Each
    /// hole is at `bolt_circle_radius` from center, distributed evenly
    /// around `axis` ("x"|"y"|"z"). `count` ≥ 1 holes, `radius` each,
    /// length `depth` along -axis from `center` (which sits on the
    /// +axis-facing opening surface).
    BoltCircle {
        id: String,
        input: String,
        axis: String,
        center: [Scalar; 3],
        bolt_circle_radius: Scalar,
        count: usize,
        radius: Scalar,
        depth: Scalar,
        segments: usize,
    },

    /// Subtract a hex-shaped through-pocket. `top_center` is the center
    /// of the opening on the +axis-facing surface. `inscribed_radius`
    /// (apothem) sets the hex size — distance from center to flat side.
    /// `depth` is the pocket length along -axis.
    HexHole {
        id: String,
        input: String,
        axis: String,
        top_center: [Scalar; 3],
        inscribed_radius: Scalar,
        depth: Scalar,
    },

    /// Subtract a square through-pocket. `top_center` on the +axis
    /// surface, `side` length, `depth` along -axis.
    SquareHole {
        id: String,
        input: String,
        axis: String,
        top_center: [Scalar; 3],
        side: Scalar,
        depth: Scalar,
    },

    /// Countersink: subtract a hole with a conical chamfer at the top
    /// (the +axis-facing surface of the body). The straight drill portion
    /// has `drill_radius` and runs the full `total_depth`. The conical
    /// portion expands from `drill_radius` at depth `csink_depth` to
    /// `csink_radius` at the top surface (depth 0). `axis` chooses the
    /// drill direction: the hole goes INTO the body in -axis from the
    /// `top_center` opening.
    Countersink {
        id: String,
        input: String,
        axis: String,
        top_center: [Scalar; 3],
        drill_radius: Scalar,
        csink_radius: Scalar,
        csink_depth: Scalar,
        total_depth: Scalar,
        segments: usize,
    },

    /// Counterbore: subtract a stepped hole from `input`. The "drill"
    /// portion has `drill_radius` and runs full `total_depth`. The
    /// "counterbore" portion is a wider cylinder of `cbore_radius` and
    /// `cbore_depth`, sitting at the top (the +axis end of the hole).
    /// `axis` is "x"|"y"|"z" — the drill direction is -axis (the hole
    /// goes INTO the body from the +axis-facing surface). `top_center`
    /// is the center of the counterbore opening on that surface.
    Counterbore {
        id: String,
        input: String,
        axis: String,
        top_center: [Scalar; 3],
        drill_radius: Scalar,
        cbore_radius: Scalar,
        cbore_depth: Scalar,
        total_depth: Scalar,
        segments: usize,
    },

    /// Multiple `Fillet`s applied to the same `input` in one operation.
    /// Builds each fillet's wedge cutter relative to the unmodified input,
    /// unions the wedges, and subtracts the composite cutter once. This
    /// avoids the "second fillet meets first fillet's curved face" failure
    /// that single-`Fillet` stacking has.
    ///
    /// Edges with overlapping wedge regions still won't work (those
    /// wedges' union itself fails). Each entry has the same fields as
    /// `Fillet` minus `id` and `input`.
    Fillets {
        id: String,
        input: String,
        edges: Vec<FilletEdge>,
    },

    /// Bevel an axis-aligned 90° edge of `input` by a 45° flat cut of
    /// `setback` (distance from the edge that the cut starts on each face).
    ///
    /// Same `axis`, `edge_min`, `edge_length`, `quadrant` semantics as
    /// `Fillet`. The cutter is a triangular prism (no rounding, no
    /// `segments`).
    Chamfer {
        id: String,
        input: String,
        axis: String,
        edge_min: [Scalar; 3],
        edge_length: Scalar,
        setback: Scalar,
        quadrant: String,
    },

    /// Stadium-shaped slot: a rounded-rectangle profile extruded along
    /// `direction`. The slot has axis-aligned (in the local profile frame)
    /// straight sides of `length` connecting two semicircles of `radius`.
    /// Currently fixed to lie in the xy plane, extruded in +z.
    Slot {
        id: String,
        length: Scalar,
        radius: Scalar,
        height: Scalar,
        segments: usize,
    },

    /// Closed-end hollow cylinder: tube with `end_thickness`-thick caps.
    /// `outer_radius`, `inner_radius`, `height`, `end_thickness`,
    /// `segments`. Inner cavity has length `height - 2 * end_thickness`.
    HollowCylinder {
        id: String,
        outer_radius: Scalar,
        inner_radius: Scalar,
        height: Scalar,
        end_thickness: Scalar,
        segments: usize,
    },

    /// Right-triangular prism: triangular cross-section with legs of
    /// length `width` (along x) and `height` (along z), extruded along
    /// `depth` (along y). Hypotenuse runs in the xz-plane from
    /// (width, 0, 0) to (0, 0, height).
    Wedge {
        id: String,
        width: Scalar,
        depth: Scalar,
        height: Scalar,
    },

    /// Regular `n`-gon prism: base is a regular polygon of `segments` sides
    /// inscribed in a circle of `radius`, extruded along +z by `height`.
    /// At `segments = 4` the result is a square prism (rotated by π/4 of
    /// the n-gon phase, like cylinder_faceted's convention).
    RegularPrism {
        id: String,
        radius: Scalar,
        height: Scalar,
        segments: usize,
    },

    /// Right pyramid with a regular `n`-gon base of `radius` (circumradius)
    /// at z=0 and an apex at (0, 0, height). `segments` is the base
    /// sidedness (≥ 3). Uses the kerf `cone_faceted` primitive — fully
    /// planar, composes with booleans (the analytic `Cone` does not).
    Pyramid {
        id: String,
        radius: Scalar,
        height: Scalar,
        segments: usize,
    },

    /// Truncated pyramid (frustum-shaped n-gon prism with two parallel
    /// regular polygons). Bottom n-gon of `bottom_radius` at z=0, top
    /// n-gon of `top_radius` at z=`height`. Both phase-shifted by π/n.
    /// `segments` is the polygon sidedness (≥ 3). Both radii must be > 0
    /// (use `Pyramid` for a true apex).
    TruncatedPyramid {
        id: String,
        bottom_radius: Scalar,
        top_radius: Scalar,
        height: Scalar,
        segments: usize,
    },

    /// Capsule (cylinder with hemispherical caps) along +z. The
    /// cylindrical body has `radius` and `height`, sitting at z ∈ [0, h].
    /// Two hemispheres of `radius` cap the bottom (z=0) and top
    /// (z=`height`). Total length along z = height + 2*radius.
    /// `stacks` and `slices` set the sphere tessellation; the cylinder
    /// uses `slices` for its lateral faceting too.
    Capsule {
        id: String,
        radius: Scalar,
        height: Scalar,
        stacks: usize,
        slices: usize,
    },

    /// PipeRun: a chain of cylindrical pipe segments along a polyline of
    /// axis-aligned points. Each consecutive pair of points must differ in
    /// EXACTLY ONE coordinate (i.e., the segment is axis-aligned). At each
    /// turn (interior point) a hemispherical cap of `radius` joins the
    /// segments. Result is the union of all segments + interior hemispheres.
    /// Useful for plumbing layouts.
    PipeRun {
        id: String,
        points: Vec<[Scalar; 3]>,
        radius: Scalar,
        segments: usize,
    },

    /// UV-style faceted sphere centered on the origin. Use this instead
    /// of the analytic `Sphere` whenever you want to compose with
    /// booleans (`Sphere`'s 1-face/0-edges topology breaks the engine).
    /// `stacks` is the number of latitude bands (≥ 2), `slices` is the
    /// longitude segments (≥ 3). Topology is V = 2 + (stacks-1) * slices.
    SphereFaceted {
        id: String,
        radius: Scalar,
        stacks: usize,
        slices: usize,
    },

    /// Hollow spherical shell: outer faceted sphere of `outer_radius`
    /// minus an inner faceted sphere of `inner_radius`. Concentric on
    /// the origin. `stacks` and `slices` apply to both spheres for
    /// matching topology.
    HollowSphere {
        id: String,
        outer_radius: Scalar,
        inner_radius: Scalar,
        stacks: usize,
        slices: usize,
    },

    /// Hemisphere (dome): top half of a faceted sphere of `radius`,
    /// sitting on the xy plane. Built as `sphere_faceted - cutter_box`
    /// where the cutter clips everything below z=0.
    Dome {
        id: String,
        radius: Scalar,
        stacks: usize,
        slices: usize,
    },

    /// Cylinder at an axis-aligned position with chosen edge axis. The
    /// cylinder runs along `axis` ("x" | "y" | "z") starting from
    /// `base[axis_idx]` and extending by `height` in +axis direction.
    /// `base` is the center of the bottom cap (in world coordinates).
    /// Uses exact coordinate permutation for orientation to keep the
    /// boolean engine happy.
    CylinderAt {
        id: String,
        base: [Scalar; 3],
        axis: String,
        radius: Scalar,
        height: Scalar,
        segments: usize,
    },

    /// L-shape (right-angle bracket) cross-section extruded along +z.
    /// `width` is the leg along +x, `height` is the leg along +y, both
    /// measured from the inner corner. `thickness` is the leg
    /// thickness. `depth` is the extrusion length along +z. Inner
    /// corner is at (thickness, thickness). Result fits in an axis-
    /// aligned bounding box of (width, height, depth).
    LBracket {
        id: String,
        width: Scalar,
        height: Scalar,
        thickness: Scalar,
        depth: Scalar,
    },

    /// U-channel cross-section extruded along +z. `width` is the
    /// outside x-extent, `height` is the y-extent of the legs (web
    /// at y=0). `thickness` is the wall thickness for both legs and
    /// the web. `depth` is the extrusion length. Result occupies
    /// x ∈ [0, width], y ∈ [0, height], z ∈ [0, depth].
    UChannel {
        id: String,
        width: Scalar,
        height: Scalar,
        thickness: Scalar,
        depth: Scalar,
    },

    /// T-beam cross-section extruded along +z. The flange is a
    /// horizontal bar of `flange_width` × `flange_thickness` sitting on
    /// top; the web is a vertical bar of `web_thickness` ×
    /// (`total_height` − `flange_thickness`) below it, centered along
    /// the flange. `depth` is the extrusion length.
    TBeam {
        id: String,
        flange_width: Scalar,
        flange_thickness: Scalar,
        web_thickness: Scalar,
        total_height: Scalar,
        depth: Scalar,
    },

    /// Hex nut: hexagonal prism (apothem `inscribed_radius`,
    /// `thickness`) with a centered through-hole of `bore_radius`.
    /// Sits at z ∈ [0, thickness].
    Nut {
        id: String,
        inscribed_radius: Scalar,
        bore_radius: Scalar,
        thickness: Scalar,
        segments: usize,
    },

    /// Flat washer: outer cylinder of `outer_radius` and `thickness`
    /// minus a centered bore of `inner_radius`. Sits at z ∈ [0, thickness].
    Washer {
        id: String,
        outer_radius: Scalar,
        inner_radius: Scalar,
        thickness: Scalar,
        segments: usize,
    },

    /// Cylindrical boss raised from xy plane: a Cylinder of `radius`
    /// and `height`, positioned at `base` (center of the bottom cap).
    /// Provided as a feature for ergonomics — equivalent to
    /// `Translate(Cylinder(...), base)`.
    RoundBoss {
        id: String,
        base: [Scalar; 3],
        radius: Scalar,
        height: Scalar,
        segments: usize,
    },

    /// Rectangular boss raised from xy plane: a Box of `extents`
    /// positioned at `corner`. Provided as a feature for ergonomics —
    /// equivalent to `BoxAt(extents, corner)`.
    RectBoss {
        id: String,
        corner: [Scalar; 3],
        extents: [Scalar; 3],
    },

    /// Trapezoidal slot (dovetail) cross-section extruded along +y by
    /// `length`. The trapezoid's `top_width` (wider, at z=`depth`) and
    /// `bottom_width` (narrower, at z=0) sit on x ∈ [-W/2, +W/2]; the
    /// slot extends in z from 0 to `depth`. Typical dovetail uses
    /// `top_width > bottom_width`. Use as a cutter via Difference.
    DovetailSlot {
        id: String,
        bottom_width: Scalar,
        top_width: Scalar,
        depth: Scalar,
        length: Scalar,
    },

    /// V-groove cutter: triangular cross-section extruded along +y by
    /// `length`. The notch opens at z=`depth` with width `top_width`,
    /// converges to a sharp edge at z=0 (a single point along x=0).
    /// Use as a cutter via Difference.
    VeeGroove {
        id: String,
        top_width: Scalar,
        depth: Scalar,
        length: Scalar,
    },

    /// Hex-head bolt: hexagonal head + cylindrical shaft, both along
    /// +z. The head is a `RegularPrism { sides: 6 }` of `head_inscribed_radius`
    /// (apothem) and `head_thickness`, sitting at z ∈ [0, head_thickness].
    /// The shaft is a Cylinder of `shaft_radius` and `shaft_length`,
    /// sitting at z ∈ [head_thickness, head_thickness + shaft_length].
    /// Result is a single solid: head ∪ shaft.
    Bolt {
        id: String,
        head_inscribed_radius: Scalar,
        head_thickness: Scalar,
        shaft_radius: Scalar,
        shaft_length: Scalar,
        segments: usize,
    },

    /// Socket-head cap screw (SHCS): cylindrical head + cylindrical
    /// shaft. Head is at z ∈ [0, head_thickness] with radius
    /// `head_radius`; shaft is at z ∈ [head_thickness, head_thickness +
    /// shaft_length] with `shaft_radius`. Result is head ∪ shaft.
    CapScrew {
        id: String,
        head_radius: Scalar,
        head_thickness: Scalar,
        shaft_radius: Scalar,
        shaft_length: Scalar,
        segments: usize,
    },

    /// I-beam (H-section) cross-section extruded along +z. Symmetric
    /// top and bottom flanges of `flange_width` × `flange_thickness`,
    /// centered web of `web_thickness` × (`total_height` − 2 ×
    /// `flange_thickness`). `depth` is the extrusion length.
    IBeam {
        id: String,
        flange_width: Scalar,
        flange_thickness: Scalar,
        web_thickness: Scalar,
        total_height: Scalar,
        depth: Scalar,
    },

    /// Tube (hollow cylinder) at an axis-aligned position with chosen
    /// edge axis. Same orientation rules as `CylinderAt`. Inner cylinder
    /// is automatically extended past both caps so the bore is a clean
    /// through-hole regardless of orientation.
    TubeAt {
        id: String,
        base: [Scalar; 3],
        axis: String,
        outer_radius: Scalar,
        inner_radius: Scalar,
        height: Scalar,
        segments: usize,
    },

    /// Regular n-pointed star prism: profile is a star polygon with
    /// `points` outer tips at `outer_radius` and `points` inner valleys
    /// at `inner_radius`, alternating. Extruded along +z by `height`.
    /// `inner_radius` must be < `outer_radius`.
    Star {
        id: String,
        points: usize,
        outer_radius: Scalar,
        inner_radius: Scalar,
        height: Scalar,
    },

    /// Hollow circular tube: outer cylinder minus a centered inner cylinder.
    /// Both share the same axis (origin, +z direction) and height.
    Tube {
        id: String,
        outer_radius: Scalar,
        inner_radius: Scalar,
        height: Scalar,
        segments: usize,
    },
    /// Box with a uniform inset wall thickness on all six faces. Equivalent
    /// to BoxAt(extents, [0,0,0]) minus BoxAt(extents - 2*wall, [wall,wall,wall]).
    HollowBox {
        id: String,
        extents: [Scalar; 3],
        wall_thickness: Scalar,
    },

    Translate {
        id: String,
        input: String,
        offset: [Scalar; 3],
    },
    /// Uniform scale of `input` around the origin. `factor` must be > 0.
    /// Volume scales as `factor³`.
    Scale {
        id: String,
        input: String,
        factor: Scalar,
    },
    /// Non-uniform scale: independent factors per axis. Only valid when
    /// `input` is a fully-faceted solid (all faces planar, all edges
    /// line segments). Faceted primitives like `Box`, `Cylinder`,
    /// `SphereFaceted`, `Pyramid`, `RegularPrism`, `LBracket`,
    /// `IBeam`, etc. are valid inputs; analytic primitives like
    /// `Sphere`, `Cone`, `Frustum`, `Torus`, `Cylinder` (raw) are not.
    /// Volume scales as `sx * sy * sz`.
    ScaleXYZ {
        id: String,
        input: String,
        factors: [Scalar; 3],
    },
    Rotate {
        id: String,
        input: String,
        axis: [Scalar; 3],
        angle_deg: Scalar,
        center: [Scalar; 3],
    },
    /// Reflect `input` across the plane defined by `plane_origin` and
    /// `plane_normal`. Volume is preserved. The result is a single mirrored
    /// body (NOT unioned with the original — for a symmetric design,
    /// follow this with a Union of input + the mirrored result).
    Mirror {
        id: String,
        input: String,
        plane_origin: [Scalar; 3],
        plane_normal: [Scalar; 3],
    },

    /// Replicate `input` `count` times along `offset`, unioning all copies.
    /// `count = 1` returns the input unchanged. `count = 0` is an error.
    LinearPattern {
        id: String,
        input: String,
        count: usize,
        offset: [Scalar; 3],
    },
    /// Replicate `input` `count` times around `axis` through `center`,
    /// distributing `total_angle_deg` evenly. `total_angle_deg = 360`
    /// gives a closed circle. Copies are unioned.
    PolarPattern {
        id: String,
        input: String,
        count: usize,
        axis: [Scalar; 3],
        center: [Scalar; 3],
        total_angle_deg: Scalar,
    },

    Union {
        id: String,
        inputs: Vec<String>,
    },
    Intersection {
        id: String,
        inputs: Vec<String>,
    },
    Difference {
        id: String,
        inputs: Vec<String>,
    },
}

impl Feature {
    pub fn id(&self) -> &str {
        match self {
            Feature::Box { id, .. }
            | Feature::BoxAt { id, .. }
            | Feature::Cylinder { id, .. }
            | Feature::Sphere { id, .. }
            | Feature::Torus { id, .. }
            | Feature::Cone { id, .. }
            | Feature::Frustum { id, .. }
            | Feature::ExtrudePolygon { id, .. }
            | Feature::Loft { id, .. }
            | Feature::TaperedExtrude { id, .. }
            | Feature::Revolve { id, .. }
            | Feature::Tube { id, .. }
            | Feature::HollowBox { id, .. }
            | Feature::CornerCut { id, .. }
            | Feature::Fillet { id, .. }
            | Feature::Fillets { id, .. }
            | Feature::Chamfer { id, .. }
            | Feature::Counterbore { id, .. }
            | Feature::Countersink { id, .. }
            | Feature::Slot { id, .. }
            | Feature::HollowCylinder { id, .. }
            | Feature::Wedge { id, .. }
            | Feature::RegularPrism { id, .. }
            | Feature::Pyramid { id, .. }
            | Feature::TruncatedPyramid { id, .. }
            | Feature::SphereFaceted { id, .. }
            | Feature::HollowSphere { id, .. }
            | Feature::Dome { id, .. }
            | Feature::Capsule { id, .. }
            | Feature::PipeRun { id, .. }
            | Feature::CylinderAt { id, .. }
            | Feature::TubeAt { id, .. }
            | Feature::Star { id, .. }
            | Feature::LBracket { id, .. }
            | Feature::UChannel { id, .. }
            | Feature::TBeam { id, .. }
            | Feature::IBeam { id, .. }
            | Feature::DovetailSlot { id, .. }
            | Feature::VeeGroove { id, .. }
            | Feature::Bolt { id, .. }
            | Feature::CapScrew { id, .. }
            | Feature::Nut { id, .. }
            | Feature::Washer { id, .. }
            | Feature::RoundBoss { id, .. }
            | Feature::RectBoss { id, .. }
            | Feature::HoleArray { id, .. }
            | Feature::BoltCircle { id, .. }
            | Feature::HexHole { id, .. }
            | Feature::SquareHole { id, .. }
            | Feature::Translate { id, .. }
            | Feature::Scale { id, .. }
            | Feature::ScaleXYZ { id, .. }
            | Feature::Rotate { id, .. }
            | Feature::Mirror { id, .. }
            | Feature::LinearPattern { id, .. }
            | Feature::PolarPattern { id, .. }
            | Feature::Union { id, .. }
            | Feature::Intersection { id, .. }
            | Feature::Difference { id, .. } => id,
        }
    }

    /// Ids this feature depends on.
    pub fn inputs(&self) -> Vec<&str> {
        match self {
            Feature::Box { .. }
            | Feature::BoxAt { .. }
            | Feature::Cylinder { .. }
            | Feature::Sphere { .. }
            | Feature::Torus { .. }
            | Feature::Cone { .. }
            | Feature::Frustum { .. }
            | Feature::ExtrudePolygon { .. }
            | Feature::Loft { .. }
            | Feature::TaperedExtrude { .. }
            | Feature::Revolve { .. }
            | Feature::Tube { .. }
            | Feature::HollowBox { .. }
            | Feature::Slot { .. }
            | Feature::HollowCylinder { .. }
            | Feature::Wedge { .. }
            | Feature::RegularPrism { .. }
            | Feature::Pyramid { .. }
            | Feature::TruncatedPyramid { .. }
            | Feature::SphereFaceted { .. }
            | Feature::HollowSphere { .. }
            | Feature::Dome { .. }
            | Feature::Capsule { .. }
            | Feature::PipeRun { .. }
            | Feature::CylinderAt { .. }
            | Feature::TubeAt { .. }
            | Feature::Star { .. }
            | Feature::LBracket { .. }
            | Feature::UChannel { .. }
            | Feature::TBeam { .. }
            | Feature::IBeam { .. }
            | Feature::DovetailSlot { .. }
            | Feature::VeeGroove { .. }
            | Feature::Bolt { .. }
            | Feature::CapScrew { .. }
            | Feature::Nut { .. }
            | Feature::Washer { .. }
            | Feature::RoundBoss { .. }
            | Feature::RectBoss { .. } => Vec::new(),
            Feature::HoleArray { input, .. }
            | Feature::BoltCircle { input, .. }
            | Feature::HexHole { input, .. }
            | Feature::SquareHole { input, .. } => vec![input.as_str()],
            Feature::Translate { input, .. }
            | Feature::Scale { input, .. }
            | Feature::ScaleXYZ { input, .. }
            | Feature::Rotate { input, .. }
            | Feature::Mirror { input, .. }
            | Feature::LinearPattern { input, .. }
            | Feature::PolarPattern { input, .. }
            | Feature::CornerCut { input, .. }
            | Feature::Fillet { input, .. }
            | Feature::Fillets { input, .. }
            | Feature::Chamfer { input, .. }
            | Feature::Counterbore { input, .. }
            | Feature::Countersink { input, .. } => {
                vec![input.as_str()]
            }
            Feature::Union { inputs, .. }
            | Feature::Intersection { inputs, .. }
            | Feature::Difference { inputs, .. } => inputs.iter().map(String::as_str).collect(),
        }
    }
}
