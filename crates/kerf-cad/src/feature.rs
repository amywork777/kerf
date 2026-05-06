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
    /// Donut: faceted torus suitable for boolean composition. Unlike the
    /// analytic [`Feature::Torus`], its faces are planar quads, so it works
    /// with the kerf boolean engine. `major_segs` is the toroidal
    /// subdivision count (≥ 3), `minor_segs` the poloidal (≥ 3). Centered
    /// on the origin, axis along +z. `major_radius` must exceed
    /// `minor_radius` (otherwise the torus self-intersects).
    Donut {
        id: String,
        major_radius: Scalar,
        minor_radius: Scalar,
        major_segs: usize,
        minor_segs: usize,
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

    /// Reference point marker: a tiny faceted sphere at `position` of
    /// `marker_radius`. Visual aid for marking locations in a model
    /// (mounting holes, datum points, etc.). Does not bind to constraints.
    RefPoint {
        id: String,
        position: [Scalar; 3],
        marker_radius: Scalar,
    },

    /// Reference axis marker: a thin faceted cylinder running along
    /// `axis` ("x" | "y" | "z") for `length`, centered at `position`.
    /// Visual aid for marking axes (rotation axes, alignment lines).
    RefAxis {
        id: String,
        position: [Scalar; 3],
        axis: String,
        length: Scalar,
        marker_radius: Scalar,
    },

    /// Arrow: a cone-shaped tip on top of a cylindrical shaft. Useful
    /// for axis indicators. Total length = `shaft_length + tip_length`.
    /// The tip cone tapers from `shaft_radius` at z = shaft_length down
    /// to a point at z = shaft_length + tip_length. (Same radius at the
    /// junction avoids the multi-cylinder coplanar-flare issue that
    /// would otherwise trip the boolean engine.)
    Arrow {
        id: String,
        shaft_radius: Scalar,
        shaft_length: Scalar,
        tip_length: Scalar,
        segments: usize,
    },

    /// Funnel: a frustum (wide at top) connected to a smaller cylinder
    /// (the spout) at the bottom. The frustum sits at z ∈ [0, neck_z]
    /// with `top_radius` at z=neck_z and `neck_radius` at z=0. The
    /// spout cylinder of `neck_radius` sits at z ∈ [-spout_length, 0].
    Funnel {
        id: String,
        top_radius: Scalar,
        neck_radius: Scalar,
        neck_z: Scalar,
        spout_length: Scalar,
        segments: usize,
    },

    /// Reference plane marker: a thin faceted box marking a planar
    /// region. `position` is the center; `axis` is the normal direction
    /// ("x" | "y" | "z"); `extents` is the (width, height) of the
    /// rectangular marker (in the perpendicular axes); `marker_thickness`
    /// is the thickness along the normal.
    RefPlane {
        id: String,
        position: [Scalar; 3],
        axis: String,
        extents: [Scalar; 2],
        marker_thickness: Scalar,
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

    /// SweepPath: like PipeRun but accepts arbitrary polylines (segments
    /// in ANY direction, not just axis-aligned). Each segment becomes a
    /// cylinder of `radius` whose axis runs along (p[i+1] - p[i]).
    /// Axis-aligned segments use the exact cyclic-permutation reorientation
    /// (no sin/cos noise); diagonal segments use a Rotation3 via
    /// face_towards, which introduces ~1e-15 floating-point noise. Single
    /// diagonals are robust; chained-diagonal differences are NOT — see
    /// the kernel's `cylinder_along_axis` rationale. Sharp miters between
    /// segments (no rounded joints).
    SweepPath {
        id: String,
        points: Vec<[Scalar; 3]>,
        radius: Scalar,
        segments: usize,
    },

    /// Coil: a helical chain of cylinder segments — useful for springs,
    /// thread roots, and any helical sweep. Built on top of SweepPath: the
    /// helix is sampled at `segments_per_turn * turns + 1` points and each
    /// pair of consecutive samples becomes a cylinder of `wire_radius`.
    /// Centered on the +z axis, starts at angle 0 in the +x direction.
    Coil {
        id: String,
        coil_radius: Scalar,
        wire_radius: Scalar,
        pitch: Scalar,
        turns: Scalar,
        segments_per_turn: usize,
        wire_segments: usize,
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

    /// C-channel (a U-channel rotated 90° so it opens to +x). Same
    /// parameter set as UChannel: outer width, height, wall thickness,
    /// and extrusion depth along +z.
    CChannel {
        id: String,
        width: Scalar,
        height: Scalar,
        thickness: Scalar,
        depth: Scalar,
    },

    /// Z-beam (Z-bar) cross-section extruded along +z. Three rectangular
    /// runs forming a "Z": bottom horizontal flange of length `flange`,
    /// vertical web of height `web`, top horizontal flange of length
    /// `flange` (offset to the right by web_thickness). Each run has
    /// thickness `thickness`.
    ZBeam {
        id: String,
        flange: Scalar,
        web: Scalar,
        thickness: Scalar,
        depth: Scalar,
    },

    /// Angle iron (equal-leg L-section) — like LBracket but with both
    /// legs the same length, and chosen extrusion axis (so it works as a
    /// frame member regardless of orientation). For now the extrusion
    /// runs along +z only.
    AngleIron {
        id: String,
        leg_length: Scalar,
        thickness: Scalar,
        depth: Scalar,
    },

    /// T-slot machining feature: a rectangular slot with a wider
    /// rectangular base, used for clamping fixtures. Cross-section is
    /// like an inverted T. Extruded along +z by `depth`. Centered on x.
    TSlot {
        id: String,
        slot_width: Scalar,
        slot_height: Scalar,
        base_width: Scalar,
        base_height: Scalar,
        depth: Scalar,
    },

    /// Keyway: a rectangular slot extruded along +z that pairs with a
    /// rectangular key on a shaft. Centered on x at y=0; extends from y=0
    /// to y=`depth_into`. Useful for mating with shafts.
    Keyway {
        id: String,
        width: Scalar,
        depth_into: Scalar,
        length: Scalar,
    },

    /// Rectangular plate with rounded corners (4 quarter-circle fillets,
    /// each with the same `corner_radius`). Useful for mounting plates
    /// where sharp corners would cause stress concentrations.
    RoundedRect {
        id: String,
        width: Scalar,
        height: Scalar,
        thickness: Scalar,
        corner_radius: Scalar,
        segments: usize,
    },

    /// Hemisphere: half of a faceted UV sphere — the +z half. Useful for
    /// domes, end caps, decorative bosses. `stacks` and `slices` are the
    /// UV subdivisions; the result has a planar circular base on z=0.
    Hemisphere {
        id: String,
        radius: Scalar,
        stacks: usize,
        slices: usize,
    },

    /// Spherical cap: portion of a faceted sphere above a horizontal
    /// cutting plane at z = `radius - cap_height`. As `cap_height`
    /// approaches `2*radius` the cap approaches a full sphere; at
    /// `radius` it's a hemisphere. Built as `Sphere - HalfspaceBox`.
    SphericalCap {
        id: String,
        radius: Scalar,
        cap_height: Scalar,
        stacks: usize,
        slices: usize,
    },

    /// Bowl: hollow hemisphere shell. Outer hemisphere of `outer_radius`
    /// minus inner hemisphere of `inner_radius`. Wall thickness =
    /// outer - inner. Open top, like a dome inverted.
    Bowl {
        id: String,
        outer_radius: Scalar,
        inner_radius: Scalar,
        stacks: usize,
        slices: usize,
    },

    /// Reference: bounding-box-of-a-feature marker. Computes axis-aligned
    /// bounding box of the input feature's solid; renders as a thin wire
    /// box. Useful for visualisation and downstream constraint references.
    BoundingBoxRef {
        id: String,
        input: String,
        wire_thickness: Scalar,
    },

    /// Reference: centroid of a feature, rendered as a small octahedron at
    /// the centroid position. Computed as the volume-weighted centroid of
    /// the input solid's tessellated triangles.
    CentroidPoint {
        id: String,
        input: String,
        marker_size: Scalar,
    },

    /// MountingFlange: thin disk with a circular pattern of bolt holes.
    /// `disk_radius` is the outer flange radius, `disk_thickness` the
    /// flange thickness (along +z), `bolt_circle_radius` the radius of
    /// the bolt-hole circle, `bolt_count` the number of bolts, and
    /// `bolt_radius` the bolt-hole radius.
    MountingFlange {
        id: String,
        disk_radius: Scalar,
        disk_thickness: Scalar,
        bolt_circle_radius: Scalar,
        bolt_count: usize,
        bolt_radius: Scalar,
        segments: usize,
    },

    /// GearBlank: a faceted cylinder with `tooth_count` rectangular notches
    /// cut around its outer circumference. Not a true involute gear (those
    /// need curve-aware extrusion), but a useful approximation for
    /// visualisation and mating-surface roughing.
    GearBlank {
        id: String,
        outer_radius: Scalar,
        root_radius: Scalar,
        tooth_count: usize,
        thickness: Scalar,
        segments_per_tooth: usize,
    },

    /// KnurledGrip: a cylinder with ridges around its circumference. The
    /// ridges are an n-sided RegularPrism slightly larger than the bare
    /// cylinder, giving a knurled / gripped look.
    KnurledGrip {
        id: String,
        radius: Scalar,
        ridge_height: Scalar,
        height: Scalar,
        ridge_count: usize,
    },

    /// Pipe: cylindrical tube whose axis is the given axis-aligned line
    /// (currently restricted to "x" / "y" / "z" — like Tube but the
    /// orientation is named, not implicit z). Outer / inner radii along
    /// the axis, length along that axis from the base point.
    Pipe {
        id: String,
        base: [Scalar; 3],
        axis: String,
        outer_radius: Scalar,
        inner_radius: Scalar,
        length: Scalar,
        segments: usize,
    },

    /// Spring: a compressed-coil version of Coil. Identical parameter
    /// shape but with an `end_caps` flag — when true, attaches a small
    /// disk at each end of the coil for end-grinding the spring (a real
    /// spring detail). For now the disk is a small RegularPrism.
    Spring {
        id: String,
        coil_radius: Scalar,
        wire_radius: Scalar,
        pitch: Scalar,
        turns: Scalar,
        segments_per_turn: usize,
        wire_segments: usize,
    },

    /// Mortise: rectangular pocket cut into the +z face of a workpiece.
    /// Centered on (cx, cy), spanning (width, length, depth) where depth
    /// is along -z. Produces a freestanding pocket solid; subtract it
    /// from your workpiece via Difference.
    Mortise {
        id: String,
        center: [Scalar; 2],
        width: Scalar,
        length: Scalar,
        depth: Scalar,
    },

    /// Tenon: rectangular tongue extending from the +z face of a
    /// workpiece. Same parameter shape as Mortise; produces the
    /// freestanding tongue solid for unioning onto the workpiece.
    Tenon {
        id: String,
        center: [Scalar; 2],
        width: Scalar,
        length: Scalar,
        height: Scalar,
    },

    /// FingerJoint: a row of `count` rectangular fingers extending in
    /// +z from a base plate. Each finger is `finger_width` wide,
    /// `finger_height` tall (in z), with `gap_width` between adjacent
    /// fingers. Total row length = count * finger_width + (count-1) * gap_width.
    /// Useful for box joints in woodworking.
    FingerJoint {
        id: String,
        count: usize,
        finger_width: Scalar,
        finger_height: Scalar,
        finger_depth: Scalar,
        gap_width: Scalar,
    },

    /// DovetailRail: like DovetailSlot but extruded over a long length —
    /// used as a sliding mate. `width` is the wider rail bottom,
    /// `top_width` the narrower top, `height` the dovetail depth, and
    /// `length` the extrude distance along +z.
    DovetailRail {
        id: String,
        width: Scalar,
        top_width: Scalar,
        height: Scalar,
        length: Scalar,
    },

    /// Pulley: a cylinder with a V-groove around its outer circumference.
    /// `outer_radius` is the rim, `groove_depth` how deep the V cuts
    /// inward, `groove_width` the V opening at the rim, `width` the
    /// pulley thickness along +z.
    Pulley {
        id: String,
        outer_radius: Scalar,
        groove_depth: Scalar,
        groove_width: Scalar,
        width: Scalar,
        segments: usize,
    },

    /// Bushing: a tube with a wider flange at one end (top-hat profile).
    /// `inner_radius` is the bore, `outer_radius` the sleeve outside,
    /// `flange_radius` the wider flange disk, `flange_thickness` the
    /// flange's thickness, `body_length` the sleeve length below the
    /// flange.
    Bushing {
        id: String,
        inner_radius: Scalar,
        outer_radius: Scalar,
        flange_radius: Scalar,
        flange_thickness: Scalar,
        body_length: Scalar,
        segments: usize,
    },

    /// Sprocket: a cylinder with `tooth_count` triangular teeth around its
    /// perimeter. Same parameter shape as GearBlank but with a triangular
    /// (instead of rectangular) tooth profile.
    Sprocket {
        id: String,
        outer_radius: Scalar,
        root_radius: Scalar,
        tooth_count: usize,
        thickness: Scalar,
    },

    /// Obelisk: 4-sided tapered prism with a square bottom and a smaller
    /// square top. Centered on the origin, axis along +z. Special case of
    /// TruncatedPyramid with 4 sides.
    Obelisk {
        id: String,
        bottom_side: Scalar,
        top_side: Scalar,
        height: Scalar,
    },

    /// AxleShaft: a long cylinder with two reduced-diameter ends (the
    /// "necks" that fit into bearings). Useful for shafts. The middle
    /// section has `body_radius` and length `body_length`; each neck has
    /// `neck_radius` and length `neck_length`.
    AxleShaft {
        id: String,
        body_radius: Scalar,
        body_length: Scalar,
        neck_radius: Scalar,
        neck_length: Scalar,
        segments: usize,
    },

    /// Column: classical column with a wider base, narrower shaft, and
    /// wider capital. base/capital are short cylinders; shaft is the
    /// main cylinder. `base_radius` >= `shaft_radius` <= `capital_radius`.
    Column {
        id: String,
        base_radius: Scalar,
        base_height: Scalar,
        shaft_radius: Scalar,
        shaft_height: Scalar,
        capital_radius: Scalar,
        capital_height: Scalar,
        segments: usize,
    },

    /// Diamond: regular octahedron — 6 vertices, 8 triangular faces.
    /// Built as two pyramids joined base-to-base. `radius` is the
    /// distance from center to each of the 6 vertices.
    Diamond {
        id: String,
        radius: Scalar,
    },

    /// TriPrism: a triangular prism. `a`, `b`, `c` are the side lengths
    /// of the triangular profile (must satisfy triangle inequality);
    /// triangle is positioned with side a along +x. `length` is the
    /// extrusion along +z.
    TriPrism {
        id: String,
        a: Scalar,
        b: Scalar,
        c: Scalar,
        length: Scalar,
    },

    /// PerforatedPlate: rectangular plate with a regular grid of
    /// circular through-holes. `nx` × `ny` grid; first hole at
    /// `(margin, margin)`; spacing `dx` along x, `dy` along y.
    PerforatedPlate {
        id: String,
        plate_width: Scalar,
        plate_height: Scalar,
        plate_thickness: Scalar,
        margin: Scalar,
        hole_radius: Scalar,
        nx: usize,
        ny: usize,
        dx: Scalar,
        dy: Scalar,
        segments: usize,
    },

    /// ChamferedPlate: rectangular plate where all 4 corners are
    /// chamfered (cut at 45°) instead of rounded. `chamfer` is the
    /// distance from each corner the chamfer cuts.
    ChamferedPlate {
        id: String,
        width: Scalar,
        height: Scalar,
        thickness: Scalar,
        chamfer: Scalar,
    },

    /// ReducerCone: a hollow frustum (a tapered tube). Concentric outer
    /// and inner cones — outer goes from `outer_bottom` radius to
    /// `outer_top` over `height`; inner same shape but smaller. Used for
    /// pipe reducers.
    ReducerCone {
        id: String,
        outer_bottom: Scalar,
        outer_top: Scalar,
        inner_bottom: Scalar,
        inner_top: Scalar,
        height: Scalar,
        segments: usize,
    },

    /// Elbow90: two perpendicular cylinders joined at right angle (an
    /// L-shaped pipe section). Each leg has the same radius. The first
    /// leg runs along +x for `leg_length`, the second from the corner
    /// along +y for the same length.
    Elbow90 {
        id: String,
        radius: Scalar,
        leg_length: Scalar,
        segments: usize,
    },

    /// Reference: a thin rod between two world-space points, drawn as
    /// a small-radius cylinder. Useful for visualising distances and
    /// constraint relationships in the rendered model.
    DistanceRod {
        id: String,
        from: [Scalar; 3],
        to: [Scalar; 3],
        radius: Scalar,
        segments: usize,
    },

    /// Reference: angular arc — a thin curved rod sweeping through a
    /// given angle in the xy plane. Used for visualising angle
    /// relationships. Center at (cx, cy, z), radius `r`, from angle
    /// `start_deg` to `start_deg + sweep_deg`.
    AngleArc {
        id: String,
        center: [Scalar; 3],
        radius: Scalar,
        start_deg: Scalar,
        sweep_deg: Scalar,
        rod_radius: Scalar,
        segments: usize,
    },

    /// SheetBend: a thin sheet-metal shape that bends 90° at a given
    /// distance from the start. First section runs `length_a` along +x
    /// at thickness `thickness`, then bends up 90° and continues
    /// `length_b` along +z. `width` is the depth along +y.
    SheetBend {
        id: String,
        length_a: Scalar,
        length_b: Scalar,
        width: Scalar,
        thickness: Scalar,
    },

    /// TrussMember: a thin cylindrical rod with rectangular end plates
    /// — used to model truss bars or struts. The rod runs along +z
    /// from end plate to end plate.
    TrussMember {
        id: String,
        rod_radius: Scalar,
        rod_length: Scalar,
        plate_width: Scalar,
        plate_thickness: Scalar,
        segments: usize,
    },

    /// Hinge: two flat leaves joined by a cylindrical knuckle pin. Each
    /// leaf is `leaf_width` x `leaf_height` x `leaf_thickness`. The
    /// knuckle pin runs along +y between the two leaves with radius
    /// `pin_radius`. Pin and leaves are unioned into a single solid.
    Hinge {
        id: String,
        leaf_width: Scalar,
        leaf_height: Scalar,
        leaf_thickness: Scalar,
        pin_radius: Scalar,
        segments: usize,
    },

    /// Cleat: T-shape wall mount — a horizontal arm and a vertical
    /// support, both rectangular. `arm_length` along +x, `arm_height`
    /// along +z, `support_width` along +x, `support_height` along +z,
    /// `thickness` along +y. The two are joined L-style at the origin.
    Cleat {
        id: String,
        arm_length: Scalar,
        arm_height: Scalar,
        support_width: Scalar,
        support_height: Scalar,
        thickness: Scalar,
    },

    /// Lattice: a 2D grid of crossing rectangular bars. Like a
    /// gridiron. `nx` × `ny` cells, each `cell_size` square; bars are
    /// `bar_thickness` wide.
    Lattice {
        id: String,
        nx: usize,
        ny: usize,
        cell_size: Scalar,
        bar_thickness: Scalar,
        depth: Scalar,
    },

    /// SocketHeadCapScrew: a hex socket head + cylindrical shaft. Head
    /// is a hex prism (6-sided, fits a hex key); shaft is a cylinder.
    SocketHeadCapScrew {
        id: String,
        head_radius: Scalar,
        head_height: Scalar,
        shaft_radius: Scalar,
        shaft_length: Scalar,
        segments: usize,
    },

    /// FlatHeadScrew: a countersunk flat-head screw — frustum head
    /// (cone-shaped) tapering from `head_radius` at the top to
    /// `shaft_radius` at the join, then cylindrical shaft below.
    FlatHeadScrew {
        id: String,
        head_radius: Scalar,
        head_height: Scalar,
        shaft_radius: Scalar,
        shaft_length: Scalar,
        segments: usize,
    },

    /// Rivet: cylindrical body + dome head. The dome is approximated
    /// by a frustum (small radius at top, larger at the body join) to
    /// avoid sphere boolean limitations.
    Rivet {
        id: String,
        body_radius: Scalar,
        body_length: Scalar,
        head_radius: Scalar,
        head_height: Scalar,
        segments: usize,
    },

    /// ShoulderBolt: a bolt with two diameters — a wider "shoulder"
    /// near the head and a narrower threaded portion below. Useful for
    /// precise positioning bolts.
    ShoulderBolt {
        id: String,
        head_radius: Scalar,
        head_height: Scalar,
        shoulder_radius: Scalar,
        shoulder_length: Scalar,
        thread_radius: Scalar,
        thread_length: Scalar,
        segments: usize,
    },

    /// EyeBolt: a bolt with a ring (eye) instead of a flat head. The
    /// ring is approximated as a faceted donut (torus_faceted) joined
    /// to a cylindrical shaft.
    EyeBolt {
        id: String,
        ring_major_radius: Scalar,
        ring_minor_radius: Scalar,
        shaft_radius: Scalar,
        shaft_length: Scalar,
        ring_segs: usize,
        shaft_segments: usize,
    },

    /// ThreadInsert: a tubular insert with a hex outer (so it can be
    /// driven into the workpiece) and a smooth inner bore. Used for
    /// strengthening threads in soft materials.
    ThreadInsert {
        id: String,
        outer_radius: Scalar,
        inner_radius: Scalar,
        length: Scalar,
        segments: usize,
    },

    /// Cam: a circular disk with an offset hole — when rotated about
    /// the hole, the outer profile traces an eccentric path. `radius`
    /// is the disk outer radius, `hole_radius` the central hole, and
    /// `eccentricity` the offset of the hole from the disk's geometric
    /// center.
    Cam {
        id: String,
        radius: Scalar,
        hole_radius: Scalar,
        eccentricity: Scalar,
        thickness: Scalar,
        segments: usize,
    },

    /// Crank: a horizontal arm with two cylinders at each end (the
    /// pivot and the wrist pin). `arm_length` is between the two pin
    /// centers; `arm_thickness` and `arm_width` define the connecting
    /// arm cross-section.
    Crank {
        id: String,
        pivot_radius: Scalar,
        wrist_radius: Scalar,
        arm_length: Scalar,
        arm_width: Scalar,
        arm_thickness: Scalar,
        segments: usize,
    },

    /// Tee: a 3-way pipe junction. Three cylinders meeting at the
    /// origin, all the same radius. Two run along the same axis (e.g.
    /// +x and -x), the third runs perpendicular (+y).
    Tee {
        id: String,
        radius: Scalar,
        leg_length: Scalar,
        segments: usize,
    },

    /// Cross: a 4-way pipe junction. Four cylinders meeting at the
    /// origin, all the same radius. Two pairs of antiparallel legs.
    Cross {
        id: String,
        radius: Scalar,
        leg_length: Scalar,
        segments: usize,
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
            | Feature::Donut { id, .. }
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
            | Feature::SweepPath { id, .. }
            | Feature::Coil { id, .. }
            | Feature::RefPoint { id, .. }
            | Feature::RefAxis { id, .. }
            | Feature::RefPlane { id, .. }
            | Feature::Arrow { id, .. }
            | Feature::Funnel { id, .. }
            | Feature::CylinderAt { id, .. }
            | Feature::TubeAt { id, .. }
            | Feature::Star { id, .. }
            | Feature::LBracket { id, .. }
            | Feature::UChannel { id, .. }
            | Feature::TBeam { id, .. }
            | Feature::IBeam { id, .. }
            | Feature::CChannel { id, .. }
            | Feature::ZBeam { id, .. }
            | Feature::AngleIron { id, .. }
            | Feature::TSlot { id, .. }
            | Feature::Keyway { id, .. }
            | Feature::RoundedRect { id, .. }
            | Feature::Hemisphere { id, .. }
            | Feature::SphericalCap { id, .. }
            | Feature::Bowl { id, .. }
            | Feature::BoundingBoxRef { id, .. }
            | Feature::CentroidPoint { id, .. }
            | Feature::MountingFlange { id, .. }
            | Feature::GearBlank { id, .. }
            | Feature::KnurledGrip { id, .. }
            | Feature::Pipe { id, .. }
            | Feature::Spring { id, .. }
            | Feature::Mortise { id, .. }
            | Feature::Tenon { id, .. }
            | Feature::FingerJoint { id, .. }
            | Feature::DovetailRail { id, .. }
            | Feature::Pulley { id, .. }
            | Feature::Bushing { id, .. }
            | Feature::Sprocket { id, .. }
            | Feature::Obelisk { id, .. }
            | Feature::AxleShaft { id, .. }
            | Feature::Column { id, .. }
            | Feature::Diamond { id, .. }
            | Feature::TriPrism { id, .. }
            | Feature::PerforatedPlate { id, .. }
            | Feature::ChamferedPlate { id, .. }
            | Feature::ReducerCone { id, .. }
            | Feature::Elbow90 { id, .. }
            | Feature::DistanceRod { id, .. }
            | Feature::AngleArc { id, .. }
            | Feature::SheetBend { id, .. }
            | Feature::TrussMember { id, .. }
            | Feature::Hinge { id, .. }
            | Feature::Cleat { id, .. }
            | Feature::Lattice { id, .. }
            | Feature::SocketHeadCapScrew { id, .. }
            | Feature::FlatHeadScrew { id, .. }
            | Feature::Rivet { id, .. }
            | Feature::ShoulderBolt { id, .. }
            | Feature::EyeBolt { id, .. }
            | Feature::ThreadInsert { id, .. }
            | Feature::Cam { id, .. }
            | Feature::Crank { id, .. }
            | Feature::Tee { id, .. }
            | Feature::Cross { id, .. }
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
            | Feature::Donut { .. }
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
            | Feature::SweepPath { .. }
            | Feature::Coil { .. }
            | Feature::RefPoint { .. }
            | Feature::RefAxis { .. }
            | Feature::RefPlane { .. }
            | Feature::Arrow { .. }
            | Feature::Funnel { .. }
            | Feature::CylinderAt { .. }
            | Feature::TubeAt { .. }
            | Feature::Star { .. }
            | Feature::LBracket { .. }
            | Feature::UChannel { .. }
            | Feature::TBeam { .. }
            | Feature::IBeam { .. }
            | Feature::CChannel { .. }
            | Feature::ZBeam { .. }
            | Feature::AngleIron { .. }
            | Feature::TSlot { .. }
            | Feature::Keyway { .. }
            | Feature::RoundedRect { .. }
            | Feature::Hemisphere { .. }
            | Feature::SphericalCap { .. }
            | Feature::Bowl { .. }
            | Feature::MountingFlange { .. }
            | Feature::GearBlank { .. }
            | Feature::KnurledGrip { .. }
            | Feature::Pipe { .. }
            | Feature::Spring { .. }
            | Feature::Mortise { .. }
            | Feature::Tenon { .. }
            | Feature::FingerJoint { .. }
            | Feature::DovetailRail { .. }
            | Feature::Pulley { .. }
            | Feature::Bushing { .. }
            | Feature::Sprocket { .. }
            | Feature::Obelisk { .. }
            | Feature::AxleShaft { .. }
            | Feature::Column { .. }
            | Feature::Diamond { .. }
            | Feature::TriPrism { .. }
            | Feature::PerforatedPlate { .. }
            | Feature::ChamferedPlate { .. }
            | Feature::ReducerCone { .. }
            | Feature::Elbow90 { .. }
            | Feature::DistanceRod { .. }
            | Feature::AngleArc { .. }
            | Feature::SheetBend { .. }
            | Feature::TrussMember { .. }
            | Feature::Hinge { .. }
            | Feature::Cleat { .. }
            | Feature::Lattice { .. }
            | Feature::SocketHeadCapScrew { .. }
            | Feature::FlatHeadScrew { .. }
            | Feature::Rivet { .. }
            | Feature::ShoulderBolt { .. }
            | Feature::EyeBolt { .. }
            | Feature::ThreadInsert { .. }
            | Feature::Cam { .. }
            | Feature::Crank { .. }
            | Feature::Tee { .. }
            | Feature::Cross { .. }
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
            | Feature::SquareHole { input, .. }
            | Feature::BoundingBoxRef { input, .. }
            | Feature::CentroidPoint { input, .. } => vec![input.as_str()],
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
