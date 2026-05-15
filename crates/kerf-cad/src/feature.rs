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
    /// Donut2: refined torus shape that preserves the exact analytic ring
    /// geometry (vertex positions satisfy the torus equation exactly).
    /// Unlike `Donut`, which uses best-fit planar quads, `Donut2` tags faces
    /// with `SurfaceKind::Torus` so the analytic boolean path can exploit the
    /// exact surface equation when available. Centered on the origin, axis
    /// along +z. `major_radius` must exceed `minor_radius`.
    Donut2 {
        id: String,
        major_radius: Scalar,
        minor_radius: Scalar,
        major_segs: usize,
        minor_segs: usize,
    },
    /// ToroidalCap: a wedge of a torus — like `Donut` sliced by two half-
    /// spaces to keep only `sweep_degrees` (0 < sweep ≤ 360) of the full
    /// toroidal sweep. At 360° it is identical to `Donut`. At 180° it is a
    /// half-torus (D-shaped cross-section in the xy plane). Centered on the
    /// origin with the sweep starting at θ = 0 and going counter-clockwise.
    ToroidalCap {
        id: String,
        major_radius: Scalar,
        minor_radius: Scalar,
        sweep_degrees: Scalar,
        major_segs: usize,
        minor_segs: usize,
    },
    /// EllipticTube: a solid tube whose cross-section is an ellipse with
    /// semi-axes `semi_major` (along x by default) and `semi_minor` (along
    /// y). The tube extends `length` units along the named `axis` ("x", "y",
    /// or "z"). Built by scaling a faceted cylinder non-uniformly.
    EllipticTube {
        id: String,
        semi_major: Scalar,
        semi_minor: Scalar,
        length: Scalar,
        axis: String,
        segments: usize,
    },
    /// Goblet2: chalice/wine-glass shape (revolved profile: flat disk foot +
    /// narrow cylindrical stem + flared cup). Distinct from `Goblet` (which
    /// has a frustum bowl) — `Goblet2` uses a cylindrical cup rim so the
    /// silhouette is a classic straight-sided chalice.
    Goblet2 {
        id: String,
        foot_radius: Scalar,
        stem_radius: Scalar,
        stem_height: Scalar,
        cup_radius: Scalar,
        cup_height: Scalar,
        segments: usize,
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

    /// Extrude a `Sketch` along a direction vector. The sketch is traced
    /// into a single closed profile via `Sketch::to_profile_2d` and routed
    /// through the same `extrude_polygon` kernel as `ExtrudePolygon`.
    ///
    /// If the sketch carries any `SketchConstraint`s, the constraint solver
    /// runs *before* tracing — the Point coordinates emitted by the trace
    /// reflect the solved configuration. Set `skip_solve = true` to bypass
    /// the solver and trace the raw authored coordinates verbatim (useful
    /// for testing the trace pipeline in isolation, or for "loose" sketches
    /// that are deliberately authored under-constrained).
    ///
    /// `skip_solve` defaults to `false` in JSON deserialization (see
    /// `#[serde(default)]`).
    SketchExtrude {
        id: String,
        sketch: crate::sketch::Sketch,
        direction: [Scalar; 3],
        /// Bypass `Sketch::solve` even when constraints are present.
        #[serde(default)]
        skip_solve: bool,
    },

    /// Revolve a `Sketch` around the z-axis. The sketch's primitives are
    /// traced and the resulting profile is treated as an open polyline in
    /// the xz-plane (same constraints as `Revolve`: first + last on the
    /// z-axis, interior x > 0). The sketch must produce exactly one
    /// profile.
    SketchRevolve {
        id: String,
        sketch: crate::sketch::Sketch,
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

    /// **EndChamfer**: bevel the rim of an axis-aligned cylindrical hole
    /// or boss. Cuts a conical ring chamfer where the cylindrical face
    /// meets the +axis end face. Differs from `Chamfer` (which bevels a
    /// straight edge of a box-like body).
    ///
    /// The cutter is a frustum: outer radius `outer_radius` at the top
    /// surface (z = `top_center[axis]`), inner radius `outer_radius -
    /// chamfer` at depth `chamfer` below the top. Subtracted from `input`,
    /// it removes a conical ring around the cylinder rim.
    ///
    /// Common use: deburring a turned shaft end so the ring chamfer
    /// matches the lead-in for a mating part.
    EndChamfer {
        id: String,
        input: String,
        axis: String,
        top_center: [Scalar; 3],
        outer_radius: Scalar,
        chamfer: Scalar,
        segments: usize,
    },

    /// **InternalChamfer**: bevel the inside rim of a cylindrical hole.
    /// Like a Countersink but the chamfered cone is the only cut — there's
    /// no through-drill. Used to add a lead-in to an existing hole or
    /// to break the sharp interior edge of a bored cavity.
    ///
    /// The cone widens from `hole_radius` at depth `chamfer_depth` to
    /// `hole_radius + chamfer_width` at the top surface (the +axis face
    /// of the body, at `top_center`).
    InternalChamfer {
        id: String,
        input: String,
        axis: String,
        top_center: [Scalar; 3],
        hole_radius: Scalar,
        chamfer_width: Scalar,
        chamfer_depth: Scalar,
        segments: usize,
    },

    /// **ConicalCounterbore**: counterbore-style stepped hole whose lower
    /// section ends in a conical (drill-tip-shaped) bottom rather than a
    /// flat bottom. The drill portion runs full `body_depth`, then tapers
    /// to a point over `tip_depth`. The wider counterbore at the top has
    /// `cbore_radius` and `cbore_depth`.
    ///
    /// Approximates a drill bit's natural conical tip — useful for
    /// representing real machined holes in non-through configurations.
    ConicalCounterbore {
        id: String,
        input: String,
        axis: String,
        top_center: [Scalar; 3],
        drill_radius: Scalar,
        cbore_radius: Scalar,
        cbore_depth: Scalar,
        body_depth: Scalar,
        tip_depth: Scalar,
        segments: usize,
    },

    /// **CrossDrilledHole**: two perpendicular through-holes drilled
    /// through `input`, intersecting at `center`. Hole 1 runs along
    /// `axis_a`, hole 2 runs along `axis_b` (must be different "x"|"y"|"z").
    /// Both holes share `radius`. `length_a` and `length_b` are the
    /// drill-cylinder lengths along each axis (must be long enough to
    /// pierce both surfaces).
    ///
    /// Common in clevis pins, alignment dowels with retainer holes,
    /// hydraulic fittings.
    CrossDrilledHole {
        id: String,
        input: String,
        center: [Scalar; 3],
        axis_a: String,
        axis_b: String,
        radius: Scalar,
        length_a: Scalar,
        length_b: Scalar,
        segments: usize,
    },

    /// **TaperedPin**: pin (cylinder) that tapers from `large_radius` at
    /// one end to `small_radius` at the other over `length` along the +z
    /// axis. Like a roll pin or alignment dowel that drives into a
    /// matching tapered hole.
    ///
    /// `large_radius` is at z=0, `small_radius` at z=`length`.
    TaperedPin {
        id: String,
        large_radius: Scalar,
        small_radius: Scalar,
        length: Scalar,
        segments: usize,
    },

    /// **FlangedNut**: hexagonal nut with an integrated wide circular
    /// flange at the bottom. Combines a hex prism (the wrenching
    /// surface) on top with a thinner, larger-radius cylindrical flange
    /// underneath, with a through-bore along the z axis.
    ///
    /// `inscribed_radius` is the hex inscribed radius (across-flats /2).
    /// `flange_radius` is the flange disk radius (must exceed the hex
    /// circumradius). `bore_radius` runs through both pieces.
    FlangedNut {
        id: String,
        inscribed_radius: Scalar,
        flange_radius: Scalar,
        flange_thickness: Scalar,
        nut_thickness: Scalar,
        bore_radius: Scalar,
        segments: usize,
    },

    /// **DowelPin**: cylindrical alignment dowel with chamfered ends on
    /// both sides. The chamfers (sloped lead-ins) help drive the dowel
    /// into a press-fit hole without scuffing.
    ///
    /// `radius` is the cylinder radius. `length` is the total length
    /// along +z. `chamfer` is the axial setback at each end (a 45°
    /// chamfer reduces the radius linearly from `radius` to `radius -
    /// chamfer` over the chamfer length).
    DowelPin {
        id: String,
        radius: Scalar,
        length: Scalar,
        chamfer: Scalar,
        segments: usize,
    },

    /// **BlindHole**: closed-bottom drilled hole in `input`. Unlike a
    /// through-hole, the hole stops at `depth` from the +axis-facing
    /// surface — leaves a closed floor. Typically used when fastener
    /// pull-through is undesired (e.g. tapped holes, set-screw seats).
    ///
    /// Drill goes from `top_center` along -axis for `depth`. `axis` is
    /// "x"|"y"|"z".
    BlindHole {
        id: String,
        input: String,
        axis: String,
        top_center: [Scalar; 3],
        radius: Scalar,
        depth: Scalar,
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

    /// TwistedExtrude: extrude a polygon while rotating linearly about
    /// the +z axis. The profile sits at z=0 with no rotation; at z=`height`
    /// it is rotated by `twist_deg` degrees about the centroid of its
    /// xy points. Built as a single `extrude_lofted` between the original
    /// profile and the rotated profile. Side faces are quad-shaped but
    /// non-planar in general (each side wraps a helical strip) — that's
    /// fine for STL/STEP export and volume calculations, but booleans
    /// against the result are degraded and may trip stitch.
    TwistedExtrude {
        id: String,
        profile: Profile2D,
        height: Scalar,
        twist_deg: Scalar,
    },

    /// HelicalRib: a rectangular cross-section sweep along a helical path.
    /// Like a coarse decorative ridge or screw root. `coil_radius` is the
    /// helix radius, `rib_size` is the square cross-section side length
    /// (sweep_cylinder_segment with segments=4 gives a square cross-section
    /// in the cylinder's local frame). Centered on +z axis like Coil.
    HelicalRib {
        id: String,
        coil_radius: Scalar,
        rib_size: Scalar,
        pitch: Scalar,
        turns: Scalar,
        segments_per_turn: usize,
    },

    /// ScrewThread: a triangular V-thread cross-section sweep along a
    /// helical path. Approximates a standard machine thread. The thread
    /// "wire" is a triangular prism (segments=3) with circumscribed radius
    /// `thread_height`. `coil_radius` is the major radius of the thread.
    ScrewThread {
        id: String,
        coil_radius: Scalar,
        thread_height: Scalar,
        pitch: Scalar,
        turns: Scalar,
        segments_per_turn: usize,
    },

    /// SpiralWedge: a helical sweep where the cross-section radius grows
    /// linearly from `wire_radius_start` at the bottom to `wire_radius_end`
    /// at the top. Useful for tapered springs or decorative spirals where
    /// the wire thickens as it climbs.
    SpiralWedge {
        id: String,
        coil_radius: Scalar,
        wire_radius_start: Scalar,
        wire_radius_end: Scalar,
        pitch: Scalar,
        turns: Scalar,
        segments_per_turn: usize,
        wire_segments: usize,
    },

    /// DoubleHelix: two intertwined helices, offset by 180° in starting
    /// angle. Each helix has the same pitch, turns, and wire radius.
    /// Resembles DNA / decorative twist columns. The union may trip stitch
    /// where the two helices come close in z; the evaluator returns the
    /// boolean error in that case.
    DoubleHelix {
        id: String,
        coil_radius: Scalar,
        wire_radius: Scalar,
        pitch: Scalar,
        turns: Scalar,
        segments_per_turn: usize,
        wire_segments: usize,
    },

    /// TaperedCoil: helical sweep where the helix radius decreases
    /// linearly with z, forming a conical spring. `coil_radius_start` is
    /// the radius at z=0; `coil_radius_end` is at z=pitch*turns.
    /// `wire_radius` is constant.
    TaperedCoil {
        id: String,
        coil_radius_start: Scalar,
        coil_radius_end: Scalar,
        wire_radius: Scalar,
        pitch: Scalar,
        turns: Scalar,
        segments_per_turn: usize,
        wire_segments: usize,
    },

    /// True sweep of an arbitrary 2D `profile` along an arbitrary 3D
    /// polyline `path`. Each pair of consecutive path points (p0, p1)
    /// defines a segment; the profile is placed in a frame perpendicular
    /// to the segment direction at each end and lofted between the two
    /// frames via `extrude_lofted`. Successive segments are unioned.
    ///
    /// Frames are computed without parallel transport — each segment uses
    /// a local frame from its tangent only, so consecutive segments may
    /// have a discrete rotation about the tangent at the join. For smooth
    /// curves at small step sizes this is visually negligible.
    ///
    /// The first path point is the profile origin; the profile is the XY
    /// plane in the local frame, with +z being the segment tangent.
    /// `slices` is reserved for future per-segment subdivision (currently
    /// each polyline edge produces exactly one loft segment); pass `1`.
    SweepProfile {
        id: String,
        profile: Profile2D,
        path: Vec<[Scalar; 3]>,
        slices: usize,
    },

    /// Loft through N profiles at N positions, in order. Each consecutive
    /// pair (profiles[i], profiles[i+1]) at (positions[i], positions[i+1])
    /// becomes a single `extrude_lofted` segment; all segments are unioned.
    ///
    /// All profiles must have the same vertex count ≥ 3. The profile XY
    /// is placed at `positions[i]` with no rotation — the profile plane
    /// is always parallel to the world XY plane. For smooth-curve lofts
    /// where the profile follows the path tangent, see `SweepProfile`.
    LoftMulti {
        id: String,
        profiles: Vec<Profile2D>,
        positions: Vec<[Scalar; 3]>,
        slices: usize,
    },

    /// Sweep `profile` along `path` while progressively rotating the
    /// profile about the path tangent by a total of `twist_angle` degrees.
    /// At path point i (of N), the profile is rotated by
    /// `twist_angle * i / (N-1)` degrees around the centroid in the local
    /// XY plane before being lifted into the segment frame. Joins between
    /// segments stay continuous because both endpoints of a segment share
    /// the same rotation as their neighbors' shared endpoint.
    SweepWithTwist {
        id: String,
        profile: Profile2D,
        path: Vec<[Scalar; 3]>,
        twist_angle: Scalar,
    },

    /// Sweep `profile` along `path` while linearly interpolating the
    /// profile scale from `start_scale` (at the first path point) to
    /// `end_scale` (at the last). Scale is applied around the profile
    /// centroid in the local XY plane. Both scales must be > 0.
    SweepWithScale {
        id: String,
        profile: Profile2D,
        path: Vec<[Scalar; 3]>,
        start_scale: Scalar,
        end_scale: Scalar,
    },

    /// Proper screw-thread approximation: sweep a triangular V-thread
    /// `profile` (built internally from `thread_height`) along a helical
    /// path of `coil_radius`, `pitch`, `turns`. Unlike `ScrewThread` —
    /// which chains short triangular cylinders along the helix — this
    /// uses a true profile sweep, so the thread cross-section orientation
    /// rotates with the helix tangent. The thread profile is an
    /// equilateral triangle with apex pointing radially outward from the
    /// helix axis.
    HelicalThread {
        id: String,
        coil_radius: Scalar,
        thread_height: Scalar,
        pitch: Scalar,
        turns: Scalar,
        segments_per_turn: usize,
    },

    /// Hollow cylinder of `outer_radius`/`inner_radius` and total `height`
    /// with progressive twist of `twist_turns` complete revolutions from
    /// bottom to top. Built as a difference of two `TwistedExtrude`-style
    /// solids: a regular `slices`-gon outer profile and a smaller inner
    /// profile, each twisted by `twist_turns * 360°` about its centroid.
    /// Inner is extended slightly past the caps for a clean through-bore.
    /// `slices` ≥ 3 is the polygonal facet count of both profiles.
    TwistedTube {
        id: String,
        outer_radius: Scalar,
        inner_radius: Scalar,
        height: Scalar,
        twist_turns: Scalar,
        slices: usize,
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

    /// SteppedShaft: cylinder with monotonically decreasing diameter
    /// steps. `radii` is a list of radii (descending recommended) and
    /// `step_lengths` is the length of each step. Both must be the
    /// same length.
    SteppedShaft {
        id: String,
        radii: Vec<Scalar>,
        step_lengths: Vec<Scalar>,
        segments: usize,
    },

    /// Trough: long U-shaped channel — same cross-section as UChannel
    /// but with closed ends (front and back walls). Useful for tray /
    /// drainage channel modelling.
    Trough {
        id: String,
        outer_width: Scalar,
        outer_height: Scalar,
        wall_thickness: Scalar,
        length: Scalar,
    },

    /// Knob: a cylinder with axial slots ("flutes") around its
    /// perimeter — like a knurling but with deeper grooves.
    Knob {
        id: String,
        radius: Scalar,
        height: Scalar,
        flute_count: usize,
        flute_depth: Scalar,
    },

    /// Plug: a tapered cylinder — same as Frustum but expressed in
    /// "plug" terms (top/bottom radius, length). Useful for sealing.
    Plug {
        id: String,
        top_radius: Scalar,
        bottom_radius: Scalar,
        length: Scalar,
        segments: usize,
    },

    /// Spike: a cone (faceted) — apex at +z, base at z=0. Same as Cone
    /// but expressed as Spike for catalogue clarity.
    Spike {
        id: String,
        base_radius: Scalar,
        height: Scalar,
        segments: usize,
    },

    /// CapBolt: a wider-headed version of Bolt with a concave cap.
    /// Shape: a cylinder for the head + a cylinder for the shaft.
    /// Ratio: head_radius is significantly larger than shaft_radius.
    CapBolt {
        id: String,
        head_radius: Scalar,
        head_height: Scalar,
        shaft_radius: Scalar,
        shaft_length: Scalar,
        segments: usize,
    },

    /// FlangedBolt: a bolt with an integrated wider washer-like
    /// flange under the head. Three cylinders stacked: head, flange,
    /// shaft.
    FlangedBolt {
        id: String,
        head_radius: Scalar,
        head_height: Scalar,
        flange_radius: Scalar,
        flange_thickness: Scalar,
        shaft_radius: Scalar,
        shaft_length: Scalar,
        segments: usize,
    },

    /// SerratedDisk: a thin disk with very fine teeth — like a
    /// crown / star washer. `tooth_count` should be high (24+).
    SerratedDisk {
        id: String,
        outer_radius: Scalar,
        root_radius: Scalar,
        tooth_count: usize,
        thickness: Scalar,
    },

    /// FerruleEnd: a frustum-shaped tube end fitting (one end larger
    /// than the other, like a flared pipe end).
    FerruleEnd {
        id: String,
        small_radius: Scalar,
        large_radius: Scalar,
        wall_thickness: Scalar,
        length: Scalar,
        segments: usize,
    },

    /// Trapezoid: an isosceles trapezoidal prism extruded along +z.
    /// Useful for keyway profiles and tapered joints.
    Trapezoid {
        id: String,
        bottom_width: Scalar,
        top_width: Scalar,
        height: Scalar,
        depth: Scalar,
    },

    /// Handle: rectangular grip — a horizontal bar with rounded ends
    /// (cylinder caps). `bar_length` is the straight portion; total
    /// length = bar_length + 2 * cap_radius.
    Handle {
        id: String,
        bar_length: Scalar,
        bar_radius: Scalar,
        cap_radius: Scalar,
        segments: usize,
    },

    /// HookHandle: a hook — straight shaft + 180° curved tip.
    /// `shaft_length` is the straight portion along +z; `bend_radius`
    /// is the radius of the U-curve at the top.
    HookHandle {
        id: String,
        shaft_length: Scalar,
        shaft_radius: Scalar,
        bend_radius: Scalar,
        segments: usize,
    },

    /// CornerBracket: a 3-leg corner support — three perpendicular
    /// rectangular plates meeting at a corner. Useful for box
    /// reinforcement.
    CornerBracket {
        id: String,
        leg_x: Scalar,
        leg_y: Scalar,
        leg_z: Scalar,
        thickness: Scalar,
    },

    /// ArcSegment: a partial torus arc — like a Donut sliced by an
    /// angular range. Approximated using AngleArc (chained cylinders).
    ArcSegment {
        id: String,
        major_radius: Scalar,
        minor_radius: Scalar,
        start_deg: Scalar,
        sweep_deg: Scalar,
        segments: usize,
    },

    /// CrossBrace: an X-shaped support — two diagonal rectangular
    /// bars crossed in a square frame. Useful for truss panels.
    CrossBrace {
        id: String,
        frame_size: Scalar,
        bar_thickness: Scalar,
        depth: Scalar,
    },

    /// WireMesh: a thin grid of crossing rods (cylinders) instead of
    /// rectangular bars. Like Lattice but with cylindrical bars.
    WireMesh {
        id: String,
        nx: usize,
        ny: usize,
        cell_size: Scalar,
        wire_radius: Scalar,
        segments: usize,
    },

    /// AnchorPoint: a clip-style anchor — a flat plate with a
    /// projecting tab and a hole through the tab.
    AnchorPoint {
        id: String,
        plate_width: Scalar,
        plate_height: Scalar,
        plate_thickness: Scalar,
        tab_height: Scalar,
        hole_radius: Scalar,
        segments: usize,
    },

    /// Stair: a parametric staircase — `step_count` steps, each `tread`
    /// deep (along +y) and `riser` tall (along +z). Total run = count*tread,
    /// total rise = count*riser. Width = `step_width` along +x.
    Stair {
        id: String,
        step_count: usize,
        tread: Scalar,
        riser: Scalar,
        step_width: Scalar,
    },

    /// Hopper: a cone funnel atop a cylindrical neck. Used for
    /// gravity-feed bins, pour spouts. `top_radius` is the wide top of
    /// the funnel, `neck_radius` the narrow bottom that joins the
    /// vertical neck. `funnel_height` is the funnel cone height,
    /// `neck_height` the neck cylinder height.
    Hopper {
        id: String,
        top_radius: Scalar,
        neck_radius: Scalar,
        funnel_height: Scalar,
        neck_height: Scalar,
        segments: usize,
    },

    /// Stand: a cylindrical platform — a thick disk of
    /// (radius, thickness). Useful for workpiece supports.
    Stand {
        id: String,
        radius: Scalar,
        thickness: Scalar,
        segments: usize,
    },

    /// Yoke: a U-shaped fork — two parallel arms with a connecting
    /// crossbar, plus an optional pivot hole through both arms. Used
    /// for clevis and shackle joints.
    Yoke {
        id: String,
        arm_length: Scalar,
        arm_thickness: Scalar,
        arm_height: Scalar,
        gap_width: Scalar,
        pivot_radius: Scalar,
        segments: usize,
    },

    /// Lever: a long rectangular bar with a pivot hole at one end.
    /// `length` is the full bar length, `width` and `thickness` the
    /// bar cross-section, `pivot_radius` the hole radius near the end.
    Lever {
        id: String,
        length: Scalar,
        width: Scalar,
        thickness: Scalar,
        pivot_radius: Scalar,
        segments: usize,
    },

    /// TaperedTube: hollow frustum — same as ReducerCone but with a
    /// constant wall thickness instead of named inner/outer radii at
    /// each end. Easier to specify when you just know "tapered tube
    /// with X-thick walls".
    TaperedTube {
        id: String,
        bottom_outer_radius: Scalar,
        top_outer_radius: Scalar,
        wall_thickness: Scalar,
        height: Scalar,
        segments: usize,
    },

    /// GussetPlate: a right-triangle reinforcement plate — used at
    /// corners where two structural members meet. `leg_length` is each
    /// leg of the right triangle, `thickness` the plate thickness.
    /// Triangle has its right angle at the origin, legs along +x and +y.
    GussetPlate {
        id: String,
        leg_length: Scalar,
        thickness: Scalar,
    },

    /// CrossKey: a + shaped key for keyed shaft-hub connections.
    /// Two perpendicular rectangular bars meeting at the center.
    CrossKey {
        id: String,
        bar_length: Scalar,
        bar_thickness: Scalar,
        bar_height: Scalar,
    },

    /// PinShaft: a long, narrow cylindrical pin — like AxleShaft but
    /// with one diameter throughout. Useful for hinge pins, dowels.
    PinShaft {
        id: String,
        radius: Scalar,
        length: Scalar,
        segments: usize,
    },

    /// Lens: a biconvex lens shape — two spherical caps joined back to
    /// back at z=0. `radius` is the sphere's radius, `cap_height` is
    /// each cap's height, so total lens thickness = 2 * cap_height.
    Lens {
        id: String,
        radius: Scalar,
        cap_height: Scalar,
        stacks: usize,
        slices: usize,
    },

    /// EggShape: a stretched faceted sphere — `radius` is the equatorial
    /// radius, `aspect_z` scales along z (>1 prolate, <1 oblate).
    EggShape {
        id: String,
        radius: Scalar,
        aspect_z: Scalar,
        stacks: usize,
        slices: usize,
    },

    /// UBendPipe: a 180° toroidal pipe section — like a U-bend in
    /// plumbing. `bend_radius` is the centerline radius of the bend,
    /// `pipe_radius` the pipe outer radius. Lies in the xz plane,
    /// from (0, 0, 0) curving up and around to (2*bend_radius, 0, 0).
    UBendPipe {
        id: String,
        bend_radius: Scalar,
        pipe_radius: Scalar,
        segments: usize,
    },

    /// SBend: an S-shaped pipe — two opposite 90° arcs joined.
    /// Useful for offsetting pipes around obstacles.
    SBend {
        id: String,
        bend_radius: Scalar,
        pipe_radius: Scalar,
        segments: usize,
    },

    /// DonutSlice: a wedge of a faceted donut — like an orange slice
    /// of a torus. Defined by sweep angle (0 to 360 degrees).
    DonutSlice {
        id: String,
        major_radius: Scalar,
        minor_radius: Scalar,
        sweep_deg: Scalar,
        major_segs: usize,
        minor_segs: usize,
    },

    /// CapsuleAt: like Capsule but with a chosen axis-aligned
    /// orientation. `axis` is "x" / "y" / "z".
    CapsuleAt {
        id: String,
        axis: String,
        center: [Scalar; 3],
        radius: Scalar,
        body_length: Scalar,
        stacks: usize,
        slices: usize,
    },

    /// ToroidalKnob: a knob with a toroidal grip ridge — cylinder body
    /// with a faceted-torus rib around its top.
    ToroidalKnob {
        id: String,
        body_radius: Scalar,
        body_height: Scalar,
        torus_major_radius: Scalar,
        torus_minor_radius: Scalar,
        body_segs: usize,
        torus_segs: usize,
    },

    /// Cup: an open-top cylindrical container — outer cylinder minus
    /// inner cylindrical cavity. `wall_thickness` defines wall + floor.
    Cup {
        id: String,
        outer_radius: Scalar,
        height: Scalar,
        wall_thickness: Scalar,
        segments: usize,
    },

    /// Bottle: a cylindrical body with a narrow cylindrical neck on
    /// top. `body_radius`/`body_height` define the lower part; the
    /// neck is a smaller cylinder of `neck_radius`/`neck_height` above.
    Bottle {
        id: String,
        body_radius: Scalar,
        body_height: Scalar,
        neck_radius: Scalar,
        neck_height: Scalar,
        segments: usize,
    },

    /// TableLeg: a tapered cylindrical leg — wider at the bottom,
    /// narrower at the top. Same params as Plug.
    TableLeg {
        id: String,
        bottom_radius: Scalar,
        top_radius: Scalar,
        height: Scalar,
        segments: usize,
    },

    /// ChairLeg: a rectangular leg — extruded square cross-section.
    ChairLeg {
        id: String,
        width: Scalar,
        depth: Scalar,
        height: Scalar,
    },

    /// Bookshelf: a multi-shelf rectangular shelving unit. `shelves`
    /// horizontal panels of size (width × depth × shelf_thickness)
    /// stacked at intervals of `clear_height`. Two side panels of
    /// height = shelves * (clear_height + shelf_thickness).
    Bookshelf {
        id: String,
        width: Scalar,
        depth: Scalar,
        shelves: usize,
        shelf_thickness: Scalar,
        clear_height: Scalar,
        side_thickness: Scalar,
    },

    /// PlanterBox: an open-top rectangular planter — like Trough but
    /// with `outer_length` instead of square cross-section.
    PlanterBox {
        id: String,
        outer_width: Scalar,
        outer_length: Scalar,
        outer_height: Scalar,
        wall_thickness: Scalar,
    },

    /// DrawerSlot: a rectangular slot for a sliding drawer — open at
    /// the front, closed on three sides (back, left, right). `width`
    /// is the slot opening, `depth` along +y, `height` along +z, walls
    /// of `wall_thickness`.
    DrawerSlot {
        id: String,
        width: Scalar,
        depth: Scalar,
        height: Scalar,
        wall_thickness: Scalar,
    },

    /// CircularRing: a thin flat ring — annulus extruded along +z. Like
    /// Tube but typically very short (a thin "washer-shaped" ring).
    CircularRing {
        id: String,
        outer_radius: Scalar,
        inner_radius: Scalar,
        thickness: Scalar,
        segments: usize,
    },

    /// PolygonRing: a regular n-sided ring — outer regular polygon
    /// minus inner regular polygon, extruded along +z.
    PolygonRing {
        id: String,
        outer_radius: Scalar,
        inner_radius: Scalar,
        sides: usize,
        thickness: Scalar,
    },

    /// CylinderShellAt: a hollow cylinder positioned at an arbitrary
    /// origin along an arbitrary axis (x/y/z). Same params as Tube but
    /// with named axis like CylinderAt.
    CylinderShellAt {
        id: String,
        base: [Scalar; 3],
        axis: String,
        outer_radius: Scalar,
        inner_radius: Scalar,
        length: Scalar,
        segments: usize,
    },

    /// QuarterTorus: a 90° torus arc. Same as ArcSegment but with
    /// fixed sweep_deg = 90.
    QuarterTorus {
        id: String,
        major_radius: Scalar,
        minor_radius: Scalar,
        segments: usize,
    },

    /// HalfTorus: a 180° torus arc.
    HalfTorus {
        id: String,
        major_radius: Scalar,
        minor_radius: Scalar,
        segments: usize,
    },

    /// SquareTube: rectangular hollow extruded tube. Like UChannel but
    /// closed on top — outer rect minus inner rect cavity, all four
    /// sides closed.
    SquareTube {
        id: String,
        outer_width: Scalar,
        outer_height: Scalar,
        wall_thickness: Scalar,
        length: Scalar,
    },

    /// HoleyPlate: a thick plate with a single circular hole in the
    /// center. Like PerforatedPlate but with one parametric hole.
    HoleyPlate {
        id: String,
        plate_width: Scalar,
        plate_height: Scalar,
        plate_thickness: Scalar,
        hole_radius: Scalar,
        segments: usize,
    },

    /// ScrewBoss: a cylindrical boss with a central pre-drilled hole
    /// for self-tapping screws. Outer cylinder + concentric narrower
    /// hole that doesn't go through (blind hole).
    ScrewBoss {
        id: String,
        outer_radius: Scalar,
        outer_height: Scalar,
        hole_radius: Scalar,
        hole_depth: Scalar,
        segments: usize,
    },

    /// Brick: a standard masonry brick with an optional shallow
    /// cylindrical "frog" depression on top (indent for mortar).
    Brick {
        id: String,
        length: Scalar,
        width: Scalar,
        height: Scalar,
        frog_radius: Scalar,
        frog_depth: Scalar,
        segments: usize,
    },

    /// CorrugatedPanel: a rectangular sheet with a sequence of
    /// triangular ridges along +x. `n_ridges` ridges of
    /// `ridge_height` total ridge height (peak above valley).
    CorrugatedPanel {
        id: String,
        length: Scalar,
        width: Scalar,
        n_ridges: usize,
        ridge_height: Scalar,
        sheet_thickness: Scalar,
    },

    /// BeltLoop: a small thin rectangular loop — like a belt buckle
    /// frame. `outer_width` × `outer_height` rectangle minus inner
    /// rectangle of (`outer_width-2t`) × (`outer_height-2t`),
    /// extruded along +z by `depth`.
    BeltLoop {
        id: String,
        outer_width: Scalar,
        outer_height: Scalar,
        wall_thickness: Scalar,
        depth: Scalar,
    },

    /// Stake: a long shaft tapering to a sharp tip. Cylinder body
    /// (`body_radius`, `body_length`) + cone tip
    /// (`tip_length`). Used for survey stakes, ground anchors.
    Stake {
        id: String,
        body_radius: Scalar,
        body_length: Scalar,
        tip_length: Scalar,
        segments: usize,
    },

    /// Bipyramid: two N-sided pyramids joined base-to-base. `n_sides`
    /// for the equator polygon, `radius` for the equator circumradius,
    /// `top_height` and `bottom_height` for each pyramid's apex
    /// distance from the equator.
    Bipyramid {
        id: String,
        n_sides: usize,
        radius: Scalar,
        top_height: Scalar,
        bottom_height: Scalar,
    },

    /// Antiprism: an N-gon at z=0 and an N-gon at z=`height`, where
    /// the top N-gon is rotated by π/N relative to the bottom (so
    /// vertices alternate). Connecting lateral faces are triangles
    /// (which is what makes it an antiprism rather than a prism).
    Antiprism {
        id: String,
        n_sides: usize,
        radius: Scalar,
        height: Scalar,
    },

    /// CableSaddle: a U-shaped clamp — a rectangular base with a
    /// semicylindrical groove on top to receive a cable.
    CableSaddle {
        id: String,
        base_length: Scalar,
        base_width: Scalar,
        base_height: Scalar,
        cable_radius: Scalar,
        segments: usize,
    },

    /// Slot3D: a stadium-shaped slot — rectangle with semicircular
    /// ends. `length` along +x is the long axis (between center of
    /// each end semicircle); `width` is the short axis (and equals the
    /// diameter of each end). Extruded along +z by `depth`.
    Slot3D {
        id: String,
        length: Scalar,
        width: Scalar,
        depth: Scalar,
        segments: usize,
    },

    /// OvalPlate: a flat plate with an elliptical outline. `a` is the
    /// semi-major axis along +x, `b` semi-minor along +y, extruded
    /// along +z by `thickness`.
    OvalPlate {
        id: String,
        a: Scalar,
        b: Scalar,
        thickness: Scalar,
        segments: usize,
    },

    /// AsymmetricBracket: an L-bracket where the two legs have
    /// different lengths. Like LBracket but `width_a` and `width_b`
    /// can differ.
    AsymmetricBracket {
        id: String,
        leg_a_length: Scalar,
        leg_b_length: Scalar,
        thickness: Scalar,
        depth: Scalar,
    },

    /// EndCap: a cylindrical cap that fits over a tube — outer
    /// cylinder + closed top + inner cavity that fits the tube. Like
    /// Cup but inverted (open at the bottom).
    EndCap {
        id: String,
        outer_radius: Scalar,
        height: Scalar,
        wall_thickness: Scalar,
        cap_thickness: Scalar,
        segments: usize,
    },

    /// RatchetTooth: an asymmetric sawtooth profile — N teeth, each
    /// with a vertical face and a sloped face. Useful for ratchet
    /// wheels and one-way clutches.
    RatchetTooth {
        id: String,
        outer_radius: Scalar,
        root_radius: Scalar,
        tooth_count: usize,
        thickness: Scalar,
    },

    /// BasePlate: a thick rectangular base with optional rounded
    /// corners. Like RoundedRect but with explicit "base" intent.
    /// `corner_radius` of 0 = sharp corners.
    BasePlate {
        id: String,
        width: Scalar,
        height: Scalar,
        thickness: Scalar,
        corner_radius: Scalar,
        segments: usize,
    },

    /// FunnelTube: a hollow funnel — outer cone shell with a constant
    /// `wall_thickness`. Like ReducerCone but easier to specify (top
    /// and bottom outer radii + wall thickness).
    FunnelTube {
        id: String,
        top_outer_radius: Scalar,
        bottom_outer_radius: Scalar,
        wall_thickness: Scalar,
        height: Scalar,
        segments: usize,
    },

    /// FlatWasher: a thin annulus — same as CircularRing with explicit
    /// "washer" intent. Catalog companion to Washer (already in the
    /// fastener bucket).
    FlatWasher {
        id: String,
        outer_radius: Scalar,
        inner_radius: Scalar,
        thickness: Scalar,
        segments: usize,
    },

    /// RibbedPlate: a flat plate with parallel rectangular ribs along
    /// its top surface. `n_ribs` evenly spaced; each rib is
    /// `rib_width` wide and `rib_height` tall.
    RibbedPlate {
        id: String,
        plate_length: Scalar,
        plate_width: Scalar,
        plate_thickness: Scalar,
        n_ribs: usize,
        rib_width: Scalar,
        rib_height: Scalar,
    },

    /// Pediment: an architectural ornament — rectangular base with a
    /// triangular gable top. Extruded along +z by `depth`. Used in
    /// classical architecture above doors and windows.
    Pediment {
        id: String,
        base_width: Scalar,
        base_height: Scalar,
        gable_height: Scalar,
        depth: Scalar,
    },

    /// Vault: a rectangular base with a half-cylinder vault on top —
    /// the classic vaulted ceiling profile. Half-cylinder spans the
    /// full base_width; vault axis runs along +y.
    Vault {
        id: String,
        base_width: Scalar,
        base_length: Scalar,
        base_height: Scalar,
        segments: usize,
    },

    /// ShelfBracket: an L-shaped bracket with a diagonal brace from
    /// the bottom of the vertical leg to the end of the horizontal
    /// leg. Adds rigidity to a simple LBracket.
    ShelfBracket {
        id: String,
        horizontal: Scalar,
        vertical: Scalar,
        thickness: Scalar,
        depth: Scalar,
    },

    /// NameTag: a rounded-corner rectangular plate with a small
    /// circular hanging hole near the top edge.
    NameTag {
        id: String,
        width: Scalar,
        height: Scalar,
        thickness: Scalar,
        corner_radius: Scalar,
        hole_radius: Scalar,
        hole_offset_from_top: Scalar,
        segments: usize,
    },

    /// Plinth: a stepped square base — a wider square slab at the
    /// bottom, a narrower square slab on top. Used as a column or
    /// pedestal base.
    Plinth {
        id: String,
        bottom_side: Scalar,
        bottom_height: Scalar,
        top_side: Scalar,
        top_height: Scalar,
    },

    /// ParapetWall: a wall with a series of square crenellations
    /// (battlements) along its top. `length` of wall, `wall_height`
    /// before merlons, `merlon_count`, each merlon `merlon_height`
    /// tall, `wall_thickness`.
    ParapetWall {
        id: String,
        length: Scalar,
        wall_height: Scalar,
        wall_thickness: Scalar,
        merlon_count: usize,
        merlon_height: Scalar,
    },

    /// BeamWithHoles: a rectangular beam with a row of evenly spaced
    /// circular through-holes. Used for trusses, lifting bars.
    BeamWithHoles {
        id: String,
        length: Scalar,
        width: Scalar,
        height: Scalar,
        hole_radius: Scalar,
        hole_count: usize,
        segments: usize,
    },

    /// Ellipsoid3D: a general triaxial ellipsoid. Built as a faceted
    /// sphere of `radius=1` then scaled by `(rx, ry, rz)`.
    Ellipsoid3D {
        id: String,
        rx: Scalar,
        ry: Scalar,
        rz: Scalar,
        stacks: usize,
        slices: usize,
    },

    /// VectorArrow: a 3D vector visualization — a thin cylinder shaft
    /// from `from` to (very near) `to`, capped by a small cone tip
    /// pointing at `to`.
    VectorArrow {
        id: String,
        from: [Scalar; 3],
        to: [Scalar; 3],
        shaft_radius: Scalar,
        head_radius: Scalar,
        head_length: Scalar,
        segments: usize,
    },

    /// BoneShape: an elongated capsule with bulbous ends —
    /// like a dumbbell. Two SphereFaceted ends + cylindrical shaft.
    BoneShape {
        id: String,
        end_radius: Scalar,
        shaft_radius: Scalar,
        shaft_length: Scalar,
        stacks: usize,
        slices: usize,
    },

    /// Pawn: chess pawn — sphere on a tapered cylindrical body atop a
    /// disk base. Stylised, not anatomically correct.
    Pawn {
        id: String,
        base_radius: Scalar,
        base_height: Scalar,
        body_top_radius: Scalar,
        body_height: Scalar,
        head_radius: Scalar,
        segments: usize,
        stacks: usize,
    },

    /// Rook: chess rook — cylinder body on disk base, with crenellated
    /// top (similar to ParapetWall but radial).
    Rook {
        id: String,
        base_radius: Scalar,
        base_height: Scalar,
        body_radius: Scalar,
        body_height: Scalar,
        segments: usize,
    },

    /// Bishop: chess bishop — cylinder body + sphere head + thin slot
    /// across the head (the bishop's traditional "mitre slot").
    Bishop {
        id: String,
        base_radius: Scalar,
        base_height: Scalar,
        body_radius: Scalar,
        body_height: Scalar,
        head_radius: Scalar,
        slot_width: Scalar,
        segments: usize,
        stacks: usize,
    },

    /// Marker3D: a 6-axis cross marker — three perpendicular thin bars
    /// joined at the origin. Useful as a reference indicator at any
    /// point in space.
    Marker3D {
        id: String,
        center: [Scalar; 3],
        axis_length: Scalar,
        bar_radius: Scalar,
        segments: usize,
    },

    /// HollowBrick: a brick with a rectangular interior cavity (a
    /// classic cinder block). Open top.
    HollowBrick {
        id: String,
        length: Scalar,
        width: Scalar,
        height: Scalar,
        wall_thickness: Scalar,
    },

    /// StadiumPlate: flat stadium-shaped plate — same outline as Slot3D
    /// but framed as a "plate" (very thin) instead of a "slot".
    StadiumPlate {
        id: String,
        length: Scalar,
        width: Scalar,
        thickness: Scalar,
        segments: usize,
    },

    /// Bowtie: two triangles joined at a central vertex, lying in xy
    /// plane and extruded along +z. `width` is the total span along
    /// +x, `height` the span along +y, `pinch` the gap at center.
    Bowtie {
        id: String,
        width: Scalar,
        height: Scalar,
        pinch: Scalar,
        depth: Scalar,
    },

    /// HollowCone: a cone with an axial bore. Outer cone (faceted)
    /// minus a central cylindrical hole.
    HollowCone {
        id: String,
        radius: Scalar,
        height: Scalar,
        bore_radius: Scalar,
        segments: usize,
    },

    /// ArchedDoorway: rectangular base with a half-circle arch on top,
    /// hollow inside (so it's a door-shaped frame). Useful for
    /// gateways, archways. Wall thickness is uniform.
    ArchedDoorway {
        id: String,
        width: Scalar,
        height: Scalar,
        wall_thickness: Scalar,
        depth: Scalar,
        segments: usize,
    },

    /// CamLobe: an elliptical cam disk — like Cam but with two
    /// different radii (semi-major and semi-minor), so the cam profile
    /// is an ellipse. The hole is at the geometric center (ellipse origin).
    CamLobe {
        id: String,
        rx: Scalar,
        ry: Scalar,
        hole_radius: Scalar,
        thickness: Scalar,
        segments: usize,
    },

    /// ButtonShape: a flat disk with rounded outer edges — like a real
    /// button. Implemented as a thin cylinder. Visualised as a flat
    /// disk rather than truly rounded since real edge rounds need
    /// Shell-class kernel work.
    ButtonShape {
        id: String,
        radius: Scalar,
        thickness: Scalar,
        segments: usize,
    },

    /// FilletedSlot: a stadium-shaped slot but with rounded corners
    /// (ie a fully rounded rectangle, parameterized by its own corner
    /// radius rather than the slot end semicircle radius). When
    /// `corner_radius == width / 2`, this is identical to Slot3D.
    FilletedSlot {
        id: String,
        length: Scalar,
        width: Scalar,
        corner_radius: Scalar,
        depth: Scalar,
        segments: usize,
    },

    /// CoinShape: a flat disk with a raised rim around its edge.
    /// `outer_radius` is the disk radius, `face_thickness` the flat
    /// portion's thickness, `rim_height` how much the rim projects
    /// above the face, `rim_width` the rim's radial width.
    CoinShape {
        id: String,
        outer_radius: Scalar,
        face_thickness: Scalar,
        rim_height: Scalar,
        rim_width: Scalar,
        segments: usize,
    },

    /// CylindricalCap: top half of a cylinder, sliced by a vertical
    /// plane through the axis. Useful for half-pipe shapes.
    CylindricalCap {
        id: String,
        radius: Scalar,
        length: Scalar,
        segments: usize,
    },

    /// SquaredRing: rectangular outer outline minus rectangular inner
    /// outline (both axis-aligned), extruded along +z.
    SquaredRing {
        id: String,
        outer_width: Scalar,
        outer_height: Scalar,
        wall_thickness: Scalar,
        depth: Scalar,
    },

    /// WaveProfile: a sinusoidal-approximated wave plate — N peaks
    /// approximated as triangle ridges (like CorrugatedPanel) but
    /// with the profile in the xy plane and extruded along +z.
    /// Different orientation than CorrugatedPanel.
    WaveProfile {
        id: String,
        wavelength: Scalar,
        amplitude: Scalar,
        n_waves: usize,
        depth: Scalar,
        height: Scalar,
    },

    /// BulletShape: a cylinder with a hemispherical cap on top —
    /// the classic projectile profile.
    BulletShape {
        id: String,
        radius: Scalar,
        body_length: Scalar,
        stacks: usize,
        slices: usize,
    },

    /// TriangularPlate: an equilateral or arbitrary triangle plate
    /// extruded by `thickness`. Specified by 3 (x, y) corner points.
    TriangularPlate {
        id: String,
        a: [Scalar; 2],
        b: [Scalar; 2],
        c: [Scalar; 2],
        thickness: Scalar,
    },

    /// Heart: a heart-shaped extruded plate. `size` is the bounding box
    /// edge length, `thickness` the extrusion depth.
    Heart {
        id: String,
        size: Scalar,
        thickness: Scalar,
        segments: usize,
    },

    /// ChainLink: an oval ring — a stadium-shaped outer profile minus
    /// a stadium-shaped inner profile. `length` and `width` are the
    /// outer dimensions, `wall_thickness` the wall around the inner
    /// stadium hole. Extruded along +z by `depth`.
    ChainLink {
        id: String,
        length: Scalar,
        width: Scalar,
        wall_thickness: Scalar,
        depth: Scalar,
        segments: usize,
    },

    /// SpiralPlate: an Archimedean-spiral pattern of N revolutions
    /// approximated as a chain of straight segments swept as a thin
    /// rod. Decorative.
    SpiralPlate {
        id: String,
        max_radius: Scalar,
        revolutions: Scalar,
        rod_radius: Scalar,
        z: Scalar,
        segments_per_revolution: usize,
    },

    /// WindowFrame: a rectangular frame — outer rectangle minus inner
    /// rectangular cutout, extruded along +z. Like SquaredRing but
    /// with explicit "window frame" intent and asymmetric inner offset.
    WindowFrame {
        id: String,
        outer_width: Scalar,
        outer_height: Scalar,
        frame_thickness: Scalar,
        depth: Scalar,
    },

    /// SquareKey: a square cross-section key — extruded along +z by
    /// `length`. Used for keyed shaft connections.
    SquareKey {
        id: String,
        side: Scalar,
        length: Scalar,
    },

    /// DiskWithSlots: a disk with N evenly-spaced radial rectangular
    /// slots cut from the rim. Used for slotted couplings, wheels.
    DiskWithSlots {
        id: String,
        radius: Scalar,
        slot_count: usize,
        slot_width: Scalar,
        slot_depth: Scalar,
        thickness: Scalar,
        segments: usize,
    },

    /// FivePointedBadge: a 5-pointed star plate (alias of Star with
    /// fixed 5 points and explicit "badge" intent).
    FivePointedBadge {
        id: String,
        outer_radius: Scalar,
        inner_radius: Scalar,
        thickness: Scalar,
    },

    /// Crescent: two overlapping disks where the smaller disk is
    /// subtracted from the larger to form a crescent moon shape.
    /// Both disks lie in xy plane, extruded along +z. The small disk
    /// is offset by `offset` from the large disk's center.
    Crescent {
        id: String,
        outer_radius: Scalar,
        inner_radius: Scalar,
        offset: Scalar,
        thickness: Scalar,
        segments: usize,
    },

    /// Hourglass: two frustums meeting waist-to-waist — wide at top
    /// and bottom, narrow in the middle. Symmetric: same radii at top
    /// and bottom, single `waist_radius` at center.
    Hourglass {
        id: String,
        end_radius: Scalar,
        waist_radius: Scalar,
        half_height: Scalar,
        segments: usize,
    },

    /// Diabolo: an inverted hourglass — wide in the middle, narrow at
    /// the ends. Like Hourglass but with the waist and end radii
    /// swapped.
    Diabolo {
        id: String,
        end_radius: Scalar,
        waist_radius: Scalar,
        half_height: Scalar,
        segments: usize,
    },

    /// TripleStep: three stacked cylindrical tiers of decreasing
    /// radius (a wedding-cake or stepped pyramid silhouette).
    TripleStep {
        id: String,
        bottom_radius: Scalar,
        bottom_height: Scalar,
        middle_radius: Scalar,
        middle_height: Scalar,
        top_radius: Scalar,
        top_height: Scalar,
        segments: usize,
    },

    /// WingedScrew: a screw with two flat "wings" on either side of
    /// the head for hand-tightening. Cylinder shaft + center hex
    /// head + two rectangular wing tabs.
    WingedScrew {
        id: String,
        shaft_radius: Scalar,
        shaft_length: Scalar,
        head_radius: Scalar,
        head_height: Scalar,
        wing_length: Scalar,
        wing_thickness: Scalar,
        segments: usize,
    },

    /// KneadHandle: a simple grip handle — flat rectangular plate
    /// with a thicker raised pad in the center. Useful as an
    /// ergonomic grip.
    KneadHandle {
        id: String,
        length: Scalar,
        width: Scalar,
        plate_thickness: Scalar,
        pad_radius: Scalar,
        pad_height: Scalar,
        segments: usize,
    },

    /// ZigzagBar: a horizontal bar with a triangular zigzag profile
    /// along its top — like a saw blade silhouette extruded.
    ZigzagBar {
        id: String,
        length: Scalar,
        depth: Scalar,
        base_height: Scalar,
        zigzag_height: Scalar,
        n_zigs: usize,
    },

    /// FishingFloat: a sphere with a small cylindrical pin sticking
    /// out the top (and a smaller one out the bottom). Stylised
    /// fishing-float silhouette.
    FishingFloat {
        id: String,
        body_radius: Scalar,
        pin_radius: Scalar,
        pin_length: Scalar,
        stacks: usize,
        slices: usize,
    },

    /// Tetrahedron: regular 4-faced pyramid with equilateral triangle
    /// base. `edge_length` is the side length of every edge.
    Tetrahedron {
        id: String,
        edge_length: Scalar,
    },

    /// Spool: a cable reel — a cylinder body with two wider disk
    /// flanges on each end. `body_radius` < `flange_radius`,
    /// `body_length` is the central cylinder length, `flange_thickness`
    /// the thickness of each end disk.
    Spool {
        id: String,
        body_radius: Scalar,
        body_length: Scalar,
        flange_radius: Scalar,
        flange_thickness: Scalar,
        segments: usize,
    },

    /// Lampshade: an open hollow frustum — same as FunnelTube but
    /// with explicit "lampshade" intent and a guaranteed open top
    /// (no rim).
    Lampshade {
        id: String,
        top_radius: Scalar,
        bottom_radius: Scalar,
        height: Scalar,
        wall_thickness: Scalar,
        segments: usize,
    },

    /// PrismHole: an n-sided regular prism with a central cylindrical
    /// hole through it.
    PrismHole {
        id: String,
        sides: usize,
        outer_radius: Scalar,
        hole_radius: Scalar,
        height: Scalar,
        segments: usize,
    },

    /// KeyholeShape: a keyhole-shaped extruded plate — circular hole
    /// at the top with a rectangular slot beneath it. The classic
    /// keyhole silhouette.
    KeyholeShape {
        id: String,
        circle_radius: Scalar,
        slot_width: Scalar,
        slot_height: Scalar,
        plate_width: Scalar,
        plate_height: Scalar,
        plate_thickness: Scalar,
        segments: usize,
    },

    /// AcornShape: a stylised acorn — sphere body topped by a small
    /// cylindrical stem. Two pieces unioned.
    AcornShape {
        id: String,
        body_radius: Scalar,
        stem_radius: Scalar,
        stem_length: Scalar,
        stacks: usize,
        slices: usize,
    },

    /// Mushroom: cylindrical stem topped by a wider spherical cap.
    /// Sphere center is at the top of the stem, so the cap drapes
    /// outward beyond the stem.
    Mushroom {
        id: String,
        stem_radius: Scalar,
        stem_height: Scalar,
        cap_radius: Scalar,
        stacks: usize,
        slices: usize,
    },

    /// Lightbulb: a cylindrical base, frustum neck transition, and
    /// spherical bulb on top. Three-piece composition.
    Lightbulb {
        id: String,
        base_radius: Scalar,
        base_height: Scalar,
        neck_height: Scalar,
        bulb_radius: Scalar,
        stacks: usize,
        slices: usize,
    },

    /// DomedRoof: a cylinder topped with a hemisphere of the same
    /// radius — silo, roundhouse, or kiln-cap shape.
    DomedRoof {
        id: String,
        radius: Scalar,
        wall_height: Scalar,
        stacks: usize,
        slices: usize,
    },

    /// Beehive: a stack of progressively narrower cylinders, evoking
    /// a traditional skep beehive. Four cylinder layers unioned.
    Beehive {
        id: String,
        base_radius: Scalar,
        layer_height: Scalar,
        layers: usize,
        slices: usize,
    },

    /// Bullet: cylindrical body with a conical tip pointing up.
    /// Apex of the cone is at z = body_height + tip_height.
    Bullet {
        id: String,
        body_radius: Scalar,
        body_height: Scalar,
        tip_height: Scalar,
        slices: usize,
    },

    /// PointedDome: hemispherical dome with a conical spire on top —
    /// minaret / orthodox-cathedral cap.
    PointedDome {
        id: String,
        radius: Scalar,
        spire_height: Scalar,
        stacks: usize,
        slices: usize,
    },

    /// WindBell: a hand-bell — wider frustum body (wider at bottom)
    /// with a small cylindrical handle on top.
    WindBell {
        id: String,
        bell_radius: Scalar,
        bell_top_radius: Scalar,
        bell_height: Scalar,
        handle_radius: Scalar,
        handle_height: Scalar,
        slices: usize,
    },

    /// PineCone: stack of progressively narrower faceted spheres,
    /// evoking the layered scales of a pinecone.
    PineCone {
        id: String,
        base_radius: Scalar,
        scale_overlap: Scalar,
        scales: usize,
        stacks: usize,
        slices: usize,
    },

    /// TopHat: a tall cylinder body with a flat disk brim around the
    /// bottom. Brim radius is wider than body radius.
    TopHat {
        id: String,
        body_radius: Scalar,
        body_height: Scalar,
        brim_radius: Scalar,
        brim_thickness: Scalar,
        slices: usize,
    },

    /// WaterTower: a wide cylindrical tank perched on a slimmer
    /// cylindrical support column.
    WaterTower {
        id: String,
        tank_radius: Scalar,
        tank_height: Scalar,
        support_radius: Scalar,
        support_height: Scalar,
        slices: usize,
    },

    /// PlantPot: an inverted frustum (wider at top) with a flat ring
    /// rim on top — terra-cotta planter shape. Open top for now is
    /// implied by the silhouette; this is a solid prim, no carve.
    PlantPot {
        id: String,
        rim_radius: Scalar,
        base_radius: Scalar,
        height: Scalar,
        rim_thickness: Scalar,
        slices: usize,
    },

    /// Buoy: a spherical float with a cylindrical mast extending
    /// upward. Used in marine markers, decorative compositions.
    Buoy {
        id: String,
        float_radius: Scalar,
        mast_radius: Scalar,
        mast_height: Scalar,
        stacks: usize,
        slices: usize,
    },

    /// Trophy: cylindrical base + slimmer cylindrical stem + bowl cup
    /// at the top (frustum bowl shape, wider opening above).
    Trophy {
        id: String,
        base_radius: Scalar,
        base_height: Scalar,
        stem_radius: Scalar,
        stem_height: Scalar,
        bowl_bottom_radius: Scalar,
        bowl_top_radius: Scalar,
        bowl_height: Scalar,
        slices: usize,
    },

    /// Goblet: a stem with a frustum bowl on top (no base disk).
    /// Wine glass / chalice silhouette.
    Goblet {
        id: String,
        stem_radius: Scalar,
        stem_height: Scalar,
        bowl_bottom_radius: Scalar,
        bowl_top_radius: Scalar,
        bowl_height: Scalar,
        slices: usize,
    },

    /// TableLamp: cylindrical base + cylindrical stem + frustum shade
    /// (wider at the bottom of the shade, narrower at top).
    TableLamp {
        id: String,
        base_radius: Scalar,
        base_height: Scalar,
        stem_radius: Scalar,
        stem_height: Scalar,
        shade_bottom_radius: Scalar,
        shade_top_radius: Scalar,
        shade_height: Scalar,
        slices: usize,
    },

    /// MushroomCloud: a frustum stem widening upward + a spherical
    /// cap on top — silhouette of a mushroom cloud.
    MushroomCloud {
        id: String,
        stem_bottom_radius: Scalar,
        stem_top_radius: Scalar,
        stem_height: Scalar,
        cloud_radius: Scalar,
        stacks: usize,
        slices: usize,
    },

    /// TieredCake: three cylindrical tiers of decreasing radius
    /// stacked on top of each other. Wedding-cake silhouette.
    TieredCake {
        id: String,
        bottom_radius: Scalar,
        middle_radius: Scalar,
        top_radius: Scalar,
        tier_height: Scalar,
        slices: usize,
    },

    /// Spindle: a cylinder body with conical caps at both ends.
    /// Symmetrical, evokes a textile spindle or shuttlecock body.
    Spindle {
        id: String,
        body_radius: Scalar,
        body_height: Scalar,
        cap_height: Scalar,
        slices: usize,
    },

    /// Ufo: a flat oblate disk (cylinder of small height) with a
    /// hemispherical dome on top — flying-saucer silhouette.
    Ufo {
        id: String,
        disk_radius: Scalar,
        disk_height: Scalar,
        dome_radius: Scalar,
        stacks: usize,
        slices: usize,
    },

    /// CrowsNest: tall slim pole with a flat circular platform at
    /// the top — lookout, antenna mast.
    CrowsNest {
        id: String,
        pole_radius: Scalar,
        pole_height: Scalar,
        platform_radius: Scalar,
        platform_thickness: Scalar,
        slices: usize,
    },

    /// Volute: a decorative scroll/spiral disk — Archimedean spiral
    /// rod plus a central disk. Used as classical column-capital
    /// ornament.
    Volute {
        id: String,
        max_radius: Scalar,
        revolutions: Scalar,
        rod_radius: Scalar,
        center_disk_radius: Scalar,
        thickness: Scalar,
        segments_per_revolution: usize,
        center_segments: usize,
    },

    /// ScrollPlate: a flat S-curve scroll plate — two oppositely-curving
    /// arc segments joined. Decorative ornament.
    ScrollPlate {
        id: String,
        radius: Scalar,
        rod_radius: Scalar,
        thickness: Scalar,
        segments: usize,
    },

    /// TableTop: a rectangular slab top supported by four cylindrical legs at
    /// the corners. The slab spans x ∈ [0, top_width], y ∈ [0, top_depth] and
    /// sits at z ∈ [leg_height, leg_height + top_thickness]. Each leg is a
    /// cylinder of `leg_radius` running from z=0 to z=leg_height, inset from
    /// each corner by `leg_inset`. Legs poke 1e-3 into the slab to seal the
    /// boolean union.
    TableTop {
        id: String,
        top_width: Scalar,
        top_depth: Scalar,
        top_thickness: Scalar,
        leg_radius: Scalar,
        leg_height: Scalar,
        leg_inset: Scalar,
        segments: usize,
    },

    /// Bench: a long rectangular seat slab on two box-shaped supports. The
    /// seat spans x ∈ [0, length], y ∈ [0, depth], z ∈ [leg_height,
    /// leg_height + seat_thickness]. Two supports of `support_thickness` x
    /// `depth` x `leg_height` sit at each end, inset from the seat ends by
    /// `support_inset`.
    Bench {
        id: String,
        length: Scalar,
        depth: Scalar,
        seat_thickness: Scalar,
        leg_height: Scalar,
        support_thickness: Scalar,
        support_inset: Scalar,
    },

    /// WindowLouver: an outer rectangular frame (window-frame style) with N
    /// horizontal angled slats inside. The frame outer is `outer_width` x
    /// `outer_height` with frame walls of `frame_thickness`, extruded by
    /// `depth`. `slats` rectangular slats of dimensions (inner_width,
    /// slat_thickness, depth) are stacked inside the inner cavity, evenly
    /// spaced. (Slats currently axis-aligned; the angle parameter is recorded
    /// for future tilt support.)
    WindowLouver {
        id: String,
        outer_width: Scalar,
        outer_height: Scalar,
        frame_thickness: Scalar,
        depth: Scalar,
        slats: usize,
        slat_thickness: Scalar,
    },

    /// Hammer: a rectangular hammer head with a cylindrical handle below it.
    /// Head is `head_length` x `head_width` x `head_height`, centered on the
    /// xy origin, sitting at z ∈ [handle_length, handle_length + head_height].
    /// Handle is a cylinder of `handle_radius` running from z=0 to
    /// z=handle_length + 1e-3 (pokes 1e-3 into head).
    Hammer {
        id: String,
        head_length: Scalar,
        head_width: Scalar,
        head_height: Scalar,
        handle_radius: Scalar,
        handle_length: Scalar,
        segments: usize,
    },

    /// ScrewDriver: a fat cylindrical handle, a thinner cylindrical shaft,
    /// and a flat-blade tip. Handle of `handle_radius` and `handle_length`
    /// at z ∈ [0, handle_length]. Shaft of `shaft_radius` (< handle_radius)
    /// at z ∈ [handle_length - 1e-3, handle_length + shaft_length]. Tip is
    /// a thin flat box of `tip_width` x `tip_thickness` x `tip_length` at
    /// the shaft's end.
    ScrewDriver {
        id: String,
        handle_radius: Scalar,
        handle_length: Scalar,
        shaft_radius: Scalar,
        shaft_length: Scalar,
        tip_width: Scalar,
        tip_thickness: Scalar,
        tip_length: Scalar,
        segments: usize,
    },

    /// Wrench: a rectangular head with a centered jaw cutout, joined to a
    /// tapered handle. The head is `head_width` x `head_length` x
    /// `thickness`, sitting at one end. The handle is a tapered prism
    /// (frustum-as-extrude) running from `head_length` along +y for
    /// `handle_length`, narrowing from `handle_root_width` to
    /// `handle_tip_width`. (No actual jaw cutout in this MVP — implemented
    /// as a solid rectangular head; jaw cutout is future work.)
    Wrench {
        id: String,
        head_width: Scalar,
        head_length: Scalar,
        thickness: Scalar,
        handle_root_width: Scalar,
        handle_tip_width: Scalar,
        handle_length: Scalar,
    },

    /// Heart3D: a 3D heart shape — two faceted spheres for the lobes joined
    /// to a downward-pointing cone for the bottom point. Lobe spheres of
    /// `lobe_radius` are placed at (±lobe_offset, 0, lobe_z). The cone
    /// has its apex at (0, 0, 0) and its base of `lobe_radius` at z=lobe_z.
    /// Uses the AcornShape pole-overlap pattern for sphere unions.
    Heart3D {
        id: String,
        lobe_radius: Scalar,
        lobe_offset: Scalar,
        lobe_z: Scalar,
        stacks: usize,
        slices: usize,
    },

    /// Star3D: a 3D 6-pointed star shape — three orthogonal elongated boxes
    /// crossing at the origin, forming a 3D plus / asterisk. Each arm is a
    /// box of `arm_length` x `arm_thickness` x `arm_thickness`, centered on
    /// origin, oriented along x, y, and z respectively.
    Star3D {
        id: String,
        arm_length: Scalar,
        arm_thickness: Scalar,
    },

    /// Cross3D: a 3D Christian cross — a vertical box of (width x depth x
    /// height) and a horizontal crossbar of (arm_span x depth x arm_height)
    /// at z = arm_z. The crossbar is centered on x. The vertical post sits
    /// at x ∈ [0, width] (with arm_span > width, the crossbar overhangs
    /// symmetrically).
    Cross3D {
        id: String,
        width: Scalar,
        depth: Scalar,
        height: Scalar,
        arm_span: Scalar,
        arm_height: Scalar,
        arm_z: Scalar,
    },

    /// Chair: a square seat box on four legs, with a vertical back rest
    /// extending up from the rear edge. Seat sits at z ∈ [leg_height,
    /// leg_height + seat_thickness], spanning x ∈ [0, seat_size] and
    /// y ∈ [0, seat_size]. Four box legs of (leg_thickness)² x leg_height,
    /// inset from corners by `leg_inset`. Back rest is a vertical box at
    /// y ∈ [0, back_thickness], x ∈ [0, seat_size], z ∈ [leg_height +
    /// seat_thickness, leg_height + seat_thickness + back_height].
    Chair {
        id: String,
        seat_size: Scalar,
        seat_thickness: Scalar,
        leg_height: Scalar,
        leg_thickness: Scalar,
        leg_inset: Scalar,
        back_thickness: Scalar,
        back_height: Scalar,
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

    /// Hollow out `input` by offsetting every face inward by `thickness`,
    /// producing a solid whose outer surface is the original and whose inner
    /// surface is the inward offset. Implemented as `input - inner_offset`
    /// using a per-vertex inward-plane-intersection algorithm.
    ///
    /// **Planar primitives only.** Inputs whose faces are not all `Plane`
    /// surfaces (Sphere, Cone, Frustum, Torus, Cylinder, and any composite
    /// containing those) are rejected with `EvalError::Invalid`. Curved-
    /// surface Shell needs offset-surface math the kernel doesn't yet have
    /// — see STATUS.md.
    ///
    /// Best-tested for **convex polyhedra**: Box, RegularPrism, Pyramid,
    /// ExtrudePolygon (convex), Wedge, Star (convex), and the structural
    /// shapes (LBracket, UChannel, TBeam, IBeam, etc.). Concave inputs may
    /// produce a self-intersecting inner solid that trips the boolean
    /// engine — those return a Boolean error.
    ///
    /// `thickness` must satisfy `0 < thickness < min_dimension/2`; otherwise
    /// `Invalid` is returned.
    Shell {
        id: String,
        input: String,
        thickness: Scalar,
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

    // -------------------------------------------------------------------
    // Reference geometry batch 3 (5 features) — ship 2026-05-08.
    // -------------------------------------------------------------------

    /// Midplane reference: a thin RefPlane-style box marker placed
    /// halfway between two world-space points along a chosen axis. The
    /// midplane's normal is `axis` ("x"|"y"|"z"); its position sits at
    /// the midpoint of `position_a` and `position_b`. `extents` is the
    /// (width, height) of the rectangular marker (in the perpendicular
    /// axes); `marker_thickness` is the thickness along the normal.
    MidPlaneRef {
        id: String,
        position_a: [Scalar; 3],
        position_b: [Scalar; 3],
        axis: String,
        extents: [Scalar; 2],
        marker_thickness: Scalar,
    },

    /// Reference plane perpendicular to a named axis ("x"|"y"|"z"),
    /// passing through `point`. Same visual as RefPlane (a thin
    /// rectangular slab) — the axis IS the plane normal. `extents` and
    /// `marker_thickness` work identically to RefPlane.
    PerpRefPlane {
        id: String,
        axis: String,
        point: [Scalar; 3],
        extents: [Scalar; 2],
        marker_thickness: Scalar,
    },

    /// Reference plane parallel to a base plane, offset by `offset`
    /// along its normal (`axis`). `base_position` defines the base
    /// plane's center. The result is a RefPlane-like marker centered at
    /// `base_position + offset * axis_unit`.
    OffsetRefPlane {
        id: String,
        base_position: [Scalar; 3],
        axis: String,
        offset: Scalar,
        extents: [Scalar; 2],
        marker_thickness: Scalar,
    },

    /// Coordinate-axis triad at the origin. Three thin cylinders along
    /// the x, y, z axes, each spanning [-length, +length] (centered on
    /// origin). Visual scaffolding for orienting a model. Built using
    /// the same robust 3-bar union as Marker3D: bars have slightly
    /// different lengths and offsets along their own axis so their
    /// caps never sit coplanar with another bar's lateral surface.
    /// `head_length` is unused and kept for backward compatibility.
    CoordinateAxes {
        id: String,
        length: Scalar,
        bar_radius: Scalar,
        head_length: Scalar,
        segments: usize,
    },

    /// Origin point marker: a tiny faceted sphere at world origin (0,0,0).
    /// Specialization of RefPoint with implicit position = origin —
    /// useful for declaring "this part is meant to be aligned to the
    /// world origin" without spelling out the position.
    OriginPoint {
        id: String,
        marker_radius: Scalar,
    },

    // -------------------------------------------------------------------
    // Reference geometry batch 2 — ship 2026-05-10.
    // -------------------------------------------------------------------

    /// CenterMarker: a 3D crosshair at a world-space `position`. Three
    /// thin rods (cylinders) pass through the point — one along each of
    /// the x, y, z axes — each centered on `position` with half-length
    /// `size / 2`. Each rod has radius `rod_radius`. Useful as a visual
    /// "snapping target" or centroid indicator in CAD models.
    CenterMarker {
        id: String,
        position: [Scalar; 3],
        size: Scalar,
        rod_radius: Scalar,
        segments: usize,
    },

    /// AxisLabel: a visual axis indicator — a cylinder shaft from `origin`
    /// in the given `direction` (does not need to be unit-length; it is
    /// normalised internally) with a cone tip at the far end. The shaft
    /// runs from `origin` for `length` units; the cone head is centred at
    /// `origin + normalised(direction) * length`. `head_radius` must be
    /// greater than `shaft_radius`. Segments >= 6 required.
    AxisLabel {
        id: String,
        origin: [Scalar; 3],
        direction: [Scalar; 3],
        length: Scalar,
        head_radius: Scalar,
        shaft_radius: Scalar,
        segments: usize,
    },

    /// DistanceMarker: a visual measurement indicator between two points.
    /// A thin rod (shaft) connects `from` to `to`; an outward-pointing cone
    /// (arrowhead) is placed at each end, centred on the end-point. Both
    /// cones point away from the shaft (i.e. the `from` cone points toward
    /// `from - dir` and the `to` cone points toward `to + dir`). Useful
    /// for annotating dimensions directly in 3D geometry.
    DistanceMarker {
        id: String,
        from: [Scalar; 3],
        to: [Scalar; 3],
        shaft_radius: Scalar,
        head_radius: Scalar,
        segments: usize,
    },

    // -------------------------------------------------------------------
    // Manufacturing batch 4 (5 features) — ship 2026-05-08.
    // -------------------------------------------------------------------

    /// CenterDrill: a starter feature for drilling — a small cone (the
    /// "spot" portion) topped by an inverted frustum (the chamfer that
    /// guides the drill bit). Built as a standalone primitive (no
    /// `input`) — acts as the cutter shape that would be subtracted
    /// from a body. The chamfer frustum sits at z ∈ [0, chamfer_depth]
    /// (radius `chamfer_radius` at top, `drill_radius` at bottom). The
    /// drill cone tapers from `drill_radius` at z=0 down to a point at
    /// z=-drill_depth.
    CenterDrill {
        id: String,
        drill_radius: Scalar,
        drill_depth: Scalar,
        chamfer_radius: Scalar,
        chamfer_depth: Scalar,
        segments: usize,
    },

    /// OilHole: a body with a thin oil-passage hole drilled at an
    /// angle. Body is a cylinder along +z (radius `body_radius`,
    /// height `body_height`). The oil hole is a thin cylinder of
    /// `hole_radius` whose axis runs from `entry` to `exit` (both in
    /// world space). Built standalone (geometry shows the body
    /// MINUS the angled cylinder).
    OilHole {
        id: String,
        body_radius: Scalar,
        body_height: Scalar,
        hole_radius: Scalar,
        entry: [Scalar; 3],
        exit: [Scalar; 3],
        body_segments: usize,
        hole_segments: usize,
    },

    /// ReliefCut: a stepped cylindrical shaft with a relief groove cut
    /// at the base of the shoulder. Geometry: a "small" cylinder
    /// (`small_radius`, height `small_height`) sitting on top of a
    /// "large" cylinder (`large_radius`, height `large_height`); a
    /// thin annular relief groove of depth `relief_depth` and width
    /// `relief_width` is cut into the large cylinder right at the
    /// shoulder (at z = large_height, radially undercutting the
    /// transition). Standalone — used as a manufacturing shape ref.
    ReliefCut {
        id: String,
        small_radius: Scalar,
        small_height: Scalar,
        large_radius: Scalar,
        large_height: Scalar,
        relief_width: Scalar,
        relief_depth: Scalar,
        segments: usize,
    },

    /// WrenchFlats: a cylindrical shaft with two flat machined faces on
    /// opposite sides (a "double-D" / hex/square wrench fitting). The
    /// shaft is +z aligned (radius `shaft_radius`, length
    /// `shaft_length`). Two flats are subtracted at z ∈
    /// [flats_z_start, flats_z_start + flats_length] — each flat is
    /// machined to leave `flat_distance` between the two opposite
    /// flat faces (i.e. flat_distance < 2*shaft_radius). Standalone
    /// primitive (no `input`).
    WrenchFlats {
        id: String,
        shaft_radius: Scalar,
        shaft_length: Scalar,
        flats_z_start: Scalar,
        flats_length: Scalar,
        flat_distance: Scalar,
        segments: usize,
    },

    /// Knurl: a knurled cylindrical grip — visual approximation of a
    /// knurl pattern by carving many shallow axial grooves into a
    /// cylinder. Cylinder is +z aligned (radius `radius`, height
    /// `height`); `groove_count` axial grooves of `groove_depth` and
    /// `groove_width` are cut at evenly-spaced angles around the
    /// circumference. Built directly via an extruded star-style
    /// profile (alternating outer-radius arcs and notch-radius
    /// notches), so no booleans are required and the result is robust.
    Knurl {
        id: String,
        radius: Scalar,
        height: Scalar,
        groove_depth: Scalar,
        groove_count: usize,
    },

    // -------------------------------------------------------------------
    // Manufacturing batch 5 (5 features) — ship 2026-05-10.
    // -------------------------------------------------------------------

    /// **ChamferedHole**: counterbore + 45° chamfer at the top edge of the
    /// bore. Drills a cylinder of `hole_radius`/`hole_depth` and adds a
    /// conical chamfer frustum (from `hole_radius` at `chamfer_depth` to
    /// `chamfer_radius` at the surface) at the top of the opening.
    /// `axis` is "x"|"y"|"z" (drill direction is -axis). `center` is the
    /// center of the opening on the +axis-facing surface.
    ChamferedHole {
        id: String,
        input: String,
        center: [Scalar; 3],
        axis: String,
        hole_radius: Scalar,
        hole_depth: Scalar,
        chamfer_radius: Scalar,
        chamfer_depth: Scalar,
    },

    /// **ThreadedHoleMarker**: visual marker for a threaded hole in `input`.
    /// Drills a cylinder of `thread_diameter / 2` radius and `depth` along
    /// -`axis` from `center` on the +axis face. Tags the hole with
    /// `face_owner_tag = "thread"` so drawing tools can render the thread
    /// symbol. No actual threads are modelled.
    ThreadedHoleMarker {
        id: String,
        input: String,
        center: [Scalar; 3],
        axis: String,
        thread_diameter: Scalar,
        depth: Scalar,
        thread_pitch: Scalar,
    },

    /// **BoltPattern**: circular array of `count` bolt holes, each of
    /// `hole_radius` and `hole_depth`, distributed at `pattern_radius` from
    /// `center` around `axis`. `phase` (radians) offsets the first hole.
    /// Drills all holes into `input` in a single feature.
    BoltPattern {
        id: String,
        input: String,
        center: [Scalar; 3],
        axis: String,
        pattern_radius: Scalar,
        hole_radius: Scalar,
        hole_depth: Scalar,
        count: usize,
        phase: Scalar,
    },

    /// **SquareDrive**: square pocket for a square key or drive. Subtracts
    /// a square prismatic pocket of `side_length` × `side_length` × `depth`
    /// from `input`, centered at `center` on the +`axis`-facing surface.
    /// The pocket axis runs in -`axis` from the opening at `center`.
    SquareDrive {
        id: String,
        input: String,
        center: [Scalar; 3],
        axis: String,
        side_length: Scalar,
        depth: Scalar,
    },

    /// **RaisedBoss**: raised cylindrical boss on `input` with an embedded
    /// blind screw hole. Unions a cylinder of `boss_radius`/`boss_height`
    /// at `center` on the +`axis` surface, then drills a blind hole of
    /// `hole_radius`/`hole_depth` along -`axis` from the top of the boss.
    RaisedBoss {
        id: String,
        input: String,
        center: [Scalar; 3],
        axis: String,
        boss_radius: Scalar,
        boss_height: Scalar,
        hole_radius: Scalar,
        hole_depth: Scalar,
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



    // --- Drawings v2 polish batch (2026-05-08, part 3) -------------------

    /// Funnel2: a tapered double-cone funnel — a wide top frustum joined
    /// to a narrow bottom frustum at a shared neck radius. Same z-anchored
    /// silhouette as Funnel but with a conical (rather than cylindrical)
    /// spout, useful for rendering hourglass-style funnels and chemistry
    /// glassware. Top sits at z ∈ [neck_z, top_z] tapering from
    /// `neck_radius` at the bottom to `top_radius` at the top. Bottom
    /// sits at z ∈ [-bottom_length, 0] tapering from `neck_radius` at the
    /// top to `bottom_radius` at the bottom.
    Funnel2 {
        id: String,
        top_radius: Scalar,
        neck_radius: Scalar,
        bottom_radius: Scalar,
        top_z: Scalar,
        bottom_length: Scalar,
        segments: usize,
    },

    /// CrossPipe: 90° T/cross-pipe junction — two perpendicular cylinders
    /// of `radius` joined at the origin. `axis_a` and `axis_b` are the
    /// axis names for each cylinder ("x", "y", or "z"); the two must be
    /// different. Each arm extends `arm_length` away from the origin in
    /// both directions along its axis. Useful for plumbing T-fittings.
    CrossPipe {
        id: String,
        radius: Scalar,
        arm_length: Scalar,
        axis_a: String,
        axis_b: String,
        segments: usize,
    },

    /// AnchorChain: a thicker chain link variant — extruded oval (stadium)
    /// outline, like ChainLink, but with a centered crossbar (a thin box)
    /// across the inside opening. Used for marine anchor chain. The
    /// crossbar adds to the link rather than replacing the bore: the
    /// pocket is split into two D-shaped halves.
    AnchorChain {
        id: String,
        length: Scalar,
        width: Scalar,
        wall_thickness: Scalar,
        bar_thickness: Scalar,
        depth: Scalar,
        segments: usize,
    },

    /// GearBlank2: gear blank with a "missing tooth" notch — same as
    /// GearBlank but with one tooth left out (a flat at the missing
    /// position) and an indexing notch carved into the root at the
    /// notch position. Visual-only approximation: the notch is a
    /// rectangular cut on the root circle, NOT a true involute tooth.
    /// Useful for index/timing gears.
    GearBlank2 {
        id: String,
        outer_radius: Scalar,
        root_radius: Scalar,
        tooth_count: usize,
        thickness: Scalar,
        notch_depth: Scalar,
        segments_per_tooth: usize,
    },

    /// PaperClipShape: bent-wire approximation of a standard paper clip
    /// — a chain of cylinder segments laid out along a serpentine
    /// rectangular path. Three nested rectangles connected at their
    /// short ends, each rectangle smaller than the last. Lies in the
    /// xy-plane at the requested `z` height; rod radius is `rod_radius`.
    PaperClipShape {
        id: String,
        outer_length: Scalar,
        outer_width: Scalar,
        gap: Scalar,
        rod_radius: Scalar,
        z: Scalar,
        segments: usize,
    },

    /// Caltrops: 4 spheres at the vertices of a regular tetrahedron,
    /// joined by 6 cylindrical struts (one per tetrahedron edge). Visual
    /// approximation of the four-spike caltrop weapon. `sphere_radius`
    /// is the bead at each vertex; `strut_radius` is the connector rod.
    Caltrops {
        id: String,
        edge_length: Scalar,
        sphere_radius: Scalar,
        strut_radius: Scalar,
        stacks: usize,
        slices: usize,
        strut_segments: usize,
    },


    /// TulipBulb: a small, elegant bulb — sphere body with a slim cylindrical
    /// neck rising from the top. Like AcornShape but proportioned for a
    /// tulip-bulb silhouette (taller neck, smaller bulb relative to neck).
    /// Uses the AcornShape pole-overlap pattern (br - 1e-3) to avoid
    /// stitch trips at the sphere/cylinder join.
    TulipBulb {
        id: String,
        bulb_radius: Scalar,
        neck_radius: Scalar,
        neck_length: Scalar,
        stacks: usize,
        slices: usize,
    },

    /// PaperLantern: a cylindrical body with a hemisphere cap on each end.
    /// Like a stretched capsule but explicitly a lantern silhouette —
    /// hemispheres of the body radius, body cylinder between them.
    PaperLantern {
        id: String,
        body_radius: Scalar,
        body_height: Scalar,
        stacks: usize,
        slices: usize,
    },

    /// PaperLanternStrips: a decorative paper-lantern look built from
    /// `strip_count` vertical strip cylinders arranged around a torus centre.
    /// Each strip is a thin cylinder of radius `minor_radius` placed at
    /// distance `axis_radius` from the z-axis, equally spaced in angle.
    /// `strip_width_deg` controls the angular arc each strip subtends (only
    /// affects the strip cylinder radius — the strip cylinder radius equals
    /// axis_radius * sin(strip_width_deg/2 * pi/180)).
    PaperLanternStrips {
        id: String,
        axis_radius: Scalar,
        minor_radius: Scalar,
        strip_count: usize,
        strip_width_deg: Scalar,
        segments: usize,
    },

    /// Trefoil: a faceted tube swept along the trefoil knot curve.
    /// Parametric curve: t in [0, 2π],
    ///   x = (sin(t) + 2 sin(2t)) * scale
    ///   y = (cos(t) − 2 cos(2t)) * scale
    ///   z = −sin(3t) * scale
    /// `tube_radius` is the radius of the circular cross-section tube.
    Trefoil {
        id: String,
        scale: Scalar,
        tube_radius: Scalar,
        segments_along: usize,
        segments_around: usize,
    },

    /// DishCap: a shallow spherical dome with a flat annular rim (skirt).
    /// The dome is a spherical cap of `radius` and `depth` (height of cap
    /// above the base plane). An annular rim of radial width `rim_width`
    /// and thickness `depth` extends outward at the base.
    DishCap {
        id: String,
        radius: Scalar,
        depth: Scalar,
        rim_width: Scalar,
        segments: usize,
    },

    /// AcornShape (new): pointed-top dome — a hemisphere base topped by a
    /// conical spire. The hemisphere of `base_radius` sits with its flat face
    /// at z=0; the cone tip sits at z = base_radius + point_height.
    /// Distinct from the earlier AcornShape (sphere body + stem) — this one
    /// models a pointed acorn cap silhouette.
    AcornShapeDome {
        id: String,
        base_radius: Scalar,
        height: Scalar,
        point_height: Scalar,
        segments: usize,
    },

    /// AcornCap: just the cap of an acorn — a hemisphere with a small
    /// cylindrical ring rim at the bottom edge. Half-sphere at top
    /// (z >= 0), thin rim cylinder at z=0.
    AcornCap {
        id: String,
        cap_radius: Scalar,
        rim_height: Scalar,
        stacks: usize,
        slices: usize,
    },

    /// HourglassFigure: a classic hourglass silhouette built from two
    /// frustums plus disk end-caps. End disks (bottom and top) are
    /// short cylinders of `end_radius`, a frustum tapers from end_radius
    /// down to waist_radius, then a mirrored frustum back up. Avoids the
    /// shared-waist boolean limit of `Hourglass` because the body is built
    /// as two frustums sharing the waist face but with different radii.
    HourglassFigure {
        id: String,
        end_radius: Scalar,
        waist_radius: Scalar,
        body_half_height: Scalar,
        cap_thickness: Scalar,
        cap_radius: Scalar,
        segments: usize,
    },

    /// Ankh: cross with a looped top — a vertical cylinder shaft, a
    /// horizontal cylinder cross-arm at the upper portion of the shaft,
    /// and a faceted torus loop sitting on top of the shaft.
    Ankh {
        id: String,
        shaft_height: Scalar,
        shaft_radius: Scalar,
        arm_length: Scalar,
        arm_radius: Scalar,
        loop_major_radius: Scalar,
        loop_minor_radius: Scalar,
        major_segs: usize,
        minor_segs: usize,
        segments: usize,
    },

    /// CamLobe2: an eccentric (offset) circular cam profile — a single
    /// circle of radius `radius`, but offset from the rotation axis by
    /// `eccentricity`. The bore (rotation axis hole) is at the origin.
    /// Distinct from `CamLobe` (which uses an elliptical profile rx, ry).
    CamLobe2 {
        id: String,
        radius: Scalar,
        eccentricity: Scalar,
        bore_radius: Scalar,
        thickness: Scalar,
        segments: usize,
    },

    /// PistonHead: a stubby cylinder body (the piston shaft section) with
    /// a wider flat crown disk on top. Optionally has `groove_count`
    /// shallow ring grooves around the body. Built as: body cylinder +
    /// crown disk, minus ring cutters for the grooves.
    PistonHead {
        id: String,
        body_radius: Scalar,
        body_height: Scalar,
        crown_radius: Scalar,
        crown_thickness: Scalar,
        groove_count: usize,
        groove_depth: Scalar,
        groove_width: Scalar,
        segments: usize,
    },

    /// PulleyGroove: a cylinder with a true V-shaped groove around its
    /// equator (formed by subtracting two opposing frustums whose
    /// shared waist is the groove valley). Distinct from `Pulley` which
    /// uses a rectangular groove.
    PulleyGroove {
        id: String,
        outer_radius: Scalar,
        groove_inner_radius: Scalar,
        width: Scalar,
        groove_width: Scalar,
        segments: usize,
    },

    /// Pinwheel: a star-shaped extruded plate with a center bore — like
    /// `Star` but explicitly with a hole in the center (for mounting on
    /// an axle). Decorative spinning toy / fan blade silhouette.
    Pinwheel {
        id: String,
        points: usize,
        outer_radius: Scalar,
        inner_radius: Scalar,
        bore_radius: Scalar,
        thickness: Scalar,
        segments: usize,
    },

    /// GearTooth: a single trapezoidal gear-tooth profile, extruded
    /// along +z. Width tapers from `root_width` at z=0 (root) to
    /// `tip_width` at the tip; tooth extends `tooth_height` in +y; thickness
    /// `thickness` in +z. Useful as a building block in compositions
    /// (e.g. arrayed around a gear blank).
    GearTooth {
        id: String,
        root_width: Scalar,
        tip_width: Scalar,
        tooth_height: Scalar,
        thickness: Scalar,
    },

    /// Helix: a helical curve or tubular wire sweep. `axis_radius` is the
    /// distance from the central axis to the curve. `pitch` is the rise per
    /// full turn. `turns` is the number of full turns (must be > 0).
    /// `axis` selects the central axis: "x", "y", or "z" (default "z").
    ///
    /// When `wire_radius` > 0 the helix is materialized as a tubular wire
    /// (like Coil/Spring) built by chaining short cylinder segments along the
    /// helical path. `segments` controls the number of straight-line segments
    /// per turn (minimum 6). When `wire_radius` == 0 the evaluator returns an
    /// error — path-only use is reserved for a future SweepPathHelix feature.
    ///
    /// Unlike Coil, the wire-overlap constraint (`wire_radius < axis_radius`)
    /// is enforced; the axis field enables non-z orientations without a
    /// separate Rotate step.
    Helix {
        id: String,
        axis_radius: Scalar,
        pitch: Scalar,
        turns: Scalar,
        wire_radius: Scalar,
        axis: String,
        segments: usize,
    },

    /// TruncatedSphere: a faceted sphere of `radius` clipped at z = `clip_z`.
    /// Everything above `clip_z` is retained. When `clip_z` = 0 the result
    /// equals a Hemisphere (volume = 2/3 π r³). When `clip_z` = -`radius`
    /// the full sphere is returned. Implemented as sphere_faceted minus a
    /// half-space box cutting z < clip_z.
    TruncatedSphere {
        id: String,
        radius: Scalar,
        clip_z: Scalar,
        segments: usize,
    },

    /// Lens2: biconvex lens shape — intersection of two faceted spheres of
    /// equal `radius`, offset along ±z so the lens has total `thickness`.
    /// The offset d satisfies: at x=y=0, the two sphere surfaces meet at
    /// ±thickness/2, giving d = sqrt(r² - 0) adjusted so that the lens
    /// half-thickness equals thickness/2. Specifically offset = sqrt(r² -
    /// (r - thickness/2)²) along z. When thickness = 2*radius the
    /// intersection is the full sphere (volume = 4/3 π r³).
    Lens2 {
        id: String,
        radius: Scalar,
        thickness: Scalar,
        segments: usize,
    },

    /// Capsule2: pill shape — cylinder body of `radius` and `length` along
    /// the chosen `axis` ("x" | "y" | "z"), capped by two hemispherical
    /// caps of the same `radius`. Volume = π r² · length + 4/3 π r³.
    /// Uses current best-practice idiom (sphere + box clip for each cap,
    /// then union with body cylinder).
    Capsule2 {
        id: String,
        radius: Scalar,
        length: Scalar,
        axis: String,
        segments: usize,
    },

    /// Triangle-soup imported from an external file (e.g. STEP, STL). The
    /// evaluator hands `vertices` and `indices` to
    /// `kerf_brep::from_triangles`, which builds a half-edge solid by
    /// pairing directed edges with their twins.
    ///
    /// `vertices` is a flat list of distinct 3D positions; `indices` is one
    /// `[a, b, c]` per triangle, indexing into `vertices`. Why this and not
    /// a stored `Solid`? Models are JSON-serializable Features; carrying
    /// raw triangle data round-trips through serde cleanly, and the
    /// evaluator reconstructs the B-rep on the fly. Vertex dedup happens
    /// inside `from_triangles`, so callers don't need to dedup themselves.
    ///
    /// Used by `kerf_cad::step_import::import_step` to wrap the imported
    /// solid into a Model.
    ///
    /// **Limitation (v1):** an ImportedMesh has no input dependencies and
    /// cannot currently participate as the input of another feature
    /// (boolean operands, transforms, fillets) because its evaluator
    /// reports an empty dep list. To compose with kerf-native features,
    /// re-author the geometry through the catalog or extend the evaluator
    /// to expose ImportedMesh as a referenceable Solid.
    ImportedMesh {
        id: String,
        vertices: Vec<[f64; 3]>,
        indices: Vec<[usize; 3]>,
    },

    /// OvoidShell: hollow egg-shaped shell. Outer ovoid is built as a
    /// sphere scaled by (1, 1, length / (2*radius_max)) so it spans
    /// `radius_min` along x/y and `length/2` along z. Inner ovoid is the
    /// same shape shrunk inward by `thickness`. The shell is the Difference
    /// of outer minus inner. Very small `thickness` → near-zero volume.
    OvoidShell {
        id: String,
        radius_min: Scalar,
        radius_max: Scalar,
        length: Scalar,
        thickness: Scalar,
        segments: usize,
    },

    /// Bagel: a faceted torus (like `Donut`) with a thin annular dome
    /// on the top surface — like a baked bagel. The base is a standard
    /// faceted torus (`major_radius` × `minor_radius`); the dome is a
    /// narrow flattened toroidal ridge of height `dome_height` sitting on
    /// top of the torus. `major_radius` must exceed `minor_radius`;
    /// `dome_height` must be > 0 and ≤ `minor_radius`. Axis along +z,
    /// centered on the origin.
    Bagel {
        id: String,
        major_radius: Scalar,
        minor_radius: Scalar,
        dome_height: Scalar,
        segments: usize,
    },

    /// Pringle: a saddle-shaped chip — a thin slab whose top surface
    /// follows z = dome_height * (x² − y²) / side². Built as a grid of
    /// points on that quadric surface, extruded downward by a small
    /// thickness (dome_height / 4) to give a solid body. The chip is
    /// `side` × `side` in the xy plane, centred on the origin.
    /// `segments` controls grid resolution (≥ 2).
    Pringle {
        id: String,
        side: Scalar,
        dome_height: Scalar,
        segments: usize,
    },

    /// Cone2: a refined cone with a smooth frustum body and a
    /// hemispherical cap at the top. `tip_radius` = 0 gives a standard
    /// pointed cone topped with a hemisphere of `tip_radius`; a small
    /// `tip_radius` > 0 gives a frustum capped with a hemisphere.
    /// Specifically: a frustum from `base_radius` at z=0 to `tip_radius`
    /// at z=`height`, plus a hemisphere of radius `tip_radius` centred at
    /// (0, 0, `height`). `base_radius` must be > `tip_radius` ≥ 0.
    Cone2 {
        id: String,
        base_radius: Scalar,
        tip_radius: Scalar,
        height: Scalar,
        segments: usize,
    },

    /// Lozenge: a rectangular box with rounded vertical edges — a
    /// discrete "pillowed" cuboid. The box has dimensions `size[0]` × `size[1]`
    /// × `size[2]` (x, y, z). Each of the four vertical (z-axis) edges is
    /// replaced by a cylindrical fillet of radius `corner_radius`. Must
    /// satisfy corner_radius > 0 and corner_radius < min(size[0], size[1]) / 2.
    /// `segments` is the arc resolution per quarter-circle (≥ 2).
    Lozenge {
        id: String,
        size: [Scalar; 3],
        corner_radius: Scalar,
        segments: usize,
    },

    /// Onion: onion-dome shape — hemispherical lower body transitioning via
    /// a frustum collar into a pointed conical spire on top. Evokes
    /// Russian-orthodox / Mughal architectural cupolas.
    Onion {
        id: String,
        base_radius: Scalar,
        mid_height: Scalar,
        point_height: Scalar,
        segments: usize,
    },

    /// WaspWaist: pinched-middle solid built from two opposing frustums
    /// sharing a common narrow waist ring. Wide at top and bottom, minimum
    /// radius at the waist mid-point. When waist_radius equals top_radius
    /// the shape degenerates to a cylinder.
    WaspWaist {
        id: String,
        top_radius: Scalar,
        waist_radius: Scalar,
        total_height: Scalar,
        segments: usize,
    },

    /// Flask: flat-bottomed laboratory flask shape — cylindrical body,
    /// smooth frustum shoulder tapering up, and a thin cylindrical neck.
    /// Models Erlenmeyer / conical-flask silhouettes.
    Flask {
        id: String,
        body_radius: Scalar,
        body_height: Scalar,
        neck_radius: Scalar,
        neck_height: Scalar,
        shoulder_height: Scalar,
        segments: usize,
    },

    /// Pear: a pear-shaped solid with a wide spherical bottom body and a
    /// narrow frustum neck tapering to a small flat top. Inspired by the
    /// natural pear fruit profile.
    Pear {
        id: String,
        body_radius: Scalar,
        neck_radius: Scalar,
        neck_height: Scalar,
        stacks: usize,
        segments: usize,
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
            | Feature::Donut2 { id, .. }
            | Feature::ToroidalCap { id, .. }
            | Feature::EllipticTube { id, .. }
            | Feature::Goblet2 { id, .. }
            | Feature::Cone { id, .. }
            | Feature::Frustum { id, .. }
            | Feature::ExtrudePolygon { id, .. }
            | Feature::Loft { id, .. }
            | Feature::TaperedExtrude { id, .. }
            | Feature::Revolve { id, .. }
            | Feature::SketchExtrude { id, .. }
            | Feature::SketchRevolve { id, .. }
            | Feature::Tube { id, .. }
            | Feature::HollowBox { id, .. }
            | Feature::Shell { id, .. }
            | Feature::CornerCut { id, .. }
            | Feature::Fillet { id, .. }
            | Feature::Fillets { id, .. }
            | Feature::Chamfer { id, .. }
            | Feature::Counterbore { id, .. }
            | Feature::Countersink { id, .. }
            | Feature::EndChamfer { id, .. }
            | Feature::InternalChamfer { id, .. }
            | Feature::ConicalCounterbore { id, .. }
            | Feature::CrossDrilledHole { id, .. }
            | Feature::TaperedPin { id, .. }
            | Feature::FlangedNut { id, .. }
            | Feature::DowelPin { id, .. }
            | Feature::BlindHole { id, .. }
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
            | Feature::TwistedExtrude { id, .. }
            | Feature::HelicalRib { id, .. }
            | Feature::ScrewThread { id, .. }
            | Feature::SpiralWedge { id, .. }
            | Feature::DoubleHelix { id, .. }
            | Feature::TaperedCoil { id, .. }
            | Feature::SweepProfile { id, .. }
            | Feature::LoftMulti { id, .. }
            | Feature::SweepWithTwist { id, .. }
            | Feature::SweepWithScale { id, .. }
            | Feature::HelicalThread { id, .. }
            | Feature::TwistedTube { id, .. }
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
            | Feature::SteppedShaft { id, .. }
            | Feature::Trough { id, .. }
            | Feature::Knob { id, .. }
            | Feature::Plug { id, .. }
            | Feature::Spike { id, .. }
            | Feature::CapBolt { id, .. }
            | Feature::FlangedBolt { id, .. }
            | Feature::SerratedDisk { id, .. }
            | Feature::FerruleEnd { id, .. }
            | Feature::Trapezoid { id, .. }
            | Feature::Handle { id, .. }
            | Feature::HookHandle { id, .. }
            | Feature::CornerBracket { id, .. }
            | Feature::ArcSegment { id, .. }
            | Feature::CrossBrace { id, .. }
            | Feature::WireMesh { id, .. }
            | Feature::AnchorPoint { id, .. }
            | Feature::Stair { id, .. }
            | Feature::Hopper { id, .. }
            | Feature::Stand { id, .. }
            | Feature::Yoke { id, .. }
            | Feature::Lever { id, .. }
            | Feature::TaperedTube { id, .. }
            | Feature::GussetPlate { id, .. }
            | Feature::CrossKey { id, .. }
            | Feature::PinShaft { id, .. }
            | Feature::Lens { id, .. }
            | Feature::EggShape { id, .. }
            | Feature::UBendPipe { id, .. }
            | Feature::SBend { id, .. }
            | Feature::DonutSlice { id, .. }
            | Feature::CapsuleAt { id, .. }
            | Feature::ToroidalKnob { id, .. }
            | Feature::Cup { id, .. }
            | Feature::Bottle { id, .. }
            | Feature::TableLeg { id, .. }
            | Feature::ChairLeg { id, .. }
            | Feature::Bookshelf { id, .. }
            | Feature::PlanterBox { id, .. }
            | Feature::DrawerSlot { id, .. }
            | Feature::CircularRing { id, .. }
            | Feature::PolygonRing { id, .. }
            | Feature::CylinderShellAt { id, .. }
            | Feature::QuarterTorus { id, .. }
            | Feature::HalfTorus { id, .. }
            | Feature::SquareTube { id, .. }
            | Feature::HoleyPlate { id, .. }
            | Feature::ScrewBoss { id, .. }
            | Feature::Brick { id, .. }
            | Feature::CorrugatedPanel { id, .. }
            | Feature::BeltLoop { id, .. }
            | Feature::Stake { id, .. }
            | Feature::Bipyramid { id, .. }
            | Feature::Antiprism { id, .. }
            | Feature::CableSaddle { id, .. }
            | Feature::Slot3D { id, .. }
            | Feature::OvalPlate { id, .. }
            | Feature::AsymmetricBracket { id, .. }
            | Feature::EndCap { id, .. }
            | Feature::RatchetTooth { id, .. }
            | Feature::BasePlate { id, .. }
            | Feature::FunnelTube { id, .. }
            | Feature::FlatWasher { id, .. }
            | Feature::RibbedPlate { id, .. }
            | Feature::Pediment { id, .. }
            | Feature::Vault { id, .. }
            | Feature::ShelfBracket { id, .. }
            | Feature::NameTag { id, .. }
            | Feature::Plinth { id, .. }
            | Feature::ParapetWall { id, .. }
            | Feature::BeamWithHoles { id, .. }
            | Feature::Ellipsoid3D { id, .. }
            | Feature::VectorArrow { id, .. }
            | Feature::BoneShape { id, .. }
            | Feature::Pawn { id, .. }
            | Feature::Rook { id, .. }
            | Feature::Bishop { id, .. }
            | Feature::Marker3D { id, .. }
            | Feature::HollowBrick { id, .. }
            | Feature::StadiumPlate { id, .. }
            | Feature::Bowtie { id, .. }
            | Feature::HollowCone { id, .. }
            | Feature::ArchedDoorway { id, .. }
            | Feature::CamLobe { id, .. }
            | Feature::ButtonShape { id, .. }
            | Feature::FilletedSlot { id, .. }
            | Feature::CoinShape { id, .. }
            | Feature::CylindricalCap { id, .. }
            | Feature::SquaredRing { id, .. }
            | Feature::WaveProfile { id, .. }
            | Feature::BulletShape { id, .. }
            | Feature::TriangularPlate { id, .. }
            | Feature::Heart { id, .. }
            | Feature::ChainLink { id, .. }
            | Feature::SpiralPlate { id, .. }
            | Feature::WindowFrame { id, .. }
            | Feature::SquareKey { id, .. }
            | Feature::DiskWithSlots { id, .. }
            | Feature::FivePointedBadge { id, .. }
            | Feature::Crescent { id, .. }
            | Feature::Hourglass { id, .. }
            | Feature::Diabolo { id, .. }
            | Feature::TripleStep { id, .. }
            | Feature::WingedScrew { id, .. }
            | Feature::KneadHandle { id, .. }
            | Feature::ZigzagBar { id, .. }
            | Feature::FishingFloat { id, .. }
            | Feature::Tetrahedron { id, .. }
            | Feature::Spool { id, .. }
            | Feature::Lampshade { id, .. }
            | Feature::PrismHole { id, .. }
            | Feature::KeyholeShape { id, .. }
            | Feature::AcornShape { id, .. }
            | Feature::Mushroom { id, .. }
            | Feature::Lightbulb { id, .. }
            | Feature::DomedRoof { id, .. }
            | Feature::Beehive { id, .. }
            | Feature::Bullet { id, .. }
            | Feature::PointedDome { id, .. }
            | Feature::WindBell { id, .. }
            | Feature::PineCone { id, .. }
            | Feature::TopHat { id, .. }
            | Feature::WaterTower { id, .. }
            | Feature::PlantPot { id, .. }
            | Feature::Buoy { id, .. }
            | Feature::Trophy { id, .. }
            | Feature::Goblet { id, .. }
            | Feature::TableLamp { id, .. }
            | Feature::MushroomCloud { id, .. }
            | Feature::TieredCake { id, .. }
            | Feature::Spindle { id, .. }
            | Feature::Ufo { id, .. }
            | Feature::CrowsNest { id, .. }
            | Feature::Volute { id, .. }
            | Feature::ScrollPlate { id, .. }
            | Feature::TulipBulb { id, .. }
            | Feature::PaperLantern { id, .. }
            | Feature::PaperLanternStrips { id, .. }
            | Feature::Trefoil { id, .. }
            | Feature::DishCap { id, .. }
            | Feature::AcornShapeDome { id, .. }
            | Feature::AcornCap { id, .. }
            | Feature::HourglassFigure { id, .. }
            | Feature::Ankh { id, .. }
            | Feature::CamLobe2 { id, .. }
            | Feature::PistonHead { id, .. }
            | Feature::PulleyGroove { id, .. }
            | Feature::Pinwheel { id, .. }
            | Feature::GearTooth { id, .. }
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
            | Feature::Funnel2 { id, .. }
            | Feature::CrossPipe { id, .. }
            | Feature::AnchorChain { id, .. }
            | Feature::GearBlank2 { id, .. }
            | Feature::PaperClipShape { id, .. }
            | Feature::Caltrops { id, .. }
            | Feature::Helix { id, .. }
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
        
            Feature::MidPlaneRef { id, .. }
            | Feature::PerpRefPlane { id, .. }
            | Feature::OffsetRefPlane { id, .. }
            | Feature::CoordinateAxes { id, .. }
            | Feature::OriginPoint { id, .. }
            | Feature::CenterMarker { id, .. }
            | Feature::AxisLabel { id, .. }
            | Feature::DistanceMarker { id, .. }
            | Feature::CenterDrill { id, .. }
            | Feature::OilHole { id, .. }
            | Feature::ReliefCut { id, .. }
            | Feature::WrenchFlats { id, .. }
            | Feature::Knurl { id, .. }
=> id,

            Feature::TableTop { id, .. }
            | Feature::Bench { id, .. }
            | Feature::WindowLouver { id, .. }
            | Feature::Hammer { id, .. }
            | Feature::ScrewDriver { id, .. }
            | Feature::Wrench { id, .. }
            | Feature::Heart3D { id, .. }
            | Feature::Star3D { id, .. }
            | Feature::Cross3D { id, .. }
            | Feature::Chair { id, .. }
            | Feature::ImportedMesh { id, .. }
            | Feature::TruncatedSphere { id, .. }
            | Feature::Lens2 { id, .. }
            | Feature::Capsule2 { id, .. }
            | Feature::OvoidShell { id, .. }
            | Feature::Bagel { id, .. }
            | Feature::Pringle { id, .. }
            | Feature::Cone2 { id, .. }
            | Feature::Lozenge { id, .. }
            | Feature::Onion { id, .. }
            | Feature::WaspWaist { id, .. }
            | Feature::Flask { id, .. }
            | Feature::Pear { id, .. }
=> id,

            Feature::ChamferedHole { id, .. }
            | Feature::ThreadedHoleMarker { id, .. }
            | Feature::BoltPattern { id, .. }
            | Feature::SquareDrive { id, .. }
            | Feature::RaisedBoss { id, .. }
=> id,
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
            | Feature::Donut2 { .. }
            | Feature::ToroidalCap { .. }
            | Feature::EllipticTube { .. }
            | Feature::Goblet2 { .. }
            | Feature::Cone { .. }
            | Feature::Frustum { .. }
            | Feature::ExtrudePolygon { .. }
            | Feature::Loft { .. }
            | Feature::TaperedExtrude { .. }
            | Feature::Revolve { .. }
            | Feature::SketchExtrude { .. }
            | Feature::SketchRevolve { .. }
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
            | Feature::TwistedExtrude { .. }
            | Feature::HelicalRib { .. }
            | Feature::ScrewThread { .. }
            | Feature::SpiralWedge { .. }
            | Feature::DoubleHelix { .. }
            | Feature::TaperedCoil { .. }
            | Feature::SweepProfile { .. }
            | Feature::LoftMulti { .. }
            | Feature::SweepWithTwist { .. }
            | Feature::SweepWithScale { .. }
            | Feature::HelicalThread { .. }
            | Feature::TwistedTube { .. }
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
            | Feature::SteppedShaft { .. }
            | Feature::Trough { .. }
            | Feature::Knob { .. }
            | Feature::Plug { .. }
            | Feature::Spike { .. }
            | Feature::CapBolt { .. }
            | Feature::FlangedBolt { .. }
            | Feature::SerratedDisk { .. }
            | Feature::FerruleEnd { .. }
            | Feature::Trapezoid { .. }
            | Feature::Handle { .. }
            | Feature::HookHandle { .. }
            | Feature::CornerBracket { .. }
            | Feature::ArcSegment { .. }
            | Feature::CrossBrace { .. }
            | Feature::WireMesh { .. }
            | Feature::AnchorPoint { .. }
            | Feature::Stair { .. }
            | Feature::Hopper { .. }
            | Feature::Stand { .. }
            | Feature::Yoke { .. }
            | Feature::Lever { .. }
            | Feature::TaperedTube { .. }
            | Feature::GussetPlate { .. }
            | Feature::CrossKey { .. }
            | Feature::PinShaft { .. }
            | Feature::Lens { .. }
            | Feature::EggShape { .. }
            | Feature::UBendPipe { .. }
            | Feature::SBend { .. }
            | Feature::DonutSlice { .. }
            | Feature::CapsuleAt { .. }
            | Feature::ToroidalKnob { .. }
            | Feature::Cup { .. }
            | Feature::Bottle { .. }
            | Feature::TableLeg { .. }
            | Feature::ChairLeg { .. }
            | Feature::Bookshelf { .. }
            | Feature::PlanterBox { .. }
            | Feature::DrawerSlot { .. }
            | Feature::CircularRing { .. }
            | Feature::PolygonRing { .. }
            | Feature::CylinderShellAt { .. }
            | Feature::QuarterTorus { .. }
            | Feature::HalfTorus { .. }
            | Feature::SquareTube { .. }
            | Feature::HoleyPlate { .. }
            | Feature::ScrewBoss { .. }
            | Feature::Brick { .. }
            | Feature::CorrugatedPanel { .. }
            | Feature::BeltLoop { .. }
            | Feature::Stake { .. }
            | Feature::Bipyramid { .. }
            | Feature::Antiprism { .. }
            | Feature::CableSaddle { .. }
            | Feature::Slot3D { .. }
            | Feature::OvalPlate { .. }
            | Feature::AsymmetricBracket { .. }
            | Feature::EndCap { .. }
            | Feature::RatchetTooth { .. }
            | Feature::BasePlate { .. }
            | Feature::FunnelTube { .. }
            | Feature::FlatWasher { .. }
            | Feature::RibbedPlate { .. }
            | Feature::Pediment { .. }
            | Feature::Vault { .. }
            | Feature::ShelfBracket { .. }
            | Feature::NameTag { .. }
            | Feature::Plinth { .. }
            | Feature::ParapetWall { .. }
            | Feature::BeamWithHoles { .. }
            | Feature::Ellipsoid3D { .. }
            | Feature::VectorArrow { .. }
            | Feature::BoneShape { .. }
            | Feature::Pawn { .. }
            | Feature::Rook { .. }
            | Feature::Bishop { .. }
            | Feature::Marker3D { .. }
            | Feature::HollowBrick { .. }
            | Feature::StadiumPlate { .. }
            | Feature::Bowtie { .. }
            | Feature::HollowCone { .. }
            | Feature::ArchedDoorway { .. }
            | Feature::CamLobe { .. }
            | Feature::ButtonShape { .. }
            | Feature::FilletedSlot { .. }
            | Feature::CoinShape { .. }
            | Feature::CylindricalCap { .. }
            | Feature::SquaredRing { .. }
            | Feature::WaveProfile { .. }
            | Feature::BulletShape { .. }
            | Feature::TriangularPlate { .. }
            | Feature::Heart { .. }
            | Feature::ChainLink { .. }
            | Feature::SpiralPlate { .. }
            | Feature::WindowFrame { .. }
            | Feature::SquareKey { .. }
            | Feature::DiskWithSlots { .. }
            | Feature::FivePointedBadge { .. }
            | Feature::Crescent { .. }
            | Feature::Hourglass { .. }
            | Feature::Diabolo { .. }
            | Feature::TripleStep { .. }
            | Feature::WingedScrew { .. }
            | Feature::KneadHandle { .. }
            | Feature::ZigzagBar { .. }
            | Feature::FishingFloat { .. }
            | Feature::Tetrahedron { .. }
            | Feature::Spool { .. }
            | Feature::Lampshade { .. }
            | Feature::PrismHole { .. }
            | Feature::KeyholeShape { .. }
            | Feature::AcornShape { .. }
            | Feature::Mushroom { .. }
            | Feature::Lightbulb { .. }
            | Feature::DomedRoof { .. }
            | Feature::Beehive { .. }
            | Feature::Bullet { .. }
            | Feature::PointedDome { .. }
            | Feature::WindBell { .. }
            | Feature::PineCone { .. }
            | Feature::TopHat { .. }
            | Feature::WaterTower { .. }
            | Feature::PlantPot { .. }
            | Feature::Buoy { .. }
            | Feature::Trophy { .. }
            | Feature::Goblet { .. }
            | Feature::TableLamp { .. }
            | Feature::MushroomCloud { .. }
            | Feature::TieredCake { .. }
            | Feature::Spindle { .. }
            | Feature::Ufo { .. }
            | Feature::CrowsNest { .. }
            | Feature::Volute { .. }
            | Feature::ScrollPlate { .. }
            | Feature::TulipBulb { .. }
            | Feature::PaperLantern { .. }
            | Feature::PaperLanternStrips { .. }
            | Feature::Trefoil { .. }
            | Feature::DishCap { .. }
            | Feature::AcornShapeDome { .. }
            | Feature::AcornCap { .. }
            | Feature::HourglassFigure { .. }
            | Feature::Ankh { .. }
            | Feature::CamLobe2 { .. }
            | Feature::PistonHead { .. }
            | Feature::PulleyGroove { .. }
            | Feature::Pinwheel { .. }
            | Feature::GearTooth { .. }
            | Feature::DovetailSlot { .. }
            | Feature::VeeGroove { .. }
            | Feature::Bolt { .. }
            | Feature::CapScrew { .. }
            | Feature::Nut { .. }
            | Feature::Washer { .. }
            | Feature::RoundBoss { .. }
            | Feature::RectBoss { .. }
            | Feature::Funnel2 { .. }
            | Feature::CrossPipe { .. }
            | Feature::AnchorChain { .. }
            | Feature::GearBlank2 { .. }
            | Feature::PaperClipShape { .. }
            | Feature::Caltrops { .. }
            | Feature::Helix { .. } => Vec::new(),
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
            | Feature::Countersink { input, .. }
            | Feature::Shell { input, .. } => {
                vec![input.as_str()]
            }
            Feature::Union { inputs, .. }
            | Feature::Intersection { inputs, .. }
            | Feature::Difference { inputs, .. } => inputs.iter().map(String::as_str).collect(),
        
            Feature::TaperedPin { .. }
            | Feature::FlangedNut { .. }
            | Feature::DowelPin { .. }
            | Feature::TruncatedSphere { .. }
            | Feature::Lens2 { .. }
            | Feature::Capsule2 { .. }
            | Feature::OvoidShell { .. } => Vec::new(),
            Feature::EndChamfer { input, .. }
            | Feature::InternalChamfer { input, .. }
            | Feature::ConicalCounterbore { input, .. }
            | Feature::CrossDrilledHole { input, .. }
            | Feature::BlindHole { input, .. } => {
                vec![input.as_str()]
            }

            Feature::MidPlaneRef { .. }
            | Feature::PerpRefPlane { .. }
            | Feature::OffsetRefPlane { .. }
            | Feature::CoordinateAxes { .. }
            | Feature::OriginPoint { .. }
            | Feature::CenterMarker { .. }
            | Feature::AxisLabel { .. }
            | Feature::DistanceMarker { .. }
            | Feature::CenterDrill { .. }
            | Feature::OilHole { .. }
            | Feature::ReliefCut { .. }
            | Feature::WrenchFlats { .. }
            | Feature::Knurl { .. } => Vec::new(),

            Feature::TableTop { .. }
            | Feature::Bench { .. }
            | Feature::WindowLouver { .. }
            | Feature::Hammer { .. }
            | Feature::ScrewDriver { .. }
            | Feature::Wrench { .. }
            | Feature::Heart3D { .. }
            | Feature::Star3D { .. }
            | Feature::Cross3D { .. }
            | Feature::Chair { .. }
            | Feature::ImportedMesh { .. }
            | Feature::Bagel { .. }
            | Feature::Pringle { .. }
            | Feature::Cone2 { .. }
            | Feature::Lozenge { .. }
            | Feature::Onion { .. }
            | Feature::WaspWaist { .. }
            | Feature::Flask { .. }
            | Feature::Pear { .. }
=> Vec::new(),

            Feature::ChamferedHole { input, .. }
            | Feature::ThreadedHoleMarker { input, .. }
            | Feature::BoltPattern { input, .. }
            | Feature::SquareDrive { input, .. }
            | Feature::RaisedBoss { input, .. } => vec![input.as_str()],
}
    }
}
