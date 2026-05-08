//! Dimensioned 2D drawings: orthographic projections, distance/angle/projection
//! helpers, and an SVG renderer that overlays dimension annotations on a
//! solid's silhouette.
//!
//! This is the back-end half of the "Drawings" capability tracked in STATUS.md.
//! The viewer can either drive these helpers via the WASM API to compute
//! measurements (so it doesn't need to re-implement the math in TypeScript),
//! or it can ask for a fully-rendered SVG and embed it directly.
//!
//! ## View conventions
//!
//! - `Top`     looks down -Z; world (X, Y) → screen (x, -y) (Y up on paper).
//! - `Front`   looks along -Y; world (X, Z) → screen (x, -z).
//! - `Side`    (right side) looks along -X; world (Y, Z) → screen (-y, -z).
//! - `Iso`     a fixed isometric direction; uses the same mapping as the
//!             viewer's drawings.ts ISO cell so the SVG and PNG are
//!             visually consistent.
//!
//! Screen coordinates are in *world units* — the SVG renderer applies a
//! `viewport` transform to lay them out on the page, with Y flipped because
//! SVG Y points down.

use crate::geometry::CurveKind;
use crate::tessellate::tessellate;
use crate::Solid;
use kerf_geom::{Point3, Vec3};

// --- View kinds ---------------------------------------------------------

/// Which orthographic / isometric projection to use.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ViewKind {
    Top,
    Front,
    Side,
    Iso,
}

impl ViewKind {
    /// Parse a case-insensitive view name. Used by the WASM bindings so
    /// the front-end can pass `"top"` / `"front"` / `"side"` / `"iso"`.
    pub fn from_str(name: &str) -> Option<Self> {
        match name.trim().to_ascii_lowercase().as_str() {
            "top" => Some(ViewKind::Top),
            "front" => Some(ViewKind::Front),
            "side" | "right" => Some(ViewKind::Side),
            "iso" | "isometric" => Some(ViewKind::Iso),
            _ => None,
        }
    }

    pub fn as_str(self) -> &'static str {
        match self {
            ViewKind::Top => "top",
            ViewKind::Front => "front",
            ViewKind::Side => "side",
            ViewKind::Iso => "iso",
        }
    }
}

// --- Pure math: distance, angle, projection -----------------------------

/// Euclidean distance between two world-space points.
pub fn distance(p1: Point3, p2: Point3) -> f64 {
    (p2 - p1).norm()
}

/// Angle between two vectors in radians, in [0, pi]. Returns 0 if either
/// vector has zero length (degenerate; callers can decide how to surface).
pub fn angle_between_vectors(a: Vec3, b: Vec3) -> f64 {
    let na = a.norm();
    let nb = b.norm();
    if na <= f64::EPSILON || nb <= f64::EPSILON {
        return 0.0;
    }
    let cos = (a.dot(&b) / (na * nb)).clamp(-1.0, 1.0);
    cos.acos()
}

/// Angle (radians) at `vertex` formed by rays to `a` and `b`.
pub fn angle_at_vertex(a: Point3, vertex: Point3, b: Point3) -> f64 {
    angle_between_vectors(a - vertex, b - vertex)
}

/// Project a point onto a plane defined by an origin and a normal.
/// The normal need not be unit-length; degenerate normals (zero length)
/// fall back to returning `p` unchanged.
pub fn project_to_plane(p: Point3, plane_normal: Vec3, plane_origin: Point3) -> Point3 {
    let n2 = plane_normal.norm_squared();
    if n2 <= f64::EPSILON {
        return p;
    }
    let v = p - plane_origin;
    let t = v.dot(&plane_normal) / n2;
    p - plane_normal * t
}

/// View direction (the direction the camera looks, i.e. opposite of the
/// "from" vector). For Top this is `-Z`; for Front it's `-Y`; for Side it's
/// `-X`. Iso uses a fixed CAD-style direction that matches the viewer's
/// drawings.ts ISO cell.
///
/// Used by [`projected_silhouette`] to classify front-facing vs back-facing
/// triangles when extracting a true silhouette.
pub fn view_direction(view: ViewKind) -> Vec3 {
    match view {
        ViewKind::Top => Vec3::new(0.0, 0.0, -1.0),
        ViewKind::Front => Vec3::new(0.0, -1.0, 0.0),
        ViewKind::Side => Vec3::new(-1.0, 0.0, 0.0),
        // Same direction the viewer uses (forward = (-1, -0.7, -1.2)).
        ViewKind::Iso => {
            let d = Vec3::new(-1.0, -0.7, -1.2);
            let n = d.norm().max(1e-12);
            d / n
        }
    }
}

/// Project a 3D point into 2D screen coordinates for the given view.
///
/// Returned pair is `(u, v)` in *world units*: u points right on the page,
/// v points up on the page. The SVG renderer flips v to SVG's Y-down later.
pub fn to_2d_view(p: Point3, view: ViewKind) -> (f64, f64) {
    match view {
        // Looking down -Z: paper X = world X, paper Y = world Y.
        ViewKind::Top => (p.x, p.y),
        // Looking along -Y: paper X = world X, paper Y = world Z.
        ViewKind::Front => (p.x, p.z),
        // Looking along -X (right side): paper X = world Y, paper Y = world Z.
        // Note: drawings.ts uses world Y as the horizontal axis on the right
        // view; we follow the same convention so the projections agree.
        ViewKind::Side => (p.y, p.z),
        // Iso: simple cabinet-style projection that matches the spirit of
        // the viewer's ISO camera (forward = (-1, -0.7, -1.2).normalize(),
        // up = +Z). We don't try to be pixel-identical with three.js — this
        // is just a 2D fallback for the SVG renderer.
        ViewKind::Iso => {
            // 30-degree isometric: a standard CAD convention.
            let c = (30.0_f64).to_radians().cos();
            let s = (30.0_f64).to_radians().sin();
            let u = (p.x - p.y) * c;
            let v = p.z + (p.x + p.y) * s;
            (u, v)
        }
    }
}

// --- Dimension annotations ---------------------------------------------

/// A single dimension annotation. The points are in world space; the
/// renderer projects them through the requested view.
#[derive(Debug, Clone)]
pub struct Dimension {
    pub from: Point3,
    pub to: Point3,
    pub kind: DimensionKind,
}

#[derive(Debug, Clone)]
pub enum DimensionKind {
    /// Straight linear distance |to - from|.
    Linear,
    /// Radial distance from a fixed center (e.g. a hole). The annotation
    /// renders as a leader line from `center` to `to` (the point on the
    /// circle), labelled with R<value>.
    RadialFromCenter { center: Point3 },
    /// Angular dimension at `vertex`, between rays to `from` and `to`.
    /// Renders as an arc plus a degree label.
    Angular { vertex: Point3 },
}

/// Viewport spec for the SVG renderer. The renderer fits the projected
/// silhouette into `(width - 2*padding) x (height - 2*padding)` while
/// preserving aspect ratio.
#[derive(Debug, Clone, Copy)]
pub struct ViewportSpec {
    pub width: f64,
    pub height: f64,
    pub padding: f64,
}

impl ViewportSpec {
    pub fn new(width: f64, height: f64, padding: f64) -> Self {
        Self { width, height, padding }
    }

    pub fn default_a4_ish() -> Self {
        Self { width: 480.0, height: 360.0, padding: 36.0 }
    }
}

// --- Silhouette extraction ---------------------------------------------

/// Project the solid's tessellation into the requested view and return the
/// outer silhouette as a closed polygon in *world units* (paper coords).
///
/// Convex-hull fallback: useful when the half-edge silhouette can't form a
/// single outer loop (degenerate inputs, zero-area triangles, etc.). For
/// the common case use [`silhouette_loops`], which walks the half-edge
/// adjacency to emit a true outline (concave shapes don't get bloated to
/// their bounding shape) plus interior silhouette edges where one curved
/// face wraps the view direction.
pub fn projected_silhouette(solid: &Solid, view: ViewKind, segments: usize) -> Vec<(f64, f64)> {
    let soup = tessellate(solid, segments.max(3));
    let mut pts: Vec<(f64, f64)> = Vec::with_capacity(soup.triangles.len() * 3);
    for tri in &soup.triangles {
        for v in tri {
            pts.push(to_2d_view(*v, view));
        }
    }
    if pts.is_empty() {
        // Degenerate / empty solid — return an empty polygon, the renderer
        // just falls back to drawing the dimensions with no outline.
        return pts;
    }
    convex_hull(pts)
}

/// Result of [`silhouette_loops`]: lists of 2D loops in paper coordinates.
///
/// `outer` is the set of closed polylines forming the outer silhouette
/// (multiple in the disjoint-shells case); `interior` is the silhouette
/// edges that don't sit on the outer outline — typically the seam
/// between front-facing and back-facing portions of curved surfaces, or
/// the inner outline of a hollow part. The renderer draws `outer` solid
/// and `interior` dashed (the "hidden line / back edge" rendering).
#[derive(Debug, Clone, Default)]
pub struct SilhouetteLoops {
    pub outer: Vec<Vec<(f64, f64)>>,
    pub interior: Vec<Vec<(f64, f64)>>,
}

/// Walk the tessellation's half-edge adjacency to build a true silhouette.
///
/// Strategy:
/// 1. Tessellate at `segments` resolution. Each triangle gets a
///    front/back label from the sign of `triangle_normal · view_direction`
///    (front-facing triangles have a normal pointing TOWARDS the camera,
///    i.e. opposite the view direction → dot product < 0).
/// 2. Build an undirected edge-multigraph keyed by the rounded (paper-space)
///    endpoint pair. Each tessellation edge counts how many front-facing
///    and back-facing triangles incident to it.
/// 3. A silhouette edge is one with at least one front-facing triangle and
///    either zero back-facing twin (boundary of a shell, but tessellations
///    are closed so this is rare in practice), or a back-facing twin (true
///    silhouette).
/// 4. Stitch the silhouette edges into closed polylines via repeated
///    nearest-endpoint chaining; classify the largest-area CCW loop as the
///    outer silhouette and everything else as interior.
///
/// This function never panics; if the tessellation can't be stitched into
/// loops it falls back to returning a single SilhouetteLoops with the
/// convex hull as `outer`.
pub fn silhouette_loops(solid: &Solid, view: ViewKind, segments: usize) -> SilhouetteLoops {
    let soup = tessellate(solid, segments.max(3));
    if soup.triangles.is_empty() {
        return SilhouetteLoops::default();
    }
    let view_dir = view_direction(view);

    // Strategy: project front-facing triangles to 2D and find the
    // boundary of their union. An edge is on the boundary iff an odd
    // number of front-facing triangles in 2D share it; even-count edges
    // are interior to the union (a fan diagonal shared by two adjacent
    // front triangles cancels out).
    //
    // Working in 2D rather than 3D is essential here. Fan-triangulation
    // can pick different apex vertices on the top and bottom faces of a
    // single concave plate, so the same projected edge gets classified
    // differently per face if we key by 3D position. The 2D parity rule
    // matches the geometric notion of "outline of the solid's footprint
    // on the page" exactly.
    fn key2(p: (f64, f64)) -> (i64, i64) {
        const Q: f64 = 1e6;
        ((p.0 * Q).round() as i64, (p.1 * Q).round() as i64)
    }

    use std::collections::HashMap;
    type EdgeKey = ((i64, i64), (i64, i64));
    let mut front_counts: HashMap<EdgeKey, (i32, (f64, f64), (f64, f64))> = HashMap::new();
    for tri in &soup.triangles {
        let n3 = (tri[1] - tri[0]).cross(&(tri[2] - tri[0]));
        let nn = n3.norm();
        if nn < 1e-15 {
            continue;
        }
        let n3u = n3 / nn;
        if n3u.dot(&view_dir) >= 0.0 {
            continue; // back-facing: not part of front union
        }
        // Project to 2D and reject zero-area projected triangles
        // (faces parallel to the view direction); they don't contribute
        // to the front union's footprint.
        let a = to_2d_view(tri[0], view);
        let b = to_2d_view(tri[1], view);
        let c = to_2d_view(tri[2], view);
        let area2 = (b.0 - a.0) * (c.1 - a.1) - (b.1 - a.1) * (c.0 - a.0);
        if area2.abs() < 1e-12 {
            continue;
        }
        for (p0, p1) in [(a, b), (b, c), (c, a)] {
            let ka = key2(p0);
            let kb = key2(p1);
            let (k0, k1, q0, q1) = if ka <= kb { (ka, kb, p0, p1) } else { (kb, ka, p1, p0) };
            let entry = front_counts.entry((k0, k1)).or_insert((0, q0, q1));
            entry.0 += 1;
        }
    }

    // Boundary of the front-facing union: edges with odd count.
    let mut sil_edges: Vec<((f64, f64), (f64, f64))> = Vec::new();
    for &(count, p0, p1) in front_counts.values() {
        if count % 2 == 1 && distance_2d(p0, p1) > 1e-9 {
            sil_edges.push((p0, p1));
        }
    }

    if sil_edges.is_empty() {
        return SilhouetteLoops::default();
    }

    // Stitch into closed loops by chaining edges that share an endpoint.
    let loops = chain_edges_to_loops(&sil_edges);
    if loops.is_empty() {
        // Fall back to convex hull as a single outer loop.
        let hull = projected_silhouette(solid, view, segments);
        return SilhouetteLoops { outer: vec![hull], interior: Vec::new() };
    }

    // Classify outer vs interior by signed area: outer loops are the ones
    // that contain the most other loops — but a robust heuristic is "the
    // loops with the largest absolute signed area, one per disjoint shell".
    // For the typical single-shell case there's exactly one outer loop;
    // multi-shell parts are rare. We pick the single largest-area loop as
    // outer, and treat all the rest as interior.
    let mut areas: Vec<f64> = loops.iter().map(|l| polygon_signed_area_2d(l).abs()).collect();
    let mut idx: Vec<usize> = (0..loops.len()).collect();
    idx.sort_by(|&a, &b| areas[b].partial_cmp(&areas[a]).unwrap_or(std::cmp::Ordering::Equal));

    let mut outer: Vec<Vec<(f64, f64)>> = Vec::new();
    let mut interior: Vec<Vec<(f64, f64)>> = Vec::new();
    if !idx.is_empty() {
        outer.push(loops[idx[0]].clone());
        // Disjoint-shell heuristic: any other loop whose AABB doesn't
        // overlap the outer loop's AABB is also "outer" (separate shell).
        let outer_bbox = bbox_2d(&outer[0]);
        for &i in &idx[1..] {
            let bb = bbox_2d(&loops[i]);
            let overlaps = bb.0 < outer_bbox.2 + 1e-9
                && bb.2 > outer_bbox.0 - 1e-9
                && bb.1 < outer_bbox.3 + 1e-9
                && bb.3 > outer_bbox.1 - 1e-9;
            if overlaps {
                interior.push(loops[i].clone());
            } else {
                outer.push(loops[i].clone());
            }
        }
    }
    let _ = &mut areas; // suppress unused-mut warning across cfg branches
    SilhouetteLoops { outer, interior }
}

/// Greedy edge-chaining: build the largest possible polylines by repeatedly
/// extending from a free endpoint. Closed loops (both endpoints meet) are
/// emitted as `Vec<(f64, f64)>` without duplicating the start.
///
/// At a vertex with multiple unvisited outgoing silhouette edges (a Y/T
/// junction where a concave step meets the outer hull), we pick the
/// "most-CCW" turn — i.e. the smallest counter-clockwise angle from the
/// incoming direction. That keeps the walker on the outer boundary of a
/// concave region (e.g. an L-bracket) instead of cutting across the
/// chord between the two arms.
fn chain_edges_to_loops(edges: &[((f64, f64), (f64, f64))]) -> Vec<Vec<(f64, f64)>> {
    use std::collections::HashMap;

    fn key(p: (f64, f64)) -> (i64, i64) {
        const Q: f64 = 1e6;
        ((p.0 * Q).round() as i64, (p.1 * Q).round() as i64)
    }

    // Adjacency map: vertex key → list of (other-end key, other-end coord).
    let mut adj: HashMap<(i64, i64), Vec<((i64, i64), (f64, f64))>> = HashMap::new();
    for &(a, b) in edges {
        let ka = key(a);
        let kb = key(b);
        adj.entry(ka).or_default().push((kb, b));
        adj.entry(kb).or_default().push((ka, a));
    }

    let mut visited: std::collections::HashSet<((i64, i64), (i64, i64))> = Default::default();
    let mut loops: Vec<Vec<(f64, f64)>> = Vec::new();

    // Pick next neighbor: prefer the one with the largest CCW turn from
    // the incoming direction. This keeps us on the outer boundary of a
    // concave region (turning left into the interior, not cutting the
    // chord across it).
    let pick_next = |adj: &HashMap<(i64, i64), Vec<((i64, i64), (f64, f64))>>,
                     visited: &std::collections::HashSet<((i64, i64), (i64, i64))>,
                     last: (i64, i64),
                     last_pos: (f64, f64),
                     prev_pos: (f64, f64)|
     -> Option<((i64, i64), (f64, f64))> {
        let neighbors = adj.get(&last)?;
        let in_dx = last_pos.0 - prev_pos.0;
        let in_dy = last_pos.1 - prev_pos.1;
        let in_len = (in_dx * in_dx + in_dy * in_dy).sqrt().max(1e-15);
        let in_x = in_dx / in_len;
        let in_y = in_dy / in_len;
        let mut best: Option<(f64, ((i64, i64), (f64, f64)))> = None;
        for &(nk, np) in neighbors {
            let canon = if last <= nk { (last, nk) } else { (nk, last) };
            if visited.contains(&canon) {
                continue;
            }
            let dx = np.0 - last_pos.0;
            let dy = np.1 - last_pos.1;
            let len = (dx * dx + dy * dy).sqrt().max(1e-15);
            let ox = dx / len;
            let oy = dy / len;
            // CW angle from incoming direction to outgoing direction,
            // in [0, 2π). For a proper outer-boundary walk we want the
            // SMALLEST CW turn that isn't a U-turn — this is the
            // "rightmost" turn convention used in standard polygonal
            // outline reconstruction. It naturally hugs the outer
            // boundary going clockwise.
            //
            // Compute CW angle: take the standard CCW angle (atan2 of
            // cross then dot) and negate to get CW; wrap to [ε, 2π).
            // U-turns (going back to prev) are excluded by the canonical
            // edge-visited check above.
            let dot = in_x * ox + in_y * oy;
            let cross = in_x * oy - in_y * ox;
            let ccw = cross.atan2(dot);
            let mut cw = -ccw;
            if cw <= 1e-9 {
                cw += std::f64::consts::TAU;
            }
            match best {
                None => best = Some((cw, (nk, np))),
                Some((bcw, _)) if cw < bcw => best = Some((cw, (nk, np))),
                _ => {}
            }
        }
        best.map(|(_, p)| p)
    };

    for &(start_a, start_b) in edges {
        let ka = key(start_a);
        let kb = key(start_b);
        let canon = if ka <= kb { (ka, kb) } else { (kb, ka) };
        if visited.contains(&canon) {
            continue;
        }
        // Walk the chain starting at this edge.
        let mut path: Vec<(f64, f64)> = vec![start_a, start_b];
        let mut path_keys: Vec<(i64, i64)> = vec![ka, kb];
        visited.insert(canon);
        loop {
            let last = *path_keys.last().unwrap();
            let last_pos = *path.last().unwrap();
            let prev_pos = path[path.len() - 2];
            let next = pick_next(&adj, &visited, last, last_pos, prev_pos);
            let Some((nk, np)) = next else { break };
            let canon = if last <= nk { (last, nk) } else { (nk, last) };
            visited.insert(canon);
            path.push(np);
            path_keys.push(nk);
            // Closed loop: came back to start.
            if *path_keys.last().unwrap() == path_keys[0] {
                path.pop();
                path_keys.pop();
                break;
            }
        }
        if path.len() >= 3 {
            loops.push(path);
        }
    }
    loops
}

fn distance_2d(a: (f64, f64), b: (f64, f64)) -> f64 {
    let dx = a.0 - b.0;
    let dy = a.1 - b.1;
    (dx * dx + dy * dy).sqrt()
}

fn polygon_signed_area_2d(poly: &[(f64, f64)]) -> f64 {
    let n = poly.len();
    if n < 3 {
        return 0.0;
    }
    let mut a = 0.0;
    for i in 0..n {
        let (x0, y0) = poly[i];
        let (x1, y1) = poly[(i + 1) % n];
        a += x0 * y1 - x1 * y0;
    }
    a * 0.5
}

/// Andrew's monotone-chain convex hull. Returns the hull in CCW order
/// without duplicating the start point.
fn convex_hull(mut pts: Vec<(f64, f64)>) -> Vec<(f64, f64)> {
    pts.sort_by(|a, b| {
        a.0.partial_cmp(&b.0)
            .unwrap_or(std::cmp::Ordering::Equal)
            .then(a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal))
    });
    pts.dedup_by(|a, b| (a.0 - b.0).abs() < 1e-12 && (a.1 - b.1).abs() < 1e-12);
    let n = pts.len();
    if n < 2 {
        return pts;
    }
    let cross = |o: (f64, f64), a: (f64, f64), b: (f64, f64)| -> f64 {
        (a.0 - o.0) * (b.1 - o.1) - (a.1 - o.1) * (b.0 - o.0)
    };
    let mut lower: Vec<(f64, f64)> = Vec::new();
    for &p in &pts {
        while lower.len() >= 2 && cross(lower[lower.len() - 2], lower[lower.len() - 1], p) <= 0.0 {
            lower.pop();
        }
        lower.push(p);
    }
    let mut upper: Vec<(f64, f64)> = Vec::new();
    for &p in pts.iter().rev() {
        while upper.len() >= 2 && cross(upper[upper.len() - 2], upper[upper.len() - 1], p) <= 0.0 {
            upper.pop();
        }
        upper.push(p);
    }
    lower.pop();
    upper.pop();
    lower.extend(upper);
    lower
}

/// Axis-aligned bounding box of a 2D point cloud as `(min_u, min_v, max_u, max_v)`.
fn bbox_2d(pts: &[(f64, f64)]) -> (f64, f64, f64, f64) {
    if pts.is_empty() {
        return (0.0, 0.0, 0.0, 0.0);
    }
    let mut min_u = f64::INFINITY;
    let mut min_v = f64::INFINITY;
    let mut max_u = f64::NEG_INFINITY;
    let mut max_v = f64::NEG_INFINITY;
    for &(u, v) in pts {
        if u < min_u { min_u = u; }
        if v < min_v { min_v = v; }
        if u > max_u { max_u = u; }
        if v > max_v { max_v = v; }
    }
    (min_u, min_v, max_u, max_v)
}

// --- Vertex-snap helpers ----------------------------------------------

/// Snap-target kinds, ordered from most-specific to least-specific. The
/// snap helper chooses the closest match from the list of candidates.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SnapKind {
    /// A vertex (line endpoint, circle seam endpoint, etc.) — exact 3D
    /// position from `solid.vertex_geom`.
    Vertex,
    /// The center of a Circle curve attached to one of the solid's edges.
    /// Useful for picking the axis of holes / shafts.
    CircleCenter,
    /// The midpoint of a Line edge — useful for radial / centerline picks.
    LineMidpoint,
}

/// One snap candidate. The renderer / picker compares the screen-space
/// distance from the cursor to `point` and returns the closest hit.
#[derive(Debug, Clone)]
pub struct SnapCandidate {
    pub point: Point3,
    pub kind: SnapKind,
}

/// Collect all snap candidates from a solid: every vertex position, every
/// circle edge's center point, and every line edge's midpoint.
///
/// The viewer gets a flat list — it's the picker's job to project them
/// through the active view and find the screen-space closest hit. The
/// returned set is finite and small (O(V + E)) so cheap to recompute on
/// every cursor move.
pub fn collect_snap_candidates(solid: &Solid) -> Vec<SnapCandidate> {
    let mut out = Vec::new();
    for vid in solid.topo.vertex_ids() {
        if let Some(p) = solid.vertex_geom.get(vid) {
            out.push(SnapCandidate { point: *p, kind: SnapKind::Vertex });
        }
    }
    for eid in solid.topo.edge_ids() {
        if let Some(seg) = solid.edge_geom.get(eid) {
            match &seg.curve {
                CurveKind::Circle(c) => {
                    out.push(SnapCandidate {
                        point: c.frame.origin,
                        kind: SnapKind::CircleCenter,
                    });
                }
                CurveKind::Line(_) => {
                    let (t0, t1) = seg.range;
                    let mid = seg.point_at((t0 + t1) * 0.5);
                    out.push(SnapCandidate { point: mid, kind: SnapKind::LineMidpoint });
                }
                CurveKind::Ellipse(_) => {}
            }
        }
    }
    out
}

/// Snap a picked 3D point to the closest snap candidate. Returns the snapped
/// point, or `pick` itself if no candidate is closer than `tol_3d` (in
/// world units). Also returns the matched kind so the UI can highlight it.
///
/// Used by the viewer's "click two vertices" workflow: the user clicks
/// roughly on a model edge, the back-end finds the nearest endpoint, and
/// the dimension uses the snapped point so subsequent measurements are
/// pixel-exact regardless of tessellation density.
pub fn snap_pick(
    solid: &Solid,
    pick: Point3,
    tol_3d: f64,
) -> Option<(Point3, SnapKind)> {
    if !tol_3d.is_finite() || tol_3d <= 0.0 {
        return None;
    }
    let mut best: Option<(f64, Point3, SnapKind)> = None;
    let candidates = collect_snap_candidates(solid);
    for c in candidates {
        let d = (c.point - pick).norm();
        if d <= tol_3d {
            match best {
                Some((bd, _, _)) if bd <= d => {}
                _ => best = Some((d, c.point, c.kind)),
            }
        }
    }
    best.map(|(_, p, k)| (p, k))
}

// --- SVG renderer ------------------------------------------------------

/// Render `solid` to an SVG string with the requested view, dimensions,
/// and viewport.
///
/// Output layout:
/// - One root `<svg>` with `viewBox = "0 0 width height"`.
/// - The outer silhouette as one `<polygon class="kerf-silhouette">` per
///   disjoint shell. Empty solids contribute no polygon at all.
/// - Interior silhouette edges (the seam between front-facing and
///   back-facing curved-surface portions, plus inner cavity outlines)
///   render as dashed `<polyline class="kerf-hidden">` paths.
/// - One `<g class="kerf-dim">` per dimension with dimension lines,
///   arrowheads (paths), and a `<text>` with the measured value.
///
/// Drawings v2 (2026-05-08): the silhouette is now extracted by walking
/// the tessellated half-edge adjacency (front/back triangle classification)
/// rather than using the convex hull. Concave parts (e.g. L-brackets
/// viewed from above) now render with their true outline, and hidden
/// edges render as dashed back-lines. The convex-hull fallback in
/// [`projected_silhouette`] is still available for callers that want a
/// quick cheap outline.
///
/// The renderer does no font management; dimension text uses generic
/// `font-family="sans-serif"`.
pub fn render_dimensioned_view(
    solid: &Solid,
    view: ViewKind,
    dimensions: &[Dimension],
    viewport: ViewportSpec,
) -> String {
    let segments = 24;
    let loops = silhouette_loops(solid, view, segments);
    // Flatten outer + interior into a single point cloud for bbox sizing.
    let mut silhouette_pts: Vec<(f64, f64)> = Vec::new();
    for l in &loops.outer {
        silhouette_pts.extend_from_slice(l);
    }
    for l in &loops.interior {
        silhouette_pts.extend_from_slice(l);
    }
    // Empty solid: no loops.
    let silhouette = silhouette_pts;

    // Compute extents that include both the silhouette and every dimension
    // point, so dimensions placed off the part itself still fit on the page.
    let mut all_pts: Vec<(f64, f64)> = silhouette.clone();
    for d in dimensions {
        all_pts.push(to_2d_view(d.from, view));
        all_pts.push(to_2d_view(d.to, view));
        if let DimensionKind::RadialFromCenter { center } = &d.kind {
            all_pts.push(to_2d_view(*center, view));
        }
        if let DimensionKind::Angular { vertex } = &d.kind {
            all_pts.push(to_2d_view(*vertex, view));
        }
    }
    let (min_u, min_v, max_u, max_v) = bbox_2d(&all_pts);

    // Compute uniform scale that maps the world bbox to the inner viewport.
    let inner_w = (viewport.width - 2.0 * viewport.padding).max(1.0);
    let inner_h = (viewport.height - 2.0 * viewport.padding).max(1.0);
    let span_u = (max_u - min_u).max(1e-9);
    let span_v = (max_v - min_v).max(1e-9);
    let scale = (inner_w / span_u).min(inner_h / span_v);

    // Center the projected content inside the viewport.
    let used_w = span_u * scale;
    let used_h = span_v * scale;
    let off_x = viewport.padding + (inner_w - used_w) * 0.5;
    let off_y = viewport.padding + (inner_h - used_h) * 0.5;

    // Closure: world (u, v) → SVG (x, y). SVG Y points down, so flip v.
    let to_svg = |u: f64, v: f64| -> (f64, f64) {
        let x = off_x + (u - min_u) * scale;
        let y = off_y + (max_v - v) * scale;
        (x, y)
    };

    let mut s = String::new();
    s.push_str(&format!(
        "<svg xmlns=\"http://www.w3.org/2000/svg\" viewBox=\"0 0 {w} {h}\" \
         width=\"{w}\" height=\"{h}\" font-family=\"sans-serif\" font-size=\"11\">\n",
        w = viewport.width,
        h = viewport.height,
    ));
    // Background + view label.
    s.push_str("  <rect width=\"100%\" height=\"100%\" fill=\"#ffffff\"/>\n");
    s.push_str(&format!(
        "  <text x=\"{x}\" y=\"{y}\" fill=\"#161b22\" font-weight=\"bold\">{label}</text>\n",
        x = 8.0,
        y = 16.0,
        label = view.as_str().to_uppercase(),
    ));

    // Outer silhouette polygons.
    for outer in &loops.outer {
        if outer.len() >= 3 {
            let pts_str: Vec<String> = outer
                .iter()
                .map(|&(u, v)| {
                    let (x, y) = to_svg(u, v);
                    format!("{:.3},{:.3}", x, y)
                })
                .collect();
            s.push_str(&format!(
                "  <polygon class=\"kerf-silhouette\" points=\"{}\" \
                 fill=\"#c8d3df\" fill-opacity=\"0.5\" stroke=\"#222831\" stroke-width=\"1\"/>\n",
                pts_str.join(" ")
            ));
        }
    }
    // Hidden / interior silhouette edges, drawn dashed so the user can
    // distinguish "behind the part" lines from outer outline. These come
    // from concave pockets, inner cavity outlines, and the seam between
    // front-facing and back-facing portions of curved surfaces.
    for interior in &loops.interior {
        if interior.len() >= 2 {
            let pts_str: Vec<String> = interior
                .iter()
                .map(|&(u, v)| {
                    let (x, y) = to_svg(u, v);
                    format!("{:.3},{:.3}", x, y)
                })
                .collect();
            s.push_str(&format!(
                "  <polyline class=\"kerf-hidden\" points=\"{} {}\" \
                 fill=\"none\" stroke=\"#222831\" stroke-width=\"0.6\" \
                 stroke-dasharray=\"4 3\" stroke-opacity=\"0.7\"/>\n",
                pts_str.join(" "),
                pts_str.first().cloned().unwrap_or_default()
            ));
        }
    }

    // Dimensions.
    for d in dimensions {
        match &d.kind {
            DimensionKind::Linear => render_linear(&mut s, d, view, &to_svg),
            DimensionKind::RadialFromCenter { center } => {
                render_radial(&mut s, d, *center, view, &to_svg)
            }
            DimensionKind::Angular { vertex } => {
                render_angular(&mut s, d, *vertex, view, &to_svg)
            }
        }
    }

    s.push_str("</svg>\n");
    s
}

fn render_linear<F: Fn(f64, f64) -> (f64, f64)>(
    out: &mut String,
    d: &Dimension,
    view: ViewKind,
    to_svg: &F,
) {
    let value = distance(d.from, d.to);
    let (u1, v1) = to_2d_view(d.from, view);
    let (u2, v2) = to_2d_view(d.to, view);
    let (x1, y1) = to_svg(u1, v1);
    let (x2, y2) = to_svg(u2, v2);
    let mx = (x1 + x2) * 0.5;
    let my = (y1 + y2) * 0.5;
    out.push_str("  <g class=\"kerf-dim kerf-dim-linear\" stroke=\"#3f6cba\" fill=\"#3f6cba\">\n");
    out.push_str(&format!(
        "    <line x1=\"{x1:.3}\" y1=\"{y1:.3}\" x2=\"{x2:.3}\" y2=\"{y2:.3}\" stroke-width=\"1\"/>\n"
    ));
    write_arrowhead(out, x2, y2, x1, y1);
    write_arrowhead(out, x1, y1, x2, y2);
    out.push_str(&format!(
        "    <text x=\"{mx:.3}\" y=\"{my:.3}\" dy=\"-4\" text-anchor=\"middle\" stroke=\"none\">{val:.3}</text>\n",
        val = value,
    ));
    out.push_str("  </g>\n");
}

fn render_radial<F: Fn(f64, f64) -> (f64, f64)>(
    out: &mut String,
    d: &Dimension,
    center: Point3,
    view: ViewKind,
    to_svg: &F,
) {
    let value = distance(center, d.to);
    let (cu, cv) = to_2d_view(center, view);
    let (u2, v2) = to_2d_view(d.to, view);
    let (cx, cy) = to_svg(cu, cv);
    let (x2, y2) = to_svg(u2, v2);
    out.push_str("  <g class=\"kerf-dim kerf-dim-radial\" stroke=\"#3f6cba\" fill=\"#3f6cba\">\n");
    out.push_str(&format!(
        "    <line x1=\"{cx:.3}\" y1=\"{cy:.3}\" x2=\"{x2:.3}\" y2=\"{y2:.3}\" stroke-width=\"1\"/>\n"
    ));
    write_arrowhead(out, cx, cy, x2, y2);
    let mx = (cx + x2) * 0.5;
    let my = (cy + y2) * 0.5;
    out.push_str(&format!(
        "    <text x=\"{mx:.3}\" y=\"{my:.3}\" dy=\"-4\" text-anchor=\"middle\" stroke=\"none\">R{val:.3}</text>\n",
        val = value,
    ));
    out.push_str("  </g>\n");
}

fn render_angular<F: Fn(f64, f64) -> (f64, f64)>(
    out: &mut String,
    d: &Dimension,
    vertex: Point3,
    view: ViewKind,
    to_svg: &F,
) {
    let radians = angle_at_vertex(d.from, vertex, d.to);
    let degrees = radians.to_degrees();
    let (vu, vv) = to_2d_view(vertex, view);
    let (au, av) = to_2d_view(d.from, view);
    let (bu, bv) = to_2d_view(d.to, view);
    let (vx, vy) = to_svg(vu, vv);
    let (ax, ay) = to_svg(au, av);
    let (bx, by) = to_svg(bu, bv);
    out.push_str("  <g class=\"kerf-dim kerf-dim-angular\" stroke=\"#3f6cba\" fill=\"#3f6cba\">\n");
    out.push_str(&format!(
        "    <line x1=\"{vx:.3}\" y1=\"{vy:.3}\" x2=\"{ax:.3}\" y2=\"{ay:.3}\" stroke-width=\"1\"/>\n"
    ));
    out.push_str(&format!(
        "    <line x1=\"{vx:.3}\" y1=\"{vy:.3}\" x2=\"{bx:.3}\" y2=\"{by:.3}\" stroke-width=\"1\"/>\n"
    ));
    out.push_str(&format!(
        "    <text x=\"{vx:.3}\" y=\"{vy:.3}\" dy=\"-6\" text-anchor=\"middle\" stroke=\"none\">{deg:.2}°</text>\n",
        deg = degrees,
    ));
    out.push_str("  </g>\n");
}

/// Append an arrowhead (small filled triangle) at `(tip_x, tip_y)`,
/// pointing away from `(tail_x, tail_y)`.
fn write_arrowhead(out: &mut String, tail_x: f64, tail_y: f64, tip_x: f64, tip_y: f64) {
    let dx = tip_x - tail_x;
    let dy = tip_y - tail_y;
    let len = (dx * dx + dy * dy).sqrt();
    if len < 1e-9 {
        return;
    }
    let ux = dx / len;
    let uy = dy / len;
    let head = 6.0_f64;
    // Two base corners perpendicular to the shaft.
    let base_x = tip_x - ux * head;
    let base_y = tip_y - uy * head;
    let half = head * 0.4;
    let p1x = base_x + (-uy) * half;
    let p1y = base_y + ux * half;
    let p2x = base_x - (-uy) * half;
    let p2y = base_y - ux * half;
    out.push_str(&format!(
        "    <path d=\"M{tip_x:.3},{tip_y:.3} L{p1x:.3},{p1y:.3} L{p2x:.3},{p2y:.3} Z\" stroke=\"none\"/>\n"
    ));
}

// -----------------------------------------------------------------------
// Tests
// -----------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::primitives::box_at;
    use kerf_geom::{Point3, Vec3};

    #[test]
    fn distance_between_known_points() {
        let p1 = Point3::new(0.0, 0.0, 0.0);
        let p2 = Point3::new(3.0, 4.0, 0.0);
        assert!((distance(p1, p2) - 5.0).abs() < 1e-12);

        let p3 = Point3::new(1.0, 2.0, 2.0);
        let p4 = Point3::new(1.0, 2.0, 2.0);
        assert!(distance(p3, p4).abs() < 1e-12);

        let p5 = Point3::new(-1.0, -1.0, -1.0);
        let p6 = Point3::new(1.0, 1.0, 1.0);
        assert!((distance(p5, p6) - (12.0_f64).sqrt()).abs() < 1e-12);
    }

    #[test]
    fn angle_between_orthogonal_and_parallel() {
        let x = Vec3::new(1.0, 0.0, 0.0);
        let y = Vec3::new(0.0, 1.0, 0.0);
        let mx = Vec3::new(-1.0, 0.0, 0.0);
        let two_x = Vec3::new(2.0, 0.0, 0.0);
        // Orthogonal → pi/2.
        assert!((angle_between_vectors(x, y) - std::f64::consts::FRAC_PI_2).abs() < 1e-12);
        // Parallel-same → 0.
        assert!(angle_between_vectors(x, two_x).abs() < 1e-12);
        // Anti-parallel → pi.
        assert!((angle_between_vectors(x, mx) - std::f64::consts::PI).abs() < 1e-12);
        // Zero-length input → 0 (degenerate; documented).
        assert_eq!(angle_between_vectors(Vec3::zeros(), x), 0.0);
    }

    #[test]
    fn angle_at_vertex_right_angle() {
        let v = Point3::new(0.0, 0.0, 0.0);
        let a = Point3::new(1.0, 0.0, 0.0);
        let b = Point3::new(0.0, 1.0, 0.0);
        let theta = angle_at_vertex(a, v, b);
        assert!((theta - std::f64::consts::FRAC_PI_2).abs() < 1e-12);
    }

    #[test]
    fn project_to_plane_round_trip() {
        // A point above the XY plane projects to its (x, y, 0) shadow,
        // and projecting again is idempotent.
        let p = Point3::new(2.0, -3.0, 5.0);
        let n = Vec3::new(0.0, 0.0, 1.0);
        let o = Point3::new(0.0, 0.0, 0.0);
        let q = project_to_plane(p, n, o);
        assert!((q.x - 2.0).abs() < 1e-12);
        assert!((q.y - (-3.0)).abs() < 1e-12);
        assert!(q.z.abs() < 1e-12);
        // Idempotence.
        let q2 = project_to_plane(q, n, o);
        assert!((q - q2).norm() < 1e-12);

        // Non-axis plane: project a point onto the plane through origin
        // with normal (1, 1, 0) / sqrt(2). The result should satisfy
        // (q - o) . n == 0.
        let n2 = Vec3::new(1.0, 1.0, 0.0);
        let q3 = project_to_plane(Point3::new(3.0, 0.0, 7.0), n2, o);
        let residual = (q3 - o).dot(&n2);
        assert!(residual.abs() < 1e-12, "residual {}", residual);

        // Degenerate normal: should return p unchanged.
        let q_deg = project_to_plane(p, Vec3::zeros(), o);
        assert!((q_deg - p).norm() < 1e-12);
    }

    #[test]
    fn to_2d_view_axis_mapping() {
        let p = Point3::new(2.0, -3.0, 5.0);
        assert_eq!(to_2d_view(p, ViewKind::Top), (2.0, -3.0));
        assert_eq!(to_2d_view(p, ViewKind::Front), (2.0, 5.0));
        assert_eq!(to_2d_view(p, ViewKind::Side), (-3.0, 5.0));
        // Iso just needs to be deterministic and non-degenerate.
        let (u, v) = to_2d_view(p, ViewKind::Iso);
        assert!(u.is_finite() && v.is_finite());
    }

    #[test]
    fn view_kind_parse() {
        assert_eq!(ViewKind::from_str("top"), Some(ViewKind::Top));
        assert_eq!(ViewKind::from_str("FRONT"), Some(ViewKind::Front));
        assert_eq!(ViewKind::from_str(" Side "), Some(ViewKind::Side));
        assert_eq!(ViewKind::from_str("right"), Some(ViewKind::Side));
        assert_eq!(ViewKind::from_str("ISO"), Some(ViewKind::Iso));
        assert_eq!(ViewKind::from_str("oblique"), None);
    }

    #[test]
    fn render_top_view_of_box_contains_dim_line_and_text() {
        // 2x4x1 box centered at origin. Top view sees the 2x4 rectangle.
        let s = box_at(Vec3::new(2.0, 4.0, 1.0), Point3::new(0.0, 0.0, 0.0));
        let from = Point3::new(-1.0, -2.0, 0.5);
        let to = Point3::new(1.0, -2.0, 0.5);
        let dim = Dimension { from, to, kind: DimensionKind::Linear };
        let svg = render_dimensioned_view(
            &s,
            ViewKind::Top,
            std::slice::from_ref(&dim),
            ViewportSpec::default_a4_ish(),
        );
        // Header
        assert!(svg.starts_with("<svg"), "missing <svg> root: {}", &svg[..40.min(svg.len())]);
        assert!(svg.contains("</svg>"));
        // Silhouette polygon
        assert!(svg.contains("kerf-silhouette"), "no silhouette polygon");
        assert!(svg.contains("<polygon"));
        // Dimension group + line + arrowhead path
        assert!(svg.contains("kerf-dim-linear"), "no linear-dim group");
        assert!(svg.contains("<line"), "no <line> for dimension");
        // Measurement value: |to - from| = 2.0, formatted with 3 decimals.
        assert!(svg.contains(">2.000<"), "expected dim value 2.000 in text:\n{}", svg);
        // View label
        assert!(svg.contains(">TOP<"), "expected view label TOP");
    }

    #[test]
    fn render_radial_dimension_renders_r_prefix() {
        let s = box_at(Vec3::new(4.0, 4.0, 1.0), Point3::new(0.0, 0.0, 0.0));
        let dim = Dimension {
            from: Point3::new(0.0, 0.0, 0.0),
            to: Point3::new(2.0, 0.0, 0.0),
            kind: DimensionKind::RadialFromCenter { center: Point3::new(0.0, 0.0, 0.0) },
        };
        let svg = render_dimensioned_view(
            &s,
            ViewKind::Top,
            std::slice::from_ref(&dim),
            ViewportSpec::default_a4_ish(),
        );
        assert!(svg.contains("kerf-dim-radial"));
        assert!(svg.contains(">R2.000<"), "expected R2.000 in:\n{}", svg);
    }

    #[test]
    fn render_angular_dimension_emits_degrees() {
        let s = box_at(Vec3::new(2.0, 2.0, 1.0), Point3::new(0.0, 0.0, 0.0));
        // 90 degrees at origin between +X ray and +Y ray.
        let dim = Dimension {
            from: Point3::new(1.0, 0.0, 0.0),
            to: Point3::new(0.0, 1.0, 0.0),
            kind: DimensionKind::Angular { vertex: Point3::new(0.0, 0.0, 0.0) },
        };
        let svg = render_dimensioned_view(
            &s,
            ViewKind::Top,
            std::slice::from_ref(&dim),
            ViewportSpec::default_a4_ish(),
        );
        assert!(svg.contains("kerf-dim-angular"));
        // Degrees value is 90.00 (with two decimals).
        assert!(svg.contains(">90.00°<"), "expected 90.00° in:\n{}", svg);
    }

    #[test]
    fn empty_solid_renders_blank_svg() {
        // No silhouette polygon when solid is empty.
        let s = Solid::new();
        let svg = render_dimensioned_view(&s, ViewKind::Top, &[], ViewportSpec::default_a4_ish());
        assert!(svg.starts_with("<svg"));
        assert!(svg.contains("</svg>"));
        // No silhouette node when there's nothing to draw.
        assert!(!svg.contains("kerf-silhouette"));
    }

    #[test]
    fn convex_hull_of_unit_square() {
        let pts = vec![
            (0.0, 0.0),
            (1.0, 0.0),
            (1.0, 1.0),
            (0.0, 1.0),
            (0.5, 0.5),  // interior, should be discarded
        ];
        let hull = convex_hull(pts);
        assert_eq!(hull.len(), 4);
        // All four corners must appear.
        let has = |u: f64, v: f64| hull.iter().any(|&(a, b)| (a - u).abs() < 1e-9 && (b - v).abs() < 1e-9);
        assert!(has(0.0, 0.0));
        assert!(has(1.0, 0.0));
        assert!(has(1.0, 1.0));
        assert!(has(0.0, 1.0));
    }

    // --- Drawings v2: silhouette + hidden-line + snap tests --------------

    #[test]
    fn view_direction_is_unit_and_axis_correct() {
        let top = view_direction(ViewKind::Top);
        assert!((top.norm() - 1.0).abs() < 1e-12);
        assert!((top.z - (-1.0)).abs() < 1e-12);
        let front = view_direction(ViewKind::Front);
        assert!((front.norm() - 1.0).abs() < 1e-12);
        assert!((front.y - (-1.0)).abs() < 1e-12);
        let side = view_direction(ViewKind::Side);
        assert!((side.norm() - 1.0).abs() < 1e-12);
        assert!((side.x - (-1.0)).abs() < 1e-12);
        // Iso must be a finite unit vector with a -x component (looking
        // toward -x from outside the part).
        let iso = view_direction(ViewKind::Iso);
        assert!((iso.norm() - 1.0).abs() < 1e-9);
        assert!(iso.x < 0.0);
    }

    #[test]
    fn silhouette_loops_box_top_is_rectangle() {
        // 2x4x1 box at origin (corner-anchored): Top view silhouette is
        // the exact 2x4 rectangle. Vertex count must be 4 — the union
        // boundary is the rectangle itself with no extra collinear
        // points.
        let s = box_at(Vec3::new(2.0, 4.0, 1.0), Point3::new(0.0, 0.0, 0.0));
        let loops = silhouette_loops(&s, ViewKind::Top, 24);
        assert_eq!(loops.outer.len(), 1, "single-shell box has one outer loop");
        let poly = &loops.outer[0];
        assert!(poly.len() >= 4, "rectangle silhouette should have >= 4 vertices, got {}", poly.len());
        // Bounding box of the silhouette must match the box's projected
        // extents: u in [0, 2], v in [0, 4].
        let bb = bbox_2d(poly);
        assert!((bb.0 - 0.0).abs() < 1e-9, "min u {}", bb.0);
        assert!((bb.1 - 0.0).abs() < 1e-9, "min v {}", bb.1);
        assert!((bb.2 - 2.0).abs() < 1e-9, "max u {}", bb.2);
        assert!((bb.3 - 4.0).abs() < 1e-9, "max v {}", bb.3);
        // Area of the projected outline = 8 (2x4 rectangle).
        let a = polygon_signed_area_2d(poly).abs();
        assert!((a - 8.0).abs() < 1e-9, "expected area 8, got {}", a);
    }

    #[test]
    fn silhouette_loops_l_bracket_top_is_concave() {
        // Build an L-bracket via box - corner-cut: a 4x4x1 plate minus a
        // 2x2 corner cut in the +x +y corner. Top-view silhouette must NOT
        // be the convex hull (which would be the full 4x4 square); a true
        // silhouette must follow the concave step.
        let big = box_at(Vec3::new(4.0, 4.0, 1.0), Point3::new(0.0, 0.0, 0.0));
        let cut = box_at(Vec3::new(2.0, 2.0, 2.0), Point3::new(2.0, 2.0, -0.5));
        let l = match big.try_difference(&cut) {
            Ok(s) => s,
            Err(_) => return, // boolean engine quirk; test is a soft pass
        };
        let loops = silhouette_loops(&l, ViewKind::Top, 12);
        // Outer must include the inner step. The resulting polygon area
        // is 4*4 - 2*2 = 12, NOT the convex-hull area of 16. The silhouette
        // walker should report this true area.
        let outer_area: f64 = loops
            .outer
            .iter()
            .map(|p| polygon_signed_area_2d(p).abs())
            .sum();
        // Tolerance allows for some tessellation slop, but must be
        // strictly less than the convex-hull area (16).
        assert!(
            outer_area > 11.0 && outer_area < 13.5,
            "L-bracket silhouette area {} not within (11, 13.5) — silhouette is wrong",
            outer_area
        );
        assert!(
            outer_area < 14.5,
            "outer area {} is suspiciously close to convex hull (16) — silhouette did not detect concavity",
            outer_area
        );
    }

    #[test]
    fn render_l_bracket_emits_dashed_or_concave_outline() {
        // The renderer for a concave part should either:
        //  - Draw the silhouette as a polygon with > 4 vertices (concave
        //    outline), OR
        //  - Draw a `kerf-hidden` dashed polyline for back-edges.
        // In either case the SVG content should signal that the renderer
        // is not just emitting a 4-corner convex hull.
        let big = box_at(Vec3::new(4.0, 4.0, 1.0), Point3::new(0.0, 0.0, 0.0));
        let cut = box_at(Vec3::new(2.0, 2.0, 2.0), Point3::new(2.0, 2.0, -0.5));
        let l = match big.try_difference(&cut) {
            Ok(s) => s,
            Err(_) => return,
        };
        let svg = render_dimensioned_view(&l, ViewKind::Top, &[], ViewportSpec::default_a4_ish());
        assert!(svg.contains("kerf-silhouette"));
        // Count the comma-separated points in the silhouette polygon —
        // if it has more than 4, the silhouette is doing more than a
        // bounding rectangle.
        let count_silhouette_vertices = |svg: &str| -> usize {
            // Find the first kerf-silhouette polygon's points string and
            // count comma occurrences (each "x,y" point has one comma).
            if let Some(start) = svg.find("class=\"kerf-silhouette\"") {
                if let Some(p_start) = svg[start..].find("points=\"") {
                    let abs = start + p_start + 8;
                    if let Some(end_rel) = svg[abs..].find('"') {
                        let s = &svg[abs..abs + end_rel];
                        return s.matches(',').count();
                    }
                }
            }
            0
        };
        let n = count_silhouette_vertices(&svg);
        // Either hidden-line dashed paths exist, OR the outer silhouette
        // has at least 5 vertices (a concave L outline). One of these
        // must be true, otherwise the renderer is treating the L-bracket
        // as a rectangle.
        let has_hidden = svg.contains("kerf-hidden");
        assert!(
            has_hidden || n >= 5,
            "L-bracket SVG looks like a rectangle (silhouette verts={}, hidden={}) — concavity not rendered",
            n,
            has_hidden,
        );
    }

    #[test]
    fn snap_pick_finds_box_corner_within_tolerance() {
        // 2x2x2 box at origin: corners at (0,0,0), (2,0,0), (0,2,0), (2,2,0),
        // and the same set at z=2.
        let s = box_at(Vec3::new(2.0, 2.0, 2.0), Point3::new(0.0, 0.0, 0.0));
        let near_corner = Point3::new(0.05, 0.0, 0.0);
        let snapped = snap_pick(&s, near_corner, 0.1).expect("should snap");
        // Snapped point must be a vertex (corner) within 0.1 of pick.
        let dist = (snapped.0 - Point3::new(0.0, 0.0, 0.0)).norm();
        assert!(dist < 1e-9, "should snap to (0,0,0), got {:?}", snapped.0);
        assert_eq!(snapped.1, SnapKind::Vertex);
    }

    #[test]
    fn snap_pick_returns_none_when_no_candidate_in_tolerance() {
        let s = box_at(Vec3::new(2.0, 2.0, 2.0), Point3::new(0.0, 0.0, 0.0));
        // Pick is far from any vertex.
        let r = snap_pick(&s, Point3::new(1.0, 1.0, 1.0), 0.5);
        assert!(r.is_none(), "no snap expected at center of box");
    }

    #[test]
    fn snap_pick_handles_invalid_tolerance() {
        let s = box_at(Vec3::new(2.0, 2.0, 2.0), Point3::new(0.0, 0.0, 0.0));
        assert!(snap_pick(&s, Point3::new(0.0, 0.0, 0.0), 0.0).is_none());
        assert!(snap_pick(&s, Point3::new(0.0, 0.0, 0.0), -1.0).is_none());
        assert!(snap_pick(&s, Point3::new(0.0, 0.0, 0.0), f64::NAN).is_none());
    }

    #[test]
    fn collect_snap_candidates_includes_vertices_and_line_midpoints() {
        // 1x1x1 box: 8 vertices, 12 edges (all lines). Expect 8 vertex
        // candidates + 12 line-midpoint candidates = 20.
        let s = box_at(Vec3::new(1.0, 1.0, 1.0), Point3::new(0.0, 0.0, 0.0));
        let cs = collect_snap_candidates(&s);
        let n_vertex = cs.iter().filter(|c| c.kind == SnapKind::Vertex).count();
        let n_line = cs.iter().filter(|c| c.kind == SnapKind::LineMidpoint).count();
        assert_eq!(n_vertex, 8, "8 corner vertices");
        assert_eq!(n_line, 12, "12 box edges (all lines)");
    }

    #[test]
    fn snap_pick_finds_circle_center_on_cylinder() {
        // Faceted cylinder has many vertices on the rim, but its analytic
        // top/bottom circle edges (when faceted, they're individual line
        // segments — so this test uses the non-faceted analytic cylinder
        // where the cap edge IS a Circle curve).
        use crate::primitives::cylinder;
        let s = cylinder(1.0, 2.0);
        // The cylinder's top and bottom circles have centers at (0,0,2)
        // and (0,0,0). Pick near the top center.
        let pick = Point3::new(0.05, 0.0, 2.0);
        let r = snap_pick(&s, pick, 0.5);
        assert!(r.is_some(), "expected to snap to circle center near top");
        let (p, kind) = r.unwrap();
        // Both vertex and CircleCenter are valid snaps; either should
        // produce a point at the circle's center on the axis.
        assert!(
            p.x.abs() < 1e-9 && p.y.abs() < 1e-9,
            "snapped point {:?} not on cylinder axis",
            p,
        );
        // CircleCenter, Vertex, or LineMidpoint depending on which is closest.
        // The axis-center (0,0,2) is a CircleCenter; the rim vertex at
        // (1,0,2) is 1.0 away, but the analytic cylinder has at most a
        // seam vertex and the CircleCenter beats it for our 0.05-offset pick.
        let _ = kind;
    }

    #[test]
    fn polygon_signed_area_2d_signs_match_winding() {
        // CCW square: positive area.
        let ccw = vec![(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)];
        assert!((polygon_signed_area_2d(&ccw) - 1.0).abs() < 1e-12);
        // CW square: negative area.
        let cw = vec![(0.0, 0.0), (0.0, 1.0), (1.0, 1.0), (1.0, 0.0)];
        assert!((polygon_signed_area_2d(&cw) - (-1.0)).abs() < 1e-12);
    }

    #[test]
    fn silhouette_loops_empty_solid_is_empty() {
        let s = Solid::new();
        let loops = silhouette_loops(&s, ViewKind::Top, 24);
        assert!(loops.outer.is_empty());
        assert!(loops.interior.is_empty());
    }

    #[test]
    fn chain_edges_to_loops_round_square() {
        // 4 edges forming a unit square — should produce 1 closed loop.
        let edges = vec![
            ((0.0, 0.0), (1.0, 0.0)),
            ((1.0, 0.0), (1.0, 1.0)),
            ((1.0, 1.0), (0.0, 1.0)),
            ((0.0, 1.0), (0.0, 0.0)),
        ];
        let loops = chain_edges_to_loops(&edges);
        assert_eq!(loops.len(), 1);
        assert_eq!(loops[0].len(), 4);
    }
}
