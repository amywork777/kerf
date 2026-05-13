//! Tessellate a Solid into a FaceSoup of triangles. v1 supports the
//! cylinder primitive's topology (planar caps + cylindrical lateral).
//!
//! For planar cap faces bounded by a single circular edge: fan-triangulate
//! around the circle center using `lateral_segments` divisions.
//!
//! For cylindrical lateral faces: emit a quad strip (2 triangles per segment).
//!
//! For other planar faces (polygon prisms etc.): fan-triangulate from vertex 0.
//!
//! ## Smooth-shading extension
//!
//! `tessellate_with_opts` accepts `TessellationOpts` and returns a
//! `TriangleSoup` that may include per-vertex `smooth_normals`.  When
//! `opts.smooth_edges` is `true`, vertices on the circular rim of a cylinder's
//! side face (detected via an adjacent `AnalyticEdge::Circle`) receive the
//! exact outward-radial normal from `Cylinder::normal_at(u, v)` rather than a
//! flat per-triangle normal.  Cap faces stay flat.
//!
//! `tessellate_with_opts` with default opts produces the same triangles as
//! `tessellate` (regression-safe).

use std::f64::consts::TAU;

use kerf_geom::{Curve as _, Surface as _};

use crate::analytic_edge::AnalyticEdge;
use crate::booleans::FaceSoup;
use crate::geometry::{CurveKind, SurfaceKind};
use crate::Solid;
use kerf_topo::{EdgeId, FaceId};

// ---------------------------------------------------------------------------
// Public types
// ---------------------------------------------------------------------------

/// Extended triangle output that carries optional per-vertex smooth normals.
///
/// `triangles[i]` is a triangle with three `Point3` vertices.
/// `smooth_normals` is either empty (no smooth-normal data) or has exactly
/// `3 * triangles.len()` entries — one `[f64; 3]` unit normal per vertex,
/// stored in the same vertex order as the triangles.
///
/// When `smooth_normals` is empty, consumers should derive per-triangle flat
/// normals from the cross product (the same as the old `FaceSoup` behaviour).
#[derive(Clone, Debug, Default)]
pub struct TriangleSoup {
    pub triangles: Vec<[kerf_geom::Point3; 3]>,
    /// Per-vertex smooth normals. Either empty or length == `3 * triangles.len()`.
    pub smooth_normals: Vec<[f64; 3]>,
}

impl TriangleSoup {
    /// Convert to a `FaceSoup` (drops smooth-normal data).
    pub fn into_face_soup(self) -> FaceSoup {
        FaceSoup { triangles: self.triangles }
    }
}

// ---------------------------------------------------------------------------
// Options
// ---------------------------------------------------------------------------

/// Options controlling the tessellator's smooth-shading and edge-subdivision
/// behaviour for `tessellate_with_opts`.
///
/// Construct via `Default::default()` for the conservative baseline (all
/// options off), then set individual flags as needed.
#[derive(Clone, Debug)]
pub struct TessellationOpts {
    /// When `true`, vertices on the circular rim of a cylinder side face
    /// receive a smooth outward-radial normal rather than a flat per-triangle
    /// normal.  Only applied to `Cylinder` side faces whose adjacent cap has
    /// an `AnalyticEdge::Circle`.  Cap faces themselves remain flat.
    ///
    /// Default: `false` (preserves original flat-shading behaviour).
    pub smooth_edges: bool,

    /// If `> 0`, subdivide cylinder side-face edges to this many segments
    /// instead of the polyhedral `lateral_segments` count.  Only takes effect
    /// when an adjacent cap has `AnalyticEdge::Circle`.
    ///
    /// `0` means "use `lateral_segments`" (same as the baseline).
    /// Default: `0`.
    pub edge_subdivision: usize,
}

impl Default for TessellationOpts {
    fn default() -> Self {
        TessellationOpts { smooth_edges: false, edge_subdivision: 0 }
    }
}

// ---------------------------------------------------------------------------
// Public entry points
// ---------------------------------------------------------------------------

/// Tessellate a `Solid` with smooth normals and optional higher-density edge
/// sampling, returning a `TriangleSoup`.
///
/// `lateral_segments` is the base polygon resolution (≥ 3).
///
/// When `opts` is `Default::default()`, the triangles are identical to those
/// produced by `tessellate(solid, lateral_segments)`.  The `smooth_normals`
/// field is populated only when `opts.smooth_edges == true` AND the solid
/// contains at least one Cylinder side face adjacent to a circular cap.
///
/// When `smooth_normals` is non-empty it always satisfies
/// `len(smooth_normals) == 3 * len(triangles)`: flat faces receive their
/// computed per-triangle normal for each of their three vertices.
pub fn tessellate_with_opts(
    solid: &Solid,
    lateral_segments: usize,
    opts: TessellationOpts,
) -> TriangleSoup {
    debug_assert!(lateral_segments >= 3);

    if !opts.smooth_edges && opts.edge_subdivision == 0 {
        // Fast path: no opts active — identical to tessellate().
        let soup = tessellate(solid, lateral_segments);
        return TriangleSoup { triangles: soup.triangles, smooth_normals: vec![] };
    }

    // Collect per-face triangle patches first so we can backfill flat normals
    // for faces that precede the first smooth face in iteration order.
    struct FacePatch {
        triangles: Vec<[kerf_geom::Point3; 3]>,
        /// `Some(normals)` if this face emitted smooth normals (len == 3 * triangles.len()).
        /// `None` if flat (normals will be computed on demand).
        smooth_normals: Option<Vec<[f64; 3]>>,
    }

    let mut patches: Vec<FacePatch> = Vec::new();
    let mut any_smooth = false;

    for face_id in solid.topo.face_ids() {
        let mut patch_soup = TriangleSoup::default();
        tessellate_one_face_opts(&mut patch_soup, solid, face_id, lateral_segments, &opts);
        let has_smooth = !patch_soup.smooth_normals.is_empty();
        if has_smooth {
            any_smooth = true;
        }
        patches.push(FacePatch {
            triangles: patch_soup.triangles,
            smooth_normals: if has_smooth { Some(patch_soup.smooth_normals) } else { None },
        });
    }

    // Assemble final TriangleSoup.
    let mut out = TriangleSoup::default();
    for patch in patches {
        if any_smooth {
            // We need smooth normals for every triangle.
            match patch.smooth_normals {
                Some(normals) => {
                    out.smooth_normals.extend(normals);
                }
                None => {
                    // Compute flat normals for this patch.
                    for tri in &patch.triangles {
                        let [a, b, c] = tri;
                        let ab = kerf_geom::Vec3::new(b.x - a.x, b.y - a.y, b.z - a.z);
                        let ac = kerf_geom::Vec3::new(c.x - a.x, c.y - a.y, c.z - a.z);
                        let n = ab.cross(&ac);
                        let len = n.norm();
                        let flat_n = if len > 1e-14 {
                            [n.x / len, n.y / len, n.z / len]
                        } else {
                            [0.0, 0.0, 1.0]
                        };
                        out.smooth_normals.push(flat_n);
                        out.smooth_normals.push(flat_n);
                        out.smooth_normals.push(flat_n);
                    }
                }
            }
        }
        out.triangles.extend(patch.triangles);
    }
    out
}

/// Tessellate a Solid into triangles. `lateral_segments` is the polygon
/// resolution for cylindrical faces and circular cap faces (must be ≥ 3).
///
/// Triangle winding is outward-normal CCW as seen from outside the solid.
pub fn tessellate(solid: &Solid, lateral_segments: usize) -> FaceSoup {
    let (soup, _) = tessellate_with_face_index(solid, lateral_segments);
    soup
}

/// Same as [`tessellate`], plus a parallel `Vec<u32>` recording which face
/// index each triangle belongs to. Face indices are sequential starting at 0
/// in the order `solid.topo.face_ids()` yields face IDs. Used by the
/// browser viewer for face-level picking.
pub fn tessellate_with_face_index(solid: &Solid, lateral_segments: usize) -> (FaceSoup, Vec<u32>) {
    debug_assert!(lateral_segments >= 3);
    let mut soup = FaceSoup::default();
    let mut face_index: Vec<u32> = Vec::new();
    let mut next_face: u32 = 0;

    for face_id in solid.topo.face_ids() {
        let before = soup.triangles.len();
        tessellate_one_face_into(&mut soup, solid, face_id, lateral_segments);
        let added = soup.triangles.len() - before;
        for _ in 0..added {
            face_index.push(next_face);
        }
        next_face = next_face.saturating_add(1);
    }
    (soup, face_index)
}

// ---------------------------------------------------------------------------
// Smooth-normal helpers
// ---------------------------------------------------------------------------

/// Check whether `face_id` (which must be a Cylinder face) is adjacent to at
/// least one face that qualifies as an analytic circular cap.
///
/// A cap qualifies if it has an `AnalyticEdge::Circle` entry in
/// `face_analytic_edges` (the post-boolean detection path), OR if its edges
/// include a `CurveKind::Circle` segment (the native `cylinder()` primitive
/// path where caps carry analytic circle geometry directly).
///
/// Returns `true` if such an adjacent cap is found.
fn cylinder_face_has_analytic_cap(solid: &Solid, face_id: FaceId) -> bool {
    // Only makes sense on cylinder surfaces.
    if !matches!(solid.face_geom.get(face_id), Some(SurfaceKind::Cylinder(_))) {
        return false;
    }
    let edges = collect_face_edges(solid, face_id);
    for eid in &edges {
        let Some(edge) = solid.topo.edge(*eid) else { continue };
        for he_id in edge.half_edges() {
            let Some(he) = solid.topo.half_edge(he_id) else { continue };
            let Some(loop_) = solid.topo.loop_(he.loop_()) else { continue };
            let adj_face_id = loop_.face();
            if adj_face_id == face_id {
                continue; // same face
            }
            // Path A: post-boolean detection attached a AnalyticEdge::Circle.
            if matches!(
                solid.face_analytic_edges.get(adj_face_id),
                Some(AnalyticEdge::Circle { .. })
            ) {
                return true;
            }
            // Path B: native cylinder() primitive — cap face edges have
            // CurveKind::Circle geometry directly.
            let cap_edges = collect_face_edges(solid, adj_face_id);
            let has_circle_edge = cap_edges.iter().any(|cap_eid| {
                solid.edge_geom.get(*cap_eid)
                    .map(|seg| matches!(&seg.curve, CurveKind::Circle(_)))
                    .unwrap_or(false)
            });
            if has_circle_edge {
                return true;
            }
        }
    }
    false
}

// ---------------------------------------------------------------------------
// Opts-aware per-face tessellator
// ---------------------------------------------------------------------------

/// Tessellate a single face into `out` using the opts-aware path.
///
/// For Cylinder side faces adjacent to an analytic-circle cap:
/// - when `opts.smooth_edges` is true, emit per-vertex smooth normals.
/// - when `opts.edge_subdivision > 0`, use that segment count instead of
///   `lateral_segments`.
///
/// All other cases fall back to the original flat tessellator.
fn tessellate_one_face_opts(
    out: &mut TriangleSoup,
    solid: &Solid,
    face_id: FaceId,
    lateral_segments: usize,
    opts: &TessellationOpts,
) {
    let surface = match solid.face_geom.get(face_id) {
        Some(s) => s,
        None => return,
    };

    // Smooth / higher-density path: only for Cylinder side faces with an
    // analytic-circle cap neighbour.
    if let SurfaceKind::Cylinder(cyl) = surface {
        let do_smooth = opts.smooth_edges;
        let do_subdivide = opts.edge_subdivision > 0;
        if (do_smooth || do_subdivide) && cylinder_face_has_analytic_cap(solid, face_id) {
            let segs = if do_subdivide { opts.edge_subdivision } else { lateral_segments };

            // Resolve height from the solid's edge geometry.
            let edges = collect_face_edges(solid, face_id);
            let height = edges
                .iter()
                .filter_map(|eid| solid.edge_geom.get(*eid))
                .find_map(|seg| match &seg.curve {
                    CurveKind::Line(_) => Some(seg.range.1 - seg.range.0),
                    _ => None,
                })
                .unwrap_or(1.0);

            let dt = TAU / segs as f64;
            for i in 0..segs {
                let t0 = i as f64 * dt;
                let t1 = (i + 1) as f64 * dt;
                let p0_bot = cyl.point_at(t0, 0.0);
                let p1_bot = cyl.point_at(t1, 0.0);
                let p0_top = cyl.point_at(t0, height);
                let p1_top = cyl.point_at(t1, height);
                out.triangles.push([p0_bot, p1_bot, p1_top]);
                out.triangles.push([p0_bot, p1_top, p0_top]);

                if do_smooth {
                    // Outward radial normals at the respective u angles.
                    let n0 = cyl.normal_at(t0, 0.0);
                    let n1 = cyl.normal_at(t1, 0.0);
                    // Triangle 1: vertices p0_bot, p1_bot, p1_top
                    out.smooth_normals.push([n0.x, n0.y, n0.z]);
                    out.smooth_normals.push([n1.x, n1.y, n1.z]);
                    out.smooth_normals.push([n1.x, n1.y, n1.z]);
                    // Triangle 2: vertices p0_bot, p1_top, p0_top
                    out.smooth_normals.push([n0.x, n0.y, n0.z]);
                    out.smooth_normals.push([n1.x, n1.y, n1.z]);
                    out.smooth_normals.push([n0.x, n0.y, n0.z]);
                }
            }
            return;
        }
    }

    // Fallback: delegate to the flat tessellator, wrapping its FaceSoup output.
    let mut flat = FaceSoup::default();
    tessellate_one_face_into(&mut flat, solid, face_id, lateral_segments);
    out.triangles.extend(flat.triangles);
    // smooth_normals stays empty; backfilling for mixed output is handled
    // by the two-pass logic in tessellate_with_opts.
}

// ---------------------------------------------------------------------------
// Original flat per-face tessellator (unchanged)
// ---------------------------------------------------------------------------

fn tessellate_one_face_into(
    soup: &mut FaceSoup,
    solid: &Solid,
    face_id: kerf_topo::FaceId,
    lateral_segments: usize,
) {
        let surface = match solid.face_geom.get(face_id) {
            Some(s) => s,
            None => return,
        };
        match surface {
            SurfaceKind::Plane(plane) => {
                let edges = collect_face_edges(solid, face_id);
                if edges.len() == 1 {
                    // Single-edge face → circular cap.
                    if let Some(circle_edge_id) = edges.first() {
                        if let Some(seg) = solid.edge_geom.get(*circle_edge_id) {
                            if let CurveKind::Circle(c) = &seg.curve {
                                let center = c.frame.origin;
                                let dt = TAU / lateral_segments as f64;
                                for i in 0..lateral_segments {
                                    let t0 = i as f64 * dt;
                                    let t1 = (i + 1) as f64 * dt;
                                    let p0 = c.point_at(t0);
                                    let p1 = c.point_at(t1);
                                    // plane.frame.z is outward normal.
                                    // For top face (normal +z): CCW winding when looking down +z
                                    //   → [center, p0, p1] is CCW (p1 is CCW of p0).
                                    // For bottom face (normal -z): [center, p1, p0].
                                    if plane.frame.z.z > 0.0 {
                                        soup.triangles.push([center, p0, p1]);
                                    } else {
                                        soup.triangles.push([center, p1, p0]);
                                    }
                                }
                                return;
                            }
                        }
                    }
                }
                // Polygonal planar face: fan from vertex 0. Use the RAW loop
                // traversal (no winding normalization) so cavity faces in
                // stitched results emit CCW-from-cavity-outward triangles —
                // matches the convention the rest of the pipeline assumes.
                if let Some(poly) = crate::booleans::face_polygon_raw(solid, face_id) {
                    if poly.len() >= 3 {
                        for i in 1..(poly.len() - 1) {
                            soup.triangles.push([poly[0], poly[i], poly[i + 1]]);
                        }
                    }
                }
            }
            SurfaceKind::Cylinder(cyl) => {
                // Lateral strip: for each angular segment emit two triangles.
                // Height comes from the seam edge's range length.
                let edges = collect_face_edges(solid, face_id);
                let height = edges
                    .iter()
                    .filter_map(|eid| solid.edge_geom.get(*eid))
                    .find_map(|seg| match &seg.curve {
                        CurveKind::Line(_) => Some(seg.range.1 - seg.range.0),
                        _ => None,
                    })
                    .unwrap_or(1.0);
                let dt = TAU / lateral_segments as f64;
                for i in 0..lateral_segments {
                    let t0 = i as f64 * dt;
                    let t1 = (i + 1) as f64 * dt;
                    let p0_bot = cyl.point_at(t0, 0.0);
                    let p1_bot = cyl.point_at(t1, 0.0);
                    let p0_top = cyl.point_at(t0, height);
                    let p1_top = cyl.point_at(t1, height);
                    // Outward normal points away from the axis. Winding CCW from outside:
                    //   [p0_bot, p1_bot, p1_top] and [p0_bot, p1_top, p0_top].
                    soup.triangles.push([p0_bot, p1_bot, p1_top]);
                    soup.triangles.push([p0_bot, p1_top, p0_top]);
                }
            }
            SurfaceKind::Cone(_cone_surf) => {
                // Walk face loop. Count circle edges:
                //   1 circle → true cone (apex on solid). Fan from apex.
                //   2 circles → frustum (apex off solid). Quad-strip between circles.
                let edges = collect_face_edges(solid, face_id);
                let circles: Vec<&kerf_geom::Circle> = edges
                    .iter()
                    .filter_map(|eid| solid.edge_geom.get(*eid))
                    .filter_map(|seg| match &seg.curve {
                        CurveKind::Circle(c) => Some(c),
                        _ => None,
                    })
                    .collect();

                let dt = TAU / lateral_segments as f64;
                if circles.len() == 1 {
                    let c = circles[0];
                    let apex = find_apex_in_face(solid, face_id);
                    for i in 0..lateral_segments {
                        let t0 = i as f64 * dt;
                        let t1 = ((i + 1) % lateral_segments) as f64 * dt;
                        let p0 = c.point_at(t0);
                        let p1 = c.point_at(t1);
                        soup.triangles.push([apex, p0, p1]);
                    }
                } else if circles.len() >= 2 {
                    // Frustum: top and bottom circles. Sort by z so winding is correct.
                    let mut c_top = circles[0];
                    let mut c_bot = circles[1];
                    if c_top.frame.origin.z < c_bot.frame.origin.z {
                        std::mem::swap(&mut c_top, &mut c_bot);
                    }
                    for i in 0..lateral_segments {
                        let t0 = i as f64 * dt;
                        let t1 = ((i + 1) % lateral_segments) as f64 * dt;
                        let p0_bot = c_bot.point_at(t0);
                        let p1_bot = c_bot.point_at(t1);
                        let p0_top = c_top.point_at(t0);
                        let p1_top = c_top.point_at(t1);
                        soup.triangles.push([p0_bot, p1_bot, p1_top]);
                        soup.triangles.push([p0_bot, p1_top, p0_top]);
                    }
                }
            }
            SurfaceKind::Sphere(sph) => {
                use std::f64::consts::PI;
                let polar_segs = (lateral_segments / 2).max(2);
                let dlat = PI / polar_segs as f64;
                let dlon = TAU / lateral_segments as f64;
                for j in 0..polar_segs {
                    let v0 = j as f64 * dlat;
                    let v1 = (j + 1) as f64 * dlat;
                    let north = j == 0;
                    let south = j == polar_segs - 1;
                    for i in 0..lateral_segments {
                        let u0 = i as f64 * dlon;
                        let u1 = ((i + 1) % lateral_segments) as f64 * dlon;
                        let p00 = sph.point_at(u0, v0);
                        let p10 = sph.point_at(u1, v0);
                        let p11 = sph.point_at(u1, v1);
                        let p01 = sph.point_at(u0, v1);
                        // CCW from outward (the radial outward normal at each
                        // sphere point): u goes CCW around z-axis as seen from
                        // +z, and v increases going from north pole (v=0) to
                        // south pole (v=π). At the equator looking outward we
                        // see u to the LEFT and v DOWN, so CCW order is
                        // p00 → p01 → p11 → p10.
                        if north {
                            soup.triangles.push([p00, p01, p11]);
                        } else if south {
                            soup.triangles.push([p00, p01, p10]);
                        } else {
                            soup.triangles.push([p00, p01, p11]);
                            soup.triangles.push([p00, p11, p10]);
                        }
                    }
                }
            }
            SurfaceKind::Torus(tor) => {
                let major_segs = lateral_segments;
                let minor_segs = (lateral_segments / 2).max(2);
                let du = TAU / major_segs as f64;
                let dv = TAU / minor_segs as f64;
                for i in 0..major_segs {
                    let u0 = i as f64 * du;
                    let u1 = ((i + 1) % major_segs) as f64 * du;
                    for j in 0..minor_segs {
                        let v0 = j as f64 * dv;
                        let v1 = ((j + 1) % minor_segs) as f64 * dv;
                        let p00 = tor.point_at(u0, v0);
                        let p10 = tor.point_at(u1, v0);
                        let p11 = tor.point_at(u1, v1);
                        let p01 = tor.point_at(u0, v1);
                        soup.triangles.push([p00, p10, p11]);
                        soup.triangles.push([p00, p11, p01]);
                    }
                }
            }
        }
}

/// Walk a cone face's outer loop to find the apex vertex position.
/// The apex is the origin of the seam half-edge that points to a non-self-loop vertex
/// (i.e., the half-edge whose origin is the apex vertex, not on the base circle).
fn find_apex_in_face(solid: &Solid, face_id: FaceId) -> kerf_geom::Point3 {
    use kerf_geom::Point3;
    let Some(face) = solid.topo.face(face_id) else {
        return Point3::origin();
    };
    let Some(loop_) = solid.topo.loop_(face.outer_loop()) else {
        return Point3::origin();
    };
    let Some(start) = loop_.half_edge() else {
        return Point3::origin();
    };
    let mut cur = start;
    loop {
        let Some(he) = solid.topo.half_edge(cur) else {
            break;
        };
        // The apex half-edge has a Line curve on its edge.
        let eid = he.edge();
        if let Some(seg) = solid.edge_geom.get(eid) {
            if matches!(&seg.curve, CurveKind::Line(_)) {
                // origin of this half-edge: if the line goes apex→bot, origin is apex.
                // apex is the vertex that is NOT on the base circle (z == height, not 0).
                let origin_vid = he.origin();
                if let Some(pos) = solid.vertex_geom.get(origin_vid) {
                    // The apex is at z = height > 0; base is at z = 0.
                    // Return whichever endpoint is not on the base (z ≠ 0).
                    if pos.z.abs() > 1e-10 {
                        return *pos;
                    }
                    // Otherwise the twin endpoint is apex; get dest vertex via twin.
                    let twin_id = he.twin();
                    if let Some(twin) = solid.topo.half_edge(twin_id) {
                        let dest_vid = twin.origin();
                        if let Some(dest_pos) = solid.vertex_geom.get(dest_vid) {
                            return *dest_pos;
                        }
                    }
                }
            }
        }
        cur = he.next();
        if cur == start {
            break;
        }
    }
    Point3::origin()
}

/// Walk a face's outer loop and collect the distinct edge IDs (de-duplicated
/// in case of self-loop — a self-loop visits the same edge twice in its
/// next-chain, but we only want one entry).
fn collect_face_edges(solid: &Solid, face_id: FaceId) -> Vec<EdgeId> {
    let mut edges: Vec<EdgeId> = Vec::new();
    let Some(face) = solid.topo.face(face_id) else {
        return edges;
    };
    let Some(loop_) = solid.topo.loop_(face.outer_loop()) else {
        return edges;
    };
    let Some(start) = loop_.half_edge() else {
        return edges;
    };
    let mut cur = start;
    loop {
        let Some(he) = solid.topo.half_edge(cur) else {
            break;
        };
        let eid = he.edge();
        if !edges.contains(&eid) {
            edges.push(eid);
        }
        cur = he.next();
        if cur == start {
            break;
        }
    }
    edges
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::primitives::{cone, cylinder, sphere, torus};

    #[test]
    fn cone_tessellation_has_expected_triangle_count() {
        // 16 segments: bot fan = 16 tris, lateral fan = 16 tris. Total = 32.
        let s = cone(1.0, 2.0);
        let soup = tessellate(&s, 16);
        assert_eq!(soup.triangles.len(), 32);
    }

    #[test]
    fn cylinder_tessellation_has_expected_triangle_count() {
        // For 16 lateral segments: top fan = 16 tris, bottom fan = 16 tris,
        // lateral strip = 32 tris (2 per quad × 16). Total = 64.
        let s = cylinder(1.0, 2.0);
        let soup = tessellate(&s, 16);
        assert_eq!(soup.triangles.len(), 64);
    }

    #[test]
    fn cylinder_to_stl_writes_valid_file() {
        let s = cylinder(1.0, 2.0);
        let soup = tessellate(&s, 12);
        let mut buf = Vec::new();
        crate::write_binary(&soup, "cylinder", &mut buf).unwrap();
        // 12 segments → 12+12+24 = 48 triangles. 80+4+50*48 = 2484 bytes.
        assert_eq!(buf.len(), 80 + 4 + 50 * 48);
    }

    #[test]
    fn sphere_tessellation_has_expected_triangle_count() {
        // 16 lateral × 8 polar:
        //   north row: 16 triangles (1 per longitude)
        //   middle 6 rows: 16 * 6 * 2 = 192 triangles
        //   south row: 16 triangles
        //   total: 16 + 192 + 16 = 224
        let s = sphere(1.0);
        let soup = tessellate(&s, 16);
        assert_eq!(soup.triangles.len(), 224);
    }

    #[test]
    fn sphere_tessellation_produces_valid_stl() {
        let s = sphere(1.0);
        let soup = tessellate(&s, 12);
        let mut buf = Vec::new();
        crate::write_binary(&soup, "sphere", &mut buf).unwrap();
        // 12 segs, 6 polar: 12 + 12*4*2 + 12 = 12 + 96 + 12 = 120 tris
        assert_eq!(buf.len(), 80 + 4 + 50 * 120);
    }

    #[test]
    fn torus_tessellation_has_expected_triangle_count() {
        // 16 major × 8 minor: 16 * 8 * 2 = 256 triangles.
        let s = torus(3.0, 1.0);
        let soup = tessellate(&s, 16);
        assert_eq!(soup.triangles.len(), 256);
    }

    // -----------------------------------------------------------------------
    // New tests for tessellate_with_opts
    // -----------------------------------------------------------------------

    /// Test 1 (regression): tessellate_with_opts with default opts (smooth_edges=false,
    /// edge_subdivision=0) produces the same triangle count as tessellate().
    #[test]
    fn with_opts_default_matches_tessellate() {
        let s = cylinder(1.0, 2.0);
        let n = 16;
        let baseline = tessellate(&s, n);
        let opts_out = tessellate_with_opts(&s, n, TessellationOpts::default());
        assert_eq!(
            baseline.triangles.len(),
            opts_out.triangles.len(),
            "default opts should match tessellate triangle count"
        );
        // No smooth normals when smooth_edges is false.
        assert!(
            opts_out.smooth_normals.is_empty(),
            "smooth_normals must be empty when smooth_edges=false"
        );
    }

    /// Test 2: smooth_edges=true on a cylinder with analytic caps produces
    /// smooth normals (one per vertex, 3 per triangle) on the side faces and
    /// the normals are unit-length outward radial vectors.
    #[test]
    fn smooth_edges_cylinder_has_smooth_normals() {
        // cylinder() has CurveKind::Circle edges on its caps — the tessellator
        // detects these via Path B in cylinder_face_has_analytic_cap.
        let s = cylinder(1.0, 2.0);

        let n = 16;
        let opts = TessellationOpts { smooth_edges: true, edge_subdivision: 0 };
        let out = tessellate_with_opts(&s, n, opts);

        // Must have some smooth normals.
        assert!(
            !out.smooth_normals.is_empty(),
            "expected smooth normals on cylinder with analytic caps"
        );
        // smooth_normals.len() must equal 3 * triangles.len() when non-empty.
        assert_eq!(
            out.smooth_normals.len(),
            3 * out.triangles.len(),
            "smooth_normals must have exactly 3 entries per triangle"
        );
        // Every smooth normal must be (approximately) unit length.
        for (i, n) in out.smooth_normals.iter().enumerate() {
            let len = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
            assert!(
                (len - 1.0).abs() < 1e-10,
                "normal[{i}] has length {len}, expected 1.0"
            );
        }
        // Normals on the cylinder side face are radial (z component ≈ 0 for a
        // z-axis cylinder).  Cap normals have z ≈ ±1.  We verify that at least
        // some normals are radial (z ≈ 0), confirming that the smooth-normal
        // emission path was exercised for the side face.
        let radial_count = out.smooth_normals.iter().filter(|n| n[2].abs() < 1e-10).count();
        assert!(
            radial_count > 0,
            "expected at least some z≈0 normals from the cylinder side face"
        );
        // Specifically, all radial normals must be exactly unit length in XY.
        for n in out.smooth_normals.iter().filter(|n| n[2].abs() < 1e-10) {
            let xy_len = (n[0] * n[0] + n[1] * n[1]).sqrt();
            assert!(
                (xy_len - 1.0).abs() < 1e-10,
                "radial normal xy_len={xy_len}, expected 1.0"
            );
        }
    }

    /// Test 3: edge_subdivision=N produces more triangles on the side face than
    /// the polyhedral segment count alone, when N > lateral_segments.
    #[test]
    fn edge_subdivision_produces_more_triangles() {
        // cylinder() has CurveKind::Circle edges on its caps — no need for
        // attach_analytic_circles.
        let s = cylinder(1.0, 2.0);

        let base_segs = 12usize;
        let subdiv = 32usize;
        assert!(subdiv > base_segs);

        let baseline = tessellate_with_opts(&s, base_segs, TessellationOpts::default());
        let subdivided = tessellate_with_opts(
            &s,
            base_segs,
            TessellationOpts { smooth_edges: false, edge_subdivision: subdiv },
        );

        // The subdivided cylinder has more triangles overall because the side
        // face uses `subdiv` segments instead of `base_segs`.
        assert!(
            subdivided.triangles.len() > baseline.triangles.len(),
            "edge_subdivision={subdiv} should produce more triangles than base_segs={base_segs}: \
             got {} vs {}",
            subdivided.triangles.len(),
            baseline.triangles.len()
        );
    }

    /// Test 4: A face without an adjacent analytic edge gets unchanged (flat)
    /// behaviour even when smooth_edges=true.
    #[test]
    fn face_without_analytic_edge_is_unchanged() {
        // A plain box has no analytic edges — all faces must tessellate flat.
        let s = crate::primitives::box_(kerf_geom::Vec3::new(2.0, 3.0, 4.0));
        let opts = TessellationOpts { smooth_edges: true, edge_subdivision: 32 };
        // Use a low segment count; box ignores lateral_segments entirely.
        let out = tessellate_with_opts(&s, 8, opts);
        let baseline = tessellate(&s, 8);
        assert_eq!(
            out.triangles.len(),
            baseline.triangles.len(),
            "box face count must be unchanged with opts"
        );
        assert!(
            out.smooth_normals.is_empty(),
            "box must not emit smooth normals (no analytic edges)"
        );
    }
}
