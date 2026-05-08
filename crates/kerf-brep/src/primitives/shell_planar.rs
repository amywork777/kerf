//! `shell_planar(input, thickness)` — offset-inward solid for planar polyhedra.
//!
//! Produces a hollow shell of thickness `thickness` from a planar B-rep solid.
//! The implementation is **planar-only**: it inspects every face's surface kind
//! and rejects inputs containing any non-`Plane` surface (curved-surface Shell
//! is multi-week kernel work — see STATUS.md).
//!
//! ## Algorithm
//!
//! For a planar polyhedron each face has an outward unit normal `n_f` and a
//! plane equation `n_f · x = d_f`. The inward-offset plane (offset by
//! `thickness` along `-n_f`) has equation `n_f · x = d_f - thickness`.
//!
//! For each vertex `v` of the input, we collect the set of incident faces (via
//! a half-edge walk around the vertex), then solve for the new vertex position
//! at the intersection of the inward-offset planes:
//!
//!   - 3 incident faces → unique solution from a 3×3 linear system.
//!   - 4+ incident faces (e.g. apex of a square pyramid) → least-squares
//!     normal-equations solution; only valid if the planes share a common
//!     intersection (true for convex polyhedra apexes).
//!
//! The output solid uses the **same topology** as the input — only the vertex
//! positions and face plane frames are recomputed. Edge curve segments are
//! rebuilt as fresh `Line` segments between the new endpoints.
//!
//! ## Limitations
//!
//! - **Convex inputs only in practice.** For a non-convex (concave) vertex,
//!   neighboring inward-offset planes may meet *past* each other, producing a
//!   self-intersecting inner solid. We do not detect this; callers feeding
//!   concave inputs (e.g. an L-bracket) can get a malformed inner solid that
//!   trips the boolean engine. The function still returns the difference
//!   attempt — failures surface as `try_difference` errors.
//! - **Topology must match between offsets.** This works for convex polyhedra
//!   where the inward offset preserves the face adjacency graph. For shapes
//!   where the inward offset would "self-prune" a face (e.g. a thin spike),
//!   the resulting inner solid is still valid topologically but geometrically
//!   collapsed; the boolean may fail.
//! - **Curved surfaces rejected outright.** Sphere/Cylinder/Frustum/Cone/Torus
//!   inputs return `ShellError::CurvedSurface`.

use kerf_geom::{Frame, Line, Plane, Point3, Vec3};

use crate::booleans::BooleanOp;
use crate::solid::try_boolean_solid;
use crate::geometry::{CurveSegment, SurfaceKind};
use crate::Solid;

/// Errors from [`shell_planar`].
#[derive(Debug)]
pub enum ShellError {
    /// Thickness must be strictly positive.
    NonPositiveThickness(f64),
    /// Input contains a non-planar face (Sphere/Cylinder/Cone/Torus surface).
    /// Curved-surface Shell is not implemented.
    CurvedSurface,
    /// Could not solve for an inward-offset vertex position. Either the
    /// vertex has fewer than 3 incident faces (degenerate topology) or the
    /// face normals at this vertex are coplanar (can happen at the apex of
    /// a degenerate pyramid).
    DegenerateVertex,
    /// The boolean difference between outer and inner shells failed —
    /// typically because the inner solid self-intersected after offset.
    BooleanFailed(String),
}

impl std::fmt::Display for ShellError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ShellError::NonPositiveThickness(t) => {
                write!(f, "shell thickness must be > 0 (got {t})")
            }
            ShellError::CurvedSurface => write!(
                f,
                "shell only supports planar inputs; found non-planar face surface"
            ),
            ShellError::DegenerateVertex => write!(
                f,
                "could not compute inward offset at vertex (degenerate normals)"
            ),
            ShellError::BooleanFailed(m) => {
                write!(f, "shell boolean (outer minus inner) failed: {m}")
            }
        }
    }
}

impl std::error::Error for ShellError {}

/// Build an inward-offset planar Shell of `input` with wall thickness
/// `thickness`. Returns `outer - inner` where `inner` is `input` with every
/// vertex moved inward to lie on its faces' inward-offset planes.
///
/// See module docs for the algorithm and limitations.
pub fn shell_planar(input: &Solid, thickness: f64) -> Result<Solid, ShellError> {
    if thickness <= 0.0 {
        return Err(ShellError::NonPositiveThickness(thickness));
    }
    // Reject any non-planar face up front — this is the "planar only" gate.
    for (_, surf) in input.face_geom.iter() {
        if !matches!(surf, SurfaceKind::Plane(_)) {
            return Err(ShellError::CurvedSurface);
        }
    }

    let inner = build_inward_offset_solid(input, thickness)?;

    // Outer minus inner = the shell. Use the non-panicking try_boolean_solid
    // so stitch failures (common with thin/concentric polyhedra) surface as
    // errors rather than crashing the caller.
    let result = try_boolean_solid(input, &inner, BooleanOp::Difference)
        .map_err(|e| ShellError::BooleanFailed(e.message))?;

    if result.face_count() == 0 {
        return Err(ShellError::BooleanFailed(
            "outer minus inner produced an empty solid".into(),
        ));
    }
    Ok(result)
}

/// Build a copy of `input` whose vertices are pushed inward along the
/// adjacent face normals by `thickness`. Topology is preserved.
fn build_inward_offset_solid(input: &Solid, thickness: f64) -> Result<Solid, ShellError> {
    let mut out = input.clone();
    // Wipe owner tags from the inner so they don't leak into the difference's
    // provenance map (those tags belong to the outer-shell faces of the input).
    out.face_owner_tag.clear();

    // Collect, per vertex, the set of incident face planes (deduplicated).
    let vertex_ids: Vec<_> = out.topo.vertex_ids().collect();
    for vid in vertex_ids {
        let faces = incident_planes(&out, vid)?;
        let new_pos = solve_offset_vertex(&faces, thickness)?;
        out.vertex_geom.insert(vid, new_pos);
    }

    // Rebuild edge geometry from the new vertex positions.
    let edge_ids: Vec<_> = out.topo.edge_ids().collect();
    for eid in edge_ids {
        let edge = out.topo.edge(eid).expect("edge exists");
        let [he_a, _] = edge.half_edges();
        let he = out.topo.half_edge(he_a).expect("half-edge exists");
        let v0 = he.origin();
        let v1 = out
            .topo
            .half_edge(he.twin())
            .expect("twin exists")
            .origin();
        let p0 = out.vertex_geom[v0];
        let p1 = out.vertex_geom[v1];
        let line = Line::through(p0, p1).ok_or(ShellError::DegenerateVertex)?;
        let len = (p1 - p0).norm();
        out.edge_geom
            .insert(eid, CurveSegment::line(line, 0.0, len));
    }

    // Rebuild face plane frames from the new vertex positions, preserving
    // the original outward normal sign so loop orientation stays consistent.
    let face_ids: Vec<_> = out.topo.face_ids().collect();
    for fid in face_ids {
        let outer_loop = out.topo.face(fid).expect("face exists").outer_loop();
        let lp = out.topo.loop_(outer_loop).expect("loop exists");
        let start = match lp.half_edge() {
            Some(h) => h,
            None => continue,
        };
        // Walk loop and collect first 3 distinct points.
        let mut pts: Vec<Point3> = Vec::new();
        let mut cur = start;
        loop {
            let he = out.topo.half_edge(cur).expect("he exists");
            let p = out.vertex_geom[he.origin()];
            if pts.iter().all(|q| (*q - p).norm() > 1e-12) {
                pts.push(p);
            }
            cur = he.next();
            if cur == start || pts.len() >= 4 {
                break;
            }
        }
        if pts.len() < 3 {
            return Err(ShellError::DegenerateVertex);
        }
        let p0 = pts[0];
        let p1 = pts[1];
        let p2 = pts[2];
        let mut normal = (p1 - p0).cross(&(p2 - p0));
        let nn = normal.norm();
        if nn < 1e-12 {
            return Err(ShellError::DegenerateVertex);
        }
        normal /= nn;
        // Preserve old outward sense if available.
        if let Some(SurfaceKind::Plane(old_p)) = out.face_geom.get(fid)
            && old_p.frame.z.dot(&normal) < 0.0
        {
            normal = -normal;
        }
        let seed = if normal.dot(&Vec3::x()).abs() < 0.9 {
            Vec3::x()
        } else {
            Vec3::y()
        };
        let x = (seed - normal * seed.dot(&normal)).normalize();
        let y = normal.cross(&x);
        let frame = Frame {
            origin: p0,
            x,
            y,
            z: normal,
        };
        out.face_geom.insert(fid, SurfaceKind::Plane(Plane::new(frame)));
    }

    Ok(out)
}

/// Plane representation used during offset: outward normal + signed offset
/// `d` such that `n · x = d` for points on the plane.
#[derive(Clone, Copy, Debug)]
struct OrientedPlane {
    n: Vec3,
    d: f64,
}

/// Collect the set of distinct face planes incident to `vid`. Walks
/// outgoing half-edges (h, h.twin().next(), …) and reads the face surface
/// for each loop.
fn incident_planes(s: &Solid, vid: kerf_topo::VertexId) -> Result<Vec<OrientedPlane>, ShellError> {
    let outgoing_start = match s.topo.vertex(vid).and_then(|v| v.outgoing()) {
        Some(h) => h,
        None => return Err(ShellError::DegenerateVertex),
    };
    let mut planes: Vec<OrientedPlane> = Vec::new();
    let mut cur = outgoing_start;
    let mut steps = 0usize;
    loop {
        steps += 1;
        if steps > 256 {
            // Safety: break out if topology is malformed.
            return Err(ShellError::DegenerateVertex);
        }
        let he = match s.topo.half_edge(cur) {
            Some(h) => h,
            None => return Err(ShellError::DegenerateVertex),
        };
        let lp = match s.topo.loop_(he.loop_()) {
            Some(l) => l,
            None => return Err(ShellError::DegenerateVertex),
        };
        let fid = lp.face();
        if let Some(SurfaceKind::Plane(plane)) = s.face_geom.get(fid) {
            let n = plane.frame.z;
            let d = n.dot(&plane.frame.origin.coords);
            // Deduplicate near-coincident planes (shouldn't happen on a clean
            // input but be robust to repeated half-edge visits).
            let already = planes.iter().any(|p| {
                (p.n - n).norm() < 1e-9 && (p.d - d).abs() < 1e-9
            });
            if !already {
                planes.push(OrientedPlane { n, d });
            }
        } else {
            // Caller already filtered curved surfaces — defensive path.
            return Err(ShellError::CurvedSurface);
        }
        // Move to the next outgoing half-edge around vid: he.twin().next().
        let twin_id = he.twin();
        let twin_he = match s.topo.half_edge(twin_id) {
            Some(h) => h,
            None => return Err(ShellError::DegenerateVertex),
        };
        cur = twin_he.next();
        if cur == outgoing_start {
            break;
        }
    }
    Ok(planes)
}

/// Solve for the inward-offset position of a vertex incident to `planes`
/// (each with outward normal `n` and offset `d`). The new position lies on
/// every plane offset inward by `thickness` (i.e. `n · x = d - thickness`).
///
/// Uses the closed-form Cramer's-rule solution for three planes:
///
///   x = ( d0·(n1×n2) + d1·(n2×n0) + d2·(n0×n1) ) / ( n0 · (n1×n2) )
///
/// For >3 incident planes we pick the triple with the largest mixed-product
/// `|n_a · (n_b × n_c)|` — the most numerically stable choice. On a convex
/// polyhedron all incident planes share a common intersection point, so
/// any non-degenerate triple yields the same answer.
fn solve_offset_vertex(
    planes: &[OrientedPlane],
    thickness: f64,
) -> Result<Point3, ShellError> {
    if planes.len() < 3 {
        return Err(ShellError::DegenerateVertex);
    }
    let mut best: Option<(usize, usize, usize, f64)> = None;
    for i in 0..planes.len() {
        for j in (i + 1)..planes.len() {
            for k in (j + 1)..planes.len() {
                let mixed = planes[i].n.dot(&planes[j].n.cross(&planes[k].n));
                let abs_m = mixed.abs();
                if best.is_none_or(|(_, _, _, b)| abs_m > b) {
                    best = Some((i, j, k, abs_m));
                }
            }
        }
    }
    let (i, j, k, det) = best.ok_or(ShellError::DegenerateVertex)?;
    if det < 1e-9 {
        return Err(ShellError::DegenerateVertex);
    }
    let n0 = planes[i].n;
    let n1 = planes[j].n;
    let n2 = planes[k].n;
    let d0 = planes[i].d - thickness;
    let d1 = planes[j].d - thickness;
    let d2 = planes[k].d - thickness;
    let denom = n0.dot(&n1.cross(&n2));
    let num = n1.cross(&n2) * d0 + n2.cross(&n0) * d1 + n0.cross(&n1) * d2;
    let x = num / denom;
    Ok(Point3::new(x.x, x.y, x.z))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::measure::solid_volume;
    use crate::primitives::{box_, extrude_polygon};

    #[test]
    fn rejects_non_positive_thickness() {
        let b = box_(Vec3::new(10.0, 10.0, 10.0));
        assert!(matches!(
            shell_planar(&b, 0.0),
            Err(ShellError::NonPositiveThickness(_))
        ));
        assert!(matches!(
            shell_planar(&b, -1.0),
            Err(ShellError::NonPositiveThickness(_))
        ));
    }

    #[test]
    fn rejects_curved_surface_input() {
        // sphere() returns an analytic sphere with a SurfaceKind::Sphere face.
        let s = crate::primitives::sphere(10.0);
        assert!(matches!(
            shell_planar(&s, 1.0),
            Err(ShellError::CurvedSurface)
        ));
    }

    #[test]
    fn shells_a_box_to_correct_volume() {
        // Box 40 × 30 × 20, wall 2 → outer 24000, inner 36 × 26 × 16 = 14976,
        // shell = 9024.
        let b = box_(Vec3::new(40.0, 30.0, 20.0));
        let shelled = shell_planar(&b, 2.0).expect("shell box");
        let v = solid_volume(&shelled);
        let expected = 40.0 * 30.0 * 20.0 - 36.0 * 26.0 * 16.0;
        // 1% relative tolerance per the task spec.
        let rel = (v - expected).abs() / expected;
        assert!(rel < 0.01, "shell box volume {} != expected {} (rel={})", v, expected, rel);
    }

    #[test]
    fn shells_a_hex_prism_to_correct_volume() {
        // Hexagonal prism, side length 10, height 5. Wall thickness 1.
        // Hex area = 3√3/2 · s² = 3√3/2 · 100 ≈ 259.808.
        // Inward offset of a regular hexagon by t along each face's inward
        // normal yields a smaller regular hexagon with the SAME apothem
        // reduction = t. Apothem of the original = s · √3/2.
        // New apothem = old - t, new side = (apothem_new) · 2/√3.
        let s = 10.0_f64;
        let h = 5.0_f64;
        let t = 1.0_f64;
        let mut profile: Vec<Point3> = Vec::with_capacity(6);
        for i in 0..6 {
            let theta = (i as f64) * std::f64::consts::TAU / 6.0;
            profile.push(Point3::new(s * theta.cos(), s * theta.sin(), 0.0));
        }
        let prism = extrude_polygon(&profile, Vec3::new(0.0, 0.0, h));
        let shelled = shell_planar(&prism, t).expect("shell hex prism");
        let v = solid_volume(&shelled);
        // Outer volume: hex area · h.
        let hex_area = 3.0 * 3.0_f64.sqrt() / 2.0 * s * s;
        let outer_v = hex_area * h;
        // Inner: apothem reduces by t (top/bottom inset) AND outer side faces
        // inset by t means side_apothem reduces by t — same result for a
        // regular hexagon because each side face's inward normal is radial.
        // So inner side = s · (1 - t / (s·√3/2)).
        let apothem = s * 3.0_f64.sqrt() / 2.0;
        let new_side = s * (1.0 - t / apothem);
        let inner_hex_area = 3.0 * 3.0_f64.sqrt() / 2.0 * new_side * new_side;
        let inner_v = inner_hex_area * (h - 2.0 * t);
        let expected = outer_v - inner_v;
        let rel = (v - expected).abs() / expected;
        assert!(
            rel < 0.02,
            "shell hex prism volume {} vs expected {} (rel={})",
            v,
            expected,
            rel
        );
    }

    #[test]
    fn shell_topology_face_count_doubles_for_box() {
        // Outer box has 6 faces; shell should have 6 outer + 6 inner walls
        // (with no holes — it's a fully-enclosed double-walled cube).
        let b = box_(Vec3::new(20.0, 20.0, 20.0));
        let shelled = shell_planar(&b, 1.0).expect("shell box");
        // After difference of two boxes (one inside the other, non-touching),
        // the result is one solid with 12 faces (6 outer + 6 inner).
        assert_eq!(shelled.face_count(), 12);
    }
}
