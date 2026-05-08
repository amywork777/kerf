//! Geometric measurement: signed volume, per-shell volume, area.
//!
//! These are *correctness checks* — the Solid's faces must walk CCW from
//! their outward normals (M40 winding normalization), and the kernel must
//! produce closed manifolds. Volume of an open mesh is meaningless.

use kerf_geom::Point3;
use kerf_topo::{EdgeId, FaceId, ShellId};

use crate::geometry::CurveKind;
use crate::Solid;

/// Signed volume of a closed manifold via the divergence theorem on
/// fan-triangulated faces. Each triangle contributes (1/6) * (p0 · p1 × p2)
/// where p0/p1/p2 are world-space vertex positions. Positive when faces walk
/// CCW from their OUTWARD normals (the kerf convention, enforced by
/// face_polygon's M40 winding normalization).
pub fn solid_volume(s: &Solid) -> f64 {
    let mut total = 0.0;
    for face_id in s.topo.face_ids() {
        total += face_signed_volume(s, face_id);
    }
    total
}

/// Volume of a single connected shell. A valid Solid may contain multiple
/// disjoint shells (e.g., union of disjoint inputs); each must enclose a
/// positive-signed volume on its own.
pub fn shell_volume(s: &Solid, shell: ShellId) -> f64 {
    let Some(sh) = s.topo.shell(shell) else {
        return 0.0;
    };
    let mut total = 0.0;
    for &face_id in sh.faces() {
        total += face_signed_volume(s, face_id);
    }
    total
}

fn face_signed_volume(s: &Solid, face_id: FaceId) -> f64 {
    // Walk the face's outer loop DIRECTLY (no winding normalization). The
    // loop order encodes the result-face's outward orientation: stitch
    // preserves input polygon order, and input polygons are normalized to
    // CCW-from-result-outward (M40 face_polygon winding fix for stitch input,
    // plus the kept-face flip in pipeline.rs for DIFF B-faces). For
    // solid_volume on a stitched result, reading the loop directly gives
    // CCW-from-result-outward — what the divergence-theorem formula needs.
    //
    // (Going through face_polygon would re-normalize against the original
    // surface frame.z, which for DIFF-flipped faces points the wrong way and
    // produces a volume with the wrong sign.)
    let Some(face) = s.topo.face(face_id) else {
        return 0.0;
    };
    let Some(lp) = s.topo.loop_(face.outer_loop()) else {
        return 0.0;
    };
    let Some(start) = lp.half_edge() else {
        return 0.0;
    };
    let mut polygon: Vec<Point3> = Vec::new();
    let mut cur = start;
    loop {
        let Some(he) = s.topo.half_edge(cur) else {
            return 0.0;
        };
        let v = he.origin();
        let Some(p) = s.vertex_geom.get(v) else {
            return 0.0;
        };
        polygon.push(*p);
        cur = he.next();
        if cur == start {
            break;
        }
    }
    if polygon.len() < 3 {
        return 0.0;
    }
    let mut v = 0.0;
    let p0 = polygon[0].coords;
    for i in 1..polygon.len() - 1 {
        let pi = polygon[i].coords;
        let pj = polygon[i + 1].coords;
        let cross = pi.cross(&pj);
        v += p0.dot(&cross);
    }
    v / 6.0
}

/// Walk a face's outer loop and return the distinct edge IDs along its
/// boundary, in loop order (no de-dup of repeated edges via self-loop).
///
/// Used by the WASM picking layer to map a clicked face to the edge buttons
/// the user can press to apply a fillet.
pub fn face_boundary_edges(s: &Solid, face_id: FaceId) -> Vec<EdgeId> {
    let mut edges: Vec<EdgeId> = Vec::new();
    let Some(face) = s.topo.face(face_id) else {
        return edges;
    };
    let Some(lp) = s.topo.loop_(face.outer_loop()) else {
        return edges;
    };
    let Some(start) = lp.half_edge() else {
        return edges;
    };
    let mut cur = start;
    loop {
        let Some(he) = s.topo.half_edge(cur) else {
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

/// One edge's geometry as needed by the picking layer: endpoints, length,
/// and a string tag describing the curve kind ("line" | "circle" | "ellipse").
#[derive(Clone, Debug)]
pub struct EdgeInfo {
    pub p_start: [f64; 3],
    pub p_end: [f64; 3],
    pub length: f64,
    pub curve_kind: &'static str,
}

/// Look up the geometric endpoints + length + curve kind of an edge. The
/// endpoints come from the half-edge origins (start of forward half-edge,
/// start of its twin → end of forward half-edge). Length is straight-line
/// distance for lines and approximate (chord-based via curve `range`) for
/// circles/ellipses.
pub fn edge_info(s: &Solid, edge_id: EdgeId) -> Option<EdgeInfo> {
    let edge = s.topo.edge(edge_id)?;
    let [he_id_a, he_id_b] = edge.half_edges();
    let he_a = s.topo.half_edge(he_id_a)?;
    let he_b = s.topo.half_edge(he_id_b)?;
    let v_start = he_a.origin();
    let v_end = he_b.origin();
    let p_start = *s.vertex_geom.get(v_start)?;
    let p_end = *s.vertex_geom.get(v_end)?;
    let curve_kind = match s.edge_geom.get(edge_id).map(|seg| &seg.curve) {
        Some(CurveKind::Line(_)) => "line",
        Some(CurveKind::Circle(_)) => "circle",
        Some(CurveKind::Ellipse(_)) => "ellipse",
        None => "unknown",
    };
    let length = (p_end - p_start).norm();
    Some(EdgeInfo {
        p_start: [p_start.x, p_start.y, p_start.z],
        p_end: [p_end.x, p_end.y, p_end.z],
        length,
        curve_kind,
    })
}

/// Detect if an edge is a (nearly) axis-aligned straight line, and if so
/// return the axis name ("x" | "y" | "z") and the lower-coord endpoint
/// ("edge_min" in Fillet terminology) plus its length along the axis.
///
/// Tolerance: 1e-6 along the off-axis components. Rationale: axis-aligned
/// primitives in this kernel are constructed exactly, but composite features
/// can carry ~1e-9 noise from boolean jitter retries, so 1e-6 leaves margin.
pub fn axis_aligned_line_edge(s: &Solid, edge_id: EdgeId) -> Option<(&'static str, [f64; 3], f64)> {
    let info = edge_info(s, edge_id)?;
    if info.curve_kind != "line" {
        return None;
    }
    let dx = info.p_end[0] - info.p_start[0];
    let dy = info.p_end[1] - info.p_start[1];
    let dz = info.p_end[2] - info.p_start[2];
    const TOL: f64 = 1e-6;
    let (axis, axis_idx, len) = if dy.abs() < TOL && dz.abs() < TOL && dx.abs() > TOL {
        ("x", 0, dx.abs())
    } else if dx.abs() < TOL && dz.abs() < TOL && dy.abs() > TOL {
        ("y", 1, dy.abs())
    } else if dx.abs() < TOL && dy.abs() < TOL && dz.abs() > TOL {
        ("z", 2, dz.abs())
    } else {
        return None;
    };
    let mut edge_min = info.p_start;
    if info.p_end[axis_idx] < edge_min[axis_idx] {
        edge_min = info.p_end;
    }
    Some((axis, edge_min, len))
}

/// For an edge that is axis-aligned, infer the body's quadrant relative to
/// that edge — which way the solid extends in the two perpendicular axes.
///
/// We sample a point a tiny offset away from the edge along each of the four
/// perpendicular diagonals (++, +-, -+, --) and pick the one closest to the
/// midpoint of the *adjacent* face polygons (which lie on the body
/// boundary). The quadrant is encoded as a 2-char string in the kerf-cad
/// `Fillet` convention: each char is "p" (positive) or "n" (negative), in
/// canonical (a, b) order for the axis (axis x → (y, z); axis y → (z, x);
/// axis z → (x, y)).
///
/// Returns `None` if neighbour faces can't be found or all four candidates
/// are equally far (e.g. the edge isn't actually a body convex corner).
pub fn quadrant_hint_for_axis_edge(
    s: &Solid,
    edge_id: EdgeId,
    axis: &str,
) -> Option<String> {
    use kerf_geom::Vec3;

    let edge = s.topo.edge(edge_id)?;
    let [he_a, he_b] = edge.half_edges();
    // The two adjacent faces are the faces of the loops of the two
    // half-edges. For a manifold solid edge, these are exactly two distinct
    // faces.
    let face_a = {
        let he = s.topo.half_edge(he_a)?;
        let lp = s.topo.loop_(he.loop_())?;
        lp.face()
    };
    let face_b = {
        let he = s.topo.half_edge(he_b)?;
        let lp = s.topo.loop_(he.loop_())?;
        lp.face()
    };

    // Compute centroids of the two adjacent faces (in world space).
    let centroid_of = |fid: FaceId| -> Option<Vec3> {
        let face = s.topo.face(fid)?;
        let lp = s.topo.loop_(face.outer_loop())?;
        let start = lp.half_edge()?;
        let mut sum = Vec3::new(0.0, 0.0, 0.0);
        let mut n = 0usize;
        let mut cur = start;
        loop {
            let he = s.topo.half_edge(cur)?;
            let p = s.vertex_geom.get(he.origin())?;
            sum += p.coords;
            n += 1;
            cur = he.next();
            if cur == start {
                break;
            }
        }
        if n == 0 {
            return None;
        }
        Some(sum / (n as f64))
    };

    let ca = centroid_of(face_a)?;
    let cb = centroid_of(face_b)?;
    // Average centroid as the "inward" reference: convex-edge body lies
    // toward the average of the two adjacent face centroids.
    let inward = (ca + cb) * 0.5;

    let info = edge_info(s, edge_id)?;
    // Edge midpoint (on axis): use start.
    let mid = [
        0.5 * (info.p_start[0] + info.p_end[0]),
        0.5 * (info.p_start[1] + info.p_end[1]),
        0.5 * (info.p_start[2] + info.p_end[2]),
    ];

    let (a_idx, b_idx) = match axis {
        "x" => (1usize, 2usize),
        "y" => (2usize, 0usize),
        "z" => (0usize, 1usize),
        _ => return None,
    };

    let da = inward[a_idx] - mid[a_idx];
    let db = inward[b_idx] - mid[b_idx];

    // Both components must have nonzero sign for a clean convex corner.
    const ZERO_TOL: f64 = 1e-9;
    if da.abs() < ZERO_TOL || db.abs() < ZERO_TOL {
        return None;
    }
    let a = if da > 0.0 { 'p' } else { 'n' };
    let b = if db > 0.0 { 'p' } else { 'n' };
    Some(format!("{a}{b}"))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::primitives::{box_, box_at};
    use kerf_geom::{Point3, Vec3};

    #[test]
    fn unit_box_has_volume_one() {
        let s = box_(Vec3::new(1.0, 1.0, 1.0));
        assert!((solid_volume(&s) - 1.0).abs() < 1e-9, "got {}", solid_volume(&s));
    }

    #[test]
    fn box_2x3x4_has_volume_24() {
        let s = box_(Vec3::new(2.0, 3.0, 4.0));
        assert!((solid_volume(&s) - 24.0).abs() < 1e-9);
    }

    #[test]
    fn translated_box_has_same_volume() {
        let a = box_at(Vec3::new(1.5, 1.5, 1.5), Point3::new(10.0, -5.0, 7.0));
        assert!((solid_volume(&a) - 1.5_f64.powi(3)).abs() < 1e-9);
    }

    #[test]
    fn empty_solid_has_zero_volume() {
        let s = Solid::new();
        assert_eq!(solid_volume(&s), 0.0);
    }

    #[test]
    fn shell_volume_of_box_matches_solid_volume() {
        let s = box_(Vec3::new(2.0, 2.0, 2.0));
        let total: f64 = s
            .topo
            .shell_ids()
            .map(|sh| shell_volume(&s, sh))
            .sum();
        assert!((total - 8.0).abs() < 1e-9);
        assert!((solid_volume(&s) - 8.0).abs() < 1e-9);
    }

    #[test]
    fn box_face_has_four_boundary_edges() {
        // Every face of a unit box is bounded by exactly 4 distinct edges.
        let s = box_(Vec3::new(1.0, 1.0, 1.0));
        for fid in s.topo.face_ids() {
            let edges = face_boundary_edges(&s, fid);
            assert_eq!(edges.len(), 4, "expected 4 boundary edges per box face");
        }
    }

    #[test]
    fn box_edge_info_lengths_match_extents() {
        // For a 2x3x4 box, the 12 edges split 4 per axis with lengths 2, 3, 4.
        let s = box_(Vec3::new(2.0, 3.0, 4.0));
        let mut counts = [0usize; 3];
        for eid in s.topo.edge_ids() {
            let (axis, _emin, len) = axis_aligned_line_edge(&s, eid)
                .expect("box edges should be axis-aligned lines");
            let i = match axis {
                "x" => 0,
                "y" => 1,
                "z" => 2,
                _ => panic!("unexpected axis"),
            };
            let expected = [2.0_f64, 3.0, 4.0][i];
            assert!((len - expected).abs() < 1e-9, "edge along {axis} has len {len}, expected {expected}");
            counts[i] += 1;
        }
        assert_eq!(counts, [4, 4, 4]);
    }

    #[test]
    fn quadrant_hint_for_box_corner_edge_is_pp() {
        // Take the +x +y +z corner edge of a unit-box-at-origin: the edge
        // where two faces meet along z, with body extending into -x, -y.
        // For axis "z", canonical perpendicular order is (x, y). The body
        // (at origin, extents 1) lies at smaller x and smaller y than the
        // (1, 1, *) edge, so the quadrant should be "nn".
        let s = box_at(Vec3::new(1.0, 1.0, 1.0), Point3::new(0.0, 0.0, 0.0));
        // Find the edge whose start is at (1, 1, 0) and end (1, 1, 1).
        let mut hits = 0;
        for eid in s.topo.edge_ids() {
            let info = edge_info(&s, eid).unwrap();
            let pa = info.p_start;
            let pb = info.p_end;
            let along_z = (pa[2] - pb[2]).abs() > 0.5;
            let at_pp = (pa[0] - 1.0).abs() < 1e-9
                && (pa[1] - 1.0).abs() < 1e-9
                && (pb[0] - 1.0).abs() < 1e-9
                && (pb[1] - 1.0).abs() < 1e-9;
            if along_z && at_pp {
                let q = quadrant_hint_for_axis_edge(&s, eid, "z").expect("should resolve quadrant");
                assert_eq!(q, "nn", "expected nn (body in -x -y from this corner edge)");
                hits += 1;
            }
        }
        assert_eq!(hits, 1, "expected exactly one edge at +x +y along z");
    }

    #[test]
    fn quadrant_hint_returns_some_for_every_box_edge() {
        // All 12 box edges are convex corners and should get a quadrant hint.
        let s = box_(Vec3::new(2.0, 3.0, 4.0));
        let mut total = 0usize;
        for eid in s.topo.edge_ids() {
            let (axis, _, _) = axis_aligned_line_edge(&s, eid).unwrap();
            let q = quadrant_hint_for_axis_edge(&s, eid, axis);
            assert!(q.is_some(), "every box edge should get a quadrant");
            assert_eq!(q.as_deref().unwrap().len(), 2);
            total += 1;
        }
        assert_eq!(total, 12);
    }
}
