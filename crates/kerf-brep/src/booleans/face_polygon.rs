//! Walk a face's outer loop and extract the 3D positions of its vertices in order.

use kerf_geom::Point3;
use kerf_topo::FaceId;

use crate::Solid;
use crate::geometry::SurfaceKind;

/// Returns the ordered list of vertex positions along the face's outer loop,
/// normalized to CCW from the face's surface outward normal.
///
/// **For boolean splitter input** (the M40 motivation): after mef + mev
/// reorderings, the raw loop can walk CW from the face's frame.z. Stitch
/// requires consistent winding so chord edges twin properly across A's and
/// B's polygons; this normalization ensures CCW-from-frame.z for ALL faces
/// of pre-stitch primitives.
///
/// **NOT for geometry computations on boolean RESULTS**: in a stitched
/// result, cavity faces carry their original surface frame.z (= the
/// unflipped original normal) but their loop walks CCW from the OPPOSITE
/// direction (= away from the material). Normalizing them via frame.z would
/// FLIP the cavity faces back toward material — wrong for volume,
/// tessellation, etc. Use [`face_polygon_raw`] for those.
pub fn face_polygon(solid: &Solid, face: FaceId) -> Option<Vec<Point3>> {
    let mut polygon = face_polygon_raw(solid, face)?;
    if let Some(SurfaceKind::Plane(plane)) = solid.face_geom.get(face) {
        let n = plane.frame.z;
        let signed_area_x2 = polygon_signed_area_x2(&polygon, &n);
        if signed_area_x2 < -1e-12 {
            polygon.reverse();
        }
    }
    rotate_polygon_off_stinger_junction(&mut polygon);
    Some(polygon)
}

/// Rotate the polygon so `polygon[0]` is not adjacent to a stinger fold.
///
/// Fjord-style polygons (annular faces represented as a single CCW walk with
/// a return-trip "stinger" connecting outer and inner rings) carry duplicated
/// vertex positions at the stinger junction. A naive fan-triangulate from
/// such a junction emits zero-area triangles. Downstream consumers
/// (tessellators, area integrators, the readiness_quality test) are happier
/// with an apex chosen from a non-degenerate stretch of the polygon. This is
/// purely a starting-vertex rotation — the polygon's set of edges and
/// winding are unchanged.
fn rotate_polygon_off_stinger_junction(polygon: &mut Vec<Point3>) {
    let n = polygon.len();
    if n < 4 {
        return;
    }
    // A vertex is a "junction" if either of its incident triangles in a fan
    // from itself would have zero signed area in 3D — i.e. the next two
    // edges are collinear with the apex. Find the first non-junction vertex.
    let mut best: Option<usize> = None;
    for start in 0..n {
        let v0 = polygon[start].coords;
        let mut all_ok = true;
        for i in 1..n - 1 {
            let v1 = polygon[(start + i) % n].coords;
            let v2 = polygon[(start + i + 1) % n].coords;
            let area_x2 = (v1 - v0).cross(&(v2 - v0)).norm();
            if area_x2 < 1e-12 {
                all_ok = false;
                break;
            }
        }
        if all_ok {
            best = Some(start);
            break;
        }
    }
    if let Some(s) = best {
        if s != 0 {
            polygon.rotate_left(s);
        }
    }
}

/// Returns the ordered list of vertex positions along the face's outer loop
/// in the loop's natural traversal direction (NO winding normalization).
///
/// Use this for geometry computations on stitched results where cavity faces
/// have already been correctly oriented by the boolean pipeline (their loops
/// walk CCW from cavity-outward, even though their surface frame.z carries
/// the original pre-flip normal).
pub fn face_polygon_raw(solid: &Solid, face: FaceId) -> Option<Vec<Point3>> {
    let f = solid.topo.face(face)?;
    let lp = solid.topo.loop_(f.outer_loop())?;
    let start = lp.half_edge()?;
    let mut polygon = Vec::new();
    let mut cur = start;
    loop {
        let he = solid.topo.half_edge(cur)?;
        let v = he.origin();
        polygon.push(*solid.vertex_geom.get(v)?);
        cur = he.next();
        if cur == start {
            break;
        }
    }
    Some(polygon)
}

/// One step in a curve-aware polygon walk: the start vertex's 3D position,
/// plus the curve carrying the half-edge that leaves it.
///
/// Tier 3-lite (curved-surface stitch wiring): the legacy `face_polygon`
/// throws away curve identity by emitting only vertex positions. This step
/// type preserves the underlying `CurveSegment` (Line / Circle / Ellipse)
/// for downstream consumers — Tier 3 polygon walkers, arc-aware
/// triangulation, and tests that need to inspect the boolean result's
/// curved boundaries — without changing the legacy function's signature.
#[derive(Clone, Debug)]
pub struct LoopStep {
    pub origin: Point3,
    pub curve: Option<crate::geometry::CurveSegment>,
}

/// Walk a face's outer loop and emit (vertex, edge_curve) pairs.
///
/// `curve` is `None` only when the corresponding edge has no edge-geometry
/// (zero-length seam, vase apex, freshly-stitched degenerate edge). The
/// `curve` of the step at index `i` describes the segment from `origin[i]`
/// to `origin[(i+1) % n]`.
///
/// Tier 3-lite of the curved-surface stitch wiring: lets a polygon-walking
/// caller branch on `CurveKind::Line` vs `Circle`/`Ellipse` and emit
/// arc-aware geometry (e.g., a triangulator that samples cylinder seam
/// curves at finer resolution than line edges).
pub fn face_polygon_with_arcs(solid: &Solid, face: FaceId) -> Option<Vec<LoopStep>> {
    let f = solid.topo.face(face)?;
    let lp = solid.topo.loop_(f.outer_loop())?;
    let start = lp.half_edge()?;
    let mut steps: Vec<LoopStep> = Vec::new();
    let mut cur = start;
    loop {
        let he = solid.topo.half_edge(cur)?;
        let v = he.origin();
        let origin = *solid.vertex_geom.get(v)?;
        let curve = solid.edge_geom.get(he.edge()).cloned();
        steps.push(LoopStep { origin, curve });
        cur = he.next();
        if cur == start {
            break;
        }
    }
    Some(steps)
}

/// Signed area times 2 of a 3D polygon projected along the given normal.
/// Sum over edges of (p_i × p_{i+1}) · n. Positive when polygon is CCW
/// from the +n viewpoint.
fn polygon_signed_area_x2(polygon: &[Point3], n: &kerf_geom::Vec3) -> f64 {
    let len = polygon.len();
    if len < 3 {
        return 0.0;
    }
    let mut sum = 0.0;
    for i in 0..len {
        let pi = polygon[i].coords;
        let pj = polygon[(i + 1) % len].coords;
        // (pi × pj) · n  = scalar triple product component contribution.
        let cross_x = pi.y * pj.z - pi.z * pj.y;
        let cross_y = pi.z * pj.x - pi.x * pj.z;
        let cross_z = pi.x * pj.y - pi.y * pj.x;
        sum += cross_x * n.x + cross_y * n.y + cross_z * n.z;
    }
    sum
}

#[cfg(test)]
mod tests {
    use super::*;
    use kerf_geom::Vec3;

    use crate::primitives::box_;

    #[test]
    fn box_faces_each_have_4_vertices() {
        let s = box_(Vec3::new(1.0, 1.0, 1.0));
        let mut face_count = 0;
        for face_id in s.topo.face_ids() {
            let poly = face_polygon(&s, face_id).expect("face must have polygon");
            assert_eq!(poly.len(), 4, "every box face is a quad");
            face_count += 1;
        }
        assert_eq!(face_count, 6);
    }

    #[test]
    fn box_face_polygons_have_3_distinct_coordinates() {
        let s = box_(Vec3::new(1.0, 2.0, 3.0));
        for face_id in s.topo.face_ids() {
            let poly = face_polygon(&s, face_id).unwrap();
            let xs: Vec<f64> = poly.iter().map(|p| p.x).collect();
            let ys: Vec<f64> = poly.iter().map(|p| p.y).collect();
            let zs: Vec<f64> = poly.iter().map(|p| p.z).collect();
            let const_count = [
                xs.iter().all(|&x| (x - xs[0]).abs() < 1e-12),
                ys.iter().all(|&y| (y - ys[0]).abs() < 1e-12),
                zs.iter().all(|&z| (z - zs[0]).abs() < 1e-12),
            ]
            .iter()
            .filter(|&&b| b)
            .count();
            assert_eq!(const_count, 1, "face must have exactly one constant axis");
        }
    }

    #[test]
    fn face_polygon_with_arcs_recovers_arc_curve_for_arc_edge() {
        // Tier 3-lite: stitch a 2-face mesh whose shared edge is an arc.
        // face_polygon_with_arcs on either face must return a step whose
        // curve is `Ellipse` (the underlying conic) — proving the arc
        // identity threads end-to-end from KeptFace.edge_tags through
        // stitch's edge_geom assignment to the polygon walker.
        use crate::booleans::stitch::{EdgeCurveTag, KeptFace, stitch};
        use crate::geometry::{CurveKind, EllipseSegment, SurfaceKind};
        use kerf_geom::{Ellipse, Frame, Plane, Point3, Tolerance};

        let p0 = Point3::new(0.0, 0.0, 0.0);
        let p1 = Point3::new(1.0, 0.0, 0.0);
        let p2 = Point3::new(0.0, 1.0, 0.0);

        let arc = EllipseSegment::new(
            Ellipse::new(Frame::world(Point3::new(0.5, 0.0, 0.0)), 0.5, 0.5),
            0.0,
            std::f64::consts::PI,
        );
        let surf = SurfaceKind::Plane(Plane::new(Frame::world(Point3::origin())));

        let front = KeptFace {
            polygon: vec![p0, p1, p2],
            surface: surf.clone(),
            owner: None,
            edge_tags: Some(vec![
                EdgeCurveTag::Arc(0),
                EdgeCurveTag::Line,
                EdgeCurveTag::Line,
            ]),
            arc_segments: vec![arc.clone()],
        };
        let back = KeptFace {
            polygon: vec![p1, p0, p2],
            surface: surf,
            owner: None,
            edge_tags: Some(vec![
                EdgeCurveTag::Arc(0),
                EdgeCurveTag::Line,
                EdgeCurveTag::Line,
            ]),
            arc_segments: vec![arc],
        };
        let solid = stitch(&[front, back], &Tolerance::default());
        let mut found_ellipse_step = false;
        for face_id in solid.topo.face_ids() {
            let steps = face_polygon_with_arcs(&solid, face_id).unwrap();
            for step in &steps {
                if let Some(c) = &step.curve {
                    if matches!(c.curve, CurveKind::Ellipse(_)) {
                        found_ellipse_step = true;
                    }
                }
            }
        }
        assert!(
            found_ellipse_step,
            "expected at least one polygon-walker step backed by an Ellipse curve"
        );
    }
}
