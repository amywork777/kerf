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
    Some(polygon)
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
}
