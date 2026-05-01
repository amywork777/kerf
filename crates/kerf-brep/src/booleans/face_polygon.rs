//! Walk a face's outer loop and extract the 3D positions of its vertices in order.

use kerf_geom::Point3;
use kerf_topo::FaceId;

use crate::Solid;

/// Returns the ordered list of vertex positions along the face's outer loop.
/// Returns `None` if the face does not exist or its outer loop is empty.
pub fn face_polygon(solid: &Solid, face: FaceId) -> Option<Vec<Point3>> {
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
