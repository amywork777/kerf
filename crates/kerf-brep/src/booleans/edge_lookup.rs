//! Find which edge of a face contains a given 3D point.

use kerf_geom::{Point3, Tolerance};
use kerf_topo::{EdgeId, FaceId, VertexId};

use crate::Solid;

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum PointLocation {
    OnVertex(VertexId),
    OnEdge { edge: EdgeId, t: f64 },
    Interior,
}

pub fn locate_point_on_face(
    solid: &Solid,
    face: FaceId,
    p: Point3,
    tol: &Tolerance,
) -> PointLocation {
    let f = match solid.topo.face(face) {
        Some(x) => x,
        None => return PointLocation::Interior,
    };
    let lp = match solid.topo.loop_(f.outer_loop()) {
        Some(x) => x,
        None => return PointLocation::Interior,
    };
    let start = match lp.half_edge() {
        Some(x) => x,
        None => return PointLocation::Interior,
    };

    let mut cur = start;
    loop {
        let he = solid.topo.half_edge(cur).unwrap();
        let v_start = he.origin();
        let twin = solid.topo.half_edge(he.twin()).unwrap();
        let v_end = twin.origin();

        let p_start = solid.vertex_geom.get(v_start).copied().unwrap();
        let p_end = solid.vertex_geom.get(v_end).copied().unwrap();

        // Vertex test first.
        if (p - p_start).norm() < tol.point_eq {
            return PointLocation::OnVertex(v_start);
        }
        if (p - p_end).norm() < tol.point_eq {
            return PointLocation::OnVertex(v_end);
        }

        // Edge interior: p = p_start + t*(p_end - p_start), t ∈ (0, 1) within tol.
        let edge_vec = p_end - p_start;
        let edge_len = edge_vec.norm();
        if edge_len > tol.point_eq {
            let edge_dir = edge_vec / edge_len;
            let t_dist = (p - p_start).dot(&edge_dir);
            if t_dist > tol.point_eq && t_dist < edge_len - tol.point_eq {
                let foot = p_start + t_dist * edge_dir;
                if (p - foot).norm() < tol.point_eq {
                    return PointLocation::OnEdge {
                        edge: he.edge(),
                        t: t_dist / edge_len,
                    };
                }
            }
        }

        cur = he.next();
        if cur == start {
            break;
        }
    }

    PointLocation::Interior
}

#[cfg(test)]
mod tests {
    use super::*;
    use kerf_geom::Vec3;

    use crate::booleans::face_polygon;
    use crate::primitives::box_;

    #[test]
    fn vertex_corner_is_recognized() {
        let s = box_(Vec3::new(1.0, 1.0, 1.0));
        // Find any face and check that one of its vertices is recognized as OnVertex.
        let face = s.topo.face_ids().next().unwrap();
        // Get the polygon to find a corner.
        let poly = face_polygon(&s, face).unwrap();
        let corner = poly[0];
        match locate_point_on_face(&s, face, corner, &Tolerance::default()) {
            PointLocation::OnVertex(_) => {}
            other => panic!("expected OnVertex, got {other:?}"),
        }
    }

    #[test]
    fn edge_midpoint_is_recognized() {
        let s = box_(Vec3::new(2.0, 2.0, 2.0));
        let face = s.topo.face_ids().next().unwrap();
        let poly = face_polygon(&s, face).unwrap();
        // Midpoint of first edge.
        let mid = Point3::from((poly[0].coords + poly[1].coords) / 2.0);
        match locate_point_on_face(&s, face, mid, &Tolerance::default()) {
            PointLocation::OnEdge { t, .. } => {
                assert!((t - 0.5).abs() < 1e-6, "t = {t}");
            }
            other => panic!("expected OnEdge, got {other:?}"),
        }
    }

    #[test]
    fn off_face_point_is_interior_or_off() {
        // A point clearly inside a face's interior (not on its boundary).
        let s = box_(Vec3::new(2.0, 2.0, 2.0));
        let face = s.topo.face_ids().next().unwrap();
        let poly = face_polygon(&s, face).unwrap();
        // Centroid is in the interior.
        let centroid = Point3::from(
            poly.iter().map(|p| p.coords).sum::<kerf_geom::Vec3>() / poly.len() as f64,
        );
        match locate_point_on_face(&s, face, centroid, &Tolerance::default()) {
            PointLocation::Interior => {}
            other => panic!("expected Interior, got {other:?}"),
        }
    }
}
