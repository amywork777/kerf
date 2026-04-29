//! Line–Plane intersection.

use super::CurveSurfaceIntersection;
use crate::curves::Line;
use crate::surfaces::Plane;
use crate::tolerance::Tolerance;

pub fn intersect_line_plane(
    line: &Line,
    plane: &Plane,
    tol: &Tolerance,
) -> CurveSurfaceIntersection {
    let n = plane.frame.z;
    let dn = line.direction.dot(&n);
    let to_origin = plane.frame.origin - line.origin;
    let on = to_origin.dot(&n);
    if dn.abs() < tol.angle_eq {
        if on.abs() < tol.point_eq {
            return CurveSurfaceIntersection::OnSurface;
        }
        return CurveSurfaceIntersection::Empty;
    }
    let t = on / dn;
    let p = line.origin + t * line.direction;
    let (u, v, _) = plane.frame.local_of(p);
    CurveSurfaceIntersection::Points(vec![(t, (u, v), p)])
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::{Frame, Point3, Vec3};
    use approx::assert_relative_eq;

    #[test]
    fn line_through_xy_plane_at_origin() {
        let line = Line::from_origin_dir(Point3::new(0.0, 0.0, -1.0), Vec3::z()).unwrap();
        let plane = Plane::new(Frame::world(Point3::origin()));
        match intersect_line_plane(&line, &plane, &Tolerance::default()) {
            CurveSurfaceIntersection::Points(pts) => {
                assert_eq!(pts.len(), 1);
                assert_relative_eq!(pts[0].2, Point3::origin(), epsilon = 1e-12);
                assert_relative_eq!(pts[0].0, 1.0, epsilon = 1e-12);
            }
            other => panic!("{other:?}"),
        }
    }

    #[test]
    fn line_parallel_above_plane_is_empty() {
        let line = Line::from_origin_dir(Point3::new(0.0, 0.0, 1.0), Vec3::x()).unwrap();
        let plane = Plane::new(Frame::world(Point3::origin()));
        assert!(matches!(
            intersect_line_plane(&line, &plane, &Tolerance::default()),
            CurveSurfaceIntersection::Empty
        ));
    }

    #[test]
    fn line_in_plane_is_on_surface() {
        let line = Line::from_origin_dir(Point3::origin(), Vec3::x()).unwrap();
        let plane = Plane::new(Frame::world(Point3::origin()));
        assert!(matches!(
            intersect_line_plane(&line, &plane, &Tolerance::default()),
            CurveSurfaceIntersection::OnSurface
        ));
    }
}
