//! Line–Cylinder intersection.

use super::CurveSurfaceIntersection;
use crate::curves::Line;
use crate::intersect::poly::solve_quadratic;
use crate::surface::Surface;
use crate::surfaces::Cylinder;
use crate::tolerance::Tolerance;

pub fn intersect_line_cylinder(
    line: &Line,
    cyl: &Cylinder,
    tol: &Tolerance,
) -> CurveSurfaceIntersection {
    let frame = cyl.frame;
    let r = cyl.radius;
    let q = line.origin - frame.origin;
    let qx = q.dot(&frame.x);
    let qy = q.dot(&frame.y);
    let dx = line.direction.dot(&frame.x);
    let dy = line.direction.dot(&frame.y);
    let a = dx * dx + dy * dy;
    let b = 2.0 * (qx * dx + qy * dy);
    let c = qx * qx + qy * qy - r * r;
    if a < tol.angle_eq * tol.angle_eq {
        if c.abs() < tol.point_eq {
            return CurveSurfaceIntersection::OnSurface;
        }
        return CurveSurfaceIntersection::Empty;
    }
    let mut points = Vec::new();
    for t in solve_quadratic(a, b, c) {
        let p = line.origin + t * line.direction;
        let ((u, v), _) = cyl.project(p);
        points.push((t, (u, v), p));
    }
    if points.is_empty() {
        CurveSurfaceIntersection::Empty
    } else {
        CurveSurfaceIntersection::Points(points)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::{Frame, Point3, Vec3};
    use approx::assert_relative_eq;

    fn unit_cyl_z() -> Cylinder {
        Cylinder::new(Frame::world(Point3::origin()), 1.0)
    }

    #[test]
    fn line_through_diameter_yields_two_points() {
        let line = Line::from_origin_dir(Point3::new(-2.0, 0.0, 0.5), Vec3::x()).unwrap();
        match intersect_line_cylinder(&line, &unit_cyl_z(), &Tolerance::default()) {
            CurveSurfaceIntersection::Points(pts) => {
                assert_eq!(pts.len(), 2);
                let xs: Vec<f64> = pts.iter().map(|p| p.2.x).collect();
                assert_relative_eq!(xs[0], -1.0, epsilon = 1e-12);
                assert_relative_eq!(xs[1], 1.0, epsilon = 1e-12);
            }
            other => panic!("{other:?}"),
        }
    }

    #[test]
    fn tangent_line_yields_one_point() {
        let line = Line::from_origin_dir(Point3::new(-2.0, 1.0, 0.0), Vec3::x()).unwrap();
        let result = intersect_line_cylinder(&line, &unit_cyl_z(), &Tolerance::default());
        if let CurveSurfaceIntersection::Points(pts) = result {
            assert_eq!(pts.len(), 1);
            assert_relative_eq!(pts[0].2, Point3::new(0.0, 1.0, 0.0), epsilon = 1e-9);
        } else {
            panic!("expected one point");
        }
    }

    #[test]
    fn line_outside_axis_is_empty() {
        let line = Line::from_origin_dir(Point3::new(-2.0, 5.0, 0.0), Vec3::x()).unwrap();
        assert!(matches!(
            intersect_line_cylinder(&line, &unit_cyl_z(), &Tolerance::default()),
            CurveSurfaceIntersection::Empty
        ));
    }

    #[test]
    fn line_along_axis_inside_is_empty() {
        let line = Line::from_origin_dir(Point3::origin(), Vec3::z()).unwrap();
        assert!(matches!(
            intersect_line_cylinder(&line, &unit_cyl_z(), &Tolerance::default()),
            CurveSurfaceIntersection::Empty
        ));
    }
}
