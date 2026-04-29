//! Line–Sphere intersection.

use super::CurveSurfaceIntersection;
use crate::curves::Line;
use crate::intersect::poly::solve_quadratic;
use crate::surface::Surface;
use crate::surfaces::Sphere;
use crate::tolerance::Tolerance;

pub fn intersect_line_sphere(
    line: &Line,
    sph: &Sphere,
    _tol: &Tolerance,
) -> CurveSurfaceIntersection {
    let q = line.origin - sph.frame.origin;
    let b = 2.0 * q.dot(&line.direction);
    let c = q.dot(&q) - sph.radius * sph.radius;
    let mut points = Vec::new();
    for t in solve_quadratic(1.0, b, c) {
        let p = line.origin + t * line.direction;
        let ((u, v), _) = sph.project(p);
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
    use crate::types::{Point3, Vec3};
    use approx::assert_relative_eq;

    #[test]
    fn line_through_center_yields_two_antipodal_points() {
        let line = Line::from_origin_dir(Point3::new(-2.0, 0.0, 0.0), Vec3::x()).unwrap();
        let sphere = Sphere::at_origin(1.0);
        if let CurveSurfaceIntersection::Points(pts) =
            intersect_line_sphere(&line, &sphere, &Tolerance::default())
        {
            assert_eq!(pts.len(), 2);
            assert_relative_eq!(pts[0].2.x, -1.0, epsilon = 1e-12);
            assert_relative_eq!(pts[1].2.x, 1.0, epsilon = 1e-12);
        } else {
            panic!();
        }
    }

    #[test]
    fn tangent_line_yields_one_point() {
        let line = Line::from_origin_dir(Point3::new(-2.0, 1.0, 0.0), Vec3::x()).unwrap();
        let sphere = Sphere::at_origin(1.0);
        if let CurveSurfaceIntersection::Points(pts) =
            intersect_line_sphere(&line, &sphere, &Tolerance::default())
        {
            assert_eq!(pts.len(), 1);
            assert_relative_eq!(pts[0].2, Point3::new(0.0, 1.0, 0.0), epsilon = 1e-9);
        } else {
            panic!();
        }
    }

    #[test]
    fn line_missing_sphere_is_empty() {
        let line = Line::from_origin_dir(Point3::new(-2.0, 5.0, 0.0), Vec3::x()).unwrap();
        let sphere = Sphere::at_origin(1.0);
        assert!(matches!(
            intersect_line_sphere(&line, &sphere, &Tolerance::default()),
            CurveSurfaceIntersection::Empty
        ));
    }
}
