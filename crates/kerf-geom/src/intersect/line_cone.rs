//! Line–Cone intersection.

use super::CurveSurfaceIntersection;
use crate::curves::Line;
use crate::intersect::poly::solve_quadratic;
use crate::surface::Surface;
use crate::surfaces::Cone;
use crate::tolerance::Tolerance;

pub fn intersect_line_cone(line: &Line, cone: &Cone, tol: &Tolerance) -> CurveSurfaceIntersection {
    let frame = cone.frame;
    let q = line.origin - frame.origin;
    let qx = q.dot(&frame.x);
    let qy = q.dot(&frame.y);
    let qz = q.dot(&frame.z);
    let dx = line.direction.dot(&frame.x);
    let dy = line.direction.dot(&frame.y);
    let dz = line.direction.dot(&frame.z);
    let t2 = cone.half_angle.tan().powi(2);
    let a = dx * dx + dy * dy - t2 * dz * dz;
    let b = 2.0 * (qx * dx + qy * dy - t2 * qz * dz);
    let c = qx * qx + qy * qy - t2 * qz * qz;
    if a.abs() < 1e-14 && b.abs() < tol.angle_eq {
        if c.abs() < tol.point_eq {
            return CurveSurfaceIntersection::OnSurface;
        }
        return CurveSurfaceIntersection::Empty;
    }
    let mut points = Vec::new();
    for t in solve_quadratic(a, b, c) {
        let p = line.origin + t * line.direction;
        let ((u, v), _) = cone.project(p);
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
    use std::f64::consts::FRAC_PI_4;

    fn pi4_cone() -> Cone {
        Cone::new(Frame::world(Point3::origin()), FRAC_PI_4)
    }

    #[test]
    fn horizontal_line_above_apex_yields_two_points() {
        let line = Line::from_origin_dir(Point3::new(-5.0, 0.0, 2.0), Vec3::x()).unwrap();
        if let CurveSurfaceIntersection::Points(pts) =
            intersect_line_cone(&line, &pi4_cone(), &Tolerance::default())
        {
            assert_eq!(pts.len(), 2);
            let xs: Vec<f64> = pts.iter().map(|p| p.2.x).collect();
            assert_relative_eq!(xs[0], -2.0, epsilon = 1e-9);
            assert_relative_eq!(xs[1], 2.0, epsilon = 1e-9);
        } else {
            panic!();
        }
    }

    #[test]
    fn line_through_apex_along_generator_is_one_or_on_surface() {
        let line = Line::from_origin_dir(Point3::origin(), Vec3::new(1.0, 0.0, 1.0)).unwrap();
        let result = intersect_line_cone(&line, &pi4_cone(), &Tolerance::default());
        match result {
            CurveSurfaceIntersection::OnSurface => {}
            CurveSurfaceIntersection::Points(pts) => {
                assert!(pts.iter().any(|p| (p.2 - Point3::origin()).norm() < 1e-9));
            }
            other => panic!("{other:?}"),
        }
    }

    #[test]
    fn line_missing_cone_is_empty() {
        let line = Line::from_origin_dir(Point3::new(-2.0, 100.0, 0.0), Vec3::x()).unwrap();
        assert!(matches!(
            intersect_line_cone(&line, &pi4_cone(), &Tolerance::default()),
            CurveSurfaceIntersection::Empty
        ));
    }
}
