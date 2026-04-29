//! Line–Circle intersection in 3D. Closed-form.

use super::{CurveCurveIntersection, CurveSurfaceIntersection, line_plane::intersect_line_plane};
use crate::curve::Curve;
use crate::curves::{Circle, Line};
use crate::intersect::poly::solve_quadratic;
use crate::surfaces::Plane;
use crate::tolerance::Tolerance;
use std::f64::consts::TAU;

pub fn intersect_line_circle(
    line: &Line,
    circle: &Circle,
    tol: &Tolerance,
) -> CurveCurveIntersection {
    let plane = Plane::new(circle.frame);
    match intersect_line_plane(line, &plane, tol) {
        CurveSurfaceIntersection::Empty => CurveCurveIntersection::Empty,
        CurveSurfaceIntersection::Points(pts) => {
            let (t_line, _uv, p) = pts[0];
            let dist_to_center = (p - circle.frame.origin).norm();
            if (dist_to_center - circle.radius).abs() < tol.point_eq {
                let (t_circle, _) = circle.project(p);
                CurveCurveIntersection::Points(vec![(t_line, t_circle, p)])
            } else {
                CurveCurveIntersection::Empty
            }
        }
        CurveSurfaceIntersection::OnSurface => {
            let q = line.origin - circle.frame.origin;
            let qx = q.dot(&circle.frame.x);
            let qy = q.dot(&circle.frame.y);
            let dx = line.direction.dot(&circle.frame.x);
            let dy = line.direction.dot(&circle.frame.y);
            let a = dx * dx + dy * dy;
            let b = 2.0 * (qx * dx + qy * dy);
            let c = qx * qx + qy * qy - circle.radius * circle.radius;
            let mut hits = Vec::new();
            for t in solve_quadratic(a, b, c) {
                let p = line.origin + t * line.direction;
                let mut t_circle = (qy + t * dy).atan2(qx + t * dx);
                if t_circle < 0.0 {
                    t_circle += TAU;
                }
                hits.push((t, t_circle, p));
            }
            if hits.is_empty() {
                CurveCurveIntersection::Empty
            } else {
                CurveCurveIntersection::Points(hits)
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::{Frame, Point3, Vec3};
    use approx::assert_relative_eq;

    fn unit_xy_circle() -> Circle {
        Circle::new(Frame::world(Point3::origin()), 1.0)
    }

    #[test]
    fn line_through_xy_plane_inside_circle_yields_one_point() {
        let line = Line::from_origin_dir(Point3::new(0.0, 0.0, -1.0), Vec3::z()).unwrap();
        let result = intersect_line_circle(&line, &unit_xy_circle(), &Tolerance::default());
        assert!(matches!(result, CurveCurveIntersection::Empty));
    }

    #[test]
    fn line_through_xy_plane_on_circle_yields_one_point() {
        let line = Line::from_origin_dir(Point3::new(1.0, 0.0, -1.0), Vec3::z()).unwrap();
        if let CurveCurveIntersection::Points(pts) =
            intersect_line_circle(&line, &unit_xy_circle(), &Tolerance::default())
        {
            assert_eq!(pts.len(), 1);
            assert_relative_eq!(pts[0].2, Point3::new(1.0, 0.0, 0.0), epsilon = 1e-12);
        } else {
            panic!();
        }
    }

    #[test]
    fn coplanar_line_through_diameter_yields_two_points() {
        let line = Line::from_origin_dir(Point3::new(-2.0, 0.0, 0.0), Vec3::x()).unwrap();
        if let CurveCurveIntersection::Points(pts) =
            intersect_line_circle(&line, &unit_xy_circle(), &Tolerance::default())
        {
            assert_eq!(pts.len(), 2);
            let xs: Vec<f64> = pts.iter().map(|p| p.2.x).collect();
            assert_relative_eq!(xs[0], -1.0, epsilon = 1e-12);
            assert_relative_eq!(xs[1], 1.0, epsilon = 1e-12);
        } else {
            panic!();
        }
    }

    #[test]
    fn coplanar_line_missing_circle_is_empty() {
        let line = Line::from_origin_dir(Point3::new(-2.0, 5.0, 0.0), Vec3::x()).unwrap();
        assert!(matches!(
            intersect_line_circle(&line, &unit_xy_circle(), &Tolerance::default()),
            CurveCurveIntersection::Empty
        ));
    }
}
