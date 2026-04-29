//! Circle–Sphere intersection. Closed-form via sphere's plane cross-section.

use crate::curve::Curve;
use crate::curves::Circle;
use crate::surface::Surface;
use crate::surfaces::Sphere;
use crate::tolerance::Tolerance;
use crate::types::Frame;
use super::{
    intersect_circle_circle, CurveCurveIntersection, CurveSurfaceIntersection,
};

pub fn intersect_circle_sphere(
    circle: &Circle,
    sphere: &Sphere,
    tol: &Tolerance,
) -> CurveSurfaceIntersection {
    let cf = circle.frame;
    let h = (sphere.frame.origin - cf.origin).dot(&cf.z);
    let r_s = sphere.radius;

    if h.abs() > r_s + tol.point_eq {
        return CurveSurfaceIntersection::Empty;
    }

    if (h.abs() - r_s).abs() < tol.point_eq {
        let point = sphere.frame.origin - h * cf.z;
        let dist = (point - cf.origin).norm();
        if (dist - circle.radius).abs() < tol.point_eq {
            let (t_circle, _) = circle.project(point);
            return CurveSurfaceIntersection::Points(vec![(t_circle, sphere.project(point).0, point)]);
        }
        return CurveSurfaceIntersection::Empty;
    }

    let o_x = sphere.frame.origin - h * cf.z;
    let r_x = (r_s * r_s - h * h).sqrt();
    let cross_frame = Frame { origin: o_x, x: cf.x, y: cf.y, z: cf.z };
    let cross = Circle::new(cross_frame, r_x);

    match intersect_circle_circle(circle, &cross, tol) {
        CurveCurveIntersection::Empty => CurveSurfaceIntersection::Empty,
        CurveCurveIntersection::Coincident => CurveSurfaceIntersection::OnSurface,
        CurveCurveIntersection::Points(pts) => {
            let translated: Vec<_> = pts
                .into_iter()
                .map(|(t_circle, _t_other, p)| (t_circle, sphere.project(p).0, p))
                .collect();
            CurveSurfaceIntersection::Points(translated)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::{Frame, Point3};
    use approx::assert_relative_eq;

    fn unit_xy(at: Point3) -> Circle { Circle::new(Frame::world(at), 1.0) }

    #[test]
    fn circle_through_unit_sphere_in_xy_at_origin_is_on_surface() {
        let circle = unit_xy(Point3::origin());
        let sphere = Sphere::at_origin(1.0);
        assert!(matches!(
            intersect_circle_sphere(&circle, &sphere, &Tolerance::default()),
            CurveSurfaceIntersection::OnSurface
        ));
    }

    #[test]
    fn small_circle_inside_big_sphere_is_empty() {
        let circle = unit_xy(Point3::origin());
        let sphere = Sphere::at_origin(5.0);
        assert!(matches!(
            intersect_circle_sphere(&circle, &sphere, &Tolerance::default()),
            CurveSurfaceIntersection::Empty
        ));
    }

    #[test]
    fn circle_crossing_sphere_yields_two_points() {
        // Unit circle in xy plane, centered at (0.5, 0, 0). The sphere cross-section at
        // z=0 is also a unit circle at origin. These two coplanar circles overlap and
        // intersect at (0.25, ±sqrt(15)/4, 0), which lie on both circles and on the
        // unit sphere (distance from origin = sqrt(0.0625 + 0.9375) = 1).
        let circle = unit_xy(Point3::new(0.5, 0.0, 0.0));
        let sphere = Sphere::at_origin(1.0);
        if let CurveSurfaceIntersection::Points(pts) =
            intersect_circle_sphere(&circle, &sphere, &Tolerance::default())
        {
            assert_eq!(pts.len(), 2);
            for p in &pts {
                assert_relative_eq!(p.2.z, 0.0, epsilon = 1e-12);
                // Each point must lie on the unit sphere.
                let dist_from_origin = (p.2.x * p.2.x + p.2.y * p.2.y + p.2.z * p.2.z).sqrt();
                assert_relative_eq!(dist_from_origin, 1.0, epsilon = 1e-9);
                // Each point must lie on the input circle (distance from (0.5,0,0) = 1).
                let dx = p.2.x - 0.5;
                let dist_from_circle_center = (dx * dx + p.2.y * p.2.y).sqrt();
                assert_relative_eq!(dist_from_circle_center, 1.0, epsilon = 1e-9);
            }
        } else { panic!(); }
    }

    #[test]
    fn circle_above_sphere_is_empty() {
        let circle = unit_xy(Point3::new(0.0, 0.0, 10.0));
        let sphere = Sphere::at_origin(1.0);
        assert!(matches!(
            intersect_circle_sphere(&circle, &sphere, &Tolerance::default()),
            CurveSurfaceIntersection::Empty
        ));
    }
}
