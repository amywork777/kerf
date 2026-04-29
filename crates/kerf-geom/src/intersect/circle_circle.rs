//! Circle–Circle intersection (coplanar case only). Closed-form.

use crate::curve::Curve;
use crate::curves::Circle;
use crate::tolerance::Tolerance;
use super::CurveCurveIntersection;

pub fn intersect_circle_circle(
    a: &Circle,
    b: &Circle,
    tol: &Tolerance,
) -> CurveCurveIntersection {
    if !tol.directions_parallel(a.frame.z, b.frame.z) {
        return CurveCurveIntersection::Empty;
    }
    let center_offset = b.frame.origin - a.frame.origin;
    if center_offset.dot(&a.frame.z).abs() > tol.point_eq {
        return CurveCurveIntersection::Empty;
    }

    let d = center_offset.norm();
    if d < tol.point_eq && (a.radius - b.radius).abs() < tol.point_eq {
        return CurveCurveIntersection::Coincident;
    }

    let r1 = a.radius;
    let r2 = b.radius;
    if d > r1 + r2 + tol.point_eq || d < (r1 - r2).abs() - tol.point_eq {
        return CurveCurveIntersection::Empty;
    }
    if d < tol.point_eq {
        return CurveCurveIntersection::Empty;
    }

    let axis = center_offset / d;
    let alpha = (r1 * r1 - r2 * r2 + d * d) / (2.0 * d);
    let h2 = r1 * r1 - alpha * alpha;
    let h = if h2 < 0.0 { 0.0 } else { h2.sqrt() };
    let mid = a.frame.origin + alpha * axis;
    let perp = a.frame.z.cross(&axis);

    let mut hits = Vec::new();
    for sign in [1.0_f64, -1.0_f64] {
        let p = mid + sign * h * perp;
        let (t_a, _) = a.project(p);
        let (t_b, _) = b.project(p);
        hits.push((t_a, t_b, p));
        if h.abs() < tol.point_eq { break; }
    }
    CurveCurveIntersection::Points(hits)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::{Frame, Point3, Vec3};
    use approx::assert_relative_eq;

    fn unit_xy(at: Point3) -> Circle {
        Circle::new(Frame::world(at), 1.0)
    }

    #[test]
    fn two_overlapping_circles_in_xy_plane() {
        let a = unit_xy(Point3::origin());
        let b = unit_xy(Point3::new(1.0, 0.0, 0.0));
        if let CurveCurveIntersection::Points(pts) =
            intersect_circle_circle(&a, &b, &Tolerance::default())
        {
            assert_eq!(pts.len(), 2);
            for p in &pts {
                assert_relative_eq!(p.2.x, 0.5, epsilon = 1e-12);
                assert_relative_eq!(p.2.y.abs(), 0.75_f64.sqrt(), epsilon = 1e-12);
            }
        } else { panic!(); }
    }

    #[test]
    fn tangent_circles_yield_one_point() {
        let a = unit_xy(Point3::origin());
        let b = unit_xy(Point3::new(2.0, 0.0, 0.0));
        if let CurveCurveIntersection::Points(pts) =
            intersect_circle_circle(&a, &b, &Tolerance::default())
        {
            assert_eq!(pts.len(), 1);
            assert_relative_eq!(pts[0].2, Point3::new(1.0, 0.0, 0.0), epsilon = 1e-12);
        } else { panic!(); }
    }

    #[test]
    fn disjoint_circles_are_empty() {
        let a = unit_xy(Point3::origin());
        let b = unit_xy(Point3::new(5.0, 0.0, 0.0));
        assert!(matches!(
            intersect_circle_circle(&a, &b, &Tolerance::default()),
            CurveCurveIntersection::Empty
        ));
    }

    #[test]
    fn identical_circles_are_coincident() {
        let a = unit_xy(Point3::origin());
        let b = unit_xy(Point3::origin());
        assert!(matches!(
            intersect_circle_circle(&a, &b, &Tolerance::default()),
            CurveCurveIntersection::Coincident
        ));
    }

    #[test]
    fn non_coplanar_circles_return_empty_for_now() {
        let a = unit_xy(Point3::origin());
        let xz_frame = Frame::from_x_yhint(Point3::origin(), Vec3::x(), Vec3::z()).unwrap();
        let b = Circle::new(xz_frame, 1.0);
        assert!(matches!(
            intersect_circle_circle(&a, &b, &Tolerance::default()),
            CurveCurveIntersection::Empty
        ));
    }
}
