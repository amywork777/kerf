//! Line–Line intersection in 3D.

use super::CurveCurveIntersection;
use crate::curves::Line;
use crate::tolerance::Tolerance;
use crate::types::Point3;

pub fn intersect_line_line(a: &Line, b: &Line, tol: &Tolerance) -> CurveCurveIntersection {
    let u = a.direction;
    let v = b.direction;
    let w = a.origin - b.origin;
    let uu = u.dot(&u);
    let uv = u.dot(&v);
    let vv = v.dot(&v);
    let uw = u.dot(&w);
    let vw = v.dot(&w);
    let det = uu * vv - uv * uv;
    if det.abs() < tol.angle_eq * tol.angle_eq {
        let proj = (a.origin - b.origin) - ((a.origin - b.origin).dot(&v)) * v;
        if proj.norm() < tol.point_eq {
            return CurveCurveIntersection::Coincident;
        }
        return CurveCurveIntersection::Empty;
    }
    let t = (uv * vw - vv * uw) / det;
    let s = (uu * vw - uv * uw) / det;
    let p_a = a.origin + t * u;
    let p_b = b.origin + s * v;
    if (p_a - p_b).norm() <= tol.point_eq {
        let mid = Point3::from((p_a.coords + p_b.coords) / 2.0);
        return CurveCurveIntersection::Points(vec![(t, s, mid)]);
    }
    CurveCurveIntersection::Empty
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::Vec3;
    use approx::assert_relative_eq;

    fn line(o: Point3, d: Vec3) -> Line {
        Line::from_origin_dir(o, d).unwrap()
    }

    #[test]
    fn intersecting_lines_at_origin() {
        let a = line(Point3::new(-1.0, 0.0, 0.0), Vec3::x());
        let b = line(Point3::new(0.0, -1.0, 0.0), Vec3::y());
        let tol = Tolerance::default();
        match intersect_line_line(&a, &b, &tol) {
            CurveCurveIntersection::Points(pts) => {
                assert_eq!(pts.len(), 1);
                assert_relative_eq!(pts[0].2, Point3::origin(), epsilon = 1e-12);
                assert_relative_eq!(pts[0].0, 1.0, epsilon = 1e-12);
                assert_relative_eq!(pts[0].1, 1.0, epsilon = 1e-12);
            }
            other => panic!("expected single point, got {other:?}"),
        }
    }

    #[test]
    fn parallel_disjoint_lines_are_empty() {
        let a = line(Point3::origin(), Vec3::x());
        let b = line(Point3::new(0.0, 1.0, 0.0), Vec3::x());
        let tol = Tolerance::default();
        assert!(matches!(
            intersect_line_line(&a, &b, &tol),
            CurveCurveIntersection::Empty
        ));
    }

    #[test]
    fn colinear_lines_are_coincident() {
        let a = line(Point3::origin(), Vec3::x());
        let b = line(Point3::new(5.0, 0.0, 0.0), Vec3::x());
        let tol = Tolerance::default();
        assert!(matches!(
            intersect_line_line(&a, &b, &tol),
            CurveCurveIntersection::Coincident
        ));
    }

    #[test]
    fn skew_lines_with_positive_distance_are_empty() {
        let a = line(Point3::origin(), Vec3::x());
        let b = line(Point3::new(0.0, 1.0, 0.0), Vec3::z());
        let tol = Tolerance::default();
        assert!(matches!(
            intersect_line_line(&a, &b, &tol),
            CurveCurveIntersection::Empty
        ));
    }
}
