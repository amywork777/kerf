//! Circle–Plane intersection. Closed-form.

use crate::curves::{Circle, Line};
use crate::surface::Surface;
use crate::surfaces::Plane;
use crate::tolerance::Tolerance;
use super::{
    intersect_line_circle, CurveCurveIntersection, CurveSurfaceIntersection,
};

pub fn intersect_circle_plane(
    circle: &Circle,
    plane: &Plane,
    tol: &Tolerance,
) -> CurveSurfaceIntersection {
    let n1 = circle.frame.z;
    let n2 = plane.frame.z;
    let o1 = circle.frame.origin;
    let o2 = plane.frame.origin;

    if tol.directions_parallel(n1, n2) {
        let signed = (o1 - o2).dot(&n2);
        if signed.abs() < tol.point_eq {
            return CurveSurfaceIntersection::OnSurface;
        }
        return CurveSurfaceIntersection::Empty;
    }

    let dir = n1.cross(&n2).normalize();
    let perp = dir.cross(&n1);
    let denom = perp.dot(&n2);
    if denom.abs() < tol.angle_eq {
        return CurveSurfaceIntersection::Empty;
    }
    let s = ((o2 - o1).dot(&n2)) / denom;
    let p0 = o1 + s * perp;
    let line = match Line::from_origin_dir(p0, dir) {
        Some(l) => l,
        None => return CurveSurfaceIntersection::Empty,
    };

    match intersect_line_circle(&line, circle, tol) {
        CurveCurveIntersection::Empty => CurveSurfaceIntersection::Empty,
        CurveCurveIntersection::Coincident => CurveSurfaceIntersection::OnSurface,
        CurveCurveIntersection::Points(pts) => {
            let translated: Vec<_> = pts
                .into_iter()
                .map(|(_t_line, t_circle, p)| (t_circle, plane.project(p).0, p))
                .collect();
            CurveSurfaceIntersection::Points(translated)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::{Frame, Point3, Vec3};
    use approx::assert_relative_eq;

    fn unit_xy(at: Point3) -> Circle { Circle::new(Frame::world(at), 1.0) }

    #[test]
    fn circle_in_xy_plane_on_xy_surface() {
        let circle = unit_xy(Point3::origin());
        let plane = Plane::new(Frame::world(Point3::origin()));
        assert!(matches!(
            intersect_circle_plane(&circle, &plane, &Tolerance::default()),
            CurveSurfaceIntersection::OnSurface
        ));
    }

    #[test]
    fn circle_above_xy_plane_is_empty() {
        let circle = unit_xy(Point3::new(0.0, 0.0, 5.0));
        let plane = Plane::new(Frame::world(Point3::origin()));
        assert!(matches!(
            intersect_circle_plane(&circle, &plane, &Tolerance::default()),
            CurveSurfaceIntersection::Empty
        ));
    }

    #[test]
    fn circle_in_xz_plane_crossing_xy_yields_two_points() {
        let xz_frame = Frame::from_x_yhint(Point3::origin(), Vec3::x(), Vec3::z()).unwrap();
        let circle = Circle::new(xz_frame, 1.0);
        let plane = Plane::new(Frame::world(Point3::origin()));
        if let CurveSurfaceIntersection::Points(pts) =
            intersect_circle_plane(&circle, &plane, &Tolerance::default())
        {
            assert_eq!(pts.len(), 2);
            let xs: Vec<f64> = pts.iter().map(|p| p.2.x).collect();
            assert_relative_eq!(xs.iter().fold(f64::INFINITY, |a, &b| a.min(b)), -1.0, epsilon = 1e-9);
            assert_relative_eq!(xs.iter().fold(f64::NEG_INFINITY, |a, &b| a.max(b)), 1.0, epsilon = 1e-9);
        } else { panic!(); }
    }
}
