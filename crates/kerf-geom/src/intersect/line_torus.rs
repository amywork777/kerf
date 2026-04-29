//! Line–Torus intersection (quartic).

use super::CurveSurfaceIntersection;
use crate::curves::Line;
use crate::intersect::poly::solve_quartic;
use crate::surface::Surface;
use crate::surfaces::Torus;
use crate::tolerance::Tolerance;

pub fn intersect_line_torus(
    line: &Line,
    tor: &Torus,
    _tol: &Tolerance,
) -> CurveSurfaceIntersection {
    let frame = tor.frame;
    let big_r = tor.major_radius;
    let small_r = tor.minor_radius;
    let q = line.origin - frame.origin;
    let qx = q.dot(&frame.x);
    let qy = q.dot(&frame.y);
    let qz = q.dot(&frame.z);
    let dx = line.direction.dot(&frame.x);
    let dy = line.direction.dot(&frame.y);
    let dz = line.direction.dot(&frame.z);

    let alpha = qx * qx + qy * qy + qz * qz + big_r * big_r - small_r * small_r;
    let beta = 2.0 * (qx * dx + qy * dy + qz * dz);
    let gamma = 1.0;
    let delta = qx * qx + qy * qy;
    let eps = 2.0 * (qx * dx + qy * dy);
    let zeta = dx * dx + dy * dy;
    let big_r2_4 = 4.0 * big_r * big_r;

    let a = gamma * gamma;
    let b = 2.0 * beta * gamma;
    let c = 2.0 * alpha * gamma + beta * beta - big_r2_4 * zeta;
    let d = 2.0 * alpha * beta - big_r2_4 * eps;
    let e = alpha * alpha - big_r2_4 * delta;

    let mut points = Vec::new();
    for t in solve_quartic(a, b, c, d, e) {
        let p = line.origin + t * line.direction;
        let ((u, v), _) = tor.project(p);
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

    fn ring() -> Torus {
        Torus::new(Frame::world(Point3::origin()), 3.0, 1.0)
    }

    #[test]
    fn line_through_hole_misses_tube() {
        let line = Line::from_origin_dir(Point3::new(0.0, 0.0, -5.0), Vec3::z()).unwrap();
        assert!(matches!(
            intersect_line_torus(&line, &ring(), &Tolerance::default()),
            CurveSurfaceIntersection::Empty
        ));
    }

    #[test]
    fn line_along_x_axis_yields_four_points() {
        // y=0,z=0 line intersects the ring at x = ±(R+r) = ±4 and x = ±(R-r) = ±2.
        let line = Line::from_origin_dir(Point3::new(-10.0, 0.0, 0.0), Vec3::x()).unwrap();
        if let CurveSurfaceIntersection::Points(pts) =
            intersect_line_torus(&line, &ring(), &Tolerance::default())
        {
            assert_eq!(pts.len(), 4);
            let xs: Vec<f64> = pts.iter().map(|p| p.2.x).collect();
            assert_relative_eq!(xs[0], -4.0, epsilon = 1e-7);
            assert_relative_eq!(xs[xs.len() - 1], 4.0, epsilon = 1e-7);
        } else {
            panic!();
        }
    }
}
