//! Circle in 3D: a frame and a radius. Parameter `t` is the angle in radians.

use crate::curve::Curve;
use crate::types::{Frame, Point3, Vec3};
use std::f64::consts::TAU;

#[derive(Clone, Copy, Debug)]
pub struct Circle {
    pub frame: Frame,
    pub radius: f64,
}

impl Circle {
    pub fn new(frame: Frame, radius: f64) -> Self {
        debug_assert!(radius > 0.0, "Circle radius must be positive");
        Circle { frame, radius }
    }
}

impl Curve for Circle {
    fn point_at(&self, t: f64) -> Point3 {
        let (s, c) = t.sin_cos();
        self.frame.origin + self.radius * (c * self.frame.x + s * self.frame.y)
    }

    fn tangent_at(&self, t: f64) -> Vec3 {
        let (s, c) = t.sin_cos();
        -s * self.frame.x + c * self.frame.y
    }

    fn domain(&self) -> (f64, f64) {
        (0.0, TAU)
    }

    fn project(&self, p: Point3) -> (f64, Point3) {
        let (lx, ly, _lz) = self.frame.local_of(p);
        let mut t = ly.atan2(lx);
        if t < 0.0 {
            t += TAU;
        }
        (t, self.point_at(t))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use std::f64::consts::FRAC_PI_2;

    fn unit_xy_circle() -> Circle {
        Circle::new(Frame::world(Point3::origin()), 1.0)
    }

    #[test]
    fn point_at_traces_unit_circle_in_xy() {
        let c = unit_xy_circle();
        assert_relative_eq!(c.point_at(0.0), Point3::new(1.0, 0.0, 0.0));
        assert_relative_eq!(
            c.point_at(FRAC_PI_2),
            Point3::new(0.0, 1.0, 0.0),
            epsilon = 1e-12
        );
    }

    #[test]
    fn tangent_is_unit_and_perpendicular_to_radius() {
        let c = unit_xy_circle();
        for t in [0.0, 0.7, 2.5, 5.1] {
            let tangent = c.tangent_at(t);
            assert_relative_eq!(tangent.norm(), 1.0, epsilon = 1e-12);
            let radius_vec = c.point_at(t) - c.frame.origin;
            assert_relative_eq!(radius_vec.dot(&tangent), 0.0, epsilon = 1e-12);
        }
    }

    #[test]
    fn project_round_trips_via_point_at() {
        let c = unit_xy_circle();
        for t in [0.1, 1.0, 3.0, 5.5] {
            let p = c.point_at(t);
            let (t2, p2) = c.project(p);
            assert_relative_eq!(t2, t, epsilon = 1e-9);
            assert_relative_eq!(p2, p, epsilon = 1e-12);
        }
    }

    #[test]
    fn project_external_point_lands_on_nearest_circle_point() {
        let c = unit_xy_circle();
        let (t, p) = c.project(Point3::new(5.0, 0.0, 9.9));
        assert_relative_eq!(t, 0.0, epsilon = 1e-12);
        assert_relative_eq!(p, Point3::new(1.0, 0.0, 0.0));
    }

    #[test]
    fn domain_is_zero_to_tau() {
        let c = unit_xy_circle();
        let (a, b) = c.domain();
        assert_eq!(a, 0.0);
        assert_relative_eq!(b, TAU);
    }
}
