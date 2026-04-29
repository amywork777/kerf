//! Right circular cylinder.
//! Param u ∈ [0, TAU) is angle; v ∈ ℝ is height along the axis (`frame.z`).
//! point(u, v) = origin + r*cos(u)*x + r*sin(u)*y + v*z.

use crate::surface::{Domain2, Surface};
use crate::types::{Frame, Point3, Vec3};
use std::f64::consts::TAU;

#[derive(Clone, Copy, Debug)]
pub struct Cylinder {
    pub frame: Frame,
    pub radius: f64,
}

impl Cylinder {
    pub fn new(frame: Frame, radius: f64) -> Self {
        debug_assert!(radius > 0.0);
        Cylinder { frame, radius }
    }
}

impl Surface for Cylinder {
    fn point_at(&self, u: f64, v: f64) -> Point3 {
        let (s, c) = u.sin_cos();
        self.frame.origin
            + self.radius * c * self.frame.x
            + self.radius * s * self.frame.y
            + v * self.frame.z
    }

    fn normal_at(&self, u: f64, _v: f64) -> Vec3 {
        let (s, c) = u.sin_cos();
        c * self.frame.x + s * self.frame.y
    }

    fn domain(&self) -> Domain2 {
        ((0.0, TAU), (f64::NEG_INFINITY, f64::INFINITY))
    }

    fn project(&self, p: Point3) -> ((f64, f64), Point3) {
        let (lx, ly, lz) = self.frame.local_of(p);
        let mut u = ly.atan2(lx);
        if u < 0.0 {
            u += TAU;
        }
        ((u, lz), self.point_at(u, lz))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    fn unit_cyl_z() -> Cylinder {
        Cylinder::new(Frame::world(Point3::origin()), 1.0)
    }

    #[test]
    fn point_at_zero_lies_on_x_axis_at_radius() {
        let c = unit_cyl_z();
        assert_relative_eq!(c.point_at(0.0, 0.0), Point3::new(1.0, 0.0, 0.0));
        assert_relative_eq!(c.point_at(0.0, 7.0), Point3::new(1.0, 0.0, 7.0));
    }

    #[test]
    fn normal_is_unit_and_perpendicular_to_axis() {
        let c = unit_cyl_z();
        for u in [0.0, 1.1, 3.0, 5.5] {
            let n = c.normal_at(u, 100.0);
            assert_relative_eq!(n.norm(), 1.0, epsilon = 1e-12);
            assert_relative_eq!(n.dot(&Vec3::z()), 0.0, epsilon = 1e-12);
        }
    }

    #[test]
    fn project_round_trips() {
        let c = unit_cyl_z();
        for (u, v) in [(0.5, -3.0), (2.0, 0.0), (5.5, 4.0)] {
            let p = c.point_at(u, v);
            let ((u2, v2), p2) = c.project(p);
            assert_relative_eq!(u2, u, epsilon = 1e-12);
            assert_relative_eq!(v2, v, epsilon = 1e-12);
            assert_relative_eq!(p2, p, epsilon = 1e-12);
        }
    }
}
