//! Torus around `frame.z`: major radius R, minor radius r.
//! u ∈ [0, TAU) sweeps around the central axis; v ∈ [0, TAU) sweeps around the tube.
//! point(u, v) = center + (R + r*cos(v)) * (cos(u)*x + sin(u)*y) + r*sin(v)*z.

use crate::surface::{Domain2, Surface};
use crate::types::{Frame, Point3, Vec3};
use std::f64::consts::TAU;

#[derive(Clone, Copy, Debug)]
pub struct Torus {
    pub frame: Frame,
    pub major_radius: f64, // R, distance from axis to tube center
    pub minor_radius: f64, // r, tube radius
}

impl Torus {
    pub fn new(frame: Frame, major_radius: f64, minor_radius: f64) -> Self {
        debug_assert!(major_radius > 0.0 && minor_radius > 0.0);
        Torus {
            frame,
            major_radius,
            minor_radius,
        }
    }
}

impl Surface for Torus {
    fn point_at(&self, u: f64, v: f64) -> Point3 {
        let (su, cu) = u.sin_cos();
        let (sv, cv) = v.sin_cos();
        let big = self.major_radius + self.minor_radius * cv;
        self.frame.origin
            + big * cu * self.frame.x
            + big * su * self.frame.y
            + self.minor_radius * sv * self.frame.z
    }

    fn normal_at(&self, u: f64, v: f64) -> Vec3 {
        let (su, cu) = u.sin_cos();
        let (sv, cv) = v.sin_cos();
        cv * (cu * self.frame.x + su * self.frame.y) + sv * self.frame.z
    }

    fn domain(&self) -> Domain2 {
        ((0.0, TAU), (0.0, TAU))
    }

    fn project(&self, p: Point3) -> ((f64, f64), Point3) {
        let d = p - self.frame.origin;
        let (lx, ly, lz) = (
            d.dot(&self.frame.x),
            d.dot(&self.frame.y),
            d.dot(&self.frame.z),
        );
        let mut u = ly.atan2(lx);
        if u < 0.0 {
            u += TAU;
        }
        let big = (lx * lx + ly * ly).sqrt();
        let dr = big - self.major_radius;
        let mut v = lz.atan2(dr);
        if v < 0.0 {
            v += TAU;
        }
        ((u, v), self.point_at(u, v))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    fn ring() -> Torus {
        Torus::new(Frame::world(Point3::origin()), 3.0, 1.0)
    }

    #[test]
    fn point_at_zero_zero_lies_on_outer_equator() {
        let t = ring();
        assert_relative_eq!(t.point_at(0.0, 0.0), Point3::new(4.0, 0.0, 0.0));
    }

    #[test]
    fn normal_is_unit_for_all_uv() {
        let t = ring();
        for (u, v) in [(0.0, 0.0), (1.2, 2.0), (3.0, 4.5)] {
            assert_relative_eq!(t.normal_at(u, v).norm(), 1.0, epsilon = 1e-12);
        }
    }

    #[test]
    fn project_round_trips_on_surface_points() {
        let t = ring();
        for (u, v) in [(0.7, 1.0), (2.5, 3.0), (4.5, 5.5)] {
            let p = t.point_at(u, v);
            let ((u2, v2), p2) = t.project(p);
            assert_relative_eq!(p2, p, epsilon = 1e-9);
            assert_relative_eq!(u2, u, epsilon = 1e-9);
            assert_relative_eq!(v2, v, epsilon = 1e-9);
        }
    }
}
