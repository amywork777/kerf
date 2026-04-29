//! Sphere with axis frame (z is the polar axis).
//! Param u ∈ [0, TAU) is azimuth; v ∈ [0, π] is polar angle from +z.
//! point(u, v) = center + r * (sin(v)*cos(u)*x + sin(v)*sin(u)*y + cos(v)*z).

use crate::surface::{Domain2, Surface};
use crate::types::{Frame, Point3, Vec3};
use std::f64::consts::{PI, TAU};

#[derive(Clone, Copy, Debug)]
pub struct Sphere {
    pub frame: Frame,
    pub radius: f64,
}

impl Sphere {
    pub fn new(frame: Frame, radius: f64) -> Self {
        debug_assert!(radius > 0.0);
        Sphere { frame, radius }
    }

    pub fn at_origin(radius: f64) -> Self {
        Sphere::new(Frame::world(Point3::origin()), radius)
    }
}

impl Surface for Sphere {
    fn point_at(&self, u: f64, v: f64) -> Point3 {
        let (su, cu) = u.sin_cos();
        let (sv, cv) = v.sin_cos();
        self.frame.origin
            + self.radius * (sv * cu * self.frame.x + sv * su * self.frame.y + cv * self.frame.z)
    }

    fn normal_at(&self, u: f64, v: f64) -> Vec3 {
        let (su, cu) = u.sin_cos();
        let (sv, cv) = v.sin_cos();
        sv * cu * self.frame.x + sv * su * self.frame.y + cv * self.frame.z
    }

    fn domain(&self) -> Domain2 { ((0.0, TAU), (0.0, PI)) }

    fn project(&self, p: Point3) -> ((f64, f64), Point3) {
        let d = p - self.frame.origin;
        if d.norm() == 0.0 {
            return ((0.0, 0.0), self.point_at(0.0, 0.0));
        }
        let (lx, ly, lz) = (d.dot(&self.frame.x), d.dot(&self.frame.y), d.dot(&self.frame.z));
        let mut u = ly.atan2(lx);
        if u < 0.0 { u += TAU; }
        let v = (lz / d.norm()).clamp(-1.0, 1.0).acos();
        ((u, v), self.point_at(u, v))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use std::f64::consts::FRAC_PI_2;

    #[test]
    fn point_at_pole_and_equator() {
        let s = Sphere::at_origin(2.0);
        assert_relative_eq!(s.point_at(0.0, 0.0), Point3::new(0.0, 0.0, 2.0));
        assert_relative_eq!(s.point_at(0.0, FRAC_PI_2), Point3::new(2.0, 0.0, 0.0), epsilon = 1e-12);
    }

    #[test]
    fn normal_is_unit_and_radial() {
        let s = Sphere::at_origin(5.0);
        for (u, v) in [(0.0, 0.5), (1.5, 1.2), (4.0, 2.5)] {
            let n = s.normal_at(u, v);
            assert_relative_eq!(n.norm(), 1.0, epsilon = 1e-12);
            let p = s.point_at(u, v);
            let radial = (p - s.frame.origin).normalize();
            assert_relative_eq!(n, radial, epsilon = 1e-12);
        }
    }

    #[test]
    fn project_round_trips() {
        let s = Sphere::at_origin(3.0);
        for (u, v) in [(0.7, 0.6), (3.0, 1.5), (5.5, 2.4)] {
            let p = s.point_at(u, v);
            let ((u2, v2), p2) = s.project(p);
            assert_relative_eq!(p2, p, epsilon = 1e-12);
            assert_relative_eq!(u2, u, epsilon = 1e-9);
            assert_relative_eq!(v2, v, epsilon = 1e-9);
        }
    }

    #[test]
    fn project_handles_external_point() {
        let s = Sphere::at_origin(1.0);
        let (_uv, q) = s.project(Point3::new(10.0, 0.0, 0.0));
        assert_relative_eq!(q, Point3::new(1.0, 0.0, 0.0), epsilon = 1e-12);
    }
}
