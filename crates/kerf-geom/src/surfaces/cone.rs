//! Right circular cone (infinite, double-napped at v=0 if `half_angle` is not 0).
//! `frame.origin` is the apex, `frame.z` points "into" the cone (toward the base).
//! Param u ∈ [0, TAU); v ∈ ℝ is signed distance from the apex along `frame.z`.
//! At v, the radius is v * tan(half_angle); v < 0 traces the opposite nappe.
//! point(u, v) = apex + v*z + v*tan(α)*(cos(u)*x + sin(u)*y).

use crate::surface::{Domain2, Surface};
use crate::types::{Frame, Point3, Vec3};
use std::f64::consts::TAU;

#[derive(Clone, Copy, Debug)]
pub struct Cone {
    pub frame: Frame,
    pub half_angle: f64, // radians, in (0, π/2)
}

impl Cone {
    pub fn new(frame: Frame, half_angle: f64) -> Self {
        debug_assert!(half_angle > 0.0 && half_angle < std::f64::consts::FRAC_PI_2);
        Cone { frame, half_angle }
    }
}

impl Surface for Cone {
    fn point_at(&self, u: f64, v: f64) -> Point3 {
        let (su, cu) = u.sin_cos();
        let r = v * self.half_angle.tan();
        self.frame.origin + v * self.frame.z + r * cu * self.frame.x + r * su * self.frame.y
    }

    fn normal_at(&self, u: f64, v: f64) -> Vec3 {
        // Outward normal: radial cos(α) component minus axial sin(α).
        // For the lower nappe (v < 0) the outward direction flips.
        let (su, cu) = u.sin_cos();
        let (sa, ca) = self.half_angle.sin_cos();
        let sign = if v < 0.0 { -1.0 } else { 1.0 };
        sign * (ca * (cu * self.frame.x + su * self.frame.y) - sa * self.frame.z)
    }

    fn domain(&self) -> Domain2 {
        ((0.0, TAU), (f64::NEG_INFINITY, f64::INFINITY))
    }

    fn project(&self, p: Point3) -> ((f64, f64), Point3) {
        let d = p - self.frame.origin;
        let (lx, ly, lz) = (
            d.dot(&self.frame.x),
            d.dot(&self.frame.y),
            d.dot(&self.frame.z),
        );
        let r_p = (lx * lx + ly * ly).sqrt();
        let mut u = ly.atan2(lx);
        if u < 0.0 {
            u += TAU;
        }
        // Closest point on the 2D line through origin with slope tan(α) to point (lz, r_p):
        let t = self.half_angle.tan();
        let v = (lz + r_p * t) / (1.0 + t * t);
        ((u, v), self.point_at(u, v))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use std::f64::consts::FRAC_PI_4;

    fn pi4_cone() -> Cone {
        Cone::new(Frame::world(Point3::origin()), FRAC_PI_4)
    }

    #[test]
    fn point_at_apex_is_origin() {
        let c = pi4_cone();
        assert_relative_eq!(c.point_at(0.0, 0.0), Point3::origin());
    }

    #[test]
    fn point_at_unit_height_has_unit_radius_for_pi4() {
        let c = pi4_cone();
        assert_relative_eq!(
            c.point_at(0.0, 1.0),
            Point3::new(1.0, 0.0, 1.0),
            epsilon = 1e-12
        );
    }

    fn cu_su(u: f64) -> (f64, f64) {
        let (s, c) = u.sin_cos();
        (c, s)
    }

    #[test]
    fn normal_is_unit_and_outward_for_both_nappes() {
        let c = pi4_cone();
        for u in [0.0_f64, 1.2, 3.3, 5.7] {
            for v in [0.5_f64, -0.5_f64] {
                let n = c.normal_at(u, v);
                assert_relative_eq!(n.norm(), 1.0, epsilon = 1e-12);
                // Outward: the normal's radial component must point the same direction
                // as the point's radial offset from the apex (sign of v gives the nappe side).
                let p = c.point_at(u, v);
                let from_apex = p - c.frame.origin;
                let (cu, su) = cu_su(u);
                let radial_dir = (cu * c.frame.x + su * c.frame.y) * v.signum();
                assert!(
                    n.dot(&radial_dir) > 0.0,
                    "normal not outward for u={u}, v={v}"
                );
                let _ = from_apex; // silence unused
            }
        }
    }

    #[test]
    fn project_round_trips_on_surface_points() {
        let c = pi4_cone();
        for (u, v) in [(0.5, 1.0), (2.0, 3.0), (5.0, 2.5)] {
            let p = c.point_at(u, v);
            let ((u2, v2), p2) = c.project(p);
            assert_relative_eq!(p2, p, epsilon = 1e-12);
            assert_relative_eq!(u2, u, epsilon = 1e-12);
            assert_relative_eq!(v2, v, epsilon = 1e-12);
        }
    }
}
