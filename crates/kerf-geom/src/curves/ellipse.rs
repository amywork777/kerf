//! Axis-aligned ellipse in a frame. Parameter `t` is the eccentric anomaly:
//!   point(t) = origin + a*cos(t)*x + b*sin(t)*y.

use crate::curve::Curve;
use crate::types::{Frame, Point3, Vec3};
use serde::{Deserialize, Serialize};
use std::f64::consts::TAU;

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct Ellipse {
    pub frame: Frame,
    pub semi_major: f64, // along frame.x
    pub semi_minor: f64, // along frame.y
}

impl Ellipse {
    pub fn new(frame: Frame, semi_major: f64, semi_minor: f64) -> Self {
        debug_assert!(semi_major > 0.0 && semi_minor > 0.0);
        Ellipse {
            frame,
            semi_major,
            semi_minor,
        }
    }
}

impl Curve for Ellipse {
    fn point_at(&self, t: f64) -> Point3 {
        let (s, c) = t.sin_cos();
        self.frame.origin + self.semi_major * c * self.frame.x + self.semi_minor * s * self.frame.y
    }

    fn tangent_at(&self, t: f64) -> Vec3 {
        let (s, c) = t.sin_cos();
        let v = -self.semi_major * s * self.frame.x + self.semi_minor * c * self.frame.y;
        v.normalize()
    }

    fn domain(&self) -> (f64, f64) {
        (0.0, TAU)
    }

    fn project(&self, p: Point3) -> (f64, Point3) {
        // Newton's method on the eccentric anomaly to minimize squared distance.
        let (lx, ly, _) = self.frame.local_of(p);
        // Initial guess from atan2 in the unit-circle parameterization.
        let mut t = (ly / self.semi_minor).atan2(lx / self.semi_major);
        for _ in 0..32 {
            let (s, c) = t.sin_cos();
            let dx = self.semi_major * c - lx;
            let dy = self.semi_minor * s - ly;
            // f(t) = -a*sin(t)*dx + b*cos(t)*dy is the gradient of d²/2 wrt t.
            let f = -self.semi_major * s * dx + self.semi_minor * c * dy;
            let fp = -self.semi_major * c * dx + self.semi_major * s * (self.semi_major * s)
                - self.semi_minor * s * dy
                + self.semi_minor * c * (self.semi_minor * c);
            if fp.abs() < 1e-18 {
                break;
            }
            let dt = f / fp;
            t -= dt;
            if dt.abs() < 1e-12 {
                break;
            }
        }
        if t < 0.0 {
            t += TAU;
        }
        if t >= TAU {
            t -= TAU;
        }
        (t, self.point_at(t))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use std::f64::consts::FRAC_PI_2;

    fn ellipse_3_2_xy() -> Ellipse {
        Ellipse::new(Frame::world(Point3::origin()), 3.0, 2.0)
    }

    #[test]
    fn point_at_matches_parametric_formula() {
        let e = ellipse_3_2_xy();
        assert_relative_eq!(e.point_at(0.0), Point3::new(3.0, 0.0, 0.0));
        assert_relative_eq!(
            e.point_at(FRAC_PI_2),
            Point3::new(0.0, 2.0, 0.0),
            epsilon = 1e-12
        );
    }

    #[test]
    fn tangent_is_unit_length() {
        let e = ellipse_3_2_xy();
        for t in [0.0, 0.6, 1.7, 4.2] {
            assert_relative_eq!(e.tangent_at(t).norm(), 1.0, epsilon = 1e-12);
        }
    }

    #[test]
    fn project_round_trips_via_point_at() {
        let e = ellipse_3_2_xy();
        for t in [0.1, 1.0, 3.0, 5.5] {
            let p = e.point_at(t);
            let (_t2, p2) = e.project(p);
            assert_relative_eq!(p2, p, epsilon = 1e-9);
        }
    }
}
