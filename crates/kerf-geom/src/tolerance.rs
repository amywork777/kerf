//! Numerical tolerance, threaded explicitly through every routine that compares values.

use crate::types::{Point3, Vec3};

#[derive(Clone, Copy, Debug)]
pub struct Tolerance {
    /// Distance below which two points are considered equal.
    pub point_eq: f64,
    /// Param-space distance below which two parameters are considered equal.
    pub param_eq: f64,
    /// Angle (radians) below which two directions are considered parallel.
    pub angle_eq: f64,
}

impl Default for Tolerance {
    fn default() -> Self {
        Tolerance {
            point_eq: 1e-9,
            param_eq: 1e-12,
            angle_eq: 1e-9,
        }
    }
}

impl Tolerance {
    pub fn points_equal(&self, a: Point3, b: Point3) -> bool {
        (a - b).norm() <= self.point_eq
    }

    pub fn params_equal(&self, a: f64, b: f64) -> bool {
        (a - b).abs() <= self.param_eq
    }

    /// True if `a` and `b` point in (approximately) the same direction.
    /// Both are assumed unit; non-unit inputs are normalized internally.
    pub fn directions_parallel(&self, a: Vec3, b: Vec3) -> bool {
        let na = a.norm();
        let nb = b.norm();
        if na == 0.0 || nb == 0.0 {
            return false;
        }
        let cos = a.dot(&b) / (na * nb);
        // 1 - cos(angle_eq) ≈ angle_eq^2 / 2 for small angles
        cos.abs() >= 1.0 - 0.5 * self.angle_eq * self.angle_eq
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_tolerance_distinguishes_distinct_points() {
        let t = Tolerance::default();
        assert!(t.points_equal(Point3::origin(), Point3::new(1e-12, 0.0, 0.0)));
        assert!(!t.points_equal(Point3::origin(), Point3::new(1e-6, 0.0, 0.0)));
    }

    #[test]
    fn directions_parallel_handles_antiparallel_and_orthogonal() {
        let t = Tolerance::default();
        assert!(t.directions_parallel(Vec3::x(), Vec3::x()));
        assert!(t.directions_parallel(Vec3::x(), -Vec3::x()));
        assert!(!t.directions_parallel(Vec3::x(), Vec3::y()));
    }

    #[test]
    fn directions_parallel_rejects_zero_vectors() {
        let t = Tolerance::default();
        assert!(!t.directions_parallel(Vec3::zeros(), Vec3::x()));
    }
}
