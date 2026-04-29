//! The `Curve` trait — an exact, parametric 3D curve.

use crate::types::{Point3, Vec3};

pub trait Curve {
    /// Point at parameter `t`.
    fn point_at(&self, t: f64) -> Point3;

    /// Unit tangent at parameter `t`.
    fn tangent_at(&self, t: f64) -> Vec3;

    /// Inclusive parameter domain. For unbounded curves (e.g. an infinite line)
    /// returns `(f64::NEG_INFINITY, f64::INFINITY)`.
    fn domain(&self) -> (f64, f64);

    /// Parameter and 3D point on the curve closest to `p`.
    fn project(&self, p: Point3) -> (f64, Point3);
}
