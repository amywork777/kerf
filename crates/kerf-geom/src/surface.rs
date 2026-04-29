//! The `Surface` trait — an exact, parametric 3D surface.

use crate::types::{Point3, Vec3};

/// A parameter rectangle `(u_min, u_max) × (v_min, v_max)`.
pub type Domain2 = ((f64, f64), (f64, f64));

pub trait Surface {
    fn point_at(&self, u: f64, v: f64) -> Point3;
    fn normal_at(&self, u: f64, v: f64) -> Vec3;
    fn domain(&self) -> Domain2;
    fn project(&self, p: Point3) -> ((f64, f64), Point3);
}
