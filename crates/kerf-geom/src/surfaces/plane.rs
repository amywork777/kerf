//! Infinite plane: `point(u, v) = origin + u*x + v*y`, normal is `z`.

use crate::surface::{Domain2, Surface};
use crate::types::{Frame, Point3, Vec3};

#[derive(Clone, Copy, Debug)]
pub struct Plane {
    pub frame: Frame,
}

impl Plane {
    pub fn new(frame: Frame) -> Self { Plane { frame } }
}

impl Surface for Plane {
    fn point_at(&self, u: f64, v: f64) -> Point3 {
        self.frame.origin + u * self.frame.x + v * self.frame.y
    }
    fn normal_at(&self, _u: f64, _v: f64) -> Vec3 { self.frame.z }
    fn domain(&self) -> Domain2 {
        ((f64::NEG_INFINITY, f64::INFINITY), (f64::NEG_INFINITY, f64::INFINITY))
    }
    fn project(&self, p: Point3) -> ((f64, f64), Point3) {
        let (lx, ly, _lz) = self.frame.local_of(p);
        ((lx, ly), self.point_at(lx, ly))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    fn xy_plane() -> Plane { Plane::new(Frame::world(Point3::origin())) }

    #[test]
    fn point_at_walks_the_plane() {
        let p = xy_plane();
        assert_relative_eq!(p.point_at(2.0, 3.0), Point3::new(2.0, 3.0, 0.0));
    }

    #[test]
    fn normal_is_z() {
        let p = xy_plane();
        assert_relative_eq!(p.normal_at(0.0, 0.0), Vec3::z());
    }

    #[test]
    fn project_drops_along_normal() {
        let p = xy_plane();
        let ((u, v), q) = p.project(Point3::new(2.0, 3.0, 9.9));
        assert_relative_eq!(u, 2.0);
        assert_relative_eq!(v, 3.0);
        assert_relative_eq!(q, Point3::new(2.0, 3.0, 0.0));
    }
}
