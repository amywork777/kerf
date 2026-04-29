//! Infinite line: `origin + t * direction`, with `direction` unit.

use crate::curve::Curve;
use crate::types::{Point3, Vec3};

#[derive(Clone, Copy, Debug)]
pub struct Line {
    pub origin: Point3,
    pub direction: Vec3, // unit
}

impl Line {
    /// Construct from two distinct points; returns `None` if they coincide.
    pub fn through(p: Point3, q: Point3) -> Option<Self> {
        let direction = (q - p).try_normalize(0.0)?;
        Some(Line { origin: p, direction })
    }

    /// Construct from origin + direction; returns `None` if direction is zero.
    pub fn from_origin_dir(origin: Point3, direction: Vec3) -> Option<Self> {
        let direction = direction.try_normalize(0.0)?;
        Some(Line { origin, direction })
    }
}

impl Curve for Line {
    fn point_at(&self, t: f64) -> Point3 { self.origin + t * self.direction }
    fn tangent_at(&self, _t: f64) -> Vec3 { self.direction }
    fn domain(&self) -> (f64, f64) { (f64::NEG_INFINITY, f64::INFINITY) }
    fn project(&self, p: Point3) -> (f64, Point3) {
        let t = (p - self.origin).dot(&self.direction);
        (t, self.point_at(t))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    fn x_axis() -> Line {
        Line::from_origin_dir(Point3::origin(), Vec3::new(2.0, 0.0, 0.0)).unwrap()
    }

    #[test]
    fn point_at_walks_along_unit_direction() {
        let l = x_axis();
        assert_relative_eq!(l.point_at(0.0), Point3::origin());
        assert_relative_eq!(l.point_at(3.0), Point3::new(3.0, 0.0, 0.0));
        assert_relative_eq!(l.point_at(-2.0), Point3::new(-2.0, 0.0, 0.0));
    }

    #[test]
    fn tangent_is_constant_unit_direction() {
        let l = x_axis();
        for t in [-1.0, 0.0, 1.0, 100.0] {
            assert_relative_eq!(l.tangent_at(t).norm(), 1.0);
            assert_relative_eq!(l.tangent_at(t), Vec3::x());
        }
    }

    #[test]
    fn project_drops_perpendicular() {
        let l = x_axis();
        let (t, p) = l.project(Point3::new(5.0, 7.0, -3.0));
        assert_relative_eq!(t, 5.0);
        assert_relative_eq!(p, Point3::new(5.0, 0.0, 0.0));
    }

    #[test]
    fn through_rejects_coincident_points() {
        assert!(Line::through(Point3::origin(), Point3::origin()).is_none());
    }

    #[test]
    fn project_round_trips_with_point_at() {
        let l = Line::from_origin_dir(
            Point3::new(1.0, 2.0, 3.0),
            Vec3::new(1.0, 1.0, 0.0),
        )
        .unwrap();
        for t in [-3.5, 0.0, 7.25] {
            let p = l.point_at(t);
            let (t2, p2) = l.project(p);
            assert_relative_eq!(t2, t, epsilon = 1e-12);
            assert_relative_eq!(p2, p, epsilon = 1e-12);
        }
    }
}
