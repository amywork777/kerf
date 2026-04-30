//! Geometry kinds attached to topology entities.
//!
//! We use enum dispatch over the M1 concrete primitives rather than trait
//! objects — this keeps Clone semantics free and lets pattern-matching pick
//! the right closed-form intersection routine in M6.

use kerf_geom::{Circle, Cone, Curve as _, Cylinder, Ellipse, Line, Plane, Point3, Sphere, Torus};

#[derive(Clone, Debug)]
pub enum CurveKind {
    Line(Line),
    Circle(Circle),
    Ellipse(Ellipse),
}

#[derive(Clone, Debug)]
pub enum SurfaceKind {
    Plane(Plane),
    Cylinder(Cylinder),
    Sphere(Sphere),
    Cone(Cone),
    Torus(Torus),
}

#[derive(Clone, Debug)]
pub struct CurveSegment {
    pub curve: CurveKind,
    pub range: (f64, f64),
    pub sense: Sense,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Sense {
    Forward,
    Reverse,
}

impl CurveSegment {
    pub fn line(line: Line, t_start: f64, t_end: f64) -> Self {
        let (range, sense) = if t_start <= t_end {
            ((t_start, t_end), Sense::Forward)
        } else {
            ((t_end, t_start), Sense::Reverse)
        };
        CurveSegment {
            curve: CurveKind::Line(line),
            range,
            sense,
        }
    }

    pub fn point_at(&self, t: f64) -> Point3 {
        match &self.curve {
            CurveKind::Line(l) => l.point_at(t),
            CurveKind::Circle(c) => c.point_at(t),
            CurveKind::Ellipse(e) => e.point_at(t),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use kerf_geom::Vec3;

    #[test]
    fn line_segment_forward_endpoint() {
        let l = Line::from_origin_dir(Point3::origin(), Vec3::x()).unwrap();
        let seg = CurveSegment::line(l, 0.0, 5.0);
        assert_eq!(seg.sense, Sense::Forward);
        assert_relative_eq!(seg.point_at(5.0), Point3::new(5.0, 0.0, 0.0));
    }

    #[test]
    fn line_segment_reverse_swaps_range_and_sets_sense() {
        let l = Line::from_origin_dir(Point3::origin(), Vec3::x()).unwrap();
        let seg = CurveSegment::line(l, 5.0, 0.0);
        assert_eq!(seg.sense, Sense::Reverse);
        assert_eq!(seg.range, (0.0, 5.0));
    }
}
