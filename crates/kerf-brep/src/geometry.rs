//! Geometry kinds attached to topology entities.
//!
//! We use enum dispatch over the M1 concrete primitives rather than trait
//! objects — this keeps Clone semantics free and lets pattern-matching pick
//! the right closed-form intersection routine in M6.

use kerf_geom::{Circle, Cone, Curve as _, Cylinder, Ellipse, Line, Plane, Point3, Sphere, Torus};
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum CurveKind {
    Line(Line),
    Circle(Circle),
    Ellipse(Ellipse),
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum SurfaceKind {
    Plane(Plane),
    Cylinder(Cylinder),
    Sphere(Sphere),
    Cone(Cone),
    Torus(Torus),
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct CurveSegment {
    pub curve: CurveKind,
    pub range: (f64, f64),
    pub sense: Sense,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
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

    /// Construct a circular arc segment.
    /// `t_start` and `t_end` are angles in radians (the circle's parameter is
    /// the polar angle in its frame). `t_start > t_end` produces a `Reverse`
    /// segment over the same canonical range.
    pub fn circle(circle: Circle, t_start: f64, t_end: f64) -> Self {
        let (range, sense) = if t_start <= t_end {
            ((t_start, t_end), Sense::Forward)
        } else {
            ((t_end, t_start), Sense::Reverse)
        };
        CurveSegment {
            curve: CurveKind::Circle(circle),
            range,
            sense,
        }
    }

    /// Construct an elliptical arc segment.
    /// `t_start` and `t_end` are eccentric-anomaly angles in radians (matching
    /// `Ellipse::point_at`'s parameter, where t=0 is the +x semi-major endpoint).
    pub fn ellipse(ellipse: Ellipse, t_start: f64, t_end: f64) -> Self {
        let (range, sense) = if t_start <= t_end {
            ((t_start, t_end), Sense::Forward)
        } else {
            ((t_end, t_start), Sense::Reverse)
        };
        CurveSegment {
            curve: CurveKind::Ellipse(ellipse),
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

    /// Endpoint at the start of the segment, respecting `sense`.
    pub fn start_point(&self) -> Point3 {
        let t = match self.sense {
            Sense::Forward => self.range.0,
            Sense::Reverse => self.range.1,
        };
        self.point_at(t)
    }

    /// Endpoint at the end of the segment, respecting `sense`.
    pub fn end_point(&self) -> Point3 {
        let t = match self.sense {
            Sense::Forward => self.range.1,
            Sense::Reverse => self.range.0,
        };
        self.point_at(t)
    }
}

/// A closed-form arc segment of an ellipse, expressed as parameter range over
/// the ellipse's eccentric-anomaly angle.
///
/// Matches the kerf-geom `Ellipse` parameterization:
/// `point(t) = origin + a·cos(t)·x + b·sin(t)·y`.
///
/// `start_angle` and `end_angle` are in radians; the segment is traversed from
/// `start_angle` to `end_angle` in the direction implied by `Sense::Forward`
/// (numerically increasing) or `Sense::Reverse` (numerically decreasing). The
/// canonical range stored in `range` is always `(min, max)`.
///
/// A `start_angle == end_angle ± TAU` segment is a full ellipse loop. Callers
/// computing curve-segment edges from a Cylinder×Plane intersection typically
/// emit a full-loop segment (closed conic), since cylindrical face boundaries
/// in kerf-cad are presently full closed curves.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct EllipseSegment {
    pub ellipse: Ellipse,
    pub range: (f64, f64),
    pub sense: Sense,
}

impl EllipseSegment {
    /// Construct a segment with explicit start/end angles. If `start > end` the
    /// segment is `Reverse`; otherwise `Forward`. Both endpoints are stored.
    pub fn new(ellipse: Ellipse, start_angle: f64, end_angle: f64) -> Self {
        let (range, sense) = if start_angle <= end_angle {
            ((start_angle, end_angle), Sense::Forward)
        } else {
            ((end_angle, start_angle), Sense::Reverse)
        };
        EllipseSegment {
            ellipse,
            range,
            sense,
        }
    }

    /// The full ellipse traversed once forward (`0` → `TAU`).
    pub fn full(ellipse: Ellipse) -> Self {
        EllipseSegment {
            ellipse,
            range: (0.0, std::f64::consts::TAU),
            sense: Sense::Forward,
        }
    }

    /// Directed arc-span (end − start) accounting for sense.
    ///
    /// Forward: `range.1 - range.0`; Reverse: `range.0 - range.1`. A full
    /// forward ellipse returns `TAU`; a full reverse ellipse returns `-TAU`.
    pub fn directed_span(&self) -> f64 {
        match self.sense {
            Sense::Forward => self.range.1 - self.range.0,
            Sense::Reverse => self.range.0 - self.range.1,
        }
    }

    pub fn point_at(&self, t: f64) -> Point3 {
        self.ellipse.point_at(t)
    }

    /// World-space point at the start of the directed segment.
    pub fn start_point(&self) -> Point3 {
        let t = match self.sense {
            Sense::Forward => self.range.0,
            Sense::Reverse => self.range.1,
        };
        self.ellipse.point_at(t)
    }

    /// World-space point at the end of the directed segment.
    pub fn end_point(&self) -> Point3 {
        let t = match self.sense {
            Sense::Forward => self.range.1,
            Sense::Reverse => self.range.0,
        };
        self.ellipse.point_at(t)
    }

    /// True iff the segment spans a full revolution (≈ `TAU` arc).
    pub fn is_full(&self) -> bool {
        let span = self.range.1 - self.range.0;
        (span - std::f64::consts::TAU).abs() < 1e-12
    }

    /// Convert to a generic `CurveSegment` (lossless: range and sense preserved).
    pub fn into_curve_segment(self) -> CurveSegment {
        CurveSegment {
            curve: CurveKind::Ellipse(self.ellipse),
            range: self.range,
            sense: self.sense,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use kerf_geom::{Frame, Vec3};
    use std::f64::consts::{FRAC_PI_2, PI, TAU};

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

    #[test]
    fn curve_segment_circle_forward_endpoints_match_quarter_arc() {
        let c = Circle::new(Frame::world(Point3::origin()), 1.0);
        let seg = CurveSegment::circle(c, 0.0, FRAC_PI_2);
        assert_eq!(seg.sense, Sense::Forward);
        assert_relative_eq!(
            seg.start_point(),
            Point3::new(1.0, 0.0, 0.0),
            epsilon = 1e-12
        );
        assert_relative_eq!(seg.end_point(), Point3::new(0.0, 1.0, 0.0), epsilon = 1e-12);
    }

    #[test]
    fn curve_segment_ellipse_forward_endpoints_match_quarter_arc() {
        let e = Ellipse::new(Frame::world(Point3::origin()), 3.0, 2.0);
        let seg = CurveSegment::ellipse(e, 0.0, FRAC_PI_2);
        assert_eq!(seg.sense, Sense::Forward);
        assert_relative_eq!(
            seg.start_point(),
            Point3::new(3.0, 0.0, 0.0),
            epsilon = 1e-12
        );
        assert_relative_eq!(seg.end_point(), Point3::new(0.0, 2.0, 0.0), epsilon = 1e-12);
    }

    #[test]
    fn curve_segment_ellipse_reverse_swaps_endpoints() {
        let e = Ellipse::new(Frame::world(Point3::origin()), 3.0, 2.0);
        let seg = CurveSegment::ellipse(e, FRAC_PI_2, 0.0);
        assert_eq!(seg.sense, Sense::Reverse);
        // Canonical range is still (0, π/2), but start_point is at the larger
        // angle because the segment is reversed.
        assert_relative_eq!(
            seg.start_point(),
            Point3::new(0.0, 2.0, 0.0),
            epsilon = 1e-12
        );
        assert_relative_eq!(seg.end_point(), Point3::new(3.0, 0.0, 0.0), epsilon = 1e-12);
    }

    #[test]
    fn ellipse_segment_round_trip() {
        // Round-trip: build a segment with explicit start/end angles, recover
        // the same world-space endpoints via start_point/end_point, then
        // round-trip through into_curve_segment and re-evaluate.
        let e = Ellipse::new(Frame::world(Point3::new(1.0, 1.0, 0.0)), 4.0, 2.0);
        let seg = EllipseSegment::new(e, 0.0, PI);
        assert_eq!(seg.sense, Sense::Forward);
        assert_relative_eq!(seg.range.0, 0.0, epsilon = 1e-12);
        assert_relative_eq!(seg.range.1, PI, epsilon = 1e-12);
        // t=0  → origin + a·x = (5,1,0)
        // t=π → origin − a·x = (−3,1,0)
        assert_relative_eq!(
            seg.start_point(),
            Point3::new(5.0, 1.0, 0.0),
            epsilon = 1e-12
        );
        assert_relative_eq!(
            seg.end_point(),
            Point3::new(-3.0, 1.0, 0.0),
            epsilon = 1e-12
        );
        assert_relative_eq!(seg.directed_span(), PI, epsilon = 1e-12);

        // Reverse round-trip.
        let rev = EllipseSegment::new(e, PI, 0.0);
        assert_eq!(rev.sense, Sense::Reverse);
        assert_relative_eq!(
            rev.start_point(),
            Point3::new(-3.0, 1.0, 0.0),
            epsilon = 1e-12
        );
        assert_relative_eq!(rev.end_point(), Point3::new(5.0, 1.0, 0.0), epsilon = 1e-12);
        assert_relative_eq!(rev.directed_span(), -PI, epsilon = 1e-12);

        // CurveSegment lossless round-trip.
        let cs = seg.clone().into_curve_segment();
        assert_eq!(cs.sense, Sense::Forward);
        assert_eq!(cs.range, (0.0, PI));
        match cs.curve {
            CurveKind::Ellipse(e2) => {
                assert_relative_eq!(e2.semi_major, 4.0);
                assert_relative_eq!(e2.semi_minor, 2.0);
            }
            other => panic!("expected Ellipse, got {other:?}"),
        }
    }

    #[test]
    fn ellipse_segment_full_loop_spans_tau() {
        let e = Ellipse::new(Frame::world(Point3::origin()), 2.0, 1.0);
        let seg = EllipseSegment::full(e);
        assert!(seg.is_full());
        assert_eq!(seg.sense, Sense::Forward);
        assert_relative_eq!(seg.directed_span(), TAU, epsilon = 1e-12);
        // A full loop's start_point and end_point are both at t=0 (and t=TAU,
        // which is the same world point modulo numerical noise).
        assert_relative_eq!(
            seg.start_point(),
            Point3::new(2.0, 0.0, 0.0),
            epsilon = 1e-12
        );
        assert_relative_eq!(seg.end_point(), seg.start_point(), epsilon = 1e-12);
    }
}
