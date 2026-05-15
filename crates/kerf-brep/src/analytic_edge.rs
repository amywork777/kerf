// TODO: wire into boolean engine (future PR)
//
// This module defines the analytic edge data model for curved-surface boolean
// intersections. The boolean engine currently produces polyline edges.
// A follow-up PR will extend it to emit AnalyticEdge values at curved-face
// intersections.

use serde::{Deserialize, Serialize};

/// Analytic representation of an edge curve. Today the B-rep represents
/// edges as straight segments (between two vertices). For curved-surface
/// booleans to produce exact intersections, we need analytic curve types.
///
/// This module defines the data model; the boolean engine still produces
/// polyline edges in the current implementation. A future PR will extend
/// the boolean engine to emit AnalyticEdge values at curved-face
/// intersections.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind")]
pub enum AnalyticEdge {
    /// Straight line segment from `start` to `end`. Equivalent to the
    /// existing Edge representation.
    Line { start: [f64; 3], end: [f64; 3] },
    /// Circular arc. `center` + `radius` + `normal` (axis of the circle)
    /// + `start_angle` + `sweep_angle` (radians). Right-hand rule around
    /// the normal.
    Circle {
        center: [f64; 3],
        radius: f64,
        normal: [f64; 3],
        start_angle: f64,
        sweep_angle: f64,
    },
    /// Elliptical arc. `center` + `major_axis` (vector) + `minor_axis`
    /// (perpendicular to major) + angles.
    Ellipse {
        center: [f64; 3],
        major_axis: [f64; 3],
        minor_axis: [f64; 3],
        start_angle: f64,
        sweep_angle: f64,
    },
    /// Cubic B-spline (degree 3). `control_points` are 4 or more points.
    BSpline {
        control_points: Vec<[f64; 3]>,
        /// Uniform clamped knot vector implied: t = [0,0,0,0,1,2,..,n,n,n,n]
        /// where n = control_points.len() - 4
        /// (matches the convention used by STEP).
        is_clamped: bool,
    },
    /// Viviani's curve — the closed intersection of two equal-radius
    /// cylinders with perpendicular axes meeting at the origin. The
    /// canonical parameterization (in cylinder-aligned local space) is:
    ///
    /// ```text
    /// x(τ) = r · (1 + cos τ)
    /// y(τ) = r · sin τ
    /// z(τ) = 2r · sin(τ/2)
    /// ```
    ///
    /// for τ ∈ [0, 4π]. The curve traces a closed figure-eight that
    /// self-intersects at τ = 2π. `frame` is a 3×3 orthonormal matrix
    /// (columns are the local x/y/z axes in world space) and `origin`
    /// is the translation. The full normalized parameter t ∈ [0, 1]
    /// maps to τ = 4π·t.
    Viviani {
        radius: f64,
        frame: [[f64; 3]; 3],
        origin: [f64; 3],
    },
}

// ---------------------------------------------------------------------------
// Helper: build an orthonormal frame (u, v) in the plane perpendicular to n.
// u is oriented so that the arc starts at start_angle.
// ---------------------------------------------------------------------------
fn orthonormal_frame(normal: [f64; 3], start_angle: f64) -> ([f64; 3], [f64; 3]) {
    // Find an arbitrary vector not parallel to normal
    let n = normalize(normal);
    let arbitrary = if n[0].abs() < 0.9 {
        [1.0, 0.0, 0.0]
    } else {
        [0.0, 1.0, 0.0]
    };
    // u0 = arbitrary projected onto the plane perpendicular to n: arbitrary - (arbitrary·n)*n
    // then v0 = n × u0 (right-hand rule)
    let dot = arbitrary[0] * n[0] + arbitrary[1] * n[1] + arbitrary[2] * n[2];
    let u0 = normalize([
        arbitrary[0] - dot * n[0],
        arbitrary[1] - dot * n[1],
        arbitrary[2] - dot * n[2],
    ]);
    let v0 = normalize(cross(n, u0));
    // Rotate u0,v0 by start_angle
    let (sa, ca) = (start_angle.sin(), start_angle.cos());
    let u = [
        ca * u0[0] - sa * v0[0],
        ca * u0[1] - sa * v0[1],
        ca * u0[2] - sa * v0[2],
    ];
    let v = [
        sa * u0[0] + ca * v0[0],
        sa * u0[1] + ca * v0[1],
        sa * u0[2] + ca * v0[2],
    ];
    (u, v)
}

fn normalize(v: [f64; 3]) -> [f64; 3] {
    let len = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
    [v[0] / len, v[1] / len, v[2] / len]
}

fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

fn add3(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

fn scale3(v: [f64; 3], s: f64) -> [f64; 3] {
    [v[0] * s, v[1] * s, v[2] * s]
}

// ---------------------------------------------------------------------------
// De Boor's algorithm for uniform clamped cubic B-spline
// Knot vector: [0,0,0,0, 1, 2, ..., n-3, n-3, n-3, n-3]
// where n = control_points.len()
// The parameter t ∈ [0,1] is mapped to [0, n-3].
// ---------------------------------------------------------------------------
fn de_boor(control_points: &[[f64; 3]], t: f64) -> [f64; 3] {
    let n = control_points.len();
    assert!(n >= 4, "BSpline requires at least 4 control points");
    let degree = 3usize;
    let num_segments = n - degree; // number of polynomial segments
    // Build uniform clamped knot vector
    // knots: [0,0,0,0, 1,2,...,(n-4), n-3, n-3, n-3, n-3]
    let knot_max = num_segments as f64;
    let mut knots: Vec<f64> = Vec::with_capacity(n + degree + 1);
    for _ in 0..=degree {
        knots.push(0.0);
    }
    for i in 1..num_segments {
        knots.push(i as f64);
    }
    for _ in 0..=degree {
        knots.push(knot_max);
    }

    // Map t in [0,1] to [0, knot_max]
    let u = t.clamp(0.0, 1.0) * knot_max;
    // Clamp u slightly below knot_max to keep it in range for the last segment
    let u = if u >= knot_max { knot_max - f64::EPSILON * knot_max * 2.0 } else { u };

    // Find knot span index k such that knots[k] <= u < knots[k+1]
    let mut k = degree;
    for i in degree..(knots.len() - degree - 1) {
        if u >= knots[i] && u < knots[i + 1] {
            k = i;
            break;
        }
    }

    // De Boor recursion
    let mut d: Vec<[f64; 3]> = (0..=degree)
        .map(|j| control_points[k - degree + j])
        .collect();

    for r in 1..=degree {
        for j in (r..=degree).rev() {
            let i = k - degree + j;
            let denom = knots[i + degree - r + 1] - knots[i];
            let alpha = if denom.abs() < 1e-14 {
                0.0
            } else {
                (u - knots[i]) / denom
            };
            d[j] = [
                (1.0 - alpha) * d[j - 1][0] + alpha * d[j][0],
                (1.0 - alpha) * d[j - 1][1] + alpha * d[j][1],
                (1.0 - alpha) * d[j - 1][2] + alpha * d[j][2],
            ];
        }
    }

    d[degree]
}

impl AnalyticEdge {
    /// Sample the curve at parameter t ∈ [0, 1] and return the world-space point.
    pub fn point_at(&self, t: f64) -> [f64; 3] {
        match self {
            AnalyticEdge::Line { start, end } => {
                // Linear lerp
                [
                    start[0] + t * (end[0] - start[0]),
                    start[1] + t * (end[1] - start[1]),
                    start[2] + t * (end[2] - start[2]),
                ]
            }
            AnalyticEdge::Circle {
                center,
                radius,
                normal,
                start_angle,
                sweep_angle,
            } => {
                let angle = t * sweep_angle;
                let (u, v) = orthonormal_frame(*normal, *start_angle);
                let cos_a = angle.cos();
                let sin_a = angle.sin();
                add3(
                    *center,
                    add3(scale3(u, *radius * cos_a), scale3(v, *radius * sin_a)),
                )
            }
            AnalyticEdge::Ellipse {
                center,
                major_axis,
                minor_axis,
                start_angle,
                sweep_angle,
            } => {
                let angle = start_angle + t * sweep_angle;
                let cos_a = angle.cos();
                let sin_a = angle.sin();
                add3(
                    *center,
                    add3(scale3(*major_axis, cos_a), scale3(*minor_axis, sin_a)),
                )
            }
            AnalyticEdge::BSpline {
                control_points,
                is_clamped: _,
            } => de_boor(control_points, t),
            AnalyticEdge::Viviani {
                radius,
                frame,
                origin,
            } => {
                let tau = t * 4.0 * std::f64::consts::PI;
                let local = [
                    radius * (1.0 + tau.cos()),
                    radius * tau.sin(),
                    2.0 * radius * (tau / 2.0).sin(),
                ];
                // Apply frame (columns are local axes) + origin.
                [
                    origin[0]
                        + frame[0][0] * local[0]
                        + frame[0][1] * local[1]
                        + frame[0][2] * local[2],
                    origin[1]
                        + frame[1][0] * local[0]
                        + frame[1][1] * local[1]
                        + frame[1][2] * local[2],
                    origin[2]
                        + frame[2][0] * local[0]
                        + frame[2][1] * local[1]
                        + frame[2][2] * local[2],
                ]
            }
        }
    }

    /// Approximate the curve as a polyline with N segments (N+1 points).
    pub fn tessellate(&self, segments: usize) -> Vec<[f64; 3]> {
        (0..=segments)
            .map(|i| self.point_at(i as f64 / segments as f64))
            .collect()
    }

    /// Endpoints (the curve's start and end at t=0 and t=1).
    pub fn endpoints(&self) -> ([f64; 3], [f64; 3]) {
        (self.point_at(0.0), self.point_at(1.0))
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------
#[cfg(test)]
mod tests {
    use super::*;

    const EPS: f64 = 1e-10;

    fn approx_eq(a: [f64; 3], b: [f64; 3]) -> bool {
        (a[0] - b[0]).abs() < EPS && (a[1] - b[1]).abs() < EPS && (a[2] - b[2]).abs() < EPS
    }

    // -----------------------------------------------------------------------
    // Test 1: Line endpoints and midpoint
    // -----------------------------------------------------------------------
    #[test]
    fn line_endpoints_and_midpoint() {
        let edge = AnalyticEdge::Line {
            start: [0.0, 0.0, 0.0],
            end: [2.0, 4.0, 6.0],
        };
        assert!(approx_eq(edge.point_at(0.0), [0.0, 0.0, 0.0]));
        assert!(approx_eq(edge.point_at(1.0), [2.0, 4.0, 6.0]));
        assert!(approx_eq(edge.point_at(0.5), [1.0, 2.0, 3.0]));
    }

    // -----------------------------------------------------------------------
    // Test 2: Circle arc endpoints and full circle closure
    // -----------------------------------------------------------------------
    #[test]
    fn circle_arc_endpoints() {
        use std::f64::consts::{FRAC_PI_2, PI};
        // Quarter circle in XY plane, center at origin, radius 1
        // normal = Z, start_angle = 0, sweep = π/2
        let edge = AnalyticEdge::Circle {
            center: [0.0, 0.0, 0.0],
            radius: 1.0,
            normal: [0.0, 0.0, 1.0],
            start_angle: 0.0,
            sweep_angle: FRAC_PI_2,
        };
        // t=0: should be at (1, 0, 0)
        let p0 = edge.point_at(0.0);
        assert!((p0[0] - 1.0).abs() < 1e-10, "p0.x={}", p0[0]);
        assert!(p0[1].abs() < 1e-10, "p0.y={}", p0[1]);
        // t=1: should be at (0, 1, 0)
        let p1 = edge.point_at(1.0);
        assert!(p1[0].abs() < 1e-10, "p1.x={}", p1[0]);
        assert!((p1[1] - 1.0).abs() < 1e-10, "p1.y={}", p1[1]);

        // Full 2π circle: endpoints match
        let full_circle = AnalyticEdge::Circle {
            center: [0.0, 0.0, 0.0],
            radius: 3.0,
            normal: [0.0, 0.0, 1.0],
            start_angle: 0.0,
            sweep_angle: 2.0 * PI,
        };
        let s = full_circle.point_at(0.0);
        let e = full_circle.point_at(1.0);
        assert!((s[0] - e[0]).abs() < 1e-9);
        assert!((s[1] - e[1]).abs() < 1e-9);
        assert!((s[2] - e[2]).abs() < 1e-9);
        // And radius should be 3 at t=0
        let r = (s[0] * s[0] + s[1] * s[1] + s[2] * s[2]).sqrt();
        assert!((r - 3.0).abs() < 1e-10);
    }

    // -----------------------------------------------------------------------
    // Test 3: Ellipse start/end for start_angle=0, sweep=π/2,
    //         major=(1,0,0), minor=(0,1,0)
    // -----------------------------------------------------------------------
    #[test]
    fn ellipse_quarter_arc() {
        use std::f64::consts::FRAC_PI_2;
        let edge = AnalyticEdge::Ellipse {
            center: [0.0, 0.0, 0.0],
            major_axis: [1.0, 0.0, 0.0],
            minor_axis: [0.0, 1.0, 0.0],
            start_angle: 0.0,
            sweep_angle: FRAC_PI_2,
        };
        // t=0: cos(0)*major + sin(0)*minor = (1,0,0)
        assert!(approx_eq(edge.point_at(0.0), [1.0, 0.0, 0.0]));
        // t=1: cos(π/2)*major + sin(π/2)*minor = (0,1,0)
        let p1 = edge.point_at(1.0);
        assert!(p1[0].abs() < 1e-10, "p1.x={}", p1[0]);
        assert!((p1[1] - 1.0).abs() < 1e-10, "p1.y={}", p1[1]);
        assert!(p1[2].abs() < 1e-10);
    }

    // -----------------------------------------------------------------------
    // Test 4: BSpline clamped — endpoints match first and last control points
    // -----------------------------------------------------------------------
    #[test]
    fn bspline_clamped_endpoints() {
        let cp = vec![
            [0.0, 0.0, 0.0],
            [1.0, 2.0, 0.0],
            [3.0, 2.0, 0.0],
            [4.0, 0.0, 0.0],
        ];
        let edge = AnalyticEdge::BSpline {
            control_points: cp.clone(),
            is_clamped: true,
        };
        let p0 = edge.point_at(0.0);
        let p1 = edge.point_at(1.0);
        // For a clamped cubic B-spline, the curve passes through first and last CPs
        assert!((p0[0] - cp[0][0]).abs() < 1e-10, "start x={}", p0[0]);
        assert!((p0[1] - cp[0][1]).abs() < 1e-10, "start y={}", p0[1]);
        assert!((p1[0] - cp[3][0]).abs() < 1e-10, "end x={}", p1[0]);
        assert!((p1[1] - cp[3][1]).abs() < 1e-10, "end y={}", p1[1]);
    }

    // -----------------------------------------------------------------------
    // Test 5: tessellate returns N+1 points
    // -----------------------------------------------------------------------
    #[test]
    fn tessellate_returns_n_plus_1_points() {
        let edge = AnalyticEdge::Line {
            start: [0.0, 0.0, 0.0],
            end: [1.0, 1.0, 1.0],
        };
        for n in [1usize, 4, 10, 100] {
            let pts = edge.tessellate(n);
            assert_eq!(pts.len(), n + 1, "tessellate({n}) returned {} points", pts.len());
        }
    }

    // -----------------------------------------------------------------------
    // Test 6: Round-trip JSON serialization for all variants
    // -----------------------------------------------------------------------
    #[test]
    fn round_trip_json_line() {
        let edge = AnalyticEdge::Line {
            start: [1.0, 2.0, 3.0],
            end: [4.0, 5.0, 6.0],
        };
        let json = serde_json::to_string(&edge).unwrap();
        let back: AnalyticEdge = serde_json::from_str(&json).unwrap();
        assert_eq!(edge, back);
    }

    #[test]
    fn round_trip_json_circle() {
        use std::f64::consts::PI;
        let edge = AnalyticEdge::Circle {
            center: [0.0, 0.0, 0.0],
            radius: 5.0,
            normal: [0.0, 0.0, 1.0],
            start_angle: 0.0,
            sweep_angle: PI,
        };
        let json = serde_json::to_string(&edge).unwrap();
        let back: AnalyticEdge = serde_json::from_str(&json).unwrap();
        assert_eq!(edge, back);
    }

    #[test]
    fn round_trip_json_ellipse() {
        use std::f64::consts::PI;
        let edge = AnalyticEdge::Ellipse {
            center: [1.0, 2.0, 3.0],
            major_axis: [2.0, 0.0, 0.0],
            minor_axis: [0.0, 1.0, 0.0],
            start_angle: 0.0,
            sweep_angle: PI / 2.0,
        };
        let json = serde_json::to_string(&edge).unwrap();
        let back: AnalyticEdge = serde_json::from_str(&json).unwrap();
        assert_eq!(edge, back);
    }

    #[test]
    fn round_trip_json_bspline() {
        let edge = AnalyticEdge::BSpline {
            control_points: vec![
                [0.0, 0.0, 0.0],
                [1.0, 1.0, 0.0],
                [2.0, -1.0, 0.0],
                [3.0, 0.0, 0.0],
            ],
            is_clamped: true,
        };
        let json = serde_json::to_string(&edge).unwrap();
        let back: AnalyticEdge = serde_json::from_str(&json).unwrap();
        assert_eq!(edge, back);
    }

    // ----- Viviani's curve: the intersection of two equal-radius
    // perpendicular cylinders. First concrete step toward cyl × cyl
    // curved booleans (see ROADMAP.md).

    fn identity_frame() -> [[f64; 3]; 3] {
        [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    }

    #[test]
    fn viviani_at_t0_is_2r_x_origin() {
        // τ=0: (r·(1+1), 0, 0) = (2r, 0, 0) in local space.
        let edge = AnalyticEdge::Viviani {
            radius: 5.0,
            frame: identity_frame(),
            origin: [0.0, 0.0, 0.0],
        };
        let p = edge.point_at(0.0);
        assert!((p[0] - 10.0).abs() < EPS, "x = 2r at t=0");
        assert!(p[1].abs() < EPS, "y = 0 at t=0");
        assert!(p[2].abs() < EPS, "z = 0 at t=0");
    }

    #[test]
    fn viviani_at_t_half_is_self_intersection() {
        // t=0.5 → τ=2π, which lands back at the self-intersection point
        // (the figure-eight crossing) which equals the t=0 point in local
        // space: (r·(1 + cos 2π), r·sin 2π, 2r·sin π) = (2r, 0, 0).
        let edge = AnalyticEdge::Viviani {
            radius: 3.0,
            frame: identity_frame(),
            origin: [10.0, -5.0, 2.0],
        };
        let p0 = edge.point_at(0.0);
        let p_half = edge.point_at(0.5);
        for i in 0..3 {
            assert!(
                (p0[i] - p_half[i]).abs() < 1e-9,
                "self-intersection: p(0)[{}] = {} vs p(0.5)[{}] = {}",
                i, p0[i], i, p_half[i]
            );
        }
    }

    #[test]
    fn viviani_endpoints_match_at_t0_and_t1() {
        // The curve is closed: point_at(0) = point_at(1) (both correspond
        // to τ=0 and τ=4π, which yield the same point in the canonical
        // parameterization).
        let edge = AnalyticEdge::Viviani {
            radius: 7.5,
            frame: identity_frame(),
            origin: [0.0, 0.0, 0.0],
        };
        let p0 = edge.point_at(0.0);
        let p1 = edge.point_at(1.0);
        for i in 0..3 {
            assert!((p0[i] - p1[i]).abs() < 1e-9, "closed curve: p(0) = p(1)");
        }
    }

    #[test]
    fn viviani_frame_rotates_curve() {
        // A 90° rotation of the frame (swap x ↔ y) should move the
        // canonical (2r, 0, 0) starting point to (0, 2r, 0).
        let frame_swap = [
            [0.0, 1.0, 0.0], // world x = local y
            [1.0, 0.0, 0.0], // world y = local x
            [0.0, 0.0, 1.0],
        ];
        let edge = AnalyticEdge::Viviani {
            radius: 4.0,
            frame: frame_swap,
            origin: [0.0, 0.0, 0.0],
        };
        let p = edge.point_at(0.0);
        // Local (2r, 0, 0) = (8, 0, 0) → world (0, 8, 0) under the swap.
        assert!(p[0].abs() < EPS);
        assert!((p[1] - 8.0).abs() < EPS);
        assert!(p[2].abs() < EPS);
    }

    #[test]
    fn viviani_tessellate_returns_valid_3d_points() {
        let edge = AnalyticEdge::Viviani {
            radius: 2.0,
            frame: identity_frame(),
            origin: [0.0, 0.0, 0.0],
        };
        let pts = edge.tessellate(64);
        assert_eq!(pts.len(), 65, "65 = 64 segments + 1");
        // All points should lie within the bounding box [0, 2r] × [-r, r] × [-2r, 2r].
        for p in &pts {
            assert!(p[0] >= -EPS && p[0] <= 4.0 + EPS, "x in [0, 2r]: {}", p[0]);
            assert!(p[1] >= -2.0 - EPS && p[1] <= 2.0 + EPS, "y in [-r, r]: {}", p[1]);
            assert!(p[2] >= -4.0 - EPS && p[2] <= 4.0 + EPS, "z in [-2r, 2r]: {}", p[2]);
        }
    }

    #[test]
    fn round_trip_json_viviani() {
        let edge = AnalyticEdge::Viviani {
            radius: 1.5,
            frame: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            origin: [10.0, 20.0, 30.0],
        };
        let json = serde_json::to_string(&edge).unwrap();
        let back: AnalyticEdge = serde_json::from_str(&json).unwrap();
        assert_eq!(edge, back);
    }
}
