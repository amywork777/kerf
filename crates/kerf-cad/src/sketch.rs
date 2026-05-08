//! 2D sketcher data model + DSL.
//!
//! A `Sketch` is a parametric description of a planar curve graph: a bag of
//! named primitives (Points, Lines, Circles, Arcs) plus a separate bag of
//! `SketchConstraint`s that describe geometric relations between those
//! primitives. Constraints are STORED but not yet enforced — the constraint
//! solver is a separate gap (see STATUS.md). Coordinate values come straight
//! from the `Scalar` fields.
//!
//! `Sketch::to_profile_2d` traces the line+arc graph into one or more closed
//! loops and emits a [`Profile2D`] per loop. Standalone Circles also emit
//! their own profile (one polygonal sample of the circle). The tracer is
//! deliberately simple: it requires every Point that participates in a
//! line/arc to have valence exactly two, so that loops are unambiguous.
//! Branching, dangling endpoints, and disjoint sub-loops on the same set of
//! points are all rejected with a clear `SketchError`.
//!
//! The new feature variants `Feature::SketchExtrude` and
//! `Feature::SketchRevolve` route the sketch through `to_profile_2d` and then
//! into the existing `extrude_polygon` / `revolve_polyline` kernel paths.

use std::collections::{BTreeMap, HashMap, HashSet};

use serde::{Deserialize, Serialize};
use thiserror::Error;

use crate::feature::Profile2D;
use crate::scalar::Scalar;

/// The plane the sketch lives on. The named-ref-plane variant is a
/// forward-compatible slot — today the evaluator treats every sketch as
/// living on its named plane *as if* it were the corresponding axis-aligned
/// plane (i.e. there is no positioning resolved from the named ref). When
/// `Feature::RefPlane` ids become first-class transforms, this can grow.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub enum SketchPlane {
    /// XY plane at z = 0 — the default for `SketchExtrude`.
    Xy,
    /// XZ plane at y = 0 — used for `SketchRevolve` (axisymmetric profiles
    /// live in this plane around the z-axis).
    Xz,
    /// YZ plane at x = 0.
    Yz,
    /// A named reference plane. Currently behaves like `Xy` for sketches —
    /// see module docs.
    NamedRefPlane(String),
}

/// One sketch primitive. Each carries a unique `id`, scoped to the sketch.
/// Lines and Arcs reference Point ids by name (so moving a Point updates
/// every primitive that touches it).
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(tag = "kind")]
pub enum SketchPrim {
    Point {
        id: String,
        x: Scalar,
        y: Scalar,
    },
    /// Straight line between two existing Points.
    Line {
        id: String,
        from: String,
        to: String,
    },
    /// Closed circle. Stands alone — it does not participate in
    /// line+arc loop tracing; its profile is emitted directly.
    Circle {
        id: String,
        center: String,
        radius: Scalar,
        /// Polygonal sample count (>= 3).
        n_segments: usize,
    },
    /// Open arc: portion of a circle from `start_angle` to `end_angle`
    /// (radians, CCW around `center`). Endpoints are sampled and used in
    /// loop tracing; intermediate samples become the discretized polyline
    /// inside the profile.
    Arc {
        id: String,
        center: String,
        radius: Scalar,
        start_angle: Scalar,
        end_angle: Scalar,
        /// Number of straight-line segments used to approximate the arc
        /// (>= 1 — the polyline has `n_segments + 1` sample points).
        n_segments: usize,
    },
}

impl SketchPrim {
    pub fn id(&self) -> &str {
        match self {
            SketchPrim::Point { id, .. }
            | SketchPrim::Line { id, .. }
            | SketchPrim::Circle { id, .. }
            | SketchPrim::Arc { id, .. } => id,
        }
    }
}

/// Geometric constraints. Round-tripped through JSON. Enforced by
/// [`Sketch::solve`] in `crate::solver`, which adjusts Point coordinates
/// to drive the residual sum to zero.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(tag = "kind")]
pub enum SketchConstraint {
    /// Two points should be coincident.
    Coincident { a: String, b: String },
    /// Two points should be `value` apart.
    Distance { a: String, b: String, value: Scalar },
    /// A line should be horizontal.
    Horizontal { line: String },
    /// A line should be vertical.
    Vertical { line: String },
    /// Two lines should be parallel.
    Parallel { line_a: String, line_b: String },
    /// Two lines should be perpendicular.
    Perpendicular { line_a: String, line_b: String },
    /// A point should be pinned in place.
    FixedPoint { point: String },
    /// A line should be tangent to a circle (the line's distance from
    /// the circle's center equals the circle's radius).
    TangentLineToCircle { line: String, circle: String },
    /// A point should lie on a line (perpendicular distance = 0).
    CoincidentOnLine { point: String, line: String },
    /// Two lines should have equal length.
    EqualLength { line_a: String, line_b: String },
    /// Two circles should have equal radius. (Resolves both radii at
    /// solve time; if the circles' radii are literal `Scalar::Lit`s,
    /// only the constraint check matters — the solver does not perturb
    /// radii, so this is satisfied iff the literal values agree.
    /// More usefully it pairs with `Scalar::Param` radii sharing the
    /// same parameter, where the radii are equal by construction.)
    EqualRadius { circle_a: String, circle_b: String },
}

/// A parametric 2D sketch: primitives + constraints, on a plane.
///
/// See module docs for what `to_profile_2d` does and what's deferred.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct Sketch {
    pub plane: SketchPlane,
    pub primitives: Vec<SketchPrim>,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub constraints: Vec<SketchConstraint>,
}

#[derive(Debug, Error, PartialEq)]
pub enum SketchError {
    #[error("duplicate primitive id '{0}'")]
    DuplicateId(String),
    #[error("primitive '{prim}' references unknown point '{point}'")]
    UnknownPoint { prim: String, point: String },
    #[error("primitive '{0}' references a non-Point primitive (Lines/Arcs need Point endpoints)")]
    NotAPoint(String),
    #[error("Circle '{id}' needs n_segments >= 3 (got {got})")]
    CircleSegments { id: String, got: usize },
    #[error("Arc '{id}' needs n_segments >= 1 (got {got})")]
    ArcSegments { id: String, got: usize },
    #[error("sketch is empty — no primitives that can form a profile")]
    Empty,
    #[error("sketch has no closed loop (open polyline / dangling endpoint at point '{0}')")]
    OpenLoop(String),
    #[error("sketch is ambiguous: point '{point}' has valence {valence} (must be 2 for a simple loop)")]
    AmbiguousValence { point: String, valence: usize },
    #[error("sketch produced multiple disjoint loops sharing no points (use one Sketch per loop)")]
    DisjointLoops,
    #[error("expected a single closed loop, got {0}")]
    NotSingleLoop(usize),
    #[error("expression error: {0}")]
    Expr(String),
}

/// One edge in the sketch loop graph: connects two Point ids, with a
/// resolved sequence of polyline samples between them (excluding the from
/// endpoint, including the to endpoint, in the order traversed from `from`
/// to `to`). Reversing direction reverses the sample sequence.
#[derive(Debug, Clone)]
struct LoopEdge {
    from: String,
    to: String,
    /// Sample points strictly between `from` and `to` (for arcs only —
    /// empty for straight lines).
    interior_samples: Vec<[f64; 2]>,
}

impl Sketch {
    /// Trace closed loops in the sketch and emit a `Profile2D` for each.
    ///
    /// Standalone `Circle` primitives emit their own profile. Lines and
    /// Arcs are walked together as edges of a graph; each connected
    /// component must form a single simple loop (every Point touched by
    /// some line/arc has valence exactly 2). Floating Points that aren't
    /// referenced by any Line or Arc are tolerated — they are pure
    /// construction geometry. Floating Points that ARE referenced by a
    /// line/arc but with valence != 2 produce an error.
    ///
    /// Coordinates resolve through the model's `params` table at trace
    /// time, so `$param` references in any Scalar field work.
    pub fn to_profile_2d(
        &self,
        params: &HashMap<String, f64>,
    ) -> Result<Vec<Profile2D>, SketchError> {
        let resolved = self.resolve(params)?;
        resolved.trace_loops()
    }
}

/// Sketch with all Scalar fields resolved to f64. Internal-only.
struct ResolvedSketch {
    points: BTreeMap<String, [f64; 2]>,
    lines: Vec<ResolvedLine>,
    arcs: Vec<ResolvedArc>,
    circles: Vec<ResolvedCircle>,
}

#[derive(Debug, Clone)]
struct ResolvedLine {
    id: String,
    from: String,
    to: String,
}

#[derive(Debug, Clone)]
struct ResolvedArc {
    #[allow(dead_code)]
    id: String,
    /// Sample sequence including both endpoints, in CCW (start_angle ->
    /// end_angle) order.
    samples: Vec<[f64; 2]>,
    /// Snapshot of from-endpoint key (matches a Point id by approximate
    /// position). For graph adjacency we anchor arcs to the named center
    /// point's pair — but the sketcher today just samples the arc and
    /// assumes the start/end coincide with named Points referenced by the
    /// adjacent Lines. To express that cleanly we store the resolved
    /// endpoints; the loop tracer matches them by approximate position to
    /// the Point id table.
    from_key: String,
    to_key: String,
}

#[derive(Debug, Clone)]
struct ResolvedCircle {
    /// The full closed sample loop, CCW.
    samples: Vec<[f64; 2]>,
}

impl Sketch {
    fn resolve(&self, params: &HashMap<String, f64>) -> Result<ResolvedSketch, SketchError> {
        // First pass: collect Points with duplicate-id check.
        let mut points: BTreeMap<String, [f64; 2]> = BTreeMap::new();
        let mut seen_ids: HashSet<String> = HashSet::new();
        for prim in &self.primitives {
            let pid = prim.id().to_string();
            if !seen_ids.insert(pid.clone()) {
                return Err(SketchError::DuplicateId(pid));
            }
            if let SketchPrim::Point { id, x, y } = prim {
                let xv = x.resolve(params).map_err(SketchError::Expr)?;
                let yv = y.resolve(params).map_err(SketchError::Expr)?;
                points.insert(id.clone(), [xv, yv]);
            }
        }

        // Second pass: lines, arcs, circles.
        let mut lines = Vec::new();
        let mut arcs = Vec::new();
        let mut circles = Vec::new();
        for prim in &self.primitives {
            match prim {
                SketchPrim::Point { .. } => {}
                SketchPrim::Line { id, from, to } => {
                    if !points.contains_key(from) {
                        return Err(SketchError::UnknownPoint {
                            prim: id.clone(),
                            point: from.clone(),
                        });
                    }
                    if !points.contains_key(to) {
                        return Err(SketchError::UnknownPoint {
                            prim: id.clone(),
                            point: to.clone(),
                        });
                    }
                    lines.push(ResolvedLine {
                        id: id.clone(),
                        from: from.clone(),
                        to: to.clone(),
                    });
                }
                SketchPrim::Circle {
                    id,
                    center,
                    radius,
                    n_segments,
                } => {
                    let &c = points.get(center).ok_or_else(|| SketchError::UnknownPoint {
                        prim: id.clone(),
                        point: center.clone(),
                    })?;
                    if *n_segments < 3 {
                        return Err(SketchError::CircleSegments {
                            id: id.clone(),
                            got: *n_segments,
                        });
                    }
                    let r = radius.resolve(params).map_err(SketchError::Expr)?;
                    let n = *n_segments;
                    let mut samples = Vec::with_capacity(n);
                    for i in 0..n {
                        let theta = 2.0 * std::f64::consts::PI * (i as f64) / (n as f64);
                        samples.push([c[0] + r * theta.cos(), c[1] + r * theta.sin()]);
                    }
                    circles.push(ResolvedCircle { samples });
                }
                SketchPrim::Arc {
                    id,
                    center,
                    radius,
                    start_angle,
                    end_angle,
                    n_segments,
                } => {
                    let &c = points.get(center).ok_or_else(|| SketchError::UnknownPoint {
                        prim: id.clone(),
                        point: center.clone(),
                    })?;
                    if *n_segments < 1 {
                        return Err(SketchError::ArcSegments {
                            id: id.clone(),
                            got: *n_segments,
                        });
                    }
                    let r = radius.resolve(params).map_err(SketchError::Expr)?;
                    let a0 = start_angle.resolve(params).map_err(SketchError::Expr)?;
                    let a1 = end_angle.resolve(params).map_err(SketchError::Expr)?;
                    let n = *n_segments;
                    let mut samples = Vec::with_capacity(n + 1);
                    for i in 0..=n {
                        let t = (i as f64) / (n as f64);
                        let theta = a0 + (a1 - a0) * t;
                        samples.push([c[0] + r * theta.cos(), c[1] + r * theta.sin()]);
                    }
                    // Find Point ids that match the arc endpoints (within
                    // a small tolerance). Required so the arc participates
                    // in loop tracing alongside Lines.
                    let from_key = match_point_id(&points, samples[0])
                        .ok_or_else(|| SketchError::OpenLoop(format!("{id}/start")))?;
                    let to_key = match_point_id(&points, samples[n])
                        .ok_or_else(|| SketchError::OpenLoop(format!("{id}/end")))?;
                    arcs.push(ResolvedArc {
                        id: id.clone(),
                        samples,
                        from_key,
                        to_key,
                    });
                }
            }
        }

        Ok(ResolvedSketch {
            points,
            lines,
            arcs,
            circles,
        })
    }
}

/// Locate a Point id whose resolved position is within `EPS` of `xy`.
/// Returns None if no Point matches — the caller should treat that as an
/// open arc endpoint (for now we require arc endpoints to coincide with
/// named Points).
fn match_point_id(points: &BTreeMap<String, [f64; 2]>, xy: [f64; 2]) -> Option<String> {
    const EPS: f64 = 1e-6;
    for (id, p) in points {
        let dx = p[0] - xy[0];
        let dy = p[1] - xy[1];
        if (dx * dx + dy * dy).sqrt() < EPS {
            return Some(id.clone());
        }
    }
    None
}

impl ResolvedSketch {
    fn trace_loops(&self) -> Result<Vec<Profile2D>, SketchError> {
        let mut profiles: Vec<Profile2D> = Vec::new();

        // Standalone circles each contribute one profile.
        for c in &self.circles {
            profiles.push(samples_to_profile(&c.samples));
        }

        // If there are no Lines or Arcs, return whatever we have from
        // circles. If there is also nothing there, fail.
        if self.lines.is_empty() && self.arcs.is_empty() {
            if profiles.is_empty() {
                return Err(SketchError::Empty);
            }
            return Ok(profiles);
        }

        // Build the loop-edge graph.
        let mut edges: Vec<LoopEdge> = Vec::new();
        for ln in &self.lines {
            edges.push(LoopEdge {
                from: ln.from.clone(),
                to: ln.to.clone(),
                interior_samples: Vec::new(),
            });
            let _ = ln.id.clone(); // id retained for future error messages
        }
        for arc in &self.arcs {
            // Interior = samples without endpoints.
            let n = arc.samples.len();
            let interior = if n >= 2 {
                arc.samples[1..n - 1].to_vec()
            } else {
                Vec::new()
            };
            edges.push(LoopEdge {
                from: arc.from_key.clone(),
                to: arc.to_key.clone(),
                interior_samples: interior,
            });
        }

        // Adjacency: point id -> [(edge_idx, other_point_id, dir)].
        // dir = false means the edge is stored with `from` matching this
        // point; true means we walk the edge backwards (from the `to` side).
        let mut adj: HashMap<String, Vec<(usize, String, bool)>> = HashMap::new();
        for (i, e) in edges.iter().enumerate() {
            adj.entry(e.from.clone()).or_default().push((i, e.to.clone(), false));
            adj.entry(e.to.clone()).or_default().push((i, e.from.clone(), true));
        }

        // Validate valences: every endpoint that appears in any edge must
        // have valence exactly 2.
        for (pid, neighbors) in &adj {
            if neighbors.len() != 2 {
                return Err(SketchError::AmbiguousValence {
                    point: pid.clone(),
                    valence: neighbors.len(),
                });
            }
        }

        // Walk connected components, each must be a single closed loop.
        let mut visited_edges = vec![false; edges.len()];

        while let Some(start_edge_idx) = visited_edges.iter().position(|v| !v) {
            // Start traversal from edges[start_edge_idx].from, walking
            // through .to first.
            let start_pt = edges[start_edge_idx].from.clone();
            let mut current_pt = start_pt.clone();
            let mut next_edge = start_edge_idx;
            let mut samples: Vec<[f64; 2]> = Vec::new();

            loop {
                if visited_edges[next_edge] {
                    return Err(SketchError::OpenLoop(current_pt));
                }
                visited_edges[next_edge] = true;

                let edge = &edges[next_edge];
                // Determine traversal direction.
                let (to_pt, fwd) = if edge.from == current_pt {
                    (edge.to.clone(), true)
                } else if edge.to == current_pt {
                    (edge.from.clone(), false)
                } else {
                    // Shouldn't happen given valence-2 invariant.
                    return Err(SketchError::OpenLoop(current_pt));
                };

                // Append samples for this edge: interior in correct
                // direction, plus the to-endpoint.
                if fwd {
                    samples.extend_from_slice(&edge.interior_samples);
                } else {
                    for s in edge.interior_samples.iter().rev() {
                        samples.push(*s);
                    }
                }
                let p_to = self.points.get(&to_pt).ok_or_else(|| {
                    SketchError::UnknownPoint {
                        prim: "loop edge".into(),
                        point: to_pt.clone(),
                    }
                })?;
                samples.push(*p_to);

                current_pt = to_pt;

                if current_pt == start_pt {
                    break;
                }

                // Find the next edge incident to current_pt that isn't
                // already visited.
                let neighbors = adj.get(&current_pt).expect("valence-2 guarantee");
                let mut found = None;
                for (e_idx, _other, _dir) in neighbors {
                    if !visited_edges[*e_idx] {
                        found = Some(*e_idx);
                        break;
                    }
                }
                match found {
                    Some(e) => next_edge = e,
                    None => return Err(SketchError::OpenLoop(current_pt)),
                }
            }

            // Loop closed. Drop the last sample (== first start point) so
            // the profile is a non-repeating CCW polygon.
            if let Some(last) = samples.last() {
                let dxdy = (last[0] - self.points[&start_pt][0],
                            last[1] - self.points[&start_pt][1]);
                if dxdy.0.hypot(dxdy.1) < 1e-9 {
                    samples.pop();
                }
            }
            // Push the start point at index 0 (the loop above starts
            // appending from the first edge's interior + to endpoint, so
            // we never wrote start_pt explicitly).
            let start_xy = self.points[&start_pt];
            let mut full = Vec::with_capacity(samples.len() + 1);
            full.push(start_xy);
            full.extend(samples);

            // Ensure CCW orientation (positive signed area). If clockwise,
            // reverse — `extrude_polygon` and `revolve_polyline` both want
            // CCW from the +direction.
            if signed_area(&full) < 0.0 {
                full.reverse();
            }
            profiles.push(samples_to_profile(&full));
        }

        // If there are multiple components from line/arc edges that don't
        // share any point, we still flag DisjointLoops only if the user
        // expected a single profile. We return all of them — callers that
        // need exactly one (SketchExtrude / SketchRevolve) check len() ==
        // 1 themselves. (Standalone circles already added above.)

        Ok(profiles)
    }
}

fn samples_to_profile(samples: &[[f64; 2]]) -> Profile2D {
    Profile2D {
        points: samples
            .iter()
            .map(|s| [Scalar::lit(s[0]), Scalar::lit(s[1])])
            .collect(),
    }
}

fn signed_area(pts: &[[f64; 2]]) -> f64 {
    let n = pts.len();
    if n < 3 {
        return 0.0;
    }
    let mut s = 0.0;
    for i in 0..n {
        let p = pts[i];
        let q = pts[(i + 1) % n];
        s += p[0] * q[1] - q[0] * p[1];
    }
    s * 0.5
}

#[cfg(test)]
mod tests {
    use super::*;

    fn rect_sketch() -> Sketch {
        Sketch {
            plane: SketchPlane::Xy,
            primitives: vec![
                SketchPrim::Point { id: "p1".into(), x: Scalar::lit(0.0), y: Scalar::lit(0.0) },
                SketchPrim::Point { id: "p2".into(), x: Scalar::lit(2.0), y: Scalar::lit(0.0) },
                SketchPrim::Point { id: "p3".into(), x: Scalar::lit(2.0), y: Scalar::lit(3.0) },
                SketchPrim::Point { id: "p4".into(), x: Scalar::lit(0.0), y: Scalar::lit(3.0) },
                SketchPrim::Line { id: "l1".into(), from: "p1".into(), to: "p2".into() },
                SketchPrim::Line { id: "l2".into(), from: "p2".into(), to: "p3".into() },
                SketchPrim::Line { id: "l3".into(), from: "p3".into(), to: "p4".into() },
                SketchPrim::Line { id: "l4".into(), from: "p4".into(), to: "p1".into() },
            ],
            constraints: vec![],
        }
    }

    #[test]
    fn rect_traces_to_one_profile() {
        let s = rect_sketch();
        let profs = s.to_profile_2d(&HashMap::new()).unwrap();
        assert_eq!(profs.len(), 1);
        assert_eq!(profs[0].points.len(), 4);
    }

    #[test]
    fn open_polyline_rejected() {
        let mut s = rect_sketch();
        // Drop the closing edge.
        s.primitives.retain(|p| p.id() != "l4");
        let err = s.to_profile_2d(&HashMap::new()).unwrap_err();
        assert!(matches!(err, SketchError::AmbiguousValence { .. }));
    }
}
