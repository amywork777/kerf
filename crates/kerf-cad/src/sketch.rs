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
//! Branching and dangling endpoints are rejected with a clear `SketchError`.
//!
//! Disjoint sub-loops on a sketch are tolerated — each becomes its own
//! `Profile2D`. Downstream features (`SketchExtrude`, `SketchRevolve`)
//! decide what to do with multiple profiles. `SketchExtrude` will union
//! disjoint loops into a single solid (and reject overlapping inner loops
//! with `SketchError::DisjointSubLoops` since polygon-with-holes isn't
//! supported yet — overlaps therefore have no well-defined boolean meaning).
//!
//! 2D editing operations — `TrimLine`, `ExtendLine`, `FilletCorner` — are
//! authored as `SketchPrim` variants too. They are resolved BEFORE loop
//! tracing: a `TrimLine` rewrites a `Line`'s endpoint, an `ExtendLine`
//! extends one, and a `FilletCorner` replaces a Point shared by two Lines
//! with a tangent arc of given radius (introducing two new Points and one
//! Arc, and rewriting the affected Lines' endpoints).
//!
//! The new feature variants `Feature::SketchExtrude` and
//! `Feature::SketchRevolve` route the sketch through `to_profile_2d` and then
//! into the existing `extrude_polygon` / `revolve_polyline` kernel paths.

use std::collections::{BTreeMap, HashMap, HashSet};

use serde::{Deserialize, Serialize};
use thiserror::Error;

use crate::feature::Profile2D;
use crate::scalar::Scalar;

/// The plane the sketch lives on. The named-ref-plane variant resolves at
/// SketchExtrude evaluation time against a `Feature::RefPlane` of the same
/// id in the model: the resulting solid is built in the local sketch frame
/// (XY) and then transformed so the sketch's local +Z axis aligns with the
/// ref plane's normal axis, with the local origin translated to the ref
/// plane's `position`.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub enum SketchPlane {
    /// XY plane at z = 0 — the default for `SketchExtrude`.
    Xy,
    /// XZ plane at y = 0 — used for `SketchRevolve` (axisymmetric profiles
    /// live in this plane around the z-axis).
    Xz,
    /// YZ plane at x = 0.
    Yz,
    /// A named reference plane. Resolved against `Feature::RefPlane` with
    /// the same id at evaluation time.
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

    /// Trim `line` (a Line primitive id) at the given Point id. The trim
    /// rewrites the Line's `from` or `to` endpoint to `at_point` —
    /// whichever one keeps the kept-portion non-degenerate. The
    /// `at_point` MUST already exist as a Point primitive in the sketch
    /// (the sketcher computes intersections at authoring time and inserts
    /// a Point there). After resolution this primitive is removed from
    /// the graph; only its effect on the Line remains.
    TrimLine {
        id: String,
        line: String,
        at_point: String,
    },

    /// Extend `line` (a Line primitive id) so that one of its endpoints
    /// reaches `to_point`. Whichever existing endpoint is geometrically
    /// closer to `to_point` is replaced by `to_point`; the other is
    /// preserved. `to_point` MUST already exist as a Point primitive.
    ExtendLine {
        id: String,
        line: String,
        to_point: String,
    },

    /// Replace the corner at `corner_point` (shared by exactly two Lines)
    /// with a tangent arc of `radius`. Two new Points are inserted at the
    /// tangent points along the two Lines; the original Lines' endpoint
    /// at `corner_point` is rewritten to its respective tangent point;
    /// and an Arc primitive is added between the two tangent points.
    /// `corner_point` itself stays in the sketch as a free / construction
    /// Point — it just no longer participates in any Line or Arc.
    FilletCorner {
        id: String,
        corner_point: String,
        radius: Scalar,
    },
}

impl SketchPrim {
    pub fn id(&self) -> &str {
        match self {
            SketchPrim::Point { id, .. }
            | SketchPrim::Line { id, .. }
            | SketchPrim::Circle { id, .. }
            | SketchPrim::Arc { id, .. }
            | SketchPrim::TrimLine { id, .. }
            | SketchPrim::ExtendLine { id, .. }
            | SketchPrim::FilletCorner { id, .. } => id,
        }
    }
}

/// Geometric constraints. Stored on the sketch and round-tripped through
/// JSON, but NOT enforced today — there is no solver. They are
/// forward-compatible slots for a future bidirectional constraint engine.
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
    #[error(
        "sketch has overlapping disjoint sub-loops (loops cross or partially overlap — only fully-disjoint or fully-nested is supported)"
    )]
    DisjointSubLoops,
    #[error("expected a single closed loop, got {0}")]
    NotSingleLoop(usize),
    #[error("operation '{op}' references unknown line '{line}'")]
    UnknownLine { op: String, line: String },
    #[error("FilletCorner '{id}' needs corner_point shared by exactly two Lines (got {n})")]
    FilletNotACorner { id: String, n: usize },
    #[error("FilletCorner '{id}': lines are colinear — no fillet possible")]
    FilletColinear { id: String },
    #[error("FilletCorner '{id}' radius must be > 0 (got {got})")]
    FilletRadius { id: String, got: f64 },
    #[error("FilletCorner '{id}': radius {radius} is too large for adjacent line lengths")]
    FilletTooLarge { id: String, radius: f64 },
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
        // Phase 0: rewrite primitives by applying TrimLine / ExtendLine /
        // FilletCorner operations. The result is a flat list containing
        // only Point / Line / Circle / Arc primitives.
        let primitives = preprocess_edits(&self.primitives, params)?;

        // Phase 1: collect Points with duplicate-id check.
        let mut points: BTreeMap<String, [f64; 2]> = BTreeMap::new();
        let mut seen_ids: HashSet<String> = HashSet::new();
        for prim in &primitives {
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

        // Phase 2: lines, arcs, circles.
        let mut lines = Vec::new();
        let mut arcs = Vec::new();
        let mut circles = Vec::new();
        for prim in &primitives {
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
                // These are eliminated in preprocess_edits.
                SketchPrim::TrimLine { .. }
                | SketchPrim::ExtendLine { .. }
                | SketchPrim::FilletCorner { .. } => {
                    unreachable!("edit primitives removed during preprocess");
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

/// Resolve all 2D-edit primitives (TrimLine / ExtendLine / FilletCorner)
/// by rewriting the primitive list. The result contains only Points,
/// Lines, Circles, and Arcs. Edits are applied in author-order, so a
/// Trim followed by a Fillet on the same Line composes correctly.
fn preprocess_edits(
    primitives: &[SketchPrim],
    params: &HashMap<String, f64>,
) -> Result<Vec<SketchPrim>, SketchError> {
    // We model the working set as a Vec of primitives, keyed by id where
    // useful.
    let mut out: Vec<SketchPrim> = primitives.iter().cloned().collect();

    // Apply edits one by one, removing each as we apply it. We loop
    // until no edits remain.
    let mut next_arc_uid: usize = 0;
    loop {
        let edit_idx = out.iter().position(|p| {
            matches!(
                p,
                SketchPrim::TrimLine { .. }
                    | SketchPrim::ExtendLine { .. }
                    | SketchPrim::FilletCorner { .. }
            )
        });
        let Some(i) = edit_idx else {
            return Ok(out);
        };
        let edit = out.remove(i);
        match edit {
            SketchPrim::TrimLine {
                id: _,
                line,
                at_point,
            } => apply_trim(&mut out, &line, &at_point, params)?,
            SketchPrim::ExtendLine {
                id: _,
                line,
                to_point,
            } => apply_extend(&mut out, &line, &to_point, params)?,
            SketchPrim::FilletCorner {
                id,
                corner_point,
                radius,
            } => apply_fillet(&mut out, &id, &corner_point, &radius, params, &mut next_arc_uid)?,
            _ => unreachable!(),
        }
    }
}

/// Resolve a Point id to its [x, y] coordinates from the current
/// primitives list (with parameter substitution).
fn lookup_point(
    primitives: &[SketchPrim],
    id: &str,
    params: &HashMap<String, f64>,
) -> Option<[f64; 2]> {
    for p in primitives {
        if let SketchPrim::Point { id: pid, x, y } = p {
            if pid == id {
                let xv = x.resolve(params).ok()?;
                let yv = y.resolve(params).ok()?;
                return Some([xv, yv]);
            }
        }
    }
    None
}

fn apply_trim(
    primitives: &mut Vec<SketchPrim>,
    line_id: &str,
    at_point: &str,
    params: &HashMap<String, f64>,
) -> Result<(), SketchError> {
    // Find the line.
    let line_idx = primitives
        .iter()
        .position(|p| matches!(p, SketchPrim::Line { id, .. } if id == line_id))
        .ok_or_else(|| SketchError::UnknownLine {
            op: "TrimLine".into(),
            line: line_id.into(),
        })?;
    let (from, to) = match &primitives[line_idx] {
        SketchPrim::Line { from, to, .. } => (from.clone(), to.clone()),
        _ => unreachable!(),
    };
    let p_from = lookup_point(primitives, &from, params).ok_or_else(|| {
        SketchError::UnknownPoint {
            prim: line_id.into(),
            point: from.clone(),
        }
    })?;
    let p_to = lookup_point(primitives, &to, params).ok_or_else(|| {
        SketchError::UnknownPoint {
            prim: line_id.into(),
            point: to.clone(),
        }
    })?;
    let p_at = lookup_point(primitives, at_point, params).ok_or_else(|| {
        SketchError::UnknownPoint {
            prim: "TrimLine".into(),
            point: at_point.into(),
        }
    })?;
    // Decide: rewrite the closer endpoint to at_point so the kept
    // segment is the longer half. A trim usually clips off one end —
    // by convention here we drop the end nearer at_point and keep the
    // farther one.
    let d_from = dist2(p_from, p_at);
    let d_to = dist2(p_to, p_at);
    let new_line = if d_from < d_to {
        SketchPrim::Line {
            id: line_id.into(),
            from: at_point.into(),
            to,
        }
    } else {
        SketchPrim::Line {
            id: line_id.into(),
            from,
            to: at_point.into(),
        }
    };
    primitives[line_idx] = new_line;
    Ok(())
}

fn apply_extend(
    primitives: &mut Vec<SketchPrim>,
    line_id: &str,
    to_point: &str,
    params: &HashMap<String, f64>,
) -> Result<(), SketchError> {
    let line_idx = primitives
        .iter()
        .position(|p| matches!(p, SketchPrim::Line { id, .. } if id == line_id))
        .ok_or_else(|| SketchError::UnknownLine {
            op: "ExtendLine".into(),
            line: line_id.into(),
        })?;
    let (from, to) = match &primitives[line_idx] {
        SketchPrim::Line { from, to, .. } => (from.clone(), to.clone()),
        _ => unreachable!(),
    };
    let p_from = lookup_point(primitives, &from, params).ok_or_else(|| {
        SketchError::UnknownPoint {
            prim: line_id.into(),
            point: from.clone(),
        }
    })?;
    let p_to = lookup_point(primitives, &to, params).ok_or_else(|| {
        SketchError::UnknownPoint {
            prim: line_id.into(),
            point: to.clone(),
        }
    })?;
    let p_target = lookup_point(primitives, to_point, params).ok_or_else(|| {
        SketchError::UnknownPoint {
            prim: "ExtendLine".into(),
            point: to_point.into(),
        }
    })?;
    // Replace whichever endpoint is closer to to_point.
    let d_from = dist2(p_from, p_target);
    let d_to = dist2(p_to, p_target);
    let new_line = if d_from < d_to {
        SketchPrim::Line {
            id: line_id.into(),
            from: to_point.into(),
            to,
        }
    } else {
        SketchPrim::Line {
            id: line_id.into(),
            from,
            to: to_point.into(),
        }
    };
    primitives[line_idx] = new_line;
    Ok(())
}

/// Replace a corner with a tangent arc. We do this by:
/// 1. Finding exactly two Lines that touch `corner_point`.
/// 2. Computing each line's outgoing direction from the corner.
/// 3. Computing the half-angle bisector and the inset distance d such
///    that an inscribed circle of radius `r` is tangent to both lines:
///    d = r / tan(theta/2), where theta is the angle between the lines.
/// 4. Tangent points are at `corner_point + d * dir_i` along each line.
/// 5. Insert two new Points (tangent points), insert one Arc between them,
///    and rewrite the two Lines so their endpoints at corner_point are
///    instead at the new tangent points.
fn apply_fillet(
    primitives: &mut Vec<SketchPrim>,
    fillet_id: &str,
    corner_point: &str,
    radius: &Scalar,
    params: &HashMap<String, f64>,
    next_arc_uid: &mut usize,
) -> Result<(), SketchError> {
    let r = radius.resolve(params).map_err(SketchError::Expr)?;
    if r <= 0.0 {
        return Err(SketchError::FilletRadius {
            id: fillet_id.into(),
            got: r,
        });
    }

    // Find lines incident on corner_point.
    let mut incident: Vec<(usize, /*from_is_corner*/ bool)> = Vec::new();
    for (i, p) in primitives.iter().enumerate() {
        if let SketchPrim::Line { from, to, .. } = p {
            if from == corner_point {
                incident.push((i, true));
            } else if to == corner_point {
                incident.push((i, false));
            }
        }
    }
    if incident.len() != 2 {
        return Err(SketchError::FilletNotACorner {
            id: fillet_id.into(),
            n: incident.len(),
        });
    }
    let (l1_idx, l1_from) = incident[0];
    let (l2_idx, l2_from) = incident[1];

    // Resolve corner point and "other" endpoints.
    let p_corner = lookup_point(primitives, corner_point, params).ok_or_else(|| {
        SketchError::UnknownPoint {
            prim: "FilletCorner".into(),
            point: corner_point.into(),
        }
    })?;

    let other_id_1 = match &primitives[l1_idx] {
        SketchPrim::Line { from, to, .. } => {
            if l1_from {
                to.clone()
            } else {
                from.clone()
            }
        }
        _ => unreachable!(),
    };
    let other_id_2 = match &primitives[l2_idx] {
        SketchPrim::Line { from, to, .. } => {
            if l2_from {
                to.clone()
            } else {
                from.clone()
            }
        }
        _ => unreachable!(),
    };
    let p_other_1 = lookup_point(primitives, &other_id_1, params).ok_or_else(|| {
        SketchError::UnknownPoint {
            prim: "FilletCorner".into(),
            point: other_id_1.clone(),
        }
    })?;
    let p_other_2 = lookup_point(primitives, &other_id_2, params).ok_or_else(|| {
        SketchError::UnknownPoint {
            prim: "FilletCorner".into(),
            point: other_id_2.clone(),
        }
    })?;

    // Outgoing unit vectors from the corner along each line.
    let (d1x, d1y) = unit(p_other_1[0] - p_corner[0], p_other_1[1] - p_corner[1])
        .ok_or_else(|| SketchError::FilletColinear {
            id: fillet_id.into(),
        })?;
    let (d2x, d2y) = unit(p_other_2[0] - p_corner[0], p_other_2[1] - p_corner[1])
        .ok_or_else(|| SketchError::FilletColinear {
            id: fillet_id.into(),
        })?;

    // Angle between outgoing rays.
    let dot = d1x * d2x + d1y * d2y;
    let dot_clamped = dot.clamp(-1.0, 1.0);
    let theta = dot_clamped.acos();
    if theta < 1e-9 || (std::f64::consts::PI - theta).abs() < 1e-9 {
        return Err(SketchError::FilletColinear {
            id: fillet_id.into(),
        });
    }
    let half = theta * 0.5;
    let inset = r / half.tan();
    if inset <= 0.0 {
        return Err(SketchError::FilletTooLarge {
            id: fillet_id.into(),
            radius: r,
        });
    }

    // Tangent points along each line, distance `inset` from the corner.
    let t1 = [p_corner[0] + inset * d1x, p_corner[1] + inset * d1y];
    let t2 = [p_corner[0] + inset * d2x, p_corner[1] + inset * d2y];

    // Validate inset doesn't run past the other endpoint.
    let len1 = ((p_other_1[0] - p_corner[0]).powi(2)
        + (p_other_1[1] - p_corner[1]).powi(2))
    .sqrt();
    let len2 = ((p_other_2[0] - p_corner[0]).powi(2)
        + (p_other_2[1] - p_corner[1]).powi(2))
    .sqrt();
    if inset >= len1 || inset >= len2 {
        return Err(SketchError::FilletTooLarge {
            id: fillet_id.into(),
            radius: r,
        });
    }

    // Arc center: along the bisector of the two outgoing rays, distance
    // r / sin(theta/2) from the corner.
    let bx = d1x + d2x;
    let by = d1y + d2y;
    let (bxn, byn) = unit(bx, by).ok_or_else(|| SketchError::FilletColinear {
        id: fillet_id.into(),
    })?;
    let center_dist = r / half.sin();
    let center = [
        p_corner[0] + center_dist * bxn,
        p_corner[1] + center_dist * byn,
    ];

    // Angles from center to t1 and t2.
    let a1 = (t1[1] - center[1]).atan2(t1[0] - center[0]);
    let a2 = (t2[1] - center[1]).atan2(t2[0] - center[0]);

    // Pick the arc direction that goes the SHORT way around (through the
    // bisector side closer to the corner). We compute both candidate arc
    // sweeps (CCW from a1 to a2 either directly or via 2π - delta) and
    // pick the smaller one.
    let mut sweep = a2 - a1;
    while sweep <= -std::f64::consts::PI {
        sweep += 2.0 * std::f64::consts::PI;
    }
    while sweep > std::f64::consts::PI {
        sweep -= 2.0 * std::f64::consts::PI;
    }
    let (arc_start, arc_end) = if sweep >= 0.0 {
        (a1, a1 + sweep)
    } else {
        (a2, a2 - sweep)
    };
    // Now arc_start < arc_end and the arc samples from arc_start CCW
    // around `center` to arc_end. Reaffirm the endpoints actually
    // coincide with t1/t2 within tolerance — but the tracer matches by
    // approximate position so a slight rounding mismatch is fine.

    // Generate unique names for the introduced primitives.
    let uid = *next_arc_uid;
    *next_arc_uid += 1;
    let cname = format!("__fillet_c_{fillet_id}_{uid}");
    let p_t1 = format!("__fillet_t1_{fillet_id}_{uid}");
    let p_t2 = format!("__fillet_t2_{fillet_id}_{uid}");
    let arc_id = format!("__fillet_arc_{fillet_id}_{uid}");

    // Insert center, tangent points.
    primitives.push(SketchPrim::Point {
        id: cname.clone(),
        x: Scalar::lit(center[0]),
        y: Scalar::lit(center[1]),
    });
    primitives.push(SketchPrim::Point {
        id: p_t1.clone(),
        x: Scalar::lit(t1[0]),
        y: Scalar::lit(t1[1]),
    });
    primitives.push(SketchPrim::Point {
        id: p_t2.clone(),
        x: Scalar::lit(t2[0]),
        y: Scalar::lit(t2[1]),
    });
    // Add arc primitive. Use generous segment count proportional to sweep.
    let abs_sweep = (arc_end - arc_start).abs();
    let n_segments = ((abs_sweep / std::f64::consts::FRAC_PI_8).ceil() as usize).max(4);
    primitives.push(SketchPrim::Arc {
        id: arc_id,
        center: cname,
        radius: Scalar::lit(r),
        start_angle: Scalar::lit(arc_start),
        end_angle: Scalar::lit(arc_end),
        n_segments,
    });

    // Determine which Point id (p_t1 or p_t2) corresponds to which line.
    // line 1's outgoing endpoint is t1; line 2's is t2. So line 1's
    // corner endpoint becomes p_t1; line 2's becomes p_t2.

    // Rewrite line 1.
    if let SketchPrim::Line { id, from, to } = primitives[l1_idx].clone() {
        let new = if l1_from {
            SketchPrim::Line {
                id,
                from: p_t1.clone(),
                to,
            }
        } else {
            SketchPrim::Line {
                id,
                from,
                to: p_t1.clone(),
            }
        };
        primitives[l1_idx] = new;
    }
    if let SketchPrim::Line { id, from, to } = primitives[l2_idx].clone() {
        let new = if l2_from {
            SketchPrim::Line {
                id,
                from: p_t2.clone(),
                to,
            }
        } else {
            SketchPrim::Line {
                id,
                from,
                to: p_t2.clone(),
            }
        };
        primitives[l2_idx] = new;
    }
    Ok(())
}

fn dist2(a: [f64; 2], b: [f64; 2]) -> f64 {
    let dx = a[0] - b[0];
    let dy = a[1] - b[1];
    dx * dx + dy * dy
}

fn unit(x: f64, y: f64) -> Option<(f64, f64)> {
    let n = (x * x + y * y).sqrt();
    if n < 1e-12 {
        None
    } else {
        Some((x / n, y / n))
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

/// Resolve a Profile2D's points to plain f64 pairs (using the model's
/// param table). Profiles emitted by `to_profile_2d` already have all
/// literal coordinates so this is a fast pass.
pub fn profile_points_xy(
    profile: &Profile2D,
    params: &HashMap<String, f64>,
) -> Result<Vec<[f64; 2]>, SketchError> {
    let mut out = Vec::with_capacity(profile.points.len());
    for p in &profile.points {
        let x = p[0].resolve(params).map_err(SketchError::Expr)?;
        let y = p[1].resolve(params).map_err(SketchError::Expr)?;
        out.push([x, y]);
    }
    Ok(out)
}

/// Even–odd ray cast: is `pt` strictly inside the polygon `poly`
/// (CCW or CW, doesn't matter)? Boundary classification is undefined,
/// but for our use (multi-loop containment) we only call this on a
/// point well inside the candidate inner polygon, so boundary cases are
/// not a problem.
pub fn point_in_polygon(pt: [f64; 2], poly: &[[f64; 2]]) -> bool {
    let n = poly.len();
    if n < 3 {
        return false;
    }
    let (x, y) = (pt[0], pt[1]);
    let mut inside = false;
    let mut j = n - 1;
    for i in 0..n {
        let (xi, yi) = (poly[i][0], poly[i][1]);
        let (xj, yj) = (poly[j][0], poly[j][1]);
        let cross = (yi > y) != (yj > y);
        if cross {
            let x_intersect = (xj - xi) * (y - yi) / (yj - yi + f64::EPSILON) + xi;
            if x < x_intersect {
                inside = !inside;
            }
        }
        j = i;
    }
    inside
}

/// Test: for two polygons that share NO common ground (their edges don't
/// overlap by construction, since they came from disjoint connected
/// components of the sketch graph), determine the relationship: either
/// `b` is fully inside `a`, fully outside, or they cross. Crossing →
/// return true so the caller can flag DisjointSubLoops.
pub fn polygons_cross(a: &[[f64; 2]], b: &[[f64; 2]]) -> bool {
    if a.len() < 3 || b.len() < 3 {
        return false;
    }
    // If any vertex of B is inside A and any other vertex of B is
    // outside A, the two cross.
    let mut any_inside = false;
    let mut any_outside = false;
    for &p in b {
        if point_in_polygon(p, a) {
            any_inside = true;
        } else {
            any_outside = true;
        }
        if any_inside && any_outside {
            return true;
        }
    }
    // Symmetric check.
    let mut any_in_b = false;
    let mut any_out_b = false;
    for &p in a {
        if point_in_polygon(p, b) {
            any_in_b = true;
        } else {
            any_out_b = true;
        }
        if any_in_b && any_out_b {
            return true;
        }
    }
    false
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
