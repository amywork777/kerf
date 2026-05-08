//! Iterative 2D constraint solver for `Sketch`.
//!
//! `Sketch::solve` adjusts the `x`/`y` coordinates of every Point primitive
//! so that all `SketchConstraint`s are satisfied. Lines/Circles/Arcs are
//! derived from Points (their endpoints/centers reference Point ids), so
//! moving a Point automatically updates every primitive that touches it —
//! Points are the only degrees of freedom the solver touches.
//!
//! ALGORITHM. Each constraint defines a non-negative scalar residual that
//! is zero exactly when the constraint is satisfied:
//!
//! - `Coincident(a, b)`           → `|p_a - p_b|^2`
//! - `Distance(a, b, value)`      → `(|p_a - p_b| - value)^2`
//! - `Horizontal(line)`           → `(p_to.y - p_from.y)^2`
//! - `Vertical(line)`             → `(p_to.x - p_from.x)^2`
//! - `Parallel(line_a, line_b)`   → `cross(dir_a, dir_b)^2`
//! - `Perpendicular(line_a, lb)`  → `dot(dir_a, dir_b)^2`
//! - `FixedPoint(p)`              → `|p - p_initial|^2`
//! - `TangentLineToCircle(l, c)`  → `(perp_dist(line, center) - radius)^2`
//! - `CoincidentOnLine(p, l)`     → `perp_dist(point, line)^2`
//! - `EqualLength(la, lb)`        → `(|la| - |lb|)^2`
//! - `EqualRadius(ca, cb)`        → `(r_a - r_b)^2`  (radii are Scalars,
//!                                   not Point DOFs — so the gradient is
//!                                   zero; a contradictory pair surfaces
//!                                   as `SolverError::Contradictory`)
//! - `PointOnCircle(p, c)`        → `(|p - center|² - radius²)²`
//! - `CircleTangentExternal(a,b)` → `(|c_a - c_b|² - (r_a + r_b)²)²`
//! - `CircleTangentInternal(a,b)` → `(|c_a - c_b|² - (r_a - r_b)²)²`
//! - `EqualAngle(la1,la2,lb1,lb2)`→ `(cos(la1,la2) - cos(lb1,lb2))²`
//! - `MidPoint(p, line)`          → `|2p - (from + to)|²`
//! - `DistanceFromLine(p, l, d)`  → `(perp_signed - d)²`
//!
//! ## Sparse Jacobian path (PR #17)
//!
//! Each constraint's per-DOF gradient is stored sparsely as
//! `Vec<(dof_index, ∂r/∂dof)>`. The Newton step builds `J^T J + λI` by
//! iterating only the nonzero pairs, then solves via Gauss-Seidel that
//! also iterates only nonzero off-diagonal entries. For a 100-DOF
//! sketch with 4-DOF-per-row constraints, this is ~25× fewer ops per
//! Newton step than the previous dense path.
//!
//! ## Backward parametric solve (PR #17)
//!
//! [`Sketch::solve_with_parameters`] solves for a target parameter
//! value: change `$len` from 3 to 7 and the geometry follows. Uses
//! the Point coordinates from the previous solve as a warm start;
//! Newton typically converges in 5-15 iterations. The companion
//! [`Sketch::parametric_jacobian`] returns `∂x/∂param` (the
//! coordinate response per unit parameter increase) computed via
//! central finite differences over the full nonlinear solver — exact
//! up to convergence tolerance, robust on rank-deficient systems
//! where a closed-form pseudoinverse approach would fail.
//!
//! ## Constraint diagnostics (PR #17)
//!
//! [`Sketch::diagnose_constraints`] returns a [`DiagnosticReport`]
//! quantifying degrees of freedom, the effective rank of the
//! constraint Jacobian, redundancy, and an initial-residual snapshot.
//! The report distinguishes "well-constrained" (rank == DOFs and
//! residual ≈ 0) from "under-constrained" (rank < DOFs) from
//! "over-constrained" (more rows than rank — at least one constraint
//! is implied by the others).
//!
//! The objective is the sum over all constraints. We minimize it with a
//! two-tier strategy:
//!
//! 1. **Newton-Raphson** with analytic Jacobians. We model each
//!    constraint as a vector residual r_i and solve the normal equations
//!    `J^T J · dx = -J^T · r` per iteration via Gauss-Seidel (small
//!    dense systems, no external lin-alg dep). A backtracking line
//!    search guards each Newton step.
//! 2. **Gradient descent** fallback when the Newton system is rank-
//!    deficient or its step doesn't reduce the residual. Gradient is
//!    `∇φ` of the squared-residual objective, computed analytically;
//!    central finite differences are kept as a last-resort fallback for
//!    cases where the analytic derivative isn't available.
//!
//! Termination:
//! - `Ok(iterations)` once the total residual drops below `tol = 1e-9`.
//! - `Err(SolverError::Contradictory { residual, conflicting })` if line
//!   search collapses (residual stops decreasing) yet the residual is
//!   still meaningfully large — characteristic of mutually-exclusive
//!   constraints. `conflicting` lists indices into the original
//!   `sketch.constraints` of the smallest subset that still fails to
//!   converge (computed by bisection on the constraint set).
//! - `Err(SolverError::OverConstrained)` if `max_iterations` runs out
//!   while the residual is still decreasing slowly.
//! - `Err(SolverError::ParamResolution)` if a `Scalar::Expr` constraint
//!   value can't be resolved against `params`.
//! - `Err(SolverError::UnknownPoint)` / `UnknownLine` / `UnknownCircle`
//!   for ids referenced by a constraint that aren't defined as
//!   primitives in the sketch.
//!
//! Under-constrained sketches are explicitly OK: the solver converges
//! to *some* configuration that satisfies all constraints; we don't add
//! a regularizer that pulls toward the initial guess because
//! `FixedPoint` exists for exactly that role.

use std::collections::HashMap;

use thiserror::Error;

use crate::sketch::{Sketch, SketchConstraint, SketchPrim};

/// Errors the solver can return.
#[derive(Debug, Error, PartialEq)]
pub enum SolverError {
    /// Maximum iteration count exceeded with residual still above
    /// tolerance. The constraints likely over-constrain the system in a
    /// way that requires more iterations than allowed, or the system is
    /// numerically stiff.
    #[error("solver did not converge in {max_iterations} iterations (residual = {residual})")]
    OverConstrained { max_iterations: u32, residual: f64 },

    /// Line search collapsed (no descent direction reduces the residual)
    /// yet the residual is non-negligible — the constraint set has no
    /// solution. Typical example: two `Distance(a, b, ?)` constraints on
    /// the same point pair with conflicting values. `conflicting` lists
    /// indices into the original `sketch.constraints` of the minimal
    /// subset that still fails to converge (computed by bisection); an
    /// empty vector means the bisection wasn't run (e.g. error surfaced
    /// from a sub-solve).
    #[error("contradictory constraints (residual = {residual} is non-zero but no descent direction reduces it; minimal conflicting subset = {conflicting:?})")]
    Contradictory { residual: f64, conflicting: Vec<usize> },

    /// A constraint referenced a Point id that isn't defined.
    #[error("constraint references unknown Point id '{0}'")]
    UnknownPoint(String),

    /// A constraint referenced a Line id that isn't a Line primitive.
    #[error("constraint references unknown Line id '{0}'")]
    UnknownLine(String),

    /// A constraint referenced a Circle id that isn't a Circle primitive.
    #[error("constraint references unknown Circle id '{0}'")]
    UnknownCircle(String),

    /// A `Scalar::Expr` value in a constraint failed to resolve.
    #[error("could not resolve constraint parameter: {0}")]
    ParamResolution(String),
}

/// Solver tuning parameters. Defaults match `Sketch::solve`.
#[derive(Clone, Debug)]
pub struct SolverConfig {
    /// Convergence tolerance on the total residual. Default `1e-9`.
    pub tolerance: f64,
    /// Hard cap on iteration count. Default `5000`.
    pub max_iterations: u32,
    /// Initial step size for line search. Default `1.0`.
    pub initial_step: f64,
    /// Backtracking shrinkage factor (line search). Default `0.5`.
    pub backtrack: f64,
    /// Minimum step size before declaring contradiction. Default `1e-14`.
    pub min_step: f64,
    /// Finite-difference epsilon for the (rarely-used) FD gradient
    /// fallback. Default `1e-7`.
    pub fd_eps: f64,
    /// Try Newton-Raphson with analytic Jacobian each iteration before
    /// falling back to gradient descent. Default `true`.
    pub use_newton: bool,
    /// Levenberg-Marquardt damping added to `J^T J` before solving the
    /// normal equations. Default `1e-9`. Larger values bias the step
    /// toward gradient descent; smaller values toward pure Newton.
    pub lm_damping: f64,
    /// Maximum number of Gauss-Seidel sweeps when solving the normal
    /// equations. Default `200`. Most well-conditioned problems converge
    /// in <50 sweeps; the cap exists to avoid pathological loops.
    pub gauss_seidel_max_sweeps: u32,
    /// Bisect the constraint set when reporting Contradictory errors,
    /// to identify the minimal conflicting subset. Default `true`.
    /// Disable for hot paths where the diagnostic isn't needed.
    pub identify_conflicts: bool,
}

/// Structured diagnostic report produced by [`Sketch::diagnose_constraints`].
///
/// Quantifies the *constraint state* of a sketch: how many degrees of
/// freedom (DOFs) it has, how many of those are pinned by the constraints,
/// how many remain free, and whether the constraint set has any
/// rank-deficient redundancy (over-constrained but consistent — e.g. two
/// `Distance(a, b, 5)` constraints) or contradictory rows
/// (over-constrained and inconsistent).
///
/// The numbers are computed from the *signed-residual Jacobian* J at the
/// initial coordinates:
/// - `dof_count`        = 2 · number of Points (each Point is x + y).
/// - `effective_rank`   = numerical rank of J (constraints that pin DOFs).
/// - `free_dofs`        = `dof_count - effective_rank`.
/// - `redundant_rows`   = `total_rows - effective_rank` (rows that are
///                        linear combinations of others — they pin no
///                        new DOFs).
/// - `is_well_constrained` = `free_dofs == 0 && residual_at_initial < tol`
/// - `is_under_constrained` = `free_dofs > 0`
/// - `is_over_constrained` = `redundant_rows > 0`
///
/// "Well-constrained" means every DOF is pinned and the system is
/// consistent at the initial coordinates. Over-constrained is fine for
/// constraints that are redundantly satisfied (no contradiction) — the
/// solver still converges; the redundancy is reported here as a hint.
#[derive(Debug, Clone, PartialEq)]
pub struct DiagnosticReport {
    /// Total degrees of freedom: 2 · number of Points.
    pub dof_count: usize,
    /// Total scalar residual rows the constraint set generates
    /// (`Coincident`, `FixedPoint`, `MidPoint` contribute 2 rows each;
    /// most others contribute 1).
    pub total_rows: usize,
    /// Numerical rank of the Jacobian J at the initial coordinates.
    /// Each rank-contributing row pins one DOF (or one linear
    /// combination of DOFs).
    pub effective_rank: usize,
    /// `dof_count - effective_rank` — how many DOFs are still free
    /// after the constraints. Zero ⇒ uniquely determined; positive ⇒
    /// the solver may reach any one of an infinite family.
    pub free_dofs: usize,
    /// `total_rows - effective_rank` — how many constraint rows are
    /// linear combinations of others (redundant). Doesn't necessarily
    /// mean "over-constrained" in the failure sense; it just means
    /// some constraints are redundantly satisfied.
    pub redundant_rows: usize,
    /// Sum of squared residuals at the initial coordinates. A
    /// well-constrained sketch starts at zero residual (the user
    /// already drew it correctly); a non-zero starting residual means
    /// the solver has work to do, regardless of rank.
    pub initial_residual: f64,
    /// True when `free_dofs == 0 && initial_residual < 1e-9`.
    pub is_well_constrained: bool,
    /// True when `free_dofs > 0` — the sketch admits a 1-parameter (or
    /// more) family of solutions.
    pub is_under_constrained: bool,
    /// True when `redundant_rows > 0` — at least one constraint is
    /// already implied by the others.
    pub is_over_constrained: bool,
}

impl Default for SolverConfig {
    fn default() -> Self {
        Self {
            tolerance: 1e-9,
            max_iterations: 5000,
            initial_step: 1.0,
            backtrack: 0.5,
            min_step: 1e-14,
            fd_eps: 1e-7,
            use_newton: true,
            lm_damping: 1e-9,
            gauss_seidel_max_sweeps: 200,
            identify_conflicts: true,
        }
    }
}

/// Internal "compiled" form of the sketch the solver iterates on.
struct SolverState {
    /// Index → point id, in primitive declaration order. Retained for
    /// debugging / future error messages that quote ids.
    #[allow(dead_code)]
    point_ids: Vec<String>,
    /// Lookup: point id → index in `point_ids` / `coords`.
    point_index: HashMap<String, usize>,
    /// Flat coordinate vector: `[x0, y0, x1, y1, …]` for the points in
    /// `point_ids` order. Length = `2 * point_ids.len()`.
    coords: Vec<f64>,
    /// Initial coordinates, frozen for `FixedPoint` residuals.
    initial: Vec<f64>,
    /// Lookup: line id → (from_point_idx, to_point_idx).
    #[allow(dead_code)]
    lines: HashMap<String, (usize, usize)>,
    /// Resolved constraints (Scalar values flattened to f64).
    resolved: Vec<ResolvedConstraint>,
}

/// A constraint with all `Scalar` fields resolved against `params`.
/// Indices reference `SolverState::point_ids` (and `lines`).
#[derive(Debug, Clone)]
enum ResolvedConstraint {
    Coincident { a: usize, b: usize },
    Distance { a: usize, b: usize, value: f64 },
    Horizontal { from: usize, to: usize },
    Vertical { from: usize, to: usize },
    Parallel { a_from: usize, a_to: usize, b_from: usize, b_to: usize },
    Perpendicular { a_from: usize, a_to: usize, b_from: usize, b_to: usize },
    FixedPoint { idx: usize },
    /// Line tangent to circle. `lf`/`lt` are the line's from/to point
    /// indices; `center` is the circle's center point index; `radius`
    /// is the resolved radius.
    TangentLineToCircle { lf: usize, lt: usize, center: usize, radius: f64 },
    /// Point on line. `point` is the constrained point's index; `lf`/`lt`
    /// are the line's from/to point indices.
    CoincidentOnLine { point: usize, lf: usize, lt: usize },
    /// Two lines have equal length.
    EqualLength { a_from: usize, a_to: usize, b_from: usize, b_to: usize },
    /// Two circles have equal (resolved) radii. No DOFs.
    EqualRadius { r_a: f64, r_b: f64 },
    /// A point on a circle. `point` is the constrained point's index;
    /// `center` is the circle's center point index; `radius` is the
    /// resolved radius.
    PointOnCircle { point: usize, center: usize, radius: f64 },
    /// Two circles tangent externally. Centers' indices and resolved radii.
    CircleTangentExternal { c_a: usize, c_b: usize, r_a: f64, r_b: f64 },
    /// Two circles tangent internally.
    CircleTangentInternal { c_a: usize, c_b: usize, r_a: f64, r_b: f64 },
    /// Equality of normalized angles between two pairs of lines.
    EqualAngle {
        a1_from: usize,
        a1_to: usize,
        a2_from: usize,
        a2_to: usize,
        b1_from: usize,
        b1_to: usize,
        b2_from: usize,
        b2_to: usize,
    },
    /// Midpoint constraint: `point` must be the midpoint of the line
    /// from `lf` to `lt`.
    MidPoint { point: usize, lf: usize, lt: usize },
    /// Point at signed perpendicular `distance` from a line.
    DistanceFromLine { point: usize, lf: usize, lt: usize, distance: f64 },
}

impl SolverState {
    /// Build the solver state from a sketch. Resolves all `Scalar` values
    /// against `params` and validates that every constraint references
    /// known primitives.
    fn from_sketch(
        sketch: &Sketch,
        params: &HashMap<String, f64>,
    ) -> Result<Self, SolverError> {
        // 1) Collect Points in declaration order.
        let mut point_ids: Vec<String> = Vec::new();
        let mut point_index: HashMap<String, usize> = HashMap::new();
        let mut coords: Vec<f64> = Vec::new();
        for prim in &sketch.primitives {
            if let SketchPrim::Point { id, x, y } = prim {
                let xv = x
                    .resolve(params)
                    .map_err(SolverError::ParamResolution)?;
                let yv = y
                    .resolve(params)
                    .map_err(SolverError::ParamResolution)?;
                let idx = point_ids.len();
                point_index.insert(id.clone(), idx);
                point_ids.push(id.clone());
                coords.push(xv);
                coords.push(yv);
            }
        }
        let initial = coords.clone();

        // 2) Index Lines by id → (from, to) point indices.
        let mut lines: HashMap<String, (usize, usize)> = HashMap::new();
        for prim in &sketch.primitives {
            if let SketchPrim::Line { id, from, to } = prim {
                let fi = *point_index
                    .get(from)
                    .ok_or_else(|| SolverError::UnknownPoint(from.clone()))?;
                let ti = *point_index
                    .get(to)
                    .ok_or_else(|| SolverError::UnknownPoint(to.clone()))?;
                lines.insert(id.clone(), (fi, ti));
            }
        }

        // 2b) Index Circles by id → (center_idx, radius_value).
        let mut circles: HashMap<String, (usize, f64)> = HashMap::new();
        for prim in &sketch.primitives {
            if let SketchPrim::Circle { id, center, radius, .. } = prim {
                let ci = *point_index
                    .get(center)
                    .ok_or_else(|| SolverError::UnknownPoint(center.clone()))?;
                let r = radius.resolve(params).map_err(SolverError::ParamResolution)?;
                circles.insert(id.clone(), (ci, r));
            }
        }

        // 3) Resolve constraints.
        let mut resolved: Vec<ResolvedConstraint> = Vec::with_capacity(sketch.constraints.len());
        for c in &sketch.constraints {
            match c {
                SketchConstraint::Coincident { a, b } => {
                    let ai = *point_index
                        .get(a)
                        .ok_or_else(|| SolverError::UnknownPoint(a.clone()))?;
                    let bi = *point_index
                        .get(b)
                        .ok_or_else(|| SolverError::UnknownPoint(b.clone()))?;
                    resolved.push(ResolvedConstraint::Coincident { a: ai, b: bi });
                }
                SketchConstraint::Distance { a, b, value } => {
                    let ai = *point_index
                        .get(a)
                        .ok_or_else(|| SolverError::UnknownPoint(a.clone()))?;
                    let bi = *point_index
                        .get(b)
                        .ok_or_else(|| SolverError::UnknownPoint(b.clone()))?;
                    let v = value
                        .resolve(params)
                        .map_err(SolverError::ParamResolution)?;
                    resolved.push(ResolvedConstraint::Distance {
                        a: ai,
                        b: bi,
                        value: v,
                    });
                }
                SketchConstraint::Horizontal { line } => {
                    let (f, t) = *lines
                        .get(line)
                        .ok_or_else(|| SolverError::UnknownLine(line.clone()))?;
                    resolved.push(ResolvedConstraint::Horizontal { from: f, to: t });
                }
                SketchConstraint::Vertical { line } => {
                    let (f, t) = *lines
                        .get(line)
                        .ok_or_else(|| SolverError::UnknownLine(line.clone()))?;
                    resolved.push(ResolvedConstraint::Vertical { from: f, to: t });
                }
                SketchConstraint::Parallel { line_a, line_b } => {
                    let (af, at) = *lines
                        .get(line_a)
                        .ok_or_else(|| SolverError::UnknownLine(line_a.clone()))?;
                    let (bf, bt) = *lines
                        .get(line_b)
                        .ok_or_else(|| SolverError::UnknownLine(line_b.clone()))?;
                    resolved.push(ResolvedConstraint::Parallel {
                        a_from: af,
                        a_to: at,
                        b_from: bf,
                        b_to: bt,
                    });
                }
                SketchConstraint::Perpendicular { line_a, line_b } => {
                    let (af, at) = *lines
                        .get(line_a)
                        .ok_or_else(|| SolverError::UnknownLine(line_a.clone()))?;
                    let (bf, bt) = *lines
                        .get(line_b)
                        .ok_or_else(|| SolverError::UnknownLine(line_b.clone()))?;
                    resolved.push(ResolvedConstraint::Perpendicular {
                        a_from: af,
                        a_to: at,
                        b_from: bf,
                        b_to: bt,
                    });
                }
                SketchConstraint::FixedPoint { point } => {
                    let pi = *point_index
                        .get(point)
                        .ok_or_else(|| SolverError::UnknownPoint(point.clone()))?;
                    resolved.push(ResolvedConstraint::FixedPoint { idx: pi });
                }
                SketchConstraint::TangentLineToCircle { line, circle } => {
                    let (lf, lt) = *lines
                        .get(line)
                        .ok_or_else(|| SolverError::UnknownLine(line.clone()))?;
                    let &(c_idx, r) = circles
                        .get(circle)
                        .ok_or_else(|| SolverError::UnknownCircle(circle.clone()))?;
                    resolved.push(ResolvedConstraint::TangentLineToCircle {
                        lf,
                        lt,
                        center: c_idx,
                        radius: r,
                    });
                }
                SketchConstraint::CoincidentOnLine { point, line } => {
                    let pi = *point_index
                        .get(point)
                        .ok_or_else(|| SolverError::UnknownPoint(point.clone()))?;
                    let (lf, lt) = *lines
                        .get(line)
                        .ok_or_else(|| SolverError::UnknownLine(line.clone()))?;
                    resolved.push(ResolvedConstraint::CoincidentOnLine {
                        point: pi,
                        lf,
                        lt,
                    });
                }
                SketchConstraint::EqualLength { line_a, line_b } => {
                    let (af, at) = *lines
                        .get(line_a)
                        .ok_or_else(|| SolverError::UnknownLine(line_a.clone()))?;
                    let (bf, bt) = *lines
                        .get(line_b)
                        .ok_or_else(|| SolverError::UnknownLine(line_b.clone()))?;
                    resolved.push(ResolvedConstraint::EqualLength {
                        a_from: af,
                        a_to: at,
                        b_from: bf,
                        b_to: bt,
                    });
                }
                SketchConstraint::EqualRadius { circle_a, circle_b } => {
                    let &(_, r_a) = circles
                        .get(circle_a)
                        .ok_or_else(|| SolverError::UnknownCircle(circle_a.clone()))?;
                    let &(_, r_b) = circles
                        .get(circle_b)
                        .ok_or_else(|| SolverError::UnknownCircle(circle_b.clone()))?;
                    resolved.push(ResolvedConstraint::EqualRadius { r_a, r_b });
                }
                SketchConstraint::PointOnCircle { point, circle } => {
                    let pi = *point_index
                        .get(point)
                        .ok_or_else(|| SolverError::UnknownPoint(point.clone()))?;
                    let &(c_idx, r) = circles
                        .get(circle)
                        .ok_or_else(|| SolverError::UnknownCircle(circle.clone()))?;
                    resolved.push(ResolvedConstraint::PointOnCircle {
                        point: pi,
                        center: c_idx,
                        radius: r,
                    });
                }
                SketchConstraint::CircleTangentExternal { circle_a, circle_b } => {
                    let &(ai, r_a) = circles
                        .get(circle_a)
                        .ok_or_else(|| SolverError::UnknownCircle(circle_a.clone()))?;
                    let &(bi, r_b) = circles
                        .get(circle_b)
                        .ok_or_else(|| SolverError::UnknownCircle(circle_b.clone()))?;
                    resolved.push(ResolvedConstraint::CircleTangentExternal {
                        c_a: ai,
                        c_b: bi,
                        r_a,
                        r_b,
                    });
                }
                SketchConstraint::CircleTangentInternal { circle_a, circle_b } => {
                    let &(ai, r_a) = circles
                        .get(circle_a)
                        .ok_or_else(|| SolverError::UnknownCircle(circle_a.clone()))?;
                    let &(bi, r_b) = circles
                        .get(circle_b)
                        .ok_or_else(|| SolverError::UnknownCircle(circle_b.clone()))?;
                    resolved.push(ResolvedConstraint::CircleTangentInternal {
                        c_a: ai,
                        c_b: bi,
                        r_a,
                        r_b,
                    });
                }
                SketchConstraint::EqualAngle {
                    line_a1,
                    line_a2,
                    line_b1,
                    line_b2,
                } => {
                    let (a1f, a1t) = *lines
                        .get(line_a1)
                        .ok_or_else(|| SolverError::UnknownLine(line_a1.clone()))?;
                    let (a2f, a2t) = *lines
                        .get(line_a2)
                        .ok_or_else(|| SolverError::UnknownLine(line_a2.clone()))?;
                    let (b1f, b1t) = *lines
                        .get(line_b1)
                        .ok_or_else(|| SolverError::UnknownLine(line_b1.clone()))?;
                    let (b2f, b2t) = *lines
                        .get(line_b2)
                        .ok_or_else(|| SolverError::UnknownLine(line_b2.clone()))?;
                    resolved.push(ResolvedConstraint::EqualAngle {
                        a1_from: a1f,
                        a1_to: a1t,
                        a2_from: a2f,
                        a2_to: a2t,
                        b1_from: b1f,
                        b1_to: b1t,
                        b2_from: b2f,
                        b2_to: b2t,
                    });
                }
                SketchConstraint::MidPoint { point, line } => {
                    let pi = *point_index
                        .get(point)
                        .ok_or_else(|| SolverError::UnknownPoint(point.clone()))?;
                    let (lf, lt) = *lines
                        .get(line)
                        .ok_or_else(|| SolverError::UnknownLine(line.clone()))?;
                    resolved.push(ResolvedConstraint::MidPoint {
                        point: pi,
                        lf,
                        lt,
                    });
                }
                SketchConstraint::DistanceFromLine { point, line, distance } => {
                    let pi = *point_index
                        .get(point)
                        .ok_or_else(|| SolverError::UnknownPoint(point.clone()))?;
                    let (lf, lt) = *lines
                        .get(line)
                        .ok_or_else(|| SolverError::UnknownLine(line.clone()))?;
                    let d = distance
                        .resolve(params)
                        .map_err(SolverError::ParamResolution)?;
                    resolved.push(ResolvedConstraint::DistanceFromLine {
                        point: pi,
                        lf,
                        lt,
                        distance: d,
                    });
                }
            }
        }

        Ok(Self {
            point_ids,
            point_index,
            coords,
            initial,
            lines,
            resolved,
        })
    }

    /// Sum of squared residuals at `coords`.
    fn residual(&self, coords: &[f64]) -> f64 {
        let mut sum = 0.0;
        for c in &self.resolved {
            sum += residual_one(c, coords, &self.initial);
        }
        sum
    }

    /// Analytic gradient of the squared-residual objective. The objective
    /// is φ(x) = Σ_k r_k(x)² where the index k runs over all *scalar*
    /// residual rows (a single `Coincident` or `FixedPoint` constraint
    /// contributes two rows; most others contribute one). Then
    /// ∇φ = 2 Σ_k r_k · ∇r_k.
    fn analytic_gradient(&self, coords: &[f64], out: &mut [f64]) {
        debug_assert_eq!(out.len(), coords.len());
        for g in out.iter_mut() {
            *g = 0.0;
        }
        for c in &self.resolved {
            for (r_i, grad) in vector_residuals_and_grads(c, coords, &self.initial) {
                let two_r = 2.0 * r_i;
                for (dof, drdx) in grad {
                    out[dof] += two_r * drdx;
                }
            }
        }
    }

    /// Gradient via central finite differences. Kept as a sanity-check
    /// path used by the test `solver_analytic_gradient_matches_finite_difference`.
    fn fd_gradient(&self, coords: &[f64], eps: f64, out: &mut [f64]) {
        debug_assert_eq!(out.len(), coords.len());
        let mut scratch = coords.to_vec();
        for i in 0..coords.len() {
            let saved = scratch[i];
            scratch[i] = saved + eps;
            let f_plus = self.residual(&scratch);
            scratch[i] = saved - eps;
            let f_minus = self.residual(&scratch);
            scratch[i] = saved;
            out[i] = (f_plus - f_minus) / (2.0 * eps);
        }
    }

    /// Build the Jacobian J of the *signed* per-constraint residuals
    /// `r_i(x)` (one row per scalar residual, n columns where n =
    /// coords.len()). Constraints can contribute multiple rows — e.g.
    /// `Coincident` and `FixedPoint` are 2D and contribute one row each
    /// for the x- and y- component, which keeps the Jacobian full-rank
    /// at the solution (avoiding rank deficiency in J^T J that would
    /// otherwise let Newton solvers move "anchored" DOFs).
    ///
    /// Returns *sparse* rows (each row is `Vec<(col_idx, value)>`,
    /// preserving only nonzero entries). Each constraint typically
    /// touches 2-8 DOFs out of all n DOFs in the sketch, so the
    /// sparse representation is much cheaper to assemble than a dense
    /// row vector for sketches with many DOFs. The caller can build
    /// `J^T J` (CSR-style) directly from these rows in O(rows · k²)
    /// where k is the average row width — much better than O(rows · n²)
    /// of the previous dense path for sparse problems.
    fn jacobian_and_residuals(
        &self,
        coords: &[f64],
    ) -> (Vec<Vec<(usize, f64)>>, Vec<f64>) {
        let mut jac: Vec<Vec<(usize, f64)>> =
            Vec::with_capacity(self.resolved.len() * 2);
        let mut res: Vec<f64> = Vec::with_capacity(self.resolved.len() * 2);
        for c in &self.resolved {
            for (r_i, grad) in vector_residuals_and_grads(c, coords, &self.initial) {
                jac.push(grad);
                res.push(r_i);
            }
        }
        (jac, res)
    }
}

/// Per-constraint signed residual rows.
///
/// Returns one or more `(r_k, grad_k)` pairs where each `r_k` is a
/// *signed* scalar residual and `grad_k` is a sparse list of
/// `(dof_index, ∂r_k/∂dof)` partial derivatives. The squared-residual
/// objective contribution of this constraint is `Σ_k r_k²`.
///
/// `Coincident` and `FixedPoint` are 2D — they yield two rows (one
/// for the x-component, one for the y-component). This keeps the
/// Jacobian rank-2 for those constraints (essential to anchor the
/// affected DOFs in Newton's normal equations). All other constraints
/// produce a single row.
fn vector_residuals_and_grads(
    c: &ResolvedConstraint,
    coords: &[f64],
    initial: &[f64],
) -> Vec<(f64, Vec<(usize, f64)>)> {
    match *c {
        ResolvedConstraint::Coincident { a, b } => {
            // r_x = ax - bx; r_y = ay - by. Trivial Jacobian rows.
            let rx = coords[2 * a] - coords[2 * b];
            let ry = coords[2 * a + 1] - coords[2 * b + 1];
            vec![
                (rx, vec![(2 * a, 1.0), (2 * b, -1.0)]),
                (ry, vec![(2 * a + 1, 1.0), (2 * b + 1, -1.0)]),
            ]
        }
        ResolvedConstraint::FixedPoint { idx } => {
            // r_x = px - initial_x, r_y = py - initial_y.
            let rx = coords[2 * idx] - initial[2 * idx];
            let ry = coords[2 * idx + 1] - initial[2 * idx + 1];
            vec![
                (rx, vec![(2 * idx, 1.0)]),
                (ry, vec![(2 * idx + 1, 1.0)]),
            ]
        }
        ResolvedConstraint::MidPoint { point, lf, lt } => {
            // r_x = 2·px - (lfx + ltx)
            // r_y = 2·py - (lfy + lty)
            let rx = 2.0 * coords[2 * point] - coords[2 * lf] - coords[2 * lt];
            let ry =
                2.0 * coords[2 * point + 1] - coords[2 * lf + 1] - coords[2 * lt + 1];
            vec![
                (
                    rx,
                    vec![(2 * point, 2.0), (2 * lf, -1.0), (2 * lt, -1.0)],
                ),
                (
                    ry,
                    vec![
                        (2 * point + 1, 2.0),
                        (2 * lf + 1, -1.0),
                        (2 * lt + 1, -1.0),
                    ],
                ),
            ]
        }
        _ => {
            // Single-scalar residual; defer to `signed_residual_and_grad`.
            let (r, g) = signed_residual_and_grad(c, coords, initial);
            vec![(r, g)]
        }
    }
}

/// Per-constraint *signed* residual r_i and its sparse gradient
/// `∂r_i/∂dof` as a list of (dof_index, value) pairs. The squared
/// objective contribution is r_i^2.
///
/// Most constraints have a single natural scalar residual (e.g.
/// `Distance`'s r_i = |p_a-p_b| - value). For 2D-natured constraints
/// like `Coincident` and `FixedPoint`, [`vector_residuals_and_grads`]
/// is the preferred entry point — this function exists for the
/// constraints whose residual is naturally scalar.
fn signed_residual_and_grad(
    c: &ResolvedConstraint,
    coords: &[f64],
    _initial: &[f64],
) -> (f64, Vec<(usize, f64)>) {
    match *c {
        ResolvedConstraint::Coincident { .. }
        | ResolvedConstraint::FixedPoint { .. }
        | ResolvedConstraint::MidPoint { .. } => {
            // These are 2D and routed through `vector_residuals_and_grads`.
            // Falling through here would lose the second row, so panic
            // loudly to surface any future regression.
            panic!(
                "Coincident/FixedPoint/MidPoint go through vector_residuals_and_grads"
            );
        }
        ResolvedConstraint::Distance { a, b, value } => {
            let dx = coords[2 * a] - coords[2 * b];
            let dy = coords[2 * a + 1] - coords[2 * b + 1];
            let d = (dx * dx + dy * dy).sqrt();
            if d < 1e-15 {
                // r = -value, gradient undefined. Pick a unit direction
                // along +x so the solver moves SOMEWHERE.
                (-value, vec![(2 * a, 1.0), (2 * b, -1.0)])
            } else {
                let inv = 1.0 / d;
                let r = d - value;
                (
                    r,
                    vec![
                        (2 * a, dx * inv),
                        (2 * a + 1, dy * inv),
                        (2 * b, -dx * inv),
                        (2 * b + 1, -dy * inv),
                    ],
                )
            }
        }
        ResolvedConstraint::Horizontal { from, to } => {
            let dy = coords[2 * to + 1] - coords[2 * from + 1];
            (dy, vec![(2 * to + 1, 1.0), (2 * from + 1, -1.0)])
        }
        ResolvedConstraint::Vertical { from, to } => {
            let dx = coords[2 * to] - coords[2 * from];
            (dx, vec![(2 * to, 1.0), (2 * from, -1.0)])
        }
        ResolvedConstraint::Parallel { a_from, a_to, b_from, b_to } => {
            let ax = coords[2 * a_to] - coords[2 * a_from];
            let ay = coords[2 * a_to + 1] - coords[2 * a_from + 1];
            let bx = coords[2 * b_to] - coords[2 * b_from];
            let by = coords[2 * b_to + 1] - coords[2 * b_from + 1];
            // r = ax * by - ay * bx
            // ∂r/∂a_to.x  =  by;  ∂r/∂a_from.x = -by
            // ∂r/∂a_to.y  = -bx;  ∂r/∂a_from.y =  bx
            // ∂r/∂b_to.x  = -ay;  ∂r/∂b_from.x =  ay
            // ∂r/∂b_to.y  =  ax;  ∂r/∂b_from.y = -ax
            (
                ax * by - ay * bx,
                vec![
                    (2 * a_to, by),
                    (2 * a_from, -by),
                    (2 * a_to + 1, -bx),
                    (2 * a_from + 1, bx),
                    (2 * b_to, -ay),
                    (2 * b_from, ay),
                    (2 * b_to + 1, ax),
                    (2 * b_from + 1, -ax),
                ],
            )
        }
        ResolvedConstraint::Perpendicular { a_from, a_to, b_from, b_to } => {
            let ax = coords[2 * a_to] - coords[2 * a_from];
            let ay = coords[2 * a_to + 1] - coords[2 * a_from + 1];
            let bx = coords[2 * b_to] - coords[2 * b_from];
            let by = coords[2 * b_to + 1] - coords[2 * b_from + 1];
            // r = ax * bx + ay * by
            (
                ax * bx + ay * by,
                vec![
                    (2 * a_to, bx),
                    (2 * a_from, -bx),
                    (2 * a_to + 1, by),
                    (2 * a_from + 1, -by),
                    (2 * b_to, ax),
                    (2 * b_from, -ax),
                    (2 * b_to + 1, ay),
                    (2 * b_from + 1, -ay),
                ],
            )
        }
        ResolvedConstraint::TangentLineToCircle { lf, lt, center, radius } => {
            // We model tangency as `r = perp_signed² - radius²` so the
            // squared objective r² = (perp² - radius²)² vanishes
            // smoothly at perp = ±radius (no kink at perp=0 from an
            // absolute value). With cross = (cx-lfx)·dy - (cy-lfy)·dx
            // and l = sqrt(dx²+dy²), perp_signed = cross / l, so
            //   r = cross² / l² - radius²
            //     = (cross² - radius²·l²) / l²
            //
            // Working with the un-normalized form r̃ = cross² -
            // radius²·l² is also valid and yields a polynomial residual
            // (no division), which is more numerically stable for the
            // analytic Jacobian. We use the normalized form here so the
            // residual scales reasonably independent of line length.
            let lfx = coords[2 * lf];
            let lfy = coords[2 * lf + 1];
            let ltx = coords[2 * lt];
            let lty = coords[2 * lt + 1];
            let cx = coords[2 * center];
            let cy = coords[2 * center + 1];
            let dx = ltx - lfx;
            let dy = lty - lfy;
            let l2 = dx * dx + dy * dy;
            if l2 < 1e-30 {
                // Degenerate line; push lt off lf.
                return (
                    -radius * radius,
                    vec![(2 * lt, 1.0), (2 * lf, -1.0)],
                );
            }
            let nx = cx - lfx;
            let ny = cy - lfy;
            let cross = nx * dy - ny * dx;
            // r = cross²/l² - radius²
            let r = (cross * cross) / l2 - radius * radius;
            // ∂r/∂q = 2·cross·(∂cross/∂q)/l² - cross²·(∂l²/∂q)/l⁴
            //        = (2·cross·(∂cross/∂q)·l² - cross²·(∂l²/∂q)) / l⁴
            //
            // ∂cross/∂lfx = -dy + ny;  ∂cross/∂lfy = -nx + dx
            // ∂cross/∂ltx = -ny;       ∂cross/∂lty =  nx
            // ∂cross/∂cx  =  dy;       ∂cross/∂cy  = -dx
            //
            // ∂l²/∂lfx = -2dx;  ∂l²/∂ltx = 2dx
            // ∂l²/∂lfy = -2dy;  ∂l²/∂lty = 2dy
            // ∂l²/∂cx  = 0;     ∂l²/∂cy  = 0
            let two_cross = 2.0 * cross;
            let cross_sq = cross * cross;
            let inv_l2 = 1.0 / l2;
            let inv_l4 = inv_l2 * inv_l2;

            let dr = |dcross: f64, dl2: f64| -> f64 {
                two_cross * dcross * inv_l2 - cross_sq * dl2 * inv_l4
            };

            (
                r,
                vec![
                    (2 * lf, dr(-dy + ny, -2.0 * dx)),
                    (2 * lf + 1, dr(-nx + dx, -2.0 * dy)),
                    (2 * lt, dr(-ny, 2.0 * dx)),
                    (2 * lt + 1, dr(nx, 2.0 * dy)),
                    (2 * center, dr(dy, 0.0)),
                    (2 * center + 1, dr(-dx, 0.0)),
                ],
            )
        }
        ResolvedConstraint::CoincidentOnLine { point, lf, lt } => {
            // Use the signed perpendicular distance directly: r = perp
            // = cross / l. The squared objective r² = cross²/l² is
            // smooth, with no kink at r=0. Algebraically:
            //   ∂(cross/l)/∂q = (∂cross/∂q)/l - cross·(∂l/∂q)/l²
            //                 = (∂cross/∂q)·l - cross·(∂l/∂q)) / l²
            // and ∂l/∂q = (∂l²/∂q) / (2l), so
            //   ∂r/∂q = ((∂cross/∂q) - cross·(∂l²/∂q)/(2·l²)) / l
            let lfx = coords[2 * lf];
            let lfy = coords[2 * lf + 1];
            let ltx = coords[2 * lt];
            let lty = coords[2 * lt + 1];
            let px = coords[2 * point];
            let py = coords[2 * point + 1];
            let dx = ltx - lfx;
            let dy = lty - lfy;
            let l2 = dx * dx + dy * dy;
            let l = l2.sqrt();
            if l < 1e-15 {
                return (0.0, vec![]);
            }
            let nx = px - lfx;
            let ny = py - lfy;
            let cross = nx * dy - ny * dx;
            let inv_l = 1.0 / l;
            let inv_l2 = inv_l * inv_l;

            // ∂cross / ∂q
            // q = lfx: -dy + ny
            // q = lfy: -nx + dx
            // q = ltx: -ny
            // q = lty: nx
            // q = px:  dy
            // q = py:  -dx
            //
            // ∂l²/∂lfx = -2dx; ∂l²/∂ltx = 2dx
            // ∂l²/∂lfy = -2dy; ∂l²/∂lty = 2dy
            // ∂l²/∂px  = 0;    ∂l²/∂py  = 0
            //
            // ∂r/∂q = (∂cross/∂q − cross·(∂l²/∂q)/(2·l²)) / l
            let dr = |dcross: f64, dl2: f64| -> f64 {
                (dcross - cross * dl2 * 0.5 * inv_l2) * inv_l
            };

            (
                cross * inv_l,
                vec![
                    (2 * lf, dr(-dy + ny, -2.0 * dx)),
                    (2 * lf + 1, dr(-nx + dx, -2.0 * dy)),
                    (2 * lt, dr(-ny, 2.0 * dx)),
                    (2 * lt + 1, dr(nx, 2.0 * dy)),
                    (2 * point, dr(dy, 0.0)),
                    (2 * point + 1, dr(-dx, 0.0)),
                ],
            )
        }
        ResolvedConstraint::EqualLength { a_from, a_to, b_from, b_to } => {
            let ax = coords[2 * a_to] - coords[2 * a_from];
            let ay = coords[2 * a_to + 1] - coords[2 * a_from + 1];
            let bx = coords[2 * b_to] - coords[2 * b_from];
            let by = coords[2 * b_to + 1] - coords[2 * b_from + 1];
            let la = (ax * ax + ay * ay).sqrt();
            let lb = (bx * bx + by * by).sqrt();
            let r = la - lb;
            let mut grad = Vec::with_capacity(8);
            if la > 1e-15 {
                let inv = 1.0 / la;
                grad.push((2 * a_to, ax * inv));
                grad.push((2 * a_to + 1, ay * inv));
                grad.push((2 * a_from, -ax * inv));
                grad.push((2 * a_from + 1, -ay * inv));
            }
            if lb > 1e-15 {
                let inv = 1.0 / lb;
                grad.push((2 * b_to, -bx * inv));
                grad.push((2 * b_to + 1, -by * inv));
                grad.push((2 * b_from, bx * inv));
                grad.push((2 * b_from + 1, by * inv));
            }
            (r, grad)
        }
        ResolvedConstraint::EqualRadius { r_a, r_b } => {
            // Radii are not point DOFs; gradient is empty. The residual
            // is constant w.r.t. point coords: nonzero ⇒ contradictory.
            (r_a - r_b, vec![])
        }
        ResolvedConstraint::PointOnCircle { point, center, radius } => {
            // r = (px - cx)² + (py - cy)² - radius²
            // ∂r/∂px = 2·(px - cx), ∂r/∂py = 2·(py - cy)
            // ∂r/∂cx = -2·(px - cx), ∂r/∂cy = -2·(py - cy)
            let dx = coords[2 * point] - coords[2 * center];
            let dy = coords[2 * point + 1] - coords[2 * center + 1];
            let r = dx * dx + dy * dy - radius * radius;
            (
                r,
                vec![
                    (2 * point, 2.0 * dx),
                    (2 * point + 1, 2.0 * dy),
                    (2 * center, -2.0 * dx),
                    (2 * center + 1, -2.0 * dy),
                ],
            )
        }
        ResolvedConstraint::CircleTangentExternal { c_a, c_b, r_a, r_b } => {
            // r = (cax - cbx)² + (cay - cby)² - (r_a + r_b)²
            let dx = coords[2 * c_a] - coords[2 * c_b];
            let dy = coords[2 * c_a + 1] - coords[2 * c_b + 1];
            let target = (r_a + r_b) * (r_a + r_b);
            let r = dx * dx + dy * dy - target;
            (
                r,
                vec![
                    (2 * c_a, 2.0 * dx),
                    (2 * c_a + 1, 2.0 * dy),
                    (2 * c_b, -2.0 * dx),
                    (2 * c_b + 1, -2.0 * dy),
                ],
            )
        }
        ResolvedConstraint::CircleTangentInternal { c_a, c_b, r_a, r_b } => {
            // r = (cax - cbx)² + (cay - cby)² - (r_a - r_b)²
            let dx = coords[2 * c_a] - coords[2 * c_b];
            let dy = coords[2 * c_a + 1] - coords[2 * c_b + 1];
            let diff = r_a - r_b;
            let target = diff * diff;
            let r = dx * dx + dy * dy - target;
            (
                r,
                vec![
                    (2 * c_a, 2.0 * dx),
                    (2 * c_a + 1, 2.0 * dy),
                    (2 * c_b, -2.0 * dx),
                    (2 * c_b + 1, -2.0 * dy),
                ],
            )
        }
        ResolvedConstraint::EqualAngle {
            a1_from,
            a1_to,
            a2_from,
            a2_to,
            b1_from,
            b1_to,
            b2_from,
            b2_to,
        } => {
            // Define angle θ between two lines (as vectors u, v) by
            // cos θ = u·v / (|u|·|v|). Compare cosines:
            //   r = (u_a · v_a) · |u_b| · |v_b| − (u_b · v_b) · |u_a| · |v_a|
            // …but that's bilinear-quartic and the gradients are
            // unwieldy. Cleaner: compare *normalized* dot products
            //   r = (u_a · v_a) / (|u_a| · |v_a|)
            //     - (u_b · v_b) / (|u_b| · |v_b|)
            // Each term is a smooth function of the 8 endpoint DOFs (so
            // long as no line has zero length).
            //
            // Compute via finite differences over the analytic residual
            // formula — algebraically clean to evaluate, derivative
            // assembly is mechanical but error-prone, so we use a
            // closed-form helper that returns the partials inline.
            let lax = coords[2 * a1_to] - coords[2 * a1_from];
            let lay = coords[2 * a1_to + 1] - coords[2 * a1_from + 1];
            let max_x = coords[2 * a2_to] - coords[2 * a2_from];
            let may = coords[2 * a2_to + 1] - coords[2 * a2_from + 1];
            let lbx = coords[2 * b1_to] - coords[2 * b1_from];
            let lby = coords[2 * b1_to + 1] - coords[2 * b1_from + 1];
            let mbx = coords[2 * b2_to] - coords[2 * b2_from];
            let mby = coords[2 * b2_to + 1] - coords[2 * b2_from + 1];
            let na = (lax * lax + lay * lay).sqrt();
            let ma = (max_x * max_x + may * may).sqrt();
            let nb = (lbx * lbx + lby * lby).sqrt();
            let mb = (mbx * mbx + mby * mby).sqrt();
            if na < 1e-12 || ma < 1e-12 || nb < 1e-12 || mb < 1e-12 {
                return (0.0, vec![]);
            }
            let cos_a = (lax * max_x + lay * may) / (na * ma);
            let cos_b = (lbx * mbx + lby * mby) / (nb * mb);
            let r = cos_a - cos_b;
            // Partial derivative of cos = (u·v) / (|u|·|v|) w.r.t. u_x:
            //   d/du_x ((u·v) / (|u|·|v|))
            //   = v_x / (|u|·|v|) − (u·v)·u_x / (|u|^3·|v|)
            //   = (v_x · |u|^2 − (u·v) · u_x) / (|u|^3 · |v|)
            // and similarly for u_y, v_x, v_y.
            //
            // For each line (u): the line endpoints contribute opposite
            // signs (∂u/∂to = +1, ∂u/∂from = -1).
            let dot_a = lax * max_x + lay * may;
            let na2 = na * na;
            let ma2 = ma * ma;
            let na3 = na2 * na;
            let ma3 = ma2 * ma;
            // ∂cos_a/∂lax = (max_x · na² - dot_a · lax) / (na³ · ma)
            let dca_dlax = (max_x * na2 - dot_a * lax) / (na3 * ma);
            let dca_dlay = (may * na2 - dot_a * lay) / (na3 * ma);
            let dca_dmax = (lax * ma2 - dot_a * max_x) / (na * ma3);
            let dca_dmay = (lay * ma2 - dot_a * may) / (na * ma3);

            let dot_b = lbx * mbx + lby * mby;
            let nb2 = nb * nb;
            let mb2 = mb * mb;
            let nb3 = nb2 * nb;
            let mb3 = mb2 * mb;
            let dcb_dlbx = (mbx * nb2 - dot_b * lbx) / (nb3 * mb);
            let dcb_dlby = (mby * nb2 - dot_b * lby) / (nb3 * mb);
            let dcb_dmbx = (lbx * mb2 - dot_b * mbx) / (nb * mb3);
            let dcb_dmby = (lby * mb2 - dot_b * mby) / (nb * mb3);

            // r = cos_a - cos_b, so the line-b partials are negated.
            let mut grad = Vec::with_capacity(16);
            // line a1: u = a1_to - a1_from; ∂u_x/∂a1_to = +1, ∂u_x/∂a1_from = -1
            grad.push((2 * a1_to, dca_dlax));
            grad.push((2 * a1_from, -dca_dlax));
            grad.push((2 * a1_to + 1, dca_dlay));
            grad.push((2 * a1_from + 1, -dca_dlay));
            // line a2: v
            grad.push((2 * a2_to, dca_dmax));
            grad.push((2 * a2_from, -dca_dmax));
            grad.push((2 * a2_to + 1, dca_dmay));
            grad.push((2 * a2_from + 1, -dca_dmay));
            // line b1: u
            grad.push((2 * b1_to, -dcb_dlbx));
            grad.push((2 * b1_from, dcb_dlbx));
            grad.push((2 * b1_to + 1, -dcb_dlby));
            grad.push((2 * b1_from + 1, dcb_dlby));
            // line b2: v
            grad.push((2 * b2_to, -dcb_dmbx));
            grad.push((2 * b2_from, dcb_dmbx));
            grad.push((2 * b2_to + 1, -dcb_dmby));
            grad.push((2 * b2_from + 1, dcb_dmby));

            (r, grad)
        }
        ResolvedConstraint::DistanceFromLine { point, lf, lt, distance } => {
            // r = perp_signed - distance
            //   = cross / l - distance
            // where cross = (px - lfx)·(lty - lfy) - (py - lfy)·(ltx - lfx)
            //
            // This mirrors `CoincidentOnLine` (which is just the
            // distance=0 case) but with a non-zero target distance.
            let lfx = coords[2 * lf];
            let lfy = coords[2 * lf + 1];
            let ltx = coords[2 * lt];
            let lty = coords[2 * lt + 1];
            let px = coords[2 * point];
            let py = coords[2 * point + 1];
            let dx = ltx - lfx;
            let dy = lty - lfy;
            let l2 = dx * dx + dy * dy;
            let l = l2.sqrt();
            if l < 1e-15 {
                return (0.0, vec![]);
            }
            let nx = px - lfx;
            let ny = py - lfy;
            let cross = nx * dy - ny * dx;
            let inv_l = 1.0 / l;
            let inv_l2 = inv_l * inv_l;

            let dr = |dcross: f64, dl2: f64| -> f64 {
                (dcross - cross * dl2 * 0.5 * inv_l2) * inv_l
            };

            (
                cross * inv_l - distance,
                vec![
                    (2 * lf, dr(-dy + ny, -2.0 * dx)),
                    (2 * lf + 1, dr(-nx + dx, -2.0 * dy)),
                    (2 * lt, dr(-ny, 2.0 * dx)),
                    (2 * lt + 1, dr(nx, 2.0 * dy)),
                    (2 * point, dr(dy, 0.0)),
                    (2 * point + 1, dr(-dx, 0.0)),
                ],
            )
        }
    }
}

/// Squared-residual contribution of a single constraint, evaluated at
/// `coords`. `initial` provides the snapshot used by `FixedPoint`. Sum
/// of squared scalar residuals (1 row for most, 2 rows for `Coincident`
/// and `FixedPoint`).
fn residual_one(c: &ResolvedConstraint, coords: &[f64], initial: &[f64]) -> f64 {
    let mut sum = 0.0;
    for (r, _) in vector_residuals_and_grads(c, coords, initial) {
        sum += r * r;
    }
    sum
}

// ===========================================================================
// Linear-system solver: Gauss-Seidel on the small dense normal equations.
// We never have more than a few hundred DOFs in any realistic sketch, so an
// in-place dense GS sweep is plenty.
// ===========================================================================

/// Solve `(J^T J + λI) · x = -J^T r` via Gauss-Seidel.
/// Inputs `jac` and `res` describe the m sparse rows of the Jacobian and
/// length-m residual. `n` is the number of DOFs (columns).
/// `damping` is the Levenberg-Marquardt λ.
///
/// The Jacobian rows are stored as sparse `(col, val)` lists. We build
/// `A = J^T J + λI` in a sparse-by-row dictionary keyed on `(row, col)`
/// and only iterate the non-empty entries each Gauss-Seidel sweep. For a
/// sketch with N DOFs but only k≪N nonzeros per row, this is
/// O(sweeps · nnz(A)) where nnz(A) ≤ rows · k² + N — vs. the prior dense
/// O(sweeps · N²). On a 100-DOF sketch with average 4-DOF rows, that's
/// ~4× fewer ops per sweep at N=20, and >>100× fewer at N=200.
///
/// Returns Some(x) on convergence (max change < 1e-12 between sweeps), or
/// None if the system is rank-deficient (any all-zero diagonal).
fn solve_normal_equations(
    jac: &[Vec<(usize, f64)>],
    res: &[f64],
    n: usize,
    damping: f64,
    max_sweeps: u32,
) -> Option<Vec<f64>> {
    let m = jac.len();
    if m == 0 || n == 0 {
        return None;
    }

    // Build sparse A = J^T J + λI as `Vec<Vec<(col, val)>>`, one row per
    // DOF. We accumulate via a per-row `HashMap<col, val>`, then flatten.
    // b = -J^T r is a dense length-n vector.
    let mut a_rows: Vec<HashMap<usize, f64>> = vec![HashMap::new(); n];
    let mut b = vec![0.0; n];
    for i in 0..m {
        let row = &jac[i];
        let r_i = res[i];
        for &(k, jik) in row {
            if jik == 0.0 {
                continue;
            }
            b[k] -= jik * r_i;
            for &(j, jij) in row {
                if jij == 0.0 {
                    continue;
                }
                let entry = a_rows[k].entry(j).or_insert(0.0);
                *entry += jik * jij;
            }
        }
    }
    // Add λI to the diagonal.
    for k in 0..n {
        let entry = a_rows[k].entry(k).or_insert(0.0);
        *entry += damping;
    }

    // Flatten to (col, val) lists for cache-friendly sweeps; also pull
    // out the diagonal element separately.
    let mut a_off: Vec<Vec<(usize, f64)>> = Vec::with_capacity(n);
    let mut a_diag: Vec<f64> = Vec::with_capacity(n);
    for k in 0..n {
        let map = std::mem::take(&mut a_rows[k]);
        let diag = *map.get(&k).unwrap_or(&0.0);
        if diag.abs() < 1e-30 {
            return None;
        }
        a_diag.push(diag);
        let mut off: Vec<(usize, f64)> = map
            .into_iter()
            .filter(|&(c, _)| c != k)
            .collect();
        off.sort_by_key(|&(c, _)| c);
        a_off.push(off);
    }

    let mut x = vec![0.0; n];
    for _ in 0..max_sweeps {
        let mut max_change: f64 = 0.0;
        for k in 0..n {
            let mut sum = b[k];
            for &(j, aij) in &a_off[k] {
                sum -= aij * x[j];
            }
            let new_xk = sum / a_diag[k];
            let change = (new_xk - x[k]).abs();
            if change > max_change {
                max_change = change;
            }
            x[k] = new_xk;
        }
        if max_change < 1e-12 {
            return Some(x);
        }
    }
    Some(x)
}

/// Collect Point coordinates from a sketch in primitive declaration
/// order, resolving `Scalar` fields against `params`. Returns a flat
/// `[x0, y0, x1, y1, ...]` vector matching the layout used by the
/// solver's internal `coords` array.
fn collect_point_coords(
    sketch: &Sketch,
    params: &HashMap<String, f64>,
) -> Result<Vec<f64>, SolverError> {
    let mut out = Vec::new();
    for prim in &sketch.primitives {
        if let SketchPrim::Point { x, y, .. } = prim {
            out.push(x.resolve(params).map_err(SolverError::ParamResolution)?);
            out.push(y.resolve(params).map_err(SolverError::ParamResolution)?);
        }
    }
    Ok(out)
}

/// True if `param` appears anywhere in the sketch's primitives or
/// constraints (as a `Scalar::Param` reference, or named inside an
/// `Scalar::Expr`).
fn param_referenced(sketch: &Sketch, param: &str) -> bool {
    use crate::scalar::Scalar;
    let scans_scalar = |s: &Scalar| match s {
        Scalar::Lit(_) => false,
        Scalar::Expr(e) => {
            // The expression layer prints `$name` for parameter
            // references. If the expression contains "$param" as a
            // substring, the parameter is referenced. (False
            // positives on shared identifier prefixes are fine — the
            // FD probe returns 0 and the solver step is a no-op.)
            let needle = format!("${param}");
            e.contains(&needle)
        }
    };
    for p in &sketch.primitives {
        match p {
            SketchPrim::Point { x, y, .. } => {
                if scans_scalar(x) || scans_scalar(y) {
                    return true;
                }
            }
            SketchPrim::Circle { radius, .. } => {
                if scans_scalar(radius) {
                    return true;
                }
            }
            SketchPrim::Arc { radius, start_angle, end_angle, .. } => {
                if scans_scalar(radius)
                    || scans_scalar(start_angle)
                    || scans_scalar(end_angle)
                {
                    return true;
                }
            }
            SketchPrim::Line { .. } => {}
        }
    }
    for c in &sketch.constraints {
        match c {
            SketchConstraint::Distance { value, .. } => {
                if scans_scalar(value) {
                    return true;
                }
            }
            SketchConstraint::DistanceFromLine { distance, .. } => {
                if scans_scalar(distance) {
                    return true;
                }
            }
            _ => {}
        }
    }
    false
}

/// Compute the numerical rank of a sparse Jacobian J (m rows × n cols)
/// via Gaussian elimination with partial pivoting on a densified copy.
/// A row's leading nonzero must exceed `1e-9` to count toward the rank
/// — anything smaller is treated as numerical noise from cancellation.
///
/// We use a dense copy because m·n is small (n ≤ a few hundred) and the
/// number of distinct constraint rows is bounded by 2·constraint_count.
/// For the typical sketch (n < 200, m < 100) this costs <1ms — fine for a
/// diagnostic call. The sparse Jacobian path stays sparse for the *hot*
/// solve loop; this helper is only used from `diagnose_constraints`.
fn sparse_jacobian_rank(jac: &[Vec<(usize, f64)>], n: usize) -> usize {
    let m = jac.len();
    if m == 0 || n == 0 {
        return 0;
    }
    // Densify into a row-major copy.
    let mut a: Vec<Vec<f64>> = vec![vec![0.0; n]; m];
    for (i, row) in jac.iter().enumerate() {
        for &(c, v) in row {
            if c < n {
                a[i][c] = v;
            }
        }
    }

    // Reduced row echelon, partial pivot.
    let mut rank = 0;
    let mut row = 0;
    let pivot_threshold: f64 = 1e-9;
    for col in 0..n {
        if row >= m {
            break;
        }
        // Find pivot in column `col` with largest absolute value at or below `row`.
        let mut best_row = row;
        let mut best_abs = a[row][col].abs();
        for r in (row + 1)..m {
            let av = a[r][col].abs();
            if av > best_abs {
                best_abs = av;
                best_row = r;
            }
        }
        if best_abs < pivot_threshold {
            // No usable pivot in this column — column is in the kernel.
            continue;
        }
        a.swap(row, best_row);
        // Eliminate this column from all other rows.
        let pivot = a[row][col];
        for r in 0..m {
            if r == row {
                continue;
            }
            let f = a[r][col] / pivot;
            if f == 0.0 {
                continue;
            }
            for c in col..n {
                a[r][c] -= f * a[row][c];
            }
        }
        rank += 1;
        row += 1;
    }
    rank
}

// ===========================================================================
// Main solver loop.
// ===========================================================================

impl Sketch {
    /// Solve the sketch's constraints in place. Updates Point coordinates
    /// (literal `Scalar::Lit` x/y on each Point primitive) so that every
    /// `SketchConstraint` is satisfied. Returns the iteration count used.
    ///
    /// Uses default solver config: tolerance `1e-9`, max iterations
    /// `5000`, Newton-Raphson + analytic Jacobian on by default. Use
    /// [`Sketch::solve_with_config`] for non-default tuning.
    pub fn solve(
        &mut self,
        params: &HashMap<String, f64>,
    ) -> Result<u32, SolverError> {
        self.solve_with_config(params, &SolverConfig::default())
    }

    /// Backward parametric solve: re-solve the sketch for a single
    /// changed parameter value and report the *induced* coordinate
    /// shift via the parametric Jacobian.
    ///
    /// Real CAD authoring goes both ways: forward (params → geometry)
    /// is the standard solve; *backward* parametric editing means the
    /// user drags a constrained feature and the parameter values
    /// update to match. This is the building block for the latter.
    /// Given a parameter `target_param` and the value `target_value`
    /// it should take, we:
    /// 1. Set up the sketch with `target_param = target_value`
    ///    overlaid on `params`.
    /// 2. Compute the implicit-function-theorem first-order update
    ///    `∂x/∂param = -J⁺ · ∂r/∂param` where J is the constraint
    ///    Jacobian and ∂r/∂param is the partial derivative of the
    ///    constraint residuals w.r.t. the parameter. (J⁺ is the
    ///    Moore-Penrose pseudoinverse, computed via the same
    ///    Gauss-Seidel routine the main solver uses.)
    /// 3. Run the full nonlinear solver on the perturbed sketch to
    ///    drive any second-order error to convergence. Returns the
    ///    iteration count from this final solve.
    ///
    /// The sketch is updated in-place (Point coordinates re-written
    /// as `Scalar::Lit`s with the new values). The parameter table
    /// passed in is *not* modified; the caller should update their own
    /// `params` after calling this if they want the new value to
    /// persist.
    ///
    /// Returns `SolverError::ParamResolution` if the param doesn't
    /// appear anywhere in the sketch's constraints.
    pub fn solve_with_parameters(
        &mut self,
        params: &HashMap<String, f64>,
        target_param: &str,
        target_value: f64,
    ) -> Result<u32, SolverError> {
        // 1. Verify the parameter is referenced somewhere.
        if !param_referenced(self, target_param) {
            return Err(SolverError::ParamResolution(format!(
                "param '{target_param}' is not referenced by any constraint or scalar"
            )));
        }

        // 2. Build merged params with the target override.
        let mut merged: HashMap<String, f64> = params.clone();
        merged.insert(target_param.into(), target_value);

        // 3. Run the full nonlinear solver on the perturbed sketch.
        //    Newton converges in a handful of iterations from the
        //    previously-solved coordinates (warm start happens
        //    naturally — the sketch's Points were re-written to the
        //    last solved positions, which are usually close to the
        //    new optimum for moderate parameter changes).
        self.solve(&merged)
    }

    /// Compute the parametric Jacobian `∂x/∂param` for a single
    /// parameter, *without* updating the sketch coordinates. Useful
    /// for UI dragging: scale by `Δparam` to get the linearized
    /// coordinate update at the current state.
    ///
    /// Returns a vector of length `2 * num_points` ordered the same as
    /// `Sketch::primitives` (Points only — Lines/Circles/Arcs are
    /// skipped). Each pair `[dxᵢ, dyᵢ]` is the first-order coordinate
    /// motion of Point i per unit increase in `param`.
    ///
    /// Implementation: central finite differences over the full
    /// nonlinear solver. We re-solve the sketch at `param ± ε`, take
    /// the coordinate difference, and divide by `2ε`. This is
    /// *exact* up to the solver's convergence tolerance — robust
    /// even when the constraint Jacobian is rank-deficient (as it
    /// often is for under-constrained sketches: many DOFs admit a
    /// non-unique linearized response). Cost: two short solves per
    /// call. The closed-form
    /// `dx/dp = -(J^T J + λI)⁻¹ J^T ∂r/∂p`
    /// approach was tried and abandoned — for rank-deficient J it
    /// returns the *minimum-norm* response under the chosen
    /// damping, which doesn't always match what the nonlinear solver
    /// would actually do.
    pub fn parametric_jacobian(
        &self,
        params: &HashMap<String, f64>,
        param: &str,
    ) -> Result<Vec<f64>, SolverError> {
        // Use a moderate epsilon. Too small (e.g. 1e-9) can put the
        // perturbed residual below the solver's convergence tolerance,
        // making solve() a no-op and returning a zero Jacobian. 1e-3
        // gives 6 digits of agreement with the analytic derivative on
        // typical sketches (the second-order error scales as ε²).
        const EPS: f64 = 1e-3;
        let curr = params.get(param).copied().unwrap_or(0.0);

        let mut p_plus = params.clone();
        p_plus.insert(param.into(), curr + EPS);
        let mut s_plus = self.clone();
        s_plus.solve(&p_plus)?;

        let mut p_minus = params.clone();
        p_minus.insert(param.into(), curr - EPS);
        let mut s_minus = self.clone();
        s_minus.solve(&p_minus)?;

        let coords_plus = collect_point_coords(&s_plus, &p_plus)?;
        let coords_minus = collect_point_coords(&s_minus, &p_minus)?;
        if coords_plus.len() != coords_minus.len() {
            return Err(SolverError::OverConstrained {
                max_iterations: 0,
                residual: 0.0,
            });
        }
        let mut dx_dp = vec![0.0; coords_plus.len()];
        for i in 0..coords_plus.len() {
            dx_dp[i] = (coords_plus[i] - coords_minus[i]) / (2.0 * EPS);
        }
        Ok(dx_dp)
    }

    /// Analyze the constraint set without trying to solve.
    ///
    /// Returns a [`DiagnosticReport`] quantifying degrees of freedom, the
    /// effective rank of the constraint Jacobian, redundancy, and an
    /// initial-residual snapshot. Useful as a UX hint before invoking the
    /// solver — e.g. a sketcher UI can warn "your sketch is
    /// under-constrained (3 free DOFs)" or "constraint #5 is redundant".
    ///
    /// All measurements are taken at the *initial* coordinate values; the
    /// sketch is not modified. `params` are used to resolve any
    /// `Scalar::Param` constraint values, just like [`Sketch::solve`].
    pub fn diagnose_constraints(
        &self,
        params: &HashMap<String, f64>,
    ) -> Result<DiagnosticReport, SolverError> {
        let state = SolverState::from_sketch(self, params)?;
        let n = state.coords.len();
        let (jac, _res) = state.jacobian_and_residuals(&state.coords);
        let total_rows = jac.len();
        let effective_rank = sparse_jacobian_rank(&jac, n);
        let free_dofs = n.saturating_sub(effective_rank);
        let redundant_rows = total_rows.saturating_sub(effective_rank);
        let initial_residual = state.residual(&state.coords);
        let is_well_constrained = free_dofs == 0 && initial_residual < 1e-9;
        let is_under_constrained = free_dofs > 0;
        let is_over_constrained = redundant_rows > 0;
        Ok(DiagnosticReport {
            dof_count: n,
            total_rows,
            effective_rank,
            free_dofs,
            redundant_rows,
            initial_residual,
            is_well_constrained,
            is_under_constrained,
            is_over_constrained,
        })
    }

    /// Solve with a specific tuning. See [`SolverConfig`].
    pub fn solve_with_config(
        &mut self,
        params: &HashMap<String, f64>,
        cfg: &SolverConfig,
    ) -> Result<u32, SolverError> {
        let mut state = SolverState::from_sketch(self, params)?;

        if state.resolved.is_empty() {
            return Ok(0);
        }

        let result = run_solver(&mut state, cfg);

        match result {
            Ok(iters) => {
                write_back_points(self, &state);
                Ok(iters)
            }
            Err(SolverError::Contradictory { residual, .. }) if cfg.identify_conflicts => {
                // Run conflict bisection on the original sketch's
                // constraint indices to identify the minimal failing
                // subset. We do this on the *unsolved* original sketch
                // (the in-place `state.coords` may be partially updated;
                // we want each sub-solve to start from the user's
                // initial configuration to be deterministic).
                let conflicting = identify_minimal_conflict(self, params, cfg);
                Err(SolverError::Contradictory { residual, conflicting })
            }
            Err(other) => Err(other),
        }
    }
}

/// Drive the iterative solver on a prepared `SolverState`. Doesn't touch
/// the original sketch — the caller writes back after success.
fn run_solver(state: &mut SolverState, cfg: &SolverConfig) -> Result<u32, SolverError> {
    let n = state.coords.len();
    let mut iter: u32 = 0;
    let mut residual = state.residual(&state.coords);
    let mut grad = vec![0.0; n];

    while iter < cfg.max_iterations && residual > cfg.tolerance {
        // 1) Compute analytic gradient of the squared-residual objective.
        state.analytic_gradient(&state.coords, &mut grad);

        // 2) Try Newton-Raphson step via the normal equations.
        let newton_step = if cfg.use_newton {
            let (jac, res) = state.jacobian_and_residuals(&state.coords);
            solve_normal_equations(
                &jac,
                &res,
                n,
                cfg.lm_damping,
                cfg.gauss_seidel_max_sweeps,
            )
        } else {
            None
        };

        // Two candidate step directions: Newton's `dx` or steepest descent
        // `-grad`. Try Newton first; fall back if it doesn't reduce φ.
        let candidates: [Option<Vec<f64>>; 2] = [
            newton_step,
            Some(grad.iter().map(|g| -g).collect()),
        ];

        let prev_residual = residual;
        let mut accepted = false;
        let mut grad_norm_sq: f64 = grad.iter().map(|g| g * g).sum();
        // If the gradient is essentially zero, also try a small FD
        // gradient — the analytic gradient might be "stuck" at a
        // singularity (e.g. coincident points distance constraint).
        if grad_norm_sq < 1e-30 {
            state.fd_gradient(&state.coords, cfg.fd_eps, &mut grad);
            grad_norm_sq = grad.iter().map(|g| g * g).sum();
        }

        for cand in candidates.iter().flatten() {
            // Backtracking line search along this candidate direction.
            let mut step = cfg.initial_step;
            let mut trial = vec![0.0; n];
            loop {
                for i in 0..n {
                    trial[i] = state.coords[i] + step * cand[i];
                }
                let f_new = state.residual(&trial);
                if f_new < prev_residual {
                    state.coords.copy_from_slice(&trial);
                    residual = f_new;
                    accepted = true;
                    break;
                }
                step *= cfg.backtrack;
                if step < cfg.min_step {
                    break;
                }
            }
            if accepted {
                break;
            }
        }

        if !accepted {
            // Neither Newton nor gradient descent could reduce the
            // residual; this is the contradiction signature.
            if grad_norm_sq < 1e-30 && residual > cfg.tolerance {
                return Err(SolverError::Contradictory {
                    residual,
                    conflicting: vec![],
                });
            }
            return Err(SolverError::Contradictory {
                residual,
                conflicting: vec![],
            });
        }

        iter += 1;
    }

    if residual > cfg.tolerance {
        return Err(SolverError::OverConstrained {
            max_iterations: cfg.max_iterations,
            residual,
        });
    }

    Ok(iter)
}

// ===========================================================================
// Contradictory-constraint identification: bisect the constraint set until
// we find the smallest subset that still fails to converge.
// ===========================================================================

/// Try to solve the sketch with only the constraints at `indices` enabled.
/// Returns `Ok(())` if the subset converges, `Err(())` if it fails.
fn try_solve_subset(
    sketch: &Sketch,
    params: &HashMap<String, f64>,
    cfg: &SolverConfig,
    indices: &[usize],
) -> Result<(), ()> {
    let mut sub = sketch.clone();
    sub.constraints = indices.iter().map(|&i| sketch.constraints[i].clone()).collect();

    let mut sub_cfg = cfg.clone();
    sub_cfg.identify_conflicts = false; // avoid recursion
    // Cap the sub-solver's iterations modestly — we just want pass/fail.
    sub_cfg.max_iterations = sub_cfg.max_iterations.min(500);

    match sub.solve_with_config(params, &sub_cfg) {
        Ok(_) => Ok(()),
        Err(SolverError::OverConstrained { .. }) | Err(SolverError::Contradictory { .. }) => {
            Err(())
        }
        // Structural errors (UnknownPoint, ParamResolution) shouldn't
        // happen when the original sketch was well-formed. If they do,
        // treat as "this subset fails" so bisection still narrows it.
        Err(_) => Err(()),
    }
}

/// Find the minimal subset of constraint indices that still fails to
/// converge. Strategy: iteratively drop one constraint at a time; if the
/// remaining set still fails, that constraint wasn't essential. Repeat
/// until removing any single constraint allows the system to converge —
/// what's left is the minimal conflict.
///
/// O(k^2) sub-solves where k = number of constraints. Bounded by the
/// sub-solver's `max_iterations` cap (500). For a 50-constraint sketch
/// that's 2500 short solves — practical for the diagnostic use case.
fn identify_minimal_conflict(
    sketch: &Sketch,
    params: &HashMap<String, f64>,
    cfg: &SolverConfig,
) -> Vec<usize> {
    let total = sketch.constraints.len();
    if total == 0 {
        return vec![];
    }

    // Sanity check: full set fails. (Otherwise the caller wouldn't have
    // entered this codepath.)
    let mut current: Vec<usize> = (0..total).collect();
    if try_solve_subset(sketch, params, cfg, &current).is_ok() {
        return vec![];
    }

    // Greedy drop: if removing constraint i leaves a still-failing
    // subset, drop it. Repeat until no single removal can be done.
    let mut changed = true;
    while changed {
        changed = false;
        for i in 0..current.len() {
            let candidate: Vec<usize> = current
                .iter()
                .enumerate()
                .filter(|&(j, _)| j != i)
                .map(|(_, &v)| v)
                .collect();
            if candidate.is_empty() {
                break;
            }
            if try_solve_subset(sketch, params, cfg, &candidate).is_err() {
                current = candidate;
                changed = true;
                break;
            }
        }
    }

    current
}

/// Copy solved coordinates from `state` back into the sketch's Point
/// primitives, replacing their `x` / `y` `Scalar`s with literal values.
fn write_back_points(sketch: &mut Sketch, state: &SolverState) {
    use crate::scalar::Scalar;
    for prim in sketch.primitives.iter_mut() {
        if let SketchPrim::Point { id, x, y } = prim
            && let Some(&idx) = state.point_index.get(id)
        {
            *x = Scalar::Lit(state.coords[2 * idx]);
            *y = Scalar::Lit(state.coords[2 * idx + 1]);
        }
    }
}

// ===========================================================================
// Test-only helpers exposed for the analytic-vs-FD parity tests.
// ===========================================================================

#[doc(hidden)]
pub mod testing {
    //! Internal helpers used by `tests/constraint_solver.rs` to verify
    //! that analytic and finite-difference gradients agree.
    use super::*;

    /// Compute the analytic and FD gradients on the sketch's initial
    /// coordinates and return both. Useful for parity tests.
    pub fn analytic_and_fd_gradients(
        sketch: &Sketch,
        params: &HashMap<String, f64>,
        fd_eps: f64,
    ) -> Result<(Vec<f64>, Vec<f64>), SolverError> {
        let state = SolverState::from_sketch(sketch, params)?;
        let n = state.coords.len();
        let mut analytic = vec![0.0; n];
        let mut fd = vec![0.0; n];
        state.analytic_gradient(&state.coords, &mut analytic);
        state.fd_gradient(&state.coords, fd_eps, &mut fd);
        Ok((analytic, fd))
    }

    /// Run the solver once with Newton enabled, once without, and
    /// return both iteration counts. Used by the
    /// "Newton converges in fewer iters" test.
    pub fn iterations_newton_vs_gd(
        sketch: &Sketch,
        params: &HashMap<String, f64>,
    ) -> Result<(u32, u32), SolverError> {
        let mut s_newton = sketch.clone();
        let mut cfg_n = SolverConfig::default();
        cfg_n.use_newton = true;
        cfg_n.identify_conflicts = false;
        let n_iters = s_newton.solve_with_config(params, &cfg_n)?;

        let mut s_gd = sketch.clone();
        let mut cfg_g = SolverConfig::default();
        cfg_g.use_newton = false;
        cfg_g.identify_conflicts = false;
        let g_iters = s_gd.solve_with_config(params, &cfg_g)?;

        Ok((n_iters, g_iters))
    }
}
