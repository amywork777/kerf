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
    fn jacobian_and_residuals(&self, coords: &[f64]) -> (Vec<Vec<f64>>, Vec<f64>) {
        let n = coords.len();
        let mut jac: Vec<Vec<f64>> = Vec::with_capacity(self.resolved.len() * 2);
        let mut res: Vec<f64> = Vec::with_capacity(self.resolved.len() * 2);
        for c in &self.resolved {
            for (r_i, grad) in vector_residuals_and_grads(c, coords, &self.initial) {
                let mut row = vec![0.0; n];
                for (dof, drdx) in grad {
                    row[dof] = drdx;
                }
                jac.push(row);
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
        ResolvedConstraint::Coincident { .. } | ResolvedConstraint::FixedPoint { .. } => {
            // These are 2D and routed through `vector_residuals_and_grads`.
            // Falling through here would lose the second row, so panic
            // loudly to surface any future regression.
            panic!("Coincident/FixedPoint go through vector_residuals_and_grads");
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
/// Inputs `jac` and `res` describe the m × n Jacobian and length-m residual.
/// `damping` is the Levenberg-Marquardt λ.
/// Returns Some(x) on convergence (max change < 1e-12 between sweeps), or
/// None if convergence didn't happen within `max_sweeps`.
fn solve_normal_equations(
    jac: &[Vec<f64>],
    res: &[f64],
    damping: f64,
    max_sweeps: u32,
) -> Option<Vec<f64>> {
    let m = jac.len();
    if m == 0 {
        return None;
    }
    let n = jac[0].len();
    if n == 0 {
        return None;
    }

    // Build A = J^T J + λI (n × n) and b = -J^T r (length n).
    let mut a = vec![vec![0.0; n]; n];
    let mut b = vec![0.0; n];
    for i in 0..m {
        for k in 0..n {
            let jik = jac[i][k];
            if jik == 0.0 {
                continue;
            }
            b[k] -= jik * res[i];
            for j in 0..n {
                let jij = jac[i][j];
                if jij == 0.0 {
                    continue;
                }
                a[k][j] += jik * jij;
            }
        }
    }
    for k in 0..n {
        a[k][k] += damping;
    }

    // Check for any all-zero diagonal (singular row); bail out.
    for k in 0..n {
        if a[k][k].abs() < 1e-30 {
            return None;
        }
    }

    let mut x = vec![0.0; n];
    for _ in 0..max_sweeps {
        let mut max_change: f64 = 0.0;
        for k in 0..n {
            let mut sum = b[k];
            for j in 0..n {
                if j != k {
                    sum -= a[k][j] * x[j];
                }
            }
            let new_xk = sum / a[k][k];
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
            solve_normal_equations(&jac, &res, cfg.lm_damping, cfg.gauss_seidel_max_sweeps)
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
