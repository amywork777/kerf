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
//!
//! The objective is the sum over all constraints. We minimize it with
//! gradient descent + backtracking line search; the gradient is computed
//! by central finite differences (the residual is cheap and the parameter
//! count is small for typical sketches, so analytic gradients aren't
//! worth the complexity here).
//!
//! Termination:
//! - `Ok(iterations)` once the total residual drops below `tol = 1e-9`.
//! - `Err(SolverError::Contradictory)` if line search collapses (residual
//!   stops decreasing) yet the residual is still meaningfully large —
//!   characteristic of mutually-exclusive constraints.
//! - `Err(SolverError::OverConstrained)` if `max_iterations` runs out
//!   while the residual is still decreasing slowly. (The two error cases
//!   look similar in practice; we distinguish by whether step size went
//!   to zero — contradiction vs over-constraint.)
//! - `Err(SolverError::ParamResolution)` if a `Scalar::Expr` constraint
//!   value can't be resolved against `params`.
//! - `Err(SolverError::UnknownPoint)` / `UnknownLine` for ids referenced
//!   by a constraint that aren't defined as primitives in the sketch.
//!
//! Under-constrained sketches are explicitly OK: gradient descent
//! converges to *some* configuration that satisfies all constraints; we
//! don't try to add a regularizer that pulls toward the initial guess
//! because `FixedPoint` exists for exactly that role.

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
    /// the same point pair with conflicting values.
    #[error("contradictory constraints (residual = {residual} is non-zero but no descent direction reduces it)")]
    Contradictory { residual: f64 },

    /// A constraint referenced a Point id that isn't defined.
    #[error("constraint references unknown Point id '{0}'")]
    UnknownPoint(String),

    /// A constraint referenced a Line id that isn't a Line primitive.
    #[error("constraint references unknown Line id '{0}'")]
    UnknownLine(String),

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
    /// Finite-difference epsilon for gradient evaluation. Default `1e-7`.
    pub fd_eps: f64,
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
    /// Lookup: line id → (from_point_idx, to_point_idx). Built during
    /// constraint resolution; retained for inspection/debugging.
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
}

impl SolverState {
    /// Build the solver state from a sketch. Resolves all `Scalar` values
    /// against `params` and validates that every constraint references
    /// known primitives.
    fn from_sketch(
        sketch: &Sketch,
        params: &HashMap<String, f64>,
    ) -> Result<Self, SolverError> {
        // 1) Collect Points in declaration order. (We snapshot resolved
        //    coordinates as the initial guess.)
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

    /// Gradient via central finite differences. `out.len() == coords.len()`.
    fn gradient(&self, coords: &[f64], eps: f64, out: &mut [f64]) {
        debug_assert_eq!(out.len(), coords.len());
        // Reuse a scratch buffer to perturb one DOF at a time.
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
}

/// Residual contribution of a single constraint, evaluated at `coords`.
/// `initial` provides the snapshot used by `FixedPoint`.
fn residual_one(c: &ResolvedConstraint, coords: &[f64], initial: &[f64]) -> f64 {
    match *c {
        ResolvedConstraint::Coincident { a, b } => {
            let dx = coords[2 * a] - coords[2 * b];
            let dy = coords[2 * a + 1] - coords[2 * b + 1];
            dx * dx + dy * dy
        }
        ResolvedConstraint::Distance { a, b, value } => {
            let dx = coords[2 * a] - coords[2 * b];
            let dy = coords[2 * a + 1] - coords[2 * b + 1];
            let d = (dx * dx + dy * dy).sqrt();
            let r = d - value;
            r * r
        }
        ResolvedConstraint::Horizontal { from, to } => {
            let dy = coords[2 * to + 1] - coords[2 * from + 1];
            dy * dy
        }
        ResolvedConstraint::Vertical { from, to } => {
            let dx = coords[2 * to] - coords[2 * from];
            dx * dx
        }
        ResolvedConstraint::Parallel { a_from, a_to, b_from, b_to } => {
            let ax = coords[2 * a_to] - coords[2 * a_from];
            let ay = coords[2 * a_to + 1] - coords[2 * a_from + 1];
            let bx = coords[2 * b_to] - coords[2 * b_from];
            let by = coords[2 * b_to + 1] - coords[2 * b_from + 1];
            let cross = ax * by - ay * bx;
            cross * cross
        }
        ResolvedConstraint::Perpendicular { a_from, a_to, b_from, b_to } => {
            let ax = coords[2 * a_to] - coords[2 * a_from];
            let ay = coords[2 * a_to + 1] - coords[2 * a_from + 1];
            let bx = coords[2 * b_to] - coords[2 * b_from];
            let by = coords[2 * b_to + 1] - coords[2 * b_from + 1];
            let dot = ax * bx + ay * by;
            dot * dot
        }
        ResolvedConstraint::FixedPoint { idx } => {
            let dx = coords[2 * idx] - initial[2 * idx];
            let dy = coords[2 * idx + 1] - initial[2 * idx + 1];
            dx * dx + dy * dy
        }
    }
}

impl Sketch {
    /// Solve the sketch's constraints in place. Updates Point coordinates
    /// (literal `Scalar::Lit` x/y on each Point primitive) so that every
    /// `SketchConstraint` is satisfied. Returns the iteration count used.
    ///
    /// `params` is the model parameter table — used to resolve any
    /// `$param` references inside constraint values (e.g.
    /// `Distance { value: Scalar::param("len") }`) and inside Point
    /// coordinates (the resolved values become the initial guess).
    ///
    /// Uses default solver config: tolerance `1e-9`, max iterations
    /// `5000`. Use [`Sketch::solve_with_config`] for non-default tuning.
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

        // Empty problem: no constraints means we're trivially satisfied.
        if state.resolved.is_empty() {
            return Ok(0);
        }

        let n = state.coords.len();
        let mut grad = vec![0.0; n];
        let mut iter: u32 = 0;
        let mut residual = state.residual(&state.coords);

        while iter < cfg.max_iterations && residual > cfg.tolerance {
            state.gradient(&state.coords, cfg.fd_eps, &mut grad);

            // If the gradient is essentially zero but the residual is
            // not, we're at a local minimum that doesn't satisfy the
            // constraints — i.e. the system is contradictory.
            let grad_norm_sq: f64 = grad.iter().map(|g| g * g).sum();
            if grad_norm_sq < 1e-30 {
                return Err(SolverError::Contradictory { residual });
            }

            // Backtracking line search along -gradient.
            let mut step = cfg.initial_step;
            // Reuse one trial buffer.
            let mut trial = vec![0.0; n];
            let prev_residual = residual;
            loop {
                for i in 0..n {
                    trial[i] = state.coords[i] - step * grad[i];
                }
                let f_new = state.residual(&trial);
                if f_new < prev_residual {
                    state.coords.copy_from_slice(&trial);
                    residual = f_new;
                    break;
                }
                step *= cfg.backtrack;
                if step < cfg.min_step {
                    // No step in -gradient direction reduces the
                    // residual, but the gradient norm is non-zero. This
                    // is the contradiction signature.
                    return Err(SolverError::Contradictory { residual });
                }
            }

            iter += 1;
        }

        if residual > cfg.tolerance {
            return Err(SolverError::OverConstrained {
                max_iterations: cfg.max_iterations,
                residual,
            });
        }

        // Write coords back into the sketch's Point primitives.
        write_back_points(self, &state);
        Ok(iter)
    }
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
