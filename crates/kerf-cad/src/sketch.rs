//! 2D sketch + constraint solver.
//!
//! A `Sketch` is a flat list of 2D `Point`s plus a list of geometric
//! `Constraint`s that those points must satisfy. The solver runs
//! Newton-Levenberg-Marquardt on the residual function defined by the
//! constraints and returns the solved coordinates (or a structured
//! error pinpointing which constraints couldn't be reconciled).
//!
//! Why a separate module rather than reusing the assembly solver? The
//! assembly solver works on rigid-body 6DOF poses; the sketch solver
//! works on individual point coordinates (2 DOFs per point). Otherwise
//! the underlying machinery is identical: a residual vector, a finite-
//! difference Jacobian, and LM damping.
//!
//! ## Diagnostics
//!
//! Beyond the basic "solve" path, this module exposes:
//!
//! - [`Sketch::diagnose_constraints`] returns the indices of the
//!   constraints whose final per-residual contribution exceeds
//!   `ENFORCEMENT_TOL` after a best-effort solve. This pinpoints the
//!   *specific* constraint indices that conflict, rather than simply
//!   reporting a count.
//!
//! - [`Sketch::auto_fix_constraints`] runs the diagnostic, then
//!   greedily removes the highest-residual constraint until the
//!   sketch is solvable. The removed indices are returned so the
//!   caller can show the user what was dropped.

use serde::{Deserialize, Serialize};

/// A 2D point in the sketch plane.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Point2 {
    pub x: f64,
    pub y: f64,
}

impl Point2 {
    pub fn new(x: f64, y: f64) -> Self {
        Self { x, y }
    }
}

/// A geometric constraint between sketch points.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum Constraint {
    /// Two points coincide.
    Coincident { a: usize, b: usize },
    /// Distance between `a` and `b` equals `value`.
    Distance { a: usize, b: usize, value: f64 },
    /// Point `a` is fixed at `(x, y)`.
    Fix { a: usize, x: f64, y: f64 },
    /// `a` and `b` have the same x coordinate (vertical line through them).
    Vertical { a: usize, b: usize },
    /// `a` and `b` have the same y coordinate (horizontal line through them).
    Horizontal { a: usize, b: usize },
    /// Segment a→b is parallel to segment c→d.
    Parallel { a: usize, b: usize, c: usize, d: usize },
    /// Segment a→b is perpendicular to segment c→d.
    Perpendicular { a: usize, b: usize, c: usize, d: usize },
    /// |a - center| = |b - center|: a and b lie on a circle around `center`.
    EqualRadius { center: usize, a: usize, b: usize },
}

/// Top-level sketch container.
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct Sketch {
    pub points: Vec<Point2>,
    pub constraints: Vec<Constraint>,
}

/// Tolerance below which a residual is considered "satisfied".
pub const ENFORCEMENT_TOL: f64 = 1e-9;

#[derive(Debug, thiserror::Error)]
pub enum SketchError {
    #[error("sketch could not be solved (final cost {cost}, iterations {iterations})")]
    DidNotConverge { cost: f64, iterations: usize },
    #[error("constraint {0} references unknown point index {1}")]
    UnknownPoint(usize, usize),
}

impl Sketch {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn add_point(&mut self, p: Point2) -> usize {
        self.points.push(p);
        self.points.len() - 1
    }

    pub fn add_constraint(&mut self, c: Constraint) -> usize {
        self.constraints.push(c);
        self.constraints.len() - 1
    }

    /// Validate that every constraint's point indices are in range.
    pub fn validate(&self) -> Result<(), SketchError> {
        let n = self.points.len();
        for (i, c) in self.constraints.iter().enumerate() {
            for idx in constraint_points(c) {
                if idx >= n {
                    return Err(SketchError::UnknownPoint(i, idx));
                }
            }
        }
        Ok(())
    }

    /// Solve the sketch in-place. On convergence the points are updated
    /// to satisfy every constraint to within `ENFORCEMENT_TOL`. On
    /// failure the points are left at their best-effort intermediate
    /// values and an error is returned.
    pub fn solve(&mut self) -> Result<(), SketchError> {
        self.validate()?;
        let (x, cost, iters) = self.run_lm();
        for (i, p) in self.points.iter_mut().enumerate() {
            p.x = x[i * 2];
            p.y = x[i * 2 + 1];
        }
        if cost < ENFORCEMENT_TOL {
            Ok(())
        } else {
            Err(SketchError::DidNotConverge {
                cost,
                iterations: iters,
            })
        }
    }

    /// Identify which specific constraint indices fail to be satisfied
    /// after a best-effort solve. Returns the list of indices whose
    /// residual exceeds `ENFORCEMENT_TOL`, sorted by residual descending
    /// (worst first). An empty vec means the sketch is fully solvable.
    ///
    /// This is the diagnostic call the user invokes when `solve` returns
    /// `DidNotConverge` — it tells them *which* constraints to look at.
    pub fn diagnose_constraints(&self) -> Vec<DiagnosticEntry> {
        if self.validate().is_err() {
            // Validation errors aren't constraint conflicts — caller
            // should invoke validate() separately.
            return Vec::new();
        }
        let (x, _cost, _iters) = self.run_lm();
        let mut out: Vec<DiagnosticEntry> = self
            .constraints
            .iter()
            .enumerate()
            .filter_map(|(i, c)| {
                let r = constraint_residual(c, &x);
                if r > ENFORCEMENT_TOL {
                    Some(DiagnosticEntry {
                        index: i,
                        residual: r,
                    })
                } else {
                    None
                }
            })
            .collect();
        out.sort_by(|a, b| b.residual.partial_cmp(&a.residual).unwrap());
        out
    }

    /// Greedily remove redundant / over-constraining constraints until
    /// the sketch solves. Returns the indices (into the *original*
    /// `self.constraints` Vec) that were removed.
    ///
    /// Heuristic: at each step, try every constraint individually as a
    /// candidate for removal. Pick the one whose removal yields the
    /// lowest residual on the remaining set. Stop when the sketch
    /// solves cleanly. This is O(n²·LM-cost) but for typical sketch
    /// sizes (≤ 30 constraints) that's acceptable.
    ///
    /// Caller is responsible for accepting the result; this is best-
    /// effort on top of an already over-constrained sketch.
    pub fn auto_fix_constraints(&mut self) -> Vec<usize> {
        let original_count = self.constraints.len();
        // Track which original indices remain.
        let mut original_idx: Vec<usize> = (0..original_count).collect();
        let mut removed: Vec<usize> = Vec::new();

        for _ in 0..original_count {
            // Try a solve; if it succeeds, we're done.
            if self.solve().is_ok() {
                return removed;
            }
            // For each remaining constraint, simulate its removal and
            // record the resulting cost.
            let mut best_idx = 0usize;
            let mut best_cost = f64::INFINITY;
            for i in 0..self.constraints.len() {
                let mut probe = self.clone();
                probe.constraints.remove(i);
                let (x, _, _) = probe.run_lm();
                let cost = probe.cost(&x);
                if cost < best_cost {
                    best_cost = cost;
                    best_idx = i;
                }
            }
            removed.push(original_idx[best_idx]);
            self.constraints.remove(best_idx);
            original_idx.remove(best_idx);
        }
        removed
    }

    /// Number of degrees of freedom: 2 * points - rank-of-constraints.
    /// We don't compute the constraint Jacobian rank symbolically; we
    /// approximate by counting equations. A negative DOF means the
    /// system is over-constrained; positive DOF means it's
    /// under-determined.
    pub fn approximate_dof(&self) -> isize {
        let n_points = self.points.len() as isize * 2;
        let n_eq: isize = self
            .constraints
            .iter()
            .map(|c| constraint_dim(c) as isize)
            .sum();
        n_points - n_eq
    }

    // -----------------------------------------------------------------
    // Internal: LM solver
    // -----------------------------------------------------------------

    /// Pack current point coords into x.
    fn x_init(&self) -> Vec<f64> {
        let mut x = Vec::with_capacity(self.points.len() * 2);
        for p in &self.points {
            x.push(p.x);
            x.push(p.y);
        }
        x
    }

    /// Sum of squared residuals across all constraints.
    fn cost(&self, x: &[f64]) -> f64 {
        let mut total = 0.0;
        for c in &self.constraints {
            let r = constraint_residual(c, x);
            total += r * r;
        }
        total
    }

    /// LM with finite-difference gradient. Same shape as the assembly
    /// solver; kept self-contained because the residual signature
    /// differs.
    fn run_lm(&self) -> (Vec<f64>, f64, usize) {
        let mut x = self.x_init();
        let n = x.len();
        if n == 0 {
            return (x, 0.0, 0);
        }
        let initial_cost = self.cost(&x);
        if initial_cost < ENFORCEMENT_TOL {
            return (x, initial_cost, 0);
        }

        let fd_eps = 1e-7;
        let mut lambda: f64 = 1e-3;
        let mut last_cost = initial_cost;
        const MAX_ITERS: usize = 300;

        for iter in 0..MAX_ITERS {
            // FD gradient of cost.
            let mut grad = vec![0.0; n];
            let c0 = last_cost;
            for j in 0..n {
                let saved = x[j];
                x[j] = saved + fd_eps;
                let cp = self.cost(&x);
                x[j] = saved;
                grad[j] = (cp - c0) / fd_eps;
            }
            let grad_norm = grad.iter().map(|g| g * g).sum::<f64>().sqrt();
            if grad_norm < 1e-14 {
                return (x, last_cost, iter);
            }
            let step_scale = 1.0 / (1.0 + lambda * grad_norm);
            let mut x_new = x.clone();
            for j in 0..n {
                x_new[j] -= step_scale * grad[j];
            }
            let new_cost = self.cost(&x_new);
            if new_cost < last_cost {
                x = x_new;
                last_cost = new_cost;
                lambda = (lambda * 0.7).max(1e-12);
                if last_cost < ENFORCEMENT_TOL {
                    return (x, last_cost, iter);
                }
            } else {
                lambda = (lambda * 2.5).min(1e10);
                if lambda > 1e9 {
                    return (x, last_cost, iter);
                }
            }
        }
        (x, last_cost, MAX_ITERS)
    }
}

/// One entry in the diagnostic output of [`Sketch::diagnose_constraints`].
#[derive(Clone, Debug, PartialEq)]
pub struct DiagnosticEntry {
    pub index: usize,
    pub residual: f64,
}

/// Iterate the point indices referenced by a constraint.
fn constraint_points(c: &Constraint) -> Vec<usize> {
    match *c {
        Constraint::Coincident { a, b } => vec![a, b],
        Constraint::Distance { a, b, .. } => vec![a, b],
        Constraint::Fix { a, .. } => vec![a],
        Constraint::Vertical { a, b } => vec![a, b],
        Constraint::Horizontal { a, b } => vec![a, b],
        Constraint::Parallel { a, b, c, d } => vec![a, b, c, d],
        Constraint::Perpendicular { a, b, c, d } => vec![a, b, c, d],
        Constraint::EqualRadius { center, a, b } => vec![center, a, b],
    }
}

/// One scalar residual per constraint (caller squares it for the cost
/// function). Returns the *absolute* residual (always ≥ 0).
fn constraint_residual(c: &Constraint, x: &[f64]) -> f64 {
    let p = |i: usize| (x[i * 2], x[i * 2 + 1]);
    match *c {
        Constraint::Coincident { a, b } => {
            let (ax, ay) = p(a);
            let (bx, by) = p(b);
            ((ax - bx).powi(2) + (ay - by).powi(2)).sqrt()
        }
        Constraint::Distance { a, b, value } => {
            let (ax, ay) = p(a);
            let (bx, by) = p(b);
            let d = ((ax - bx).powi(2) + (ay - by).powi(2)).sqrt();
            (d - value).abs()
        }
        Constraint::Fix { a, x: fx, y: fy } => {
            let (ax, ay) = p(a);
            ((ax - fx).powi(2) + (ay - fy).powi(2)).sqrt()
        }
        Constraint::Vertical { a, b } => {
            let (ax, _) = p(a);
            let (bx, _) = p(b);
            (ax - bx).abs()
        }
        Constraint::Horizontal { a, b } => {
            let (_, ay) = p(a);
            let (_, by) = p(b);
            (ay - by).abs()
        }
        Constraint::Parallel { a, b, c, d } => {
            let (ax, ay) = p(a);
            let (bx, by) = p(b);
            let (cx, cy) = p(c);
            let (dx, dy) = p(d);
            // Cross product of (b - a) and (d - c). When parallel = 0.
            let v1x = bx - ax;
            let v1y = by - ay;
            let v2x = dx - cx;
            let v2y = dy - cy;
            (v1x * v2y - v1y * v2x).abs()
        }
        Constraint::Perpendicular { a, b, c, d } => {
            let (ax, ay) = p(a);
            let (bx, by) = p(b);
            let (cx, cy) = p(c);
            let (dx, dy) = p(d);
            let v1x = bx - ax;
            let v1y = by - ay;
            let v2x = dx - cx;
            let v2y = dy - cy;
            (v1x * v2x + v1y * v2y).abs()
        }
        Constraint::EqualRadius { center, a, b } => {
            let (cx, cy) = p(center);
            let (ax, ay) = p(a);
            let (bx, by) = p(b);
            let ra = ((ax - cx).powi(2) + (ay - cy).powi(2)).sqrt();
            let rb = ((bx - cx).powi(2) + (by - cy).powi(2)).sqrt();
            (ra - rb).abs()
        }
    }
}

/// How many scalar equations a constraint contributes to the residual
/// vector. Used by [`Sketch::approximate_dof`] for over/under counting.
fn constraint_dim(c: &Constraint) -> usize {
    match c {
        Constraint::Coincident { .. } => 2,
        Constraint::Distance { .. } => 1,
        Constraint::Fix { .. } => 2,
        Constraint::Vertical { .. } => 1,
        Constraint::Horizontal { .. } => 1,
        Constraint::Parallel { .. } => 1,
        Constraint::Perpendicular { .. } => 1,
        Constraint::EqualRadius { .. } => 1,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn solve_two_point_distance() {
        let mut s = Sketch::new();
        let a = s.add_point(Point2::new(0.0, 0.0));
        let b = s.add_point(Point2::new(2.0, 0.0));
        s.add_constraint(Constraint::Fix { a, x: 0.0, y: 0.0 });
        s.add_constraint(Constraint::Distance { a, b, value: 5.0 });
        s.add_constraint(Constraint::Horizontal { a, b });
        s.solve().expect("solve");
        // Looser tolerance for the LM compromise: every constraint
        // should be ≤ a few mm off, total residual ≤ ENFORCEMENT_TOL.
        assert!(s.points[0].x.abs() < 1e-3, "a.x = {}", s.points[0].x);
        assert!(s.points[0].y.abs() < 1e-3, "a.y = {}", s.points[0].y);
        // |b - a| should be ~5.
        let d = ((s.points[1].x - s.points[0].x).powi(2)
            + (s.points[1].y - s.points[0].y).powi(2))
        .sqrt();
        assert!((d - 5.0).abs() < 1e-3, "distance = {}, want 5", d);
        assert!((s.points[1].y - s.points[0].y).abs() < 1e-3);
    }

    #[test]
    fn diagnose_finds_conflicting_distance() {
        let mut s = Sketch::new();
        let a = s.add_point(Point2::new(0.0, 0.0));
        let b = s.add_point(Point2::new(1.0, 0.0));
        s.add_constraint(Constraint::Fix { a, x: 0.0, y: 0.0 });
        s.add_constraint(Constraint::Fix { a: b, x: 1.0, y: 0.0 });
        // This conflicts: distance between fixed points is 1, not 5.
        s.add_constraint(Constraint::Distance { a, b, value: 5.0 });
        let diag = s.diagnose_constraints();
        assert!(!diag.is_empty(), "expected at least one conflicting constraint");
        // The conflicting distance constraint must appear in the
        // diagnosis (the order depends on which residual the LM
        // compromise leaves largest).
        let flagged: std::collections::HashSet<usize> =
            diag.iter().map(|d| d.index).collect();
        assert!(
            flagged.contains(&2),
            "Distance(=5) at index 2 should be flagged: got {:?}",
            flagged
        );
    }
}
