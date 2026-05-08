//! Constraint solver completion tests (PR #17): backward parametric
//! solve, sparse Jacobian on large sketches, the 5 new constraint
//! variants, and the `Sketch::diagnose_constraints` diagnostic API.
//!
//! These tests exercise the gaps PR #16 left open. They share the same
//! style as `constraint_solver.rs`: small `Sketch` constructions,
//! call `solve`/`solve_with_parameters`/`diagnose_constraints`, and
//! assert geometric / structural properties of the result.

use std::collections::HashMap;

use kerf_cad::{
    DiagnosticReport, Scalar, Sketch, SketchConstraint, SketchPlane, SketchPrim,
    SolverError,
};

fn pt(id: &str, x: f64, y: f64) -> SketchPrim {
    SketchPrim::Point {
        id: id.into(),
        x: Scalar::lit(x),
        y: Scalar::lit(y),
    }
}

fn line(id: &str, from: &str, to: &str) -> SketchPrim {
    SketchPrim::Line {
        id: id.into(),
        from: from.into(),
        to: to.into(),
    }
}

fn circle(id: &str, center: &str, radius: f64, n_segments: usize) -> SketchPrim {
    SketchPrim::Circle {
        id: id.into(),
        center: center.into(),
        radius: Scalar::lit(radius),
        n_segments,
    }
}

fn point_xy(s: &Sketch, id: &str) -> (f64, f64) {
    let params: HashMap<String, f64> = HashMap::new();
    for p in &s.primitives {
        if let SketchPrim::Point { id: pid, x, y } = p
            && pid == id
        {
            return (
                x.resolve(&params).expect("x resolves"),
                y.resolve(&params).expect("y resolves"),
            );
        }
    }
    panic!("no point with id '{id}'");
}

// ===========================================================================
// TIER 3: 5 new constraint variants
// ===========================================================================

#[test]
fn solver_point_on_circle() {
    // Anchor a circle at (0,0) with radius 3. Drop a point at (5, 0)
    // (off the circle) and constrain it to lie on the circle. After
    // solve, the point's distance to the center should be 3.
    let mut s = Sketch {
        plane: SketchPlane::Xy,
        primitives: vec![
            pt("c", 0.0, 0.0),
            pt("p", 5.0, 0.0),
            circle("circ", "c", 3.0, 16),
        ],
        constraints: vec![
            SketchConstraint::FixedPoint { point: "c".into() },
            SketchConstraint::PointOnCircle {
                point: "p".into(),
                circle: "circ".into(),
            },
        ],
    };
    s.solve(&HashMap::new()).expect("PointOnCircle converges");
    let (cx, cy) = point_xy(&s, "c");
    let (px, py) = point_xy(&s, "p");
    let d = ((px - cx).powi(2) + (py - cy).powi(2)).sqrt();
    assert!((d - 3.0).abs() < 1e-3, "distance from center = {d}, expected 3");
}

#[test]
fn solver_circle_tangent_to_circle_external() {
    // Two circles, both anchored at their centers. We anchor the
    // FIRST circle's center and let the second move freely. They
    // must be externally tangent (centers' distance = r1 + r2 = 5).
    let mut s = Sketch {
        plane: SketchPlane::Xy,
        primitives: vec![
            pt("c1", 0.0, 0.0),
            pt("c2", 1.0, 0.0),
            circle("ca", "c1", 2.0, 16),
            circle("cb", "c2", 3.0, 16),
        ],
        constraints: vec![
            SketchConstraint::FixedPoint { point: "c1".into() },
            SketchConstraint::CircleTangentExternal {
                circle_a: "ca".into(),
                circle_b: "cb".into(),
            },
        ],
    };
    s.solve(&HashMap::new()).expect("ExternalTangent converges");
    let (c1x, c1y) = point_xy(&s, "c1");
    let (c2x, c2y) = point_xy(&s, "c2");
    let d = ((c1x - c2x).powi(2) + (c1y - c2y).powi(2)).sqrt();
    assert!(
        (d - 5.0).abs() < 1e-3,
        "centers distance = {d}, expected 5 (= r_a + r_b)"
    );
}

#[test]
fn solver_circle_tangent_to_circle_internal() {
    // Internal tangency: distance between centers = |r_a - r_b| = 4 - 1 = 3.
    let mut s = Sketch {
        plane: SketchPlane::Xy,
        primitives: vec![
            pt("c1", 0.0, 0.0),
            pt("c2", 1.5, 0.5),
            circle("ca", "c1", 4.0, 16),
            circle("cb", "c2", 1.0, 16),
        ],
        constraints: vec![
            SketchConstraint::FixedPoint { point: "c1".into() },
            SketchConstraint::CircleTangentInternal {
                circle_a: "ca".into(),
                circle_b: "cb".into(),
            },
        ],
    };
    s.solve(&HashMap::new()).expect("InternalTangent converges");
    let (c1x, c1y) = point_xy(&s, "c1");
    let (c2x, c2y) = point_xy(&s, "c2");
    let d = ((c1x - c2x).powi(2) + (c1y - c2y).powi(2)).sqrt();
    assert!(
        (d - 3.0).abs() < 1e-3,
        "centers distance = {d}, expected 3 (= |r_a - r_b|)"
    );
}

#[test]
fn solver_equal_angle() {
    // Two angle-pairs, all four lines have free 'to' endpoints. Anchor
    // the 'from' endpoint of each line. Set one angle to 90° (lines
    // perpendicular by construction); set the other angle to ~30° at
    // start. After EqualAngle constraint, both angles' cosines should
    // match.
    let mut s = Sketch {
        plane: SketchPlane::Xy,
        primitives: vec![
            // First angle pair: a1 along +x, a2 along +y (already at 90°).
            pt("a0", 0.0, 0.0),
            pt("a1", 1.0, 0.0),
            pt("a2", 0.0, 1.0),
            // Second angle pair: b1 along +x, b2 at 60° initially
            // (cos = 0.5). After solve, cos(b1, b2) should match
            // cos(a1, a2) = 0.
            pt("b0", 5.0, 5.0),
            pt("b1", 6.0, 5.0),
            pt("b2", 5.5, 5.866),
            line("la1", "a0", "a1"),
            line("la2", "a0", "a2"),
            line("lb1", "b0", "b1"),
            line("lb2", "b0", "b2"),
        ],
        constraints: vec![
            SketchConstraint::FixedPoint { point: "a0".into() },
            SketchConstraint::FixedPoint { point: "a1".into() },
            SketchConstraint::FixedPoint { point: "a2".into() },
            SketchConstraint::FixedPoint { point: "b0".into() },
            SketchConstraint::FixedPoint { point: "b1".into() },
            SketchConstraint::EqualAngle {
                line_a1: "la1".into(),
                line_a2: "la2".into(),
                line_b1: "lb1".into(),
                line_b2: "lb2".into(),
            },
        ],
    };
    s.solve(&HashMap::new()).expect("EqualAngle converges");
    let (a0x, a0y) = point_xy(&s, "a0");
    let (a1x, a1y) = point_xy(&s, "a1");
    let (a2x, a2y) = point_xy(&s, "a2");
    let (b0x, b0y) = point_xy(&s, "b0");
    let (b1x, b1y) = point_xy(&s, "b1");
    let (b2x, b2y) = point_xy(&s, "b2");
    let dot = |ux: f64, uy: f64, vx: f64, vy: f64| ux * vx + uy * vy;
    let mag = |ux: f64, uy: f64| (ux * ux + uy * uy).sqrt();
    let lax = a1x - a0x;
    let lay = a1y - a0y;
    let max_x = a2x - a0x;
    let may = a2y - a0y;
    let lbx = b1x - b0x;
    let lby = b1y - b0y;
    let mbx = b2x - b0x;
    let mby = b2y - b0y;
    let cos_a = dot(lax, lay, max_x, may) / (mag(lax, lay) * mag(max_x, may));
    let cos_b = dot(lbx, lby, mbx, mby) / (mag(lbx, lby) * mag(mbx, mby));
    assert!(
        (cos_a - cos_b).abs() < 1e-3,
        "cos_a = {cos_a}, cos_b = {cos_b} should match after EqualAngle"
    );
}

#[test]
fn solver_midpoint() {
    // Anchor a line from (0, 0) to (4, 2). A free point starts at (10,
    // 10). After MidPoint constraint, the point should be at (2, 1).
    let mut s = Sketch {
        plane: SketchPlane::Xy,
        primitives: vec![
            pt("a", 0.0, 0.0),
            pt("b", 4.0, 2.0),
            pt("m", 10.0, 10.0),
            line("l", "a", "b"),
        ],
        constraints: vec![
            SketchConstraint::FixedPoint { point: "a".into() },
            SketchConstraint::FixedPoint { point: "b".into() },
            SketchConstraint::MidPoint {
                point: "m".into(),
                line: "l".into(),
            },
        ],
    };
    s.solve(&HashMap::new()).expect("MidPoint converges");
    let (mx, my) = point_xy(&s, "m");
    assert!(
        (mx - 2.0).abs() < 1e-3 && (my - 1.0).abs() < 1e-3,
        "midpoint = ({mx}, {my}), expected (2, 1)"
    );
}

#[test]
fn solver_distance_from_line() {
    // A horizontal line on the x-axis (y=0). A point near the line.
    // Constrain its *signed* perpendicular distance from the line.
    // The signed perpendicular is the 2D cross of (lt-lf) with
    // (p-lf), divided by |lt-lf|. With lt-lf = (4, 0), a point at
    // y > 0 produces a *negative* cross (counter-clockwise side is
    // negative under this convention).
    //
    // We pick the test target to verify the magnitude — set distance
    // = -3 so the solved point lands at y = +3 (above the line),
    // confirming both sign convention and magnitude.
    let mut s = Sketch {
        plane: SketchPlane::Xy,
        primitives: vec![
            pt("a", 0.0, 0.0),
            pt("b", 4.0, 0.0),
            pt("p", 1.0, 0.1),
            line("l", "a", "b"),
        ],
        constraints: vec![
            SketchConstraint::FixedPoint { point: "a".into() },
            SketchConstraint::FixedPoint { point: "b".into() },
            SketchConstraint::DistanceFromLine {
                point: "p".into(),
                line: "l".into(),
                distance: Scalar::lit(-3.0),
            },
        ],
    };
    s.solve(&HashMap::new()).expect("DistanceFromLine converges");
    let (_, py) = point_xy(&s, "p");
    // Magnitude should match.
    assert!(
        (py.abs() - 3.0).abs() < 1e-3,
        "|point y| = {}, expected 3",
        py.abs()
    );
    // Verify sign convention: distance=-3 with line direction +x
    // produces a point at y=+3 (per cross-product convention).
    assert!(py > 0.0, "expected point above line (y>0), got y={py}");
}

// ===========================================================================
// TIER 2: sparse Jacobian — solver still works on a 50+ DOF sketch
// ===========================================================================

#[test]
fn solver_handles_100_dof_sketch_efficiently() {
    // Build a 50-point chain of horizontally-coincident pairs:
    //   - 50 points start at random-ish positions
    //   - Constraint: pair (2i, 2i+1) Coincident → forces them to merge
    // Total DOFs = 100. Constraints touch only 4 of those at a time —
    // a textbook sparse-Jacobian case. Without sparse storage, building
    // J^T J at 100 × 100 = 10000 entries per Newton step would dominate
    // runtime; with sparse, it's ~25 nonzero pairs per row.
    let mut prims: Vec<SketchPrim> = Vec::new();
    let mut constraints: Vec<SketchConstraint> = Vec::new();
    for i in 0..50 {
        let id_a = format!("p{}_a", i);
        let id_b = format!("p{}_b", i);
        prims.push(pt(&id_a, i as f64, 0.0));
        prims.push(pt(&id_b, i as f64 + 0.3, 0.5));
        constraints.push(SketchConstraint::Coincident {
            a: id_a.clone(),
            b: id_b.clone(),
        });
    }
    let mut s = Sketch {
        plane: SketchPlane::Xy,
        primitives: prims,
        constraints,
    };
    let start = std::time::Instant::now();
    let iters = s.solve(&HashMap::new()).expect("100-DOF sketch solves");
    let elapsed = start.elapsed();
    // Coincident constraints are linear so Newton finishes in 1 iter
    // and the line search accepts the full step. We give a generous
    // upper bound on iterations and a generous time budget — the test
    // is mostly about "didn't blow up" and confirming sparse path is
    // wired in.
    assert!(iters <= 5, "100-DOF sketch should converge in ≤ 5 iters (got {iters})");
    assert!(
        elapsed.as_millis() < 2000,
        "100-DOF sketch should solve in <2s with sparse Jacobian; took {}ms",
        elapsed.as_millis()
    );
    // Verify all coincident pairs actually coincide.
    for i in 0..50 {
        let (ax, ay) = point_xy(&s, &format!("p{}_a", i));
        let (bx, by) = point_xy(&s, &format!("p{}_b", i));
        assert!(
            (ax - bx).abs() < 1e-3 && (ay - by).abs() < 1e-3,
            "pair {i} not coincident: ({ax},{ay}) vs ({bx},{by})"
        );
    }
}

#[test]
fn solver_sparse_jacobian_handles_50_dof() {
    // 25-point chain with Coincident constraints — linear residuals,
    // so Newton converges in 1-2 steps. Each constraint touches 4
    // DOFs out of 50. With dense J^T J (50×50 = 2500 entries), Newton
    // would still finish but the per-step cost is O(rows · n²).
    // Sparse storage keeps it O(rows · k²) where k=4, so ~625 ops per
    // step instead of 31000.
    let mut prims: Vec<SketchPrim> = Vec::new();
    let mut constraints: Vec<SketchConstraint> = Vec::new();
    for i in 0..25 {
        let id_a = format!("p{}_a", i);
        let id_b = format!("p{}_b", i);
        prims.push(pt(&id_a, i as f64, 0.0));
        prims.push(pt(&id_b, i as f64 + 0.7, 0.5));
        constraints.push(SketchConstraint::Coincident {
            a: id_a,
            b: id_b,
        });
    }
    let mut s = Sketch {
        plane: SketchPlane::Xy,
        primitives: prims,
        constraints,
    };
    let start = std::time::Instant::now();
    s.solve(&HashMap::new()).expect("50-DOF chain solves");
    let elapsed = start.elapsed();
    assert!(
        elapsed.as_millis() < 5000,
        "50-DOF chain should solve in <5s; took {}ms",
        elapsed.as_millis()
    );
    // Verify all coincident pairs converged.
    for i in 0..25 {
        let (ax, ay) = point_xy(&s, &format!("p{}_a", i));
        let (bx, by) = point_xy(&s, &format!("p{}_b", i));
        assert!(
            (ax - bx).abs() < 1e-3 && (ay - by).abs() < 1e-3,
            "pair {i} not coincident: ({ax},{ay}) vs ({bx},{by})"
        );
    }
}

// ===========================================================================
// TIER 4: diagnostics
// ===========================================================================

#[test]
fn sketch_diagnose_under_constrained_reports_dofs() {
    // Two points, only one constrained. 4 DOFs total, 1 row of
    // constraints (Distance contributes 1 scalar row). Effective rank
    // = 1, free DOFs = 3 (rotation of the pair around any anchor + one
    // direction is undetermined).
    let s = Sketch {
        plane: SketchPlane::Xy,
        primitives: vec![pt("a", 0.0, 0.0), pt("b", 3.0, 4.0)],
        constraints: vec![SketchConstraint::Distance {
            a: "a".into(),
            b: "b".into(),
            value: Scalar::lit(5.0),
        }],
    };
    let report: DiagnosticReport = s
        .diagnose_constraints(&HashMap::new())
        .expect("diagnose ok");
    assert_eq!(report.dof_count, 4, "two points → 4 DOFs");
    assert_eq!(report.total_rows, 1, "one Distance row");
    assert_eq!(report.effective_rank, 1, "Distance pins one DOF");
    assert_eq!(report.free_dofs, 3, "expect 3 free DOFs");
    assert_eq!(report.redundant_rows, 0);
    assert!(report.is_under_constrained);
    assert!(!report.is_over_constrained);
    assert!(!report.is_well_constrained);
}

#[test]
fn sketch_diagnose_well_constrained() {
    // FixedPoint(a) (rank 2) + Distance(a, b, 5) + Horizontal(line ab)
    // = 4 rows, 4 DOFs, full rank. Initial coords already satisfy
    // FixedPoint and Horizontal but not Distance (distance is 1, not 5).
    // Rank should be 4 (since the rows are linearly independent over
    // the 4 DOFs); residual non-zero.
    let s = Sketch {
        plane: SketchPlane::Xy,
        primitives: vec![pt("a", 0.0, 0.0), pt("b", 1.0, 0.0), line("l", "a", "b")],
        constraints: vec![
            SketchConstraint::FixedPoint { point: "a".into() },
            SketchConstraint::FixedPoint { point: "b".into() },
            SketchConstraint::Horizontal { line: "l".into() },
        ],
    };
    let report = s
        .diagnose_constraints(&HashMap::new())
        .expect("diagnose ok");
    assert_eq!(report.dof_count, 4);
    // 2 FixedPoint constraints contribute 2 rows each (4 total) +
    // 1 Horizontal row = 5 rows.
    assert_eq!(report.total_rows, 5);
    // 4 DOFs pinned (2x FixedPoint), Horizontal is redundantly
    // satisfied at the initial coords (b.y - a.y = 0). The Horizontal
    // row is a linear combination of the FixedPoint rows.
    assert!(report.effective_rank >= 4);
    assert_eq!(report.free_dofs, 0);
    assert!(report.redundant_rows >= 1, "Horizontal redundant w/ FixedPoints");
    assert!(!report.is_under_constrained);
    // The initial position satisfies all constraints, so:
    assert!(report.is_well_constrained);
}

#[test]
fn sketch_diagnose_over_constrained_redundant() {
    // Same Distance constraint twice — rank 1, two rows, one redundant.
    let s = Sketch {
        plane: SketchPlane::Xy,
        primitives: vec![pt("a", 0.0, 0.0), pt("b", 5.0, 0.0)],
        constraints: vec![
            SketchConstraint::Distance {
                a: "a".into(),
                b: "b".into(),
                value: Scalar::lit(5.0),
            },
            SketchConstraint::Distance {
                a: "a".into(),
                b: "b".into(),
                value: Scalar::lit(5.0),
            },
        ],
    };
    let report = s
        .diagnose_constraints(&HashMap::new())
        .expect("diagnose ok");
    assert_eq!(report.dof_count, 4);
    assert_eq!(report.total_rows, 2);
    assert_eq!(report.effective_rank, 1);
    assert_eq!(report.redundant_rows, 1);
    assert!(report.is_over_constrained, "duplicate Distance is redundant");
    assert!(report.is_under_constrained, "still under-constrained too");
}

// ===========================================================================
// TIER 1: backward parametric solving
// ===========================================================================

#[test]
fn solver_with_parameter_propagates_correctly() {
    // A line a-b with Distance constraint referencing $len. Initial
    // params: len = 3. solve_with_parameters("len", 7.0) should:
    //   1. Re-solve so |a - b| == 7.0.
    //   2. Update the Point coordinates to reflect the new geometry.
    // (Anchor 'a' so the result is deterministic.)
    let mut s = Sketch {
        plane: SketchPlane::Xy,
        primitives: vec![pt("a", 0.0, 0.0), pt("b", 1.0, 0.0)],
        constraints: vec![
            SketchConstraint::FixedPoint { point: "a".into() },
            SketchConstraint::Distance {
                a: "a".into(),
                b: "b".into(),
                value: Scalar::param("len"),
            },
        ],
    };
    let mut params: HashMap<String, f64> = HashMap::new();
    params.insert("len".into(), 3.0);
    // Sanity: solving with len=3 puts b at distance 3.
    s.solve(&params).expect("initial solve ok");
    let (ax, ay) = point_xy(&s, "a");
    let (bx, by) = point_xy(&s, "b");
    let d = ((ax - bx).powi(2) + (ay - by).powi(2)).sqrt();
    assert!((d - 3.0).abs() < 1e-3, "initial distance = {d}, expected 3");
    // Now backward solve: we want len → 7.
    s.solve_with_parameters(&params, "len", 7.0)
        .expect("backward solve ok");
    let (ax2, ay2) = point_xy(&s, "a");
    let (bx2, by2) = point_xy(&s, "b");
    // 'a' should still be at origin (FixedPoint).
    assert!(ax2.abs() < 1e-3 && ay2.abs() < 1e-3, "anchor moved: ({ax2},{ay2})");
    let d2 = ((ax2 - bx2).powi(2) + (ay2 - by2).powi(2)).sqrt();
    assert!(
        (d2 - 7.0).abs() < 1e-3,
        "new distance = {d2}, expected 7 after solve_with_parameters"
    );
}

#[test]
fn solver_parametric_jacobian_returns_directions() {
    // Distance(a, b, $len) with a fixed and b also fixed. The fully
    // pinned setup has J^T J full rank, so the parametric Jacobian
    // returns the unique solution: 'a' doesn't move, 'b' moves along
    // (a → b) at unit speed (since the distance scales 1:1 with len).
    //
    // We pin 'b' along a specific axis (Horizontal + extra anchor) to
    // avoid rank deficiency. Initial b at (3, 0); make the line
    // horizontal by anchoring b.y. We do that with FixedPoint(b)
    // *after* the initial solve picks b. Concretely: anchor a,
    // require Horizontal(line a-b), and require Distance(a, b) =
    // $len. This pins the system: b is at (len, 0) for any value of
    // len, so ∂b.x/∂len = 1, ∂b.y/∂len = 0.
    let mut s = Sketch {
        plane: SketchPlane::Xy,
        primitives: vec![pt("a", 0.0, 0.0), pt("b", 3.0, 0.5), line("l", "a", "b")],
        constraints: vec![
            SketchConstraint::FixedPoint { point: "a".into() },
            SketchConstraint::Horizontal { line: "l".into() },
            SketchConstraint::Distance {
                a: "a".into(),
                b: "b".into(),
                value: Scalar::param("len"),
            },
        ],
    };
    let mut params: HashMap<String, f64> = HashMap::new();
    params.insert("len".into(), 5.0);
    // Solve once so the sketch is at a clean state for the parametric
    // probe.
    s.solve(&params).expect("initial solve");
    let dx_dp = s.parametric_jacobian(&params, "len").expect("jac ok");
    // Layout: [a.x, a.y, b.x, b.y]
    assert!(
        dx_dp[0].abs() < 1e-2,
        "a.x ∂/∂len = {} (should be 0, anchored)",
        dx_dp[0]
    );
    assert!(
        dx_dp[1].abs() < 1e-2,
        "a.y ∂/∂len = {} (should be 0, anchored)",
        dx_dp[1]
    );
    assert!(
        (dx_dp[2] - 1.0).abs() < 0.05,
        "b.x ∂/∂len = {}, expected ~1.0 (line lies along x-axis)",
        dx_dp[2]
    );
    assert!(
        dx_dp[3].abs() < 0.05,
        "b.y ∂/∂len = {} (should be 0, line is horizontal)",
        dx_dp[3]
    );
}

#[test]
fn solver_with_parameter_unknown_param_errors() {
    // Calling solve_with_parameters with a parameter no constraint
    // references should return ParamResolution.
    let mut s = Sketch {
        plane: SketchPlane::Xy,
        primitives: vec![pt("a", 0.0, 0.0), pt("b", 1.0, 0.0)],
        constraints: vec![SketchConstraint::Distance {
            a: "a".into(),
            b: "b".into(),
            value: Scalar::lit(5.0),
        }],
    };
    let err = s
        .solve_with_parameters(&HashMap::new(), "ghost", 7.0)
        .unwrap_err();
    assert!(
        matches!(err, SolverError::ParamResolution(_)),
        "expected ParamResolution, got {err:?}"
    );
}
