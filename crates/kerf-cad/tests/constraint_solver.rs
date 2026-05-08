//! Iterative 2D constraint solver tests.
//!
//! Each test builds a small `Sketch`, calls `solve`, and verifies that
//! the resulting Point coordinates satisfy the constraint(s). A handful
//! of negative tests cover the structural error paths
//! (`Contradictory`, `OverConstrained`, parameter resolution).

use std::collections::HashMap;

use kerf_cad::{Scalar, Sketch, SketchConstraint, SketchPlane, SketchPrim, SolverError};

/// Look up a Point by id and return its resolved (x, y) as f64s. Panics
/// if missing or unresolvable — tests are happy to crash loudly.
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

#[test]
fn solver_distance_constraint() {
    // Two points start 1.0 apart; constrain them to be 5.0 apart. After
    // solve, the distance should match within tolerance.
    let mut s = Sketch {
        plane: SketchPlane::Xy,
        primitives: vec![pt("a", 0.0, 0.0), pt("b", 1.0, 0.0)],
        constraints: vec![SketchConstraint::Distance {
            a: "a".into(),
            b: "b".into(),
            value: Scalar::lit(5.0),
        }],
    };
    let iters = s.solve(&HashMap::new()).expect("solve converges");
    assert!(iters > 0, "should take at least one iteration");

    let (ax, ay) = point_xy(&s, "a");
    let (bx, by) = point_xy(&s, "b");
    let d = ((ax - bx).powi(2) + (ay - by).powi(2)).sqrt();
    assert!((d - 5.0).abs() < 1e-4, "distance = {d}, expected 5.0");
}

#[test]
fn solver_horizontal_line() {
    // A line from (0, 0) to (1, 1) constrained to be horizontal. After
    // solve, both endpoints should share a y coordinate. (Exact value
    // doesn't matter — under-constrained in y, but the residual goes
    // to zero.)
    let mut s = Sketch {
        plane: SketchPlane::Xy,
        primitives: vec![pt("a", 0.0, 0.0), pt("b", 1.0, 1.0), line("l", "a", "b")],
        constraints: vec![SketchConstraint::Horizontal { line: "l".into() }],
    };
    s.solve(&HashMap::new()).expect("solve converges");
    let (_, ay) = point_xy(&s, "a");
    let (_, by) = point_xy(&s, "b");
    assert!((ay - by).abs() < 1e-4, "ay = {ay}, by = {by}");
}

#[test]
fn solver_vertical_line() {
    // A diagonal line constrained to be vertical. After solve, x's match.
    let mut s = Sketch {
        plane: SketchPlane::Xy,
        primitives: vec![pt("a", 0.0, 0.0), pt("b", 1.0, 1.0), line("l", "a", "b")],
        constraints: vec![SketchConstraint::Vertical { line: "l".into() }],
    };
    s.solve(&HashMap::new()).expect("solve converges");
    let (ax, _) = point_xy(&s, "a");
    let (bx, _) = point_xy(&s, "b");
    assert!((ax - bx).abs() < 1e-4, "ax = {ax}, bx = {bx}");
}

#[test]
fn solver_parallel_lines() {
    // Two non-parallel lines; constrain them parallel. Verify the cross
    // product of their direction vectors goes to zero.
    let mut s = Sketch {
        plane: SketchPlane::Xy,
        primitives: vec![
            pt("a1", 0.0, 0.0),
            pt("a2", 1.0, 0.5),
            pt("b1", 0.0, 2.0),
            pt("b2", 1.0, 1.0),
            line("la", "a1", "a2"),
            line("lb", "b1", "b2"),
        ],
        constraints: vec![SketchConstraint::Parallel {
            line_a: "la".into(),
            line_b: "lb".into(),
        }],
    };
    s.solve(&HashMap::new()).expect("solve converges");
    let (a1x, a1y) = point_xy(&s, "a1");
    let (a2x, a2y) = point_xy(&s, "a2");
    let (b1x, b1y) = point_xy(&s, "b1");
    let (b2x, b2y) = point_xy(&s, "b2");
    let ax = a2x - a1x;
    let ay = a2y - a1y;
    let bx = b2x - b1x;
    let by = b2y - b1y;
    let cross = ax * by - ay * bx;
    assert!(cross.abs() < 1e-4, "cross = {cross}, expected ~0");
}

#[test]
fn solver_perpendicular_lines() {
    // Two parallel lines (cross = 0) constrained to be perpendicular
    // (dot = 0). Starting from a small non-zero perpendicular offset
    // so the gradient is non-zero. After solve, dot should vanish.
    let mut s = Sketch {
        plane: SketchPlane::Xy,
        primitives: vec![
            pt("a1", 0.0, 0.0),
            pt("a2", 1.0, 0.1),
            pt("b1", 0.0, 0.0),
            pt("b2", 1.0, 0.5),
            line("la", "a1", "a2"),
            line("lb", "b1", "b2"),
        ],
        constraints: vec![SketchConstraint::Perpendicular {
            line_a: "la".into(),
            line_b: "lb".into(),
        }],
    };
    s.solve(&HashMap::new()).expect("solve converges");
    let (a1x, a1y) = point_xy(&s, "a1");
    let (a2x, a2y) = point_xy(&s, "a2");
    let (b1x, b1y) = point_xy(&s, "b1");
    let (b2x, b2y) = point_xy(&s, "b2");
    let ax = a2x - a1x;
    let ay = a2y - a1y;
    let bx = b2x - b1x;
    let by = b2y - b1y;
    let dot = ax * bx + ay * by;
    assert!(dot.abs() < 1e-4, "dot = {dot}, expected ~0");
}

#[test]
fn solver_coincident_points() {
    // Two distinct points constrained coincident. Both should converge
    // to the same coordinate.
    let mut s = Sketch {
        plane: SketchPlane::Xy,
        primitives: vec![pt("a", 0.0, 0.0), pt("b", 3.0, 4.0)],
        constraints: vec![SketchConstraint::Coincident {
            a: "a".into(),
            b: "b".into(),
        }],
    };
    s.solve(&HashMap::new()).expect("solve converges");
    let (ax, ay) = point_xy(&s, "a");
    let (bx, by) = point_xy(&s, "b");
    assert!(
        (ax - bx).abs() < 1e-4 && (ay - by).abs() < 1e-4,
        "a = ({ax}, {ay}), b = ({bx}, {by})"
    );
}

#[test]
fn solver_fixed_point_anchors_movement() {
    // A FixedPoint constraint should keep one point in place even when
    // another constraint would otherwise move it. Setup: two points at
    // (0,0) and (1,0). Distance = 5.0 alone could move either; pinning
    // 'a' forces 'b' to absorb the motion.
    let mut s = Sketch {
        plane: SketchPlane::Xy,
        primitives: vec![pt("a", 0.0, 0.0), pt("b", 1.0, 0.0)],
        constraints: vec![
            SketchConstraint::FixedPoint { point: "a".into() },
            SketchConstraint::Distance {
                a: "a".into(),
                b: "b".into(),
                value: Scalar::lit(5.0),
            },
        ],
    };
    s.solve(&HashMap::new()).expect("solve converges");
    let (ax, ay) = point_xy(&s, "a");
    let (bx, by) = point_xy(&s, "b");
    // 'a' should be (essentially) at its initial (0, 0).
    assert!(ax.abs() < 1e-4 && ay.abs() < 1e-4, "a moved to ({ax}, {ay})");
    // 'b' should be 5.0 away from 'a'.
    let d = ((ax - bx).powi(2) + (ay - by).powi(2)).sqrt();
    assert!((d - 5.0).abs() < 1e-4, "distance = {d}");
}

#[test]
fn solver_overconstrained_returns_error() {
    // Hard cap iterations to 1 so even a solvable problem cannot
    // converge — checks the OverConstrained error path. The setup
    // itself is feasible, but a single gradient step starting from a
    // tight initial residual can't drop the residual below 1e-9.
    use kerf_cad::SolverConfig;
    let mut s = Sketch {
        plane: SketchPlane::Xy,
        primitives: vec![
            pt("a", 0.0, 0.0),
            pt("b", 1.0, 0.5),
            pt("c", 0.0, 2.0),
            pt("d", 1.0, 3.0),
            line("la", "a", "b"),
            line("lb", "c", "d"),
        ],
        constraints: vec![SketchConstraint::Parallel {
            line_a: "la".into(),
            line_b: "lb".into(),
        }],
    };
    let cfg = SolverConfig {
        max_iterations: 1,
        ..SolverConfig::default()
    };
    let err = s.solve_with_config(&HashMap::new(), &cfg).unwrap_err();
    assert!(
        matches!(err, SolverError::OverConstrained { .. }),
        "expected OverConstrained, got {err:?}"
    );
}

#[test]
fn solver_contradictory_returns_error() {
    // Two Distance constraints on the same pair with different values —
    // no configuration satisfies both.
    let mut s = Sketch {
        plane: SketchPlane::Xy,
        primitives: vec![pt("a", 0.0, 0.0), pt("b", 1.0, 0.0)],
        constraints: vec![
            SketchConstraint::Distance {
                a: "a".into(),
                b: "b".into(),
                value: Scalar::lit(5.0),
            },
            SketchConstraint::Distance {
                a: "a".into(),
                b: "b".into(),
                value: Scalar::lit(10.0),
            },
        ],
    };
    let err = s.solve(&HashMap::new()).unwrap_err();
    // Either Contradictory (gradient-collapse signature, the expected
    // case for two opposing distance constraints) or OverConstrained
    // (line-search keeps shrinking) is fine — both correctly refuse to
    // pretend a solution exists. We just need to not return Ok.
    assert!(
        matches!(err, SolverError::Contradictory { .. } | SolverError::OverConstrained { .. }),
        "expected a non-convergence error, got {err:?}"
    );
}

#[test]
fn solver_param_resolution() {
    // Distance constraint with a Scalar::Param value. The solver must
    // resolve "len" against the params table at solve time.
    let mut s = Sketch {
        plane: SketchPlane::Xy,
        primitives: vec![pt("a", 0.0, 0.0), pt("b", 1.0, 0.0)],
        constraints: vec![SketchConstraint::Distance {
            a: "a".into(),
            b: "b".into(),
            value: Scalar::param("len"),
        }],
    };
    let mut params = HashMap::new();
    params.insert("len".into(), 5.0);
    s.solve(&params).expect("solve converges");
    let (ax, ay) = point_xy(&s, "a");
    let (bx, by) = point_xy(&s, "b");
    let d = ((ax - bx).powi(2) + (ay - by).powi(2)).sqrt();
    assert!((d - 5.0).abs() < 1e-4, "distance = {d}, expected 5.0");
}

#[test]
fn solver_unknown_point_returns_error() {
    // A constraint that references an undefined Point id should fail at
    // setup time with UnknownPoint.
    let mut s = Sketch {
        plane: SketchPlane::Xy,
        primitives: vec![pt("a", 0.0, 0.0)],
        constraints: vec![SketchConstraint::Coincident {
            a: "a".into(),
            b: "ghost".into(),
        }],
    };
    let err = s.solve(&HashMap::new()).unwrap_err();
    assert!(
        matches!(err, SolverError::UnknownPoint(ref id) if id == "ghost"),
        "expected UnknownPoint(ghost), got {err:?}"
    );
}

#[test]
fn solver_no_constraints_is_noop() {
    // A sketch with no constraints should solve in 0 iterations and
    // leave coordinates untouched.
    let mut s = Sketch {
        plane: SketchPlane::Xy,
        primitives: vec![pt("a", 1.5, 2.5), pt("b", 3.5, 4.5)],
        constraints: vec![],
    };
    let iters = s.solve(&HashMap::new()).expect("noop solve");
    assert_eq!(iters, 0);
    let (ax, ay) = point_xy(&s, "a");
    assert!((ax - 1.5).abs() < 1e-12 && (ay - 2.5).abs() < 1e-12);
}

#[test]
fn solver_horizontal_plus_distance_fixes_geometry() {
    // Combine constraints: horizontal line + fixed start + distance.
    // Result is fully determined: 'a' stays at origin, 'b' lies on the
    // x-axis at distance 4.
    let mut s = Sketch {
        plane: SketchPlane::Xy,
        primitives: vec![pt("a", 0.0, 0.0), pt("b", 0.5, 0.5), line("l", "a", "b")],
        constraints: vec![
            SketchConstraint::FixedPoint { point: "a".into() },
            SketchConstraint::Horizontal { line: "l".into() },
            SketchConstraint::Distance {
                a: "a".into(),
                b: "b".into(),
                value: Scalar::lit(4.0),
            },
        ],
    };
    s.solve(&HashMap::new()).expect("solve converges");
    let (ax, ay) = point_xy(&s, "a");
    let (bx, by) = point_xy(&s, "b");
    assert!(ax.abs() < 1e-4 && ay.abs() < 1e-4, "a = ({ax}, {ay})");
    assert!(by.abs() < 1e-4, "b should land on x-axis, by = {by}");
    let d = ((ax - bx).powi(2) + (ay - by).powi(2)).sqrt();
    assert!((d - 4.0).abs() < 1e-4, "distance = {d}");
}
