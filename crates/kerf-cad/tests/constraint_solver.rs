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
    // converge — checks the OverConstrained error path. We disable
    // Newton (which would solve the parallel-lines problem in a
    // single linearized step) and use plain gradient descent so the
    // single-iteration cap is biting.
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
        use_newton: false,
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

// ===========================================================================
// Newton-Raphson, analytic Jacobians, and conflict-identification tests
// (added in PR #12 alongside the new constraint variants).
// ===========================================================================

fn circle(id: &str, center: &str, radius: f64, n_segments: usize) -> SketchPrim {
    SketchPrim::Circle {
        id: id.into(),
        center: center.into(),
        radius: Scalar::lit(radius),
        n_segments,
    }
}

#[test]
fn solver_analytic_gradient_matches_finite_difference() {
    // For each of the 7 original `SketchConstraint` variants, build a
    // small sketch and check that the analytic gradient (the primary
    // path used by the solver) agrees with central finite differences
    // at the initial coordinates within a tight tolerance.
    use kerf_cad::solver::testing::analytic_and_fd_gradients;

    let cases: Vec<(&str, Sketch)> = vec![
        (
            "Distance",
            Sketch {
                plane: SketchPlane::Xy,
                primitives: vec![pt("a", 0.0, 0.0), pt("b", 3.0, 4.0)],
                constraints: vec![SketchConstraint::Distance {
                    a: "a".into(),
                    b: "b".into(),
                    value: Scalar::lit(2.0),
                }],
            },
        ),
        (
            "Horizontal",
            Sketch {
                plane: SketchPlane::Xy,
                primitives: vec![pt("a", 0.0, 0.0), pt("b", 1.0, 0.7), line("l", "a", "b")],
                constraints: vec![SketchConstraint::Horizontal { line: "l".into() }],
            },
        ),
        (
            "Vertical",
            Sketch {
                plane: SketchPlane::Xy,
                primitives: vec![pt("a", 0.2, 0.0), pt("b", 1.0, 1.0), line("l", "a", "b")],
                constraints: vec![SketchConstraint::Vertical { line: "l".into() }],
            },
        ),
        (
            "Parallel",
            Sketch {
                plane: SketchPlane::Xy,
                primitives: vec![
                    pt("a1", 0.0, 0.0),
                    pt("a2", 1.5, 0.5),
                    pt("b1", 0.0, 2.0),
                    pt("b2", 1.0, 3.0),
                    line("la", "a1", "a2"),
                    line("lb", "b1", "b2"),
                ],
                constraints: vec![SketchConstraint::Parallel {
                    line_a: "la".into(),
                    line_b: "lb".into(),
                }],
            },
        ),
        (
            "Perpendicular",
            Sketch {
                plane: SketchPlane::Xy,
                primitives: vec![
                    pt("a1", 0.0, 0.0),
                    pt("a2", 1.0, 0.4),
                    pt("b1", 0.0, 0.0),
                    pt("b2", 1.0, 1.2),
                    line("la", "a1", "a2"),
                    line("lb", "b1", "b2"),
                ],
                constraints: vec![SketchConstraint::Perpendicular {
                    line_a: "la".into(),
                    line_b: "lb".into(),
                }],
            },
        ),
        (
            "Coincident",
            Sketch {
                plane: SketchPlane::Xy,
                primitives: vec![pt("a", 0.0, 0.0), pt("b", 0.7, 0.3)],
                constraints: vec![SketchConstraint::Coincident {
                    a: "a".into(),
                    b: "b".into(),
                }],
            },
        ),
        (
            "FixedPoint",
            Sketch {
                plane: SketchPlane::Xy,
                primitives: vec![pt("a", 0.5, 0.5)],
                // FixedPoint at the initial position has zero residual
                // and zero gradient on both paths — fine for parity.
                // We check at a moved-from-initial state by perturbing
                // through a second constraint's residual… but for a
                // pure-parity check, even the trivial case suffices.
                constraints: vec![SketchConstraint::FixedPoint { point: "a".into() }],
            },
        ),
    ];

    for (name, s) in &cases {
        let (analytic, fd) =
            analytic_and_fd_gradients(s, &HashMap::new(), 1e-6).expect("solver state ok");
        assert_eq!(analytic.len(), fd.len(), "{name}: dim mismatch");
        for (i, (a, f)) in analytic.iter().zip(fd.iter()).enumerate() {
            let diff = (a - f).abs();
            // Tolerance: FD has ~1e-5 truncation noise; the gradient of
            // a quadratic is exact under FD. We allow 1e-3 absolute /
            // 1e-3 relative to handle the worst-case truncation on
            // bilinear (Parallel/Perpendicular) and sqrt-flavored
            // constraints (Distance/Coincident).
            let scale = a.abs().max(f.abs()).max(1.0);
            assert!(
                diff < 1e-3 * scale,
                "{name}[{i}]: analytic = {a}, fd = {f}, diff = {diff}"
            );
        }
    }
}

#[test]
fn solver_newton_converges_in_fewer_iterations() {
    // On a problem where the residual is well-modeled by its linear
    // approximation, Newton-Raphson should take fewer iterations than
    // plain gradient descent. We use a 4-point right-triangle setup
    // with multiple distance constraints — Newton typically needs
    // 5-15 iterations, gradient descent hundreds.
    use kerf_cad::solver::testing::iterations_newton_vs_gd;

    let s = Sketch {
        plane: SketchPlane::Xy,
        primitives: vec![
            pt("a", 0.0, 0.0),
            pt("b", 1.1, 0.0),
            pt("c", 0.0, 1.1),
        ],
        constraints: vec![
            SketchConstraint::FixedPoint { point: "a".into() },
            SketchConstraint::Horizontal { line: "ab".into() },
            SketchConstraint::Vertical { line: "ac".into() },
            SketchConstraint::Distance {
                a: "a".into(),
                b: "b".into(),
                value: Scalar::lit(3.0),
            },
            SketchConstraint::Distance {
                a: "a".into(),
                b: "c".into(),
                value: Scalar::lit(4.0),
            },
        ]
        .into_iter()
        .chain(std::iter::once(SketchConstraint::Distance {
            a: "b".into(),
            b: "c".into(),
            value: Scalar::lit(5.0),
        }))
        .collect(),
    };
    // We need lines `ab` and `ac` for Horizontal/Vertical. Inject them.
    let mut s = s;
    s.primitives.push(line("ab", "a", "b"));
    s.primitives.push(line("ac", "a", "c"));

    let (n_iters, g_iters) = iterations_newton_vs_gd(&s, &HashMap::new())
        .expect("both solvers should converge");
    assert!(
        n_iters < g_iters,
        "Newton took {n_iters} iters, gradient descent took {g_iters} — Newton should be faster"
    );
}

#[test]
fn solver_contradictory_identifies_minimal_subset() {
    // 4 constraints, only 2 of which are mutually contradictory:
    //   (0) FixedPoint a   — fine
    //   (1) Distance a-b = 5   ← conflicts with (3)
    //   (2) Horizontal line   — fine on its own
    //   (3) Distance a-b = 10  ← conflicts with (1)
    // The minimal failing subset is {1, 3}.
    let mut s = Sketch {
        plane: SketchPlane::Xy,
        primitives: vec![pt("a", 0.0, 0.0), pt("b", 1.0, 0.0), line("l", "a", "b")],
        constraints: vec![
            SketchConstraint::FixedPoint { point: "a".into() },
            SketchConstraint::Distance {
                a: "a".into(),
                b: "b".into(),
                value: Scalar::lit(5.0),
            },
            SketchConstraint::Horizontal { line: "l".into() },
            SketchConstraint::Distance {
                a: "a".into(),
                b: "b".into(),
                value: Scalar::lit(10.0),
            },
        ],
    };
    let err = s.solve(&HashMap::new()).unwrap_err();
    match err {
        SolverError::Contradictory { conflicting, .. } => {
            assert!(
                conflicting.contains(&1) && conflicting.contains(&3),
                "minimal conflicting subset must include both Distance constraints (1, 3); got {conflicting:?}"
            );
            assert!(
                conflicting.len() <= 2,
                "minimal subset should be size 2 (both Distance); got {conflicting:?}"
            );
        }
        other => panic!("expected Contradictory, got {other:?}"),
    }
}

#[test]
fn solver_tangent_line_to_circle() {
    // A line at y=0 (horizontal, anchored) and a circle centered at
    // (2, 3) with radius 2. Constrain the line tangent to the circle.
    // The line being horizontal (y=0) is 3 units from the circle's
    // center; tangency requires |3| - 2 = 0, so the center must move
    // to y = ±2 (or the line must move). With both endpoints free in
    // y after solve, the residual goes to zero — we just check that.
    let mut s = Sketch {
        plane: SketchPlane::Xy,
        primitives: vec![
            pt("a", 0.0, 0.0),
            pt("b", 4.0, 0.0),
            pt("c", 2.0, 3.0),
            line("l", "a", "b"),
            circle("circ", "c", 2.0, 16),
        ],
        constraints: vec![
            SketchConstraint::FixedPoint { point: "a".into() },
            SketchConstraint::FixedPoint { point: "b".into() },
            SketchConstraint::Horizontal { line: "l".into() },
            SketchConstraint::TangentLineToCircle {
                line: "l".into(),
                circle: "circ".into(),
            },
        ],
    };
    s.solve(&HashMap::new()).expect("tangent solve converges");
    let (ax, ay) = point_xy(&s, "a");
    let (bx, by) = point_xy(&s, "b");
    let (cx, cy) = point_xy(&s, "c");
    // Line stayed at y=0 (anchored).
    assert!(ay.abs() < 1e-4 && by.abs() < 1e-4, "line endpoints stayed on y=0");
    let _ = (ax, bx, cx);
    // Center y must be ±2 (distance from y=0 is the circle's radius).
    assert!(
        (cy.abs() - 2.0).abs() < 1e-3,
        "circle center y should be ±2 after tangent solve, got {cy}"
    );
}

#[test]
fn solver_coincident_on_line() {
    // A point off the line constrained to lie on it. The line is the
    // segment from (0,0) to (4,0) (horizontal); the constrained point
    // starts at (1, 2). After solve it should land on y=0.
    let mut s = Sketch {
        plane: SketchPlane::Xy,
        primitives: vec![
            pt("a", 0.0, 0.0),
            pt("b", 4.0, 0.0),
            pt("p", 1.0, 2.0),
            line("l", "a", "b"),
        ],
        constraints: vec![
            SketchConstraint::FixedPoint { point: "a".into() },
            SketchConstraint::FixedPoint { point: "b".into() },
            SketchConstraint::CoincidentOnLine {
                point: "p".into(),
                line: "l".into(),
            },
        ],
    };
    s.solve(&HashMap::new()).expect("on-line solve converges");
    let (_, py) = point_xy(&s, "p");
    assert!(py.abs() < 1e-4, "point y should be 0, got {py}");
}

#[test]
fn solver_equal_length() {
    // Two lines: la from (0,0) to (3,0) (length 3), lb from (0,1) to
    // (1,1) (length 1). Anchor la's endpoints; constrain |la| = |lb|.
    // After solve, |lb| should be 3 (since la is fixed at length 3).
    let mut s = Sketch {
        plane: SketchPlane::Xy,
        primitives: vec![
            pt("a1", 0.0, 0.0),
            pt("a2", 3.0, 0.0),
            pt("b1", 0.0, 1.0),
            pt("b2", 1.0, 1.0),
            line("la", "a1", "a2"),
            line("lb", "b1", "b2"),
        ],
        constraints: vec![
            SketchConstraint::FixedPoint { point: "a1".into() },
            SketchConstraint::FixedPoint { point: "a2".into() },
            SketchConstraint::FixedPoint { point: "b1".into() },
            SketchConstraint::EqualLength {
                line_a: "la".into(),
                line_b: "lb".into(),
            },
        ],
    };
    s.solve(&HashMap::new()).expect("equal-length solve converges");
    let (b1x, b1y) = point_xy(&s, "b1");
    let (b2x, b2y) = point_xy(&s, "b2");
    let len_b = ((b2x - b1x).powi(2) + (b2y - b1y).powi(2)).sqrt();
    assert!(
        (len_b - 3.0).abs() < 1e-3,
        "|lb| should equal |la| = 3, got {len_b}"
    );
}

#[test]
fn solver_equal_radius() {
    // Two circles with literal radii 2 and 2 — equal-radius constraint
    // is trivially satisfied (residual=0 at start).
    let mut s = Sketch {
        plane: SketchPlane::Xy,
        primitives: vec![
            pt("c1", 0.0, 0.0),
            pt("c2", 5.0, 0.0),
            circle("ca", "c1", 2.0, 16),
            circle("cb", "c2", 2.0, 16),
        ],
        constraints: vec![SketchConstraint::EqualRadius {
            circle_a: "ca".into(),
            circle_b: "cb".into(),
        }],
    };
    s.solve(&HashMap::new()).expect("equal-radius solve converges trivially");

    // Disagreeing literal radii → contradictory (no DOFs to fix it).
    let mut s2 = Sketch {
        plane: SketchPlane::Xy,
        primitives: vec![
            pt("c1", 0.0, 0.0),
            pt("c2", 5.0, 0.0),
            circle("ca", "c1", 2.0, 16),
            circle("cb", "c2", 3.0, 16),
        ],
        constraints: vec![SketchConstraint::EqualRadius {
            circle_a: "ca".into(),
            circle_b: "cb".into(),
        }],
    };
    let err = s2.solve(&HashMap::new()).unwrap_err();
    assert!(
        matches!(err, SolverError::Contradictory { .. }),
        "expected Contradictory for unequal radii, got {err:?}"
    );
}

#[test]
fn solver_unknown_circle_returns_error() {
    // EqualRadius referencing a non-existent circle id should fail at
    // setup time with UnknownCircle.
    let mut s = Sketch {
        plane: SketchPlane::Xy,
        primitives: vec![
            pt("c1", 0.0, 0.0),
            circle("ca", "c1", 2.0, 16),
        ],
        constraints: vec![SketchConstraint::EqualRadius {
            circle_a: "ca".into(),
            circle_b: "ghost".into(),
        }],
    };
    let err = s.solve(&HashMap::new()).unwrap_err();
    assert!(
        matches!(err, SolverError::UnknownCircle(ref id) if id == "ghost"),
        "expected UnknownCircle(ghost), got {err:?}"
    );
}

#[test]
fn solver_combined_tangent_and_equal_length() {
    // Compose tangent + equal-length to verify multiple new variants
    // play well together. Two horizontal lines + a circle. Constrain:
    // - both lines equal length
    // - one line tangent to the circle
    // After solve, the geometry should satisfy both.
    let mut s = Sketch {
        plane: SketchPlane::Xy,
        primitives: vec![
            pt("a1", 0.0, 0.0),
            pt("a2", 3.0, 0.0),
            pt("b1", 0.0, 5.0),
            pt("b2", 2.0, 5.0),
            pt("c", 1.5, 3.0),
            line("la", "a1", "a2"),
            line("lb", "b1", "b2"),
            circle("circ", "c", 1.0, 16),
        ],
        constraints: vec![
            SketchConstraint::FixedPoint { point: "a1".into() },
            SketchConstraint::FixedPoint { point: "a2".into() },
            SketchConstraint::FixedPoint { point: "b1".into() },
            SketchConstraint::Horizontal { line: "lb".into() },
            SketchConstraint::EqualLength {
                line_a: "la".into(),
                line_b: "lb".into(),
            },
            SketchConstraint::TangentLineToCircle {
                line: "la".into(),
                circle: "circ".into(),
            },
        ],
    };
    s.solve(&HashMap::new()).expect("combined solve converges");
    // |lb| == |la| == 3.
    let (b1x, b1y) = point_xy(&s, "b1");
    let (b2x, b2y) = point_xy(&s, "b2");
    let len_b = ((b2x - b1x).powi(2) + (b2y - b1y).powi(2)).sqrt();
    assert!((len_b - 3.0).abs() < 1e-3, "|lb| = {len_b}, expected 3");
    // Circle center distance from line `la` (the x-axis segment) is
    // |c.y| = 1 (the radius).
    let (_, cy) = point_xy(&s, "c");
    assert!((cy.abs() - 1.0).abs() < 1e-3, "|c.y| = {}, expected 1.0", cy.abs());
}
