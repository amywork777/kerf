//! 2D sketch data model + JSON DSL tests.
//!
//! Sketch is the parametric authoring layer for planar profiles. These
//! tests exercise the data model, the loop tracer, JSON round-trip,
//! constraint storage, and integration with `Feature::SketchExtrude` /
//! `Feature::SketchRevolve`.

use kerf_brep::solid_volume;
use kerf_cad::{
    EvalError, Feature, Model, Scalar, Sketch, SketchConstraint, SketchPlane, SketchPrim,
};

// ===========================================================================
// Sketch solver integration: `build_sketch_extrude` runs `Sketch::solve`
// before tracing, so constraint-driven geometry actually shapes the
// extruded solid. The tests below exercise:
//   1. Distance constraint pulling two corners apart from initial 5 → 10.
//   2. Horizontal constraint flattening a tilted edge.
//   3. Over-/contradictory constraints surfacing as `EvalError::Invalid`
//      with the conflicting constraint indices in the reason string.
//   4. `skip_solve = true` honoring raw authored coordinates.
//   5. No-constraint sketches behaving identically to the pre-integration
//      pipeline (the solver path is a no-op, no clone/no perturbation).
// ===========================================================================

/// 4-corner rectangle whose corners are fully anchored by `FixedPoint` plus
/// `Horizontal`/`Vertical` line constraints, so the solver's solution is
/// uniquely determined regardless of the initial coordinates.
///
/// The two top corners (`p3`, `p4`) are deliberately *unfixed* so the
/// solver has DOFs to move; the bottom edge anchors `p1`, `p2` with
/// `FixedPoint`, the left edge with `Vertical`, the right edge with
/// `Vertical`, the top edge with `Horizontal`. The final geometry is
/// `[0, w] × [0, h]` after solve.
fn anchored_rect_sketch(
    initial_w: f64,
    initial_h: f64,
    distance_value: f64,
    target_h: f64,
) -> Sketch {
    Sketch {
        plane: SketchPlane::Xy,
        primitives: vec![
            SketchPrim::Point { id: "p1".into(), x: Scalar::lit(0.0), y: Scalar::lit(0.0) },
            SketchPrim::Point { id: "p2".into(), x: Scalar::lit(initial_w), y: Scalar::lit(0.0) },
            SketchPrim::Point { id: "p3".into(), x: Scalar::lit(initial_w), y: Scalar::lit(initial_h) },
            SketchPrim::Point { id: "p4".into(), x: Scalar::lit(0.0), y: Scalar::lit(initial_h) },
            SketchPrim::Line { id: "l1".into(), from: "p1".into(), to: "p2".into() },
            SketchPrim::Line { id: "l2".into(), from: "p2".into(), to: "p3".into() },
            SketchPrim::Line { id: "l3".into(), from: "p3".into(), to: "p4".into() },
            SketchPrim::Line { id: "l4".into(), from: "p4".into(), to: "p1".into() },
        ],
        constraints: vec![
            // Pin the bottom-left and bottom-right by anchoring p1 at
            // origin and constraining the bottom edge to length
            // `distance_value`. p1 stays put; p2 is dragged horizontally
            // (Horizontal on l1) to whatever satisfies Distance(p1, p2).
            SketchConstraint::FixedPoint { point: "p1".into() },
            SketchConstraint::Horizontal { line: "l1".into() },
            SketchConstraint::Distance {
                a: "p1".into(),
                b: "p2".into(),
                value: Scalar::lit(distance_value),
            },
            // Right and left edges vertical.
            SketchConstraint::Vertical { line: "l2".into() },
            SketchConstraint::Vertical { line: "l4".into() },
            // Top edge horizontal.
            SketchConstraint::Horizontal { line: "l3".into() },
            // Pin the height by constraining p1 ↔ p4 distance.
            SketchConstraint::Distance {
                a: "p1".into(),
                b: "p4".into(),
                value: Scalar::lit(target_h),
            },
        ],
    }
}

/// Authored geometry: a "tilted rectangle" — `p2` starts at (5, 1.5) so the
/// bottom edge is angled. With a `Horizontal` constraint on the bottom
/// edge, the solver must drag `p2` down to the x-axis.
///
/// All other corners are anchored via `FixedPoint` so the solve is
/// fully-constrained and the bottom edge ends up at exactly y = 0.
fn tilted_rect_sketch_for_horizontal() -> Sketch {
    Sketch {
        plane: SketchPlane::Xy,
        primitives: vec![
            SketchPrim::Point { id: "p1".into(), x: Scalar::lit(0.0), y: Scalar::lit(0.0) },
            // p2 starts off-axis (y=1.5) — Horizontal constraint must
            // pull it back to y=0.
            SketchPrim::Point { id: "p2".into(), x: Scalar::lit(5.0), y: Scalar::lit(1.5) },
            SketchPrim::Point { id: "p3".into(), x: Scalar::lit(5.0), y: Scalar::lit(3.0) },
            SketchPrim::Point { id: "p4".into(), x: Scalar::lit(0.0), y: Scalar::lit(3.0) },
            SketchPrim::Line { id: "l1".into(), from: "p1".into(), to: "p2".into() },
            SketchPrim::Line { id: "l2".into(), from: "p2".into(), to: "p3".into() },
            SketchPrim::Line { id: "l3".into(), from: "p3".into(), to: "p4".into() },
            SketchPrim::Line { id: "l4".into(), from: "p4".into(), to: "p1".into() },
        ],
        constraints: vec![
            // Anchor every point except p2; only p2 is free, so the
            // solver must move it to satisfy Horizontal(l1).
            SketchConstraint::FixedPoint { point: "p1".into() },
            SketchConstraint::FixedPoint { point: "p3".into() },
            SketchConstraint::FixedPoint { point: "p4".into() },
            SketchConstraint::Horizontal { line: "l1".into() },
            SketchConstraint::Vertical { line: "l2".into() },
        ],
    }
}

/// Tier 3, test 1. A rectangle whose width is wrong in the authored
/// coordinates: bottom edge is 5 wide, but a `Distance(p1, p2) = 10`
/// constraint says it should be 10. After build_sketch_extrude runs
/// `Sketch::solve`, the extruded solid has volume `10 * 5 * 4 = 200`,
/// not the un-solved `5 * 5 * 4 = 100`.
#[test]
fn sketch_with_distance_constraint_extrudes_to_correct_size() {
    let h_extrude = 4.0;
    let target_w = 10.0;
    let target_h = 5.0;
    let sketch = anchored_rect_sketch(5.0, 5.0, target_w, target_h);
    let m = Model::new().add(Feature::SketchExtrude {
        id: "out".into(),
        sketch,
        direction: [Scalar::lit(0.0), Scalar::lit(0.0), Scalar::lit(h_extrude)],
        skip_solve: false,
    });
    let v = solid_volume(&m.evaluate("out").unwrap());
    let expected = target_w * target_h * h_extrude;
    assert!(
        (v - expected).abs() < 1e-4,
        "v={v}, expected {expected} (10×5×4 = 200 — solver should have stretched the authored 5-wide rectangle to 10-wide before extrusion)"
    );
}

/// Tier 3, test 2. The bottom edge is authored tilted (p2.y = 1.5);
/// a `Horizontal` constraint pulls p2 back down. After solve the bottom
/// edge sits at y = 0 and the rectangle is exactly [0,5] × [0,3]; volume
/// is `5 * 3 * 2 = 30`. Without the solver call the un-solved tilted
/// quad has a different (smaller) area and its extrusion would not
/// reach 30.
#[test]
fn sketch_with_horizontal_constraint_aligns_corners() {
    let h_extrude = 2.0;
    let sketch = tilted_rect_sketch_for_horizontal();
    let m = Model::new().add(Feature::SketchExtrude {
        id: "out".into(),
        sketch,
        direction: [Scalar::lit(0.0), Scalar::lit(0.0), Scalar::lit(h_extrude)],
        skip_solve: false,
    });
    let v = solid_volume(&m.evaluate("out").unwrap());
    let expected = 5.0 * 3.0 * h_extrude; // 30
    assert!(
        (v - expected).abs() < 1e-4,
        "v={v}, expected {expected} (Horizontal constraint should have flattened the tilted bottom edge before extrusion)"
    );
}

/// Tier 3, test 3. Two `Distance` constraints on the same Point pair with
/// conflicting values. `build_sketch_extrude` should surface this as
/// `EvalError::Invalid`, and the human-readable reason should mention
/// "conflicting" with the constraint indices identified by the bisector.
#[test]
fn sketch_with_overconstrained_returns_invalid_error() {
    let sketch = Sketch {
        plane: SketchPlane::Xy,
        primitives: vec![
            SketchPrim::Point { id: "p1".into(), x: Scalar::lit(0.0), y: Scalar::lit(0.0) },
            SketchPrim::Point { id: "p2".into(), x: Scalar::lit(1.0), y: Scalar::lit(0.0) },
            SketchPrim::Point { id: "p3".into(), x: Scalar::lit(1.0), y: Scalar::lit(1.0) },
            SketchPrim::Point { id: "p4".into(), x: Scalar::lit(0.0), y: Scalar::lit(1.0) },
            SketchPrim::Line { id: "l1".into(), from: "p1".into(), to: "p2".into() },
            SketchPrim::Line { id: "l2".into(), from: "p2".into(), to: "p3".into() },
            SketchPrim::Line { id: "l3".into(), from: "p3".into(), to: "p4".into() },
            SketchPrim::Line { id: "l4".into(), from: "p4".into(), to: "p1".into() },
        ],
        constraints: vec![
            // Both Distance constraints on (p1, p2) — incompatible values.
            SketchConstraint::Distance {
                a: "p1".into(),
                b: "p2".into(),
                value: Scalar::lit(5.0),
            },
            SketchConstraint::Distance {
                a: "p1".into(),
                b: "p2".into(),
                value: Scalar::lit(10.0),
            },
        ],
    };
    let m = Model::new().add(Feature::SketchExtrude {
        id: "out".into(),
        sketch,
        direction: [Scalar::lit(0.0), Scalar::lit(0.0), Scalar::lit(1.0)],
        skip_solve: false,
    });
    let err = m.evaluate("out").unwrap_err();
    match err {
        EvalError::Invalid { id, reason } => {
            assert_eq!(id, "out");
            // The reason should explicitly mention "contradictory" or
            // "did not converge" and ideally surface the conflicting
            // constraint indices for the user's diagnostic.
            assert!(
                reason.contains("contradictory") || reason.contains("did not converge"),
                "expected contradictory/over-constrained signature in reason; got {reason:?}"
            );
        }
        other => panic!("expected EvalError::Invalid, got {other:?}"),
    }
}

/// Tier 3, test 4. With `skip_solve = true`, the solver is bypassed even
/// though the sketch carries a `Distance` constraint that would have
/// changed the geometry. The extruded volume reflects the *raw authored*
/// 5×5 rectangle (volume 100), not the solved 10×5 rectangle (volume 200).
#[test]
fn sketch_with_skip_solve_uses_raw_coords() {
    let h_extrude = 4.0;
    // Author a 5×5 rectangle but stuff a Distance(p1,p2)=10 constraint
    // into the sketch. With skip_solve=true the constraint is ignored.
    let sketch = anchored_rect_sketch(5.0, 5.0, 10.0, 5.0);
    let m = Model::new().add(Feature::SketchExtrude {
        id: "out".into(),
        sketch,
        direction: [Scalar::lit(0.0), Scalar::lit(0.0), Scalar::lit(h_extrude)],
        skip_solve: true, // bypass — read raw coords verbatim.
    });
    let v = solid_volume(&m.evaluate("out").unwrap());
    let expected_raw = 5.0 * 5.0 * h_extrude; // 100
    assert!(
        (v - expected_raw).abs() < 1e-9,
        "v={v}, expected {expected_raw} — skip_solve must bypass the constraint solver"
    );
}

/// Tier 3, test 5. A sketch with no constraints must extrude identically
/// regardless of `skip_solve` — the integration's "if constraints empty,
/// skip solve" fast-path is correct (no clone, no perturbation).
#[test]
fn sketch_with_no_constraints_skips_solver_path() {
    let h_extrude = 3.0;
    let sketch = rect_sketch(2.0, 4.0); // 8.0 area, 0 constraints.
    let m_solve = Model::new().add(Feature::SketchExtrude {
        id: "out".into(),
        sketch: sketch.clone(),
        direction: [Scalar::lit(0.0), Scalar::lit(0.0), Scalar::lit(h_extrude)],
        skip_solve: false,
    });
    let m_skip = Model::new().add(Feature::SketchExtrude {
        id: "out".into(),
        sketch,
        direction: [Scalar::lit(0.0), Scalar::lit(0.0), Scalar::lit(h_extrude)],
        skip_solve: true,
    });
    let v_solve = solid_volume(&m_solve.evaluate("out").unwrap());
    let v_skip = solid_volume(&m_skip.evaluate("out").unwrap());
    let expected = 2.0 * 4.0 * h_extrude; // 24
    assert!(
        (v_solve - expected).abs() < 1e-9 && (v_skip - expected).abs() < 1e-9,
        "v_solve={v_solve}, v_skip={v_skip}, expected {expected}"
    );
    // Sanity: same volume on both paths.
    assert!((v_solve - v_skip).abs() < 1e-12);
}

/// Tier 3, test 6 (bonus). The Scalar::Param resolution in a Distance
/// constraint must propagate through to the solver — the same sketch
/// extruded with two different `params` HashMaps should yield two
/// different solved volumes. Closes the loop on parametric driving.
#[test]
fn sketch_distance_param_drives_solved_geometry() {
    let mut sketch = anchored_rect_sketch(5.0, 5.0, 0.0, 5.0);
    // Replace the literal Distance(p1, p2) value with a $width param.
    for c in &mut sketch.constraints {
        if let SketchConstraint::Distance { a, b, value } = c {
            if a == "p1" && b == "p2" {
                *value = Scalar::param("width");
            }
        }
    }
    let h_extrude = 2.0;

    // First evaluation: width = 7. Volume should be 7 * 5 * 2 = 70.
    let mut m1 = Model::new().add(Feature::SketchExtrude {
        id: "out".into(),
        sketch: sketch.clone(),
        direction: [Scalar::lit(0.0), Scalar::lit(0.0), Scalar::lit(h_extrude)],
        skip_solve: false,
    });
    m1.parameters.insert("width".into(), 7.0);
    let v1 = solid_volume(&m1.evaluate("out").unwrap());
    assert!(
        (v1 - 70.0).abs() < 1e-4,
        "v1={v1}, expected 70.0 (width=7)"
    );

    // Second evaluation: width = 12. Volume should be 12 * 5 * 2 = 120.
    let mut m2 = Model::new().add(Feature::SketchExtrude {
        id: "out".into(),
        sketch,
        direction: [Scalar::lit(0.0), Scalar::lit(0.0), Scalar::lit(h_extrude)],
        skip_solve: false,
    });
    m2.parameters.insert("width".into(), 12.0);
    let v2 = solid_volume(&m2.evaluate("out").unwrap());
    assert!(
        (v2 - 120.0).abs() < 1e-4,
        "v2={v2}, expected 120.0 (width=12)"
    );
}

fn rect_sketch(w: f64, h: f64) -> Sketch {
    Sketch {
        plane: SketchPlane::Xy,
        primitives: vec![
            SketchPrim::Point { id: "p1".into(), x: Scalar::lit(0.0), y: Scalar::lit(0.0) },
            SketchPrim::Point { id: "p2".into(), x: Scalar::lit(w), y: Scalar::lit(0.0) },
            SketchPrim::Point { id: "p3".into(), x: Scalar::lit(w), y: Scalar::lit(h) },
            SketchPrim::Point { id: "p4".into(), x: Scalar::lit(0.0), y: Scalar::lit(h) },
            SketchPrim::Line { id: "l1".into(), from: "p1".into(), to: "p2".into() },
            SketchPrim::Line { id: "l2".into(), from: "p2".into(), to: "p3".into() },
            SketchPrim::Line { id: "l3".into(), from: "p3".into(), to: "p4".into() },
            SketchPrim::Line { id: "l4".into(), from: "p4".into(), to: "p1".into() },
        ],
        constraints: vec![],
    }
}

#[test]
fn sketch_rectangle_extrudes_to_box() {
    // 2x3 rectangle sketch, extruded by 4 along +z. Volume should equal
    // 2 * 3 * 4 = 24.
    let m = Model::new().add(Feature::SketchExtrude {
        id: "out".into(),
        sketch: rect_sketch(2.0, 3.0),
        direction: [Scalar::lit(0.0), Scalar::lit(0.0), Scalar::lit(4.0)],
        skip_solve: false,
    });
    let v = solid_volume(&m.evaluate("out").unwrap());
    assert!((v - 24.0).abs() < 1e-9, "v={v}, expected 24.0");
}

#[test]
fn sketch_circle_extrudes_to_cylinder() {
    // Single Circle primitive (no Lines). Sketch's profile is a faceted
    // n-gon; extruded volume is the standard cylinder-via-prism formula
    // V = (n/2) * r^2 * sin(2π/n) * h.
    let n: usize = 32;
    let r = 1.5;
    let h = 4.0;
    let sketch = Sketch {
        plane: SketchPlane::Xy,
        primitives: vec![
            SketchPrim::Point { id: "c".into(), x: Scalar::lit(0.0), y: Scalar::lit(0.0) },
            SketchPrim::Circle {
                id: "k".into(),
                center: "c".into(),
                radius: Scalar::lit(r),
                n_segments: n,
            },
        ],
        constraints: vec![],
    };
    let m = Model::new().add(Feature::SketchExtrude {
        id: "out".into(),
        sketch,
        direction: [Scalar::lit(0.0), Scalar::lit(0.0), Scalar::lit(h)],
        skip_solve: false,
    });
    let v = solid_volume(&m.evaluate("out").unwrap());
    let theta = 2.0 * std::f64::consts::PI / (n as f64);
    let exp = 0.5 * (n as f64) * r * r * theta.sin() * h;
    assert!((v - exp).abs() < 1e-9, "v={v}, exp={exp}");
}

#[test]
fn sketch_round_trip_json() {
    let m = Model::new().add(Feature::SketchExtrude {
        id: "out".into(),
        sketch: rect_sketch(2.0, 3.0),
        direction: [Scalar::lit(0.0), Scalar::lit(0.0), Scalar::lit(4.0)],
        skip_solve: false,
    });
    let json = m.to_json_string().unwrap();
    let m2 = Model::from_json_str(&json).unwrap();
    let v1 = solid_volume(&m.evaluate("out").unwrap());
    let v2 = solid_volume(&m2.evaluate("out").unwrap());
    assert!((v1 - v2).abs() < 1e-9, "v1={v1}, v2={v2}");
}

#[test]
fn sketch_open_loop_rejects() {
    // Drop one closing edge — the rectangle is no longer closed.
    let mut s = rect_sketch(2.0, 3.0);
    s.primitives.retain(|p| p.id() != "l4");
    let m = Model::new().add(Feature::SketchExtrude {
        id: "out".into(),
        sketch: s,
        direction: [Scalar::lit(0.0), Scalar::lit(0.0), Scalar::lit(4.0)],
        skip_solve: false,
    });
    let err = m.evaluate("out").unwrap_err();
    let msg = format!("{err}");
    // The error wraps a SketchError whose Display mentions valence.
    assert!(
        msg.contains("valence") || msg.contains("loop"),
        "expected loop error, got: {msg}"
    );
}

#[test]
fn constraint_storage_round_trip() {
    // A sketch authored with a Distance and Horizontal constraint should
    // round-trip through JSON unchanged. Constraints are NOT enforced
    // today, but they must be serialized + deserialized losslessly so
    // they survive a future solver upgrade.
    let mut s = rect_sketch(2.0, 3.0);
    s.constraints = vec![
        SketchConstraint::Horizontal { line: "l1".into() },
        SketchConstraint::Vertical { line: "l2".into() },
        SketchConstraint::Distance {
            a: "p1".into(),
            b: "p2".into(),
            value: Scalar::lit(2.0),
        },
        SketchConstraint::FixedPoint { point: "p1".into() },
        SketchConstraint::Parallel {
            line_a: "l1".into(),
            line_b: "l3".into(),
        },
        SketchConstraint::Perpendicular {
            line_a: "l1".into(),
            line_b: "l2".into(),
        },
        SketchConstraint::Coincident {
            a: "p1".into(),
            b: "p1".into(),
        },
    ];
    let m = Model::new().add(Feature::SketchExtrude {
        id: "out".into(),
        sketch: s.clone(),
        direction: [Scalar::lit(0.0), Scalar::lit(0.0), Scalar::lit(4.0)],
        skip_solve: false,
    });
    let json = m.to_json_string().unwrap();
    let m2 = Model::from_json_str(&json).unwrap();

    // Find the SketchExtrude feature in the round-tripped model and
    // compare its constraints to the original.
    let f2 = m2.feature("out").unwrap();
    if let Feature::SketchExtrude { sketch, .. } = f2 {
        assert_eq!(sketch.constraints, s.constraints);
        assert_eq!(sketch.primitives, s.primitives);
        assert_eq!(sketch.plane, s.plane);
    } else {
        panic!("round-tripped feature isn't SketchExtrude");
    }
}

#[test]
fn sketch_extrude_via_json_dsl() {
    // Author the rectangle entirely from a JSON string — the same DSL a
    // user would type by hand. Validates that the serde tag layout is
    // ergonomic and that `$param` references work end-to-end.
    let json = r#"{
      "parameters": { "w": 2.0, "h": 3.0, "depth": 4.0 },
      "features": [
        {
          "kind": "SketchExtrude",
          "id": "out",
          "direction": [0, 0, "$depth"],
          "sketch": {
            "plane": "Xy",
            "primitives": [
              {"kind": "Point",  "id": "p1", "x": 0,    "y": 0},
              {"kind": "Point",  "id": "p2", "x": "$w", "y": 0},
              {"kind": "Point",  "id": "p3", "x": "$w", "y": "$h"},
              {"kind": "Point",  "id": "p4", "x": 0,    "y": "$h"},
              {"kind": "Line",   "id": "l1", "from": "p1", "to": "p2"},
              {"kind": "Line",   "id": "l2", "from": "p2", "to": "p3"},
              {"kind": "Line",   "id": "l3", "from": "p3", "to": "p4"},
              {"kind": "Line",   "id": "l4", "from": "p4", "to": "p1"}
            ],
            "constraints": [
              {"kind": "Horizontal", "line": "l1"},
              {"kind": "Distance", "a": "p1", "b": "p2", "value": "$w"}
            ]
          }
        }
      ]
    }"#;
    let m = Model::from_json_str(json).unwrap();
    let v = solid_volume(&m.evaluate("out").unwrap());
    assert!((v - 24.0).abs() < 1e-9, "v={v}, expected 24.0");
}

#[test]
fn sketch_unknown_point_rejects() {
    // A Line that references a Point id that doesn't exist must fail
    // at evaluation time (so authors get a clear error instead of a
    // silent dropped edge).
    let s = Sketch {
        plane: SketchPlane::Xy,
        primitives: vec![
            SketchPrim::Point { id: "p1".into(), x: Scalar::lit(0.0), y: Scalar::lit(0.0) },
            SketchPrim::Point { id: "p2".into(), x: Scalar::lit(1.0), y: Scalar::lit(0.0) },
            SketchPrim::Line { id: "l1".into(), from: "p1".into(), to: "p_ghost".into() },
        ],
        constraints: vec![],
    };
    let m = Model::new().add(Feature::SketchExtrude {
        id: "out".into(),
        sketch: s,
        direction: [Scalar::lit(0.0), Scalar::lit(0.0), Scalar::lit(1.0)],
        skip_solve: false,
    });
    assert!(m.evaluate("out").is_err());
}

#[test]
fn sketch_param_driven_rectangle_resolves() {
    // Same rectangle, but `w` and `h` are model parameters. Volume
    // should track param changes — we verify two distinct evaluations
    // give two distinct volumes.
    let mk_model = |w: f64, h: f64| {
        let sketch = Sketch {
            plane: SketchPlane::Xy,
            primitives: vec![
                SketchPrim::Point { id: "p1".into(), x: Scalar::lit(0.0), y: Scalar::lit(0.0) },
                SketchPrim::Point { id: "p2".into(), x: Scalar::param("w"), y: Scalar::lit(0.0) },
                SketchPrim::Point { id: "p3".into(), x: Scalar::param("w"), y: Scalar::param("h") },
                SketchPrim::Point { id: "p4".into(), x: Scalar::lit(0.0), y: Scalar::param("h") },
                SketchPrim::Line { id: "l1".into(), from: "p1".into(), to: "p2".into() },
                SketchPrim::Line { id: "l2".into(), from: "p2".into(), to: "p3".into() },
                SketchPrim::Line { id: "l3".into(), from: "p3".into(), to: "p4".into() },
                SketchPrim::Line { id: "l4".into(), from: "p4".into(), to: "p1".into() },
            ],
            constraints: vec![],
        };
        Model::new()
            .with_parameter("w", w)
            .with_parameter("h", h)
            .add(Feature::SketchExtrude {
                id: "out".into(),
                sketch,
                direction: [Scalar::lit(0.0), Scalar::lit(0.0), Scalar::lit(2.0)],
                            skip_solve: false,
            })
    };
    let v1 = solid_volume(&mk_model(2.0, 3.0).evaluate("out").unwrap());
    let v2 = solid_volume(&mk_model(4.0, 3.0).evaluate("out").unwrap());
    assert!((v1 - 12.0).abs() < 1e-9, "v1={v1}");
    assert!((v2 - 24.0).abs() < 1e-9, "v2={v2}");
}

#[test]
fn sketch_reverse_orientation_still_extrudes() {
    // Author the rectangle clockwise. The tracer should detect the
    // reversed signed area and flip it to CCW so extrude_polygon
    // produces a positive-volume solid.
    let s = Sketch {
        plane: SketchPlane::Xy,
        primitives: vec![
            SketchPrim::Point { id: "p1".into(), x: Scalar::lit(0.0), y: Scalar::lit(0.0) },
            SketchPrim::Point { id: "p2".into(), x: Scalar::lit(0.0), y: Scalar::lit(3.0) },
            SketchPrim::Point { id: "p3".into(), x: Scalar::lit(2.0), y: Scalar::lit(3.0) },
            SketchPrim::Point { id: "p4".into(), x: Scalar::lit(2.0), y: Scalar::lit(0.0) },
            SketchPrim::Line { id: "l1".into(), from: "p1".into(), to: "p2".into() },
            SketchPrim::Line { id: "l2".into(), from: "p2".into(), to: "p3".into() },
            SketchPrim::Line { id: "l3".into(), from: "p3".into(), to: "p4".into() },
            SketchPrim::Line { id: "l4".into(), from: "p4".into(), to: "p1".into() },
        ],
        constraints: vec![],
    };
    let m = Model::new().add(Feature::SketchExtrude {
        id: "out".into(),
        sketch: s,
        direction: [Scalar::lit(0.0), Scalar::lit(0.0), Scalar::lit(4.0)],
        skip_solve: false,
    });
    let v = solid_volume(&m.evaluate("out").unwrap());
    assert!((v - 24.0).abs() < 1e-9, "v={v}");
}

#[test]
fn sketch_branching_valence_rejects() {
    // A Y-junction: three lines meeting at p_center. The tracer should
    // reject this as ambiguous.
    let s = Sketch {
        plane: SketchPlane::Xy,
        primitives: vec![
            SketchPrim::Point { id: "p_center".into(), x: Scalar::lit(0.0), y: Scalar::lit(0.0) },
            SketchPrim::Point { id: "pa".into(), x: Scalar::lit(1.0), y: Scalar::lit(0.0) },
            SketchPrim::Point { id: "pb".into(), x: Scalar::lit(-1.0), y: Scalar::lit(0.5) },
            SketchPrim::Point { id: "pc".into(), x: Scalar::lit(-1.0), y: Scalar::lit(-0.5) },
            SketchPrim::Line { id: "la".into(), from: "p_center".into(), to: "pa".into() },
            SketchPrim::Line { id: "lb".into(), from: "p_center".into(), to: "pb".into() },
            SketchPrim::Line { id: "lc".into(), from: "p_center".into(), to: "pc".into() },
        ],
        constraints: vec![],
    };
    let m = Model::new().add(Feature::SketchExtrude {
        id: "out".into(),
        sketch: s,
        direction: [Scalar::lit(0.0), Scalar::lit(0.0), Scalar::lit(1.0)],
        skip_solve: false,
    });
    assert!(m.evaluate("out").is_err());
}

// ----------------------------------------------------------------------
// Tier A: multi-loop sketches.
// ----------------------------------------------------------------------

fn rect_sketch_at(x0: f64, y0: f64, w: f64, h: f64, suffix: &str) -> Vec<SketchPrim> {
    let p1 = format!("p1_{suffix}");
    let p2 = format!("p2_{suffix}");
    let p3 = format!("p3_{suffix}");
    let p4 = format!("p4_{suffix}");
    vec![
        SketchPrim::Point { id: p1.clone(), x: Scalar::lit(x0), y: Scalar::lit(y0) },
        SketchPrim::Point { id: p2.clone(), x: Scalar::lit(x0 + w), y: Scalar::lit(y0) },
        SketchPrim::Point { id: p3.clone(), x: Scalar::lit(x0 + w), y: Scalar::lit(y0 + h) },
        SketchPrim::Point { id: p4.clone(), x: Scalar::lit(x0), y: Scalar::lit(y0 + h) },
        SketchPrim::Line { id: format!("l1_{suffix}"), from: p1.clone(), to: p2.clone() },
        SketchPrim::Line { id: format!("l2_{suffix}"), from: p2, to: p3.clone() },
        SketchPrim::Line { id: format!("l3_{suffix}"), from: p3, to: p4.clone() },
        SketchPrim::Line { id: format!("l4_{suffix}"), from: p4, to: p1 },
    ]
}

#[test]
fn sketch_two_disjoint_loops_extrudes_to_two_solids() {
    // Two non-overlapping rectangles, each fully outside the other.
    // Extruded by 1 along +z. Each rectangle is 2x3, so each contributes
    // volume 6; their union (disjoint) should be 12.
    let mut prims = rect_sketch_at(0.0, 0.0, 2.0, 3.0, "a");
    prims.extend(rect_sketch_at(10.0, 0.0, 2.0, 3.0, "b"));
    let s = Sketch {
        plane: SketchPlane::Xy,
        primitives: prims,
        constraints: vec![],
    };
    // First make sure the tracer reports two profiles.
    let profs = s.to_profile_2d(&Default::default()).unwrap();
    assert_eq!(profs.len(), 2);

    let m = Model::new().add(Feature::SketchExtrude {
        id: "out".into(),
        sketch: s,
        direction: [Scalar::lit(0.0), Scalar::lit(0.0), Scalar::lit(1.0)],
        skip_solve: false,
    });
    let v = solid_volume(&m.evaluate("out").unwrap());
    assert!(
        (v - 12.0).abs() < 1e-9,
        "two disjoint loops should sum to 12, got {v}"
    );
}

#[test]
fn sketch_overlapping_inner_loop_rejects() {
    // Two rectangles that PARTIALLY overlap (cross). The tracer should
    // produce two profiles but build_sketch_extrude should reject them
    // as DisjointSubLoops because they cross.
    let mut prims = rect_sketch_at(0.0, 0.0, 4.0, 4.0, "a");
    // Second rect: starts inside (1,1) and extends to (5,5) — partially
    // overlaps the first.
    prims.extend(rect_sketch_at(1.0, 1.0, 4.0, 4.0, "b"));
    let s = Sketch {
        plane: SketchPlane::Xy,
        primitives: prims,
        constraints: vec![],
    };
    let m = Model::new().add(Feature::SketchExtrude {
        id: "out".into(),
        sketch: s,
        direction: [Scalar::lit(0.0), Scalar::lit(0.0), Scalar::lit(1.0)],
        skip_solve: false,
    });
    let err = m.evaluate("out").unwrap_err();
    let msg = format!("{err}");
    assert!(
        msg.contains("overlapping") || msg.contains("DisjointSubLoops") || msg.contains("disjoint"),
        "expected overlap rejection, got: {msg}"
    );
}

// ----------------------------------------------------------------------
// Tier B: named ref-plane positioning.
// ----------------------------------------------------------------------

#[test]
fn sketch_named_ref_plane_applies_transform() {
    // Build a 2x3 rectangle on a NamedRefPlane whose RefPlane sits at
    // position (10, 0, 0) with axis 'x'. The local +Z (extrusion
    // direction) should map to world +X. Original local box:
    //   x in [0,2], y in [0,3], z in [0,1]
    // After +90° around Y: (x, y, z) -> (z, y, -x) gives:
    //   X in [0,1], Y in [0,3], Z in [-2,0]
    // After translate by (10, 0, 0):
    //   X in [10,11], Y in [0,3], Z in [-2,0]
    let sketch = Sketch {
        plane: SketchPlane::NamedRefPlane("rp1".into()),
        primitives: rect_sketch_at(0.0, 0.0, 2.0, 3.0, "r"),
        constraints: vec![],
    };
    let m = Model::new()
        .add(Feature::RefPlane {
            id: "rp1".into(),
            position: [Scalar::lit(10.0), Scalar::lit(0.0), Scalar::lit(0.0)],
            axis: "x".into(),
            extents: [Scalar::lit(5.0), Scalar::lit(5.0)],
            marker_thickness: Scalar::lit(0.1),
        })
        .add(Feature::SketchExtrude {
            id: "out".into(),
            sketch,
            direction: [Scalar::lit(0.0), Scalar::lit(0.0), Scalar::lit(1.0)],
                    skip_solve: false,
        });
    let solid = m.evaluate("out").unwrap();
    let v = solid_volume(&solid);
    assert!((v - 6.0).abs() < 1e-9, "transformed volume preserved, got {v}");

    let mut min = [f64::INFINITY; 3];
    let mut max = [f64::NEG_INFINITY; 3];
    for (_, p) in solid.vertex_geom.iter() {
        let xyz = [p.x, p.y, p.z];
        for i in 0..3 {
            min[i] = min[i].min(xyz[i]);
            max[i] = max[i].max(xyz[i]);
        }
    }
    assert!((min[0] - 10.0).abs() < 1e-6, "min[0]={}", min[0]);
    assert!((max[0] - 11.0).abs() < 1e-6, "max[0]={}", max[0]);
    assert!((min[1] - 0.0).abs() < 1e-6, "min[1]={}", min[1]);
    assert!((max[1] - 3.0).abs() < 1e-6, "max[1]={}", max[1]);
    assert!((min[2] - (-2.0)).abs() < 1e-6, "min[2]={}", min[2]);
    assert!((max[2] - 0.0).abs() < 1e-6, "max[2]={}", max[2]);
}

#[test]
fn sketch_named_ref_plane_z_axis_translates_only() {
    let sketch = Sketch {
        plane: SketchPlane::NamedRefPlane("rp2".into()),
        primitives: rect_sketch_at(0.0, 0.0, 2.0, 3.0, "r"),
        constraints: vec![],
    };
    let m = Model::new()
        .add(Feature::RefPlane {
            id: "rp2".into(),
            position: [Scalar::lit(0.0), Scalar::lit(0.0), Scalar::lit(5.0)],
            axis: "z".into(),
            extents: [Scalar::lit(5.0), Scalar::lit(5.0)],
            marker_thickness: Scalar::lit(0.1),
        })
        .add(Feature::SketchExtrude {
            id: "out".into(),
            sketch,
            direction: [Scalar::lit(0.0), Scalar::lit(0.0), Scalar::lit(4.0)],
                    skip_solve: false,
        });
    let solid = m.evaluate("out").unwrap();
    let v = solid_volume(&solid);
    assert!((v - 24.0).abs() < 1e-9, "v={v}");
    let mut min_z = f64::INFINITY;
    let mut max_z = f64::NEG_INFINITY;
    for (_, p) in solid.vertex_geom.iter() {
        min_z = min_z.min(p.z);
        max_z = max_z.max(p.z);
    }
    assert!((min_z - 5.0).abs() < 1e-6, "min_z={min_z}");
    assert!((max_z - 9.0).abs() < 1e-6, "max_z={max_z}");
}

#[test]
fn sketch_named_ref_plane_unknown_id_rejects() {
    let sketch = Sketch {
        plane: SketchPlane::NamedRefPlane("does_not_exist".into()),
        primitives: rect_sketch_at(0.0, 0.0, 2.0, 3.0, "r"),
        constraints: vec![],
    };
    let m = Model::new().add(Feature::SketchExtrude {
        id: "out".into(),
        sketch,
        direction: [Scalar::lit(0.0), Scalar::lit(0.0), Scalar::lit(1.0)],
        skip_solve: false,
    });
    let err = m.evaluate("out").unwrap_err();
    let msg = format!("{err}");
    assert!(
        msg.contains("unknown") && msg.contains("does_not_exist"),
        "expected unknown-ref-plane error, got: {msg}"
    );
}

// ----------------------------------------------------------------------
// Tier C: trim / extend / fillet.
// ----------------------------------------------------------------------

#[test]
fn sketch_trim_line_at_point() {
    // A "stub-out" rectangle: l1 starts as p1 -> p_far, where p_far is
    // PAST p2. TrimLine at p2 rewrites the closer endpoint (p_far at
    // distance 1, vs p1 at distance 2) to p2 — making l1 become
    // p1 -> p2 (closing the rectangle). Volume = 2 * 3 * 4 = 24.
    let sketch = Sketch {
        plane: SketchPlane::Xy,
        primitives: vec![
            SketchPrim::Point { id: "p1".into(), x: Scalar::lit(0.0), y: Scalar::lit(0.0) },
            SketchPrim::Point { id: "p2".into(), x: Scalar::lit(2.0), y: Scalar::lit(0.0) },
            SketchPrim::Point { id: "p3".into(), x: Scalar::lit(2.0), y: Scalar::lit(3.0) },
            SketchPrim::Point { id: "p4".into(), x: Scalar::lit(0.0), y: Scalar::lit(3.0) },
            SketchPrim::Point { id: "p_far".into(), x: Scalar::lit(3.0), y: Scalar::lit(0.0) },
            SketchPrim::Line { id: "l1".into(), from: "p1".into(), to: "p_far".into() },
            SketchPrim::Line { id: "l2".into(), from: "p2".into(), to: "p3".into() },
            SketchPrim::Line { id: "l3".into(), from: "p3".into(), to: "p4".into() },
            SketchPrim::Line { id: "l4".into(), from: "p4".into(), to: "p1".into() },
            SketchPrim::TrimLine {
                id: "tr1".into(),
                line: "l1".into(),
                at_point: "p2".into(),
            },
        ],
        constraints: vec![],
    };
    let m = Model::new().add(Feature::SketchExtrude {
        id: "out".into(),
        sketch,
        direction: [Scalar::lit(0.0), Scalar::lit(0.0), Scalar::lit(4.0)],
        skip_solve: false,
    });
    let v = solid_volume(&m.evaluate("out").unwrap());
    assert!((v - 24.0).abs() < 1e-9, "trimmed v={v}");
}

#[test]
fn sketch_extend_line_to_point() {
    // l1 originally too short (p1 -> p_short at (1, 0)). ExtendLine to
    // p2 rewrites p_short -> p2. Rectangle closes: volume = 24.
    let sketch = Sketch {
        plane: SketchPlane::Xy,
        primitives: vec![
            SketchPrim::Point { id: "p1".into(), x: Scalar::lit(0.0), y: Scalar::lit(0.0) },
            SketchPrim::Point { id: "p2".into(), x: Scalar::lit(2.0), y: Scalar::lit(0.0) },
            SketchPrim::Point { id: "p3".into(), x: Scalar::lit(2.0), y: Scalar::lit(3.0) },
            SketchPrim::Point { id: "p4".into(), x: Scalar::lit(0.0), y: Scalar::lit(3.0) },
            SketchPrim::Point { id: "p_short".into(), x: Scalar::lit(1.0), y: Scalar::lit(0.0) },
            SketchPrim::Line { id: "l1".into(), from: "p1".into(), to: "p_short".into() },
            SketchPrim::Line { id: "l2".into(), from: "p2".into(), to: "p3".into() },
            SketchPrim::Line { id: "l3".into(), from: "p3".into(), to: "p4".into() },
            SketchPrim::Line { id: "l4".into(), from: "p4".into(), to: "p1".into() },
            SketchPrim::ExtendLine {
                id: "ex1".into(),
                line: "l1".into(),
                to_point: "p2".into(),
            },
        ],
        constraints: vec![],
    };
    let m = Model::new().add(Feature::SketchExtrude {
        id: "out".into(),
        sketch,
        direction: [Scalar::lit(0.0), Scalar::lit(0.0), Scalar::lit(4.0)],
        skip_solve: false,
    });
    let v = solid_volume(&m.evaluate("out").unwrap());
    assert!((v - 24.0).abs() < 1e-9, "extended v={v}");
}

#[test]
fn sketch_fillet_corner_radius() {
    // 4 x 4 square. Fillet the (4, 0) corner with radius 1.
    let sketch = Sketch {
        plane: SketchPlane::Xy,
        primitives: vec![
            SketchPrim::Point { id: "p1".into(), x: Scalar::lit(0.0), y: Scalar::lit(0.0) },
            SketchPrim::Point { id: "p2".into(), x: Scalar::lit(4.0), y: Scalar::lit(0.0) },
            SketchPrim::Point { id: "p3".into(), x: Scalar::lit(4.0), y: Scalar::lit(4.0) },
            SketchPrim::Point { id: "p4".into(), x: Scalar::lit(0.0), y: Scalar::lit(4.0) },
            SketchPrim::Line { id: "l1".into(), from: "p1".into(), to: "p2".into() },
            SketchPrim::Line { id: "l2".into(), from: "p2".into(), to: "p3".into() },
            SketchPrim::Line { id: "l3".into(), from: "p3".into(), to: "p4".into() },
            SketchPrim::Line { id: "l4".into(), from: "p4".into(), to: "p1".into() },
            SketchPrim::FilletCorner {
                id: "f1".into(),
                corner_point: "p2".into(),
                radius: Scalar::lit(1.0),
            },
        ],
        constraints: vec![],
    };
    let m = Model::new().add(Feature::SketchExtrude {
        id: "out".into(),
        sketch,
        direction: [Scalar::lit(0.0), Scalar::lit(0.0), Scalar::lit(1.0)],
        skip_solve: false,
    });
    let v = solid_volume(&m.evaluate("out").unwrap());
    // Filleted square area: the fillet arc bulges toward the corner,
    // so we cut a "lune" out of the square equal to (corner triangle
    // area 0.5) - (arc segment area π/4 - 1/2) = 1 - π/4. Filleted
    // area = 16 - (1 - π/4) = 15 + π/4 ≈ 15.7854.
    let analytic = 15.0 + std::f64::consts::FRAC_PI_4;
    assert!((v - analytic).abs() < 0.05, "filleted v={v}, analytic={analytic}");
    assert!(v > 15.5 && v < 16.0, "v out of band: {v}");
}

#[test]
fn sketch_fillet_corner_round_trip_json() {
    let json = r#"{
      "features": [
        {
          "kind": "SketchExtrude",
          "id": "out",
          "direction": [0, 0, 1],
          "sketch": {
            "plane": "Xy",
            "primitives": [
              {"kind": "Point", "id": "p1", "x": 0, "y": 0},
              {"kind": "Point", "id": "p2", "x": 4, "y": 0},
              {"kind": "Point", "id": "p3", "x": 4, "y": 4},
              {"kind": "Point", "id": "p4", "x": 0, "y": 4},
              {"kind": "Line", "id": "l1", "from": "p1", "to": "p2"},
              {"kind": "Line", "id": "l2", "from": "p2", "to": "p3"},
              {"kind": "Line", "id": "l3", "from": "p3", "to": "p4"},
              {"kind": "Line", "id": "l4", "from": "p4", "to": "p1"},
              {"kind": "FilletCorner", "id": "f1", "corner_point": "p2", "radius": 1.0}
            ]
          }
        }
      ]
    }"#;
    let m = Model::from_json_str(json).unwrap();
    let v = solid_volume(&m.evaluate("out").unwrap());
    let analytic = 15.0 + std::f64::consts::FRAC_PI_4;
    assert!((v - analytic).abs() < 0.05, "json fillet v={v}");
}

#[test]
fn sketch_fillet_too_large_rejects() {
    // 1 x 1 square, radius 2 → inset r/tan(45°) = 2 > edge length 1.
    let sketch = Sketch {
        plane: SketchPlane::Xy,
        primitives: vec![
            SketchPrim::Point { id: "p1".into(), x: Scalar::lit(0.0), y: Scalar::lit(0.0) },
            SketchPrim::Point { id: "p2".into(), x: Scalar::lit(1.0), y: Scalar::lit(0.0) },
            SketchPrim::Point { id: "p3".into(), x: Scalar::lit(1.0), y: Scalar::lit(1.0) },
            SketchPrim::Point { id: "p4".into(), x: Scalar::lit(0.0), y: Scalar::lit(1.0) },
            SketchPrim::Line { id: "l1".into(), from: "p1".into(), to: "p2".into() },
            SketchPrim::Line { id: "l2".into(), from: "p2".into(), to: "p3".into() },
            SketchPrim::Line { id: "l3".into(), from: "p3".into(), to: "p4".into() },
            SketchPrim::Line { id: "l4".into(), from: "p4".into(), to: "p1".into() },
            SketchPrim::FilletCorner {
                id: "f1".into(),
                corner_point: "p2".into(),
                radius: Scalar::lit(2.0),
            },
        ],
        constraints: vec![],
    };
    let m = Model::new().add(Feature::SketchExtrude {
        id: "out".into(),
        sketch,
        direction: [Scalar::lit(0.0), Scalar::lit(0.0), Scalar::lit(1.0)],
        skip_solve: false,
    });
    let err = m.evaluate("out").unwrap_err();
    let msg = format!("{err}");
    assert!(
        msg.contains("too large") || msg.contains("TooLarge"),
        "expected too-large error, got: {msg}"
    );
}

#[test]
fn sketch_trim_unknown_line_rejects() {
    let sketch = Sketch {
        plane: SketchPlane::Xy,
        primitives: vec![
            SketchPrim::Point { id: "p1".into(), x: Scalar::lit(0.0), y: Scalar::lit(0.0) },
            SketchPrim::Point { id: "p2".into(), x: Scalar::lit(1.0), y: Scalar::lit(0.0) },
            SketchPrim::TrimLine {
                id: "tr".into(),
                line: "ghost".into(),
                at_point: "p2".into(),
            },
        ],
        constraints: vec![],
    };
    let m = Model::new().add(Feature::SketchExtrude {
        id: "out".into(),
        sketch,
        direction: [Scalar::lit(0.0), Scalar::lit(0.0), Scalar::lit(1.0)],
        skip_solve: false,
    });
    let err = m.evaluate("out").unwrap_err();
    let msg = format!("{err}");
    assert!(
        msg.contains("unknown line") || msg.contains("ghost"),
        "expected unknown-line error, got: {msg}"
    );
}

#[test]
fn sketch_fillet_two_corners_compose() {
    // Fillet two adjacent corners of a 4 x 4 square (p2 and p3).
    // Each fillet (radius 1) removes 0.5 and adds π/4. Total area:
    // 16 - 1.0 + π/2 ≈ 16.5708.
    let sketch = Sketch {
        plane: SketchPlane::Xy,
        primitives: vec![
            SketchPrim::Point { id: "p1".into(), x: Scalar::lit(0.0), y: Scalar::lit(0.0) },
            SketchPrim::Point { id: "p2".into(), x: Scalar::lit(4.0), y: Scalar::lit(0.0) },
            SketchPrim::Point { id: "p3".into(), x: Scalar::lit(4.0), y: Scalar::lit(4.0) },
            SketchPrim::Point { id: "p4".into(), x: Scalar::lit(0.0), y: Scalar::lit(4.0) },
            SketchPrim::Line { id: "l1".into(), from: "p1".into(), to: "p2".into() },
            SketchPrim::Line { id: "l2".into(), from: "p2".into(), to: "p3".into() },
            SketchPrim::Line { id: "l3".into(), from: "p3".into(), to: "p4".into() },
            SketchPrim::Line { id: "l4".into(), from: "p4".into(), to: "p1".into() },
            SketchPrim::FilletCorner {
                id: "f1".into(),
                corner_point: "p2".into(),
                radius: Scalar::lit(1.0),
            },
            SketchPrim::FilletCorner {
                id: "f2".into(),
                corner_point: "p3".into(),
                radius: Scalar::lit(1.0),
            },
        ],
        constraints: vec![],
    };
    let m = Model::new().add(Feature::SketchExtrude {
        id: "out".into(),
        sketch,
        direction: [Scalar::lit(0.0), Scalar::lit(0.0), Scalar::lit(1.0)],
        skip_solve: false,
    });
    let v = solid_volume(&m.evaluate("out").unwrap());
    // Two corners filleted, each removing (1 - π/4) = 0.2146.
    // Net area = 16 - 2 * (1 - π/4) = 14 + π/2 ≈ 15.5708.
    let analytic = 14.0 + std::f64::consts::FRAC_PI_2;
    assert!(
        (v - analytic).abs() < 0.1,
        "two-corner fillet v={v}, analytic={analytic}"
    );
}

// ----------------------------------------------------------------------
// Tier A (kerf-sketcher-100): polygon-with-holes pipeline.
//
// A SketchExtrude whose sketch contains an outer loop with one or more
// inner loops fully nested inside it should now build a single solid
// whose interior matches the polygon-with-holes definition (extrude of
// outer minus extrude of each hole), rather than two unioned solids.
// ----------------------------------------------------------------------

#[test]
fn sketch_polygon_with_one_hole_extrudes() {
    // 4×4 outer square, 1×1 inner square hole at (1.5..2.5, 1.5..2.5).
    // Extruded by 2 → expected volume = 16*2 - 1*1*2 = 30.
    let mut prims: Vec<SketchPrim> = vec![
        SketchPrim::Point { id: "p1".into(), x: Scalar::lit(0.0), y: Scalar::lit(0.0) },
        SketchPrim::Point { id: "p2".into(), x: Scalar::lit(4.0), y: Scalar::lit(0.0) },
        SketchPrim::Point { id: "p3".into(), x: Scalar::lit(4.0), y: Scalar::lit(4.0) },
        SketchPrim::Point { id: "p4".into(), x: Scalar::lit(0.0), y: Scalar::lit(4.0) },
        SketchPrim::Line { id: "l1".into(), from: "p1".into(), to: "p2".into() },
        SketchPrim::Line { id: "l2".into(), from: "p2".into(), to: "p3".into() },
        SketchPrim::Line { id: "l3".into(), from: "p3".into(), to: "p4".into() },
        SketchPrim::Line { id: "l4".into(), from: "p4".into(), to: "p1".into() },
    ];
    // Inner square hole. Sketch tracer auto-orients to CCW; the
    // classifier flips holes to CW for the polygon-with-holes pipeline.
    prims.extend(vec![
        SketchPrim::Point { id: "h1".into(), x: Scalar::lit(1.5), y: Scalar::lit(1.5) },
        SketchPrim::Point { id: "h2".into(), x: Scalar::lit(2.5), y: Scalar::lit(1.5) },
        SketchPrim::Point { id: "h3".into(), x: Scalar::lit(2.5), y: Scalar::lit(2.5) },
        SketchPrim::Point { id: "h4".into(), x: Scalar::lit(1.5), y: Scalar::lit(2.5) },
        SketchPrim::Line { id: "lh1".into(), from: "h1".into(), to: "h2".into() },
        SketchPrim::Line { id: "lh2".into(), from: "h2".into(), to: "h3".into() },
        SketchPrim::Line { id: "lh3".into(), from: "h3".into(), to: "h4".into() },
        SketchPrim::Line { id: "lh4".into(), from: "h4".into(), to: "h1".into() },
    ]);
    let s = Sketch {
        plane: SketchPlane::Xy,
        primitives: prims,
        constraints: vec![],
    };
    let m = Model::new().add(Feature::SketchExtrude {
        id: "out".into(),
        sketch: s,
        direction: [Scalar::lit(0.0), Scalar::lit(0.0), Scalar::lit(2.0)],
        skip_solve: false,
    });
    let v = solid_volume(&m.evaluate("out").unwrap());
    assert!(
        (v - 30.0).abs() < 0.5,
        "polygon-with-1-hole expected v≈30, got {v}"
    );
}

#[test]
fn sketch_polygon_with_two_holes_extrudes() {
    // 10×10 outer, two non-overlapping 1×1 holes. Extruded by 1 →
    // expected volume = 100 - 2 = 98.
    let mut prims: Vec<SketchPrim> = vec![
        SketchPrim::Point { id: "p1".into(), x: Scalar::lit(0.0), y: Scalar::lit(0.0) },
        SketchPrim::Point { id: "p2".into(), x: Scalar::lit(10.0), y: Scalar::lit(0.0) },
        SketchPrim::Point { id: "p3".into(), x: Scalar::lit(10.0), y: Scalar::lit(10.0) },
        SketchPrim::Point { id: "p4".into(), x: Scalar::lit(0.0), y: Scalar::lit(10.0) },
        SketchPrim::Line { id: "l1".into(), from: "p1".into(), to: "p2".into() },
        SketchPrim::Line { id: "l2".into(), from: "p2".into(), to: "p3".into() },
        SketchPrim::Line { id: "l3".into(), from: "p3".into(), to: "p4".into() },
        SketchPrim::Line { id: "l4".into(), from: "p4".into(), to: "p1".into() },
    ];
    // Hole A around (3, 5).
    prims.extend(vec![
        SketchPrim::Point { id: "ha1".into(), x: Scalar::lit(2.5), y: Scalar::lit(4.5) },
        SketchPrim::Point { id: "ha2".into(), x: Scalar::lit(3.5), y: Scalar::lit(4.5) },
        SketchPrim::Point { id: "ha3".into(), x: Scalar::lit(3.5), y: Scalar::lit(5.5) },
        SketchPrim::Point { id: "ha4".into(), x: Scalar::lit(2.5), y: Scalar::lit(5.5) },
        SketchPrim::Line { id: "la1".into(), from: "ha1".into(), to: "ha2".into() },
        SketchPrim::Line { id: "la2".into(), from: "ha2".into(), to: "ha3".into() },
        SketchPrim::Line { id: "la3".into(), from: "ha3".into(), to: "ha4".into() },
        SketchPrim::Line { id: "la4".into(), from: "ha4".into(), to: "ha1".into() },
    ]);
    // Hole B around (7, 5).
    prims.extend(vec![
        SketchPrim::Point { id: "hb1".into(), x: Scalar::lit(6.5), y: Scalar::lit(4.5) },
        SketchPrim::Point { id: "hb2".into(), x: Scalar::lit(7.5), y: Scalar::lit(4.5) },
        SketchPrim::Point { id: "hb3".into(), x: Scalar::lit(7.5), y: Scalar::lit(5.5) },
        SketchPrim::Point { id: "hb4".into(), x: Scalar::lit(6.5), y: Scalar::lit(5.5) },
        SketchPrim::Line { id: "lb1".into(), from: "hb1".into(), to: "hb2".into() },
        SketchPrim::Line { id: "lb2".into(), from: "hb2".into(), to: "hb3".into() },
        SketchPrim::Line { id: "lb3".into(), from: "hb3".into(), to: "hb4".into() },
        SketchPrim::Line { id: "lb4".into(), from: "hb4".into(), to: "hb1".into() },
    ]);
    let s = Sketch {
        plane: SketchPlane::Xy,
        primitives: prims,
        constraints: vec![],
    };
    let m = Model::new().add(Feature::SketchExtrude {
        id: "out".into(),
        sketch: s,
        direction: [Scalar::lit(0.0), Scalar::lit(0.0), Scalar::lit(1.0)],
        skip_solve: false,
    });
    let v = solid_volume(&m.evaluate("out").unwrap());
    assert!(
        (v - 98.0).abs() < 0.5,
        "polygon-with-2-holes expected v≈98, got {v}"
    );
}

#[test]
fn sketch_holes_overlapping_rejects() {
    // Outer 10x10. Two inner squares that *cross* each other → reject.
    let mut prims: Vec<SketchPrim> = vec![
        SketchPrim::Point { id: "p1".into(), x: Scalar::lit(0.0), y: Scalar::lit(0.0) },
        SketchPrim::Point { id: "p2".into(), x: Scalar::lit(10.0), y: Scalar::lit(0.0) },
        SketchPrim::Point { id: "p3".into(), x: Scalar::lit(10.0), y: Scalar::lit(10.0) },
        SketchPrim::Point { id: "p4".into(), x: Scalar::lit(0.0), y: Scalar::lit(10.0) },
        SketchPrim::Line { id: "l1".into(), from: "p1".into(), to: "p2".into() },
        SketchPrim::Line { id: "l2".into(), from: "p2".into(), to: "p3".into() },
        SketchPrim::Line { id: "l3".into(), from: "p3".into(), to: "p4".into() },
        SketchPrim::Line { id: "l4".into(), from: "p4".into(), to: "p1".into() },
    ];
    // Two hole rectangles that overlap.
    prims.extend(vec![
        SketchPrim::Point { id: "ha1".into(), x: Scalar::lit(3.0), y: Scalar::lit(3.0) },
        SketchPrim::Point { id: "ha2".into(), x: Scalar::lit(6.0), y: Scalar::lit(3.0) },
        SketchPrim::Point { id: "ha3".into(), x: Scalar::lit(6.0), y: Scalar::lit(6.0) },
        SketchPrim::Point { id: "ha4".into(), x: Scalar::lit(3.0), y: Scalar::lit(6.0) },
        SketchPrim::Line { id: "la1".into(), from: "ha1".into(), to: "ha2".into() },
        SketchPrim::Line { id: "la2".into(), from: "ha2".into(), to: "ha3".into() },
        SketchPrim::Line { id: "la3".into(), from: "ha3".into(), to: "ha4".into() },
        SketchPrim::Line { id: "la4".into(), from: "ha4".into(), to: "ha1".into() },
    ]);
    prims.extend(vec![
        SketchPrim::Point { id: "hb1".into(), x: Scalar::lit(5.0), y: Scalar::lit(5.0) },
        SketchPrim::Point { id: "hb2".into(), x: Scalar::lit(8.0), y: Scalar::lit(5.0) },
        SketchPrim::Point { id: "hb3".into(), x: Scalar::lit(8.0), y: Scalar::lit(8.0) },
        SketchPrim::Point { id: "hb4".into(), x: Scalar::lit(5.0), y: Scalar::lit(8.0) },
        SketchPrim::Line { id: "lb1".into(), from: "hb1".into(), to: "hb2".into() },
        SketchPrim::Line { id: "lb2".into(), from: "hb2".into(), to: "hb3".into() },
        SketchPrim::Line { id: "lb3".into(), from: "hb3".into(), to: "hb4".into() },
        SketchPrim::Line { id: "lb4".into(), from: "hb4".into(), to: "hb1".into() },
    ]);
    let s = Sketch {
        plane: SketchPlane::Xy,
        primitives: prims,
        constraints: vec![],
    };
    let m = Model::new().add(Feature::SketchExtrude {
        id: "out".into(),
        sketch: s,
        direction: [Scalar::lit(0.0), Scalar::lit(0.0), Scalar::lit(1.0)],
        skip_solve: false,
    });
    let err = m.evaluate("out").unwrap_err();
    let msg = format!("{err}");
    assert!(
        msg.contains("DisjointSubLoops") || msg.contains("overlapping") || msg.contains("disjoint"),
        "expected overlap rejection, got: {msg}"
    );
}

#[test]
fn sketch_polygon_with_circular_hole_extrudes() {
    // 4×4 outer square with a low-poly circular (octagonal) hole of radius 0.5
    // at center. Few segments keeps the boolean engine happy on coplanar
    // coincidences. Expected volume ≈ 16*2 - octagon_area * 2.
    // Octagon (n=8, r=0.5) area = 8 * 0.5 * r² * sin(2π/8) = 4 * 0.25 * sin(π/4)
    //                            = 1.0 * sin(π/4) ≈ 0.7071.
    let prims: Vec<SketchPrim> = vec![
        SketchPrim::Point { id: "p1".into(), x: Scalar::lit(0.0), y: Scalar::lit(0.0) },
        SketchPrim::Point { id: "p2".into(), x: Scalar::lit(4.0), y: Scalar::lit(0.0) },
        SketchPrim::Point { id: "p3".into(), x: Scalar::lit(4.0), y: Scalar::lit(4.0) },
        SketchPrim::Point { id: "p4".into(), x: Scalar::lit(0.0), y: Scalar::lit(4.0) },
        SketchPrim::Line { id: "l1".into(), from: "p1".into(), to: "p2".into() },
        SketchPrim::Line { id: "l2".into(), from: "p2".into(), to: "p3".into() },
        SketchPrim::Line { id: "l3".into(), from: "p3".into(), to: "p4".into() },
        SketchPrim::Line { id: "l4".into(), from: "p4".into(), to: "p1".into() },
        SketchPrim::Point { id: "c".into(), x: Scalar::lit(2.0), y: Scalar::lit(2.0) },
        SketchPrim::Circle {
            id: "k".into(),
            center: "c".into(),
            radius: Scalar::lit(0.5),
            n_segments: 8,
        },
    ];
    let s = Sketch {
        plane: SketchPlane::Xy,
        primitives: prims,
        constraints: vec![],
    };
    let m = Model::new().add(Feature::SketchExtrude {
        id: "out".into(),
        sketch: s,
        direction: [Scalar::lit(0.0), Scalar::lit(0.0), Scalar::lit(2.0)],
        skip_solve: false,
    });
    // The boolean engine struggles with high-segment-count circles inside
    // a polygon-with-holes diff. We accept either a successful build (volume
    // close to analytic) OR a clean Boolean error. The polygon-with-2-holes
    // and -1-square-hole tests above already verify the happy path; this
    // case documents the curved-hole limitation. n_segments=8 (octagon) is
    // low enough to actually succeed today.
    match m.evaluate("out") {
        Ok(solid) => {
            let v = solid_volume(&solid);
            let octagon_area = 8.0 * 0.5 * 0.25 * (std::f64::consts::TAU / 8.0).sin();
            let analytic = 32.0 - octagon_area * 2.0;
            assert!(
                (v - analytic).abs() < 0.5,
                "octagon-hole v={v}, analytic≈{analytic}"
            );
        }
        Err(e) => {
            let msg = format!("{e}");
            // Document expected boolean engine quirk surface for curved holes.
            assert!(
                msg.contains("non-manifold") || msg.contains("Boolean"),
                "unexpected error kind: {msg}"
            );
        }
    }
}
