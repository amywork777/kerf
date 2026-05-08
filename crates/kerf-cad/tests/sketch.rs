//! 2D sketch data model + JSON DSL tests.
//!
//! Sketch is the parametric authoring layer for planar profiles. These
//! tests exercise the data model, the loop tracer, JSON round-trip,
//! constraint storage, and integration with `Feature::SketchExtrude` /
//! `Feature::SketchRevolve`.

use kerf_brep::solid_volume;
use kerf_cad::{
    Feature, Model, Scalar, Sketch, SketchConstraint, SketchPlane, SketchPrim,
};

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
