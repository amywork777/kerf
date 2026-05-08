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
