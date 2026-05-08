//! Step 2 — Model::evaluate walks the DAG and produces a Solid.

use kerf_brep::solid_volume;
use kerf_cad::{lits, EvalError, Feature, Model, Profile2D, Scalar};

#[test]
fn evaluates_a_single_box_to_a_solid_with_correct_volume() {
    let m = Model::new().add(Feature::Box {
        id: "body".into(),
        extents: lits([10.0, 5.0, 2.0]),
    });
    let s = m.evaluate("body").expect("evaluation should succeed");
    assert!((solid_volume(&s) - 100.0).abs() < 1e-9);
}

#[test]
fn evaluates_bracket_with_two_holes() {
    // The canonical example: 100x60x8 plate with two 5mm-diameter through holes.
    let m = Model::new()
        .add(Feature::Box {
            id: "body".into(),
            extents: lits([100.0, 60.0, 8.0]),
        })
        .add(Feature::Cylinder {
            id: "hole1".into(),
            radius: Scalar::lit(2.5),
            height: Scalar::lit(10.0),
            segments: 16,
        })
        .add(Feature::Translate {
            id: "hole1_pos".into(),
            input: "hole1".into(),
            offset: lits([10.0, 10.0, -1.0]),
        })
        .add(Feature::Cylinder {
            id: "hole2".into(),
            radius: Scalar::lit(2.5),
            height: Scalar::lit(10.0),
            segments: 16,
        })
        .add(Feature::Translate {
            id: "hole2_pos".into(),
            input: "hole2".into(),
            offset: lits([90.0, 10.0, -1.0]),
        })
        .add(Feature::Difference {
            id: "out".into(),
            inputs: vec!["body".into(), "hole1_pos".into(), "hole2_pos".into()],
        });

    let s = m.evaluate("out").expect("bracket should evaluate cleanly");
    let v = solid_volume(&s);

    let body_vol = 100.0 * 60.0 * 8.0;
    let n = 16.0_f64;
    let r = 2.5_f64;
    let hole_area = 0.5 * n * r * r * (2.0 * std::f64::consts::PI / n).sin();
    let hole_vol = hole_area * 8.0;
    let expected = body_vol - 2.0 * hole_vol;
    let rel_err = (v - expected).abs() / expected.abs();
    assert!(rel_err < 0.01, "v={v}, expected≈{expected}, rel_err={rel_err}");
}

#[test]
fn unknown_target_returns_error() {
    let m = Model::new().add(Feature::Box {
        id: "body".into(),
        extents: lits([1.0, 1.0, 1.0]),
    });
    match m.evaluate("nope") {
        Err(EvalError::UnknownId(id)) => assert_eq!(id, "nope"),
        other => panic!("expected UnknownId, got {other:?}"),
    }
}

#[test]
fn missing_dependency_returns_error() {
    let m = Model::new().add(Feature::Translate {
        id: "t".into(),
        input: "ghost".into(),
        offset: lits([1.0, 0.0, 0.0]),
    });
    match m.evaluate("t") {
        Err(EvalError::UnknownId(id)) => assert_eq!(id, "ghost"),
        other => panic!("expected UnknownId(ghost), got {other:?}"),
    }
}

#[test]
fn cycle_returns_error() {
    let m = Model::new()
        .add(Feature::Translate {
            id: "a".into(),
            input: "b".into(),
            offset: lits([1.0, 0.0, 0.0]),
        })
        .add(Feature::Translate {
            id: "b".into(),
            input: "a".into(),
            offset: lits([1.0, 0.0, 0.0]),
        });
    match m.evaluate("a") {
        Err(EvalError::Cycle(_)) => {}
        other => panic!("expected Cycle, got {other:?}"),
    }
}

#[test]
fn extrude_polygon_works() {
    let m = Model::new().add(Feature::ExtrudePolygon {
        id: "p".into(),
        profile: Profile2D {
            points: vec![
                [Scalar::lit(0.0), Scalar::lit(0.0)],
                [Scalar::lit(1.0), Scalar::lit(0.0)],
                [Scalar::lit(1.0), Scalar::lit(1.0)],
                [Scalar::lit(0.0), Scalar::lit(1.0)],
            ],
        },
        direction: lits([0.0, 0.0, 3.0]),
    });
    let s = m.evaluate("p").expect("extrude should succeed");
    assert!((solid_volume(&s) - 3.0).abs() < 1e-9);
}

#[test]
fn translate_preserves_volume() {
    let m = Model::new()
        .add(Feature::Box {
            id: "body".into(),
            extents: lits([2.0, 3.0, 4.0]),
        })
        .add(Feature::Translate {
            id: "moved".into(),
            input: "body".into(),
            offset: lits([100.0, 50.0, 25.0]),
        });
    let s = m.evaluate("moved").unwrap();
    assert!((solid_volume(&s) - 24.0).abs() < 1e-9);
}

#[test]
fn rotate_preserves_volume() {
    let m = Model::new()
        .add(Feature::Box {
            id: "body".into(),
            extents: lits([2.0, 3.0, 4.0]),
        })
        .add(Feature::Rotate {
            id: "spun".into(),
            input: "body".into(),
            axis: lits([0.0, 0.0, 1.0]),
            angle_deg: Scalar::lit(47.0),
            center: lits([1.0, 1.5, 2.0]),
        });
    let s = m.evaluate("spun").unwrap();
    assert!((solid_volume(&s) - 24.0).abs() < 1e-9);
}
