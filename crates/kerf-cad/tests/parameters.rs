//! Step 5 — Model parameters: Scalar fields can be either a literal f64 or
//! a "$name" reference resolved from Model.parameters at eval time.

use kerf_brep::solid_volume;
use kerf_cad::{lits, Feature, Model, Profile2D, Scalar};

#[test]
fn parameter_substitution_changes_evaluated_volume() {
    fn build(plate_x: f64) -> Model {
        Model::new()
            .with_parameter("plate_x", plate_x)
            .with_parameter("plate_y", 60.0)
            .with_parameter("plate_z", 8.0)
            .add(Feature::Box {
                id: "body".into(),
                extents: [
                    Scalar::param("plate_x"),
                    Scalar::param("plate_y"),
                    Scalar::param("plate_z"),
                ],
            })
    }

    let small = build(100.0).evaluate("body").unwrap();
    let large = build(200.0).evaluate("body").unwrap();
    let v_small = solid_volume(&small);
    let v_large = solid_volume(&large);
    assert!((v_small - 100.0 * 60.0 * 8.0).abs() < 1e-9);
    assert!((v_large - 200.0 * 60.0 * 8.0).abs() < 1e-9);
    // And the ratio is exactly 2:1 since only x changed.
    assert!((v_large / v_small - 2.0).abs() < 1e-9);
}

#[test]
fn missing_parameter_returns_error() {
    let m = Model::new().add(Feature::Box {
        id: "body".into(),
        extents: [Scalar::param("nope"), Scalar::lit(1.0), Scalar::lit(1.0)],
    });
    let r = m.evaluate("body");
    assert!(r.is_err(), "missing param should error");
    let msg = format!("{}", r.unwrap_err());
    assert!(msg.contains("nope"), "error should mention the missing param: {msg}");
}

#[test]
fn parameters_round_trip_through_json() {
    let json = r#"{
        "parameters": {"radius": 2.5},
        "features": [
            {"kind": "Cylinder", "id": "c", "radius": "$radius", "height": 5.0, "segments": 16}
        ]
    }"#;
    let m = Model::from_json_str(json).expect("parse");
    let s = m.evaluate("c").unwrap();
    let v = solid_volume(&s);
    // Faceted-cylinder area for n=16, r=2.5 — same closed form as the bracket test.
    let n = 16.0_f64;
    let r = 2.5_f64;
    let area = 0.5 * n * r * r * (2.0 * std::f64::consts::PI / n).sin();
    let expected = area * 5.0;
    assert!((v - expected).abs() / expected < 1e-9, "v={v}, expected={expected}");
}

#[test]
fn lits_helper_constructs_literal_arrays() {
    let m = Model::new().add(Feature::Box {
        id: "body".into(),
        extents: lits([1.0, 2.0, 3.0]),
    });
    let s = m.evaluate("body").unwrap();
    assert!((solid_volume(&s) - 6.0).abs() < 1e-9);
}

#[test]
fn parameters_work_for_extrude_profile_too() {
    let m = Model::new()
        .with_parameter("side", 4.0)
        .add(Feature::ExtrudePolygon {
            id: "p".into(),
            profile: Profile2D {
                points: vec![
                    [Scalar::lit(0.0), Scalar::lit(0.0)],
                    [Scalar::param("side"), Scalar::lit(0.0)],
                    [Scalar::param("side"), Scalar::param("side")],
                    [Scalar::lit(0.0), Scalar::param("side")],
                ],
            },
            direction: lits([0.0, 0.0, 2.0]),
        });
    let s = m.evaluate("p").unwrap();
    // 4x4 square × 2 thick = 32.
    assert!((solid_volume(&s) - 32.0).abs() < 1e-9);
}
