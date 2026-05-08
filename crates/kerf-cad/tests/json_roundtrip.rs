//! Step 3 — Model can be serialized to JSON and read back, producing the
//! same Solid on evaluation.

use kerf_brep::solid_volume;
use kerf_cad::{lits, Feature, Model, Profile2D, Scalar};

fn approx(a: f64, b: f64, eps: f64) -> bool {
    (a - b).abs() <= eps
}

#[test]
fn json_roundtrip_preserves_evaluation_result() {
    let m = Model::new()
        .add(Feature::Box {
            id: "body".into(),
            extents: lits([10.0, 5.0, 2.0]),
        })
        .add(Feature::Cylinder {
            id: "hole".into(),
            radius: Scalar::lit(1.0),
            height: Scalar::lit(5.0),
            segments: 16,
        })
        .add(Feature::Translate {
            id: "hole_pos".into(),
            input: "hole".into(),
            offset: lits([5.0, 2.5, -1.0]),
        })
        .add(Feature::Difference {
            id: "out".into(),
            inputs: vec!["body".into(), "hole_pos".into()],
        });

    let json = m.to_json_string().expect("serialize");
    let m2 = Model::from_json_str(&json).expect("deserialize");

    assert_eq!(m2.len(), m.len());
    let v1 = solid_volume(&m.evaluate("out").unwrap());
    let v2 = solid_volume(&m2.evaluate("out").unwrap());
    assert!(approx(v1, v2, 1e-9), "v1={v1}, v2={v2}");
}

#[test]
fn json_roundtrip_covers_every_feature_variant() {
    let m = Model::new()
        .add(Feature::Box {
            id: "a".into(),
            extents: lits([1.0, 1.0, 1.0]),
        })
        .add(Feature::BoxAt {
            id: "b".into(),
            extents: lits([1.0, 1.0, 1.0]),
            origin: lits([5.0, 0.0, 0.0]),
        })
        .add(Feature::Cylinder {
            id: "c".into(),
            radius: Scalar::lit(1.0),
            height: Scalar::lit(2.0),
            segments: 12,
        })
        .add(Feature::Sphere {
            id: "d".into(),
            radius: Scalar::lit(1.0),
        })
        .add(Feature::Torus {
            id: "e".into(),
            major_radius: Scalar::lit(2.0),
            minor_radius: Scalar::lit(0.5),
        })
        .add(Feature::Cone {
            id: "f".into(),
            radius: Scalar::lit(1.0),
            height: Scalar::lit(2.0),
        })
        .add(Feature::Frustum {
            id: "g".into(),
            top_radius: Scalar::lit(0.5),
            bottom_radius: Scalar::lit(1.0),
            height: Scalar::lit(2.0),
        })
        .add(Feature::ExtrudePolygon {
            id: "h".into(),
            profile: Profile2D {
                points: vec![
                    [Scalar::lit(0.0), Scalar::lit(0.0)],
                    [Scalar::lit(1.0), Scalar::lit(0.0)],
                    [Scalar::lit(1.0), Scalar::lit(1.0)],
                    [Scalar::lit(0.0), Scalar::lit(1.0)],
                ],
            },
            direction: lits([0.0, 0.0, 1.0]),
        })
        .add(Feature::Translate {
            id: "i".into(),
            input: "a".into(),
            offset: lits([1.0, 0.0, 0.0]),
        })
        .add(Feature::Rotate {
            id: "j".into(),
            input: "a".into(),
            axis: lits([0.0, 0.0, 1.0]),
            angle_deg: Scalar::lit(30.0),
            center: lits([0.0, 0.0, 0.0]),
        })
        .add(Feature::Union {
            id: "k".into(),
            inputs: vec!["a".into(), "i".into()],
        })
        .add(Feature::Intersection {
            id: "l".into(),
            inputs: vec!["a".into(), "i".into()],
        })
        .add(Feature::Difference {
            id: "m".into(),
            inputs: vec!["a".into(), "i".into()],
        });

    let json = m.to_json_string().expect("serialize");
    let m2 = Model::from_json_str(&json).expect("deserialize");

    for f in [
        "a", "b", "c", "d", "e", "f", "g", "h", "i", "j", "k", "l", "m",
    ] {
        assert_eq!(
            m.feature(f),
            m2.feature(f),
            "feature {f} mismatch after round-trip"
        );
    }
}

#[test]
fn deserialize_rebuilds_index_so_lookup_works() {
    let json = r#"{
        "features": [
            {"kind": "Box", "id": "a", "extents": [1.0, 1.0, 1.0]}
        ]
    }"#;
    let m = Model::from_json_str(json).expect("deserialize");
    assert!(m.feature("a").is_some(), "lookup must work after deserialize");
}

#[test]
fn read_and_write_via_path() {
    use std::env;
    let dir = env::temp_dir();
    let path = dir.join("kerf-cad-roundtrip-test.json");

    let m = Model::new().add(Feature::Box {
        id: "x".into(),
        extents: lits([2.0, 2.0, 2.0]),
    });
    m.write_json_path(&path).expect("write");
    let m2 = Model::read_json_path(&path).expect("read");
    assert_eq!(m.len(), m2.len());
    assert!(m2.feature("x").is_some());

    let _ = std::fs::remove_file(&path);
}
