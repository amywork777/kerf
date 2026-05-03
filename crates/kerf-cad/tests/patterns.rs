//! LinearPattern + PolarPattern features.

use kerf_brep::solid_volume;
use kerf_cad::{lits, Feature, Model, Scalar};

#[test]
fn linear_pattern_creates_count_copies_unioned() {
    // 3 disjoint unit cubes along x — total volume 3.
    let m = Model::new()
        .add(Feature::Box {
            id: "u".into(),
            extents: lits([1.0, 1.0, 1.0]),
        })
        .add(Feature::LinearPattern {
            id: "out".into(),
            input: "u".into(),
            count: 3,
            offset: lits([5.0, 0.0, 0.0]),
        });
    let s = m.evaluate("out").unwrap();
    let v = solid_volume(&s);
    assert!((v - 3.0).abs() < 1e-9, "v={v}");
    assert_eq!(s.shell_count(), 3, "3 disjoint cubes → 3 shells");
}

#[test]
fn linear_pattern_with_count_one_returns_input_unchanged() {
    let m = Model::new()
        .add(Feature::Box { id: "u".into(), extents: lits([2.0, 3.0, 4.0]) })
        .add(Feature::LinearPattern {
            id: "out".into(),
            input: "u".into(),
            count: 1,
            offset: lits([10.0, 0.0, 0.0]),
        });
    let s = m.evaluate("out").unwrap();
    assert!((solid_volume(&s) - 24.0).abs() < 1e-9);
}

#[test]
fn linear_pattern_zero_count_is_invalid() {
    let m = Model::new()
        .add(Feature::Box { id: "u".into(), extents: lits([1.0, 1.0, 1.0]) })
        .add(Feature::LinearPattern {
            id: "out".into(),
            input: "u".into(),
            count: 0,
            offset: lits([1.0, 0.0, 0.0]),
        });
    assert!(m.evaluate("out").is_err());
}

#[test]
fn polar_pattern_3_around_z_axis() {
    // Place a small box offset from origin, replicate 3 around z.
    let m = Model::new()
        .add(Feature::BoxAt {
            id: "stud".into(),
            extents: lits([0.5, 0.5, 1.0]),
            origin: lits([5.0, -0.25, 0.0]),
        })
        .add(Feature::PolarPattern {
            id: "out".into(),
            input: "stud".into(),
            count: 3,
            axis: lits([0.0, 0.0, 1.0]),
            center: lits([0.0, 0.0, 0.0]),
            total_angle_deg: Scalar::lit(360.0),
        });
    let s = m.evaluate("out").unwrap();
    // 3 disjoint studs of 0.5 × 0.5 × 1 = 0.25 each → 0.75 total.
    assert!((solid_volume(&s) - 0.75).abs() < 1e-9);
    assert_eq!(s.shell_count(), 3);
}

#[test]
fn polar_pattern_round_trips_via_json() {
    let m = Model::new()
        .add(Feature::BoxAt {
            id: "stud".into(),
            extents: lits([0.5, 0.5, 1.0]),
            origin: lits([5.0, -0.25, 0.0]),
        })
        .add(Feature::PolarPattern {
            id: "out".into(),
            input: "stud".into(),
            count: 4,
            axis: lits([0.0, 0.0, 1.0]),
            center: lits([0.0, 0.0, 0.0]),
            total_angle_deg: Scalar::lit(360.0),
        });
    let json = m.to_json_string().unwrap();
    let m2 = Model::from_json_str(&json).unwrap();
    let v1 = solid_volume(&m.evaluate("out").unwrap());
    let v2 = solid_volume(&m2.evaluate("out").unwrap());
    assert!((v1 - v2).abs() < 1e-9);
}
