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
fn tube_is_outer_minus_inner_cylinder() {
    let m = Model::new().add(Feature::Tube {
        id: "t".into(),
        outer_radius: Scalar::lit(5.0),
        inner_radius: Scalar::lit(3.0),
        height: Scalar::lit(10.0),
        segments: 24,
    });
    let s = m.evaluate("t").unwrap();
    let v = kerf_brep::solid_volume(&s);
    let n = 24.0_f64;
    let area = |r: f64| 0.5 * n * r * r * (2.0 * std::f64::consts::PI / n).sin();
    let expected = (area(5.0) - area(3.0)) * 10.0;
    let rel = (v - expected).abs() / expected.abs();
    assert!(rel < 1e-9, "v={v} expected={expected}");
}

#[test]
fn tube_inner_must_be_smaller_than_outer() {
    let m = Model::new().add(Feature::Tube {
        id: "t".into(),
        outer_radius: Scalar::lit(2.0),
        inner_radius: Scalar::lit(5.0),
        height: Scalar::lit(10.0),
        segments: 12,
    });
    assert!(m.evaluate("t").is_err());
}

#[test]
fn hollow_box_is_outer_minus_inset_inner() {
    let m = Model::new().add(Feature::HollowBox {
        id: "hb".into(),
        extents: lits([10.0, 10.0, 10.0]),
        wall_thickness: Scalar::lit(1.0),
    });
    let s = m.evaluate("hb").unwrap();
    let v = kerf_brep::solid_volume(&s);
    // outer 10³ minus inner 8³ = 1000 - 512 = 488
    assert!((v - 488.0).abs() < 1e-9, "v={v}");
}

#[test]
fn hollow_box_rejects_too_thick_wall() {
    let m = Model::new().add(Feature::HollowBox {
        id: "hb".into(),
        extents: lits([4.0, 4.0, 4.0]),
        wall_thickness: Scalar::lit(3.0), // 2*3=6 > 4
    });
    assert!(m.evaluate("hb").is_err());
}

#[test]
fn corner_cut_subtracts_a_corner() {
    let m = Model::new()
        .add(Feature::Box { id: "block".into(), extents: lits([10.0, 10.0, 10.0]) })
        .add(Feature::CornerCut {
            id: "out".into(),
            input: "block".into(),
            corner: lits([8.0, 8.0, 8.0]),
            extents: lits([3.0, 3.0, 3.0]),
        });
    let s = m.evaluate("out").unwrap();
    let v = kerf_brep::solid_volume(&s);
    // 1000 minus 2x2x2 corner = 992 (cutter [8,8,8] to [11,11,11] but block goes to 10,10,10
    // so only the 2x2x2 portion inside the block is subtracted)
    assert!((v - 992.0).abs() < 1e-9, "v={v}");
}

#[test]
fn corner_cut_rejects_zero_extent() {
    let m = Model::new()
        .add(Feature::Box { id: "block".into(), extents: lits([10.0, 10.0, 10.0]) })
        .add(Feature::CornerCut {
            id: "out".into(),
            input: "block".into(),
            corner: lits([0.0, 0.0, 0.0]),
            extents: lits([0.0, 1.0, 1.0]),
        });
    assert!(m.evaluate("out").is_err());
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
