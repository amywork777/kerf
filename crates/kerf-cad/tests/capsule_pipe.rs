//! Capsule and PipeRun feature tests.

use kerf_brep::solid_volume;
use kerf_cad::{lits, Feature, Model, Scalar};

#[test]
fn capsule_volume_is_cylinder_plus_two_hemispheres() {
    let r = 1.0;
    let h = 4.0;
    let m = Model::new().add(Feature::Capsule {
        id: "out".into(),
        radius: Scalar::lit(r),
        height: Scalar::lit(h),
        stacks: 16,
        slices: 16,
    });
    let v = solid_volume(&m.evaluate("out").unwrap());
    // Cylinder volume: π·r²·h. Plus full sphere (two hemispheres):
    // (4/3)·π·r³.
    let exp = std::f64::consts::PI * r * r * h + (4.0 / 3.0) * std::f64::consts::PI * r.powi(3);
    let rel = (v - exp).abs() / exp;
    // Faceted approximation tolerance.
    assert!(rel < 0.10, "v={v}, exp={exp}, rel={rel}");
}

#[test]
fn capsule_round_trips_via_json() {
    let m = Model::new().add(Feature::Capsule {
        id: "out".into(),
        radius: Scalar::lit(1.0),
        height: Scalar::lit(3.0),
        stacks: 8,
        slices: 16,
    });
    let json = m.to_json_string().unwrap();
    let m2 = Model::from_json_str(&json).unwrap();
    let v1 = solid_volume(&m.evaluate("out").unwrap());
    let v2 = solid_volume(&m2.evaluate("out").unwrap());
    assert!((v1 - v2).abs() < 1e-9);
}

#[test]
fn pipe_run_two_segment_l_bend_works() {
    // L-shape: from (0,0,0) → (10,0,0) → (10,0,5).
    let m = Model::new().add(Feature::PipeRun {
        id: "out".into(),
        points: vec![
            lits([0.0, 0.0, 0.0]),
            lits([10.0, 0.0, 0.0]),
            lits([10.0, 0.0, 5.0]),
        ],
        radius: Scalar::lit(0.5),
        segments: 16,
    });
    let s = m.evaluate("out").unwrap();
    let v = solid_volume(&s);
    // Should be roughly 2 cylinder segments minus their overlap at the
    // joint. Sanity check it's positive and within reasonable bounds.
    let single_cyl_vol = std::f64::consts::PI * 0.25 * 10.0;
    let total_no_overlap = std::f64::consts::PI * 0.25 * 15.0;
    assert!(v > single_cyl_vol, "v={v} too small");
    assert!(v < total_no_overlap * 1.1, "v={v} too large");
}

#[test]
fn pipe_run_rejects_diagonal_segments() {
    let m = Model::new().add(Feature::PipeRun {
        id: "out".into(),
        points: vec![lits([0.0, 0.0, 0.0]), lits([1.0, 1.0, 0.0])],
        radius: Scalar::lit(0.5),
        segments: 16,
    });
    assert!(m.evaluate("out").is_err());
}

#[test]
fn pipe_run_rejects_one_point() {
    let m = Model::new().add(Feature::PipeRun {
        id: "out".into(),
        points: vec![lits([0.0, 0.0, 0.0])],
        radius: Scalar::lit(0.5),
        segments: 16,
    });
    assert!(m.evaluate("out").is_err());
}

#[test]
fn pipe_run_round_trips_via_json() {
    let m = Model::new().add(Feature::PipeRun {
        id: "out".into(),
        points: vec![
            lits([0.0, 0.0, 0.0]),
            lits([5.0, 0.0, 0.0]),
        ],
        radius: Scalar::lit(0.5),
        segments: 16,
    });
    let json = m.to_json_string().unwrap();
    let m2 = Model::from_json_str(&json).unwrap();
    let v1 = solid_volume(&m.evaluate("out").unwrap());
    let v2 = solid_volume(&m2.evaluate("out").unwrap());
    assert!((v1 - v2).abs() < 1e-9);
}
