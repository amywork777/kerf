//! Counterbore feature: stepped hole for socket-head fasteners.

use kerf_brep::solid_volume;
use kerf_cad::{lits, Feature, Model, Scalar};

#[test]
fn counterbore_z_axis_volume_matches() {
    // 20×20×10 box, counterbore at top center (10, 10, 10).
    // drill r=2, cbore r=4, cbore d=3, total d=10.
    // Removed volume = π*4²*3 + π*2²*7 (drill below cbore)
    //                = 48π + 28π = 76π
    // Approximated by 32-segment polygons.
    let m = Model::new()
        .add(Feature::Box {
            id: "body".into(),
            extents: lits([20.0, 20.0, 10.0]),
        })
        .add(Feature::Counterbore {
            id: "out".into(),
            input: "body".into(),
            axis: "z".into(),
            top_center: lits([10.0, 10.0, 10.0]),
            drill_radius: Scalar::lit(2.0),
            cbore_radius: Scalar::lit(4.0),
            cbore_depth: Scalar::lit(3.0),
            total_depth: Scalar::lit(10.0),
            segments: 32,
        });
    let s = m.evaluate("out").unwrap();
    let v = solid_volume(&s);
    let removed = std::f64::consts::PI * (16.0 * 3.0 + 4.0 * 7.0);
    let exp = 20.0 * 20.0 * 10.0 - removed;
    let rel = (v - exp).abs() / exp;
    assert!(rel < 0.01, "v={v}, exp={exp}, rel={rel}");
}

#[test]
fn counterbore_rejects_cbore_smaller_than_drill() {
    let m = Model::new()
        .add(Feature::Box {
            id: "body".into(),
            extents: lits([10.0, 10.0, 5.0]),
        })
        .add(Feature::Counterbore {
            id: "out".into(),
            input: "body".into(),
            axis: "z".into(),
            top_center: lits([5.0, 5.0, 5.0]),
            drill_radius: Scalar::lit(3.0),
            cbore_radius: Scalar::lit(2.0),
            cbore_depth: Scalar::lit(1.0),
            total_depth: Scalar::lit(4.0),
            segments: 16,
        });
    assert!(m.evaluate("out").is_err());
}

#[test]
fn counterbore_rejects_cbore_depth_geq_total_depth() {
    let m = Model::new()
        .add(Feature::Box {
            id: "body".into(),
            extents: lits([10.0, 10.0, 5.0]),
        })
        .add(Feature::Counterbore {
            id: "out".into(),
            input: "body".into(),
            axis: "z".into(),
            top_center: lits([5.0, 5.0, 5.0]),
            drill_radius: Scalar::lit(1.0),
            cbore_radius: Scalar::lit(2.0),
            cbore_depth: Scalar::lit(4.0),
            total_depth: Scalar::lit(4.0),
            segments: 16,
        });
    assert!(m.evaluate("out").is_err());
}

#[test]
fn counterbore_round_trips_via_json() {
    let m = Model::new()
        .add(Feature::Box {
            id: "body".into(),
            extents: lits([20.0, 20.0, 10.0]),
        })
        .add(Feature::Counterbore {
            id: "out".into(),
            input: "body".into(),
            axis: "z".into(),
            top_center: lits([10.0, 10.0, 10.0]),
            drill_radius: Scalar::lit(2.0),
            cbore_radius: Scalar::lit(4.0),
            cbore_depth: Scalar::lit(3.0),
            total_depth: Scalar::lit(10.0),
            segments: 24,
        });
    let json = m.to_json_string().unwrap();
    let m2 = Model::from_json_str(&json).unwrap();
    let v1 = solid_volume(&m.evaluate("out").unwrap());
    let v2 = solid_volume(&m2.evaluate("out").unwrap());
    assert!((v1 - v2).abs() < 1e-9);
}
