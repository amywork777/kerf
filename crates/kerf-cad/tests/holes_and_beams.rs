//! IBeam, HoleArray, BoltCircle, HexHole, SquareHole tests.

use kerf_brep::solid_volume;
use kerf_cad::{lits, Feature, Model, Scalar};

// -------- IBeam --------

#[test]
fn ibeam_volume_matches_two_flanges_plus_web() {
    let fw = 6.0;
    let ft = 0.8;
    let wt = 1.0;
    let th = 5.0;
    let d = 4.0;
    let m = Model::new().add(Feature::IBeam {
        id: "out".into(),
        flange_width: Scalar::lit(fw),
        flange_thickness: Scalar::lit(ft),
        web_thickness: Scalar::lit(wt),
        total_height: Scalar::lit(th),
        depth: Scalar::lit(d),
    });
    let v = solid_volume(&m.evaluate("out").unwrap());
    let area = 2.0 * fw * ft + wt * (th - 2.0 * ft);
    let exp = area * d;
    assert!((v - exp).abs() < 1e-6, "v={v}, exp={exp}");
}

#[test]
fn ibeam_rejects_thick_flanges() {
    let m = Model::new().add(Feature::IBeam {
        id: "out".into(),
        flange_width: Scalar::lit(4.0),
        flange_thickness: Scalar::lit(2.0),
        web_thickness: Scalar::lit(0.5),
        total_height: Scalar::lit(4.0),
        depth: Scalar::lit(2.0),
    });
    assert!(m.evaluate("out").is_err());
}

// -------- HoleArray --------

#[test]
fn hole_array_drills_three_holes() {
    let m = Model::new()
        .add(Feature::Box {
            id: "body".into(),
            extents: lits([20.0, 6.0, 4.0]),
        })
        .add(Feature::HoleArray {
            id: "out".into(),
            input: "body".into(),
            axis: "z".into(),
            start: lits([5.0, 3.0, 4.0]),
            offset: lits([5.0, 0.0, 0.0]),
            count: 3,
            radius: Scalar::lit(1.0),
            depth: Scalar::lit(4.0),
            segments: 32,
        });
    let v = solid_volume(&m.evaluate("out").unwrap());
    let exp = 20.0 * 6.0 * 4.0 - 3.0 * std::f64::consts::PI * 1.0 * 4.0;
    let rel = (v - exp).abs() / exp;
    assert!(rel < 0.01, "v={v}, exp={exp}, rel={rel}");
}

#[test]
fn hole_array_rejects_zero_count() {
    let m = Model::new()
        .add(Feature::Box {
            id: "body".into(),
            extents: lits([10.0, 10.0, 5.0]),
        })
        .add(Feature::HoleArray {
            id: "out".into(),
            input: "body".into(),
            axis: "z".into(),
            start: lits([5.0, 5.0, 5.0]),
            offset: lits([1.0, 0.0, 0.0]),
            count: 0,
            radius: Scalar::lit(1.0),
            depth: Scalar::lit(4.0),
            segments: 16,
        });
    assert!(m.evaluate("out").is_err());
}

// -------- BoltCircle --------

#[test]
fn bolt_circle_drills_evenly_distributed_holes() {
    let m = Model::new()
        .add(Feature::Cylinder {
            id: "body".into(),
            radius: Scalar::lit(20.0),
            height: Scalar::lit(5.0),
            segments: 64,
        })
        .add(Feature::BoltCircle {
            id: "out".into(),
            input: "body".into(),
            axis: "z".into(),
            center: lits([0.0, 0.0, 5.0]),
            bolt_circle_radius: Scalar::lit(15.0),
            count: 4,
            radius: Scalar::lit(1.5),
            depth: Scalar::lit(5.0),
            segments: 24,
        });
    let s = m.evaluate("out").unwrap();
    let v = solid_volume(&s);
    // Body volume ~ π*20²*5 = 6283.19, minus 4 holes of π*1.5²*5 = 35.34 each.
    let body_area = 0.5 * 64.0 * 20.0 * 20.0 * (2.0 * std::f64::consts::PI / 64.0).sin();
    let body_v = body_area * 5.0;
    let hole_v = std::f64::consts::PI * 1.5 * 1.5 * 5.0 * 4.0;
    let exp = body_v - hole_v;
    let rel = (v - exp).abs() / exp;
    assert!(rel < 0.01, "v={v}, exp={exp}, rel={rel}");
}

// -------- HexHole --------

#[test]
fn hex_hole_volume_matches_hex_pocket() {
    let m = Model::new()
        .add(Feature::Box {
            id: "body".into(),
            extents: lits([10.0, 10.0, 5.0]),
        })
        .add(Feature::HexHole {
            id: "out".into(),
            input: "body".into(),
            axis: "z".into(),
            top_center: lits([5.0, 5.0, 5.0]),
            inscribed_radius: Scalar::lit(2.0),
            depth: Scalar::lit(5.0),
        });
    let v = solid_volume(&m.evaluate("out").unwrap());
    // Hex with inscribed radius (apothem) 2.0 has area 6 * (1/2) * side * apothem
    // where side = 2 * apothem * tan(π/6) = 2 * 2 * (1/√3) = 4/√3.
    // Or equivalently: 2 * sqrt(3) * apothem² = 2 * 1.732 * 4 = 13.856.
    let apothem = 2.0;
    let area = 2.0 * 3.0_f64.sqrt() * apothem * apothem;
    let exp = 10.0 * 10.0 * 5.0 - area * 5.0;
    assert!((v - exp).abs() < 1e-3, "v={v}, exp={exp}");
}

// -------- SquareHole --------

#[test]
fn square_hole_volume_matches_square_pocket() {
    let m = Model::new()
        .add(Feature::Box {
            id: "body".into(),
            extents: lits([10.0, 10.0, 5.0]),
        })
        .add(Feature::SquareHole {
            id: "out".into(),
            input: "body".into(),
            axis: "z".into(),
            top_center: lits([5.0, 5.0, 5.0]),
            side: Scalar::lit(3.0),
            depth: Scalar::lit(5.0),
        });
    let v = solid_volume(&m.evaluate("out").unwrap());
    let exp = 10.0 * 10.0 * 5.0 - 3.0 * 3.0 * 5.0;
    assert!((v - exp).abs() < 1e-6, "v={v}, exp={exp}");
}

#[test]
fn square_hole_x_axis_volume_matches() {
    // Drill a square pocket through one face along x-axis.
    let m = Model::new()
        .add(Feature::Box {
            id: "body".into(),
            extents: lits([10.0, 8.0, 6.0]),
        })
        .add(Feature::SquareHole {
            id: "out".into(),
            input: "body".into(),
            axis: "x".into(),
            top_center: lits([10.0, 4.0, 3.0]),
            side: Scalar::lit(2.0),
            depth: Scalar::lit(10.0),
        });
    let v = solid_volume(&m.evaluate("out").unwrap());
    let exp = 10.0 * 8.0 * 6.0 - 2.0 * 2.0 * 10.0;
    assert!((v - exp).abs() < 1e-6, "v={v}, exp={exp}");
}

#[test]
fn all_pocket_features_round_trip_via_json() {
    for f in [
        Feature::HoleArray {
            id: "out".into(),
            input: "body".into(),
            axis: "z".into(),
            start: lits([1.0, 1.0, 5.0]),
            offset: lits([2.0, 0.0, 0.0]),
            count: 3,
            radius: Scalar::lit(0.5),
            depth: Scalar::lit(5.0),
            segments: 16,
        },
        Feature::BoltCircle {
            id: "out".into(),
            input: "body".into(),
            axis: "z".into(),
            center: lits([5.0, 5.0, 5.0]),
            bolt_circle_radius: Scalar::lit(3.0),
            count: 4,
            radius: Scalar::lit(0.4),
            depth: Scalar::lit(5.0),
            segments: 16,
        },
        Feature::HexHole {
            id: "out".into(),
            input: "body".into(),
            axis: "z".into(),
            top_center: lits([5.0, 5.0, 5.0]),
            inscribed_radius: Scalar::lit(1.0),
            depth: Scalar::lit(5.0),
        },
        Feature::SquareHole {
            id: "out".into(),
            input: "body".into(),
            axis: "z".into(),
            top_center: lits([5.0, 5.0, 5.0]),
            side: Scalar::lit(2.0),
            depth: Scalar::lit(5.0),
        },
    ] {
        let m = Model::new()
            .add(Feature::Box {
                id: "body".into(),
                extents: lits([10.0, 10.0, 5.0]),
            })
            .add(f);
        let json = m.to_json_string().unwrap();
        let m2 = Model::from_json_str(&json).unwrap();
        let v1 = solid_volume(&m.evaluate("out").unwrap());
        let v2 = solid_volume(&m2.evaluate("out").unwrap());
        assert!((v1 - v2).abs() < 1e-9);
    }
}
