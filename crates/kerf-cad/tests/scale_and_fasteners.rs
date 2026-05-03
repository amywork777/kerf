//! Scale transform and Bolt/CapScrew composed primitives.

use kerf_brep::solid_volume;
use kerf_cad::{lits, Feature, Model, Scalar};

// -------- Scale --------

#[test]
fn scale_box_volume_scales_by_factor_cubed() {
    let m = Model::new()
        .add(Feature::Box {
            id: "body".into(),
            extents: lits([1.0, 1.0, 1.0]),
        })
        .add(Feature::Scale {
            id: "out".into(),
            input: "body".into(),
            factor: Scalar::lit(2.0),
        });
    let v = solid_volume(&m.evaluate("out").unwrap());
    assert!((v - 8.0).abs() < 1e-9, "v={v}");
}

#[test]
fn scale_then_translate_preserves_relative_size() {
    let m = Model::new()
        .add(Feature::Box {
            id: "body".into(),
            extents: lits([2.0, 3.0, 4.0]),
        })
        .add(Feature::Scale {
            id: "scaled".into(),
            input: "body".into(),
            factor: Scalar::lit(0.5),
        })
        .add(Feature::Translate {
            id: "out".into(),
            input: "scaled".into(),
            offset: lits([10.0, 0.0, 0.0]),
        });
    let v = solid_volume(&m.evaluate("out").unwrap());
    let exp = 2.0 * 3.0 * 4.0 * 0.125;
    assert!((v - exp).abs() < 1e-9);
}

#[test]
fn scale_rejects_non_positive_factor() {
    let m = Model::new()
        .add(Feature::Box {
            id: "body".into(),
            extents: lits([1.0, 1.0, 1.0]),
        })
        .add(Feature::Scale {
            id: "out".into(),
            input: "body".into(),
            factor: Scalar::lit(0.0),
        });
    assert!(m.evaluate("out").is_err());
}

#[test]
fn scale_round_trips_via_json() {
    let m = Model::new()
        .add(Feature::Box {
            id: "body".into(),
            extents: lits([1.0, 1.0, 1.0]),
        })
        .add(Feature::Scale {
            id: "out".into(),
            input: "body".into(),
            factor: Scalar::expr("0.5 * 2"),
        });
    let json = m.to_json_string().unwrap();
    let m2 = Model::from_json_str(&json).unwrap();
    let v1 = solid_volume(&m.evaluate("out").unwrap());
    let v2 = solid_volume(&m2.evaluate("out").unwrap());
    assert!((v1 - v2).abs() < 1e-9);
}

// -------- Bolt --------

#[test]
fn bolt_volume_matches_head_plus_shaft() {
    let hir = 5.0; // hex apothem
    let ht = 4.0;
    let sr = 2.5;
    let sl = 20.0;
    let m = Model::new().add(Feature::Bolt {
        id: "out".into(),
        head_inscribed_radius: Scalar::lit(hir),
        head_thickness: Scalar::lit(ht),
        shaft_radius: Scalar::lit(sr),
        shaft_length: Scalar::lit(sl),
        segments: 24,
    });
    let v = solid_volume(&m.evaluate("out").unwrap());
    // Head: hexagonal area = 2*sqrt(3)*apothem².
    let head_area = 2.0 * 3.0_f64.sqrt() * hir * hir;
    let head_v = head_area * ht;
    // Shaft: faceted cylinder.
    let shaft_area = 0.5 * 24.0 * sr * sr * (2.0 * std::f64::consts::PI / 24.0).sin();
    let shaft_v = shaft_area * sl;
    let exp = head_v + shaft_v;
    let rel = (v - exp).abs() / exp;
    assert!(rel < 0.05, "v={v}, exp={exp}, rel={rel}");
}

// -------- CapScrew --------

#[test]
fn cap_screw_volume_matches_two_cylinders() {
    let hr = 4.0;
    let ht = 4.0;
    let sr = 2.0;
    let sl = 20.0;
    let m = Model::new().add(Feature::CapScrew {
        id: "out".into(),
        head_radius: Scalar::lit(hr),
        head_thickness: Scalar::lit(ht),
        shaft_radius: Scalar::lit(sr),
        shaft_length: Scalar::lit(sl),
        segments: 24,
    });
    let v = solid_volume(&m.evaluate("out").unwrap());
    let head_area = 0.5 * 24.0 * hr * hr * (2.0 * std::f64::consts::PI / 24.0).sin();
    let shaft_area = 0.5 * 24.0 * sr * sr * (2.0 * std::f64::consts::PI / 24.0).sin();
    let exp = head_area * ht + shaft_area * sl;
    let rel = (v - exp).abs() / exp;
    assert!(rel < 0.02, "v={v}, exp={exp}, rel={rel}");
}

#[test]
fn cap_screw_rejects_head_smaller_than_shaft() {
    let m = Model::new().add(Feature::CapScrew {
        id: "out".into(),
        head_radius: Scalar::lit(1.0),
        head_thickness: Scalar::lit(2.0),
        shaft_radius: Scalar::lit(2.0),
        shaft_length: Scalar::lit(5.0),
        segments: 24,
    });
    assert!(m.evaluate("out").is_err());
}

#[test]
fn fasteners_round_trip_via_json() {
    for f in [
        Feature::Bolt {
            id: "out".into(),
            head_inscribed_radius: Scalar::lit(4.0),
            head_thickness: Scalar::lit(3.0),
            shaft_radius: Scalar::lit(2.0),
            shaft_length: Scalar::lit(15.0),
            segments: 24,
        },
        Feature::CapScrew {
            id: "out".into(),
            head_radius: Scalar::lit(3.5),
            head_thickness: Scalar::lit(3.0),
            shaft_radius: Scalar::lit(2.0),
            shaft_length: Scalar::lit(15.0),
            segments: 24,
        },
    ] {
        let m = Model::new().add(f);
        let json = m.to_json_string().unwrap();
        let m2 = Model::from_json_str(&json).unwrap();
        let v1 = solid_volume(&m.evaluate("out").unwrap());
        let v2 = solid_volume(&m2.evaluate("out").unwrap());
        assert!((v1 - v2).abs() < 1e-9);
    }
}
