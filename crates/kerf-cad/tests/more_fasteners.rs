//! Nut, Washer, RoundBoss, RectBoss feature tests.

use kerf_brep::solid_volume;
use kerf_cad::{lits, Feature, Model, Scalar};

#[test]
fn nut_volume_matches_hex_minus_bore() {
    let ir = 4.0;
    let br = 1.5;
    let t = 3.0;
    let m = Model::new().add(Feature::Nut {
        id: "out".into(),
        inscribed_radius: Scalar::lit(ir),
        bore_radius: Scalar::lit(br),
        thickness: Scalar::lit(t),
        segments: 24,
    });
    let v = solid_volume(&m.evaluate("out").unwrap());
    let hex_area = 2.0 * 3.0_f64.sqrt() * ir * ir;
    let bore_area = 0.5 * 24.0 * br * br * (2.0 * std::f64::consts::PI / 24.0).sin();
    let exp = (hex_area - bore_area) * t;
    let rel = (v - exp).abs() / exp;
    assert!(rel < 0.02, "v={v}, exp={exp}, rel={rel}");
}

#[test]
fn nut_rejects_oversized_bore() {
    let m = Model::new().add(Feature::Nut {
        id: "out".into(),
        inscribed_radius: Scalar::lit(2.0),
        bore_radius: Scalar::lit(2.0),
        thickness: Scalar::lit(1.0),
        segments: 24,
    });
    assert!(m.evaluate("out").is_err());
}

#[test]
fn washer_volume_matches_annulus_extrude() {
    let r_out = 5.0;
    let r_in = 2.5;
    let t = 1.0;
    let m = Model::new().add(Feature::Washer {
        id: "out".into(),
        outer_radius: Scalar::lit(r_out),
        inner_radius: Scalar::lit(r_in),
        thickness: Scalar::lit(t),
        segments: 32,
    });
    let v = solid_volume(&m.evaluate("out").unwrap());
    let area_o = 0.5 * 32.0 * r_out * r_out * (2.0 * std::f64::consts::PI / 32.0).sin();
    let area_i = 0.5 * 32.0 * r_in * r_in * (2.0 * std::f64::consts::PI / 32.0).sin();
    let exp = (area_o - area_i) * t;
    let rel = (v - exp).abs() / exp;
    assert!(rel < 0.01, "v={v}, exp={exp}, rel={rel}");
}

#[test]
fn round_boss_at_offset_volume_correct() {
    let m = Model::new().add(Feature::RoundBoss {
        id: "out".into(),
        base: lits([10.0, 5.0, 2.0]),
        radius: Scalar::lit(2.0),
        height: Scalar::lit(3.0),
        segments: 24,
    });
    let v = solid_volume(&m.evaluate("out").unwrap());
    let area = 0.5 * 24.0 * 2.0 * 2.0 * (2.0 * std::f64::consts::PI / 24.0).sin();
    let exp = area * 3.0;
    assert!((v - exp).abs() < 1e-6, "v={v}");
}

#[test]
fn rect_boss_at_offset_volume_correct() {
    let m = Model::new().add(Feature::RectBoss {
        id: "out".into(),
        corner: lits([1.0, 2.0, 3.0]),
        extents: lits([4.0, 5.0, 6.0]),
    });
    let v = solid_volume(&m.evaluate("out").unwrap());
    assert!((v - 120.0).abs() < 1e-6, "v={v}");
}

#[test]
fn boss_features_compose_with_union() {
    // Plate + cylindrical boss riser = simple flange.
    let m = Model::new()
        .add(Feature::Box {
            id: "plate".into(),
            extents: lits([20.0, 20.0, 4.0]),
        })
        .add(Feature::RoundBoss {
            id: "riser".into(),
            base: lits([10.0, 10.0, 4.0]),
            radius: Scalar::lit(3.0),
            height: Scalar::lit(5.0),
            segments: 24,
        })
        .add(Feature::Union {
            id: "out".into(),
            inputs: vec!["plate".into(), "riser".into()],
        });
    let v = solid_volume(&m.evaluate("out").unwrap());
    let plate_v = 20.0 * 20.0 * 4.0;
    let riser_area = 0.5 * 24.0 * 9.0 * (2.0 * std::f64::consts::PI / 24.0).sin();
    let riser_v = riser_area * 5.0;
    let exp = plate_v + riser_v;
    let rel = (v - exp).abs() / exp;
    assert!(rel < 0.02, "v={v}, exp={exp}, rel={rel}");
}

#[test]
fn nut_washer_round_trip_via_json() {
    for f in [
        Feature::Nut {
            id: "out".into(),
            inscribed_radius: Scalar::lit(4.0),
            bore_radius: Scalar::lit(1.5),
            thickness: Scalar::lit(3.0),
            segments: 24,
        },
        Feature::Washer {
            id: "out".into(),
            outer_radius: Scalar::lit(5.0),
            inner_radius: Scalar::lit(2.5),
            thickness: Scalar::lit(1.0),
            segments: 32,
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
