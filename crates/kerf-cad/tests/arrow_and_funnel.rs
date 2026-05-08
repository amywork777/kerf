//! Arrow and Funnel composite primitive tests.

use kerf_brep::solid_volume;
use kerf_cad::{Feature, Model, Scalar};

#[test]
fn arrow_volume_is_shaft_plus_tip() {
    let sr = 0.5;
    let sl = 4.0;
    let tl = 1.5;
    let segments = 32;
    let m = Model::new().add(Feature::Arrow {
        id: "out".into(),
        shaft_radius: Scalar::lit(sr),
        shaft_length: Scalar::lit(sl),
        tip_length: Scalar::lit(tl),
        segments,
    });
    let v = solid_volume(&m.evaluate("out").unwrap());
    let n = segments as f64;
    let area = 0.5 * n * sr * sr * (2.0 * std::f64::consts::PI / n).sin();
    let shaft_v = area * sl;
    let tip_v = (1.0 / 3.0) * area * tl;
    let exp = shaft_v + tip_v;
    let rel = (v - exp).abs() / exp;
    assert!(rel < 0.02, "v={v}, exp={exp}, rel={rel}");
}

#[test]
fn arrow_round_trips_via_json() {
    let m = Model::new().add(Feature::Arrow {
        id: "out".into(),
        shaft_radius: Scalar::lit(0.5),
        shaft_length: Scalar::lit(4.0),
        tip_length: Scalar::lit(1.5),
        segments: 16,
    });
    let json = m.to_json_string().unwrap();
    let m2 = Model::from_json_str(&json).unwrap();
    let v1 = solid_volume(&m.evaluate("out").unwrap());
    let v2 = solid_volume(&m2.evaluate("out").unwrap());
    assert!((v1 - v2).abs() < 1e-9);
}

#[test]
fn funnel_volume_is_frustum_plus_spout() {
    let tr = 3.0;
    let nr = 1.0;
    let nz = 4.0;
    let sl = 2.0;
    let segments = 32;
    let m = Model::new().add(Feature::Funnel {
        id: "out".into(),
        top_radius: Scalar::lit(tr),
        neck_radius: Scalar::lit(nr),
        neck_z: Scalar::lit(nz),
        spout_length: Scalar::lit(sl),
        segments,
    });
    let v = solid_volume(&m.evaluate("out").unwrap());
    let n = segments as f64;
    let area_of = |r: f64| 0.5 * n * r * r * (2.0 * std::f64::consts::PI / n).sin();
    let a_top = area_of(tr);
    let a_neck = area_of(nr);
    let frustum_v = nz / 3.0 * (a_top + a_neck + (a_top * a_neck).sqrt());
    let spout_v = a_neck * sl;
    let exp = frustum_v + spout_v;
    let rel = (v - exp).abs() / exp;
    assert!(rel < 0.02, "v={v}, exp={exp}, rel={rel}");
}

#[test]
fn funnel_rejects_neck_geq_top() {
    let m = Model::new().add(Feature::Funnel {
        id: "out".into(),
        top_radius: Scalar::lit(1.0),
        neck_radius: Scalar::lit(1.0),
        neck_z: Scalar::lit(2.0),
        spout_length: Scalar::lit(1.0),
        segments: 16,
    });
    assert!(m.evaluate("out").is_err());
}
