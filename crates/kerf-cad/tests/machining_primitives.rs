//! DovetailSlot and VeeGroove tests.

use kerf_brep::solid_volume;
use kerf_cad::{lits, Feature, Model, Scalar};

#[test]
fn dovetail_slot_volume_matches_trapezoid() {
    let bw = 2.0;
    let tw = 4.0;
    let d = 3.0;
    let l = 5.0;
    let m = Model::new().add(Feature::DovetailSlot {
        id: "out".into(),
        bottom_width: Scalar::lit(bw),
        top_width: Scalar::lit(tw),
        depth: Scalar::lit(d),
        length: Scalar::lit(l),
    });
    let v = solid_volume(&m.evaluate("out").unwrap());
    let area = 0.5 * (bw + tw) * d; // trapezoid area
    let exp = area * l;
    assert!((v - exp).abs() < 1e-9, "v={v}, exp={exp}");
}

#[test]
fn vee_groove_volume_matches_triangle() {
    let tw = 3.0;
    let d = 2.0;
    let l = 5.0;
    let m = Model::new().add(Feature::VeeGroove {
        id: "out".into(),
        top_width: Scalar::lit(tw),
        depth: Scalar::lit(d),
        length: Scalar::lit(l),
    });
    let v = solid_volume(&m.evaluate("out").unwrap());
    let area = 0.5 * tw * d;
    let exp = area * l;
    assert!((v - exp).abs() < 1e-9, "v={v}, exp={exp}");
}

#[test]
fn dovetail_slot_used_as_cutter() {
    // 10×10×5 box with a dovetail slot cut into the top.
    let m = Model::new()
        .add(Feature::Box {
            id: "body".into(),
            extents: lits([10.0, 10.0, 5.0]),
        })
        .add(Feature::DovetailSlot {
            id: "slot".into(),
            bottom_width: Scalar::lit(2.0),
            top_width: Scalar::lit(3.0),
            depth: Scalar::lit(2.0),
            length: Scalar::lit(12.0),
        })
        .add(Feature::Translate {
            id: "slot_pos".into(),
            input: "slot".into(),
            offset: lits([5.0, -1.0, 3.0]),
        })
        .add(Feature::Difference {
            id: "out".into(),
            inputs: vec!["body".into(), "slot_pos".into()],
        });
    let v = solid_volume(&m.evaluate("out").unwrap());
    let body_v = 10.0 * 10.0 * 5.0;
    // Slot has trapezoid area = (2+3)/2 * 2 = 5, length within body = 10
    // (extends from y=-1 to y=11, body is y=0..10, intersection length = 10).
    let removed = 5.0 * 10.0;
    let exp = body_v - removed;
    let rel = (v - exp).abs() / exp;
    assert!(rel < 0.01, "v={v}, exp={exp}, rel={rel}");
}

#[test]
fn machining_primitives_round_trip_via_json() {
    for f in [
        Feature::DovetailSlot {
            id: "out".into(),
            bottom_width: Scalar::lit(2.0),
            top_width: Scalar::lit(3.0),
            depth: Scalar::lit(1.5),
            length: Scalar::lit(4.0),
        },
        Feature::VeeGroove {
            id: "out".into(),
            top_width: Scalar::lit(3.0),
            depth: Scalar::lit(2.0),
            length: Scalar::lit(5.0),
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
