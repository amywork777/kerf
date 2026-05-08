//! Structural cross-section primitives: LBracket, UChannel, TBeam.

use kerf_brep::solid_volume;
use kerf_cad::{Feature, Model, Scalar};

#[test]
fn l_bracket_volume_matches_two_rectangles() {
    let w = 4.0;
    let h = 3.0;
    let t = 0.5;
    let d = 5.0;
    let m = Model::new().add(Feature::LBracket {
        id: "out".into(),
        width: Scalar::lit(w),
        height: Scalar::lit(h),
        thickness: Scalar::lit(t),
        depth: Scalar::lit(d),
    });
    let s = m.evaluate("out").unwrap();
    let v = solid_volume(&s);
    // Cross-section area = w*t + (h-t)*t = t*(w + h - t)
    let area = t * (w + h - t);
    let exp = area * d;
    assert!((v - exp).abs() < 1e-6, "v={v}, exp={exp}");
}

#[test]
fn u_channel_volume_matches_three_rectangles() {
    let w = 5.0;
    let h = 4.0;
    let t = 0.5;
    let d = 6.0;
    let m = Model::new().add(Feature::UChannel {
        id: "out".into(),
        width: Scalar::lit(w),
        height: Scalar::lit(h),
        thickness: Scalar::lit(t),
        depth: Scalar::lit(d),
    });
    let s = m.evaluate("out").unwrap();
    let v = solid_volume(&s);
    // Cross-section area = total area minus the cavity:
    //   w*h - (w - 2*t)*(h - t)
    let area = w * h - (w - 2.0 * t) * (h - t);
    let exp = area * d;
    assert!((v - exp).abs() < 1e-6, "v={v}, exp={exp}");
}

#[test]
fn t_beam_volume_matches_two_rectangles() {
    let fw = 6.0;
    let ft = 1.0;
    let wt = 1.5;
    let th = 5.0;
    let d = 8.0;
    let m = Model::new().add(Feature::TBeam {
        id: "out".into(),
        flange_width: Scalar::lit(fw),
        flange_thickness: Scalar::lit(ft),
        web_thickness: Scalar::lit(wt),
        total_height: Scalar::lit(th),
        depth: Scalar::lit(d),
    });
    let s = m.evaluate("out").unwrap();
    let v = solid_volume(&s);
    // Cross-section: flange (fw × ft) + web (wt × (th - ft))
    let area = fw * ft + wt * (th - ft);
    let exp = area * d;
    assert!((v - exp).abs() < 1e-6, "v={v}, exp={exp}");
}

#[test]
fn l_bracket_rejects_thickness_too_large() {
    let m = Model::new().add(Feature::LBracket {
        id: "out".into(),
        width: Scalar::lit(2.0),
        height: Scalar::lit(2.0),
        thickness: Scalar::lit(2.0),
        depth: Scalar::lit(1.0),
    });
    assert!(m.evaluate("out").is_err());
}

#[test]
fn u_channel_rejects_thick_walls() {
    let m = Model::new().add(Feature::UChannel {
        id: "out".into(),
        width: Scalar::lit(2.0),
        height: Scalar::lit(3.0),
        thickness: Scalar::lit(1.0),
        depth: Scalar::lit(1.0),
    });
    assert!(m.evaluate("out").is_err());
}

#[test]
fn t_beam_rejects_thick_web() {
    let m = Model::new().add(Feature::TBeam {
        id: "out".into(),
        flange_width: Scalar::lit(3.0),
        flange_thickness: Scalar::lit(0.5),
        web_thickness: Scalar::lit(3.5),
        total_height: Scalar::lit(4.0),
        depth: Scalar::lit(2.0),
    });
    assert!(m.evaluate("out").is_err());
}

#[test]
fn structural_shapes_round_trip_via_json() {
    for f in [
        Feature::LBracket {
            id: "out".into(),
            width: Scalar::lit(3.0),
            height: Scalar::lit(2.0),
            thickness: Scalar::lit(0.4),
            depth: Scalar::lit(5.0),
        },
        Feature::UChannel {
            id: "out".into(),
            width: Scalar::lit(4.0),
            height: Scalar::lit(3.0),
            thickness: Scalar::lit(0.5),
            depth: Scalar::lit(5.0),
        },
        Feature::TBeam {
            id: "out".into(),
            flange_width: Scalar::lit(5.0),
            flange_thickness: Scalar::lit(0.8),
            web_thickness: Scalar::lit(1.2),
            total_height: Scalar::lit(4.0),
            depth: Scalar::lit(6.0),
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
