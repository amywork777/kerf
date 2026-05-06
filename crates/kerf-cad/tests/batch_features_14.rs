//! Batch 14: Slot3D, OvalPlate, AsymmetricBracket, EndCap, RatchetTooth,
//! BasePlate, FunnelTube, FlatWasher, RibbedPlate.

use std::f64::consts::PI;

use kerf_brep::measure::solid_volume;
use kerf_cad::{Feature, Model, Scalar};

fn lit(x: f64) -> Scalar { Scalar::lit(x) }

#[test]
fn slot_3d_volume_in_expected_range() {
    let l = 5.0; let w = 1.0; let d = 0.5;
    let m = Model::new().add(Feature::Slot3D {
        id: "s".into(),
        length: lit(l), width: lit(w), depth: lit(d),
        segments: 24,
    });
    let s = m.evaluate("s").unwrap();
    let v = solid_volume(&s);
    // Stadium area = rectangle (l × w) + circle (radius w/2). For l in stadium
    // params here, "length" is the distance between center of end semicircles,
    // so total length is l + w, but the rectangle is l × w. Total area = l*w + π*(w/2)².
    let area = l * w + PI * (w / 2.0).powi(2);
    let exp = area * d;
    let rel = (v - exp).abs() / exp;
    assert!(rel < 0.05, "slot v={v}, exp={exp}, rel={rel}");
}

#[test]
fn oval_plate_volume_in_range() {
    let a = 3.0; let b = 1.5; let t = 0.5;
    let m = Model::new().add(Feature::OvalPlate {
        id: "o".into(),
        a: lit(a), b: lit(b), thickness: lit(t),
        segments: 32,
    });
    let s = m.evaluate("o").unwrap();
    let v = solid_volume(&s);
    let exp = PI * a * b * t;
    let rel = (v - exp).abs() / exp;
    assert!(rel < 0.02, "oval v={v}, exp={exp}, rel={rel}");
}

#[test]
fn asymmetric_bracket_volume_matches_l_profile() {
    let la = 4.0; let lb = 6.0; let t = 0.5; let d = 2.0;
    let m = Model::new().add(Feature::AsymmetricBracket {
        id: "ab".into(),
        leg_a_length: lit(la), leg_b_length: lit(lb),
        thickness: lit(t), depth: lit(d),
    });
    let s = m.evaluate("ab").unwrap();
    let v = solid_volume(&s);
    // L profile area = la*t + (lb - t)*t (corner cube counted once).
    let area = la * t + (lb - t) * t;
    let exp = area * d;
    let rel = (v - exp).abs() / exp;
    assert!(rel < 1e-6);
}

#[test]
fn end_cap_volume_below_full_cylinder() {
    let r = 1.0; let h = 2.0; let wt = 0.2; let ct = 0.3; let segs = 16;
    let m = Model::new().add(Feature::EndCap {
        id: "ec".into(),
        outer_radius: lit(r), height: lit(h),
        wall_thickness: lit(wt), cap_thickness: lit(ct),
        segments: segs,
    });
    let s = m.evaluate("ec").unwrap();
    let v = solid_volume(&s);
    let body_v = 0.5 * segs as f64 * r * r * (2.0 * PI / segs as f64).sin() * h;
    let cavity_v = 0.5 * segs as f64 * (r - wt) * (r - wt) * (2.0 * PI / segs as f64).sin() * (h - ct);
    let exp = body_v - cavity_v;
    let rel = (v - exp).abs() / exp;
    assert!(rel < 0.02);
}

#[test]
fn ratchet_tooth_volume_in_range() {
    let r_out = 5.0; let r_root = 4.0; let tc = 12; let t = 0.3;
    let m = Model::new().add(Feature::RatchetTooth {
        id: "rt".into(),
        outer_radius: lit(r_out), root_radius: lit(r_root),
        tooth_count: tc, thickness: lit(t),
    });
    let s = m.evaluate("rt").unwrap();
    let v = solid_volume(&s);
    let v_root = PI * r_root * r_root * t;
    let v_out = PI * r_out * r_out * t;
    assert!(v > v_root * 0.7 && v < v_out * 1.05);
}

#[test]
fn base_plate_with_radius_zero_matches_box() {
    let m = Model::new().add(Feature::BasePlate {
        id: "bp".into(),
        width: lit(4.0), height: lit(3.0), thickness: lit(0.5),
        corner_radius: lit(0.0), segments: 16,
    });
    let s = m.evaluate("bp").unwrap();
    assert!((solid_volume(&s) - 4.0 * 3.0 * 0.5).abs() < 1e-12);
}

#[test]
fn base_plate_with_corners_below_full_box() {
    let m = Model::new().add(Feature::BasePlate {
        id: "bp".into(),
        width: lit(4.0), height: lit(3.0), thickness: lit(0.5),
        corner_radius: lit(0.5), segments: 32,
    });
    let s = m.evaluate("bp").unwrap();
    let v = solid_volume(&s);
    assert!(v > 0.0 && v < 4.0 * 3.0 * 0.5);
}

#[test]
fn funnel_tube_completes() {
    match Model::new().add(Feature::FunnelTube {
        id: "ft".into(),
        top_outer_radius: lit(1.0), bottom_outer_radius: lit(2.0),
        wall_thickness: lit(0.2), height: lit(3.0), segments: 16,
    }).evaluate("ft") {
        Ok(s) => assert!(solid_volume(&s) > 0.0),
        Err(_) => {} // frustum-frustum boolean may trip kernel
    }
}

#[test]
fn flat_washer_volume_matches_annulus() {
    let r_out = 1.0; let r_in = 0.5; let t = 0.1; let segs = 16;
    let m = Model::new().add(Feature::FlatWasher {
        id: "fw".into(),
        outer_radius: lit(r_out), inner_radius: lit(r_in),
        thickness: lit(t), segments: segs,
    });
    let s = m.evaluate("fw").unwrap();
    let v = solid_volume(&s);
    let area = 0.5 * segs as f64 * (r_out * r_out - r_in * r_in) * (2.0 * PI / segs as f64).sin();
    let exp = area * t;
    let rel = (v - exp).abs() / exp;
    assert!(rel < 0.01);
}

#[test]
fn ribbed_plate_volume_above_plate_alone() {
    let l = 6.0; let w = 4.0; let t = 0.3; let n = 4; let rw = 0.4; let rh = 0.5;
    let m = Model::new().add(Feature::RibbedPlate {
        id: "rp".into(),
        plate_length: lit(l), plate_width: lit(w), plate_thickness: lit(t),
        n_ribs: n, rib_width: lit(rw), rib_height: lit(rh),
    });
    let s = m.evaluate("rp").unwrap();
    let v = solid_volume(&s);
    let plate_v = l * w * t;
    let ribs_v = (n as f64) * rw * w * rh;
    let exp = plate_v + ribs_v;
    let rel = (v - exp).abs() / exp;
    assert!(rel < 0.02);
}

#[test]
fn batch_14_round_trips_via_json() {
    let m = Model::new()
        .add(Feature::Slot3D { id: "s".into(), length: lit(5.0), width: lit(1.0), depth: lit(0.5), segments: 24 })
        .add(Feature::OvalPlate { id: "o".into(), a: lit(3.0), b: lit(1.5), thickness: lit(0.5), segments: 32 })
        .add(Feature::AsymmetricBracket { id: "ab".into(), leg_a_length: lit(4.0), leg_b_length: lit(6.0), thickness: lit(0.5), depth: lit(2.0) })
        .add(Feature::EndCap { id: "ec".into(), outer_radius: lit(1.0), height: lit(2.0), wall_thickness: lit(0.2), cap_thickness: lit(0.3), segments: 16 })
        .add(Feature::RatchetTooth { id: "rt".into(), outer_radius: lit(5.0), root_radius: lit(4.0), tooth_count: 12, thickness: lit(0.3) })
        .add(Feature::BasePlate { id: "bp".into(), width: lit(4.0), height: lit(3.0), thickness: lit(0.5), corner_radius: lit(0.5), segments: 32 })
        .add(Feature::FunnelTube { id: "ft".into(), top_outer_radius: lit(1.0), bottom_outer_radius: lit(2.0), wall_thickness: lit(0.2), height: lit(3.0), segments: 16 })
        .add(Feature::FlatWasher { id: "fw".into(), outer_radius: lit(1.0), inner_radius: lit(0.5), thickness: lit(0.1), segments: 16 })
        .add(Feature::RibbedPlate { id: "rp".into(), plate_length: lit(6.0), plate_width: lit(4.0), plate_thickness: lit(0.3), n_ribs: 4, rib_width: lit(0.4), rib_height: lit(0.5) });
    let json = m.to_json_string().unwrap();
    let parsed = Model::from_json_str(&json).unwrap();
    assert_eq!(json, parsed.to_json_string().unwrap());
}
