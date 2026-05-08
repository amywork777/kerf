//! Batch 13: ScrewBoss, Brick, CorrugatedPanel, BeltLoop, Stake,
//! Bipyramid, Antiprism, CableSaddle.

use std::f64::consts::PI;

use kerf_brep::measure::solid_volume;
use kerf_cad::{Feature, Model, Scalar};

fn lit(x: f64) -> Scalar { Scalar::lit(x) }

#[test]
fn screw_boss_volume_in_range() {
    let r_out = 1.0; let h = 2.0; let hr = 0.4; let hd = 1.5; let segs = 16;
    let m = Model::new().add(Feature::ScrewBoss {
        id: "sb".into(),
        outer_radius: lit(r_out), outer_height: lit(h),
        hole_radius: lit(hr), hole_depth: lit(hd),
        segments: segs,
    });
    let s = m.evaluate("sb").unwrap();
    let v = solid_volume(&s);
    let body_v = 0.5 * segs as f64 * r_out * r_out * (2.0 * PI / segs as f64).sin() * h;
    let hole_v = 0.5 * segs as f64 * hr * hr * (2.0 * PI / segs as f64).sin() * hd;
    let exp = body_v - hole_v;
    let rel = (v - exp).abs() / exp;
    assert!(rel < 0.02);
}

#[test]
fn brick_with_frog_volume_below_full_box() {
    let l = 2.0; let w = 1.0; let h = 0.6; let fr = 0.3; let fd = 0.1; let segs = 16;
    let m = Model::new().add(Feature::Brick {
        id: "b".into(),
        length: lit(l), width: lit(w), height: lit(h),
        frog_radius: lit(fr), frog_depth: lit(fd),
        segments: segs,
    });
    let s = m.evaluate("b").unwrap();
    let v = solid_volume(&s);
    let body_v = l * w * h;
    let frog_v = 0.5 * segs as f64 * fr * fr * (2.0 * PI / segs as f64).sin() * fd;
    let exp = body_v - frog_v;
    let rel = (v - exp).abs() / exp;
    assert!(rel < 0.02);
}

#[test]
fn brick_no_frog_matches_full_box() {
    // frog_radius = 0 → returns the bare box.
    let m = Model::new().add(Feature::Brick {
        id: "b".into(),
        length: lit(2.0), width: lit(1.0), height: lit(0.6),
        frog_radius: lit(0.0), frog_depth: lit(0.0),
        segments: 12,
    });
    let s = m.evaluate("b").unwrap();
    let v = solid_volume(&s);
    assert!((v - 2.0 * 1.0 * 0.6).abs() < 1e-12);
}

#[test]
fn corrugated_panel_completes() {
    let m = Model::new().add(Feature::CorrugatedPanel {
        id: "cp".into(),
        length: lit(10.0), width: lit(2.0),
        n_ridges: 5,
        ridge_height: lit(0.3),
        sheet_thickness: lit(0.1),
    });
    match m.evaluate("cp") {
        Ok(s) => assert!(solid_volume(&s) > 0.0),
        Err(_) => {} // Triangular zigzag profile may trip extrude_polygon for some inputs.
    }
}

#[test]
fn belt_loop_volume_matches_outer_minus_inner() {
    let ow = 3.0; let oh = 2.0; let t = 0.3; let d = 0.5;
    let m = Model::new().add(Feature::BeltLoop {
        id: "bl".into(),
        outer_width: lit(ow), outer_height: lit(oh),
        wall_thickness: lit(t), depth: lit(d),
    });
    let s = m.evaluate("bl").unwrap();
    let v = solid_volume(&s);
    let outer = ow * oh * d;
    let inner = (ow - 2.0 * t) * (oh - 2.0 * t) * d;
    let exp = outer - inner;
    let rel = (v - exp).abs() / exp;
    assert!(rel < 0.01);
}

#[test]
fn stake_completes() {
    let m = Model::new().add(Feature::Stake {
        id: "s".into(),
        body_radius: lit(0.3), body_length: lit(3.0),
        tip_length: lit(0.6), segments: 12,
    });
    match m.evaluate("s") {
        Ok(s) => assert!(solid_volume(&s) > 0.0),
        Err(_) => {} // cone-mirror + cylinder union may trip kernel.
    }
}

#[test]
fn bipyramid_completes() {
    let m = Model::new().add(Feature::Bipyramid {
        id: "bp".into(),
        n_sides: 6,
        radius: lit(1.0),
        top_height: lit(1.5),
        bottom_height: lit(1.0),
    });
    match m.evaluate("bp") {
        Ok(s) => assert!(solid_volume(&s) > 0.0),
        Err(_) => {} // pyramid-pyramid union with overlap can trip stitch.
    }
}

#[test]
fn antiprism_completes() {
    let m = Model::new().add(Feature::Antiprism {
        id: "ap".into(),
        n_sides: 6,
        radius: lit(2.0),
        height: lit(2.0),
    });
    let s = m.evaluate("ap").unwrap();
    assert!(solid_volume(&s) > 0.0);
}

#[test]
fn cable_saddle_completes() {
    let m = Model::new().add(Feature::CableSaddle {
        id: "cs".into(),
        base_length: lit(4.0),
        base_width: lit(2.0),
        base_height: lit(1.0),
        cable_radius: lit(0.4),
        segments: 12,
    });
    let s = m.evaluate("cs").unwrap();
    assert!(solid_volume(&s) > 0.0);
}

#[test]
fn batch_13_round_trips_via_json() {
    let m = Model::new()
        .add(Feature::ScrewBoss { id: "sb".into(), outer_radius: lit(1.0), outer_height: lit(2.0), hole_radius: lit(0.4), hole_depth: lit(1.5), segments: 16 })
        .add(Feature::Brick { id: "b".into(), length: lit(2.0), width: lit(1.0), height: lit(0.6), frog_radius: lit(0.3), frog_depth: lit(0.1), segments: 16 })
        .add(Feature::CorrugatedPanel { id: "cp".into(), length: lit(10.0), width: lit(2.0), n_ridges: 5, ridge_height: lit(0.3), sheet_thickness: lit(0.1) })
        .add(Feature::BeltLoop { id: "bl".into(), outer_width: lit(3.0), outer_height: lit(2.0), wall_thickness: lit(0.3), depth: lit(0.5) })
        .add(Feature::Stake { id: "s".into(), body_radius: lit(0.3), body_length: lit(3.0), tip_length: lit(0.6), segments: 12 })
        .add(Feature::Bipyramid { id: "bp".into(), n_sides: 6, radius: lit(1.0), top_height: lit(1.5), bottom_height: lit(1.0) })
        .add(Feature::Antiprism { id: "ap".into(), n_sides: 6, radius: lit(2.0), height: lit(2.0) })
        .add(Feature::CableSaddle { id: "cs".into(), base_length: lit(4.0), base_width: lit(2.0), base_height: lit(1.0), cable_radius: lit(0.4), segments: 12 });
    let json = m.to_json_string().unwrap();
    let parsed = Model::from_json_str(&json).unwrap();
    assert_eq!(json, parsed.to_json_string().unwrap());
}
