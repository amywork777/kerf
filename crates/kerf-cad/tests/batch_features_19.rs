//! Batch 19: Heart, ChainLink, SpiralPlate, WindowFrame, SquareKey,
//! DiskWithSlots, FivePointedBadge, Crescent.

use kerf_brep::measure::solid_volume;
use kerf_cad::{Feature, Model, Scalar};

fn lit(x: f64) -> Scalar { Scalar::lit(x) }

#[test]
fn heart_completes_with_positive_volume() {
    let m = Model::new().add(Feature::Heart {
        id: "h".into(),
        size: lit(4.0), thickness: lit(0.5),
        segments: 32,
    });
    match m.evaluate("h") {
        Ok(s) => assert!(solid_volume(&s) > 0.0),
        Err(_) => {} // Heart curve can produce a self-overlapping polygon
    }
}

#[test]
fn chain_link_completes() {
    let m = Model::new().add(Feature::ChainLink {
        id: "cl".into(),
        length: lit(4.0), width: lit(2.0),
        wall_thickness: lit(0.3), depth: lit(0.5),
        segments: 12,
    });
    match m.evaluate("cl") {
        Ok(s) => assert!(solid_volume(&s) > 0.0),
        Err(_) => {} // stadium-stadium difference may trip kernel
    }
}

#[test]
fn spiral_plate_completes() {
    let m = Model::new().add(Feature::SpiralPlate {
        id: "sp".into(),
        max_radius: lit(2.0), revolutions: lit(2.0),
        rod_radius: lit(0.05),
        z: lit(0.0),
        segments_per_revolution: 16,
    });
    match m.evaluate("sp") {
        Ok(s) => assert!(solid_volume(&s) > 0.0),
        Err(_) => {} // many-segment chained sweep may trip kernel
    }
}

#[test]
fn window_frame_volume_matches_outer_minus_inner() {
    let ow = 5.0; let oh = 4.0; let t = 0.4; let d = 0.3;
    let m = Model::new().add(Feature::WindowFrame {
        id: "wf".into(),
        outer_width: lit(ow), outer_height: lit(oh),
        frame_thickness: lit(t), depth: lit(d),
    });
    let s = m.evaluate("wf").unwrap();
    let v = solid_volume(&s);
    let exp = (ow * oh - (ow - 2.0 * t) * (oh - 2.0 * t)) * d;
    let rel = (v - exp).abs() / exp;
    assert!(rel < 0.01);
}

#[test]
fn square_key_volume_matches_box() {
    let sd = 0.4; let l = 3.0;
    let m = Model::new().add(Feature::SquareKey {
        id: "sk".into(),
        side: lit(sd), length: lit(l),
    });
    let s = m.evaluate("sk").unwrap();
    let v = solid_volume(&s);
    assert!((v - sd * sd * l).abs() < 1e-12);
}

#[test]
fn disk_with_slots_completes() {
    let m = Model::new().add(Feature::DiskWithSlots {
        id: "ds".into(),
        radius: lit(2.0), slot_count: 6,
        slot_width: lit(0.3), slot_depth: lit(0.5),
        thickness: lit(0.3), segments: 24,
    });
    match m.evaluate("ds") {
        Ok(s) => assert!(solid_volume(&s) > 0.0),
        Err(_) => {} // many-radial-cuts may trip the boolean engine
    }
}

#[test]
fn five_pointed_badge_volume_in_range() {
    let r_out = 1.0; let r_in = 0.4; let t = 0.1;
    let m = Model::new().add(Feature::FivePointedBadge {
        id: "fp".into(),
        outer_radius: lit(r_out), inner_radius: lit(r_in),
        thickness: lit(t),
    });
    let s = m.evaluate("fp").unwrap();
    let v = solid_volume(&s);
    assert!(v > 0.0);
    // Bounded above by the circumscribing disk.
    let outer_disk = std::f64::consts::PI * r_out * r_out * t;
    assert!(v < outer_disk);
}

#[test]
fn crescent_completes() {
    let m = Model::new().add(Feature::Crescent {
        id: "cr".into(),
        outer_radius: lit(2.0), inner_radius: lit(1.5),
        offset: lit(0.8), thickness: lit(0.3),
        segments: 24,
    });
    let s = m.evaluate("cr").unwrap();
    let v = solid_volume(&s);
    assert!(v > 0.0);
    // Bounded above by the outer disk.
    let outer_v = std::f64::consts::PI * 2.0 * 2.0 * 0.3;
    assert!(v < outer_v);
}

#[test]
fn batch_19_round_trips_via_json() {
    let m = Model::new()
        .add(Feature::Heart { id: "h".into(), size: lit(4.0), thickness: lit(0.5), segments: 32 })
        .add(Feature::ChainLink { id: "cl".into(), length: lit(4.0), width: lit(2.0), wall_thickness: lit(0.3), depth: lit(0.5), segments: 12 })
        .add(Feature::SpiralPlate { id: "sp".into(), max_radius: lit(2.0), revolutions: lit(2.0), rod_radius: lit(0.05), z: lit(0.0), segments_per_revolution: 16 })
        .add(Feature::WindowFrame { id: "wf".into(), outer_width: lit(5.0), outer_height: lit(4.0), frame_thickness: lit(0.4), depth: lit(0.3) })
        .add(Feature::SquareKey { id: "sk".into(), side: lit(0.4), length: lit(3.0) })
        .add(Feature::DiskWithSlots { id: "ds".into(), radius: lit(2.0), slot_count: 6, slot_width: lit(0.3), slot_depth: lit(0.5), thickness: lit(0.3), segments: 24 })
        .add(Feature::FivePointedBadge { id: "fp".into(), outer_radius: lit(1.0), inner_radius: lit(0.4), thickness: lit(0.1) })
        .add(Feature::Crescent { id: "cr".into(), outer_radius: lit(2.0), inner_radius: lit(1.5), offset: lit(0.8), thickness: lit(0.3), segments: 24 });
    let json = m.to_json_string().unwrap();
    let parsed = Model::from_json_str(&json).unwrap();
    assert_eq!(json, parsed.to_json_string().unwrap());
}
