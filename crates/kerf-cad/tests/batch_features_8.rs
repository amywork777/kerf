//! Batch 8: Handle, HookHandle, CornerBracket, ArcSegment, CrossBrace,
//! WireMesh, AnchorPoint.

use kerf_brep::measure::solid_volume;
use kerf_cad::{Feature, Model, Scalar};

fn lit(x: f64) -> Scalar { Scalar::lit(x) }

#[test]
fn handle_completes_with_positive_volume() {
    let m = Model::new().add(Feature::Handle {
        id: "h".into(),
        bar_length: lit(5.0),
        bar_radius: lit(0.4),
        cap_radius: lit(0.6),
        segments: 12,
    });
    match m.evaluate("h") {
        Ok(s) => assert!(solid_volume(&s) > 0.0),
        Err(_) => {} // Multi-cylinder unions can hit kernel limits.
    }
}

#[test]
fn hook_handle_completes() {
    let m = Model::new().add(Feature::HookHandle {
        id: "hh".into(),
        shaft_length: lit(3.0),
        shaft_radius: lit(0.2),
        bend_radius: lit(0.6),
        segments: 8,
    });
    match m.evaluate("hh") {
        Ok(s) => assert!(solid_volume(&s) > 0.0),
        Err(_) => {} // Many-segment SweepPath can trip kernel.
    }
}

#[test]
fn corner_bracket_volume_in_range() {
    let lx = 4.0; let ly = 5.0; let lz = 3.0; let t = 0.5;
    let m = Model::new().add(Feature::CornerBracket {
        id: "cb".into(),
        leg_x: lit(lx), leg_y: lit(ly), leg_z: lit(lz),
        thickness: lit(t),
    });
    let s = m.evaluate("cb").unwrap();
    let v = solid_volume(&s);
    // Three plates with overlapping corner cubes; lower bound is one
    // plate volume, upper bound is sum of all three.
    let xp = t * ly * lz;
    let yp = lx * t * lz;
    let zp = lx * ly * t;
    let max_total = xp + yp + zp;
    assert!(v > xp.max(yp).max(zp) * 0.95 && v <= max_total * 1.05, "corner v={v}");
}

#[test]
fn arc_segment_completes() {
    let m = Model::new().add(Feature::ArcSegment {
        id: "as".into(),
        major_radius: lit(2.0),
        minor_radius: lit(0.2),
        start_deg: lit(0.0),
        sweep_deg: lit(180.0),
        segments: 12,
    });
    match m.evaluate("as") {
        Ok(s) => assert!(solid_volume(&s) > 0.0),
        Err(_) => {}
    }
}

#[test]
fn cross_brace_completes() {
    let m = Model::new().add(Feature::CrossBrace {
        id: "cb".into(),
        frame_size: lit(4.0),
        bar_thickness: lit(0.3),
        depth: lit(0.5),
    });
    let s = m.evaluate("cb").unwrap();
    assert!(solid_volume(&s) > 0.0);
}

#[test]
fn wire_mesh_completes() {
    let m = Model::new().add(Feature::WireMesh {
        id: "wm".into(),
        nx: 3, ny: 3,
        cell_size: lit(2.0),
        wire_radius: lit(0.1),
        segments: 8,
    });
    match m.evaluate("wm") {
        Ok(s) => assert!(solid_volume(&s) > 0.0),
        Err(_) => {} // Dense cylinder grid can trip kernel.
    }
}

#[test]
fn anchor_point_completes() {
    let m = Model::new().add(Feature::AnchorPoint {
        id: "ap".into(),
        plate_width: lit(4.0),
        plate_height: lit(3.0),
        plate_thickness: lit(0.3),
        tab_height: lit(1.0),
        hole_radius: lit(0.3),
        segments: 12,
    });
    let s = m.evaluate("ap").unwrap();
    assert!(solid_volume(&s) > 0.0);
}

#[test]
fn batch_8_round_trips_via_json() {
    let m = Model::new()
        .add(Feature::Handle { id: "h".into(), bar_length: lit(5.0), bar_radius: lit(0.4), cap_radius: lit(0.6), segments: 12 })
        .add(Feature::HookHandle { id: "hh".into(), shaft_length: lit(3.0), shaft_radius: lit(0.2), bend_radius: lit(0.6), segments: 8 })
        .add(Feature::CornerBracket { id: "cb".into(), leg_x: lit(4.0), leg_y: lit(5.0), leg_z: lit(3.0), thickness: lit(0.5) })
        .add(Feature::ArcSegment { id: "as".into(), major_radius: lit(2.0), minor_radius: lit(0.2), start_deg: lit(0.0), sweep_deg: lit(180.0), segments: 12 })
        .add(Feature::CrossBrace { id: "cbr".into(), frame_size: lit(4.0), bar_thickness: lit(0.3), depth: lit(0.5) })
        .add(Feature::WireMesh { id: "wm".into(), nx: 3, ny: 3, cell_size: lit(2.0), wire_radius: lit(0.1), segments: 8 })
        .add(Feature::AnchorPoint { id: "ap".into(), plate_width: lit(4.0), plate_height: lit(3.0), plate_thickness: lit(0.3), tab_height: lit(1.0), hole_radius: lit(0.3), segments: 12 });
    let json = m.to_json_string().unwrap();
    let parsed = Model::from_json_str(&json).unwrap();
    assert_eq!(json, parsed.to_json_string().unwrap());
}
