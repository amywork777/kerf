//! Manufacturing batch 5: ChamferedHole, ThreadedHoleMarker, BoltPattern,
//! SquareDrive, RaisedBoss.

use std::f64::consts::PI;

use kerf_brep::solid_volume;
use kerf_cad::{lits, Feature, Model, Scalar};

fn lit(x: f64) -> Scalar {
    Scalar::lit(x)
}

/// Build a 10×10×10 box named "body" plus one more feature.
fn body_model(feat: Feature) -> Model {
    Model::new()
        .add(Feature::Box {
            id: "body".into(),
            extents: lits([10.0, 10.0, 10.0]),
        })
        .add(feat)
}

// ---------------------------------------------------------------------------
// ChamferedHole
// ---------------------------------------------------------------------------

#[test]
fn chamfered_hole_removes_more_than_bore_alone() {
    // A bore of r=1.5, d=8 has volume π*r²*d ≈ 56.5.
    // Adding a chamfer (frustum) of bottom_r=1.5, top_r=2.5, h=1
    // should remove strictly more material.
    let m = body_model(Feature::ChamferedHole {
        id: "ch".into(),
        input: "body".into(),
        center: lits([5.0, 5.0, 10.0]),
        axis: "z".into(),
        hole_radius: lit(1.5),
        hole_depth: lit(8.0),
        chamfer_radius: lit(2.5),
        chamfer_depth: lit(1.0),
    });
    let solid = m.evaluate("ch").unwrap();
    let v = solid_volume(&solid);

    // Plain box volume: 10^3 = 1000.
    let box_vol = 1000.0_f64;
    // Bore cylinder volume (faceted 16 sides, but approximate analytically):
    let bore_vol = PI * 1.5_f64.powi(2) * 8.0;
    // Result must be less than box_vol - bore_vol (chamfer removes more).
    assert!(
        v < box_vol - bore_vol,
        "expected v < box - bore, got v={v}, box={box_vol}, bore={bore_vol}"
    );
    // But not by more than bore + chamfer-frustum volume (conservative upper bound).
    let chamfer_frustum_vol = PI * 1.0 * (2.5_f64.powi(2) + 2.5 * 1.5 + 1.5_f64.powi(2)) / 3.0;
    assert!(
        v > box_vol - bore_vol - chamfer_frustum_vol - 5.0,
        "chamfer removed unreasonably much: v={v}"
    );
    assert!(v > 0.0, "result must have positive volume");
}

#[test]
fn chamfered_hole_round_trip_json() {
    let m = body_model(Feature::ChamferedHole {
        id: "ch".into(),
        input: "body".into(),
        center: lits([5.0, 5.0, 10.0]),
        axis: "z".into(),
        hole_radius: lit(1.5),
        hole_depth: lit(8.0),
        chamfer_radius: lit(2.5),
        chamfer_depth: lit(1.0),
    });
    let json = m.to_json_string().unwrap();
    let m2 = Model::from_json_str(&json).unwrap();
    let v1 = solid_volume(&m.evaluate("ch").unwrap());
    let v2 = solid_volume(&m2.evaluate("ch").unwrap());
    assert!((v1 - v2).abs() < 1e-9, "v1={v1}, v2={v2}");
}

#[test]
fn chamfered_hole_rejects_bad_params() {
    // chamfer_radius <= hole_radius should fail.
    let m = body_model(Feature::ChamferedHole {
        id: "ch".into(),
        input: "body".into(),
        center: lits([5.0, 5.0, 10.0]),
        axis: "z".into(),
        hole_radius: lit(2.0),
        hole_depth: lit(5.0),
        chamfer_radius: lit(1.5), // < hole_radius
        chamfer_depth: lit(1.0),
    });
    assert!(m.evaluate("ch").is_err(), "should reject chamfer_radius <= hole_radius");
}

// ---------------------------------------------------------------------------
// ThreadedHoleMarker
// ---------------------------------------------------------------------------

#[test]
fn threaded_hole_marker_removes_bore_volume() {
    let td = 3.0_f64;
    let d = 8.0_f64;
    let m = body_model(Feature::ThreadedHoleMarker {
        id: "thm".into(),
        input: "body".into(),
        center: lits([5.0, 5.0, 10.0]),
        axis: "z".into(),
        thread_diameter: lit(td),
        depth: lit(d),
        thread_pitch: lit(0.5),
    });
    let solid = m.evaluate("thm").unwrap();
    let v = solid_volume(&solid);

    // Should remove roughly π*(td/2)²*d material.
    let bore_vol = PI * (td / 2.0).powi(2) * d;
    let box_vol = 1000.0_f64;
    // v must be less than box_vol and approximately box_vol - bore_vol.
    assert!(v < box_vol, "volume must decrease: v={v}");
    // Within 10% of theoretical (faceted approximation).
    let approx = box_vol - bore_vol;
    assert!(
        (v - approx).abs() < approx * 0.12,
        "v={v} not close to {approx}"
    );
}

#[test]
fn threaded_hole_marker_round_trip_json() {
    let m = body_model(Feature::ThreadedHoleMarker {
        id: "thm".into(),
        input: "body".into(),
        center: lits([5.0, 5.0, 10.0]),
        axis: "z".into(),
        thread_diameter: lit(3.0),
        depth: lit(8.0),
        thread_pitch: lit(0.5),
    });
    let json = m.to_json_string().unwrap();
    let m2 = Model::from_json_str(&json).unwrap();
    let v1 = solid_volume(&m.evaluate("thm").unwrap());
    let v2 = solid_volume(&m2.evaluate("thm").unwrap());
    assert!((v1 - v2).abs() < 1e-9, "v1={v1}, v2={v2}");
}

// ---------------------------------------------------------------------------
// BoltPattern
// ---------------------------------------------------------------------------

#[test]
fn bolt_pattern_removes_count_times_bore_volume() {
    let hr = 0.6_f64;
    let hd = 8.0_f64;
    let count = 4usize;
    let m = body_model(Feature::BoltPattern {
        id: "bp".into(),
        input: "body".into(),
        center: lits([5.0, 5.0, 10.0]),
        axis: "z".into(),
        pattern_radius: lit(3.5),
        hole_radius: lit(hr),
        hole_depth: lit(hd),
        count,
        phase: lit(0.0),
    });
    let solid = m.evaluate("bp").unwrap();
    let v = solid_volume(&solid);

    let one_bore = PI * hr * hr * hd;
    let total_bore = one_bore * count as f64;
    let box_vol = 1000.0_f64;
    // Volume must be less than box - total_bore (faceted bores are approximate).
    assert!(v < box_vol - total_bore * 0.8, "should remove ~4 bores: v={v}, expected < {}", box_vol - total_bore * 0.8);
    assert!(v > 0.0, "result must have positive volume");
}

#[test]
fn bolt_pattern_round_trip_json() {
    let m = body_model(Feature::BoltPattern {
        id: "bp".into(),
        input: "body".into(),
        center: lits([5.0, 5.0, 10.0]),
        axis: "z".into(),
        pattern_radius: lit(3.5),
        hole_radius: lit(0.6),
        hole_depth: lit(8.0),
        count: 4,
        phase: lit(0.0),
    });
    let json = m.to_json_string().unwrap();
    let m2 = Model::from_json_str(&json).unwrap();
    let v1 = solid_volume(&m.evaluate("bp").unwrap());
    let v2 = solid_volume(&m2.evaluate("bp").unwrap());
    assert!((v1 - v2).abs() < 1e-9, "v1={v1}, v2={v2}");
}

#[test]
fn bolt_pattern_rejects_zero_count() {
    let m = body_model(Feature::BoltPattern {
        id: "bp".into(),
        input: "body".into(),
        center: lits([5.0, 5.0, 10.0]),
        axis: "z".into(),
        pattern_radius: lit(3.5),
        hole_radius: lit(0.6),
        hole_depth: lit(8.0),
        count: 0,
        phase: lit(0.0),
    });
    assert!(m.evaluate("bp").is_err(), "count=0 must be rejected");
}

// ---------------------------------------------------------------------------
// SquareDrive
// ---------------------------------------------------------------------------

#[test]
fn square_drive_removes_square_pocket_volume() {
    let s = 3.0_f64;
    let d = 6.0_f64;
    let m = body_model(Feature::SquareDrive {
        id: "sd".into(),
        input: "body".into(),
        center: lits([5.0, 5.0, 10.0]),
        axis: "z".into(),
        side_length: lit(s),
        depth: lit(d),
    });
    let solid = m.evaluate("sd").unwrap();
    let v = solid_volume(&solid);

    let pocket_vol = s * s * d;
    let box_vol = 1000.0_f64;
    // Volume must be approximately box - pocket.
    let expected = box_vol - pocket_vol;
    assert!(
        (v - expected).abs() < expected * 0.02,
        "v={v}, expected~{expected}"
    );
}

#[test]
fn square_drive_round_trip_json() {
    let m = body_model(Feature::SquareDrive {
        id: "sd".into(),
        input: "body".into(),
        center: lits([5.0, 5.0, 10.0]),
        axis: "z".into(),
        side_length: lit(3.0),
        depth: lit(6.0),
    });
    let json = m.to_json_string().unwrap();
    let m2 = Model::from_json_str(&json).unwrap();
    let v1 = solid_volume(&m.evaluate("sd").unwrap());
    let v2 = solid_volume(&m2.evaluate("sd").unwrap());
    assert!((v1 - v2).abs() < 1e-9, "v1={v1}, v2={v2}");
}

#[test]
fn square_drive_rejects_zero_depth() {
    let m = body_model(Feature::SquareDrive {
        id: "sd".into(),
        input: "body".into(),
        center: lits([5.0, 5.0, 10.0]),
        axis: "z".into(),
        side_length: lit(3.0),
        depth: lit(0.0),
    });
    assert!(m.evaluate("sd").is_err(), "depth=0 must be rejected");
}

// ---------------------------------------------------------------------------
// RaisedBoss
// ---------------------------------------------------------------------------

#[test]
fn raised_boss_increases_volume_over_input() {
    let br = 2.0_f64;
    let bh = 4.0_f64;
    let hr = 0.8_f64;
    let hd = 3.0_f64;
    let m = body_model(Feature::RaisedBoss {
        id: "rb".into(),
        input: "body".into(),
        center: lits([5.0, 5.0, 10.0]),
        axis: "z".into(),
        boss_radius: lit(br),
        boss_height: lit(bh),
        hole_radius: lit(hr),
        hole_depth: lit(hd),
    });
    let solid = m.evaluate("rb").unwrap();
    let v = solid_volume(&solid);

    // Boss adds cylinder volume, hole subtracts.
    let boss_vol = PI * br * br * bh;
    let hole_vol = PI * hr * hr * hd;
    let net_add = boss_vol - hole_vol;
    let box_vol = 1000.0_f64;
    // Result should be > box_vol (boss adds more than hole removes).
    assert!(
        v > box_vol,
        "RaisedBoss must add net volume: v={v}, box={box_vol}"
    );
    // And within a reasonable range.
    let expected = box_vol + net_add;
    assert!(
        (v - expected).abs() < expected * 0.12,
        "v={v} far from {expected}"
    );
}

#[test]
fn raised_boss_round_trip_json() {
    let m = body_model(Feature::RaisedBoss {
        id: "rb".into(),
        input: "body".into(),
        center: lits([5.0, 5.0, 10.0]),
        axis: "z".into(),
        boss_radius: lit(2.0),
        boss_height: lit(4.0),
        hole_radius: lit(0.8),
        hole_depth: lit(3.0),
    });
    let json = m.to_json_string().unwrap();
    let m2 = Model::from_json_str(&json).unwrap();
    let v1 = solid_volume(&m.evaluate("rb").unwrap());
    let v2 = solid_volume(&m2.evaluate("rb").unwrap());
    assert!((v1 - v2).abs() < 1e-9, "v1={v1}, v2={v2}");
}

#[test]
fn raised_boss_rejects_hole_larger_than_boss() {
    let m = body_model(Feature::RaisedBoss {
        id: "rb".into(),
        input: "body".into(),
        center: lits([5.0, 5.0, 10.0]),
        axis: "z".into(),
        boss_radius: lit(2.0),
        boss_height: lit(4.0),
        hole_radius: lit(2.5), // > boss_radius
        hole_depth: lit(3.0),
    });
    assert!(m.evaluate("rb").is_err(), "hole_radius >= boss_radius must be rejected");
}
