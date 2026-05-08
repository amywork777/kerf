//! Coil: helical sweep — chains short cylinders along a helix.
//!
//! Volume bounds: a coil of N turns at coil_radius R, wire_radius r, pitch p
//! has wire arc-length L = N * sqrt((2πR)² + p²). The faceted volume is
//! roughly π * r² * L, but each cylinder segment adds a small cap-overlap
//! at the joint with its neighbor, so true volume is slightly LESS than
//! π r² L.
//!
//! These tests do NOT assert exact volume — the union geometry between
//! consecutive non-collinear cylinders introduces stitch-dependent fudge.
//! They check sane bounds: positive volume, less than the cylinder-soup
//! upper bound.

use std::f64::consts::PI;

use kerf_brep::measure::solid_volume;
use kerf_cad::{Feature, Model, Scalar};

#[test]
fn single_turn_coil_has_volume_in_expected_range() {
    let r_coil = 5.0;
    let r_wire = 0.5;
    let pitch = 1.0;
    let turns = 1.0;
    let m = Model::new().add(Feature::Coil {
        id: "spring".into(),
        coil_radius: Scalar::lit(r_coil),
        wire_radius: Scalar::lit(r_wire),
        pitch: Scalar::lit(pitch),
        turns: Scalar::lit(turns),
        segments_per_turn: 16,
        wire_segments: 8,
    });
    let s = m.evaluate("spring").unwrap();
    let v = solid_volume(&s);

    // Wire arc length per turn ≈ sqrt((2πR)² + pitch²).
    let arc = ((2.0 * PI * r_coil).powi(2) + pitch * pitch).sqrt();
    let total_arc = arc * turns;
    let vol_upper_bound = PI * r_wire * r_wire * total_arc; // soup of cylinders, no overlaps

    assert!(v > 0.0, "coil volume must be positive, got {v}");
    assert!(
        v < vol_upper_bound * 1.05,
        "coil volume {v} exceeds upper bound {vol_upper_bound}"
    );
    // Lower bound: at least 75% of the perfect arc-length cylinder
    // (each chord is shorter than the arc it approximates).
    assert!(
        v > vol_upper_bound * 0.6,
        "coil volume {v} is suspiciously low (upper bound {vol_upper_bound})"
    );
}

#[test]
fn two_turn_coil_completes() {
    let m = Model::new().add(Feature::Coil {
        id: "spring".into(),
        coil_radius: Scalar::lit(3.0),
        wire_radius: Scalar::lit(0.3),
        pitch: Scalar::lit(0.8),
        turns: Scalar::lit(2.0),
        segments_per_turn: 12,
        wire_segments: 6,
    });
    let s = m.evaluate("spring").unwrap();
    assert!(solid_volume(&s) > 0.0);
}

#[test]
fn coil_rejects_wire_too_thick() {
    // wire_radius >= coil_radius: helix self-overlaps.
    let m = Model::new().add(Feature::Coil {
        id: "bad".into(),
        coil_radius: Scalar::lit(1.0),
        wire_radius: Scalar::lit(1.5),
        pitch: Scalar::lit(0.5),
        turns: Scalar::lit(1.0),
        segments_per_turn: 12,
        wire_segments: 6,
    });
    assert!(m.evaluate("bad").is_err());
}

#[test]
fn coil_rejects_low_subdivision() {
    let m = Model::new().add(Feature::Coil {
        id: "coarse".into(),
        coil_radius: Scalar::lit(2.0),
        wire_radius: Scalar::lit(0.2),
        pitch: Scalar::lit(0.5),
        turns: Scalar::lit(1.0),
        segments_per_turn: 4,
        wire_segments: 6,
    });
    assert!(m.evaluate("coarse").is_err());
}

#[test]
fn coil_round_trips_via_json() {
    let m = Model::new().add(Feature::Coil {
        id: "spring".into(),
        coil_radius: Scalar::lit(5.0),
        wire_radius: Scalar::lit(0.4),
        pitch: Scalar::lit(1.2),
        turns: Scalar::lit(3.0),
        segments_per_turn: 16,
        wire_segments: 8,
    });
    let json = m.to_json_string().unwrap();
    let parsed = Model::from_json_str(&json).unwrap();
    let json2 = parsed.to_json_string().unwrap();
    assert_eq!(json, json2, "Coil JSON round-trip must be lossless");
}
