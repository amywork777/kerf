//! Helix feature tests.
//!
//! The Helix feature produces a tubular wire swept along a helical path.
//! Unlike Coil (which only sweeps around the z-axis), Helix supports x/y/z
//! axis selection via the `axis` field.
//!
//! Volume notes: a helix of N turns, axis_radius R, wire_radius r, pitch p
//! has wire arc-length ≈ N * sqrt((2πR)² + p²). A full torus (pitch=0, same
//! radius) has volume 2π²Rr². Our helix volume should be in the same ballpark.

use std::f64::consts::PI;

use kerf_brep::measure::solid_volume;
use kerf_cad::{Feature, Model, Scalar};

// ── Test 1: basic z-axis wire helix produces a valid solid with plausible volume ──

#[test]
fn helix_z_axis_wire_produces_valid_solid() {
    let axis_radius = 10.0_f64;
    let wire_radius = 1.0_f64;
    let pitch = 5.0_f64;
    let turns = 2.0_f64;

    let m = Model::new().add(Feature::Helix {
        id: "h".into(),
        axis_radius: Scalar::lit(axis_radius),
        pitch: Scalar::lit(pitch),
        turns: Scalar::lit(turns),
        wire_radius: Scalar::lit(wire_radius),
        axis: "z".into(),
        segments: 64,
    });
    let s = m.evaluate("h").unwrap();
    let v = solid_volume(&s);
    assert!(v > 0.0, "Helix volume must be positive, got {v}");

    // Reference: torus volume = 2π²Rr² ≈ 197 for R=10, r=1.
    // Our helix (2 turns, small pitch) should be in the same order of magnitude.
    let torus_vol = 2.0 * PI * PI * axis_radius * wire_radius * wire_radius;
    // With turns=2 and non-zero pitch, wire length > 2 torus arc. Relax upper bound.
    let wire_arc = turns * ((2.0 * PI * axis_radius).powi(2) + pitch * pitch).sqrt();
    let upper = PI * wire_radius * wire_radius * wire_arc * 1.1; // cylinder-soup upper + 10%
    let lower = torus_vol * 0.5; // at least half a torus worth

    assert!(
        v > lower,
        "Helix volume {v:.2} is too small (lower bound {lower:.2} = half torus vol)"
    );
    assert!(
        v < upper,
        "Helix volume {v:.2} exceeds upper bound {upper:.2}"
    );
}

// ── Test 2: zero turns (t == 0) → error ──

#[test]
fn helix_zero_turns_returns_error() {
    let m = Model::new().add(Feature::Helix {
        id: "h0".into(),
        axis_radius: Scalar::lit(5.0),
        pitch: Scalar::lit(2.0),
        turns: Scalar::lit(0.0),
        wire_radius: Scalar::lit(0.5),
        axis: "z".into(),
        segments: 16,
    });
    assert!(
        m.evaluate("h0").is_err(),
        "Helix with 0 turns must return an error"
    );
}

// ── Test 3: JSON round-trip preserves Helix ──

#[test]
fn helix_round_trips_via_json() {
    let m = Model::new().add(Feature::Helix {
        id: "wire".into(),
        axis_radius: Scalar::lit(8.0),
        pitch: Scalar::lit(3.0),
        turns: Scalar::lit(1.5),
        wire_radius: Scalar::lit(0.6),
        axis: "z".into(),
        segments: 20,
    });
    let json1 = m.to_json_string().unwrap();
    assert!(json1.contains("\"Helix\""), "serialized JSON must contain variant name");
    let parsed = Model::from_json_str(&json1).unwrap();
    let json2 = parsed.to_json_string().unwrap();
    assert_eq!(json1, json2, "Helix JSON round-trip must be lossless");
}

// ── Test 4: axis="x" works the same as axis="z" (different orientation, same volume) ──

#[test]
fn helix_x_axis_produces_same_volume_as_z_axis() {
    let make = |axis: &str| {
        Model::new()
            .add(Feature::Helix {
                id: "h".into(),
                axis_radius: Scalar::lit(6.0),
                pitch: Scalar::lit(2.5),
                turns: Scalar::lit(1.0),
                wire_radius: Scalar::lit(0.4),
                axis: axis.into(),
                segments: 24,
            })
            .evaluate("h")
            .unwrap()
    };
    let v_z = solid_volume(&make("z"));
    let v_x = solid_volume(&make("x"));
    // Volumes should match within 1% — only the orientation differs.
    let rel_err = (v_z - v_x).abs() / v_z.max(v_x);
    assert!(
        rel_err < 0.01,
        "axis=x and axis=z should produce the same volume (rel_err={rel_err:.4}, vz={v_z:.4}, vx={v_x:.4})"
    );
}

// ── Test 7: axis="y" works the same as axis="z" (locks in y-axis symmetry) ──

#[test]
fn helix_y_axis_produces_same_volume_as_z_axis() {
    let make = |axis: &str| {
        Model::new()
            .add(Feature::Helix {
                id: "h".into(),
                axis_radius: Scalar::lit(6.0),
                pitch: Scalar::lit(2.5),
                turns: Scalar::lit(1.0),
                wire_radius: Scalar::lit(0.4),
                axis: axis.into(),
                segments: 24,
            })
            .evaluate("h")
            .unwrap()
    };
    let v_z = solid_volume(&make("z"));
    let v_y = solid_volume(&make("y"));
    // Volumes should match within 1% — only the orientation differs.
    let rel_err = (v_z - v_y).abs() / v_z.max(v_y);
    assert!(
        rel_err < 0.01,
        "axis=y and axis=z should produce the same volume (rel_err={rel_err:.4}, vz={v_z:.4}, vy={v_y:.4})"
    );
}

// ── Test 5: wire_radius == 0 → error (path-only not supported) ──

#[test]
fn helix_zero_wire_radius_returns_error() {
    let m = Model::new().add(Feature::Helix {
        id: "path".into(),
        axis_radius: Scalar::lit(5.0),
        pitch: Scalar::lit(2.0),
        turns: Scalar::lit(1.0),
        wire_radius: Scalar::lit(0.0),
        axis: "z".into(),
        segments: 16,
    });
    assert!(
        m.evaluate("path").is_err(),
        "Helix with wire_radius=0 (path-only) must return an error"
    );
}

// ── Test 6: wire_radius >= axis_radius → error (self-overlap) ──

#[test]
fn helix_wire_too_thick_returns_error() {
    let m = Model::new().add(Feature::Helix {
        id: "bad".into(),
        axis_radius: Scalar::lit(3.0),
        pitch: Scalar::lit(2.0),
        turns: Scalar::lit(1.0),
        wire_radius: Scalar::lit(4.0), // larger than axis_radius
        axis: "z".into(),
        segments: 16,
    });
    assert!(
        m.evaluate("bad").is_err(),
        "Helix with wire_radius >= axis_radius must return an error"
    );
}
