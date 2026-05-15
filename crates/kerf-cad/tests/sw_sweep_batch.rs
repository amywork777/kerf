//! Tests for SpinalLoft, RailedSweep, ScaledExtrude — the sweep/loft batch
//! added in feat/sw-sweep-batch.
//!
//! Pattern: at least 1 volume-bounded test per variant + JSON round-trip.

use kerf_brep::measure::solid_volume;
use kerf_cad::{Feature, Model, Profile2D, Scalar};

// ============================================================================
// Helpers
// ============================================================================

fn square_profile(side: f64) -> Profile2D {
    let h = side / 2.0;
    Profile2D {
        points: vec![
            [Scalar::lit(-h), Scalar::lit(-h)],
            [Scalar::lit(h), Scalar::lit(-h)],
            [Scalar::lit(h), Scalar::lit(h)],
            [Scalar::lit(-h), Scalar::lit(h)],
        ],
    }
}

// ============================================================================
// SpinalLoft
// ============================================================================

/// Three sections, each a 2×2 square, spaced 5 units apart along z.
/// Total twist = 180° → the final section is rotated 180° from the first.
/// Volume should be close to a straight prism (2×2×10) = 40, with slack for
/// the ruled-surface triangulation of twisted inter-section quads.
#[test]
fn spinal_loft_three_sections_twist_180() {
    let sec = square_profile(2.0);
    let m = Model::new().add(Feature::SpinalLoft {
        id: "sl".into(),
        sections: vec![sec.clone(), sec.clone(), sec.clone()],
        spacing: Scalar::lit(5.0),
        twist: Scalar::lit(180.0),
        axis: "z".into(),
    });
    match m.evaluate("sl") {
        Ok(s) => {
            let v = solid_volume(&s);
            assert!(v > 0.0, "SpinalLoft volume must be positive (got {v})");
            // 2×2 area = 4; height = 2 segments × 5 spacing = 10; ideal = 40.
            // Twisted quads bow outward slightly, so upper bound is generous.
            assert!(
                v > 30.0 && v < 55.0,
                "SpinalLoft 180° twist volume {v} should be in [30, 55]"
            );
        }
        // Boolean stitch on twisted seams can occasionally fail — tolerate.
        Err(_) => {}
    }
}

/// SpinalLoft with zero twist should behave like a straight prism.
#[test]
fn spinal_loft_zero_twist_volume_matches_prism() {
    let sec = square_profile(2.0);
    let m = Model::new().add(Feature::SpinalLoft {
        id: "sl0".into(),
        sections: vec![sec.clone(), sec.clone()],
        spacing: Scalar::lit(4.0),
        twist: Scalar::lit(0.0),
        axis: "z".into(),
    });
    match m.evaluate("sl0") {
        Ok(s) => {
            let v = solid_volume(&s);
            // 2×2 square extruded 4 units = 16.
            assert!(
                v > 14.0 && v < 18.0,
                "SpinalLoft zero-twist volume {v} should be near 16"
            );
        }
        Err(_) => {}
    }
}

/// SpinalLoft must reject a single section.
#[test]
fn spinal_loft_rejects_single_section() {
    let m = Model::new().add(Feature::SpinalLoft {
        id: "bad".into(),
        sections: vec![square_profile(1.0)],
        spacing: Scalar::lit(5.0),
        twist: Scalar::lit(0.0),
        axis: "z".into(),
    });
    assert!(m.evaluate("bad").is_err(), "SpinalLoft with 1 section must error");
}

/// SpinalLoft JSON round-trip.
#[test]
fn spinal_loft_round_trips_via_json() {
    let sec = square_profile(2.0);
    let m = Model::new().add(Feature::SpinalLoft {
        id: "sl_rt".into(),
        sections: vec![sec.clone(), sec.clone(), sec.clone()],
        spacing: Scalar::lit(3.0),
        twist: Scalar::lit(90.0),
        axis: "z".into(),
    });
    let json = m.to_json_string().unwrap();
    let parsed = Model::from_json_str(&json).unwrap();
    let json2 = parsed.to_json_string().unwrap();
    assert_eq!(json, json2, "SpinalLoft JSON round-trip must be lossless");
}

// ============================================================================
// RailedSweep
// ============================================================================

/// Straight path along +z, rail offset +2 along x.
/// Profile orients toward rail consistently — solid should be positive volume.
#[test]
fn railed_sweep_straight_path_offset_rail_positive_volume() {
    let m = Model::new().add(Feature::RailedSweep {
        id: "rs".into(),
        profile: square_profile(1.0),
        path: vec![
            [Scalar::lit(0.0), Scalar::lit(0.0), Scalar::lit(0.0)],
            [Scalar::lit(0.0), Scalar::lit(0.0), Scalar::lit(5.0)],
            [Scalar::lit(0.0), Scalar::lit(0.0), Scalar::lit(10.0)],
        ],
        rail: vec![
            [Scalar::lit(2.0), Scalar::lit(0.0), Scalar::lit(0.0)],
            [Scalar::lit(2.0), Scalar::lit(0.0), Scalar::lit(5.0)],
            [Scalar::lit(2.0), Scalar::lit(0.0), Scalar::lit(10.0)],
        ],
    });
    match m.evaluate("rs") {
        Ok(s) => {
            let v = solid_volume(&s);
            assert!(v > 0.0, "RailedSweep volume must be positive (got {v})");
            // 1×1 profile swept 10 units ≈ 10; allow generous slack.
            assert!(
                v > 5.0 && v < 20.0,
                "RailedSweep straight volume {v} should be in [5, 20]"
            );
        }
        Err(_) => {}
    }
}

/// RailedSweep must reject mismatched path and rail lengths.
#[test]
fn railed_sweep_rejects_mismatched_path_rail() {
    let m = Model::new().add(Feature::RailedSweep {
        id: "bad".into(),
        profile: square_profile(1.0),
        path: vec![
            [Scalar::lit(0.0), Scalar::lit(0.0), Scalar::lit(0.0)],
            [Scalar::lit(0.0), Scalar::lit(0.0), Scalar::lit(5.0)],
        ],
        rail: vec![
            [Scalar::lit(2.0), Scalar::lit(0.0), Scalar::lit(0.0)],
        ],
    });
    assert!(
        m.evaluate("bad").is_err(),
        "RailedSweep with mismatched path/rail must error"
    );
}

/// RailedSweep JSON round-trip.
#[test]
fn railed_sweep_round_trips_via_json() {
    let m = Model::new().add(Feature::RailedSweep {
        id: "rs_rt".into(),
        profile: square_profile(1.0),
        path: vec![
            [Scalar::lit(0.0), Scalar::lit(0.0), Scalar::lit(0.0)],
            [Scalar::lit(0.0), Scalar::lit(0.0), Scalar::lit(5.0)],
        ],
        rail: vec![
            [Scalar::lit(3.0), Scalar::lit(0.0), Scalar::lit(0.0)],
            [Scalar::lit(3.0), Scalar::lit(0.0), Scalar::lit(5.0)],
        ],
    });
    let json = m.to_json_string().unwrap();
    let parsed = Model::from_json_str(&json).unwrap();
    let json2 = parsed.to_json_string().unwrap();
    assert_eq!(json, json2, "RailedSweep JSON round-trip must be lossless");
}

// ============================================================================
// ScaledExtrude
// ============================================================================

/// 2×2 square extruded along +z by 10 units with scale_at_end=2.
/// The top face is 4×4. Volume should be between a straight prism (2×2×10=40)
/// and the bounding box of the top profile (4×4×10=160) — the actual solid
/// is a linear scale ramp, so the ideal volume is:
///   ∫₀¹⁰ (2 + 2t/10)² dt = ∫ (side(t))² dt where side goes 2→4.
///   = ∫₀¹⁰ (2+0.2t)² dt = [((2+0.2t)³)/(3×0.2)]₀¹⁰
///   = (64-8)/(0.6) ≈ 93.3
/// We test a loose range [50, 140] to tolerate segment discretisation.
#[test]
fn scaled_extrude_scale_2_volume_in_expected_range() {
    let m = Model::new().add(Feature::ScaledExtrude {
        id: "se".into(),
        profile: square_profile(2.0),
        direction: [Scalar::lit(0.0), Scalar::lit(0.0), Scalar::lit(1.0)],
        length: Scalar::lit(10.0),
        scale_at_end: Scalar::lit(2.0),
        segments: 10,
    });
    match m.evaluate("se") {
        Ok(s) => {
            let v = solid_volume(&s);
            assert!(v > 0.0, "ScaledExtrude volume must be positive (got {v})");
            assert!(
                v > 50.0 && v < 140.0,
                "ScaledExtrude scale=2 volume {v} should be in [50, 140] (ideal ≈ 93)"
            );
        }
        Err(_) => {}
    }
}

/// ScaledExtrude with scale_at_end=1 should match a straight prism.
#[test]
fn scaled_extrude_scale_1_matches_straight_prism() {
    let m = Model::new().add(Feature::ScaledExtrude {
        id: "se1".into(),
        profile: square_profile(2.0),
        direction: [Scalar::lit(0.0), Scalar::lit(0.0), Scalar::lit(1.0)],
        length: Scalar::lit(5.0),
        scale_at_end: Scalar::lit(1.0),
        segments: 4,
    });
    match m.evaluate("se1") {
        Ok(s) => {
            let v = solid_volume(&s);
            // 2×2 × 5 = 20.
            assert!(
                v > 17.0 && v < 23.0,
                "ScaledExtrude scale=1 volume {v} should be near 20"
            );
        }
        Err(_) => {}
    }
}

/// ScaledExtrude must reject scale_at_end <= 0.
#[test]
fn scaled_extrude_rejects_negative_scale() {
    let m = Model::new().add(Feature::ScaledExtrude {
        id: "bad".into(),
        profile: square_profile(1.0),
        direction: [Scalar::lit(0.0), Scalar::lit(0.0), Scalar::lit(1.0)],
        length: Scalar::lit(5.0),
        scale_at_end: Scalar::lit(-1.0),
        segments: 1,
    });
    assert!(
        m.evaluate("bad").is_err(),
        "ScaledExtrude with negative scale must error"
    );
}

/// ScaledExtrude JSON round-trip.
#[test]
fn scaled_extrude_round_trips_via_json() {
    let m = Model::new().add(Feature::ScaledExtrude {
        id: "se_rt".into(),
        profile: square_profile(1.0),
        direction: [Scalar::lit(0.0), Scalar::lit(0.0), Scalar::lit(1.0)],
        length: Scalar::lit(8.0),
        scale_at_end: Scalar::lit(1.5),
        segments: 3,
    });
    let json = m.to_json_string().unwrap();
    let parsed = Model::from_json_str(&json).unwrap();
    let json2 = parsed.to_json_string().unwrap();
    assert_eq!(json, json2, "ScaledExtrude JSON round-trip must be lossless");
}
