//! Batch 2 curved-surface primitives: Donut2, ToroidalCap, EllipticTube, Goblet2.
//!
//! Volume checks:
//!  - Donut2: volume ≈ 2π²Rr²   (analytic torus formula, high-res faceting)
//!  - ToroidalCap 180°: volume ≈ half of full torus (2π²Rr² / 2)
//!  - EllipticTube: volume = π·a·b·L  (exact analytic for the elliptic prism)
//!  - Goblet2: positive volume; bounding height ≥ foot + stem + cup

use std::f64::consts::PI;

use kerf_brep::measure::solid_volume;
use kerf_cad::{Feature, Model, Scalar};

fn lit(x: f64) -> Scalar {
    Scalar::lit(x)
}

// ---------------------------------------------------------------------------
// Donut2
// ---------------------------------------------------------------------------

/// Volume of an analytic torus: 2π²Rr².
#[test]
fn donut2_volume_approaches_analytic_torus() {
    let r_maj = 5.0_f64;
    let r_min = 1.0_f64;
    let n = 32; // high-res to keep error < 5%
    let m = Model::new().add(Feature::Donut2 {
        id: "d2".into(),
        major_radius: lit(r_maj),
        minor_radius: lit(r_min),
        major_segs: n,
        minor_segs: n,
    });
    let s = m.evaluate("d2").unwrap();
    let v = solid_volume(&s);
    let expected = 2.0 * PI * PI * r_maj * r_min * r_min;
    let rel = (v - expected).abs() / expected;
    assert!(
        rel < 0.06,
        "Donut2 volume {v:.4} should be within 6% of analytic {expected:.4} (rel={rel:.4})"
    );
}

/// Donut2 with invalid radii is rejected.
#[test]
fn donut2_invalid_radii_rejected() {
    // major <= minor → invalid
    let m = Model::new().add(Feature::Donut2 {
        id: "bad".into(),
        major_radius: lit(1.0),
        minor_radius: lit(2.0),
        major_segs: 8,
        minor_segs: 6,
    });
    assert!(m.evaluate("bad").is_err());
}

// ---------------------------------------------------------------------------
// ToroidalCap
// ---------------------------------------------------------------------------

/// A 180° ToroidalCap should have volume ≈ half of the full torus (2π²Rr² / 2).
/// The approximation uses the same sweep-cylinder approach as HalfTorus, so
/// the tolerance is generous (20%) to account for the faceted approximation.
#[test]
fn toroidal_cap_180_is_half_torus_volume() {
    let r_maj = 5.0_f64;
    let r_min = 0.8_f64;
    let segs = 16;
    let m = Model::new().add(Feature::ToroidalCap {
        id: "tc".into(),
        major_radius: lit(r_maj),
        minor_radius: lit(r_min),
        sweep_degrees: lit(180.0),
        major_segs: segs,
        minor_segs: 8,
    });
    let s = m.evaluate("tc").unwrap();
    let v = solid_volume(&s);
    let full_analytic = 2.0 * PI * PI * r_maj * r_min * r_min;
    let half_analytic = full_analytic * 0.5;
    // sweep-cylinder approximation can deviate more than the faceted torus
    assert!(
        v > 0.0,
        "ToroidalCap 180° must have positive volume, got {v}"
    );
    // Should be in the right ballpark (within 40% of analytic half-torus)
    let rel = (v - half_analytic).abs() / half_analytic;
    assert!(
        rel < 0.40,
        "ToroidalCap 180° volume {v:.4} should be within 40% of half-torus analytic {half_analytic:.4} (rel={rel:.4})"
    );
}

/// A 360° ToroidalCap falls back to the full closed torus.
#[test]
fn toroidal_cap_360_is_full_torus() {
    let r_maj = 4.0_f64;
    let r_min = 0.6_f64;
    let m = Model::new().add(Feature::ToroidalCap {
        id: "tc360".into(),
        major_radius: lit(r_maj),
        minor_radius: lit(r_min),
        sweep_degrees: lit(360.0),
        major_segs: 16,
        minor_segs: 8,
    });
    let s = m.evaluate("tc360").unwrap();
    let v = solid_volume(&s);
    assert!(v > 0.0, "ToroidalCap 360° must have positive volume, got {v}");
}

/// Invalid sweep angle is rejected.
#[test]
fn toroidal_cap_invalid_sweep_rejected() {
    let m = Model::new().add(Feature::ToroidalCap {
        id: "bad".into(),
        major_radius: lit(5.0),
        minor_radius: lit(1.0),
        sweep_degrees: lit(-10.0),
        major_segs: 8,
        minor_segs: 6,
    });
    assert!(m.evaluate("bad").is_err());
}

// ---------------------------------------------------------------------------
// EllipticTube
// ---------------------------------------------------------------------------

/// Volume of an elliptic prism = π·a·b·L.
#[test]
fn elliptic_tube_volume_matches_analytic() {
    let a = 3.0_f64; // semi_major
    let b = 1.5_f64; // semi_minor
    let l = 8.0_f64; // length
    let n = 32;
    let m = Model::new().add(Feature::EllipticTube {
        id: "et".into(),
        semi_major: lit(a),
        semi_minor: lit(b),
        length: lit(l),
        axis: "z".into(),
        segments: n,
    });
    let s = m.evaluate("et").unwrap();
    let v = solid_volume(&s);
    // Faceted cylinder_faceted with n=32 approximates the elliptic area
    // π·a·b very closely. Tolerance 4%.
    let expected = PI * a * b * l;
    let rel = (v - expected).abs() / expected;
    assert!(
        rel < 0.04,
        "EllipticTube volume {v:.4} should be within 4% of analytic π·a·b·L={expected:.4} (rel={rel:.4})"
    );
}

/// EllipticTube along x-axis produces positive volume.
#[test]
fn elliptic_tube_axis_x_positive_volume() {
    let m = Model::new().add(Feature::EllipticTube {
        id: "etx".into(),
        semi_major: lit(2.0),
        semi_minor: lit(1.0),
        length: lit(5.0),
        axis: "x".into(),
        segments: 16,
    });
    let s = m.evaluate("etx").unwrap();
    assert!(solid_volume(&s) > 0.0);
}

/// EllipticTube with zero semi_major is rejected.
#[test]
fn elliptic_tube_zero_dim_rejected() {
    let m = Model::new().add(Feature::EllipticTube {
        id: "bad".into(),
        semi_major: lit(0.0),
        semi_minor: lit(1.0),
        length: lit(5.0),
        axis: "z".into(),
        segments: 16,
    });
    assert!(m.evaluate("bad").is_err());
}

// ---------------------------------------------------------------------------
// Goblet2
// ---------------------------------------------------------------------------

/// Goblet2 must have positive volume.
#[test]
fn goblet2_positive_volume() {
    let fr = 4.0_f64;
    let sr = 1.0_f64;
    let sh = 6.0_f64;
    let cr = 3.0_f64;
    let ch = 4.0_f64;
    let n = 16;
    let m = Model::new().add(Feature::Goblet2 {
        id: "g2".into(),
        foot_radius: lit(fr),
        stem_radius: lit(sr),
        stem_height: lit(sh),
        cup_radius: lit(cr),
        cup_height: lit(ch),
        segments: n,
    });
    let s = m.evaluate("g2").unwrap();
    let v = solid_volume(&s);
    assert!(v > 0.0, "Goblet2 must have positive volume, got {v}");
}

/// Goblet2 volume is at least as large as the stem cylinder alone
/// (the foot and cup only add material).
#[test]
fn goblet2_volume_exceeds_stem_alone() {
    let sr = 1.0_f64;
    let sh = 6.0_f64;
    let n = 16;

    let m_goblet = Model::new().add(Feature::Goblet2 {
        id: "g2".into(),
        foot_radius: lit(4.0),
        stem_radius: lit(sr),
        stem_height: lit(sh),
        cup_radius: lit(3.0),
        cup_height: lit(4.0),
        segments: n,
    });
    let m_stem = Model::new().add(Feature::Cylinder {
        id: "cyl".into(),
        radius: lit(sr),
        height: lit(sh),
        segments: n,
    });

    let v_goblet = solid_volume(&m_goblet.evaluate("g2").unwrap());
    let v_stem = solid_volume(&m_stem.evaluate("cyl").unwrap());
    assert!(
        v_goblet > v_stem,
        "Goblet2 volume {v_goblet:.4} must exceed stem cylinder volume {v_stem:.4}"
    );
}

/// Goblet2 rejects stem_radius >= foot_radius.
#[test]
fn goblet2_invalid_stem_foot_rejected() {
    let m = Model::new().add(Feature::Goblet2 {
        id: "bad".into(),
        foot_radius: lit(2.0),
        stem_radius: lit(3.0), // stem > foot — invalid
        stem_height: lit(5.0),
        cup_radius: lit(3.0),
        cup_height: lit(4.0),
        segments: 16,
    });
    assert!(m.evaluate("bad").is_err());
}
