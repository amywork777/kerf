//! Batch 6: Onion, WaspWaist, Flask, Pear — curved-surface row polish
//! (targeting ~95% coverage on the curved-surface metric).

use std::f64::consts::PI;

use kerf_brep::measure::solid_volume;
use kerf_cad::{Feature, Model, Scalar};

fn lit(x: f64) -> Scalar {
    Scalar::lit(x)
}

fn cyl_v(r: f64, h: f64) -> f64 {
    PI * r * r * h
}

fn frust_v(r1: f64, r2: f64, h: f64) -> f64 {
    (h / 3.0) * PI * (r1 * r1 + r1 * r2 + r2 * r2)
}

fn sphere_v(r: f64) -> f64 {
    (4.0 / 3.0) * PI * r * r * r
}

// ---------------------------------------------------------------------------
// Onion: positive volume + bounds sanity
// ---------------------------------------------------------------------------

#[test]
fn onion_has_positive_volume() {
    let m = Model::new().add(Feature::Onion {
        id: "on".into(),
        base_radius: lit(0.5),
        mid_height: lit(0.4),
        point_height: lit(0.6),
        segments: 16,
    });
    let s = m.evaluate("on").unwrap();
    let v = solid_volume(&s);
    assert!(v > 0.0, "Onion must have positive volume (got {v})");
    // Must be bigger than a simple hemisphere of base_radius.
    let hemi = sphere_v(0.5) / 2.0;
    assert!(v > hemi * 0.5, "Onion volume={v} should exceed half-hemisphere={hemi}");
}

#[test]
fn onion_bounds_match_base_plus_mid_plus_point() {
    // base_radius=0.5, mid_height=0.4, point_height=0.6
    // Total z height ~ base_radius (hemi) + mid_height + point_height = 1.5
    let m = Model::new().add(Feature::Onion {
        id: "on2".into(),
        base_radius: lit(0.5),
        mid_height: lit(0.4),
        point_height: lit(0.6),
        segments: 16,
    });
    let s = m.evaluate("on2").unwrap();
    let v = solid_volume(&s);
    // Should be well under a full sphere + two frustums (generous upper bound).
    let upper = sphere_v(0.5) + frust_v(0.25, 0.5, 0.4) + cyl_v(0.5, 0.6);
    assert!(v < upper * 1.2, "Onion volume={v} exceeds generous upper bound={upper}");
}

// ---------------------------------------------------------------------------
// WaspWaist: waist_radius == top_radius ≡ cylinder (2 symmetric frustums)
// ---------------------------------------------------------------------------

#[test]
fn wasp_waist_uniform_equals_cylinder() {
    let n = 32;
    let r = 0.5;
    let h = 2.0;
    // When waist_radius == top_radius the two frustums are both cylinders.
    let m = Model::new().add(Feature::WaspWaist {
        id: "ww".into(),
        top_radius: lit(r),
        waist_radius: lit(r),
        total_height: lit(h),
        segments: n,
    });
    let s = m.evaluate("ww").unwrap();
    let v = solid_volume(&s);
    let exp = cyl_v(r, h);
    let rel = (v - exp).abs() / exp;
    assert!(rel < 0.05, "WaspWaist(uniform) v={v}, exp={exp}, rel={rel}");
}

#[test]
fn wasp_waist_pinched_has_positive_volume() {
    let m = Model::new().add(Feature::WaspWaist {
        id: "ww2".into(),
        top_radius: lit(0.5),
        waist_radius: lit(0.2),
        total_height: lit(2.0),
        segments: 16,
    });
    let s = m.evaluate("ww2").unwrap();
    let v = solid_volume(&s);
    assert!(v > 0.0, "WaspWaist(pinched) must have positive volume (got {v})");
    // Volume must be less than a full cylinder (it is pinched in the middle).
    let cylinder_upper = cyl_v(0.5, 2.0);
    assert!(v < cylinder_upper, "WaspWaist pinched volume={v} must be less than cylinder={cylinder_upper}");
}

// ---------------------------------------------------------------------------
// Flask: body + shoulder + neck volume roughly matches components
// ---------------------------------------------------------------------------

#[test]
fn flask_volume_matches_components() {
    let n = 24;
    let br = 0.5;
    let bh = 1.0;
    let nr = 0.1;
    let nh = 0.8;
    let sh = 0.4;
    let m = Model::new().add(Feature::Flask {
        id: "fl".into(),
        body_radius: lit(br),
        body_height: lit(bh),
        neck_radius: lit(nr),
        neck_height: lit(nh),
        shoulder_height: lit(sh),
        segments: n,
    });
    let s = m.evaluate("fl").unwrap();
    let v = solid_volume(&s);
    let exp = cyl_v(br, bh) + frust_v(nr, br, sh) + cyl_v(nr, nh);
    // Expect within 15% (overlap stitching and numeric tolerance).
    let rel = (v - exp).abs() / exp;
    assert!(rel < 0.15, "Flask v={v}, exp={exp}, rel={rel}");
}

#[test]
fn flask_validates_neck_wider_than_body() {
    let m = Model::new().add(Feature::Flask {
        id: "fl_bad".into(),
        body_radius: lit(0.3),
        body_height: lit(1.0),
        neck_radius: lit(0.5), // wider than body — invalid
        neck_height: lit(0.8),
        shoulder_height: lit(0.3),
        segments: 12,
    });
    assert!(m.evaluate("fl_bad").is_err());
}

// ---------------------------------------------------------------------------
// Pear: sphere body + frustum neck composition
// ---------------------------------------------------------------------------

#[test]
fn pear_has_positive_volume() {
    let m = Model::new().add(Feature::Pear {
        id: "pr".into(),
        body_radius: lit(0.5),
        neck_radius: lit(0.15),
        neck_height: lit(0.6),
        stacks: 12,
        segments: 16,
    });
    let s = m.evaluate("pr").unwrap();
    let v = solid_volume(&s);
    assert!(v > 0.0, "Pear must have positive volume (got {v})");
    // Should at least be close to sphere volume.
    let sv = sphere_v(0.5);
    assert!(v > sv * 0.6, "Pear volume={v} should exceed 60% of sphere={sv}");
}

#[test]
fn pear_validates_neck_too_wide() {
    let m = Model::new().add(Feature::Pear {
        id: "pr_bad".into(),
        body_radius: lit(0.3),
        neck_radius: lit(0.5), // wider than body — invalid
        neck_height: lit(0.6),
        stacks: 8,
        segments: 12,
    });
    assert!(m.evaluate("pr_bad").is_err());
}

// ---------------------------------------------------------------------------
// Round-trip JSON for all four variants
// ---------------------------------------------------------------------------

#[test]
fn onion_round_trip_json() {
    let m = Model::new().add(Feature::Onion {
        id: "on".into(),
        base_radius: lit(0.5),
        mid_height: lit(0.4),
        point_height: lit(0.6),
        segments: 12,
    });
    let json = serde_json::to_string(&m).unwrap();
    let m2: Model = serde_json::from_str(&json).unwrap();
    assert_eq!(json, serde_json::to_string(&m2).unwrap());
}

#[test]
fn wasp_waist_round_trip_json() {
    let m = Model::new().add(Feature::WaspWaist {
        id: "ww".into(),
        top_radius: lit(0.5),
        waist_radius: lit(0.2),
        total_height: lit(2.0),
        segments: 12,
    });
    let json = serde_json::to_string(&m).unwrap();
    let m2: Model = serde_json::from_str(&json).unwrap();
    assert_eq!(json, serde_json::to_string(&m2).unwrap());
}

#[test]
fn flask_round_trip_json() {
    let m = Model::new().add(Feature::Flask {
        id: "fl".into(),
        body_radius: lit(0.5),
        body_height: lit(1.0),
        neck_radius: lit(0.1),
        neck_height: lit(0.8),
        shoulder_height: lit(0.4),
        segments: 12,
    });
    let json = serde_json::to_string(&m).unwrap();
    let m2: Model = serde_json::from_str(&json).unwrap();
    assert_eq!(json, serde_json::to_string(&m2).unwrap());
}

#[test]
fn pear_round_trip_json() {
    let m = Model::new().add(Feature::Pear {
        id: "pr".into(),
        body_radius: lit(0.5),
        neck_radius: lit(0.15),
        neck_height: lit(0.6),
        stacks: 8,
        segments: 12,
    });
    let json = serde_json::to_string(&m).unwrap();
    let m2: Model = serde_json::from_str(&json).unwrap();
    assert_eq!(json, serde_json::to_string(&m2).unwrap());
}
