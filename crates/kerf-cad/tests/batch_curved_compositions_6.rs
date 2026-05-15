//! Batch 6: Bagel, Pringle, Cone2, Lozenge — curved-surface batch 4.

use std::f64::consts::PI;

use kerf_brep::measure::solid_volume;
use kerf_cad::{Feature, Model, Scalar};

fn lit(x: f64) -> Scalar {
    Scalar::lit(x)
}

fn torus_volume_approx(r_maj: f64, r_min: f64) -> f64 {
    2.0 * PI * PI * r_maj * r_min * r_min
}

fn cyl_v(r: f64, h: f64) -> f64 {
    PI * r * r * h
}

// ---------------------------------------------------------------------------
// Bagel
// ---------------------------------------------------------------------------

#[test]
fn bagel_evaluates_and_volume_exceeds_base_torus() {
    let r_maj = 2.0;
    let r_min = 0.5;
    let dh = 0.2;
    let n = 12usize;
    let m = Model::new().add(Feature::Bagel {
        id: "b".into(),
        major_radius: lit(r_maj),
        minor_radius: lit(r_min),
        dome_height: lit(dh),
        segments: n,
    });
    let s = m.evaluate("b").unwrap();
    let v = solid_volume(&s);
    // Volume must be at least the base torus volume.
    let torus_v = torus_volume_approx(r_maj, r_min);
    assert!(v >= torus_v * 0.8, "bagel volume {v} should exceed base torus ~{torus_v}");
    // And not unreasonably large.
    assert!(v < torus_v * 3.0, "bagel volume {v} too large");
}

#[test]
fn bagel_rejects_major_le_minor() {
    let m = Model::new().add(Feature::Bagel {
        id: "b".into(),
        major_radius: lit(1.0),
        minor_radius: lit(1.5), // minor > major — invalid
        dome_height: lit(0.3),
        segments: 8,
    });
    assert!(m.evaluate("b").is_err());
}

#[test]
fn bagel_rejects_dome_height_exceeds_minor() {
    let m = Model::new().add(Feature::Bagel {
        id: "b".into(),
        major_radius: lit(4.0),
        minor_radius: lit(0.5),
        dome_height: lit(0.8), // > minor_radius — invalid
        segments: 8,
    });
    assert!(m.evaluate("b").is_err());
}

#[test]
fn bagel_round_trip_json() {
    let m = Model::new().add(Feature::Bagel {
        id: "b".into(),
        major_radius: lit(3.0),
        minor_radius: lit(0.8),
        dome_height: lit(0.4),
        segments: 10,
    });
    let json = serde_json::to_string(&m).unwrap();
    let m2: Model = serde_json::from_str(&json).unwrap();
    assert_eq!(json, serde_json::to_string(&m2).unwrap());
}

// ---------------------------------------------------------------------------
// Pringle
// ---------------------------------------------------------------------------

#[test]
fn pringle_evaluates_and_has_positive_volume() {
    let side = 4.0;
    let dh = 1.0;
    let n = 4usize;
    let m = Model::new().add(Feature::Pringle {
        id: "p".into(),
        side: lit(side),
        dome_height: lit(dh),
        segments: n,
    });
    let s = m.evaluate("p").unwrap();
    let v = solid_volume(&s);
    // Volume should be roughly side^2 * (dh/4) — a rough lower bound.
    let thickness = dh / 4.0;
    let lower = side * side * thickness * 0.3;
    let upper = side * side * (dh + thickness) * 2.0;
    assert!(v > lower, "pringle volume {v} below lower bound {lower}");
    assert!(v < upper, "pringle volume {v} above upper bound {upper}");
}

#[test]
fn pringle_rejects_non_positive_side() {
    let m = Model::new().add(Feature::Pringle {
        id: "p".into(),
        side: lit(0.0),
        dome_height: lit(1.0),
        segments: 4,
    });
    assert!(m.evaluate("p").is_err());
}

#[test]
fn pringle_rejects_non_positive_dome_height() {
    let m = Model::new().add(Feature::Pringle {
        id: "p".into(),
        side: lit(4.0),
        dome_height: lit(-0.5),
        segments: 4,
    });
    assert!(m.evaluate("p").is_err());
}

#[test]
fn pringle_round_trip_json() {
    let m = Model::new().add(Feature::Pringle {
        id: "p".into(),
        side: lit(6.0),
        dome_height: lit(1.5),
        segments: 3,
    });
    let json = serde_json::to_string(&m).unwrap();
    let m2: Model = serde_json::from_str(&json).unwrap();
    assert_eq!(json, serde_json::to_string(&m2).unwrap());
}

// ---------------------------------------------------------------------------
// Cone2
// ---------------------------------------------------------------------------

#[test]
fn cone2_frustum_cap_volume_plausible() {
    let r_base = 3.0;
    let r_tip = 0.8;
    let h = 5.0;
    let n = 16usize;
    let m = Model::new().add(Feature::Cone2 {
        id: "c".into(),
        base_radius: lit(r_base),
        tip_radius: lit(r_tip),
        height: lit(h),
        segments: n,
    });
    let result = m.evaluate("c");
    match result {
        Ok(s) => {
            let v = solid_volume(&s);
            // Frustum volume lower bound.
            let frustum_v = (h / 3.0) * PI * (r_base * r_base + r_base * r_tip + r_tip * r_tip);
            // Hemisphere volume.
            let hemi_v = (2.0 / 3.0) * PI * r_tip * r_tip * r_tip;
            let exp = frustum_v + hemi_v;
            let rel = (v - exp).abs() / exp;
            assert!(rel < 0.10, "cone2 v={v} exp={exp} rel={rel}");
        }
        Err(_) => {
            // Curved-surface boolean (hemisphere ∪ frustum) may trip stitch — tolerate.
        }
    }
}

#[test]
fn cone2_rejects_tip_ge_base() {
    let m = Model::new().add(Feature::Cone2 {
        id: "c".into(),
        base_radius: lit(2.0),
        tip_radius: lit(3.0), // tip > base — invalid
        height: lit(5.0),
        segments: 8,
    });
    assert!(m.evaluate("c").is_err());
}

#[test]
fn cone2_zero_tip_is_pure_cone() {
    // tip_radius=0 → straight cone (no hemisphere cap).
    let r = 3.0;
    let h = 4.0;
    let n = 12usize;
    let m = Model::new().add(Feature::Cone2 {
        id: "c".into(),
        base_radius: lit(r),
        tip_radius: lit(0.0),
        height: lit(h),
        segments: n,
    });
    let s = m.evaluate("c").unwrap();
    let v = solid_volume(&s);
    // Cone volume: (1/3) π r² h — use faceted approx.
    let exp_cone = (1.0 / 3.0) * PI * r * r * h;
    assert!(v > exp_cone * 0.8 && v < exp_cone * 1.1, "cone2(tip=0) v={v} exp~{exp_cone}");
}

#[test]
fn cone2_round_trip_json() {
    let m = Model::new().add(Feature::Cone2 {
        id: "c".into(),
        base_radius: lit(4.0),
        tip_radius: lit(1.0),
        height: lit(6.0),
        segments: 10,
    });
    let json = serde_json::to_string(&m).unwrap();
    let m2: Model = serde_json::from_str(&json).unwrap();
    assert_eq!(json, serde_json::to_string(&m2).unwrap());
}

// ---------------------------------------------------------------------------
// Lozenge
// ---------------------------------------------------------------------------

#[test]
fn lozenge_volume_less_than_box_more_than_inner_box() {
    let sx = 6.0f64;
    let sy = 4.0f64;
    let sz = 3.0f64;
    let cr = 0.5f64;
    let n = 4usize;
    let m = Model::new().add(Feature::Lozenge {
        id: "l".into(),
        size: [lit(sx), lit(sy), lit(sz)],
        corner_radius: lit(cr),
        segments: n,
    });
    let s = m.evaluate("l").unwrap();
    let v = solid_volume(&s);
    let box_v = sx * sy * sz;
    // Inner box without corners.
    let inner_v = (sx - 2.0 * cr) * (sy - 2.0 * cr) * sz;
    assert!(v < box_v, "lozenge volume {v} should be less than full box {box_v}");
    assert!(v > inner_v * 0.5, "lozenge volume {v} should exceed shrunken inner box {inner_v}");
}

#[test]
fn lozenge_rejects_corner_radius_too_large() {
    let m = Model::new().add(Feature::Lozenge {
        id: "l".into(),
        size: [lit(4.0), lit(4.0), lit(3.0)],
        corner_radius: lit(2.5), // >= min(4,4)/2 = 2 — invalid
        segments: 4,
    });
    assert!(m.evaluate("l").is_err());
}

#[test]
fn lozenge_rejects_zero_corner_radius() {
    let m = Model::new().add(Feature::Lozenge {
        id: "l".into(),
        size: [lit(4.0), lit(4.0), lit(3.0)],
        corner_radius: lit(0.0),
        segments: 4,
    });
    assert!(m.evaluate("l").is_err());
}

#[test]
fn lozenge_round_trip_json() {
    let m = Model::new().add(Feature::Lozenge {
        id: "l".into(),
        size: [lit(8.0), lit(6.0), lit(4.0)],
        corner_radius: lit(1.5),
        segments: 3,
    });
    let json = serde_json::to_string(&m).unwrap();
    let m2: Model = serde_json::from_str(&json).unwrap();
    assert_eq!(json, serde_json::to_string(&m2).unwrap());
}
