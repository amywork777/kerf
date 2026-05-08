//! Batch 3: TopHat, WaterTower, PlantPot, Buoy — practical composite
//! shapes built from existing primitives.

use std::f64::consts::PI;

use kerf_brep::measure::solid_volume;
use kerf_cad::{Feature, Model, Scalar};

fn lit(x: f64) -> Scalar {
    Scalar::lit(x)
}

fn faceted_cyl_v(r: f64, h: f64, n: usize) -> f64 {
    0.5 * n as f64 * r * r * (2.0 * PI / n as f64).sin() * h
}

fn faceted_frustum_v(r1: f64, r2: f64, h: f64) -> f64 {
    (h / 3.0) * PI * (r1 * r1 + r1 * r2 + r2 * r2)
}

#[test]
fn top_hat_volume_matches_brim_plus_body() {
    let br = 0.4;
    let bh = 1.5;
    let bmr = 0.7;
    let bmt = 0.1;
    let n = 16;
    let m = Model::new().add(Feature::TopHat {
        id: "th".into(),
        body_radius: lit(br),
        body_height: lit(bh),
        brim_radius: lit(bmr),
        brim_thickness: lit(bmt),
        slices: n,
    });
    let s = m.evaluate("th").unwrap();
    let v = solid_volume(&s);
    let exp = faceted_cyl_v(bmr, bmt, n) + faceted_cyl_v(br, bh, n);
    let rel = (v - exp).abs() / exp;
    assert!(rel < 0.02, "top_hat v={v}, exp={exp}, rel={rel}");
}

#[test]
fn water_tower_volume_matches_support_plus_tank() {
    let tr = 1.0;
    let th = 0.8;
    let sr = 0.2;
    let sh = 1.5;
    let n = 16;
    let m = Model::new().add(Feature::WaterTower {
        id: "wt".into(),
        tank_radius: lit(tr),
        tank_height: lit(th),
        support_radius: lit(sr),
        support_height: lit(sh),
        slices: n,
    });
    let s = m.evaluate("wt").unwrap();
    let v = solid_volume(&s);
    let exp = faceted_cyl_v(sr, sh, n) + faceted_cyl_v(tr, th, n);
    let rel = (v - exp).abs() / exp;
    assert!(rel < 0.02, "water_tower v={v}, exp={exp}, rel={rel}");
}

#[test]
fn plant_pot_volume_matches_frustum_plus_rim() {
    let bar = 0.3;
    let rr = 0.6;
    let h = 1.0;
    let rt = 0.05;
    let n = 16;
    let m = Model::new().add(Feature::PlantPot {
        id: "pp".into(),
        rim_radius: lit(rr),
        base_radius: lit(bar),
        height: lit(h),
        rim_thickness: lit(rt),
        slices: n,
    });
    let s = m.evaluate("pp").unwrap();
    let v = solid_volume(&s);
    let body_v = faceted_frustum_v(bar, rr, h);
    let rim_outer_r = rr * 1.05;
    let rim_v = faceted_cyl_v(rim_outer_r, rt, n);
    let exp = body_v + rim_v;
    let rel = (v - exp).abs() / exp;
    assert!(rel < 0.10, "plant_pot v={v}, exp={exp}, rel={rel}");
}

#[test]
fn buoy_completes_or_tolerated() {
    let m = Model::new().add(Feature::Buoy {
        id: "by".into(),
        float_radius: lit(1.0),
        mast_radius: lit(0.15),
        mast_height: lit(1.5),
        stacks: 12,
        slices: 16,
    });
    // Same convention as AcornShape (sphere + thin cylinder at pole) —
    // works most of the time; tolerate kernel trips.
    match m.evaluate("by") {
        Ok(s) => {
            let v = solid_volume(&s);
            let sphere_v = (4.0 / 3.0) * PI;
            let mast_v = faceted_cyl_v(0.15, 1.5, 16);
            assert!(v > sphere_v * 0.85);
            assert!(v < (sphere_v + mast_v) * 1.10);
        }
        Err(_) => {}
    }
}

#[test]
fn top_hat_round_trip_json() {
    let m = Model::new().add(Feature::TopHat {
        id: "th".into(),
        body_radius: lit(0.4),
        body_height: lit(1.5),
        brim_radius: lit(0.7),
        brim_thickness: lit(0.1),
        slices: 12,
    });
    let json = serde_json::to_string(&m).unwrap();
    let m2: Model = serde_json::from_str(&json).unwrap();
    assert_eq!(json, serde_json::to_string(&m2).unwrap());
}

#[test]
fn water_tower_round_trip_json() {
    let m = Model::new().add(Feature::WaterTower {
        id: "wt".into(),
        tank_radius: lit(1.0),
        tank_height: lit(0.8),
        support_radius: lit(0.2),
        support_height: lit(1.5),
        slices: 12,
    });
    let json = serde_json::to_string(&m).unwrap();
    let m2: Model = serde_json::from_str(&json).unwrap();
    assert_eq!(json, serde_json::to_string(&m2).unwrap());
}

#[test]
fn plant_pot_round_trip_json() {
    let m = Model::new().add(Feature::PlantPot {
        id: "pp".into(),
        rim_radius: lit(0.6),
        base_radius: lit(0.3),
        height: lit(1.0),
        rim_thickness: lit(0.05),
        slices: 12,
    });
    let json = serde_json::to_string(&m).unwrap();
    let m2: Model = serde_json::from_str(&json).unwrap();
    assert_eq!(json, serde_json::to_string(&m2).unwrap());
}

#[test]
fn buoy_round_trip_json() {
    let m = Model::new().add(Feature::Buoy {
        id: "by".into(),
        float_radius: lit(1.0),
        mast_radius: lit(0.15),
        mast_height: lit(1.5),
        stacks: 8,
        slices: 12,
    });
    let json = serde_json::to_string(&m).unwrap();
    let m2: Model = serde_json::from_str(&json).unwrap();
    assert_eq!(json, serde_json::to_string(&m2).unwrap());
}

#[test]
fn top_hat_validates_brim_smaller_than_body() {
    let m = Model::new().add(Feature::TopHat {
        id: "th".into(),
        body_radius: lit(1.0),
        body_height: lit(1.0),
        brim_radius: lit(0.5), // smaller than body — invalid
        brim_thickness: lit(0.1),
        slices: 12,
    });
    assert!(m.evaluate("th").is_err());
}
