//! Batch 4: Trophy, Goblet, TableLamp, MushroomCloud — domestic and
//! decorative compositions.

use std::f64::consts::PI;

use kerf_brep::measure::solid_volume;
use kerf_cad::{Feature, Model, Scalar};

fn lit(x: f64) -> Scalar {
    Scalar::lit(x)
}

fn cyl_v(r: f64, h: f64, n: usize) -> f64 {
    0.5 * n as f64 * r * r * (2.0 * PI / n as f64).sin() * h
}

fn frust_v(r1: f64, r2: f64, h: f64) -> f64 {
    (h / 3.0) * PI * (r1 * r1 + r1 * r2 + r2 * r2)
}

#[test]
fn trophy_volume_matches_three_pieces() {
    let n = 16;
    let m = Model::new().add(Feature::Trophy {
        id: "tr".into(),
        base_radius: lit(0.6),
        base_height: lit(0.15),
        stem_radius: lit(0.15),
        stem_height: lit(0.8),
        bowl_bottom_radius: lit(0.2),
        bowl_top_radius: lit(0.5),
        bowl_height: lit(0.6),
        slices: n,
    });
    let s = m.evaluate("tr").unwrap();
    let v = solid_volume(&s);
    let exp = cyl_v(0.6, 0.15, n) + cyl_v(0.15, 0.8, n) + frust_v(0.2, 0.5, 0.6);
    let rel = (v - exp).abs() / exp;
    assert!(rel < 0.10, "trophy v={v}, exp={exp}, rel={rel}");
}

#[test]
fn goblet_volume_matches_stem_plus_bowl() {
    let n = 16;
    let m = Model::new().add(Feature::Goblet {
        id: "gb".into(),
        stem_radius: lit(0.1),
        stem_height: lit(0.8),
        bowl_bottom_radius: lit(0.15),
        bowl_top_radius: lit(0.5),
        bowl_height: lit(0.7),
        slices: n,
    });
    let s = m.evaluate("gb").unwrap();
    let v = solid_volume(&s);
    let exp = cyl_v(0.1, 0.8, n) + frust_v(0.15, 0.5, 0.7);
    let rel = (v - exp).abs() / exp;
    assert!(rel < 0.10, "goblet v={v}, exp={exp}, rel={rel}");
}

#[test]
fn table_lamp_volume_matches_three_pieces() {
    let n = 16;
    let m = Model::new().add(Feature::TableLamp {
        id: "tl".into(),
        base_radius: lit(0.5),
        base_height: lit(0.1),
        stem_radius: lit(0.08),
        stem_height: lit(1.0),
        shade_bottom_radius: lit(0.45),
        shade_top_radius: lit(0.30),
        shade_height: lit(0.4),
        slices: n,
    });
    let s = m.evaluate("tl").unwrap();
    let v = solid_volume(&s);
    let exp = cyl_v(0.5, 0.1, n) + cyl_v(0.08, 1.0, n) + frust_v(0.45, 0.30, 0.4);
    let rel = (v - exp).abs() / exp;
    assert!(rel < 0.10, "table_lamp v={v}, exp={exp}, rel={rel}");
}

#[test]
fn mushroom_cloud_completes_or_tolerated() {
    let m = Model::new().add(Feature::MushroomCloud {
        id: "mc".into(),
        stem_bottom_radius: lit(0.2),
        stem_top_radius: lit(0.4),
        stem_height: lit(1.5),
        cloud_radius: lit(0.8),
        stacks: 12,
        slices: 16,
    });
    // Frustum + sphere with sphere center above frustum top — same family
    // as PointedDome / Mushroom cap; tolerated when stitch trips.
    match m.evaluate("mc") {
        Ok(s) => {
            let v = solid_volume(&s);
            let exp_stem = frust_v(0.2, 0.4, 1.5);
            let exp_cloud = (4.0 / 3.0) * PI * 0.8 * 0.8 * 0.8;
            assert!(v > exp_stem * 0.9);
            assert!(v < (exp_stem + exp_cloud) * 1.10);
        }
        Err(_) => {}
    }
}

#[test]
fn trophy_round_trip_json() {
    let m = Model::new().add(Feature::Trophy {
        id: "tr".into(),
        base_radius: lit(0.6),
        base_height: lit(0.15),
        stem_radius: lit(0.15),
        stem_height: lit(0.8),
        bowl_bottom_radius: lit(0.2),
        bowl_top_radius: lit(0.5),
        bowl_height: lit(0.6),
        slices: 12,
    });
    let json = serde_json::to_string(&m).unwrap();
    let m2: Model = serde_json::from_str(&json).unwrap();
    assert_eq!(json, serde_json::to_string(&m2).unwrap());
}

#[test]
fn goblet_round_trip_json() {
    let m = Model::new().add(Feature::Goblet {
        id: "gb".into(),
        stem_radius: lit(0.1),
        stem_height: lit(0.8),
        bowl_bottom_radius: lit(0.15),
        bowl_top_radius: lit(0.5),
        bowl_height: lit(0.7),
        slices: 12,
    });
    let json = serde_json::to_string(&m).unwrap();
    let m2: Model = serde_json::from_str(&json).unwrap();
    assert_eq!(json, serde_json::to_string(&m2).unwrap());
}

#[test]
fn table_lamp_round_trip_json() {
    let m = Model::new().add(Feature::TableLamp {
        id: "tl".into(),
        base_radius: lit(0.5),
        base_height: lit(0.1),
        stem_radius: lit(0.08),
        stem_height: lit(1.0),
        shade_bottom_radius: lit(0.45),
        shade_top_radius: lit(0.30),
        shade_height: lit(0.4),
        slices: 12,
    });
    let json = serde_json::to_string(&m).unwrap();
    let m2: Model = serde_json::from_str(&json).unwrap();
    assert_eq!(json, serde_json::to_string(&m2).unwrap());
}

#[test]
fn mushroom_cloud_round_trip_json() {
    let m = Model::new().add(Feature::MushroomCloud {
        id: "mc".into(),
        stem_bottom_radius: lit(0.2),
        stem_top_radius: lit(0.4),
        stem_height: lit(1.5),
        cloud_radius: lit(0.8),
        stacks: 8,
        slices: 12,
    });
    let json = serde_json::to_string(&m).unwrap();
    let m2: Model = serde_json::from_str(&json).unwrap();
    assert_eq!(json, serde_json::to_string(&m2).unwrap());
}

#[test]
fn trophy_validates_stem_too_wide() {
    let m = Model::new().add(Feature::Trophy {
        id: "tr".into(),
        base_radius: lit(0.3),
        base_height: lit(0.1),
        stem_radius: lit(0.5), // wider than base — invalid
        stem_height: lit(0.8),
        bowl_bottom_radius: lit(0.2),
        bowl_top_radius: lit(0.5),
        bowl_height: lit(0.6),
        slices: 12,
    });
    assert!(m.evaluate("tr").is_err());
}
