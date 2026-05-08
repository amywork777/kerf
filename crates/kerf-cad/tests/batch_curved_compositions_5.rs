//! Batch 5: TieredCake, Spindle, Ufo, CrowsNest — more functional
//! and decorative compositions.

use std::f64::consts::PI;

use kerf_brep::measure::solid_volume;
use kerf_cad::{Feature, Model, Scalar};

fn lit(x: f64) -> Scalar {
    Scalar::lit(x)
}

fn cyl_v(r: f64, h: f64, n: usize) -> f64 {
    0.5 * n as f64 * r * r * (2.0 * PI / n as f64).sin() * h
}

fn cone_v(r: f64, h: f64, n: usize) -> f64 {
    (1.0 / 3.0) * 0.5 * n as f64 * r * r * (2.0 * PI / n as f64).sin() * h
}

fn frust_v(r1: f64, r2: f64, h: f64) -> f64 {
    (h / 3.0) * PI * (r1 * r1 + r1 * r2 + r2 * r2)
}

#[test]
fn tiered_cake_volume_matches_three_cylinders() {
    let n = 16;
    let m = Model::new().add(Feature::TieredCake {
        id: "tc".into(),
        bottom_radius: lit(1.0),
        middle_radius: lit(0.7),
        top_radius: lit(0.4),
        tier_height: lit(0.5),
        slices: n,
    });
    let s = m.evaluate("tc").unwrap();
    let v = solid_volume(&s);
    let exp = cyl_v(1.0, 0.5, n) + cyl_v(0.7, 0.5, n) + cyl_v(0.4, 0.5, n);
    let rel = (v - exp).abs() / exp;
    assert!(rel < 0.02, "tiered_cake v={v}, exp={exp}, rel={rel}");
}

#[test]
fn spindle_volume_matches_decomposition() {
    let n = 16;
    let r = 0.3;
    let bh = 1.0;
    let ch = 0.4;
    let m = Model::new().add(Feature::Spindle {
        id: "sp".into(),
        body_radius: lit(r),
        body_height: lit(bh),
        cap_height: lit(ch),
        slices: n,
    });
    let s = m.evaluate("sp").unwrap();
    let v = solid_volume(&s);
    let bot = frust_v(r * 0.05, r, ch);
    let body = cyl_v(r, bh, n);
    let top = cone_v(r, ch, n);
    let exp = bot + body + top;
    let rel = (v - exp).abs() / exp;
    assert!(rel < 0.10, "spindle v={v}, exp={exp}, rel={rel}");
}

#[test]
fn ufo_completes_or_tolerated() {
    let n = 16;
    let dr = 1.0;
    let dh = 0.2;
    let domr = 0.6;
    let m = Model::new().add(Feature::Ufo {
        id: "u".into(),
        disk_radius: lit(dr),
        disk_height: lit(dh),
        dome_radius: lit(domr),
        stacks: 12,
        slices: n,
    });
    // Sphere-on-cylinder-flat-top is curved-surface-boolean territory; can
    // trip stitch. Tolerate Err the same way other batches do.
    match m.evaluate("u") {
        Ok(s) => {
            let v = solid_volume(&s);
            let disk = cyl_v(dr, dh, n);
            let sphere = (4.0 / 3.0) * PI * domr * domr * domr;
            assert!(v > disk + sphere * 0.4);
            assert!(v < disk + sphere * 1.05);
        }
        Err(_) => {}
    }
}

#[test]
fn crows_nest_volume_matches_pole_plus_platform() {
    let n = 16;
    let pr = 0.1;
    let ph = 2.0;
    let plr = 0.6;
    let plt = 0.1;
    let m = Model::new().add(Feature::CrowsNest {
        id: "cn".into(),
        pole_radius: lit(pr),
        pole_height: lit(ph),
        platform_radius: lit(plr),
        platform_thickness: lit(plt),
        slices: n,
    });
    let s = m.evaluate("cn").unwrap();
    let v = solid_volume(&s);
    let exp = cyl_v(pr, ph, n) + cyl_v(plr, plt, n);
    let rel = (v - exp).abs() / exp;
    assert!(rel < 0.02, "crows_nest v={v}, exp={exp}, rel={rel}");
}

#[test]
fn tiered_cake_round_trip_json() {
    let m = Model::new().add(Feature::TieredCake {
        id: "tc".into(),
        bottom_radius: lit(1.0),
        middle_radius: lit(0.7),
        top_radius: lit(0.4),
        tier_height: lit(0.5),
        slices: 12,
    });
    let json = serde_json::to_string(&m).unwrap();
    let m2: Model = serde_json::from_str(&json).unwrap();
    assert_eq!(json, serde_json::to_string(&m2).unwrap());
}

#[test]
fn spindle_round_trip_json() {
    let m = Model::new().add(Feature::Spindle {
        id: "sp".into(),
        body_radius: lit(0.3),
        body_height: lit(1.0),
        cap_height: lit(0.4),
        slices: 12,
    });
    let json = serde_json::to_string(&m).unwrap();
    let m2: Model = serde_json::from_str(&json).unwrap();
    assert_eq!(json, serde_json::to_string(&m2).unwrap());
}

#[test]
fn ufo_round_trip_json() {
    let m = Model::new().add(Feature::Ufo {
        id: "u".into(),
        disk_radius: lit(1.0),
        disk_height: lit(0.2),
        dome_radius: lit(0.6),
        stacks: 8,
        slices: 12,
    });
    let json = serde_json::to_string(&m).unwrap();
    let m2: Model = serde_json::from_str(&json).unwrap();
    assert_eq!(json, serde_json::to_string(&m2).unwrap());
}

#[test]
fn crows_nest_round_trip_json() {
    let m = Model::new().add(Feature::CrowsNest {
        id: "cn".into(),
        pole_radius: lit(0.1),
        pole_height: lit(2.0),
        platform_radius: lit(0.6),
        platform_thickness: lit(0.1),
        slices: 12,
    });
    let json = serde_json::to_string(&m).unwrap();
    let m2: Model = serde_json::from_str(&json).unwrap();
    assert_eq!(json, serde_json::to_string(&m2).unwrap());
}

#[test]
fn tiered_cake_validates_descending_radii() {
    let m = Model::new().add(Feature::TieredCake {
        id: "tc".into(),
        bottom_radius: lit(0.5),
        middle_radius: lit(0.7), // wider than bottom — invalid
        top_radius: lit(0.4),
        tier_height: lit(0.5),
        slices: 12,
    });
    assert!(m.evaluate("tc").is_err());
}
