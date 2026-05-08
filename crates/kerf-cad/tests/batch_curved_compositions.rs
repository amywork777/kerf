//! Batch: Mushroom, Lightbulb, DomedRoof, Beehive — curved-surface
//! compositions of existing primitives. Volumes verified against
//! per-piece bounds (sphere/cylinder overlap is hard to close-form).

use std::f64::consts::PI;

use kerf_brep::measure::solid_volume;
use kerf_cad::{Feature, Model, Scalar};

fn lit(x: f64) -> Scalar {
    Scalar::lit(x)
}

fn faceted_cylinder_volume(r: f64, h: f64, slices: usize) -> f64 {
    // Faceted cylinder cross-section is a regular polygon, not a perfect
    // circle. Area = (1/2) * n * r^2 * sin(2π/n).
    0.5 * slices as f64 * r * r * (2.0 * PI / slices as f64).sin() * h
}

fn faceted_sphere_volume(r: f64, _stacks: usize, _slices: usize) -> f64 {
    // Approximated as analytic sphere; faceted sphere is slightly smaller
    // but within a few percent for reasonable stacks/slices.
    (4.0 / 3.0) * PI * r * r * r
}

#[test]
fn mushroom_volume_within_bounds() {
    let stem_r = 0.4;
    let stem_h = 1.5;
    let cap_r = 1.2;
    let stacks = 12;
    let slices = 16;
    let m = Model::new().add(Feature::Mushroom {
        id: "m".into(),
        stem_radius: lit(stem_r),
        stem_height: lit(stem_h),
        cap_radius: lit(cap_r),
        stacks,
        slices,
    });
    let s = m.evaluate("m").unwrap();
    let v = solid_volume(&s);
    let stem_vol = faceted_cylinder_volume(stem_r, stem_h, slices);
    let cap_vol = faceted_sphere_volume(cap_r, stacks, slices);
    // Frustum collar between stem and cap (height = min(cap-stem, stem*0.5)).
    let collar_h = (cap_r - stem_r).min(stem_h * 0.5).max(1e-2);
    let collar_vol = (collar_h / 3.0) * PI * (stem_r * stem_r + stem_r * cap_r + cap_r * cap_r);
    // Lower bound: at least the stem.
    assert!(v > stem_vol * 0.9, "mushroom v={v} below stem alone {stem_vol}");
    // Upper bound: stem + collar + cap, with slack for faceted vs analytic.
    assert!(
        v < (stem_vol + collar_vol + cap_vol) * 1.10,
        "mushroom v={v} exceeds stem+collar+cap {}",
        stem_vol + collar_vol + cap_vol
    );
}

#[test]
fn lightbulb_volume_within_bounds() {
    let basr = 0.3;
    let bash = 0.4;
    let nh = 0.5;
    let bulb_r = 1.0;
    let stacks = 12;
    let slices = 16;
    let m = Model::new().add(Feature::Lightbulb {
        id: "lb".into(),
        base_radius: lit(basr),
        base_height: lit(bash),
        neck_height: lit(nh),
        bulb_radius: lit(bulb_r),
        stacks,
        slices,
    });
    let s = m.evaluate("lb").unwrap();
    let v = solid_volume(&s);
    let base_vol = faceted_cylinder_volume(basr, bash, slices);
    let bulb_vol = faceted_sphere_volume(bulb_r, stacks, slices);
    // Frustum volume formula: (h/3)*(r1^2 + r1*r2 + r2^2) * (n/2π)*sin(2π/n) approx π
    let neck_vol = (nh / 3.0) * PI * (basr * basr + basr * bulb_r + bulb_r * bulb_r);
    // Lower bound: at least the bulb
    assert!(v > bulb_vol * 0.9, "lightbulb v={v} below bulb alone {bulb_vol}");
    // Upper bound: slack for analytic-vs-faceted differences
    assert!(
        v < (base_vol + neck_vol + bulb_vol) * 1.1,
        "lightbulb v={v} exceeds sum {}",
        base_vol + neck_vol + bulb_vol
    );
}

#[test]
fn domed_roof_volume_matches_cylinder_plus_half_sphere() {
    let r = 1.0;
    let wh = 2.0;
    let stacks = 12;
    let slices = 16;
    let m = Model::new().add(Feature::DomedRoof {
        id: "dr".into(),
        radius: lit(r),
        wall_height: lit(wh),
        stacks,
        slices,
    });
    let s = m.evaluate("dr").unwrap();
    let v = solid_volume(&s);
    // DomedRoof volume = cylinder + top half of sphere (bottom half is
    // entirely buried inside the cylinder top).
    let cyl_vol = faceted_cylinder_volume(r, wh, slices);
    let half_sphere = 0.5 * faceted_sphere_volume(r, stacks, slices);
    let exp = cyl_vol + half_sphere;
    let rel = (v - exp).abs() / exp;
    assert!(rel < 0.10, "domed_roof v={v}, exp={exp}, rel={rel}");
}

#[test]
fn beehive_volume_matches_layer_sum() {
    let r0 = 1.0;
    let lh = 0.5;
    let layers = 4;
    let slices = 16;
    let m = Model::new().add(Feature::Beehive {
        id: "bh".into(),
        base_radius: lit(r0),
        layer_height: lit(lh),
        layers,
        slices,
    });
    let s = m.evaluate("bh").unwrap();
    let v = solid_volume(&s);
    // Each layer's radius shrinks linearly to 0.4 * r0 at the top.
    let mut exp = 0.0;
    for i in 0..layers {
        let frac = i as f64 / (layers as f64 - 1.0);
        let r_i = r0 * (1.0 - 0.6 * frac);
        exp += faceted_cylinder_volume(r_i, lh, slices);
    }
    // Each layer overlaps its neighbour by 1e-3 * faceted_area, negligible.
    let rel = (v - exp).abs() / exp;
    assert!(rel < 0.02, "beehive v={v}, exp={exp}, rel={rel}");
}

#[test]
fn mushroom_round_trip_json() {
    let m = Model::new().add(Feature::Mushroom {
        id: "m".into(),
        stem_radius: lit(0.5),
        stem_height: lit(1.0),
        cap_radius: lit(1.5),
        stacks: 8,
        slices: 12,
    });
    let json = serde_json::to_string(&m).unwrap();
    let m2: Model = serde_json::from_str(&json).unwrap();
    let json2 = serde_json::to_string(&m2).unwrap();
    assert_eq!(json, json2);
}

#[test]
fn lightbulb_round_trip_json() {
    let m = Model::new().add(Feature::Lightbulb {
        id: "lb".into(),
        base_radius: lit(0.3),
        base_height: lit(0.4),
        neck_height: lit(0.5),
        bulb_radius: lit(1.0),
        stacks: 8,
        slices: 12,
    });
    let json = serde_json::to_string(&m).unwrap();
    let m2: Model = serde_json::from_str(&json).unwrap();
    let json2 = serde_json::to_string(&m2).unwrap();
    assert_eq!(json, json2);
}

#[test]
fn domed_roof_round_trip_json() {
    let m = Model::new().add(Feature::DomedRoof {
        id: "dr".into(),
        radius: lit(1.0),
        wall_height: lit(2.0),
        stacks: 8,
        slices: 12,
    });
    let json = serde_json::to_string(&m).unwrap();
    let m2: Model = serde_json::from_str(&json).unwrap();
    let json2 = serde_json::to_string(&m2).unwrap();
    assert_eq!(json, json2);
}

#[test]
fn beehive_round_trip_json() {
    let m = Model::new().add(Feature::Beehive {
        id: "bh".into(),
        base_radius: lit(1.0),
        layer_height: lit(0.5),
        layers: 4,
        slices: 16,
    });
    let json = serde_json::to_string(&m).unwrap();
    let m2: Model = serde_json::from_str(&json).unwrap();
    let json2 = serde_json::to_string(&m2).unwrap();
    assert_eq!(json, json2);
}

#[test]
fn mushroom_validates_invalid_dims() {
    let m = Model::new().add(Feature::Mushroom {
        id: "m".into(),
        stem_radius: lit(1.0),
        stem_height: lit(1.0),
        cap_radius: lit(0.5), // smaller than stem
        stacks: 8,
        slices: 12,
    });
    assert!(m.evaluate("m").is_err(), "Mushroom with cap<stem should error");
}
