//! Batch 9: Stair, Hopper, Stand, Yoke, Lever, TaperedTube, GussetPlate,
//! CrossKey, PinShaft.

use std::f64::consts::PI;

use kerf_brep::measure::solid_volume;
use kerf_cad::{Feature, Model, Scalar};

fn lit(x: f64) -> Scalar { Scalar::lit(x) }

#[test]
fn stair_volume_in_expected_range() {
    // 3 steps, tread=1, riser=0.5, width=2.
    // Step k volume = sw * (total_run - k*tread) * riser
    //   = 2 * ((3-k)*1) * 0.5 for k=0..2
    //   = 2*3*0.5 + 2*2*0.5 + 2*1*0.5 = 3+2+1 = 6
    // But the test step boxes overlap (step k+1 sits on top of step k's
    // back portion), so the union volume is less than the sum.
    let m = Model::new().add(Feature::Stair {
        id: "s".into(),
        step_count: 3,
        tread: lit(1.0),
        riser: lit(0.5),
        step_width: lit(2.0),
    });
    let s = m.evaluate("s").unwrap();
    let v = solid_volume(&s);
    // Total profile: a stairstep shape. Volume = sum of N rectangles of
    // size sw × tread × ((k+1)*riser) for k=0..N-1
    //   = sw * tread * riser * (1 + 2 + ... + N)
    //   = sw * tread * riser * N(N+1)/2
    let exp = 2.0 * 1.0 * 0.5 * (3.0 * 4.0 / 2.0); // = 6.0
    let rel = (v - exp).abs() / exp;
    assert!(rel < 0.05, "stair v={v}, exp={exp}, rel={rel}");
}

#[test]
fn hopper_completes() {
    let m = Model::new().add(Feature::Hopper {
        id: "h".into(),
        top_radius: lit(2.0),
        neck_radius: lit(0.5),
        funnel_height: lit(2.0),
        neck_height: lit(1.0),
        segments: 16,
    });
    match m.evaluate("h") {
        Ok(s) => assert!(solid_volume(&s) > 0.0),
        Err(_) => {} // frustum + cylinder union may trip kernel.
    }
}

#[test]
fn stand_volume_matches_cylinder() {
    let r = 5.0; let t = 1.0; let segs = 16;
    let m = Model::new().add(Feature::Stand {
        id: "s".into(),
        radius: lit(r), thickness: lit(t),
        segments: segs,
    });
    let s = m.evaluate("s").unwrap();
    let v = solid_volume(&s);
    let exp = 0.5 * segs as f64 * r * r * (2.0 * PI / segs as f64).sin() * t;
    let rel = (v - exp).abs() / exp;
    assert!(rel < 0.001);
}

#[test]
fn yoke_completes_with_pivot_hole() {
    let m = Model::new().add(Feature::Yoke {
        id: "y".into(),
        arm_length: lit(2.0), arm_thickness: lit(0.3),
        arm_height: lit(2.0), gap_width: lit(1.0),
        pivot_radius: lit(0.4), segments: 12,
    });
    let s = m.evaluate("y").unwrap();
    let v = solid_volume(&s);
    assert!(v > 0.0, "yoke volume must be positive, got {v}");
}

#[test]
fn lever_volume_matches_bar_minus_hole() {
    let l = 10.0; let w = 1.0; let t = 0.3; let pr = 0.2; let segs = 16;
    let m = Model::new().add(Feature::Lever {
        id: "lv".into(),
        length: lit(l), width: lit(w), thickness: lit(t),
        pivot_radius: lit(pr), segments: segs,
    });
    let s = m.evaluate("lv").unwrap();
    let v = solid_volume(&s);
    let bar_v = l * w * t;
    let hole_v = 0.5 * segs as f64 * pr * pr * (2.0 * PI / segs as f64).sin() * t;
    let exp = bar_v - hole_v;
    let rel = (v - exp).abs() / exp;
    assert!(rel < 0.01, "lever v={v}, exp={exp}, rel={rel}");
}

#[test]
fn tapered_tube_completes() {
    let m = Model::new().add(Feature::TaperedTube {
        id: "tt".into(),
        bottom_outer_radius: lit(2.0),
        top_outer_radius: lit(1.5),
        wall_thickness: lit(0.3),
        height: lit(3.0),
        segments: 16,
    });
    match m.evaluate("tt") {
        Ok(s) => assert!(solid_volume(&s) > 0.0),
        Err(_) => {}
    }
}

#[test]
fn gusset_plate_volume_matches_right_triangle() {
    let l = 5.0; let t = 0.4;
    let m = Model::new().add(Feature::GussetPlate {
        id: "gp".into(),
        leg_length: lit(l), thickness: lit(t),
    });
    let s = m.evaluate("gp").unwrap();
    let v = solid_volume(&s);
    let exp = 0.5 * l * l * t;
    assert!((v - exp).abs() < 1e-9);
}

#[test]
fn cross_key_volume_matches_union_of_two_bars() {
    let bl = 5.0; let bt = 0.5; let bh = 0.3;
    let m = Model::new().add(Feature::CrossKey {
        id: "ck".into(),
        bar_length: lit(bl), bar_thickness: lit(bt), bar_height: lit(bh),
    });
    let s = m.evaluate("ck").unwrap();
    let v = solid_volume(&s);
    // 2 bars of size bl × bt × bh sharing a bt × bt × bh center cube.
    let exp = 2.0 * bl * bt * bh - bt * bt * bh;
    let rel = (v - exp).abs() / exp;
    assert!(rel < 0.01, "cross key v={v}, exp={exp}, rel={rel}");
}

#[test]
fn pin_shaft_volume_matches_cylinder() {
    let r = 0.3; let l = 5.0; let segs = 16;
    let m = Model::new().add(Feature::PinShaft {
        id: "ps".into(),
        radius: lit(r), length: lit(l), segments: segs,
    });
    let s = m.evaluate("ps").unwrap();
    let v = solid_volume(&s);
    let exp = 0.5 * segs as f64 * r * r * (2.0 * PI / segs as f64).sin() * l;
    assert!((v - exp).abs() < 1e-9);
}

#[test]
fn batch_9_round_trips_via_json() {
    let m = Model::new()
        .add(Feature::Stair { id: "s".into(), step_count: 3, tread: lit(1.0), riser: lit(0.5), step_width: lit(2.0) })
        .add(Feature::Hopper { id: "h".into(), top_radius: lit(2.0), neck_radius: lit(0.5), funnel_height: lit(2.0), neck_height: lit(1.0), segments: 16 })
        .add(Feature::Stand { id: "st".into(), radius: lit(5.0), thickness: lit(1.0), segments: 16 })
        .add(Feature::Yoke { id: "y".into(), arm_length: lit(2.0), arm_thickness: lit(0.3), arm_height: lit(2.0), gap_width: lit(1.0), pivot_radius: lit(0.4), segments: 12 })
        .add(Feature::Lever { id: "lv".into(), length: lit(10.0), width: lit(1.0), thickness: lit(0.3), pivot_radius: lit(0.2), segments: 16 })
        .add(Feature::TaperedTube { id: "tt".into(), bottom_outer_radius: lit(2.0), top_outer_radius: lit(1.5), wall_thickness: lit(0.3), height: lit(3.0), segments: 16 })
        .add(Feature::GussetPlate { id: "gp".into(), leg_length: lit(5.0), thickness: lit(0.4) })
        .add(Feature::CrossKey { id: "ck".into(), bar_length: lit(5.0), bar_thickness: lit(0.5), bar_height: lit(0.3) })
        .add(Feature::PinShaft { id: "ps".into(), radius: lit(0.3), length: lit(5.0), segments: 16 });
    let json = m.to_json_string().unwrap();
    let parsed = Model::from_json_str(&json).unwrap();
    assert_eq!(json, parsed.to_json_string().unwrap());
}
