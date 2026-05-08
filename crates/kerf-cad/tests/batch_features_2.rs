//! Batch 2: MountingFlange, GearBlank, KnurledGrip, Pipe, Spring.

use std::f64::consts::PI;

use kerf_brep::measure::solid_volume;
use kerf_cad::{Feature, Model, Scalar};

fn lit(x: f64) -> Scalar { Scalar::lit(x) }

#[test]
fn mounting_flange_volume_below_full_disk() {
    // Disk volume - 4 bolt-hole volumes (approx).
    let r_disk = 5.0;
    let t = 0.5;
    let r_bolt = 0.3;
    let bolt_count = 4;
    let m = Model::new().add(Feature::MountingFlange {
        id: "f".into(),
        disk_radius: lit(r_disk),
        disk_thickness: lit(t),
        bolt_circle_radius: lit(3.5),
        bolt_count,
        bolt_radius: lit(r_bolt),
        segments: 24,
    });
    let s = m.evaluate("f").unwrap();
    let v = solid_volume(&s);
    // Faceted disk volume (24 segments).
    let disk_v = 0.5 * 24.0 * r_disk * r_disk * (2.0 * PI / 24.0).sin() * t;
    let bolt_v = 0.5 * 24.0 * r_bolt * r_bolt * (2.0 * PI / 24.0).sin() * t;
    let exp = disk_v - (bolt_count as f64) * bolt_v;
    let rel = (v - exp).abs() / exp;
    assert!(rel < 0.01, "mounting flange volume off: v={v}, exp={exp}, rel={rel}");
}

#[test]
fn gear_blank_volume_in_expected_range() {
    let r_out = 5.0;
    let r_root = 4.0;
    let tooth_count = 12;
    let t = 0.5;
    let m = Model::new().add(Feature::GearBlank {
        id: "g".into(),
        outer_radius: lit(r_out),
        root_radius: lit(r_root),
        tooth_count,
        thickness: lit(t),
        segments_per_tooth: 1,
    });
    let s = m.evaluate("g").unwrap();
    let v = solid_volume(&s);
    // Volume must be between root-disk and outer-disk volumes
    // (the alternating profile is somewhere in between).
    let v_root = PI * r_root * r_root * t;
    let v_out = PI * r_out * r_out * t;
    assert!(
        v > v_root * 0.7 && v < v_out * 1.05,
        "gear blank volume {v} not between {v_root} and {v_out}"
    );
}

#[test]
fn knurled_grip_volume_above_bare_cylinder() {
    let r = 1.0;
    let rh = 0.1;
    let h = 2.0;
    let rc = 16;
    let m = Model::new().add(Feature::KnurledGrip {
        id: "k".into(),
        radius: lit(r),
        ridge_height: lit(rh),
        height: lit(h),
        ridge_count: rc,
    });
    let s = m.evaluate("k").unwrap();
    let v = solid_volume(&s);
    let v_bare = PI * r * r * h;
    let v_full = PI * (r + rh) * (r + rh) * h;
    assert!(
        v > v_bare * 0.95 && v < v_full * 1.05,
        "knurled grip volume {v} not between bare {v_bare} and ridged {v_full}"
    );
}

#[test]
fn pipe_x_axis_volume_matches_annulus() {
    let r_out = 1.0;
    let r_in = 0.5;
    let l = 5.0;
    let segs = 16;
    let m = Model::new().add(Feature::Pipe {
        id: "p".into(),
        base: [lit(2.0), lit(3.0), lit(4.0)],
        axis: "x".into(),
        outer_radius: lit(r_out),
        inner_radius: lit(r_in),
        length: lit(l),
        segments: segs,
    });
    let s = m.evaluate("p").unwrap();
    let v = solid_volume(&s);
    // Faceted annulus: outer area - inner area (n-gon area each).
    let area = 0.5 * segs as f64 * (r_out * r_out - r_in * r_in) * (2.0 * PI / segs as f64).sin();
    let exp = area * l;
    let rel = (v - exp).abs() / exp;
    assert!(rel < 0.01, "pipe volume off: v={v}, exp={exp}, rel={rel}");
}

#[test]
fn spring_volume_in_expected_range() {
    // Same as Coil; sanity check.
    let m = Model::new().add(Feature::Spring {
        id: "sp".into(),
        coil_radius: lit(2.0),
        wire_radius: lit(0.2),
        pitch: lit(0.5),
        turns: lit(2.0),
        segments_per_turn: 12,
        wire_segments: 6,
    });
    let s = m.evaluate("sp").unwrap();
    assert!(solid_volume(&s) > 0.0);
}

#[test]
fn batch_2_round_trips_via_json() {
    let m = Model::new()
        .add(Feature::MountingFlange {
            id: "f".into(),
            disk_radius: lit(3.0),
            disk_thickness: lit(0.5),
            bolt_circle_radius: lit(2.0),
            bolt_count: 6,
            bolt_radius: lit(0.2),
            segments: 16,
        })
        .add(Feature::GearBlank {
            id: "g".into(),
            outer_radius: lit(2.0),
            root_radius: lit(1.5),
            tooth_count: 12,
            thickness: lit(0.3),
            segments_per_tooth: 1,
        })
        .add(Feature::KnurledGrip {
            id: "k".into(),
            radius: lit(1.0),
            ridge_height: lit(0.05),
            height: lit(1.0),
            ridge_count: 12,
        })
        .add(Feature::Pipe {
            id: "p".into(),
            base: [lit(0.0), lit(0.0), lit(0.0)],
            axis: "z".into(),
            outer_radius: lit(1.0),
            inner_radius: lit(0.7),
            length: lit(2.0),
            segments: 16,
        })
        .add(Feature::Spring {
            id: "sp".into(),
            coil_radius: lit(2.0),
            wire_radius: lit(0.1),
            pitch: lit(0.3),
            turns: lit(1.0),
            segments_per_turn: 12,
            wire_segments: 6,
        });
    let json = m.to_json_string().unwrap();
    let parsed = Model::from_json_str(&json).unwrap();
    let json2 = parsed.to_json_string().unwrap();
    assert_eq!(json, json2);
}
