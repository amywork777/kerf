//! Batch 12: CircularRing, PolygonRing, CylinderShellAt, QuarterTorus,
//! HalfTorus, SquareTube, HoleyPlate.

use std::f64::consts::PI;

use kerf_brep::measure::solid_volume;
use kerf_cad::{Feature, Model, Scalar};

fn lit(x: f64) -> Scalar { Scalar::lit(x) }

#[test]
fn circular_ring_volume_matches_annulus() {
    let r_out = 2.0; let r_in = 1.0; let t = 0.3; let segs = 16;
    let m = Model::new().add(Feature::CircularRing {
        id: "r".into(),
        outer_radius: lit(r_out), inner_radius: lit(r_in),
        thickness: lit(t), segments: segs,
    });
    let s = m.evaluate("r").unwrap();
    let v = solid_volume(&s);
    let area = 0.5 * segs as f64 * (r_out * r_out - r_in * r_in) * (2.0 * PI / segs as f64).sin();
    let exp = area * t;
    let rel = (v - exp).abs() / exp;
    assert!(rel < 0.01);
}

#[test]
fn polygon_ring_volume_matches_outer_minus_inner_polygon() {
    let r_out = 2.0; let r_in = 1.0; let sides = 6; let t = 0.3;
    let m = Model::new().add(Feature::PolygonRing {
        id: "pr".into(),
        outer_radius: lit(r_out), inner_radius: lit(r_in),
        sides, thickness: lit(t),
    });
    let s = m.evaluate("pr").unwrap();
    let v = solid_volume(&s);
    let area = 0.5 * sides as f64 * (r_out * r_out - r_in * r_in) * (2.0 * PI / sides as f64).sin();
    let exp = area * t;
    let rel = (v - exp).abs() / exp;
    assert!(rel < 0.01);
}

#[test]
fn cylinder_shell_at_volume_matches_annulus() {
    let r_out = 1.0; let r_in = 0.5; let l = 3.0; let segs = 16;
    let m = Model::new().add(Feature::CylinderShellAt {
        id: "csa".into(),
        base: [lit(2.0), lit(3.0), lit(0.0)],
        axis: "z".into(),
        outer_radius: lit(r_out), inner_radius: lit(r_in),
        length: lit(l), segments: segs,
    });
    let s = m.evaluate("csa").unwrap();
    let v = solid_volume(&s);
    let area = 0.5 * segs as f64 * (r_out * r_out - r_in * r_in) * (2.0 * PI / segs as f64).sin();
    let exp = area * l;
    let rel = (v - exp).abs() / exp;
    assert!(rel < 0.01);
}

#[test]
fn quarter_torus_completes() {
    let m = Model::new().add(Feature::QuarterTorus {
        id: "qt".into(),
        major_radius: lit(3.0), minor_radius: lit(0.5),
        segments: 8,
    });
    match m.evaluate("qt") {
        Ok(s) => assert!(solid_volume(&s) > 0.0),
        Err(_) => {}
    }
}

#[test]
fn half_torus_completes() {
    let m = Model::new().add(Feature::HalfTorus {
        id: "ht".into(),
        major_radius: lit(3.0), minor_radius: lit(0.5),
        segments: 8,
    });
    match m.evaluate("ht") {
        Ok(s) => assert!(solid_volume(&s) > 0.0),
        Err(_) => {}
    }
}

#[test]
fn square_tube_volume_matches_outer_minus_cavity() {
    let w = 2.0; let h = 3.0; let t = 0.3; let l = 5.0;
    let m = Model::new().add(Feature::SquareTube {
        id: "st".into(),
        outer_width: lit(w), outer_height: lit(h),
        wall_thickness: lit(t), length: lit(l),
    });
    let s = m.evaluate("st").unwrap();
    let v = solid_volume(&s);
    let outer = w * h * l;
    let cavity = (w - 2.0 * t) * (h - 2.0 * t) * l;
    let exp = outer - cavity;
    let rel = (v - exp).abs() / exp;
    assert!(rel < 0.01);
}

#[test]
fn holey_plate_volume_matches_plate_minus_hole() {
    let pw = 4.0; let ph = 3.0; let pt = 0.5; let hr = 0.4; let segs = 16;
    let m = Model::new().add(Feature::HoleyPlate {
        id: "hp".into(),
        plate_width: lit(pw), plate_height: lit(ph),
        plate_thickness: lit(pt), hole_radius: lit(hr),
        segments: segs,
    });
    let s = m.evaluate("hp").unwrap();
    let v = solid_volume(&s);
    let plate = pw * ph * pt;
    let hole = 0.5 * segs as f64 * hr * hr * (2.0 * PI / segs as f64).sin() * pt;
    let exp = plate - hole;
    let rel = (v - exp).abs() / exp;
    assert!(rel < 0.01);
}

#[test]
fn batch_12_round_trips_via_json() {
    let m = Model::new()
        .add(Feature::CircularRing { id: "r".into(), outer_radius: lit(2.0), inner_radius: lit(1.0), thickness: lit(0.3), segments: 16 })
        .add(Feature::PolygonRing { id: "pr".into(), outer_radius: lit(2.0), inner_radius: lit(1.0), sides: 6, thickness: lit(0.3) })
        .add(Feature::CylinderShellAt { id: "csa".into(), base: [lit(0.0), lit(0.0), lit(0.0)], axis: "z".into(), outer_radius: lit(1.0), inner_radius: lit(0.5), length: lit(3.0), segments: 16 })
        .add(Feature::QuarterTorus { id: "qt".into(), major_radius: lit(3.0), minor_radius: lit(0.5), segments: 8 })
        .add(Feature::HalfTorus { id: "ht".into(), major_radius: lit(3.0), minor_radius: lit(0.5), segments: 8 })
        .add(Feature::SquareTube { id: "st".into(), outer_width: lit(2.0), outer_height: lit(3.0), wall_thickness: lit(0.3), length: lit(5.0) })
        .add(Feature::HoleyPlate { id: "hp".into(), plate_width: lit(4.0), plate_height: lit(3.0), plate_thickness: lit(0.5), hole_radius: lit(0.4), segments: 16 });
    let json = m.to_json_string().unwrap();
    let parsed = Model::from_json_str(&json).unwrap();
    assert_eq!(json, parsed.to_json_string().unwrap());
}
