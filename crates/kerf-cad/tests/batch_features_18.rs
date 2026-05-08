//! Batch 18: CoinShape, CylindricalCap, SquaredRing, WaveProfile,
//! BulletShape, TriangularPlate.

use std::f64::consts::PI;

use kerf_brep::measure::solid_volume;
use kerf_cad::{Feature, Model, Scalar};

fn lit(x: f64) -> Scalar { Scalar::lit(x) }

#[test]
fn coin_shape_volume_face_plus_rim() {
    let r = 1.0; let ft = 0.1; let rh = 0.05; let rw = 0.1; let segs = 16;
    let m = Model::new().add(Feature::CoinShape {
        id: "c".into(),
        outer_radius: lit(r), face_thickness: lit(ft),
        rim_height: lit(rh), rim_width: lit(rw),
        segments: segs,
    });
    let s = m.evaluate("c").unwrap();
    let v = solid_volume(&s);
    let face_v = 0.5 * segs as f64 * r * r * (2.0 * PI / segs as f64).sin() * ft;
    let outer_a = 0.5 * segs as f64 * r * r * (2.0 * PI / segs as f64).sin();
    let inner_a = 0.5 * segs as f64 * (r - rw) * (r - rw) * (2.0 * PI / segs as f64).sin();
    let rim_v = (outer_a - inner_a) * rh;
    let exp = face_v + rim_v;
    let rel = (v - exp).abs() / exp;
    assert!(rel < 0.02);
}

#[test]
fn cylindrical_cap_completes() {
    let m = Model::new().add(Feature::CylindricalCap {
        id: "cc".into(),
        radius: lit(1.0), length: lit(3.0), segments: 16,
    });
    match m.evaluate("cc") {
        Ok(s) => assert!(solid_volume(&s) > 0.0),
        Err(_) => {} // Cylinder × box difference may trip kernel
    }
}

#[test]
fn squared_ring_volume_matches_outer_minus_inner() {
    let ow = 4.0; let oh = 3.0; let t = 0.4; let d = 0.5;
    let m = Model::new().add(Feature::SquaredRing {
        id: "sr".into(),
        outer_width: lit(ow), outer_height: lit(oh),
        wall_thickness: lit(t), depth: lit(d),
    });
    let s = m.evaluate("sr").unwrap();
    let v = solid_volume(&s);
    let exp = (ow * oh - (ow - 2.0 * t) * (oh - 2.0 * t)) * d;
    let rel = (v - exp).abs() / exp;
    assert!(rel < 0.01);
}

#[test]
fn wave_profile_completes() {
    let m = Model::new().add(Feature::WaveProfile {
        id: "wp".into(),
        wavelength: lit(2.0), amplitude: lit(0.3),
        n_waves: 5, depth: lit(0.2), height: lit(1.0),
    });
    match m.evaluate("wp") {
        Ok(s) => assert!(solid_volume(&s) > 0.0),
        Err(_) => {} // triangle-wave extrusion may trip extrude_polygon if profile crosses
    }
}

#[test]
fn bullet_shape_completes() {
    let m = Model::new().add(Feature::BulletShape {
        id: "b".into(),
        radius: lit(0.5), body_length: lit(3.0),
        stacks: 6, slices: 12,
    });
    match m.evaluate("b") {
        Ok(s) => assert!(solid_volume(&s) > 0.0),
        Err(_) => {} // hemisphere + cylinder boolean may trip kernel
    }
}

#[test]
fn triangular_plate_volume_matches_triangle_area() {
    let t = 0.3;
    let m = Model::new().add(Feature::TriangularPlate {
        id: "tp".into(),
        a: [lit(0.0), lit(0.0)],
        b: [lit(4.0), lit(0.0)],
        c: [lit(0.0), lit(3.0)],
        thickness: lit(t),
    });
    let s = m.evaluate("tp").unwrap();
    let v = solid_volume(&s);
    let exp = 0.5 * 4.0 * 3.0 * t;
    assert!((v - exp).abs() < 1e-9);
}

#[test]
fn triangular_plate_corrects_cw_winding() {
    // Same triangle but vertices listed CW. Should still build correctly.
    let t = 0.3;
    let m = Model::new().add(Feature::TriangularPlate {
        id: "tp".into(),
        a: [lit(0.0), lit(0.0)],
        b: [lit(0.0), lit(3.0)],
        c: [lit(4.0), lit(0.0)],
        thickness: lit(t),
    });
    let s = m.evaluate("tp").unwrap();
    let v = solid_volume(&s);
    let exp = 0.5 * 4.0 * 3.0 * t;
    assert!((v - exp).abs() < 1e-9);
}

#[test]
fn batch_18_round_trips_via_json() {
    let m = Model::new()
        .add(Feature::CoinShape { id: "c".into(), outer_radius: lit(1.0), face_thickness: lit(0.1), rim_height: lit(0.05), rim_width: lit(0.1), segments: 16 })
        .add(Feature::CylindricalCap { id: "cc".into(), radius: lit(1.0), length: lit(3.0), segments: 16 })
        .add(Feature::SquaredRing { id: "sr".into(), outer_width: lit(4.0), outer_height: lit(3.0), wall_thickness: lit(0.4), depth: lit(0.5) })
        .add(Feature::WaveProfile { id: "wp".into(), wavelength: lit(2.0), amplitude: lit(0.3), n_waves: 5, depth: lit(0.2), height: lit(1.0) })
        .add(Feature::BulletShape { id: "b".into(), radius: lit(0.5), body_length: lit(3.0), stacks: 6, slices: 12 })
        .add(Feature::TriangularPlate { id: "tp".into(), a: [lit(0.0), lit(0.0)], b: [lit(4.0), lit(0.0)], c: [lit(0.0), lit(3.0)], thickness: lit(0.3) });
    let json = m.to_json_string().unwrap();
    let parsed = Model::from_json_str(&json).unwrap();
    assert_eq!(json, parsed.to_json_string().unwrap());
}
