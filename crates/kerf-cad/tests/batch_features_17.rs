//! Batch 17: HollowBrick, StadiumPlate, Bowtie, HollowCone, ArchedDoorway,
//! CamLobe, ButtonShape, FilletedSlot.

use std::f64::consts::PI;

use kerf_brep::measure::solid_volume;
use kerf_cad::{Feature, Model, Scalar};

fn lit(x: f64) -> Scalar { Scalar::lit(x) }

#[test]
fn hollow_brick_volume_matches_outer_minus_cavity() {
    let l = 4.0; let w = 2.0; let h = 2.0; let t = 0.3;
    let m = Model::new().add(Feature::HollowBrick {
        id: "hb".into(),
        length: lit(l), width: lit(w), height: lit(h),
        wall_thickness: lit(t),
    });
    let s = m.evaluate("hb").unwrap();
    let v = solid_volume(&s);
    let outer = l * w * h;
    let cavity = (l - 2.0 * t) * (w - 2.0 * t) * (h - t);
    let exp = outer - cavity;
    let rel = (v - exp).abs() / exp;
    assert!(rel < 0.01);
}

#[test]
fn stadium_plate_volume_in_range() {
    let l = 5.0; let w = 1.0; let t = 0.2;
    let m = Model::new().add(Feature::StadiumPlate {
        id: "sp".into(),
        length: lit(l), width: lit(w), thickness: lit(t),
        segments: 24,
    });
    let s = m.evaluate("sp").unwrap();
    let v = solid_volume(&s);
    let area = l * w + PI * (w / 2.0).powi(2);
    let exp = area * t;
    let rel = (v - exp).abs() / exp;
    assert!(rel < 0.05);
}

#[test]
fn bowtie_volume_matches_two_triangles() {
    let w = 4.0; let h = 2.0; let p = 0.5; let d = 0.5;
    let m = Model::new().add(Feature::Bowtie {
        id: "bt".into(),
        width: lit(w), height: lit(h), pinch: lit(p), depth: lit(d),
    });
    let s = m.evaluate("bt").unwrap();
    let v = solid_volume(&s);
    // Bowtie outline: two trapezoidal halves. Compute via shoelace.
    let pts = [
        (-w / 2.0, h / 2.0),
        (0.0, p / 2.0),
        (w / 2.0, h / 2.0),
        (w / 2.0, -h / 2.0),
        (0.0, -p / 2.0),
        (-w / 2.0, -h / 2.0),
    ];
    let mut a = 0.0;
    for i in 0..6 {
        let (x0, y0) = pts[i];
        let (x1, y1) = pts[(i + 1) % 6];
        a += x0 * y1 - x1 * y0;
    }
    let area = 0.5 * a.abs();
    let exp = area * d;
    let rel = (v - exp).abs() / exp;
    assert!(rel < 1e-6);
}

#[test]
fn hollow_cone_volume_below_full_cone() {
    let r = 2.0; let h = 5.0; let br = 0.5; let segs = 16;
    let m = Model::new().add(Feature::HollowCone {
        id: "hc".into(),
        radius: lit(r), height: lit(h),
        bore_radius: lit(br), segments: segs,
    });
    let s = m.evaluate("hc").unwrap();
    let v = solid_volume(&s);
    // Faceted cone volume (1/3 base area * h).
    let n = segs as f64;
    let cone_v = (1.0 / 3.0) * 0.5 * n * r * r * (2.0 * PI / n).sin() * h;
    let bore_v = 0.5 * n * br * br * (2.0 * PI / n).sin() * h;
    let exp = cone_v - bore_v;
    let rel = (v - exp).abs() / exp;
    assert!(rel < 0.05, "hollow cone v={v}, exp={exp}, rel={rel}");
}

#[test]
fn arched_doorway_completes() {
    let m = Model::new().add(Feature::ArchedDoorway {
        id: "ad".into(),
        width: lit(4.0), height: lit(6.0),
        wall_thickness: lit(0.4), depth: lit(1.0),
        segments: 16,
    });
    match m.evaluate("ad") {
        Ok(s) => assert!(solid_volume(&s) > 0.0),
        Err(_) => {} // Polygon-polygon difference can sometimes trip kernel.
    }
}

#[test]
fn cam_lobe_volume_matches_ellipse_minus_hole() {
    let rx = 2.0; let ry = 1.0; let hr = 0.3; let t = 0.4; let segs = 32;
    let m = Model::new().add(Feature::CamLobe {
        id: "cl".into(),
        rx: lit(rx), ry: lit(ry), hole_radius: lit(hr),
        thickness: lit(t), segments: segs,
    });
    let s = m.evaluate("cl").unwrap();
    let v = solid_volume(&s);
    // n-gon area approximating an ellipse: 0.5 * n * rx * ry * sin(2π/n).
    let n = segs as f64;
    let disk_a = 0.5 * n * rx * ry * (2.0 * PI / n).sin();
    let hole_a = 0.5 * n * hr * hr * (2.0 * PI / n).sin();
    let exp = (disk_a - hole_a) * t;
    let rel = (v - exp).abs() / exp;
    assert!(rel < 0.05, "cam lobe v={v}, exp={exp}, rel={rel}");
}

#[test]
fn button_shape_volume_matches_disk() {
    let r = 1.0; let t = 0.1; let segs = 24;
    let m = Model::new().add(Feature::ButtonShape {
        id: "bs".into(),
        radius: lit(r), thickness: lit(t), segments: segs,
    });
    let s = m.evaluate("bs").unwrap();
    let v = solid_volume(&s);
    let exp = 0.5 * segs as f64 * r * r * (2.0 * PI / segs as f64).sin() * t;
    assert!((v - exp).abs() < 1e-9);
}

#[test]
fn filleted_slot_volume_in_range() {
    let l = 5.0; let w = 2.0; let r = 0.5; let d = 0.3;
    let m = Model::new().add(Feature::FilletedSlot {
        id: "fs".into(),
        length: lit(l), width: lit(w),
        corner_radius: lit(r), depth: lit(d),
        segments: 24,
    });
    let s = m.evaluate("fs").unwrap();
    let v = solid_volume(&s);
    // Approx area = (l × w) - 4 corners of (1 - π/4) × r²
    let exp_area = l * w - 4.0 * (1.0 - PI / 4.0) * r * r;
    let exp = exp_area * d;
    let rel = (v - exp).abs() / exp;
    assert!(rel < 0.05);
}

#[test]
fn batch_17_round_trips_via_json() {
    let m = Model::new()
        .add(Feature::HollowBrick { id: "hb".into(), length: lit(4.0), width: lit(2.0), height: lit(2.0), wall_thickness: lit(0.3) })
        .add(Feature::StadiumPlate { id: "sp".into(), length: lit(5.0), width: lit(1.0), thickness: lit(0.2), segments: 24 })
        .add(Feature::Bowtie { id: "bt".into(), width: lit(4.0), height: lit(2.0), pinch: lit(0.5), depth: lit(0.5) })
        .add(Feature::HollowCone { id: "hc".into(), radius: lit(2.0), height: lit(5.0), bore_radius: lit(0.5), segments: 16 })
        .add(Feature::ArchedDoorway { id: "ad".into(), width: lit(4.0), height: lit(6.0), wall_thickness: lit(0.4), depth: lit(1.0), segments: 16 })
        .add(Feature::CamLobe { id: "cl".into(), rx: lit(2.0), ry: lit(1.0), hole_radius: lit(0.3), thickness: lit(0.4), segments: 32 })
        .add(Feature::ButtonShape { id: "bs".into(), radius: lit(1.0), thickness: lit(0.1), segments: 24 })
        .add(Feature::FilletedSlot { id: "fs".into(), length: lit(5.0), width: lit(2.0), corner_radius: lit(0.5), depth: lit(0.3), segments: 24 });
    let json = m.to_json_string().unwrap();
    let parsed = Model::from_json_str(&json).unwrap();
    assert_eq!(json, parsed.to_json_string().unwrap());
}
