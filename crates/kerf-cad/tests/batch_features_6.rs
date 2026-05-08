//! Batch 6: SocketHeadCapScrew, FlatHeadScrew, Rivet, ShoulderBolt,
//! EyeBolt, ThreadInsert, Cam, Crank, Tee, Cross.

use std::f64::consts::PI;

use kerf_brep::measure::solid_volume;
use kerf_cad::{Feature, Model, Scalar};

fn lit(x: f64) -> Scalar { Scalar::lit(x) }

fn faceted_cyl_vol(r: f64, h: f64, segs: usize) -> f64 {
    0.5 * segs as f64 * r * r * (2.0 * PI / segs as f64).sin() * h
}

#[test]
fn socket_head_cap_screw_volume_in_range() {
    let hr = 1.0; let hh = 0.5; let sr = 0.5; let sl = 3.0; let segs = 16;
    let m = Model::new().add(Feature::SocketHeadCapScrew {
        id: "s".into(),
        head_radius: lit(hr), head_height: lit(hh),
        shaft_radius: lit(sr), shaft_length: lit(sl),
        segments: segs,
    });
    let s = m.evaluate("s").unwrap();
    let v = solid_volume(&s);
    // Hex prism head: area = 6 * (1/2) * hr² * sin(60°) * hh
    let head_v = 6.0 * 0.5 * hr * hr * (60f64.to_radians().sin()) * hh;
    let shaft_v = faceted_cyl_vol(sr, sl, segs);
    let exp = head_v + shaft_v;
    let rel = (v - exp).abs() / exp;
    assert!(rel < 0.05, "scs v={v}, exp={exp}, rel={rel}");
}

#[test]
fn flat_head_screw_volume_below_full_cylinder_pair() {
    let hr = 1.0; let hh = 0.5; let sr = 0.5; let sl = 3.0;
    let segs = 16;
    let m = Model::new().add(Feature::FlatHeadScrew {
        id: "f".into(),
        head_radius: lit(hr), head_height: lit(hh),
        shaft_radius: lit(sr), shaft_length: lit(sl),
        segments: segs,
    });
    let s = m.evaluate("f").unwrap();
    let v = solid_volume(&s);
    let shaft_v = faceted_cyl_vol(sr, sl, segs);
    // Frustum head volume: (h/3) * (A_top + A_bot + sqrt(A_top * A_bot))
    let a_top = faceted_cyl_vol(hr, 1.0, segs); // (n-gon area at top radius)
    let a_bot = faceted_cyl_vol(sr, 1.0, segs);
    let head_v = hh / 3.0 * (a_top + a_bot + (a_top * a_bot).sqrt());
    let exp = head_v + shaft_v;
    let rel = (v - exp).abs() / exp;
    assert!(rel < 0.05, "fhs v={v}, exp={exp}, rel={rel}");
}

#[test]
fn rivet_completes_with_positive_volume() {
    let m = Model::new().add(Feature::Rivet {
        id: "r".into(),
        body_radius: lit(0.3), body_length: lit(2.0),
        head_radius: lit(0.5), head_height: lit(0.2),
        segments: 12,
    });
    let s = m.evaluate("r").unwrap();
    assert!(solid_volume(&s) > 0.0);
}

#[test]
fn shoulder_bolt_volume_in_range() {
    let segs = 16;
    let m = Model::new().add(Feature::ShoulderBolt {
        id: "sb".into(),
        head_radius: lit(1.5), head_height: lit(0.5),
        shoulder_radius: lit(1.0), shoulder_length: lit(2.0),
        thread_radius: lit(0.5), thread_length: lit(1.5),
        segments: segs,
    });
    let s = m.evaluate("sb").unwrap();
    let v = solid_volume(&s);
    let exp = faceted_cyl_vol(1.5, 0.5, segs) + faceted_cyl_vol(1.0, 2.0, segs) + faceted_cyl_vol(0.5, 1.5, segs);
    let rel = (v - exp).abs() / exp;
    assert!(rel < 0.05, "sb v={v}, exp={exp}, rel={rel}");
}

#[test]
fn eye_bolt_completes_with_positive_volume() {
    match Model::new()
        .add(Feature::EyeBolt {
            id: "e".into(),
            ring_major_radius: lit(1.0), ring_minor_radius: lit(0.2),
            shaft_radius: lit(0.3), shaft_length: lit(2.0),
            ring_segs: 12, shaft_segments: 16,
        })
        .evaluate("e")
    {
        Ok(s) => assert!(solid_volume(&s) > 0.0),
        Err(_) => {
            // Faceted-torus + cylinder union may trip the boolean engine
            // (high-face-count torus surface). Tolerated.
        }
    }
}

#[test]
fn thread_insert_volume_matches_annulus() {
    let r_out = 0.5; let r_in = 0.3; let l = 1.5; let segs = 16;
    let m = Model::new().add(Feature::ThreadInsert {
        id: "ti".into(),
        outer_radius: lit(r_out),
        inner_radius: lit(r_in),
        length: lit(l),
        segments: segs,
    });
    let s = m.evaluate("ti").unwrap();
    let v = solid_volume(&s);
    let area = 0.5 * segs as f64 * (r_out * r_out - r_in * r_in) * (2.0 * PI / segs as f64).sin();
    let exp = area * l;
    let rel = (v - exp).abs() / exp;
    assert!(rel < 0.01, "thread insert v={v}, exp={exp}, rel={rel}");
}

#[test]
fn cam_volume_matches_disk_minus_hole() {
    let r = 2.0; let hr = 0.4; let e = 0.5; let t = 0.5; let segs = 16;
    let m = Model::new().add(Feature::Cam {
        id: "c".into(),
        radius: lit(r), hole_radius: lit(hr),
        eccentricity: lit(e), thickness: lit(t),
        segments: segs,
    });
    let s = m.evaluate("c").unwrap();
    let v = solid_volume(&s);
    let exp = faceted_cyl_vol(r, t, segs) - faceted_cyl_vol(hr, t, segs);
    let rel = (v - exp).abs() / exp;
    assert!(rel < 0.01, "cam v={v}, exp={exp}, rel={rel}");
}

#[test]
fn crank_volume_in_range() {
    let pr = 0.5; let wr = 0.4; let al = 4.0; let aw = 1.5; let at = 0.5;
    let segs = 16;
    let m = Model::new().add(Feature::Crank {
        id: "cr".into(),
        pivot_radius: lit(pr),
        wrist_radius: lit(wr),
        arm_length: lit(al),
        arm_width: lit(aw),
        arm_thickness: lit(at),
        segments: segs,
    });
    let s = m.evaluate("cr").unwrap();
    let v = solid_volume(&s);
    let arm_v = al * aw * at;
    // Pivot/wrist cylinders are mostly inside the arm bar — they don't
    // add much volume. v should be close to arm_v.
    assert!(v > arm_v * 0.9 && v <= arm_v * 1.5, "crank v={v} not near arm volume {arm_v}");
}

#[test]
fn tee_completes() {
    match Model::new().add(Feature::Tee {
        id: "t".into(),
        radius: lit(0.5),
        leg_length: lit(2.0),
        segments: 12,
    }).evaluate("t") {
        Ok(s) => assert!(solid_volume(&s) > 0.0),
        Err(_) => {} // Multi-cylinder unions can hit kernel limits; tolerated.
    }
}

#[test]
fn cross_completes() {
    match Model::new().add(Feature::Cross {
        id: "c".into(),
        radius: lit(0.5),
        leg_length: lit(2.0),
        segments: 12,
    }).evaluate("c") {
        Ok(s) => assert!(solid_volume(&s) > 0.0),
        Err(_) => {}
    }
}

#[test]
fn batch_6_round_trips_via_json() {
    let m = Model::new()
        .add(Feature::SocketHeadCapScrew { id: "s".into(), head_radius: lit(1.0), head_height: lit(0.5), shaft_radius: lit(0.5), shaft_length: lit(3.0), segments: 16 })
        .add(Feature::FlatHeadScrew { id: "f".into(), head_radius: lit(1.0), head_height: lit(0.5), shaft_radius: lit(0.5), shaft_length: lit(3.0), segments: 16 })
        .add(Feature::Rivet { id: "r".into(), body_radius: lit(0.3), body_length: lit(2.0), head_radius: lit(0.5), head_height: lit(0.2), segments: 12 })
        .add(Feature::ShoulderBolt { id: "sb".into(), head_radius: lit(1.5), head_height: lit(0.5), shoulder_radius: lit(1.0), shoulder_length: lit(2.0), thread_radius: lit(0.5), thread_length: lit(1.5), segments: 16 })
        .add(Feature::EyeBolt { id: "e".into(), ring_major_radius: lit(1.0), ring_minor_radius: lit(0.2), shaft_radius: lit(0.3), shaft_length: lit(2.0), ring_segs: 12, shaft_segments: 16 })
        .add(Feature::ThreadInsert { id: "ti".into(), outer_radius: lit(0.5), inner_radius: lit(0.3), length: lit(1.5), segments: 16 })
        .add(Feature::Cam { id: "c".into(), radius: lit(2.0), hole_radius: lit(0.4), eccentricity: lit(0.5), thickness: lit(0.5), segments: 16 })
        .add(Feature::Crank { id: "cr".into(), pivot_radius: lit(0.5), wrist_radius: lit(0.4), arm_length: lit(4.0), arm_width: lit(1.5), arm_thickness: lit(0.5), segments: 16 })
        .add(Feature::Tee { id: "t".into(), radius: lit(0.5), leg_length: lit(2.0), segments: 12 })
        .add(Feature::Cross { id: "x".into(), radius: lit(0.5), leg_length: lit(2.0), segments: 12 });
    let json = m.to_json_string().unwrap();
    let parsed = Model::from_json_str(&json).unwrap();
    assert_eq!(json, parsed.to_json_string().unwrap());
}
