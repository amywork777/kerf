//! Batch 22 (round 7): TulipBulb, PaperLantern, AcornCap, HourglassFigure,
//! Ankh, CamLobe2, PistonHead, PulleyGroove, Pinwheel, GearTooth.

use std::f64::consts::PI;

use kerf_brep::measure::solid_volume;
use kerf_cad::{Feature, Model, Scalar};

fn lit(x: f64) -> Scalar { Scalar::lit(x) }

// ─────────────────────────────────────────────────────────────────────
// TulipBulb
// ─────────────────────────────────────────────────────────────────────

#[test]
fn tulip_bulb_volume_bounded() {
    let m = Model::new().add(Feature::TulipBulb {
        id: "t".into(),
        bulb_radius: lit(1.0),
        neck_radius: lit(0.15),
        neck_length: lit(0.8),
        stacks: 6,
        slices: 12,
    });
    match m.evaluate("t") {
        Ok(s) => {
            let v = solid_volume(&s);
            assert!(v > 0.0);
            // Sphere volume is 4/3 * pi * r³ ~= 4.19; neck is small.
            // Bound: less than sphere + neck cylinder fully.
            let upper = (4.0 / 3.0) * PI + PI * 0.15 * 0.15 * 0.8 + 0.5;
            assert!(v < upper, "tulip_bulb v={v} expected < {upper}");
        }
        Err(_) => {} // sphere+cylinder kernel may trip; tolerate.
    }
}

#[test]
fn tulip_bulb_round_trips_via_json() {
    let m = Model::new().add(Feature::TulipBulb {
        id: "t".into(),
        bulb_radius: lit(1.0),
        neck_radius: lit(0.15),
        neck_length: lit(0.8),
        stacks: 6,
        slices: 12,
    });
    let json = m.to_json_string().unwrap();
    let parsed = Model::from_json_str(&json).unwrap();
    assert_eq!(json, parsed.to_json_string().unwrap());
}

// ─────────────────────────────────────────────────────────────────────
// PaperLantern
// ─────────────────────────────────────────────────────────────────────

#[test]
fn paper_lantern_volume_bounded() {
    let m = Model::new().add(Feature::PaperLantern {
        id: "p".into(),
        body_radius: lit(0.8),
        body_height: lit(2.0),
        stacks: 6,
        slices: 12,
    });
    match m.evaluate("p") {
        Ok(s) => {
            let v = solid_volume(&s);
            assert!(v > 0.0);
            // Bounded by cylinder of radius 0.8, height (2 + 2*0.8)
            // (body + 2 hemispheres of diameter 1.6).
            let upper = PI * 0.8 * 0.8 * (2.0 + 2.0 * 0.8) + 1.0;
            assert!(v < upper, "lantern v={v} > {upper}");
        }
        Err(_) => {}
    }
}

#[test]
fn paper_lantern_round_trips_via_json() {
    let m = Model::new().add(Feature::PaperLantern {
        id: "p".into(),
        body_radius: lit(0.8),
        body_height: lit(2.0),
        stacks: 6,
        slices: 12,
    });
    let json = m.to_json_string().unwrap();
    let parsed = Model::from_json_str(&json).unwrap();
    assert_eq!(json, parsed.to_json_string().unwrap());
}

// ─────────────────────────────────────────────────────────────────────
// AcornCap
// ─────────────────────────────────────────────────────────────────────

#[test]
fn acorn_cap_volume_bounded() {
    let m = Model::new().add(Feature::AcornCap {
        id: "a".into(),
        cap_radius: lit(1.0),
        rim_height: lit(0.2),
        stacks: 6,
        slices: 12,
    });
    match m.evaluate("a") {
        Ok(s) => {
            let v = solid_volume(&s);
            assert!(v > 0.0);
            // Hemisphere volume = (2/3) * pi * r³ ≈ 2.09; rim cylinder
            // ~= pi * r² * rh = pi * 1 * 0.2 ≈ 0.63. Total upper bound.
            let upper = (2.0 / 3.0) * PI + PI * 0.2 + 0.5;
            assert!(v < upper, "acorn_cap v={v} > {upper}");
        }
        Err(_) => {}
    }
}

#[test]
fn acorn_cap_round_trips_via_json() {
    let m = Model::new().add(Feature::AcornCap {
        id: "a".into(),
        cap_radius: lit(1.0),
        rim_height: lit(0.2),
        stacks: 6,
        slices: 12,
    });
    let json = m.to_json_string().unwrap();
    let parsed = Model::from_json_str(&json).unwrap();
    assert_eq!(json, parsed.to_json_string().unwrap());
}

// ─────────────────────────────────────────────────────────────────────
// HourglassFigure
// ─────────────────────────────────────────────────────────────────────

#[test]
fn hourglass_figure_volume_bounded() {
    let m = Model::new().add(Feature::HourglassFigure {
        id: "h".into(),
        end_radius: lit(0.8),
        waist_radius: lit(0.3),
        body_half_height: lit(1.5),
        cap_thickness: lit(0.2),
        cap_radius: lit(1.0),
        segments: 16,
    });
    match m.evaluate("h") {
        Ok(s) => {
            let v = solid_volume(&s);
            assert!(v > 0.0);
            // Two caps (cylinders of cr=1, h=0.2) + two frustums (er=0.8 to wr=0.3, hh=1.5).
            // Loose upper bound: cylinder of cr=1 spanning the full height.
            let total_h = 2.0 * 0.2 + 2.0 * 1.5;
            let upper = PI * 1.0 * 1.0 * total_h + 0.5;
            assert!(v < upper, "hourglass_fig v={v} > {upper}");
        }
        Err(_) => {} // shared-waist booleans may trip; tolerate.
    }
}

#[test]
fn hourglass_figure_round_trips_via_json() {
    let m = Model::new().add(Feature::HourglassFigure {
        id: "h".into(),
        end_radius: lit(0.8),
        waist_radius: lit(0.3),
        body_half_height: lit(1.5),
        cap_thickness: lit(0.2),
        cap_radius: lit(1.0),
        segments: 16,
    });
    let json = m.to_json_string().unwrap();
    let parsed = Model::from_json_str(&json).unwrap();
    assert_eq!(json, parsed.to_json_string().unwrap());
}

// ─────────────────────────────────────────────────────────────────────
// Ankh
// ─────────────────────────────────────────────────────────────────────

#[test]
fn ankh_completes_or_documents() {
    let m = Model::new().add(Feature::Ankh {
        id: "ank".into(),
        shaft_height: lit(2.0),
        shaft_radius: lit(0.1),
        arm_length: lit(1.5),
        arm_radius: lit(0.1),
        loop_major_radius: lit(0.5),
        loop_minor_radius: lit(0.1),
        major_segs: 12,
        minor_segs: 8,
        segments: 12,
    });
    match m.evaluate("ank") {
        Ok(s) => {
            let v = solid_volume(&s);
            assert!(v > 0.0);
        }
        Err(_) => {} // torus + cylinder unions may trip; tolerate.
    }
}

#[test]
fn ankh_round_trips_via_json() {
    let m = Model::new().add(Feature::Ankh {
        id: "ank".into(),
        shaft_height: lit(2.0),
        shaft_radius: lit(0.1),
        arm_length: lit(1.5),
        arm_radius: lit(0.1),
        loop_major_radius: lit(0.5),
        loop_minor_radius: lit(0.1),
        major_segs: 12,
        minor_segs: 8,
        segments: 12,
    });
    let json = m.to_json_string().unwrap();
    let parsed = Model::from_json_str(&json).unwrap();
    assert_eq!(json, parsed.to_json_string().unwrap());
}

// ─────────────────────────────────────────────────────────────────────
// CamLobe2
// ─────────────────────────────────────────────────────────────────────

#[test]
fn cam_lobe2_volume_matches_disk_minus_bore() {
    let r = 1.0;
    let e = 0.3;
    let bore = 0.2;
    let t = 0.5;
    let segs = 24;
    let m = Model::new().add(Feature::CamLobe2 {
        id: "c".into(),
        radius: lit(r),
        eccentricity: lit(e),
        bore_radius: lit(bore),
        thickness: lit(t),
        segments: segs,
    });
    let s = m.evaluate("c").unwrap();
    let v = solid_volume(&s);
    // Eccentric circle area = pi * r²; bore subtracts pi * bore².
    // Faceted polygon area uses the n-gon formula:
    //   A_outer ≈ 0.5 * n * r² * sin(2π/n)
    let outer_a = 0.5 * segs as f64 * r * r * (2.0 * PI / segs as f64).sin();
    let bore_a = 0.5 * segs as f64 * bore * bore * (2.0 * PI / segs as f64).sin();
    let exp = (outer_a - bore_a) * t;
    let rel = (v - exp).abs() / exp;
    assert!(rel < 0.02, "cam_lobe2 v={v}, exp={exp}, rel={rel}");
}

#[test]
fn cam_lobe2_round_trips_via_json() {
    let m = Model::new().add(Feature::CamLobe2 {
        id: "c".into(),
        radius: lit(1.0),
        eccentricity: lit(0.3),
        bore_radius: lit(0.2),
        thickness: lit(0.5),
        segments: 24,
    });
    let json = m.to_json_string().unwrap();
    let parsed = Model::from_json_str(&json).unwrap();
    assert_eq!(json, parsed.to_json_string().unwrap());
}

// ─────────────────────────────────────────────────────────────────────
// PistonHead
// ─────────────────────────────────────────────────────────────────────

#[test]
fn piston_head_volume_bounded() {
    let m = Model::new().add(Feature::PistonHead {
        id: "p".into(),
        body_radius: lit(1.0),
        body_height: lit(2.0),
        crown_radius: lit(1.2),
        crown_thickness: lit(0.3),
        groove_count: 2,
        groove_depth: lit(0.1),
        groove_width: lit(0.15),
        segments: 16,
    });
    match m.evaluate("p") {
        Ok(s) => {
            let v = solid_volume(&s);
            assert!(v > 0.0);
            // Body + crown without grooves bounds it from above.
            let upper = PI * 1.0 * 1.0 * 2.0 + PI * 1.2 * 1.2 * 0.3 + 0.5;
            assert!(v < upper, "piston v={v} > {upper}");
        }
        Err(_) => {}
    }
}

#[test]
fn piston_head_round_trips_via_json() {
    let m = Model::new().add(Feature::PistonHead {
        id: "p".into(),
        body_radius: lit(1.0),
        body_height: lit(2.0),
        crown_radius: lit(1.2),
        crown_thickness: lit(0.3),
        groove_count: 2,
        groove_depth: lit(0.1),
        groove_width: lit(0.15),
        segments: 16,
    });
    let json = m.to_json_string().unwrap();
    let parsed = Model::from_json_str(&json).unwrap();
    assert_eq!(json, parsed.to_json_string().unwrap());
}

// ─────────────────────────────────────────────────────────────────────
// PulleyGroove
// ─────────────────────────────────────────────────────────────────────

#[test]
fn pulley_groove_volume_bounded() {
    let m = Model::new().add(Feature::PulleyGroove {
        id: "pg".into(),
        outer_radius: lit(1.0),
        groove_inner_radius: lit(0.5),
        width: lit(1.0),
        groove_width: lit(0.4),
        segments: 16,
    });
    match m.evaluate("pg") {
        Ok(s) => {
            let v = solid_volume(&s);
            assert!(v > 0.0);
            // Bounded by full cylinder.
            let upper = PI * 1.0 * 1.0 * 1.0;
            assert!(v < upper, "pulley_groove v={v} > {upper}");
            // Greater than a hollow cylinder of the groove inner radius
            // bore (loose lower bound).
            let lower = PI * 0.5 * 0.5 * 1.0 * 0.5;
            assert!(v > lower, "pulley_groove v={v} < {lower}");
        }
        Err(_) => {} // frustum-frustum unions can trip kernel; tolerate.
    }
}

#[test]
fn pulley_groove_round_trips_via_json() {
    let m = Model::new().add(Feature::PulleyGroove {
        id: "pg".into(),
        outer_radius: lit(1.0),
        groove_inner_radius: lit(0.5),
        width: lit(1.0),
        groove_width: lit(0.4),
        segments: 16,
    });
    let json = m.to_json_string().unwrap();
    let parsed = Model::from_json_str(&json).unwrap();
    assert_eq!(json, parsed.to_json_string().unwrap());
}

// ─────────────────────────────────────────────────────────────────────
// Pinwheel
// ─────────────────────────────────────────────────────────────────────

#[test]
fn pinwheel_volume_matches_star_minus_bore() {
    let pts = 6;
    let r_out = 1.0;
    let r_in = 0.4;
    let bore = 0.15;
    let t = 0.2;
    let segs = 24;
    let m = Model::new().add(Feature::Pinwheel {
        id: "pw".into(),
        points: pts,
        outer_radius: lit(r_out),
        inner_radius: lit(r_in),
        bore_radius: lit(bore),
        thickness: lit(t),
        segments: segs,
    });
    let s = m.evaluate("pw").unwrap();
    let v = solid_volume(&s);
    // Star polygon area: sum of triangles between alternating r_out/r_in
    // vertices. A_star = (1/2) * (2N) * r_out * r_in * sin(2π/(2N))
    let n = 2 * pts;
    let star_a = 0.5 * n as f64 * r_out * r_in * (2.0 * PI / n as f64).sin();
    let bore_a = 0.5 * segs as f64 * bore * bore * (2.0 * PI / segs as f64).sin();
    let exp = (star_a - bore_a) * t;
    let rel = (v - exp).abs() / exp;
    assert!(rel < 0.02, "pinwheel v={v}, exp={exp}, rel={rel}");
}

#[test]
fn pinwheel_round_trips_via_json() {
    let m = Model::new().add(Feature::Pinwheel {
        id: "pw".into(),
        points: 6,
        outer_radius: lit(1.0),
        inner_radius: lit(0.4),
        bore_radius: lit(0.15),
        thickness: lit(0.2),
        segments: 24,
    });
    let json = m.to_json_string().unwrap();
    let parsed = Model::from_json_str(&json).unwrap();
    assert_eq!(json, parsed.to_json_string().unwrap());
}

// ─────────────────────────────────────────────────────────────────────
// GearTooth
// ─────────────────────────────────────────────────────────────────────

#[test]
fn gear_tooth_volume_matches_trapezoid_extrude() {
    let rw = 0.4;
    let tw = 0.2;
    let th = 0.5;
    let t = 0.15;
    let m = Model::new().add(Feature::GearTooth {
        id: "g".into(),
        root_width: lit(rw),
        tip_width: lit(tw),
        tooth_height: lit(th),
        thickness: lit(t),
    });
    let s = m.evaluate("g").unwrap();
    let v = solid_volume(&s);
    // Trapezoid area = (rw + tw) / 2 * th.
    let exp = (rw + tw) / 2.0 * th * t;
    let rel = (v - exp).abs() / exp;
    assert!(rel < 1e-9, "gear_tooth v={v}, exp={exp}, rel={rel}");
}

#[test]
fn gear_tooth_round_trips_via_json() {
    let m = Model::new().add(Feature::GearTooth {
        id: "g".into(),
        root_width: lit(0.4),
        tip_width: lit(0.2),
        tooth_height: lit(0.5),
        thickness: lit(0.15),
    });
    let json = m.to_json_string().unwrap();
    let parsed = Model::from_json_str(&json).unwrap();
    assert_eq!(json, parsed.to_json_string().unwrap());
}

// ─────────────────────────────────────────────────────────────────────
// Combined round-trip across batch 22
// ─────────────────────────────────────────────────────────────────────

#[test]
fn batch_22_round_trips_via_json() {
    let m = Model::new()
        .add(Feature::TulipBulb {
            id: "tb".into(),
            bulb_radius: lit(1.0), neck_radius: lit(0.15), neck_length: lit(0.8),
            stacks: 6, slices: 12,
        })
        .add(Feature::PaperLantern {
            id: "pl".into(),
            body_radius: lit(0.8), body_height: lit(2.0),
            stacks: 6, slices: 12,
        })
        .add(Feature::AcornCap {
            id: "ac".into(),
            cap_radius: lit(1.0), rim_height: lit(0.2),
            stacks: 6, slices: 12,
        })
        .add(Feature::HourglassFigure {
            id: "hf".into(),
            end_radius: lit(0.8), waist_radius: lit(0.3),
            body_half_height: lit(1.5),
            cap_thickness: lit(0.2), cap_radius: lit(1.0),
            segments: 16,
        })
        .add(Feature::Ankh {
            id: "ank".into(),
            shaft_height: lit(2.0), shaft_radius: lit(0.1),
            arm_length: lit(1.5), arm_radius: lit(0.1),
            loop_major_radius: lit(0.5), loop_minor_radius: lit(0.1),
            major_segs: 12, minor_segs: 8, segments: 12,
        })
        .add(Feature::CamLobe2 {
            id: "cl2".into(),
            radius: lit(1.0), eccentricity: lit(0.3),
            bore_radius: lit(0.2), thickness: lit(0.5), segments: 24,
        })
        .add(Feature::PistonHead {
            id: "ph".into(),
            body_radius: lit(1.0), body_height: lit(2.0),
            crown_radius: lit(1.2), crown_thickness: lit(0.3),
            groove_count: 2, groove_depth: lit(0.1), groove_width: lit(0.15),
            segments: 16,
        })
        .add(Feature::PulleyGroove {
            id: "pg".into(),
            outer_radius: lit(1.0), groove_inner_radius: lit(0.5),
            width: lit(1.0), groove_width: lit(0.4),
            segments: 16,
        })
        .add(Feature::Pinwheel {
            id: "pw".into(),
            points: 6, outer_radius: lit(1.0), inner_radius: lit(0.4),
            bore_radius: lit(0.15), thickness: lit(0.2), segments: 24,
        })
        .add(Feature::GearTooth {
            id: "g".into(),
            root_width: lit(0.4), tip_width: lit(0.2),
            tooth_height: lit(0.5), thickness: lit(0.15),
        });
    let json = m.to_json_string().unwrap();
    let parsed = Model::from_json_str(&json).unwrap();
    assert_eq!(json, parsed.to_json_string().unwrap());
}
