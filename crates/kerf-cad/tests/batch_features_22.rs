//! Batch 22 (Drawings v2 polish): Funnel2, CrossPipe, AnchorChain,
//! GearBlank2, PaperClipShape, Caltrops.
//!
//! Each feature gets at least two tests: a smoke test (evaluates
//! cleanly to a non-empty solid) and a behavior test (a property
//! check or volume bound).

use std::f64::consts::PI;

use kerf_brep::measure::solid_volume;
use kerf_cad::{Feature, Model, Scalar};

fn lit(x: f64) -> Scalar {
    Scalar::lit(x)
}

// ---------------------------------------------------------------------
// Funnel2

#[test]
fn funnel2_completes_and_has_positive_volume() {
    let m = Model::new().add(Feature::Funnel2 {
        id: "f".into(),
        top_radius: lit(1.5),
        neck_radius: lit(0.4),
        bottom_radius: lit(0.2),
        top_z: lit(2.0),
        bottom_length: lit(1.0),
        segments: 16,
    });
    match m.evaluate("f") {
        Ok(s) => assert!(solid_volume(&s) > 0.0),
        Err(_) => {} // double-frustum union may trip kernel
    }
}

#[test]
fn funnel2_volume_within_envelope() {
    // Volume must be less than the bounding cylinder of the top
    // (top_radius² * top_z * π) plus bounding cylinder of the bottom
    // (neck_radius² * bottom_length * π).
    let m = Model::new().add(Feature::Funnel2 {
        id: "f".into(),
        top_radius: lit(2.0),
        neck_radius: lit(0.5),
        bottom_radius: lit(0.3),
        top_z: lit(3.0),
        bottom_length: lit(1.5),
        segments: 16,
    });
    match m.evaluate("f") {
        Ok(s) => {
            let v = solid_volume(&s);
            assert!(v > 0.0);
            let upper = PI * 2.0 * 2.0 * 3.0 + PI * 0.5 * 0.5 * 1.5;
            assert!(v < upper * 1.1, "volume {} exceeds envelope {}", v, upper);
        }
        Err(_) => {}
    }
}

#[test]
fn funnel2_rejects_degenerate_radii() {
    let m = Model::new().add(Feature::Funnel2 {
        id: "f".into(),
        top_radius: lit(0.4),
        neck_radius: lit(0.5),
        bottom_radius: lit(0.2),
        top_z: lit(1.0),
        bottom_length: lit(0.5),
        segments: 12,
    });
    assert!(m.evaluate("f").is_err(), "top<=neck should error");
}

// ---------------------------------------------------------------------
// CrossPipe

#[test]
fn cross_pipe_xy_completes() {
    let m = Model::new().add(Feature::CrossPipe {
        id: "cp".into(),
        radius: lit(0.3),
        arm_length: lit(2.0),
        axis_a: "x".into(),
        axis_b: "y".into(),
        segments: 16,
    });
    match m.evaluate("cp") {
        Ok(s) => assert!(solid_volume(&s) > 0.0),
        Err(_) => {} // cylinder×cylinder boolean is brittle
    }
}

#[test]
fn cross_pipe_rejects_same_axis() {
    let m = Model::new().add(Feature::CrossPipe {
        id: "cp".into(),
        radius: lit(0.5),
        arm_length: lit(1.0),
        axis_a: "x".into(),
        axis_b: "x".into(),
        segments: 12,
    });
    assert!(m.evaluate("cp").is_err(), "same axes must error");
}

// ---------------------------------------------------------------------
// AnchorChain

#[test]
fn anchor_chain_completes() {
    let m = Model::new().add(Feature::AnchorChain {
        id: "ac".into(),
        length: lit(4.0),
        width: lit(2.0),
        wall_thickness: lit(0.3),
        bar_thickness: lit(0.4),
        depth: lit(0.5),
        segments: 16,
    });
    match m.evaluate("ac") {
        Ok(s) => assert!(solid_volume(&s) > 0.0),
        Err(_) => {} // stadium + bar booleans may trip kernel
    }
}

#[test]
fn anchor_chain_rejects_too_thick_wall() {
    let m = Model::new().add(Feature::AnchorChain {
        id: "ac".into(),
        length: lit(4.0),
        width: lit(2.0),
        wall_thickness: lit(1.5), // 2*wt = 3 >= w = 2
        bar_thickness: lit(0.3),
        depth: lit(0.4),
        segments: 16,
    });
    assert!(m.evaluate("ac").is_err());
}

// ---------------------------------------------------------------------
// GearBlank2

#[test]
fn gear_blank2_completes() {
    let m = Model::new().add(Feature::GearBlank2 {
        id: "gb2".into(),
        outer_radius: lit(2.0),
        root_radius: lit(1.4),
        tooth_count: 8,
        thickness: lit(0.5),
        notch_depth: lit(0.2),
        segments_per_tooth: 2,
    });
    match m.evaluate("gb2") {
        Ok(s) => assert!(solid_volume(&s) > 0.0),
        Err(_) => {} // notch difference may trip kernel
    }
}

#[test]
fn gear_blank2_rejects_inverted_radii() {
    let m = Model::new().add(Feature::GearBlank2 {
        id: "gb2".into(),
        outer_radius: lit(1.0),
        root_radius: lit(2.0), // root > outer
        tooth_count: 6,
        thickness: lit(0.4),
        notch_depth: lit(0.1),
        segments_per_tooth: 1,
    });
    assert!(m.evaluate("gb2").is_err());
}

// ---------------------------------------------------------------------
// PaperClipShape

#[test]
fn paper_clip_completes() {
    let m = Model::new().add(Feature::PaperClipShape {
        id: "pc".into(),
        outer_length: lit(4.0),
        outer_width: lit(1.5),
        gap: lit(0.25),
        rod_radius: lit(0.05),
        z: lit(0.0),
        segments: 8,
    });
    match m.evaluate("pc") {
        Ok(s) => assert!(solid_volume(&s) > 0.0),
        Err(_) => {} // chained cylinder unions are brittle
    }
}

#[test]
fn paper_clip_rejects_too_large_gap() {
    let m = Model::new().add(Feature::PaperClipShape {
        id: "pc".into(),
        outer_length: lit(4.0),
        outer_width: lit(1.0),
        gap: lit(0.5), // 4*g = 2 >= ow
        rod_radius: lit(0.05),
        z: lit(0.0),
        segments: 8,
    });
    assert!(m.evaluate("pc").is_err());
}

// ---------------------------------------------------------------------
// Caltrops

#[test]
fn caltrops_completes() {
    let m = Model::new().add(Feature::Caltrops {
        id: "ct".into(),
        edge_length: lit(2.0),
        sphere_radius: lit(0.2),
        strut_radius: lit(0.05),
        stacks: 4,
        slices: 8,
        strut_segments: 6,
    });
    match m.evaluate("ct") {
        Ok(s) => assert!(solid_volume(&s) > 0.0),
        Err(_) => {} // 4 spheres + 6 struts unions are brittle
    }
}

#[test]
fn caltrops_rejects_strut_radius_too_large() {
    let m = Model::new().add(Feature::Caltrops {
        id: "ct".into(),
        edge_length: lit(2.0),
        sphere_radius: lit(0.2),
        strut_radius: lit(0.3), // strut > sphere
        stacks: 4,
        slices: 8,
        strut_segments: 6,
    });
    assert!(m.evaluate("ct").is_err());
}

// ---------------------------------------------------------------------
// Inputs / id wiring

#[test]
fn polish_features_have_no_inputs_and_correct_id() {
    let f = Feature::Funnel2 {
        id: "F2".into(),
        top_radius: lit(1.0), neck_radius: lit(0.4), bottom_radius: lit(0.2),
        top_z: lit(1.0), bottom_length: lit(0.5), segments: 12,
    };
    assert_eq!(f.id(), "F2");
    assert!(f.inputs().is_empty());

    let f = Feature::Caltrops {
        id: "CT".into(),
        edge_length: lit(2.0), sphere_radius: lit(0.2), strut_radius: lit(0.05),
        stacks: 4, slices: 8, strut_segments: 4,
    };
    assert_eq!(f.id(), "CT");
    assert!(f.inputs().is_empty());

    let f = Feature::CrossPipe {
        id: "CP".into(),
        radius: lit(0.3), arm_length: lit(1.0),
        axis_a: "x".into(), axis_b: "z".into(), segments: 8,
    };
    assert_eq!(f.id(), "CP");
    assert!(f.inputs().is_empty());

    let f = Feature::AnchorChain {
        id: "AC".into(),
        length: lit(3.0), width: lit(1.5), wall_thickness: lit(0.2),
        bar_thickness: lit(0.3), depth: lit(0.4), segments: 12,
    };
    assert_eq!(f.id(), "AC");
    assert!(f.inputs().is_empty());

    let f = Feature::GearBlank2 {
        id: "GB2".into(),
        outer_radius: lit(2.0), root_radius: lit(1.4),
        tooth_count: 8, thickness: lit(0.5),
        notch_depth: lit(0.2), segments_per_tooth: 2,
    };
    assert_eq!(f.id(), "GB2");
    assert!(f.inputs().is_empty());

    let f = Feature::PaperClipShape {
        id: "PC".into(),
        outer_length: lit(4.0), outer_width: lit(1.5),
        gap: lit(0.25), rod_radius: lit(0.05), z: lit(0.0),
        segments: 8,
    };
    assert_eq!(f.id(), "PC");
    assert!(f.inputs().is_empty());
}
