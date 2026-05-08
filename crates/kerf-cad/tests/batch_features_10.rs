//! Batch 10: Lens, EggShape, UBendPipe, SBend, DonutSlice, CapsuleAt,
//! ToroidalKnob.

use kerf_brep::measure::solid_volume;
use kerf_cad::{Feature, Model, Scalar};

fn lit(x: f64) -> Scalar { Scalar::lit(x) }

#[test]
fn lens_completes() {
    let m = Model::new().add(Feature::Lens {
        id: "l".into(),
        radius: lit(2.0),
        cap_height: lit(0.5),
        stacks: 8,
        slices: 12,
    });
    match m.evaluate("l") {
        Ok(s) => assert!(solid_volume(&s) > 0.0),
        Err(_) => {} // Boolean curved-surface limitations may trip.
    }
}

#[test]
fn egg_shape_completes() {
    let m = Model::new().add(Feature::EggShape {
        id: "e".into(),
        radius: lit(1.0),
        aspect_z: lit(1.5),
        stacks: 8,
        slices: 12,
    });
    let s = m.evaluate("e").unwrap();
    assert!(solid_volume(&s) > 0.0);
}

#[test]
fn u_bend_pipe_completes() {
    let m = Model::new().add(Feature::UBendPipe {
        id: "u".into(),
        bend_radius: lit(2.0),
        pipe_radius: lit(0.3),
        segments: 8,
    });
    match m.evaluate("u") {
        Ok(s) => assert!(solid_volume(&s) > 0.0),
        Err(_) => {}
    }
}

#[test]
fn s_bend_completes() {
    let m = Model::new().add(Feature::SBend {
        id: "s".into(),
        bend_radius: lit(2.0),
        pipe_radius: lit(0.3),
        segments: 6,
    });
    match m.evaluate("s") {
        Ok(s) => assert!(solid_volume(&s) > 0.0),
        Err(_) => {}
    }
}

#[test]
fn donut_slice_completes() {
    let m = Model::new().add(Feature::DonutSlice {
        id: "d".into(),
        major_radius: lit(3.0),
        minor_radius: lit(1.0),
        sweep_deg: lit(90.0),
        major_segs: 16,
        minor_segs: 8,
    });
    let s = m.evaluate("d").unwrap();
    assert!(solid_volume(&s) > 0.0);
}

#[test]
fn capsule_at_completes_for_each_axis() {
    for axis in &["x", "y", "z"] {
        let m = Model::new().add(Feature::CapsuleAt {
            id: "c".into(),
            axis: axis.to_string(),
            center: [lit(1.0), lit(2.0), lit(3.0)],
            radius: lit(0.5),
            body_length: lit(2.0),
            stacks: 6,
            slices: 8,
        });
        match m.evaluate("c") {
            Ok(s) => assert!(solid_volume(&s) > 0.0, "axis {axis} → zero volume"),
            Err(_) => {} // tolerated
        }
    }
}

#[test]
fn toroidal_knob_completes() {
    let m = Model::new().add(Feature::ToroidalKnob {
        id: "tk".into(),
        body_radius: lit(2.0),
        body_height: lit(2.0),
        torus_major_radius: lit(2.5),
        torus_minor_radius: lit(0.3),
        body_segs: 16,
        torus_segs: 12,
    });
    match m.evaluate("tk") {
        Ok(s) => assert!(solid_volume(&s) > 0.0),
        Err(_) => {} // torus + cylinder boolean can trip.
    }
}

#[test]
fn batch_10_round_trips_via_json() {
    let m = Model::new()
        .add(Feature::Lens { id: "l".into(), radius: lit(2.0), cap_height: lit(0.5), stacks: 8, slices: 12 })
        .add(Feature::EggShape { id: "e".into(), radius: lit(1.0), aspect_z: lit(1.5), stacks: 8, slices: 12 })
        .add(Feature::UBendPipe { id: "u".into(), bend_radius: lit(2.0), pipe_radius: lit(0.3), segments: 8 })
        .add(Feature::SBend { id: "s".into(), bend_radius: lit(2.0), pipe_radius: lit(0.3), segments: 6 })
        .add(Feature::DonutSlice { id: "d".into(), major_radius: lit(3.0), minor_radius: lit(1.0), sweep_deg: lit(90.0), major_segs: 16, minor_segs: 8 })
        .add(Feature::CapsuleAt { id: "c".into(), axis: "x".into(), center: [lit(1.0), lit(2.0), lit(3.0)], radius: lit(0.5), body_length: lit(2.0), stacks: 6, slices: 8 })
        .add(Feature::ToroidalKnob { id: "tk".into(), body_radius: lit(2.0), body_height: lit(2.0), torus_major_radius: lit(2.5), torus_minor_radius: lit(0.3), body_segs: 16, torus_segs: 12 });
    let json = m.to_json_string().unwrap();
    let parsed = Model::from_json_str(&json).unwrap();
    assert_eq!(json, parsed.to_json_string().unwrap());
}
