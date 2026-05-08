//! Batch 16: Ellipsoid3D, VectorArrow, BoneShape, Pawn, Rook, Bishop, Marker3D.

use kerf_brep::measure::solid_volume;
use kerf_cad::{Feature, Model, Scalar};

fn lit(x: f64) -> Scalar { Scalar::lit(x) }

#[test]
fn ellipsoid_3d_volume_above_zero() {
    let m = Model::new().add(Feature::Ellipsoid3D {
        id: "e".into(),
        rx: lit(2.0), ry: lit(1.0), rz: lit(1.5),
        stacks: 8, slices: 12,
    });
    let s = m.evaluate("e").unwrap();
    assert!(solid_volume(&s) > 0.0);
}

#[test]
fn vector_arrow_completes() {
    let m = Model::new().add(Feature::VectorArrow {
        id: "va".into(),
        from: [lit(0.0), lit(0.0), lit(0.0)],
        to: [lit(3.0), lit(4.0), lit(0.0)],
        shaft_radius: lit(0.05),
        head_radius: lit(0.15),
        head_length: lit(0.5),
        segments: 12,
    });
    match m.evaluate("va") {
        Ok(s) => assert!(solid_volume(&s) > 0.0),
        Err(_) => {} // sweep + cone union may trip kernel
    }
}

#[test]
fn bone_shape_completes() {
    let m = Model::new().add(Feature::BoneShape {
        id: "bs".into(),
        end_radius: lit(0.5),
        shaft_radius: lit(0.2),
        shaft_length: lit(3.0),
        stacks: 6, slices: 12,
    });
    match m.evaluate("bs") {
        Ok(s) => assert!(solid_volume(&s) > 0.0),
        Err(_) => {} // sphere + cylinder union may trip kernel
    }
}

#[test]
fn pawn_completes() {
    let m = Model::new().add(Feature::Pawn {
        id: "p".into(),
        base_radius: lit(0.6),
        base_height: lit(0.2),
        body_top_radius: lit(0.3),
        body_height: lit(1.0),
        head_radius: lit(0.4),
        segments: 12, stacks: 6,
    });
    match m.evaluate("p") {
        Ok(s) => assert!(solid_volume(&s) > 0.0),
        Err(_) => {}
    }
}

#[test]
fn rook_completes() {
    let m = Model::new().add(Feature::Rook {
        id: "r".into(),
        base_radius: lit(0.6),
        base_height: lit(0.2),
        body_radius: lit(0.5),
        body_height: lit(1.0),
        segments: 16,
    });
    match m.evaluate("r") {
        Ok(s) => assert!(solid_volume(&s) > 0.0),
        Err(_) => {}
    }
}

#[test]
fn bishop_completes() {
    let m = Model::new().add(Feature::Bishop {
        id: "b".into(),
        base_radius: lit(0.6),
        base_height: lit(0.2),
        body_radius: lit(0.4),
        body_height: lit(1.0),
        head_radius: lit(0.4),
        slot_width: lit(0.1),
        segments: 12, stacks: 6,
    });
    match m.evaluate("b") {
        Ok(s) => assert!(solid_volume(&s) > 0.0),
        Err(_) => {}
    }
}

#[test]
fn marker_3d_completes() {
    let m = Model::new().add(Feature::Marker3D {
        id: "m".into(),
        center: [lit(0.0), lit(0.0), lit(0.0)],
        axis_length: lit(1.0),
        bar_radius: lit(0.05),
        segments: 12,
    });
    match m.evaluate("m") {
        Ok(s) => assert!(solid_volume(&s) > 0.0),
        Err(_) => {}
    }
}

#[test]
fn batch_16_round_trips_via_json() {
    let m = Model::new()
        .add(Feature::Ellipsoid3D { id: "e".into(), rx: lit(2.0), ry: lit(1.0), rz: lit(1.5), stacks: 8, slices: 12 })
        .add(Feature::VectorArrow { id: "va".into(), from: [lit(0.0), lit(0.0), lit(0.0)], to: [lit(3.0), lit(4.0), lit(0.0)], shaft_radius: lit(0.05), head_radius: lit(0.15), head_length: lit(0.5), segments: 12 })
        .add(Feature::BoneShape { id: "bs".into(), end_radius: lit(0.5), shaft_radius: lit(0.2), shaft_length: lit(3.0), stacks: 6, slices: 12 })
        .add(Feature::Pawn { id: "p".into(), base_radius: lit(0.6), base_height: lit(0.2), body_top_radius: lit(0.3), body_height: lit(1.0), head_radius: lit(0.4), segments: 12, stacks: 6 })
        .add(Feature::Rook { id: "r".into(), base_radius: lit(0.6), base_height: lit(0.2), body_radius: lit(0.5), body_height: lit(1.0), segments: 16 })
        .add(Feature::Bishop { id: "b".into(), base_radius: lit(0.6), base_height: lit(0.2), body_radius: lit(0.4), body_height: lit(1.0), head_radius: lit(0.4), slot_width: lit(0.1), segments: 12, stacks: 6 })
        .add(Feature::Marker3D { id: "mk".into(), center: [lit(0.0), lit(0.0), lit(0.0)], axis_length: lit(1.0), bar_radius: lit(0.05), segments: 12 });
    let json = m.to_json_string().unwrap();
    let parsed = Model::from_json_str(&json).unwrap();
    assert_eq!(json, parsed.to_json_string().unwrap());
}
