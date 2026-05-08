//! Reference geometry batch 3: MidPlaneRef, PerpRefPlane, OffsetRefPlane,
//! CoordinateAxes, OriginPoint.

use kerf_brep::solid_volume;
use kerf_cad::{lits, Feature, Model, Scalar};

fn lit(x: f64) -> Scalar { Scalar::lit(x) }

// -------------------------------------------------------------------------
// MidPlaneRef
// -------------------------------------------------------------------------

#[test]
fn midplane_ref_volume_matches_thin_box() {
    // Two points along x: (0,0,0) and (10,0,0). Midplane perp to x with
    // normal "x" is a thin box of t=0.05, ext=(8,6) in yz.
    let m = Model::new().add(Feature::MidPlaneRef {
        id: "out".into(),
        position_a: lits([0.0, 0.0, 0.0]),
        position_b: lits([10.0, 0.0, 0.0]),
        axis: "x".into(),
        extents: [lit(8.0), lit(6.0)],
        marker_thickness: lit(0.05),
    });
    let s = m.evaluate("out").unwrap();
    let v = solid_volume(&s);
    let exp = 0.05 * 8.0 * 6.0;
    assert!((v - exp).abs() < 1e-9, "v={v}, exp={exp}");
}

#[test]
fn midplane_ref_round_trip_via_json() {
    let m = Model::new().add(Feature::MidPlaneRef {
        id: "out".into(),
        position_a: lits([1.0, 2.0, 3.0]),
        position_b: lits([1.0, 2.0, 9.0]),
        axis: "z".into(),
        extents: [lit(4.0), lit(5.0)],
        marker_thickness: lit(0.1),
    });
    let json = m.to_json_string().unwrap();
    let m2 = Model::from_json_str(&json).unwrap();
    let v1 = solid_volume(&m.evaluate("out").unwrap());
    let v2 = solid_volume(&m2.evaluate("out").unwrap());
    assert!((v1 - v2).abs() < 1e-9);
}

// -------------------------------------------------------------------------
// PerpRefPlane
// -------------------------------------------------------------------------

#[test]
fn perp_ref_plane_volume_matches_thin_box() {
    let m = Model::new().add(Feature::PerpRefPlane {
        id: "out".into(),
        axis: "z".into(),
        point: lits([0.0, 0.0, 5.0]),
        extents: [lit(3.0), lit(4.0)],
        marker_thickness: lit(0.02),
    });
    let s = m.evaluate("out").unwrap();
    let v = solid_volume(&s);
    let exp = 0.02 * 3.0 * 4.0;
    assert!((v - exp).abs() < 1e-9, "v={v}, exp={exp}");
}

#[test]
fn perp_ref_plane_round_trip_via_json() {
    let m = Model::new().add(Feature::PerpRefPlane {
        id: "out".into(),
        axis: "y".into(),
        point: lits([1.0, 2.0, 3.0]),
        extents: [lit(2.5), lit(2.5)],
        marker_thickness: lit(0.05),
    });
    let json = m.to_json_string().unwrap();
    let m2 = Model::from_json_str(&json).unwrap();
    let v1 = solid_volume(&m.evaluate("out").unwrap());
    let v2 = solid_volume(&m2.evaluate("out").unwrap());
    assert!((v1 - v2).abs() < 1e-9);
}

// -------------------------------------------------------------------------
// OffsetRefPlane
// -------------------------------------------------------------------------

#[test]
fn offset_ref_plane_volume_matches_thin_box() {
    let m = Model::new().add(Feature::OffsetRefPlane {
        id: "out".into(),
        base_position: lits([0.0, 0.0, 0.0]),
        axis: "z".into(),
        offset: lit(2.0),
        extents: [lit(5.0), lit(5.0)],
        marker_thickness: lit(0.04),
    });
    let s = m.evaluate("out").unwrap();
    let v = solid_volume(&s);
    let exp = 0.04 * 5.0 * 5.0;
    assert!((v - exp).abs() < 1e-9, "v={v}, exp={exp}");
}

#[test]
fn offset_ref_plane_negative_offset_works() {
    // Offset of -3 along x: marker should sit at x=-3 (centered).
    let m = Model::new().add(Feature::OffsetRefPlane {
        id: "out".into(),
        base_position: lits([0.0, 0.0, 0.0]),
        axis: "x".into(),
        offset: lit(-3.0),
        extents: [lit(2.0), lit(2.0)],
        marker_thickness: lit(0.1),
    });
    let s = m.evaluate("out").unwrap();
    let v = solid_volume(&s);
    let exp = 0.1 * 2.0 * 2.0;
    assert!((v - exp).abs() < 1e-9, "v={v}, exp={exp}");
}

#[test]
fn offset_ref_plane_round_trip_via_json() {
    let m = Model::new().add(Feature::OffsetRefPlane {
        id: "out".into(),
        base_position: lits([0.0, 0.0, 0.0]),
        axis: "z".into(),
        offset: lit(1.5),
        extents: [lit(3.0), lit(3.0)],
        marker_thickness: lit(0.05),
    });
    let json = m.to_json_string().unwrap();
    let m2 = Model::from_json_str(&json).unwrap();
    let v1 = solid_volume(&m.evaluate("out").unwrap());
    let v2 = solid_volume(&m2.evaluate("out").unwrap());
    assert!((v1 - v2).abs() < 1e-9);
}

// -------------------------------------------------------------------------
// CoordinateAxes
// -------------------------------------------------------------------------

#[test]
fn coordinate_axes_has_positive_volume() {
    let m = Model::new().add(Feature::CoordinateAxes {
        id: "ax".into(),
        length: lit(5.0),
        bar_radius: lit(0.05),
        head_length: lit(0.3),
        segments: 8,
    });
    let s = m.evaluate("ax").unwrap();
    let v = solid_volume(&s);
    // 3 bars, each ~10 long (-l to +l), radius 0.05.
    // Each bar: ~π*0.0025*10 = ~0.0785; three bars = ~0.236
    // less small overlap volume at origin.
    assert!(v > 0.1 && v < 0.5, "v={v}");
}

#[test]
fn coordinate_axes_round_trip_via_json() {
    let m = Model::new().add(Feature::CoordinateAxes {
        id: "ax".into(),
        length: lit(2.0),
        bar_radius: lit(0.02),
        head_length: lit(0.2),
        segments: 8,
    });
    let json = m.to_json_string().unwrap();
    let m2 = Model::from_json_str(&json).unwrap();
    let v1 = solid_volume(&m.evaluate("ax").unwrap());
    let v2 = solid_volume(&m2.evaluate("ax").unwrap());
    assert!((v1 - v2).abs() < 1e-9, "v1={v1}, v2={v2}");
}

// -------------------------------------------------------------------------
// OriginPoint
// -------------------------------------------------------------------------

#[test]
fn origin_point_renders_as_small_sphere() {
    let m = Model::new().add(Feature::OriginPoint {
        id: "op".into(),
        marker_radius: lit(0.1),
    });
    let s = m.evaluate("op").unwrap();
    let v = solid_volume(&s);
    // (4/3)π*0.001 ≈ 0.00419, faceted will be a bit less.
    assert!(v > 0.0 && v < 0.01, "v={v}");
}

#[test]
fn origin_point_round_trip_via_json() {
    let m = Model::new().add(Feature::OriginPoint {
        id: "op".into(),
        marker_radius: lit(0.2),
    });
    let json = m.to_json_string().unwrap();
    let m2 = Model::from_json_str(&json).unwrap();
    let v1 = solid_volume(&m.evaluate("op").unwrap());
    let v2 = solid_volume(&m2.evaluate("op").unwrap());
    assert!((v1 - v2).abs() < 1e-9, "v1={v1}, v2={v2}");
}

// -------------------------------------------------------------------------
// Validation rejection
// -------------------------------------------------------------------------

#[test]
fn rejects_invalid_inputs() {
    // Zero thickness rejected.
    let m = Model::new().add(Feature::PerpRefPlane {
        id: "out".into(),
        axis: "z".into(),
        point: lits([0.0, 0.0, 0.0]),
        extents: [lit(1.0), lit(1.0)],
        marker_thickness: lit(0.0),
    });
    assert!(m.evaluate("out").is_err());

    // Negative marker rejected.
    let m = Model::new().add(Feature::OriginPoint {
        id: "op".into(),
        marker_radius: lit(-0.1),
    });
    assert!(m.evaluate("op").is_err());

    // Bad axis rejected.
    let m = Model::new().add(Feature::MidPlaneRef {
        id: "out".into(),
        position_a: lits([0.0, 0.0, 0.0]),
        position_b: lits([1.0, 0.0, 0.0]),
        axis: "w".into(),
        extents: [lit(1.0), lit(1.0)],
        marker_thickness: lit(0.05),
    });
    assert!(m.evaluate("out").is_err());

    // CoordinateAxes negative bar_radius rejected.
    let m = Model::new().add(Feature::CoordinateAxes {
        id: "ax".into(),
        length: lit(1.0),
        bar_radius: lit(-0.1),
        head_length: lit(0.1),
        segments: 6,
    });
    assert!(m.evaluate("ax").is_err());
}
