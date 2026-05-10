//! Reference geometry batch 5: CenterMarker, AxisLabel, DistanceMarker.

use kerf_brep::solid_volume;
use kerf_cad::{lits, Feature, Model, Scalar};

fn lit(x: f64) -> Scalar {
    Scalar::lit(x)
}

// -------------------------------------------------------------------------
// CenterMarker
// -------------------------------------------------------------------------

#[test]
fn center_marker_has_three_rods_volume() {
    // Three thin rods of radius r and length sz, centred at origin.
    // Expected volume ≈ 3 × π × r² × sz (ignoring small overlaps).
    let sz = 10.0_f64;
    let r = 0.05_f64;
    let m = Model::new().add(Feature::CenterMarker {
        id: "cm".into(),
        position: lits([0.0, 0.0, 0.0]),
        size: lit(sz),
        rod_radius: lit(r),
        segments: 8,
    });
    let s = m.evaluate("cm").unwrap();
    let v = solid_volume(&s);
    let single_rod = std::f64::consts::PI * r * r * sz;
    // Allow ±20% for faceting and small stagger adjustments.
    let lo = single_rod * 3.0 * 0.80;
    let hi = single_rod * 3.0 * 1.20;
    assert!(v > lo && v < hi, "v={v}, expected ∈ ({lo}, {hi})");
}

#[test]
fn center_marker_at_offset_position() {
    // Marker at (5, 5, 5) should produce positive volume.
    let m = Model::new().add(Feature::CenterMarker {
        id: "cm".into(),
        position: lits([5.0, 5.0, 5.0]),
        size: lit(4.0),
        rod_radius: lit(0.05),
        segments: 8,
    });
    let v = solid_volume(&m.evaluate("cm").unwrap());
    assert!(v > 0.0, "v={v}");
}

#[test]
fn center_marker_round_trip_via_json() {
    let m = Model::new().add(Feature::CenterMarker {
        id: "cm".into(),
        position: lits([1.0, 2.0, 3.0]),
        size: lit(6.0),
        rod_radius: lit(0.03),
        segments: 6,
    });
    let json = m.to_json_string().unwrap();
    let m2 = Model::from_json_str(&json).unwrap();
    let v1 = solid_volume(&m.evaluate("cm").unwrap());
    let v2 = solid_volume(&m2.evaluate("cm").unwrap());
    assert!((v1 - v2).abs() < 1e-9, "v1={v1}, v2={v2}");
}

#[test]
fn center_marker_rejects_invalid() {
    // Zero size rejected.
    assert!(Model::new()
        .add(Feature::CenterMarker {
            id: "x".into(),
            position: lits([0.0, 0.0, 0.0]),
            size: lit(0.0),
            rod_radius: lit(0.05),
            segments: 6,
        })
        .evaluate("x")
        .is_err());
    // Negative rod_radius rejected.
    assert!(Model::new()
        .add(Feature::CenterMarker {
            id: "x".into(),
            position: lits([0.0, 0.0, 0.0]),
            size: lit(5.0),
            rod_radius: lit(-0.1),
            segments: 6,
        })
        .evaluate("x")
        .is_err());
    // Too-few segments rejected.
    assert!(Model::new()
        .add(Feature::CenterMarker {
            id: "x".into(),
            position: lits([0.0, 0.0, 0.0]),
            size: lit(5.0),
            rod_radius: lit(0.05),
            segments: 2,
        })
        .evaluate("x")
        .is_err());
}

// -------------------------------------------------------------------------
// AxisLabel
// -------------------------------------------------------------------------

#[test]
fn axis_label_shaft_plus_cone_positive_volume() {
    // Shaft along +z, length 5, then cone head. Total > 0.
    let m = Model::new().add(Feature::AxisLabel {
        id: "al".into(),
        origin: lits([0.0, 0.0, 0.0]),
        direction: lits([0.0, 0.0, 1.0]),
        length: lit(5.0),
        head_radius: lit(0.4),
        shaft_radius: lit(0.1),
        segments: 8,
    });
    let v = solid_volume(&m.evaluate("al").unwrap());
    // Shaft ≈ π*0.01*5 ≈ 0.157; head ≈ (1/3)*π*0.16*0.4 ≈ 0.067; sum > 0.2
    assert!(v > 0.1, "v={v}");
}

#[test]
fn axis_label_diagonal_direction() {
    // Direction not aligned to any axis: (1,1,0).
    let m = Model::new().add(Feature::AxisLabel {
        id: "al".into(),
        origin: lits([0.0, 0.0, 0.0]),
        direction: lits([1.0, 1.0, 0.0]),
        length: lit(4.0),
        head_radius: lit(0.3),
        shaft_radius: lit(0.08),
        segments: 8,
    });
    let v = solid_volume(&m.evaluate("al").unwrap());
    assert!(v > 0.0, "v={v}");
}

#[test]
fn axis_label_round_trip_via_json() {
    let m = Model::new().add(Feature::AxisLabel {
        id: "al".into(),
        origin: lits([1.0, 0.0, 0.0]),
        direction: lits([0.0, 1.0, 0.0]),
        length: lit(3.0),
        head_radius: lit(0.2),
        shaft_radius: lit(0.05),
        segments: 8,
    });
    let json = m.to_json_string().unwrap();
    let m2 = Model::from_json_str(&json).unwrap();
    let v1 = solid_volume(&m.evaluate("al").unwrap());
    let v2 = solid_volume(&m2.evaluate("al").unwrap());
    assert!((v1 - v2).abs() < 1e-9, "v1={v1}, v2={v2}");
}

#[test]
fn axis_label_rejects_invalid() {
    // head_radius <= shaft_radius rejected.
    assert!(Model::new()
        .add(Feature::AxisLabel {
            id: "x".into(),
            origin: lits([0.0, 0.0, 0.0]),
            direction: lits([1.0, 0.0, 0.0]),
            length: lit(5.0),
            head_radius: lit(0.1),
            shaft_radius: lit(0.1),
            segments: 8,
        })
        .evaluate("x")
        .is_err());
    // Zero direction rejected.
    assert!(Model::new()
        .add(Feature::AxisLabel {
            id: "x".into(),
            origin: lits([0.0, 0.0, 0.0]),
            direction: lits([0.0, 0.0, 0.0]),
            length: lit(5.0),
            head_radius: lit(0.3),
            shaft_radius: lit(0.1),
            segments: 8,
        })
        .evaluate("x")
        .is_err());
    // Too-few segments rejected.
    assert!(Model::new()
        .add(Feature::AxisLabel {
            id: "x".into(),
            origin: lits([0.0, 0.0, 0.0]),
            direction: lits([1.0, 0.0, 0.0]),
            length: lit(5.0),
            head_radius: lit(0.3),
            shaft_radius: lit(0.1),
            segments: 5,
        })
        .evaluate("x")
        .is_err());
}

// -------------------------------------------------------------------------
// DistanceMarker
// -------------------------------------------------------------------------

#[test]
fn distance_marker_shaft_plus_two_heads_volume() {
    // Shaft along x from (0,0,0) to (10,0,0), head_radius 0.4, shaft 0.05.
    // Shaft vol ≈ π*0.0025*10 ≈ 0.0785
    // Each head vol ≈ (1/3)*π*0.16*0.4 ≈ 0.067  (two heads ≈ 0.134)
    // Total > 0.1
    let m = Model::new().add(Feature::DistanceMarker {
        id: "dm".into(),
        from: lits([0.0, 0.0, 0.0]),
        to: lits([10.0, 0.0, 0.0]),
        shaft_radius: lit(0.05),
        head_radius: lit(0.4),
        segments: 8,
    });
    let v = solid_volume(&m.evaluate("dm").unwrap());
    assert!(v > 0.1, "v={v}");
}

#[test]
fn distance_marker_diagonal_segment() {
    // Non-axis-aligned direction: (1,1,1).
    let m = Model::new().add(Feature::DistanceMarker {
        id: "dm".into(),
        from: lits([0.0, 0.0, 0.0]),
        to: lits([5.0, 5.0, 5.0]),
        shaft_radius: lit(0.05),
        head_radius: lit(0.3),
        segments: 8,
    });
    let v = solid_volume(&m.evaluate("dm").unwrap());
    assert!(v > 0.0, "v={v}");
}

#[test]
fn distance_marker_round_trip_via_json() {
    let m = Model::new().add(Feature::DistanceMarker {
        id: "dm".into(),
        from: lits([0.0, 0.0, 0.0]),
        to: lits([0.0, 0.0, 8.0]),
        shaft_radius: lit(0.04),
        head_radius: lit(0.3),
        segments: 8,
    });
    let json = m.to_json_string().unwrap();
    let m2 = Model::from_json_str(&json).unwrap();
    let v1 = solid_volume(&m.evaluate("dm").unwrap());
    let v2 = solid_volume(&m2.evaluate("dm").unwrap());
    assert!((v1 - v2).abs() < 1e-9, "v1={v1}, v2={v2}");
}

#[test]
fn distance_marker_rejects_invalid() {
    // Coincident from/to rejected.
    assert!(Model::new()
        .add(Feature::DistanceMarker {
            id: "x".into(),
            from: lits([1.0, 1.0, 1.0]),
            to: lits([1.0, 1.0, 1.0]),
            shaft_radius: lit(0.05),
            head_radius: lit(0.3),
            segments: 8,
        })
        .evaluate("x")
        .is_err());
    // head_radius <= shaft_radius rejected.
    assert!(Model::new()
        .add(Feature::DistanceMarker {
            id: "x".into(),
            from: lits([0.0, 0.0, 0.0]),
            to: lits([5.0, 0.0, 0.0]),
            shaft_radius: lit(0.3),
            head_radius: lit(0.1),
            segments: 8,
        })
        .evaluate("x")
        .is_err());
    // Too-few segments rejected.
    assert!(Model::new()
        .add(Feature::DistanceMarker {
            id: "x".into(),
            from: lits([0.0, 0.0, 0.0]),
            to: lits([5.0, 0.0, 0.0]),
            shaft_radius: lit(0.05),
            head_radius: lit(0.3),
            segments: 2,
        })
        .evaluate("x")
        .is_err());
}
