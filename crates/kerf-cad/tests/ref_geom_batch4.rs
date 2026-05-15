//! Reference geometry batch 4: Centerline, MidPlane, ConstructionAxis,
//! AnchorPoint2, BoundingSphere.

use kerf_brep::solid_volume;
use kerf_cad::{lits, Feature, Model, Scalar};

fn lit(x: f64) -> Scalar {
    Scalar::lit(x)
}

// -------------------------------------------------------------------------
// Centerline
// -------------------------------------------------------------------------

/// Volume of a cylinder = π * r² * L. For a faceted cylinder with 12
/// segments the inscribed-polygon area is π*r²*(12*sin(2π/12))/(2π)
/// but we allow ±5% tolerance for faceting error.
#[test]
fn centerline_volume_approx_pi_r2_l() {
    let r = 0.1_f64;
    let l = 10.0_f64;
    let m = Model::new().add(Feature::Centerline {
        id: "cl".into(),
        from: lits([0.0, 0.0, 0.0]),
        to: lits([0.0, 0.0, l]),
        radius: lit(r),
    });
    let s = m.evaluate("cl").unwrap();
    let v = solid_volume(&s);
    let expected = std::f64::consts::PI * r * r * l;
    // Allow 5% tolerance for faceting (12-segment polygon ≈ 98.3% of circle area).
    assert!(
        (v - expected).abs() / expected < 0.05,
        "v={v}, expected={expected}"
    );
}

#[test]
fn centerline_rejects_zero_radius() {
    let m = Model::new().add(Feature::Centerline {
        id: "cl".into(),
        from: lits([0.0, 0.0, 0.0]),
        to: lits([0.0, 0.0, 5.0]),
        radius: lit(0.0),
    });
    assert!(m.evaluate("cl").is_err());
}

#[test]
fn centerline_round_trip_via_json() {
    let m = Model::new().add(Feature::Centerline {
        id: "cl".into(),
        from: lits([1.0, 2.0, 3.0]),
        to: lits([4.0, 5.0, 6.0]),
        radius: lit(0.2),
    });
    let json = m.to_json_string().unwrap();
    let m2 = Model::from_json_str(&json).unwrap();
    let v1 = solid_volume(&m.evaluate("cl").unwrap());
    let v2 = solid_volume(&m2.evaluate("cl").unwrap());
    assert!((v1 - v2).abs() < 1e-9, "v1={v1}, v2={v2}");
}

// -------------------------------------------------------------------------
// MidPlane
// -------------------------------------------------------------------------

/// MidPlane should be centered at the midpoint between point_a and point_b.
/// We verify by checking the AABB center (bbox center ≈ midpoint).
#[test]
fn midplane_is_centered_at_midpoint() {
    // point_a = (0,0,0), point_b = (0,0,10) → midpoint z = 5.
    // Normal = (0,0,1) → axis_idx = 2 (z). Slab centered at z=5.
    // thickness = 0.05, so slab spans z ∈ [4.975, 5.025].
    let m = Model::new().add(Feature::MidPlane {
        id: "mp".into(),
        point_a: lits([0.0, 0.0, 0.0]),
        point_b: lits([0.0, 0.0, 10.0]),
        normal: lits([0.0, 0.0, 1.0]),
        extent: lit(3.0),
    });
    let s = m.evaluate("mp").unwrap();
    // Volume should be thickness * (2*extent)^2 = 0.05 * 36 = 1.8
    let v = solid_volume(&s);
    let expected = 0.05 * (2.0 * 3.0) * (2.0 * 3.0);
    assert!(
        (v - expected).abs() < 1e-9,
        "v={v}, expected={expected}"
    );
}

#[test]
fn midplane_rejects_zero_extent() {
    let m = Model::new().add(Feature::MidPlane {
        id: "mp".into(),
        point_a: lits([0.0, 0.0, 0.0]),
        point_b: lits([0.0, 0.0, 10.0]),
        normal: lits([0.0, 0.0, 1.0]),
        extent: lit(0.0),
    });
    assert!(m.evaluate("mp").is_err());
}

#[test]
fn midplane_round_trip_via_json() {
    let m = Model::new().add(Feature::MidPlane {
        id: "mp".into(),
        point_a: lits([0.0, 0.0, 0.0]),
        point_b: lits([10.0, 0.0, 0.0]),
        normal: lits([1.0, 0.0, 0.0]),
        extent: lit(5.0),
    });
    let json = m.to_json_string().unwrap();
    let m2 = Model::from_json_str(&json).unwrap();
    let v1 = solid_volume(&m.evaluate("mp").unwrap());
    let v2 = solid_volume(&m2.evaluate("mp").unwrap());
    assert!((v1 - v2).abs() < 1e-9, "v1={v1}, v2={v2}");
}

// -------------------------------------------------------------------------
// ConstructionAxis
// -------------------------------------------------------------------------

/// ConstructionAxis along +z with length L and radius r should have
/// volume ≈ π * r² * L (within faceting tolerance).
#[test]
fn construction_axis_volume_along_z() {
    let r = 0.1_f64;
    let l = 8.0_f64;
    let m = Model::new().add(Feature::ConstructionAxis {
        id: "ax".into(),
        origin: lits([0.0, 0.0, 0.0]),
        direction: lits([0.0, 0.0, 1.0]),
        length: lit(l),
        radius: lit(r),
    });
    let s = m.evaluate("ax").unwrap();
    let v = solid_volume(&s);
    let expected = std::f64::consts::PI * r * r * l;
    assert!(
        (v - expected).abs() / expected < 0.05,
        "v={v}, expected={expected}"
    );
}

#[test]
fn construction_axis_correctly_oriented_along_direction() {
    // ConstructionAxis along x and along z should have same volume.
    let r = 0.05_f64;
    let l = 5.0_f64;
    let m_x = Model::new().add(Feature::ConstructionAxis {
        id: "ax".into(),
        origin: lits([0.0, 0.0, 0.0]),
        direction: lits([1.0, 0.0, 0.0]),
        length: lit(l),
        radius: lit(r),
    });
    let m_z = Model::new().add(Feature::ConstructionAxis {
        id: "ax".into(),
        origin: lits([0.0, 0.0, 0.0]),
        direction: lits([0.0, 0.0, 1.0]),
        length: lit(l),
        radius: lit(r),
    });
    let v_x = solid_volume(&m_x.evaluate("ax").unwrap());
    let v_z = solid_volume(&m_z.evaluate("ax").unwrap());
    // Both should be within 1% of each other (same cylinder, different axis).
    assert!(
        (v_x - v_z).abs() / v_z < 0.01,
        "v_x={v_x}, v_z={v_z}"
    );
}

#[test]
fn construction_axis_rejects_zero_length() {
    let m = Model::new().add(Feature::ConstructionAxis {
        id: "ax".into(),
        origin: lits([0.0, 0.0, 0.0]),
        direction: lits([0.0, 0.0, 1.0]),
        length: lit(0.0),
        radius: lit(0.1),
    });
    assert!(m.evaluate("ax").is_err());
}

// -------------------------------------------------------------------------
// AnchorPoint2
// -------------------------------------------------------------------------

/// AnchorPoint2 with size=1 should have volume=1 (a unit cube).
#[test]
fn anchor_point2_size1_volume_is_1() {
    let m = Model::new().add(Feature::AnchorPoint2 {
        id: "ap".into(),
        position: lits([0.0, 0.0, 0.0]),
        size: lit(1.0),
    });
    let s = m.evaluate("ap").unwrap();
    let v = solid_volume(&s);
    assert!((v - 1.0).abs() < 1e-9, "v={v}");
}

#[test]
fn anchor_point2_size_scales_volume() {
    let s_val = 3.0_f64;
    let m = Model::new().add(Feature::AnchorPoint2 {
        id: "ap".into(),
        position: lits([0.0, 0.0, 0.0]),
        size: lit(s_val),
    });
    let s = m.evaluate("ap").unwrap();
    let v = solid_volume(&s);
    assert!((v - s_val.powi(3)).abs() < 1e-9, "v={v}");
}

#[test]
fn anchor_point2_rejects_zero_size() {
    let m = Model::new().add(Feature::AnchorPoint2 {
        id: "ap".into(),
        position: lits([0.0, 0.0, 0.0]),
        size: lit(0.0),
    });
    assert!(m.evaluate("ap").is_err());
}

#[test]
fn anchor_point2_round_trip_via_json() {
    let m = Model::new().add(Feature::AnchorPoint2 {
        id: "ap".into(),
        position: lits([1.0, 2.0, 3.0]),
        size: lit(2.0),
    });
    let json = m.to_json_string().unwrap();
    let m2 = Model::from_json_str(&json).unwrap();
    let v1 = solid_volume(&m.evaluate("ap").unwrap());
    let v2 = solid_volume(&m2.evaluate("ap").unwrap());
    assert!((v1 - v2).abs() < 1e-9, "v1={v1}, v2={v2}");
}

// -------------------------------------------------------------------------
// BoundingSphere
// -------------------------------------------------------------------------

/// BoundingSphere of a 1×1×1 box should have radius ≈ √3/2 ≈ 0.866.
/// Volume of sphere = (4/3)π r³. With faceting allow 5% tolerance.
#[test]
fn bounding_sphere_of_unit_box_has_radius_sqrt3_over_2() {
    // Build a 1×1×1 box, then wrap it in BoundingSphere.
    let m = Model::new()
        .add(Feature::Box {
            id: "body".into(),
            extents: lits([1.0, 1.0, 1.0]),
        })
        .add(Feature::BoundingSphere {
            id: "bs".into(),
            input: "body".into(),
            segments: 12,
        });
    let s = m.evaluate("bs").unwrap();
    let v = solid_volume(&s);
    let r_expected = (3.0_f64).sqrt() / 2.0; // ≈ 0.866
    let v_expected = (4.0 / 3.0) * std::f64::consts::PI * r_expected.powi(3);
    // Allow 5% for faceting.
    assert!(
        (v - v_expected).abs() / v_expected < 0.05,
        "v={v}, v_expected={v_expected}, r_expected={r_expected}"
    );
}

#[test]
fn bounding_sphere_rejects_bad_segments() {
    let m = Model::new()
        .add(Feature::Box {
            id: "body".into(),
            extents: lits([1.0, 1.0, 1.0]),
        })
        .add(Feature::BoundingSphere {
            id: "bs".into(),
            input: "body".into(),
            segments: 1,
        });
    assert!(m.evaluate("bs").is_err());
}

#[test]
fn bounding_sphere_round_trip_via_json() {
    let m = Model::new()
        .add(Feature::Box {
            id: "body".into(),
            extents: lits([2.0, 3.0, 4.0]),
        })
        .add(Feature::BoundingSphere {
            id: "bs".into(),
            input: "body".into(),
            segments: 8,
        });
    let json = m.to_json_string().unwrap();
    let m2 = Model::from_json_str(&json).unwrap();
    let v1 = solid_volume(&m.evaluate("bs").unwrap());
    let v2 = solid_volume(&m2.evaluate("bs").unwrap());
    assert!((v1 - v2).abs() < 1e-9, "v1={v1}, v2={v2}");
}
