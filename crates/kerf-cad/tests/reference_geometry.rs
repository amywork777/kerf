//! Reference geometry primitives: RefPoint, RefAxis, RefPlane.

use kerf_brep::solid_volume;
use kerf_cad::{lits, Feature, Model, Scalar};

#[test]
fn ref_point_renders_as_small_sphere() {
    let m = Model::new().add(Feature::RefPoint {
        id: "out".into(),
        position: lits([5.0, 3.0, 2.0]),
        marker_radius: Scalar::lit(0.1),
    });
    let s = m.evaluate("out").unwrap();
    let v = solid_volume(&s);
    // (4/3)*π*r³ ≈ 0.00419, faceted will be a bit less.
    assert!(v > 0.0 && v < 0.01, "v={v}");
}

#[test]
fn ref_axis_along_z_volume_matches_thin_cylinder() {
    let m = Model::new().add(Feature::RefAxis {
        id: "out".into(),
        position: lits([0.0, 0.0, 5.0]),
        axis: "z".into(),
        length: Scalar::lit(10.0),
        marker_radius: Scalar::lit(0.05),
    });
    let s = m.evaluate("out").unwrap();
    let v = solid_volume(&s);
    let exp = std::f64::consts::PI * 0.0025 * 10.0;
    let rel = (v - exp).abs() / exp;
    assert!(rel < 0.10, "v={v}, exp={exp}, rel={rel}");
}

#[test]
fn ref_plane_perpendicular_to_x_is_thin_box_in_yz() {
    let m = Model::new().add(Feature::RefPlane {
        id: "out".into(),
        position: lits([5.0, 5.0, 5.0]),
        axis: "x".into(),
        extents: [Scalar::lit(8.0), Scalar::lit(6.0)],
        marker_thickness: Scalar::lit(0.05),
    });
    let s = m.evaluate("out").unwrap();
    let v = solid_volume(&s);
    let exp = 0.05 * 8.0 * 6.0;
    assert!((v - exp).abs() < 1e-9, "v={v}, exp={exp}");
}

#[test]
fn ref_geometry_round_trips_via_json() {
    for f in [
        Feature::RefPoint {
            id: "out".into(),
            position: lits([1.0, 2.0, 3.0]),
            marker_radius: Scalar::lit(0.1),
        },
        Feature::RefAxis {
            id: "out".into(),
            position: lits([0.0, 0.0, 0.0]),
            axis: "z".into(),
            length: Scalar::lit(10.0),
            marker_radius: Scalar::lit(0.05),
        },
        Feature::RefPlane {
            id: "out".into(),
            position: lits([0.0, 0.0, 0.0]),
            axis: "z".into(),
            extents: [Scalar::lit(5.0), Scalar::lit(5.0)],
            marker_thickness: Scalar::lit(0.05),
        },
    ] {
        let m = Model::new().add(f);
        let json = m.to_json_string().unwrap();
        let m2 = Model::from_json_str(&json).unwrap();
        let v1 = solid_volume(&m.evaluate("out").unwrap());
        let v2 = solid_volume(&m2.evaluate("out").unwrap());
        assert!((v1 - v2).abs() < 1e-9);
    }
}
