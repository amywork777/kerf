//! Fillet feature: round an axis-aligned 90° edge with a quarter-circle.

use kerf_brep::solid_volume;
use kerf_cad::{lits, Feature, Model, Scalar};

#[test]
fn fillet_box_z_edge_pp_volume_matches_analytic() {
    // 10 × 10 × 5 box. Round its z-axis edge at the origin (body in +x +y)
    // with radius 2. Volume removed = (r² - πr²/4) * height.
    // With faceted cylinder (16 segments), area_quarter = (1/4)*n/2*r²*sin(2π/n)
    // = 4 * sin(π/8) ≈ 3.0614. So removed = (4 - 3.0614) * 5 ≈ 4.693.
    let m = Model::new()
        .add(Feature::Box {
            id: "body".into(),
            extents: lits([10.0, 10.0, 5.0]),
        })
        .add(Feature::Fillet {
            id: "out".into(),
            input: "body".into(),
            axis: "z".into(),
            edge_min: lits([0.0, 0.0, 0.0]),
            edge_length: Scalar::lit(5.0),
            radius: Scalar::lit(2.0),
            quadrant: "pp".into(),
            segments: 16,
        });
    let s = m.evaluate("out").unwrap();
    let v = solid_volume(&s);
    let n = 16.0;
    let r = 2.0;
    let area_quarter = 0.25 * (n / 2.0) * r * r * (2.0 * std::f64::consts::PI / n).sin();
    let exp = 500.0 - (r * r - area_quarter) * 5.0;
    assert!(
        (v - exp).abs() < 0.05,
        "v={v}, exp={exp}, diff={}",
        v - exp
    );
}

#[test]
fn fillet_box_x_edge_volume_matches() {
    // 8 × 6 × 4 box, fillet x-edge at (0, 0, 0), length 8, radius 1.5.
    // Body in +y +z (quadrant pp).
    let m = Model::new()
        .add(Feature::Box {
            id: "body".into(),
            extents: lits([8.0, 6.0, 4.0]),
        })
        .add(Feature::Fillet {
            id: "out".into(),
            input: "body".into(),
            axis: "x".into(),
            edge_min: lits([0.0, 0.0, 0.0]),
            edge_length: Scalar::lit(8.0),
            radius: Scalar::lit(1.5),
            quadrant: "pp".into(),
            segments: 16,
        });
    let s = m.evaluate("out").unwrap();
    let v = solid_volume(&s);
    let n = 16.0;
    let r = 1.5;
    let area_quarter = 0.25 * (n / 2.0) * r * r * (2.0 * std::f64::consts::PI / n).sin();
    let body_v = 8.0 * 6.0 * 4.0;
    let exp = body_v - (r * r - area_quarter) * 8.0;
    assert!((v - exp).abs() < 0.05, "v={v}, exp={exp}");
}

#[test]
fn fillet_box_y_edge_pn_quadrant() {
    // y-axis edge at (10, 0, 0), edge_length 6, radius 1.0.
    // Box occupies x ∈ [0, 10], y ∈ [0, 6], z ∈ [0, 4].
    // Edge along y at x=10, z=0; body extends -x, +z from edge → quadrant
    // (z=p, x=n) in canonical (a=z, b=x) order = "pn".
    let m = Model::new()
        .add(Feature::Box {
            id: "body".into(),
            extents: lits([10.0, 6.0, 4.0]),
        })
        .add(Feature::Fillet {
            id: "out".into(),
            input: "body".into(),
            axis: "y".into(),
            edge_min: lits([10.0, 0.0, 0.0]),
            edge_length: Scalar::lit(6.0),
            radius: Scalar::lit(1.0),
            quadrant: "pn".into(),
            segments: 16,
        });
    let s = m.evaluate("out").unwrap();
    let v = solid_volume(&s);
    let n = 16.0;
    let r = 1.0;
    let area_quarter = 0.25 * (n / 2.0) * r * r * (2.0 * std::f64::consts::PI / n).sin();
    let exp = 240.0 - (r * r - area_quarter) * 6.0;
    assert!((v - exp).abs() < 0.05, "v={v}, exp={exp}");
}

#[test]
fn fillet_round_trips_via_json() {
    let m = Model::new()
        .add(Feature::Box {
            id: "body".into(),
            extents: lits([4.0, 4.0, 4.0]),
        })
        .add(Feature::Fillet {
            id: "out".into(),
            input: "body".into(),
            axis: "z".into(),
            edge_min: lits([0.0, 0.0, 0.0]),
            edge_length: Scalar::lit(4.0),
            radius: Scalar::lit(0.5),
            quadrant: "pp".into(),
            segments: 12,
        });
    let json = m.to_json_string().unwrap();
    let m2 = Model::from_json_str(&json).unwrap();
    let v1 = solid_volume(&m.evaluate("out").unwrap());
    let v2 = solid_volume(&m2.evaluate("out").unwrap());
    assert!((v1 - v2).abs() < 1e-9);
}

#[test]
fn fillet_rejects_non_positive_radius() {
    let m = Model::new()
        .add(Feature::Box {
            id: "body".into(),
            extents: lits([4.0, 4.0, 4.0]),
        })
        .add(Feature::Fillet {
            id: "out".into(),
            input: "body".into(),
            axis: "z".into(),
            edge_min: lits([0.0, 0.0, 0.0]),
            edge_length: Scalar::lit(4.0),
            radius: Scalar::lit(0.0),
            quadrant: "pp".into(),
            segments: 12,
        });
    assert!(m.evaluate("out").is_err());
}

#[test]
fn fillet_rejects_bad_axis() {
    let m = Model::new()
        .add(Feature::Box {
            id: "body".into(),
            extents: lits([4.0, 4.0, 4.0]),
        })
        .add(Feature::Fillet {
            id: "out".into(),
            input: "body".into(),
            axis: "w".into(),
            edge_min: lits([0.0, 0.0, 0.0]),
            edge_length: Scalar::lit(4.0),
            radius: Scalar::lit(0.5),
            quadrant: "pp".into(),
            segments: 12,
        });
    assert!(m.evaluate("out").is_err());
}

#[test]
fn fillet_rejects_bad_quadrant() {
    let m = Model::new()
        .add(Feature::Box {
            id: "body".into(),
            extents: lits([4.0, 4.0, 4.0]),
        })
        .add(Feature::Fillet {
            id: "out".into(),
            input: "body".into(),
            axis: "z".into(),
            edge_min: lits([0.0, 0.0, 0.0]),
            edge_length: Scalar::lit(4.0),
            radius: Scalar::lit(0.5),
            quadrant: "xx".into(),
            segments: 12,
        });
    assert!(m.evaluate("out").is_err());
}
