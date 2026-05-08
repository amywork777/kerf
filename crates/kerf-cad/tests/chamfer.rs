//! Chamfer feature: bevel an axis-aligned 90° edge.

use kerf_brep::solid_volume;
use kerf_cad::{lits, Feature, Model, Scalar};

#[test]
fn chamfer_box_z_edge_pp_volume_matches() {
    // 10 × 10 × 5 box. Bevel the z-edge at origin (body in +x +y) with
    // 45° flat cut, setback 1.0 on each face.
    // Volume removed = (1/2 * setback²) * edge_length = 0.5 * 1 * 5 = 2.5.
    let m = Model::new()
        .add(Feature::Box {
            id: "body".into(),
            extents: lits([10.0, 10.0, 5.0]),
        })
        .add(Feature::Chamfer {
            id: "out".into(),
            input: "body".into(),
            axis: "z".into(),
            edge_min: lits([0.0, 0.0, 0.0]),
            edge_length: Scalar::lit(5.0),
            setback: Scalar::lit(1.0),
            quadrant: "pp".into(),
        });
    let s = m.evaluate("out").unwrap();
    let v = solid_volume(&s);
    let exp = 500.0 - 0.5 * 1.0 * 1.0 * 5.0;
    assert!((v - exp).abs() < 1e-6, "v={v}, exp={exp}");
}

#[test]
fn chamfer_box_x_edge_volume_matches() {
    let m = Model::new()
        .add(Feature::Box {
            id: "body".into(),
            extents: lits([8.0, 6.0, 4.0]),
        })
        .add(Feature::Chamfer {
            id: "out".into(),
            input: "body".into(),
            axis: "x".into(),
            edge_min: lits([0.0, 0.0, 0.0]),
            edge_length: Scalar::lit(8.0),
            setback: Scalar::lit(1.5),
            quadrant: "pp".into(),
        });
    let s = m.evaluate("out").unwrap();
    let v = solid_volume(&s);
    let exp = 8.0 * 6.0 * 4.0 - 0.5 * 1.5 * 1.5 * 8.0;
    assert!((v - exp).abs() < 1e-6, "v={v}, exp={exp}");
}

#[test]
fn chamfer_round_trips_via_json() {
    let m = Model::new()
        .add(Feature::Box {
            id: "body".into(),
            extents: lits([4.0, 4.0, 4.0]),
        })
        .add(Feature::Chamfer {
            id: "out".into(),
            input: "body".into(),
            axis: "y".into(),
            edge_min: lits([4.0, 0.0, 0.0]),
            edge_length: Scalar::lit(4.0),
            setback: Scalar::lit(0.5),
            quadrant: "pn".into(),
        });
    let json = m.to_json_string().unwrap();
    let m2 = Model::from_json_str(&json).unwrap();
    let v1 = solid_volume(&m.evaluate("out").unwrap());
    let v2 = solid_volume(&m2.evaluate("out").unwrap());
    assert!((v1 - v2).abs() < 1e-9);
}

#[test]
fn chamfer_rejects_non_positive_setback() {
    let m = Model::new()
        .add(Feature::Box {
            id: "body".into(),
            extents: lits([4.0, 4.0, 4.0]),
        })
        .add(Feature::Chamfer {
            id: "out".into(),
            input: "body".into(),
            axis: "z".into(),
            edge_min: lits([0.0, 0.0, 0.0]),
            edge_length: Scalar::lit(4.0),
            setback: Scalar::lit(-0.5),
            quadrant: "pp".into(),
        });
    assert!(m.evaluate("out").is_err());
}
