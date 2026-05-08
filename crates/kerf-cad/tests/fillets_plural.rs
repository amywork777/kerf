//! Plural Fillets feature: multi-edge fillet via composite cutter union.

use kerf_brep::solid_volume;
use kerf_cad::{lits, Feature, FilletEdge, Model, Scalar};

#[test]
fn fillets_two_diagonally_opposite_corners_match_chained() {
    // Two diagonally-opposite z-edges; chained Fillet works for these too.
    // Verify Fillets gives the same volume.
    let chained = Model::new()
        .add(Feature::Box {
            id: "body".into(),
            extents: lits([100.0, 60.0, 8.0]),
        })
        .add(Feature::Fillet {
            id: "f1".into(),
            input: "body".into(),
            axis: "z".into(),
            edge_min: lits([0.0, 0.0, 0.0]),
            edge_length: Scalar::lit(8.0),
            radius: Scalar::lit(5.0),
            quadrant: "pp".into(),
            segments: 16,
        })
        .add(Feature::Fillet {
            id: "out".into(),
            input: "f1".into(),
            axis: "z".into(),
            edge_min: lits([100.0, 60.0, 0.0]),
            edge_length: Scalar::lit(8.0),
            radius: Scalar::lit(5.0),
            quadrant: "nn".into(),
            segments: 16,
        });
    let plural = Model::new()
        .add(Feature::Box {
            id: "body".into(),
            extents: lits([100.0, 60.0, 8.0]),
        })
        .add(Feature::Fillets {
            id: "out".into(),
            input: "body".into(),
            edges: vec![
                FilletEdge {
                    axis: "z".into(),
                    edge_min: lits([0.0, 0.0, 0.0]),
                    edge_length: Scalar::lit(8.0),
                    radius: Scalar::lit(5.0),
                    quadrant: "pp".into(),
                    segments: 16,
                },
                FilletEdge {
                    axis: "z".into(),
                    edge_min: lits([100.0, 60.0, 0.0]),
                    edge_length: Scalar::lit(8.0),
                    radius: Scalar::lit(5.0),
                    quadrant: "nn".into(),
                    segments: 16,
                },
            ],
        });
    let v_chained = solid_volume(&chained.evaluate("out").unwrap());
    let v_plural = solid_volume(&plural.evaluate("out").unwrap());
    assert!(
        (v_chained - v_plural).abs() < 1e-6,
        "chained={v_chained}, plural={v_plural}"
    );
}

#[test]
fn fillets_all_four_z_corners_succeeds() {
    // The chained-Fillet approach FAILS here (adjacent fillets share faces).
    // Plural Fillets should handle it because the four wedges don't overlap.
    let m = Model::new()
        .add(Feature::Box {
            id: "body".into(),
            extents: lits([100.0, 60.0, 8.0]),
        })
        .add(Feature::Fillets {
            id: "out".into(),
            input: "body".into(),
            edges: vec![
                FilletEdge {
                    axis: "z".into(),
                    edge_min: lits([0.0, 0.0, 0.0]),
                    edge_length: Scalar::lit(8.0),
                    radius: Scalar::lit(5.0),
                    quadrant: "pp".into(),
                    segments: 16,
                },
                FilletEdge {
                    axis: "z".into(),
                    edge_min: lits([100.0, 0.0, 0.0]),
                    edge_length: Scalar::lit(8.0),
                    radius: Scalar::lit(5.0),
                    quadrant: "np".into(),
                    segments: 16,
                },
                FilletEdge {
                    axis: "z".into(),
                    edge_min: lits([0.0, 60.0, 0.0]),
                    edge_length: Scalar::lit(8.0),
                    radius: Scalar::lit(5.0),
                    quadrant: "pn".into(),
                    segments: 16,
                },
                FilletEdge {
                    axis: "z".into(),
                    edge_min: lits([100.0, 60.0, 0.0]),
                    edge_length: Scalar::lit(8.0),
                    radius: Scalar::lit(5.0),
                    quadrant: "nn".into(),
                    segments: 16,
                },
            ],
        });
    // Just succeed and produce a reasonable volume.
    let s = m.evaluate("out").unwrap();
    let v = solid_volume(&s);
    let removed_per_corner = (5.0 * 5.0 - 0.25 * 16.0 / 2.0 * 5.0 * 5.0 * (2.0 * std::f64::consts::PI / 16.0).sin()) * 8.0;
    let exp = 100.0 * 60.0 * 8.0 - 4.0 * removed_per_corner;
    let rel = (v - exp).abs() / exp;
    assert!(rel < 0.005, "v={v}, exp~={exp}, rel={rel}");
}

#[test]
fn fillets_round_trips_via_json() {
    let m = Model::new()
        .add(Feature::Box {
            id: "body".into(),
            extents: lits([10.0, 10.0, 4.0]),
        })
        .add(Feature::Fillets {
            id: "out".into(),
            input: "body".into(),
            edges: vec![FilletEdge {
                axis: "z".into(),
                edge_min: lits([0.0, 0.0, 0.0]),
                edge_length: Scalar::lit(4.0),
                radius: Scalar::lit(1.0),
                quadrant: "pp".into(),
                segments: 12,
            }],
        });
    let json = m.to_json_string().unwrap();
    let m2 = Model::from_json_str(&json).unwrap();
    let v1 = solid_volume(&m.evaluate("out").unwrap());
    let v2 = solid_volume(&m2.evaluate("out").unwrap());
    assert!((v1 - v2).abs() < 1e-9);
}

#[test]
fn fillets_rejects_empty_edges() {
    let m = Model::new()
        .add(Feature::Box {
            id: "body".into(),
            extents: lits([4.0, 4.0, 4.0]),
        })
        .add(Feature::Fillets {
            id: "out".into(),
            input: "body".into(),
            edges: vec![],
        });
    assert!(m.evaluate("out").is_err());
}
