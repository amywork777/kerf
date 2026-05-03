//! Mirror feature: reflect a body across a plane and produce a valid solid.

use kerf_brep::solid_volume;
use kerf_cad::{lits, Feature, Model, Scalar};

#[test]
fn mirror_a_box_across_yz_plane_preserves_volume() {
    let m = Model::new()
        .add(Feature::BoxAt {
            id: "body".into(),
            extents: lits([2.0, 3.0, 4.0]),
            origin: lits([1.0, 0.0, 0.0]),  // body in x = [1, 3]
        })
        .add(Feature::Mirror {
            id: "out".into(),
            input: "body".into(),
            plane_origin: lits([0.0, 0.0, 0.0]),
            plane_normal: lits([1.0, 0.0, 0.0]),
        });
    let s = m.evaluate("out").unwrap();
    let v = solid_volume(&s);
    assert!((v - 24.0).abs() < 1e-9, "v={v}");
}

#[test]
fn mirror_round_trips_via_json() {
    let m = Model::new()
        .add(Feature::Box { id: "body".into(), extents: lits([1.0, 1.0, 1.0]) })
        .add(Feature::Mirror {
            id: "out".into(),
            input: "body".into(),
            plane_origin: lits([0.5, 0.0, 0.0]),
            plane_normal: lits([1.0, 0.0, 0.0]),
        });
    let json = m.to_json_string().unwrap();
    let m2 = Model::from_json_str(&json).unwrap();
    let v1 = solid_volume(&m.evaluate("out").unwrap());
    let v2 = solid_volume(&m2.evaluate("out").unwrap());
    assert!((v1 - v2).abs() < 1e-9);
}

#[test]
fn mirror_rejects_zero_plane_normal() {
    let m = Model::new()
        .add(Feature::Box { id: "body".into(), extents: lits([1.0, 1.0, 1.0]) })
        .add(Feature::Mirror {
            id: "out".into(),
            input: "body".into(),
            plane_origin: lits([0.0, 0.0, 0.0]),
            plane_normal: lits([0.0, 0.0, 0.0]),
        });
    assert!(m.evaluate("out").is_err());
}

#[test]
fn mirror_then_volume_is_correct_for_arbitrary_plane() {
    let m = Model::new()
        .add(Feature::Box { id: "body".into(), extents: lits([2.0, 2.0, 2.0]) })
        .add(Feature::Mirror {
            id: "out".into(),
            input: "body".into(),
            plane_origin: lits([10.0, 0.0, 0.0]),
            plane_normal: lits([1.0, 1.0, 0.0]),
        });
    let v = solid_volume(&m.evaluate("out").unwrap());
    assert!((v - 8.0).abs() < 1e-9, "v={v}");
}

#[test]
fn mirrored_body_unions_cleanly_with_original_for_symmetric_design() {
    // Place a half-body in +x, mirror across yz plane, union the two for a
    // symmetric body straddling x=0. Volume = 2 × (2 × 2 × 2) = 16.
    let m = Model::new()
        .add(Feature::BoxAt {
            id: "right".into(),
            extents: lits([2.0, 2.0, 2.0]),
            origin: lits([0.5, -1.0, -1.0]),  // x ∈ [0.5, 2.5]
        })
        .add(Feature::Mirror {
            id: "left".into(),
            input: "right".into(),
            plane_origin: lits([0.0, 0.0, 0.0]),
            plane_normal: lits([1.0, 0.0, 0.0]),
        })
        .add(Feature::Union {
            id: "out".into(),
            inputs: vec!["right".into(), "left".into()],
        });
    let s = m.evaluate("out").unwrap();
    let v = solid_volume(&s);
    assert!((v - 16.0).abs() < 1e-9, "v={v}");
    assert_eq!(s.shell_count(), 2, "two disjoint mirrored boxes");
}
