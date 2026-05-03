//! Mirror feature: reflect a body across a plane.
//!
//! KNOWN LIMITATION: the current implementation reflects geometry but does
//! not reverse the half-edge loop walks in kerf-topo. The result is a
//! topologically-valid solid whose face polygons walk CW-from-outside (the
//! mirror image of the original). Visualisations / tessellation through
//! face_polygon (which winding-normalises via signed area) display
//! correctly, but `solid_volume` reads the raw loop and reports a negative
//! number whose magnitude equals the original volume. Boolean composition
//! with another solid is not supported until loop-reversal lands in
//! kerf-topo. The tests below assert topology integrity and round-trip,
//! plus the magnitude property of volume.

use kerf_brep::solid_volume;
use kerf_cad::{lits, Feature, Model, Scalar};

#[test]
fn mirror_produces_a_solid_of_the_same_size() {
    let m = Model::new()
        .add(Feature::BoxAt {
            id: "body".into(),
            extents: lits([2.0, 3.0, 4.0]),
            origin: lits([1.0, 0.0, 0.0]),
        })
        .add(Feature::Mirror {
            id: "out".into(),
            input: "body".into(),
            plane_origin: lits([0.0, 0.0, 0.0]),
            plane_normal: lits([1.0, 0.0, 0.0]),
        });
    let s = m.evaluate("out").unwrap();
    assert_eq!(s.vertex_count(), 8);
    assert_eq!(s.face_count(), 6);
    // Magnitude matches the original 2×3×4 = 24.
    assert!((solid_volume(&s).abs() - 24.0).abs() < 1e-9);
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
    let s1 = m.evaluate("out").unwrap();
    let s2 = m2.evaluate("out").unwrap();
    assert_eq!(s1.face_count(), s2.face_count());
    assert_eq!(s1.vertex_count(), s2.vertex_count());
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
