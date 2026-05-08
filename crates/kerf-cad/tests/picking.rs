//! GAP 1: face_owner_tag picking provenance.
//!
//! When a model is `Difference("op", "body", "drill")` the resulting solid's
//! cylindrical hole face must trace back to "drill" and the flat exterior
//! faces to "body". This is the foundation for "click a face → schedule a
//! fillet on that feature's edges" in the viewer.

use kerf_brep::geometry::SurfaceKind;
use kerf_cad::{Feature, Model, Scalar};

fn box_minus_cylinder_model() -> Model {
    Model::new()
        // Big plate.
        .add(Feature::Box {
            id: "body".into(),
            extents: [Scalar::lit(10.0), Scalar::lit(10.0), Scalar::lit(2.0)],
        })
        // Through hole — cylinder taller than the plate.
        .add(Feature::Cylinder {
            id: "drill".into(),
            radius: Scalar::lit(1.0),
            height: Scalar::lit(4.0),
            segments: 24,
        })
        .add(Feature::Difference {
            id: "op".into(),
            inputs: vec!["body".into(), "drill".into()],
        })
}

#[test]
fn primitive_features_tag_every_face_with_their_id() {
    let m = Model::new().add(Feature::Box {
        id: "body".into(),
        extents: [Scalar::lit(2.0), Scalar::lit(2.0), Scalar::lit(2.0)],
    });
    let s = m.evaluate("body").unwrap();
    let face_ids: Vec<_> = s.topo.face_ids().collect();
    assert_eq!(face_ids.len(), 6, "box has 6 faces");
    for fid in face_ids {
        let tag = s.face_owner_tag.get(fid);
        assert_eq!(tag.map(String::as_str), Some("body"));
    }
}

#[test]
fn difference_propagates_owner_tags_from_inputs() {
    let m = box_minus_cylinder_model();
    let s = m.evaluate("op").unwrap();
    // Every face of the result should carry an owner tag.
    let face_ids: Vec<_> = s.topo.face_ids().collect();
    assert!(!face_ids.is_empty(), "result solid must have faces");
    let mut body_count = 0;
    let mut drill_count = 0;
    let mut other = Vec::new();
    for fid in face_ids {
        let tag = s
            .face_owner_tag
            .get(fid)
            .cloned()
            .unwrap_or_else(|| "<UNTAGGED>".into());
        match tag.as_str() {
            "body" => body_count += 1,
            "drill" => drill_count += 1,
            _ => other.push(tag),
        }
    }
    assert!(other.is_empty(), "unexpected tags: {other:?}");
    assert!(body_count > 0, "no body-tagged faces");
    assert!(drill_count > 0, "no drill-tagged faces");
}

#[test]
fn drill_contributes_at_least_one_face_to_hole_wall() {
    // After through-drilling, the hole-wall faces (whatever the kernel +
    // chord-merge finally retain — the actual count varies with how
    // siblings merge) must trace back to "drill", not "body". This is the
    // load-bearing invariant for picking: clicking ANY hole-wall face
    // must surface "drill" as the owner.
    let m = box_minus_cylinder_model();
    let s = m.evaluate("op").unwrap();
    let mut drill_faces = 0;
    let mut body_faces = 0;
    for fid in s.topo.face_ids() {
        match s.face_owner_tag.get(fid).map(String::as_str) {
            Some("drill") => drill_faces += 1,
            Some("body") => body_faces += 1,
            _ => {}
        }
    }
    assert!(drill_faces >= 1, "no drill-tagged faces in result");
    assert!(body_faces >= 1, "no body-tagged faces in result");
    // The plate has 6 base faces; drill contributes faces beyond that.
    let total: usize = s.topo.face_ids().count();
    assert!(total > 6, "result has only {total} faces — drill didn't add any?");
}

#[test]
fn flat_exterior_face_traces_back_to_body() {
    let m = box_minus_cylinder_model();
    let s = m.evaluate("op").unwrap();
    // At least one Plane face must be tagged "body" (the side faces of the
    // plate are not touched by the drill at all, so they survive verbatim).
    let mut body_planes = 0;
    for fid in s.topo.face_ids() {
        let Some(surface) = s.face_geom.get(fid) else { continue };
        if matches!(surface, SurfaceKind::Plane(_)) {
            if s.face_owner_tag.get(fid).map(String::as_str) == Some("body") {
                body_planes += 1;
            }
        }
    }
    assert!(body_planes >= 4, "expected at least 4 'body' plane faces (the 4 sides), got {body_planes}");
}

#[test]
fn boolean_feature_does_not_overwrite_input_tags() {
    // The Difference feature itself has id "op", but it should NOT relabel
    // every face as "op" — input ownership must survive. This guards
    // against a regression where we naively tag all faces with the feature
    // id without checking face_owner_tag first.
    let m = box_minus_cylinder_model();
    let s = m.evaluate("op").unwrap();
    let any_op = s
        .topo
        .face_ids()
        .any(|fid| s.face_owner_tag.get(fid).map(String::as_str) == Some("op"));
    assert!(!any_op, "Difference feature must not overwrite input owner tags");
}
