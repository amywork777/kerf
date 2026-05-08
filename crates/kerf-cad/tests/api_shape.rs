//! Step 1 — verify the public API surface compiles.

use kerf_cad::{lits, Feature, Model, Profile2D, Scalar};

#[test]
fn builder_accepts_box_and_lookup_returns_it() {
    let m = Model::new().add(Feature::Box {
        id: "body".into(),
        extents: lits([10.0, 5.0, 2.0]),
    });
    let f = m.feature("body").expect("body should exist");
    match f {
        Feature::Box { extents, .. } => {
            assert_eq!(extents[0], Scalar::Lit(10.0));
            assert_eq!(extents[1], Scalar::Lit(5.0));
            assert_eq!(extents[2], Scalar::Lit(2.0));
        }
        _ => panic!("expected Box"),
    }
}

#[test]
fn duplicate_id_returns_error_on_add() {
    let m = Model::new().add(Feature::Box {
        id: "x".into(),
        extents: lits([1.0, 1.0, 1.0]),
    });
    let r = m.try_add(Feature::Box {
        id: "x".into(),
        extents: lits([2.0, 2.0, 2.0]),
    });
    assert!(r.is_err(), "duplicate id should be rejected");
}

#[test]
fn all_feature_variants_construct() {
    let _ = Feature::Box {
        id: "a".into(),
        extents: lits([1.0, 1.0, 1.0]),
    };
    let _ = Feature::BoxAt {
        id: "a".into(),
        extents: lits([1.0, 1.0, 1.0]),
        origin: lits([0.0, 0.0, 0.0]),
    };
    let _ = Feature::Cylinder {
        id: "a".into(),
        radius: Scalar::lit(1.0),
        height: Scalar::lit(2.0),
        segments: 16,
    };
    let _ = Feature::Sphere {
        id: "a".into(),
        radius: Scalar::lit(1.0),
    };
    let _ = Feature::Torus {
        id: "a".into(),
        major_radius: Scalar::lit(2.0),
        minor_radius: Scalar::lit(0.5),
    };
    let _ = Feature::Cone {
        id: "a".into(),
        radius: Scalar::lit(1.0),
        height: Scalar::lit(2.0),
    };
    let _ = Feature::Frustum {
        id: "a".into(),
        top_radius: Scalar::lit(0.5),
        bottom_radius: Scalar::lit(1.0),
        height: Scalar::lit(2.0),
    };
    let _ = Feature::ExtrudePolygon {
        id: "a".into(),
        profile: Profile2D {
            points: vec![
                [Scalar::lit(0.0), Scalar::lit(0.0)],
                [Scalar::lit(1.0), Scalar::lit(0.0)],
                [Scalar::lit(1.0), Scalar::lit(1.0)],
                [Scalar::lit(0.0), Scalar::lit(1.0)],
            ],
        },
        direction: lits([0.0, 0.0, 1.0]),
    };
    let _ = Feature::Translate {
        id: "a".into(),
        input: "b".into(),
        offset: lits([1.0, 0.0, 0.0]),
    };
    let _ = Feature::Rotate {
        id: "a".into(),
        input: "b".into(),
        axis: lits([0.0, 0.0, 1.0]),
        angle_deg: Scalar::lit(90.0),
        center: lits([0.0, 0.0, 0.0]),
    };
    let _ = Feature::Union {
        id: "a".into(),
        inputs: vec!["b".into(), "c".into()],
    };
    let _ = Feature::Intersection {
        id: "a".into(),
        inputs: vec!["b".into(), "c".into()],
    };
    let _ = Feature::Difference {
        id: "a".into(),
        inputs: vec!["b".into(), "c".into()],
    };
}

#[test]
fn feature_id_is_accessible() {
    let f = Feature::Box {
        id: "the-id".into(),
        extents: lits([1.0, 1.0, 1.0]),
    };
    assert_eq!(f.id(), "the-id");
    let f2 = Feature::Difference {
        id: "diff".into(),
        inputs: vec!["a".into(), "b".into()],
    };
    assert_eq!(f2.id(), "diff");
}
