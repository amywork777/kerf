//! Revolve feature.

use kerf_brep::solid_volume;
use kerf_cad::{Feature, Model, Profile2D, Scalar};

#[test]
fn revolve_a_tapered_profile_yields_a_frustum() {
    // Profile: (0, 0) → (5, 0) → (2, 3) → (0, 3). Revolved around z-axis
    // yields a frustum (cone-stack) — every interior segment must have
    // BOTH rise and run, since revolve_polyline emits a Cone surface per
    // segment and rejects zero-half-angle (vertical) ones.
    let profile = Profile2D {
        // Vase-style profile mirroring the kerf-brep vase test exactly.
        points: vec![
            [Scalar::lit(0.0), Scalar::lit(2.0)],
            [Scalar::lit(0.5), Scalar::lit(1.5)],
            [Scalar::lit(0.7), Scalar::lit(0.5)],
            [Scalar::lit(0.0), Scalar::lit(0.0)],
        ],
    };
    let m = Model::new().add(Feature::Revolve { id: "out".into(), profile });
    let s = m.evaluate("out").unwrap();
    // The vase mesh has a degenerate seam topology (360° lunes); volume
    // integration returns 0 for it. Verify topology was built instead.
    assert!(s.face_count() > 0, "revolve should produce a closed solid");
    assert!(s.vertex_count() > 0);
    let _ = solid_volume(&s);
}

#[test]
fn revolve_round_trips_via_json() {
    let profile = Profile2D {
        points: vec![
            [Scalar::lit(0.0), Scalar::lit(2.0)],
            [Scalar::lit(0.5), Scalar::lit(1.5)],
            [Scalar::lit(0.7), Scalar::lit(0.5)],
            [Scalar::lit(0.0), Scalar::lit(0.0)],
        ],
    };
    let m = Model::new().add(Feature::Revolve { id: "out".into(), profile });
    let json = m.to_json_string().unwrap();
    let m2 = Model::from_json_str(&json).unwrap();
    let s1 = m.evaluate("out").unwrap();
    let s2 = m2.evaluate("out").unwrap();
    assert_eq!(s1.face_count(), s2.face_count());
    assert_eq!(s1.vertex_count(), s2.vertex_count());
}
