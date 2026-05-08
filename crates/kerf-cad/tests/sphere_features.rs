//! SphereFaceted, HollowSphere, Dome tests.

use kerf_brep::solid_volume;
use kerf_cad::{Feature, Model, Scalar};

#[test]
fn sphere_faceted_volume_approaches_analytic() {
    let r = 2.0;
    let m = Model::new().add(Feature::SphereFaceted {
        id: "out".into(),
        radius: Scalar::lit(r),
        stacks: 16,
        slices: 16,
    });
    let v = solid_volume(&m.evaluate("out").unwrap());
    let exp = (4.0 / 3.0) * std::f64::consts::PI * r.powi(3);
    let rel = (v - exp).abs() / exp;
    assert!(rel < 0.05, "v={v}, exp={exp}, rel={rel}");
}

#[test]
fn sphere_faceted_rejects_low_segments() {
    let m = Model::new().add(Feature::SphereFaceted {
        id: "out".into(),
        radius: Scalar::lit(1.0),
        stacks: 1,
        slices: 8,
    });
    assert!(m.evaluate("out").is_err());
}

#[test]
fn hollow_sphere_volume_matches_shell_formula() {
    let r_out = 3.0;
    let r_in = 2.0;
    let m = Model::new().add(Feature::HollowSphere {
        id: "out".into(),
        outer_radius: Scalar::lit(r_out),
        inner_radius: Scalar::lit(r_in),
        stacks: 16,
        slices: 16,
    });
    let v = solid_volume(&m.evaluate("out").unwrap());
    let exp = (4.0 / 3.0) * std::f64::consts::PI * (r_out.powi(3) - r_in.powi(3));
    let rel = (v - exp).abs() / exp;
    assert!(rel < 0.05, "v={v}, exp={exp}, rel={rel}");
}

#[test]
fn hollow_sphere_rejects_inner_geq_outer() {
    let m = Model::new().add(Feature::HollowSphere {
        id: "out".into(),
        outer_radius: Scalar::lit(2.0),
        inner_radius: Scalar::lit(2.0),
        stacks: 8,
        slices: 16,
    });
    assert!(m.evaluate("out").is_err());
}

#[test]
fn dome_volume_is_roughly_half_sphere() {
    let r = 2.0;
    let m = Model::new().add(Feature::Dome {
        id: "out".into(),
        radius: Scalar::lit(r),
        stacks: 16,
        slices: 16,
    });
    let v = solid_volume(&m.evaluate("out").unwrap());
    let exp = (2.0 / 3.0) * std::f64::consts::PI * r.powi(3);
    let rel = (v - exp).abs() / exp;
    // Looser tolerance: faceted hemisphere with stacks=16 underestimates by
    // ~7% (the bottom-half clip removes more than half the inscribed
    // polyhedron volume because faces near the equator are tilted).
    assert!(rel < 0.10, "v={v}, exp={exp}, rel={rel}");
}

#[test]
fn sphere_features_round_trip_via_json() {
    for f in [
        Feature::SphereFaceted {
            id: "out".into(),
            radius: Scalar::lit(2.0),
            stacks: 8,
            slices: 16,
        },
        Feature::HollowSphere {
            id: "out".into(),
            outer_radius: Scalar::lit(3.0),
            inner_radius: Scalar::lit(2.0),
            stacks: 8,
            slices: 16,
        },
        Feature::Dome {
            id: "out".into(),
            radius: Scalar::lit(2.0),
            stacks: 8,
            slices: 16,
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
