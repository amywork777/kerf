//! Loft and TaperedExtrude tests.

use kerf_brep::solid_volume;
use kerf_cad::{Feature, Model, Profile2D, Scalar};

fn lit_xy(x: f64, y: f64) -> [Scalar; 2] {
    [Scalar::lit(x), Scalar::lit(y)]
}

#[test]
fn loft_between_two_squares_matches_prism() {
    // Loft from (0,0)-(2,0)-(2,2)-(0,2) to itself: should be a 2x2x3 box.
    let pts = vec![lit_xy(0.0, 0.0), lit_xy(2.0, 0.0), lit_xy(2.0, 2.0), lit_xy(0.0, 2.0)];
    let m = Model::new().add(Feature::Loft {
        id: "out".into(),
        bottom: Profile2D { points: pts.clone() },
        top: Profile2D { points: pts },
        height: Scalar::lit(3.0),
    });
    let v = solid_volume(&m.evaluate("out").unwrap());
    assert!((v - 12.0).abs() < 1e-9, "v={v}");
}

#[test]
fn loft_pyramid_via_4_to_1_squares() {
    // Bottom: 2x2 square. Top: tiny square (apex-like). Volume should
    // approach a pyramid as top → 0.
    let bottom = vec![lit_xy(-1.0, -1.0), lit_xy(1.0, -1.0), lit_xy(1.0, 1.0), lit_xy(-1.0, 1.0)];
    let top = vec![lit_xy(-0.001, -0.001), lit_xy(0.001, -0.001), lit_xy(0.001, 0.001), lit_xy(-0.001, 0.001)];
    let m = Model::new().add(Feature::Loft {
        id: "out".into(),
        bottom: Profile2D { points: bottom },
        top: Profile2D { points: top },
        height: Scalar::lit(3.0),
    });
    let v = solid_volume(&m.evaluate("out").unwrap());
    // Frustum formula: V = h/3 * (A1 + A2 + sqrt(A1*A2))
    //   = 3/3 * (4 + 4e-6 + sqrt(16e-6))
    //   ≈ 4.004
    let exp = 1.0 * (4.0 + 4e-6 + (16e-6_f64).sqrt());
    assert!((v - exp).abs() < 0.01, "v={v}, exp={exp}");
}

#[test]
fn tapered_extrude_with_scale_one_is_a_prism() {
    let m = Model::new().add(Feature::TaperedExtrude {
        id: "out".into(),
        profile: Profile2D {
            points: vec![
                lit_xy(0.0, 0.0),
                lit_xy(2.0, 0.0),
                lit_xy(2.0, 2.0),
                lit_xy(0.0, 2.0),
            ],
        },
        height: Scalar::lit(3.0),
        top_scale: Scalar::lit(1.0),
    });
    let v = solid_volume(&m.evaluate("out").unwrap());
    assert!((v - 12.0).abs() < 1e-9, "v={v}");
}

#[test]
fn tapered_extrude_half_scale_volume_matches() {
    // 2x2 base scaled to 1x1 top over height 4. Volume of frustum:
    // h/3 * (A1 + A2 + sqrt(A1*A2)) = 4/3 * (4 + 1 + 2) = 28/3 ≈ 9.333.
    let m = Model::new().add(Feature::TaperedExtrude {
        id: "out".into(),
        profile: Profile2D {
            points: vec![
                lit_xy(-1.0, -1.0),
                lit_xy(1.0, -1.0),
                lit_xy(1.0, 1.0),
                lit_xy(-1.0, 1.0),
            ],
        },
        height: Scalar::lit(4.0),
        top_scale: Scalar::lit(0.5),
    });
    let v = solid_volume(&m.evaluate("out").unwrap());
    let exp = 4.0 / 3.0 * (4.0 + 1.0 + 2.0);
    assert!((v - exp).abs() < 1e-9, "v={v}, exp={exp}");
}

#[test]
fn loft_rejects_mismatched_vertex_counts() {
    let m = Model::new().add(Feature::Loft {
        id: "out".into(),
        bottom: Profile2D { points: vec![lit_xy(0.0, 0.0), lit_xy(1.0, 0.0), lit_xy(0.5, 1.0)] },
        top: Profile2D { points: vec![lit_xy(0.0, 0.0), lit_xy(1.0, 0.0), lit_xy(1.0, 1.0), lit_xy(0.0, 1.0)] },
        height: Scalar::lit(2.0),
    });
    assert!(m.evaluate("out").is_err());
}

#[test]
fn tapered_extrude_rejects_zero_scale() {
    let m = Model::new().add(Feature::TaperedExtrude {
        id: "out".into(),
        profile: Profile2D {
            points: vec![lit_xy(0.0, 0.0), lit_xy(1.0, 0.0), lit_xy(0.0, 1.0)],
        },
        height: Scalar::lit(1.0),
        top_scale: Scalar::lit(0.0),
    });
    assert!(m.evaluate("out").is_err());
}

#[test]
fn loft_round_trips_via_json() {
    let m = Model::new().add(Feature::Loft {
        id: "out".into(),
        bottom: Profile2D {
            points: vec![lit_xy(0.0, 0.0), lit_xy(2.0, 0.0), lit_xy(2.0, 2.0), lit_xy(0.0, 2.0)],
        },
        top: Profile2D {
            points: vec![lit_xy(0.5, 0.5), lit_xy(1.5, 0.5), lit_xy(1.5, 1.5), lit_xy(0.5, 1.5)],
        },
        height: Scalar::lit(3.0),
    });
    let json = m.to_json_string().unwrap();
    let m2 = Model::from_json_str(&json).unwrap();
    let v1 = solid_volume(&m.evaluate("out").unwrap());
    let v2 = solid_volume(&m2.evaluate("out").unwrap());
    assert!((v1 - v2).abs() < 1e-9);
}
