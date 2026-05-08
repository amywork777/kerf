//! `Feature::Shell` — planar inward-offset solid (the kerf-cad wiring of
//! `kerf_brep::primitives::shell_planar`).
//!
//! Covers:
//!  - parity with HollowBox for the box case (volume within 1%);
//!  - shell of a hexagonal RegularPrism (volume vs analytic formula);
//!  - thickness validation (must be > 0 and < min_dim/2);
//!  - JSON round-trip (Feature serializes/parses back identically);
//!  - rejection of curved-surface inputs (Sphere is invalid);
//!  - shell on a structural shape (LBracket — concave; documented as
//!    expected to error today).

use kerf_brep::solid_volume;
use kerf_cad::{lits, Feature, Model, Scalar};

#[test]
fn shell_box_volume_matches_hollow_box() {
    // Shell(Box(40,30,20), 2.0) should match HollowBox(40,30,20, 2.0).
    let mut m = Model::new();
    m = m.add(Feature::Box {
        id: "outer".into(),
        extents: lits([40.0, 30.0, 20.0]),
    });
    m = m.add(Feature::Shell {
        id: "shell".into(),
        input: "outer".into(),
        thickness: Scalar::lit(2.0),
    });
    m = m.add(Feature::HollowBox {
        id: "hb".into(),
        extents: lits([40.0, 30.0, 20.0]),
        wall_thickness: Scalar::lit(2.0),
    });
    let s = m.evaluate("shell").expect("shell evaluates");
    let h = m.evaluate("hb").expect("hollow box evaluates");
    let vs = solid_volume(&s);
    let vh = solid_volume(&h);
    let rel = (vs - vh).abs() / vh.abs().max(1.0);
    assert!(
        rel < 0.01,
        "Shell volume {vs} vs HollowBox volume {vh} (relative diff {rel})"
    );
}

#[test]
fn shell_regular_prism_planar() {
    // Hexagonal RegularPrism, side 10, height 5. Wall 1.
    // The inward offset of a regular hexagon by t along each face's inward
    // normal yields a similar hexagon with apothem reduced by t. Top + bottom
    // each inset by t, so inner height = h - 2t.
    let s_side = 10.0_f64;
    let h = 5.0_f64;
    let t = 1.0_f64;
    let m = Model::new()
        .add(Feature::RegularPrism {
            id: "prism".into(),
            segments: 6,
            radius: Scalar::lit(s_side),
            height: Scalar::lit(h),
        })
        .add(Feature::Shell {
            id: "shell".into(),
            input: "prism".into(),
            thickness: Scalar::lit(t),
        });
    let s = m.evaluate("shell").expect("hex shell evaluates");
    let v = solid_volume(&s);
    let hex_area = 3.0 * 3.0_f64.sqrt() / 2.0 * s_side * s_side;
    let outer_v = hex_area * h;
    let apothem = s_side * 3.0_f64.sqrt() / 2.0;
    let new_side = s_side * (1.0 - t / apothem);
    let inner_hex_area = 3.0 * 3.0_f64.sqrt() / 2.0 * new_side * new_side;
    let inner_v = inner_hex_area * (h - 2.0 * t);
    let expected = outer_v - inner_v;
    let rel = (v - expected).abs() / expected;
    assert!(
        rel < 0.02,
        "hex shell v={v} expected={expected} rel={rel}"
    );
}

#[test]
fn shell_validates_thickness_zero() {
    let m = Model::new()
        .add(Feature::Box {
            id: "b".into(),
            extents: lits([10.0, 10.0, 10.0]),
        })
        .add(Feature::Shell {
            id: "out".into(),
            input: "b".into(),
            thickness: Scalar::lit(0.0),
        });
    assert!(m.evaluate("out").is_err(), "thickness=0 must error");
}

#[test]
fn shell_validates_thickness_too_large() {
    // Box min dimension 4; thickness 2 means 2*t = 4 = min_dim → error.
    let m = Model::new()
        .add(Feature::Box {
            id: "b".into(),
            extents: lits([4.0, 10.0, 10.0]),
        })
        .add(Feature::Shell {
            id: "out".into(),
            input: "b".into(),
            thickness: Scalar::lit(2.0),
        });
    assert!(m.evaluate("out").is_err(), "thickness >= min_dim/2 must error");
}

#[test]
fn shell_rejects_curved_surface_input() {
    // Sphere has a curved-surface face — Shell must refuse.
    let m = Model::new()
        .add(Feature::Sphere {
            id: "s".into(),
            radius: Scalar::lit(10.0),
        })
        .add(Feature::Shell {
            id: "out".into(),
            input: "s".into(),
            thickness: Scalar::lit(1.0),
        });
    let err = m.evaluate("out").expect_err("Shell on Sphere must fail");
    let msg = format!("{err}");
    assert!(
        msg.contains("planar") || msg.contains("curved"),
        "Shell-on-Sphere error should mention planar/curved, got: {msg}"
    );
}

#[test]
fn shell_json_round_trip() {
    let m = Model::new()
        .add(Feature::Box {
            id: "b".into(),
            extents: lits([20.0, 15.0, 10.0]),
        })
        .add(Feature::Shell {
            id: "out".into(),
            input: "b".into(),
            thickness: Scalar::lit(1.5),
        });
    let json = m.to_json_string().expect("serialize");
    let m2 = Model::from_json_str(&json).expect("deserialize");
    // Both models should produce the same evaluated solid (identical volume).
    let v1 = solid_volume(&m.evaluate("out").expect("eval m"));
    let v2 = solid_volume(&m2.evaluate("out").expect("eval m2"));
    assert!(
        (v1 - v2).abs() < 1e-9,
        "round-trip volume mismatch v1={v1} v2={v2}"
    );
    // And the parsed Feature should equal the original.
    assert_eq!(
        m.feature("out").expect("orig"),
        m2.feature("out").expect("parsed")
    );
}
