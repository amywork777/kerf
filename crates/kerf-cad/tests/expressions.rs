//! Scalar expression evaluation: arithmetic, parentheses, $vars, and a
//! handful of math builtins (sin, cos, sqrt, abs).

use kerf_brep::solid_volume;
use kerf_cad::{lits, Feature, Model, Scalar};

#[test]
fn expression_with_arithmetic_evaluates_correctly() {
    let m = Model::new()
        .with_parameter("plate_x", 100.0)
        .add(Feature::Box {
            id: "body".into(),
            extents: [
                Scalar::expr("$plate_x / 2"),
                Scalar::expr("$plate_x / 4 + 10"),
                Scalar::expr("8"),
            ],
        });
    let s = m.evaluate("body").unwrap();
    let v = solid_volume(&s);
    // 50 × 35 × 8 = 14000
    assert!((v - 14000.0).abs() < 1e-9, "v={v}");
}

#[test]
fn expression_supports_parens_and_precedence() {
    let m = Model::new()
        .with_parameter("a", 4.0)
        .with_parameter("b", 6.0)
        .add(Feature::Box {
            id: "body".into(),
            extents: [
                Scalar::expr("($a + $b) * 2"),  // 20
                Scalar::expr("$a * $b - 4"),     // 20
                Scalar::expr("$b - $a"),          // 2
            ],
        });
    let s = m.evaluate("body").unwrap();
    assert!((solid_volume(&s) - 800.0).abs() < 1e-9);
}

#[test]
fn expression_supports_unary_minus() {
    let m = Model::new()
        .with_parameter("h", 5.0)
        .add(Feature::BoxAt {
            id: "body".into(),
            extents: lits([1.0, 1.0, 1.0]),
            origin: [Scalar::expr("-$h"), Scalar::lit(0.0), Scalar::lit(0.0)],
        });
    let s = m.evaluate("body").unwrap();
    assert!((solid_volume(&s) - 1.0).abs() < 1e-9);
}

#[test]
fn expression_supports_math_builtins() {
    use std::f64::consts::PI;
    let m = Model::new()
        .with_parameter("theta_deg", 90.0)
        .with_parameter("r", 4.0)
        .add(Feature::Box {
            id: "body".into(),
            extents: [
                // sin(90°) = 1, so this is r * 1 = 4
                Scalar::expr("$r * sin($theta_deg * 3.14159265358979 / 180)"),
                Scalar::expr("sqrt(16)"),         // 4
                Scalar::expr("abs(-3) + 2"),      // 5
            ],
        });
    let s = m.evaluate("body").unwrap();
    let v = solid_volume(&s);
    assert!((v - 80.0).abs() < 1e-6, "v={v}");
    let _ = PI;
}

#[test]
fn missing_var_in_expression_errors() {
    let m = Model::new().add(Feature::Box {
        id: "body".into(),
        extents: [Scalar::expr("$nope + 1"), Scalar::lit(1.0), Scalar::lit(1.0)],
    });
    let r = m.evaluate("body");
    assert!(r.is_err());
    assert!(format!("{}", r.unwrap_err()).contains("nope"));
}

#[test]
fn malformed_expression_errors() {
    // unclosed paren
    let m = Model::new().add(Feature::Box {
        id: "body".into(),
        extents: [Scalar::expr("(1 + 2"), Scalar::lit(1.0), Scalar::lit(1.0)],
    });
    assert!(m.evaluate("body").is_err());
    // trailing operator
    let m = Model::new().add(Feature::Box {
        id: "body".into(),
        extents: [Scalar::expr("1 +"), Scalar::lit(1.0), Scalar::lit(1.0)],
    });
    assert!(m.evaluate("body").is_err());
}

#[test]
fn json_round_trip_preserves_expressions() {
    let m = Model::new()
        .with_parameter("plate_x", 100.0)
        .add(Feature::Box {
            id: "body".into(),
            extents: [Scalar::expr("$plate_x / 2"), Scalar::lit(60.0), Scalar::lit(8.0)],
        });
    let json = m.to_json_string().unwrap();
    let m2 = Model::from_json_str(&json).unwrap();
    let v1 = solid_volume(&m.evaluate("body").unwrap());
    let v2 = solid_volume(&m2.evaluate("body").unwrap());
    assert!((v1 - v2).abs() < 1e-9);
}

#[test]
fn bare_dollar_name_is_still_a_param() {
    // backwards compat: Scalar::param produces a Scalar that evaluates as
    // a single parameter lookup.
    let m = Model::new()
        .with_parameter("x", 7.0)
        .add(Feature::Box {
            id: "body".into(),
            extents: [Scalar::param("x"), Scalar::lit(2.0), Scalar::lit(3.0)],
        });
    let s = m.evaluate("body").unwrap();
    assert!((solid_volume(&s) - 42.0).abs() < 1e-9);
}
