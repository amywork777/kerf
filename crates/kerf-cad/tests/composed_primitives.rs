//! Composed primitives: Slot, HollowSphere, HollowCylinder, Wedge, RegularPrism.

use kerf_brep::solid_volume;
use kerf_cad::{Feature, Model, Scalar};

// -------- Slot --------

#[test]
fn slot_volume_matches_stadium_formula() {
    // Stadium = rect length × 2r + two semicircles of radius r = circle.
    // Volume = (length * 2r + π * r²) * height.
    // With segments-faceted semicircles: area = length * 2r + n * 0.5 * r² *
    // sin(2π/(2n)) approx... use a high segment count and 5% tolerance.
    let length = 4.0;
    let r = 1.0;
    let h = 2.0;
    let segments = 32;
    let m = Model::new().add(Feature::Slot {
        id: "out".into(),
        length: Scalar::lit(length),
        radius: Scalar::lit(r),
        height: Scalar::lit(h),
        segments,
    });
    let s = m.evaluate("out").unwrap();
    let v = solid_volume(&s);
    let exp = (length * 2.0 * r + std::f64::consts::PI * r * r) * h;
    let rel = (v - exp).abs() / exp;
    assert!(rel < 0.02, "v={v}, exp={exp}, rel={rel}");
}

#[test]
fn slot_zero_length_is_a_disk() {
    // length=0 reduces to a polygonal cylinder of radius r, height h.
    let r = 1.5;
    let h = 3.0;
    let segments = 24;
    let m = Model::new().add(Feature::Slot {
        id: "out".into(),
        length: Scalar::lit(0.0),
        radius: Scalar::lit(r),
        height: Scalar::lit(h),
        segments,
    });
    let s = m.evaluate("out").unwrap();
    let v = solid_volume(&s);
    let exp = std::f64::consts::PI * r * r * h;
    let rel = (v - exp).abs() / exp;
    assert!(rel < 0.02, "v={v}, exp={exp}, rel={rel}");
}

#[test]
fn slot_round_trips_via_json() {
    let m = Model::new().add(Feature::Slot {
        id: "out".into(),
        length: Scalar::lit(4.0),
        radius: Scalar::lit(1.0),
        height: Scalar::lit(2.0),
        segments: 16,
    });
    let json = m.to_json_string().unwrap();
    let m2 = Model::from_json_str(&json).unwrap();
    let v1 = solid_volume(&m.evaluate("out").unwrap());
    let v2 = solid_volume(&m2.evaluate("out").unwrap());
    assert!((v1 - v2).abs() < 1e-9);
}

// -------- HollowCylinder --------

#[test]
fn hollow_cylinder_volume_matches_formula() {
    // outer cylinder volume - inner cavity cylinder volume
    let r_out = 3.0;
    let r_in = 2.5;
    let h = 5.0;
    let t = 0.5;
    let segments = 32;
    let m = Model::new().add(Feature::HollowCylinder {
        id: "out".into(),
        outer_radius: Scalar::lit(r_out),
        inner_radius: Scalar::lit(r_in),
        height: Scalar::lit(h),
        end_thickness: Scalar::lit(t),
        segments,
    });
    let s = m.evaluate("out").unwrap();
    let v = solid_volume(&s);
    let cavity_h = h - 2.0 * t;
    let exp = std::f64::consts::PI * (r_out * r_out * h - r_in * r_in * cavity_h);
    let rel = (v - exp).abs() / exp;
    assert!(rel < 0.02, "v={v}, exp={exp}, rel={rel}");
}

#[test]
fn hollow_cylinder_rejects_thick_ends() {
    let m = Model::new().add(Feature::HollowCylinder {
        id: "out".into(),
        outer_radius: Scalar::lit(2.0),
        inner_radius: Scalar::lit(1.0),
        height: Scalar::lit(2.0),
        end_thickness: Scalar::lit(1.0),
        segments: 16,
    });
    assert!(m.evaluate("out").is_err());
}

// -------- Wedge --------

#[test]
fn wedge_volume_matches_half_box() {
    let w = 4.0;
    let d = 3.0;
    let h = 2.0;
    let m = Model::new().add(Feature::Wedge {
        id: "out".into(),
        width: Scalar::lit(w),
        depth: Scalar::lit(d),
        height: Scalar::lit(h),
    });
    let s = m.evaluate("out").unwrap();
    let v = solid_volume(&s);
    let exp = 0.5 * w * d * h;
    assert!((v - exp).abs() < 1e-6, "v={v}, exp={exp}");
}

// -------- RegularPrism --------

#[test]
fn regular_prism_hex_volume_matches_formula() {
    let r = 1.0;
    let h = 2.0;
    let n = 6;
    let m = Model::new().add(Feature::RegularPrism {
        id: "out".into(),
        radius: Scalar::lit(r),
        height: Scalar::lit(h),
        segments: n,
    });
    let s = m.evaluate("out").unwrap();
    let v = solid_volume(&s);
    // Inscribed n-gon area = (n/2) * r² * sin(2π/n)
    let area = 0.5 * (n as f64) * r * r * (2.0 * std::f64::consts::PI / n as f64).sin();
    let exp = area * h;
    assert!((v - exp).abs() < 1e-6, "v={v}, exp={exp}");
}

#[test]
fn regular_prism_round_trips_via_json() {
    let m = Model::new().add(Feature::RegularPrism {
        id: "out".into(),
        radius: Scalar::lit(2.0),
        height: Scalar::lit(3.0),
        segments: 8,
    });
    let json = m.to_json_string().unwrap();
    let m2 = Model::from_json_str(&json).unwrap();
    let v1 = solid_volume(&m.evaluate("out").unwrap());
    let v2 = solid_volume(&m2.evaluate("out").unwrap());
    assert!((v1 - v2).abs() < 1e-9);
}
