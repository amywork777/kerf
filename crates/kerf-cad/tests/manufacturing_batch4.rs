//! Manufacturing batch 4: CenterDrill, OilHole, ReliefCut, WrenchFlats,
//! Knurl.

use std::f64::consts::PI;

use kerf_brep::solid_volume;
use kerf_cad::{lits, Feature, Model, Scalar};

fn lit(x: f64) -> Scalar { Scalar::lit(x) }

// -------------------------------------------------------------------------
// CenterDrill
// -------------------------------------------------------------------------

#[test]
fn center_drill_volume_bounds_match_two_cones() {
    let dr = 0.2;
    let dd = 0.5;
    let cr = 0.4;
    let cd = 0.15;
    let segs = 12;
    let m = Model::new().add(Feature::CenterDrill {
        id: "cd".into(),
        drill_radius: lit(dr),
        drill_depth: lit(dd),
        chamfer_radius: lit(cr),
        chamfer_depth: lit(cd),
        segments: segs,
    });
    let s = m.evaluate("cd").unwrap();
    let v = solid_volume(&s);
    // Lower bound: drill cone alone (πr²h/3).
    // Upper bound: chamfer-frustum-volume + drill-cone-volume.
    //   frustum volume = (πh/3)*(R² + R*r + r²)
    let drill_cone = PI * dr * dr * dd / 3.0;
    let chamfer_frustum = PI * cd * (cr * cr + cr * dr + dr * dr) / 3.0;
    let upper = drill_cone + chamfer_frustum + 0.05;
    let lower = drill_cone * 0.5; // faceted is smaller
    assert!(v > lower && v < upper, "v={v}, lower={lower}, upper={upper}");
}

#[test]
fn center_drill_round_trip_via_json() {
    let m = Model::new().add(Feature::CenterDrill {
        id: "cd".into(),
        drill_radius: lit(0.1),
        drill_depth: lit(0.3),
        chamfer_radius: lit(0.2),
        chamfer_depth: lit(0.1),
        segments: 12,
    });
    let json = m.to_json_string().unwrap();
    let m2 = Model::from_json_str(&json).unwrap();
    let v1 = solid_volume(&m.evaluate("cd").unwrap());
    let v2 = solid_volume(&m2.evaluate("cd").unwrap());
    assert!((v1 - v2).abs() < 1e-9, "v1={v1}, v2={v2}");
}

// -------------------------------------------------------------------------
// OilHole
// -------------------------------------------------------------------------

#[test]
fn oil_hole_volume_less_than_solid_body() {
    let br = 1.0;
    let bh = 4.0;
    let hr = 0.1;
    let segs = 16;
    // Angled hole through the body: enters from outside the body on one
    // side, exits the other side.
    let m = Model::new().add(Feature::OilHole {
        id: "oh".into(),
        body_radius: lit(br),
        body_height: lit(bh),
        hole_radius: lit(hr),
        entry: lits([-1.5, 0.0, 1.0]),
        exit: lits([1.5, 0.0, 3.0]),
        body_segments: segs,
        hole_segments: 12,
    });
    let s = m.evaluate("oh").unwrap();
    let v = solid_volume(&s);
    // Body alone is a faceted cylinder. Volume should be slightly less
    // than body volume.
    let body_v = 0.5 * segs as f64 * br * br * (2.0 * PI / segs as f64).sin() * bh;
    assert!(
        v < body_v && v > 0.9 * body_v,
        "oil_hole v={v}, body_v={body_v}"
    );
}

#[test]
fn oil_hole_round_trip_via_json() {
    let m = Model::new().add(Feature::OilHole {
        id: "oh".into(),
        body_radius: lit(1.0),
        body_height: lit(3.0),
        hole_radius: lit(0.08),
        entry: lits([-1.2, 0.0, 0.5]),
        exit: lits([1.2, 0.0, 2.5]),
        body_segments: 12,
        hole_segments: 8,
    });
    let json = m.to_json_string().unwrap();
    let m2 = Model::from_json_str(&json).unwrap();
    let v1 = solid_volume(&m.evaluate("oh").unwrap());
    let v2 = solid_volume(&m2.evaluate("oh").unwrap());
    assert!((v1 - v2).abs() < 1e-9, "v1={v1}, v2={v2}");
}

// -------------------------------------------------------------------------
// ReliefCut
// -------------------------------------------------------------------------

#[test]
fn relief_cut_volume_less_than_simple_stepped_shaft() {
    let sr = 0.5;
    let sh = 1.0;
    let lr = 1.0;
    let lh = 1.5;
    let segs = 16;
    let m = Model::new().add(Feature::ReliefCut {
        id: "rc".into(),
        small_radius: lit(sr),
        small_height: lit(sh),
        large_radius: lit(lr),
        large_height: lit(lh),
        relief_width: lit(0.1),
        relief_depth: lit(0.2),
        segments: segs,
    });
    let s = m.evaluate("rc").unwrap();
    let v = solid_volume(&s);
    let area_small = 0.5 * segs as f64 * sr * sr * (2.0 * PI / segs as f64).sin();
    let area_large = 0.5 * segs as f64 * lr * lr * (2.0 * PI / segs as f64).sin();
    // Without relief the full shaft volume is small_cyl + large_cyl.
    let full = area_small * sh + area_large * lh;
    assert!(
        v < full && v > 0.7 * full,
        "relief v={v}, full={full}"
    );
}

#[test]
fn relief_cut_round_trip_via_json() {
    let m = Model::new().add(Feature::ReliefCut {
        id: "rc".into(),
        small_radius: lit(0.4),
        small_height: lit(1.0),
        large_radius: lit(0.8),
        large_height: lit(1.0),
        relief_width: lit(0.1),
        relief_depth: lit(0.15),
        segments: 12,
    });
    let json = m.to_json_string().unwrap();
    let m2 = Model::from_json_str(&json).unwrap();
    let v1 = solid_volume(&m.evaluate("rc").unwrap());
    let v2 = solid_volume(&m2.evaluate("rc").unwrap());
    assert!((v1 - v2).abs() < 1e-9, "v1={v1}, v2={v2}");
}

// -------------------------------------------------------------------------
// WrenchFlats
// -------------------------------------------------------------------------

#[test]
fn wrench_flats_volume_less_than_full_cylinder() {
    let r = 1.0;
    let l = 4.0;
    let zs = 1.0;
    let fl = 1.5;
    let fd = 1.4; // < 2*r, so flats actually cut into the cylinder
    let segs = 16;
    let m = Model::new().add(Feature::WrenchFlats {
        id: "wf".into(),
        shaft_radius: lit(r),
        shaft_length: lit(l),
        flats_z_start: lit(zs),
        flats_length: lit(fl),
        flat_distance: lit(fd),
        segments: segs,
    });
    let s = m.evaluate("wf").unwrap();
    let v = solid_volume(&s);
    let cyl_area = 0.5 * segs as f64 * r * r * (2.0 * PI / segs as f64).sin();
    let cyl_v = cyl_area * l;
    // Removing two flats can't reduce volume to less than a thinner box
    // (flat_distance × 2*r × flats_length), so v ≥ that lower bound.
    let lower = cyl_area * (l - fl) + fd * (2.0 * r) * fl * 0.5;
    assert!(v < cyl_v && v > lower, "wrench v={v}, cyl={cyl_v}, lower={lower}");
}

#[test]
fn wrench_flats_round_trip_via_json() {
    let m = Model::new().add(Feature::WrenchFlats {
        id: "wf".into(),
        shaft_radius: lit(0.5),
        shaft_length: lit(2.0),
        flats_z_start: lit(0.5),
        flats_length: lit(0.8),
        flat_distance: lit(0.7),
        segments: 12,
    });
    let json = m.to_json_string().unwrap();
    let m2 = Model::from_json_str(&json).unwrap();
    let v1 = solid_volume(&m.evaluate("wf").unwrap());
    let v2 = solid_volume(&m2.evaluate("wf").unwrap());
    assert!((v1 - v2).abs() < 1e-9, "v1={v1}, v2={v2}");
}

// -------------------------------------------------------------------------
// Knurl
// -------------------------------------------------------------------------

#[test]
fn knurl_volume_less_than_smooth_cylinder() {
    let r = 1.0;
    let h = 2.0;
    let gd = 0.1;
    let count = 24;
    let m = Model::new().add(Feature::Knurl {
        id: "k".into(),
        radius: lit(r),
        height: lit(h),
        groove_depth: lit(gd),
        groove_count: count,
    });
    let s = m.evaluate("k").unwrap();
    let v = solid_volume(&s);
    // Profile is 2*count vertices alternating r and r-gd. Area is
    // sum of 2*count triangles.
    let n = 2 * count;
    let area: f64 = (0..n)
        .map(|i| {
            let r0 = if i % 2 == 0 { r } else { r - gd };
            let r1 = if (i + 1) % 2 == 0 { r } else { r - gd };
            let theta0 = 2.0 * PI * (i as f64) / (n as f64);
            let theta1 = 2.0 * PI * ((i + 1) as f64) / (n as f64);
            let x0 = r0 * theta0.cos();
            let y0 = r0 * theta0.sin();
            let x1 = r1 * theta1.cos();
            let y1 = r1 * theta1.sin();
            // Triangle from origin: 0.5 * |x0*y1 - x1*y0|
            0.5 * (x0 * y1 - x1 * y0).abs()
        })
        .sum();
    let exp = area * h;
    let rel = (v - exp).abs() / exp;
    assert!(rel < 0.05, "knurl v={v}, exp={exp}, rel={rel}");
    // And less than smooth cylinder.
    assert!(v < PI * r * r * h, "knurl v={v} should be < smooth cyl");
}

#[test]
fn knurl_round_trip_via_json() {
    let m = Model::new().add(Feature::Knurl {
        id: "k".into(),
        radius: lit(0.5),
        height: lit(1.5),
        groove_depth: lit(0.05),
        groove_count: 16,
    });
    let json = m.to_json_string().unwrap();
    let m2 = Model::from_json_str(&json).unwrap();
    let v1 = solid_volume(&m.evaluate("k").unwrap());
    let v2 = solid_volume(&m2.evaluate("k").unwrap());
    assert!((v1 - v2).abs() < 1e-9, "v1={v1}, v2={v2}");
}

// -------------------------------------------------------------------------
// Validation rejection
// -------------------------------------------------------------------------

#[test]
fn rejects_invalid_inputs() {
    // CenterDrill: chamfer_radius < drill_radius rejected.
    let m = Model::new().add(Feature::CenterDrill {
        id: "cd".into(),
        drill_radius: lit(0.5),
        drill_depth: lit(1.0),
        chamfer_radius: lit(0.3), // smaller, invalid
        chamfer_depth: lit(0.2),
        segments: 12,
    });
    assert!(m.evaluate("cd").is_err());

    // OilHole: zero-length entry-exit.
    let m = Model::new().add(Feature::OilHole {
        id: "oh".into(),
        body_radius: lit(1.0),
        body_height: lit(2.0),
        hole_radius: lit(0.1),
        entry: lits([0.0, 0.0, 0.0]),
        exit: lits([0.0, 0.0, 0.0]),
        body_segments: 12,
        hole_segments: 8,
    });
    assert!(m.evaluate("oh").is_err());

    // WrenchFlats: flat_distance >= 2*radius (no actual cut) rejected.
    let m = Model::new().add(Feature::WrenchFlats {
        id: "wf".into(),
        shaft_radius: lit(1.0),
        shaft_length: lit(2.0),
        flats_z_start: lit(0.0),
        flats_length: lit(1.0),
        flat_distance: lit(2.5), // > 2*r
        segments: 12,
    });
    assert!(m.evaluate("wf").is_err());

    // Knurl: groove_depth >= radius rejected.
    let m = Model::new().add(Feature::Knurl {
        id: "k".into(),
        radius: lit(0.5),
        height: lit(1.0),
        groove_depth: lit(0.6),
        groove_count: 16,
    });
    assert!(m.evaluate("k").is_err());

    // ReliefCut: small_radius >= large_radius rejected.
    let m = Model::new().add(Feature::ReliefCut {
        id: "rc".into(),
        small_radius: lit(1.0),
        small_height: lit(1.0),
        large_radius: lit(1.0), // == small, invalid
        large_height: lit(1.0),
        relief_width: lit(0.1),
        relief_depth: lit(0.1),
        segments: 12,
    });
    assert!(m.evaluate("rc").is_err());
}
