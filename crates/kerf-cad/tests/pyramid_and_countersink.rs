//! Pyramid (cone_faceted) and Countersink (cone_faceted-based) tests.

use kerf_brep::solid_volume;
use kerf_cad::{lits, Feature, Model, Scalar};

// -------- Pyramid --------

#[test]
fn pyramid_volume_matches_one_third_base_height() {
    // Square pyramid (n=4) with phase π/4: base is square of side r·√2,
    // area = 2r². Volume = (1/3) * 2r² * h.
    let r = 1.0;
    let h = 3.0;
    let m = Model::new().add(Feature::Pyramid {
        id: "out".into(),
        radius: Scalar::lit(r),
        height: Scalar::lit(h),
        segments: 4,
    });
    let v = solid_volume(&m.evaluate("out").unwrap());
    let exp = (1.0 / 3.0) * 2.0 * r * r * h;
    assert!((v - exp).abs() < 1e-9, "v={v}, exp={exp}");
}

#[test]
fn pyramid_high_n_approaches_circular_cone() {
    let r = 2.0;
    let h = 4.0;
    let m = Model::new().add(Feature::Pyramid {
        id: "out".into(),
        radius: Scalar::lit(r),
        height: Scalar::lit(h),
        segments: 64,
    });
    let v = solid_volume(&m.evaluate("out").unwrap());
    let exp = (1.0 / 3.0) * std::f64::consts::PI * r * r * h;
    let rel = (v - exp).abs() / exp;
    assert!(rel < 0.01, "v={v}, exp={exp}, rel={rel}");
}

#[test]
fn pyramid_subtracts_from_box() {
    let m = Model::new()
        .add(Feature::Box {
            id: "body".into(),
            extents: lits([10.0, 10.0, 5.0]),
        })
        .add(Feature::Pyramid {
            id: "pyr".into(),
            radius: Scalar::lit(1.0),
            height: Scalar::lit(2.0),
            segments: 12,
        })
        .add(Feature::Translate {
            id: "pyr_pos".into(),
            input: "pyr".into(),
            offset: lits([5.0, 5.0, 1.0]),
        })
        .add(Feature::Difference {
            id: "out".into(),
            inputs: vec!["body".into(), "pyr_pos".into()],
        });
    let v = solid_volume(&m.evaluate("out").unwrap());
    let n = 12.0;
    let pyr_base_area = 0.5 * n * 1.0 * 1.0 * (2.0 * std::f64::consts::PI / n).sin();
    let pyr_v = (1.0 / 3.0) * pyr_base_area * 2.0;
    let exp = 10.0 * 10.0 * 5.0 - pyr_v;
    assert!((v - exp).abs() < 1e-6, "v={v}, exp={exp}");
}

#[test]
fn pyramid_round_trips_via_json() {
    let m = Model::new().add(Feature::Pyramid {
        id: "out".into(),
        radius: Scalar::lit(1.5),
        height: Scalar::lit(2.5),
        segments: 8,
    });
    let json = m.to_json_string().unwrap();
    let m2 = Model::from_json_str(&json).unwrap();
    let v1 = solid_volume(&m.evaluate("out").unwrap());
    let v2 = solid_volume(&m2.evaluate("out").unwrap());
    assert!((v1 - v2).abs() < 1e-9);
}

// -------- Countersink --------

#[test]
fn countersink_volume_matches_drill_plus_cone() {
    // 20×20×10 box, countersink at top center (10, 10, 10).
    // drill r=2, csink r=4, csink d=3, total d=10.
    // Removed volume = drill cylinder (full depth) + cone-section.
    //   Drill: π·2²·10 = 40π (going from z=0 to z=10).
    //   Wait — actually the cone REPLACES the upper portion of the drill.
    //   The shape removed: drill of r=2 from z=0..10 unioned with cone
    //   that expands from r=2 at z=7 to r=4 at z=10.
    //   The cone region at z∈[7, 10] has radius 2 + (4-2)*(z-7)/3 = 2 + 2*(z-7)/3.
    //   Cone volume (frustum) = (h/3)·(A1 + A2 + sqrt(A1·A2)) where
    //     h=3, A1 = π·4 = 12.566, A2 = π·16 = 50.265.
    //     V_cone = (3/3)·(12.566 + 50.265 + sqrt(12.566·50.265))
    //            ≈ 12.566 + 50.265 + sqrt(631.65)
    //            ≈ 12.566 + 50.265 + 25.13 = 87.96
    //   But the drill cylinder ALSO occupies z∈[7, 10] with r=2 (volume π·4·3 = 12.57).
    //   Union: V_drill + V_cone - V_overlap = (π·4·10) + V_cone_frustum - π·4·3
    //        = 40π - 12π + V_cone = 28π + V_cone (drill at z<7 + frustum at z=7..10)
    //   With faceted polygons we approximate.
    let m = Model::new()
        .add(Feature::Box {
            id: "body".into(),
            extents: lits([20.0, 20.0, 10.0]),
        })
        .add(Feature::Countersink {
            id: "out".into(),
            input: "body".into(),
            axis: "z".into(),
            top_center: lits([10.0, 10.0, 10.0]),
            drill_radius: Scalar::lit(2.0),
            csink_radius: Scalar::lit(4.0),
            csink_depth: Scalar::lit(3.0),
            total_depth: Scalar::lit(10.0),
            segments: 32,
        });
    let v = solid_volume(&m.evaluate("out").unwrap());
    // Approximate expected: body - (drill below csink + frustum above)
    let drill_below = std::f64::consts::PI * 4.0 * 7.0; // r²·h for drill below cone
    let h = 3.0;
    let a1 = std::f64::consts::PI * 4.0; // r=2
    let a2 = std::f64::consts::PI * 16.0; // r=4
    let frustum_v = (h / 3.0) * (a1 + a2 + (a1 * a2).sqrt());
    let exp = 20.0 * 20.0 * 10.0 - drill_below - frustum_v;
    let rel = (v - exp).abs() / exp;
    assert!(rel < 0.02, "v={v}, exp={exp}, rel={rel}");
}

#[test]
fn countersink_rejects_csink_smaller_than_drill() {
    let m = Model::new()
        .add(Feature::Box {
            id: "body".into(),
            extents: lits([10.0, 10.0, 5.0]),
        })
        .add(Feature::Countersink {
            id: "out".into(),
            input: "body".into(),
            axis: "z".into(),
            top_center: lits([5.0, 5.0, 5.0]),
            drill_radius: Scalar::lit(3.0),
            csink_radius: Scalar::lit(2.0),
            csink_depth: Scalar::lit(1.0),
            total_depth: Scalar::lit(4.0),
            segments: 16,
        });
    assert!(m.evaluate("out").is_err());
}

#[test]
fn countersink_round_trips_via_json() {
    let m = Model::new()
        .add(Feature::Box {
            id: "body".into(),
            extents: lits([20.0, 20.0, 10.0]),
        })
        .add(Feature::Countersink {
            id: "out".into(),
            input: "body".into(),
            axis: "z".into(),
            top_center: lits([10.0, 10.0, 10.0]),
            drill_radius: Scalar::lit(2.0),
            csink_radius: Scalar::lit(4.0),
            csink_depth: Scalar::lit(3.0),
            total_depth: Scalar::lit(10.0),
            segments: 24,
        });
    let json = m.to_json_string().unwrap();
    let m2 = Model::from_json_str(&json).unwrap();
    let v1 = solid_volume(&m.evaluate("out").unwrap());
    let v2 = solid_volume(&m2.evaluate("out").unwrap());
    assert!((v1 - v2).abs() < 1e-9);
}
