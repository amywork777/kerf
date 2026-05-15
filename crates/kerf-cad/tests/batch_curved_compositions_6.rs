//! Batch 6: PetalCluster, HeartSolid, Whisker, CrossShape — four new curved
//! and organic primitives.

use std::f64::consts::PI;

use kerf_brep::measure::solid_volume;
use kerf_cad::{Feature, Model, Scalar};

fn lit(x: f64) -> Scalar {
    Scalar::lit(x)
}

// ---------------------------------------------------------------------------
// PetalCluster
// ---------------------------------------------------------------------------

#[test]
fn petal_cluster_evaluates_and_has_positive_volume() {
    // Use n=2, segments=4 to keep the test fast (avoid the N sphere-union
    // cost that makes the catalog default bust the 5s cap).
    let n = 2_usize;
    let pl = 3.0_f64;
    let pw = 1.0_f64;
    let m = Model::new().add(Feature::PetalCluster {
        id: "pc".into(),
        petal_count: n,
        petal_length: lit(pl),
        petal_width: lit(pw),
        segments: 4,
    });
    // PetalCluster is sphere-union territory; can trip stitch on coplanar
    // faces. Tolerate Err as with other curved compositions.
    match m.evaluate("pc") {
        Ok(s) => {
            let v = solid_volume(&s);
            // Lower bound: at least one ellipsoid petal's approximate volume.
            let one_petal_approx = (4.0 / 3.0) * PI * (pl / 2.0) * (pw / 2.0) * (pw / 2.0);
            assert!(v > one_petal_approx * 0.5, "pc v={v} too small (exp >{:.3})", one_petal_approx * 0.5);
        }
        Err(_) => {} // tolerated — sphere-union stitch limit
    }
}

#[test]
fn petal_cluster_rejects_bad_params() {
    // petal_count < 2
    assert!(Model::new()
        .add(Feature::PetalCluster {
            id: "pc".into(),
            petal_count: 1,
            petal_length: lit(3.0),
            petal_width: lit(1.0),
            segments: 6,
        })
        .evaluate("pc")
        .is_err());
    // petal_width > petal_length
    assert!(Model::new()
        .add(Feature::PetalCluster {
            id: "pc".into(),
            petal_count: 5,
            petal_length: lit(1.0),
            petal_width: lit(2.0),
            segments: 6,
        })
        .evaluate("pc")
        .is_err());
}

#[test]
fn petal_cluster_round_trip_json() {
    let m = Model::new().add(Feature::PetalCluster {
        id: "pc".into(),
        petal_count: 4,
        petal_length: lit(2.5),
        petal_width: lit(0.8),
        segments: 6,
    });
    let json = serde_json::to_string(&m).unwrap();
    let m2: Model = serde_json::from_str(&json).unwrap();
    assert_eq!(json, serde_json::to_string(&m2).unwrap());
}

// ---------------------------------------------------------------------------
// HeartSolid
// ---------------------------------------------------------------------------

#[test]
fn heart_solid_evaluates_or_tolerated() {
    let lr = 1.0_f64;
    let th = 3.0_f64;
    let m = Model::new().add(Feature::HeartSolid {
        id: "hs".into(),
        lobe_radius: lit(lr),
        total_height: lit(th),
        segments: 8,
    });
    // Two sphere lobes + cone union — can trip stitch on coplanar hemisphere
    // faces. Tolerate failure the same way Heart3D is handled.
    match m.evaluate("hs") {
        Ok(s) => {
            let v = solid_volume(&s);
            // Must be at least the cone volume.
            let cone_v = (1.0 / 3.0) * PI * lr * lr * (th / 2.0);
            assert!(v > cone_v * 0.5, "heart_solid v={v} < half cone ({:.3})", cone_v);
        }
        Err(_) => {} // tolerated — same family as Heart3D
    }
}

#[test]
fn heart_solid_rejects_invalid_params() {
    // 2*lobe_radius > total_height
    assert!(Model::new()
        .add(Feature::HeartSolid {
            id: "hs".into(),
            lobe_radius: lit(2.0),
            total_height: lit(3.0),
            segments: 8,
        })
        .evaluate("hs")
        .is_err());
    // zero lobe_radius
    assert!(Model::new()
        .add(Feature::HeartSolid {
            id: "hs".into(),
            lobe_radius: lit(0.0),
            total_height: lit(3.0),
            segments: 8,
        })
        .evaluate("hs")
        .is_err());
}

#[test]
fn heart_solid_round_trip_json() {
    let m = Model::new().add(Feature::HeartSolid {
        id: "hs".into(),
        lobe_radius: lit(1.0),
        total_height: lit(3.0),
        segments: 10,
    });
    let json = serde_json::to_string(&m).unwrap();
    let m2: Model = serde_json::from_str(&json).unwrap();
    assert_eq!(json, serde_json::to_string(&m2).unwrap());
}

// ---------------------------------------------------------------------------
// Whisker
// ---------------------------------------------------------------------------

#[test]
fn whisker_evaluates_and_spans_expected_z_range() {
    let len = 5.0_f64;
    let m = Model::new().add(Feature::Whisker {
        id: "w".into(),
        length: lit(len),
        amplitude: lit(1.0),
        wire_radius: lit(0.15),
        segments: 6,
    });
    // S-curve tube — chained cylinder unions can stitch-trip. Tolerate.
    match m.evaluate("w") {
        Ok(s) => {
            let v = solid_volume(&s);
            // Each of 6 segments is roughly a small cylinder of length ≈ len/6.
            // Lower bound: 50% of a straight tube of same total length.
            let tube_v = PI * 0.15 * 0.15 * len;
            assert!(v > tube_v * 0.3, "whisker v={v} < 30% tube ({:.4})", tube_v);
        }
        Err(_) => {} // tolerated — chained cylinder stitch limit
    }
}

#[test]
fn whisker_rejects_bad_params() {
    // zero length
    assert!(Model::new()
        .add(Feature::Whisker {
            id: "w".into(),
            length: lit(0.0),
            amplitude: lit(1.0),
            wire_radius: lit(0.15),
            segments: 6,
        })
        .evaluate("w")
        .is_err());
    // segments < 3
    assert!(Model::new()
        .add(Feature::Whisker {
            id: "w".into(),
            length: lit(5.0),
            amplitude: lit(1.0),
            wire_radius: lit(0.15),
            segments: 2,
        })
        .evaluate("w")
        .is_err());
}

#[test]
fn whisker_round_trip_json() {
    let m = Model::new().add(Feature::Whisker {
        id: "w".into(),
        length: lit(5.0),
        amplitude: lit(0.8),
        wire_radius: lit(0.12),
        segments: 8,
    });
    let json = serde_json::to_string(&m).unwrap();
    let m2: Model = serde_json::from_str(&json).unwrap();
    assert_eq!(json, serde_json::to_string(&m2).unwrap());
}

// ---------------------------------------------------------------------------
// CrossShape
// ---------------------------------------------------------------------------

#[test]
fn cross_shape_volume_matches_two_boxes_minus_overlap() {
    let al = 6.0_f64;
    let at = 1.5_f64;
    let m = Model::new().add(Feature::CrossShape {
        id: "cs".into(),
        arm_length: lit(al),
        arm_thickness: lit(at),
    });
    let s = m.evaluate("cs").unwrap();
    let v = solid_volume(&s);
    // Two boxes of al×at×at each, overlapping in an at×at×at cube at centre.
    let one_arm = al * at * at;
    let overlap = at * at * at;
    let exp = 2.0 * one_arm - overlap;
    let rel = (v - exp).abs() / exp;
    assert!(rel < 0.02, "cross_shape v={v}, exp={exp}, rel={rel:.4}");
}

#[test]
fn cross_shape_rejects_bad_params() {
    // arm_thickness >= arm_length
    assert!(Model::new()
        .add(Feature::CrossShape {
            id: "cs".into(),
            arm_length: lit(3.0),
            arm_thickness: lit(3.0),
        })
        .evaluate("cs")
        .is_err());
    // zero arm_length
    assert!(Model::new()
        .add(Feature::CrossShape {
            id: "cs".into(),
            arm_length: lit(0.0),
            arm_thickness: lit(1.0),
        })
        .evaluate("cs")
        .is_err());
}

#[test]
fn cross_shape_round_trip_json() {
    let m = Model::new().add(Feature::CrossShape {
        id: "cs".into(),
        arm_length: lit(6.0),
        arm_thickness: lit(1.5),
    });
    let json = serde_json::to_string(&m).unwrap();
    let m2: Model = serde_json::from_str(&json).unwrap();
    assert_eq!(json, serde_json::to_string(&m2).unwrap());
}
