//! GAP 2: SweepPath — chain of cylinders along an arbitrary polyline.
//!
//! Tests volume against the analytic formula for axis-aligned and diagonal
//! segments. Verifies a JSON round-trip. Does NOT test rounded joints — the
//! sweep produces sharp miters intentionally (rounded joints trip the
//! boolean engine on multi-cylinder-and-sphere overlap, same as PipeRun).

use std::f64::consts::PI;

use kerf_brep::measure::solid_volume;
use kerf_cad::{Feature, Model, Scalar};

fn faceted_cyl_vol(r: f64, h: f64, segs: usize) -> f64 {
    // Faceted cylinder volume: regular n-gon area * h. n-gon area with
    // circumradius r: 0.5 * n * r^2 * sin(2pi/n).
    0.5 * segs as f64 * r * r * (2.0 * PI / segs as f64).sin() * h
}

#[test]
fn single_axis_aligned_segment_matches_cylinder_volume() {
    // (0,0,0) → (5,0,0), radius 1, 12 segments. Axis-aligned: hits the
    // exact-permutation fast path, so volume should be EXACTLY the
    // cylinder volume (no floating-point drift from rotation).
    let r = 1.0;
    let h = 5.0;
    let segs = 12;
    let m = Model::new().add(Feature::SweepPath {
        id: "sweep".into(),
        points: vec![
            [Scalar::lit(0.0), Scalar::lit(0.0), Scalar::lit(0.0)],
            [Scalar::lit(h), Scalar::lit(0.0), Scalar::lit(0.0)],
        ],
        radius: Scalar::lit(r),
        segments: segs,
    });
    let s = m.evaluate("sweep").unwrap();
    let vol = solid_volume(&s);
    let exp = faceted_cyl_vol(r, h, segs);
    assert!(
        (vol - exp).abs() < 1e-9,
        "axis-aligned single segment volume mismatch: got {vol}, expected {exp}"
    );
}

#[test]
fn single_diagonal_segment_volume_matches_analytic() {
    // (0,0,0) → (3,4,0): a 5-unit segment in the xy plane along (3,4)/5.
    // The rotation introduces floating-point noise, but volume is invariant.
    let r = 0.5;
    let segs = 16;
    let m = Model::new().add(Feature::SweepPath {
        id: "sweep".into(),
        points: vec![
            [Scalar::lit(0.0), Scalar::lit(0.0), Scalar::lit(0.0)],
            [Scalar::lit(3.0), Scalar::lit(4.0), Scalar::lit(0.0)],
        ],
        radius: Scalar::lit(r),
        segments: segs,
    });
    let s = m.evaluate("sweep").unwrap();
    let vol = solid_volume(&s);
    let exp = faceted_cyl_vol(r, 5.0, segs);
    assert!(
        (vol - exp).abs() < 1e-6,
        "diagonal single segment volume mismatch: got {vol}, expected {exp}"
    );
}

#[test]
fn l_bend_volume_matches_two_cylinders_minus_overlap() {
    // L-bend: (0,0,0) → (10,0,0) → (10,10,0), radius 0.5, 12 segs.
    // Two perpendicular cylinders share one cylinder-length of overlap
    // around the corner point, but they're perpendicular so the overlap
    // is small. A loose lower bound: each cylinder contributes its full
    // volume minus their overlap (≤ overlap_box volume = (2r)^3). The
    // result must be at least sum_minus_overlap_box and at most their
    // sum.
    let r = 0.5;
    let segs = 12;
    let h = 10.0;
    let m = Model::new().add(Feature::SweepPath {
        id: "sweep".into(),
        points: vec![
            [Scalar::lit(0.0), Scalar::lit(0.0), Scalar::lit(0.0)],
            [Scalar::lit(h), Scalar::lit(0.0), Scalar::lit(0.0)],
            [Scalar::lit(h), Scalar::lit(h), Scalar::lit(0.0)],
        ],
        radius: Scalar::lit(r),
        segments: segs,
    });
    let s = m.evaluate("sweep").unwrap();
    let vol = solid_volume(&s);
    let v_one = faceted_cyl_vol(r, h, segs);
    let two_sum = 2.0 * v_one;
    let overlap_upper_bound = (2.0 * r).powi(3); // bbox of overlap is (2r)^3
    assert!(
        vol > two_sum - overlap_upper_bound && vol <= two_sum,
        "L-bend volume out of expected range: got {vol}, two_sum={two_sum}, two_sum-overlap={}",
        two_sum - overlap_upper_bound
    );
}

#[test]
fn sweep_path_with_diagonal_zigzag_completes() {
    // Just verify that a diagonal zigzag evaluates to *some* solid
    // without panicking. Volume invariants are checked above.
    let m = Model::new().add(Feature::SweepPath {
        id: "sweep".into(),
        points: vec![
            [Scalar::lit(0.0), Scalar::lit(0.0), Scalar::lit(0.0)],
            [Scalar::lit(3.0), Scalar::lit(0.0), Scalar::lit(0.0)],
            [Scalar::lit(5.0), Scalar::lit(2.0), Scalar::lit(0.0)],
            [Scalar::lit(5.0), Scalar::lit(2.0), Scalar::lit(3.0)],
        ],
        radius: Scalar::lit(0.3),
        segments: 12,
    });
    let s = m.evaluate("sweep").unwrap();
    let v = solid_volume(&s);
    assert!(v > 0.0, "zigzag sweep should have positive volume, got {v}");
}

#[test]
fn sweep_path_round_trips_via_json() {
    let m = Model::new().add(Feature::SweepPath {
        id: "sweep".into(),
        points: vec![
            [Scalar::lit(0.0), Scalar::lit(0.0), Scalar::lit(0.0)],
            [Scalar::lit(2.0), Scalar::lit(3.0), Scalar::lit(0.0)],
        ],
        radius: Scalar::lit(0.4),
        segments: 16,
    });
    let json = m.to_json_string().unwrap();
    let parsed = Model::from_json_str(&json).unwrap();
    // Compare via re-serialized JSON to dodge the missing PartialEq on Model.
    let json2 = parsed.to_json_string().unwrap();
    assert_eq!(json, json2, "JSON round-trip must be lossless");
}

#[test]
fn sweep_path_rejects_zero_length_segment() {
    let m = Model::new().add(Feature::SweepPath {
        id: "sweep".into(),
        points: vec![
            [Scalar::lit(1.0), Scalar::lit(2.0), Scalar::lit(3.0)],
            [Scalar::lit(1.0), Scalar::lit(2.0), Scalar::lit(3.0)],
        ],
        radius: Scalar::lit(0.5),
        segments: 12,
    });
    let r = m.evaluate("sweep");
    assert!(r.is_err(), "zero-length sweep segment must error");
}
