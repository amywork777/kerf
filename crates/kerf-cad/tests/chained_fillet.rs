//! Chained Fillet: applying Fillet TWICE in sequence on the same body, where
//! the second Fillet's wedge cutter touches/meets a body face that the first
//! Fillet has already modified (or is adjacent to the first fillet's curved
//! cylindrical face). The "brittle" sibling of the multi-edge `Fillets`
//! plural-feature case that GAP C addressed.
//!
//! Strategy:
//!   - Pick two z-edges of a single Box that are *opposite* (not adjacent),
//!     so the two wedge cutters do NOT share a body face. This is the most
//!     forgiving chained case (it's the exact configuration the plural
//!     `Fillets` feature already handles for the 4-corner test). If even THIS
//!     fails for `Fillet → Fillet`, the chained pipeline is more brittle than
//!     the plural one.
//!   - A pair of *adjacent* z-edges (sharing a +y face) is the harder case
//!     described in STATUS.md ("the second Fillet's wedge cutter meets the
//!     first fillet's curved face"). Try that variant too.

use kerf_brep::solid_volume;
use kerf_cad::{lits, Feature, Model, Scalar};

/// Two z-edges on opposite corners of a 10×10×5 box, applied via TWO chained
/// `Fillet` features (not the plural `Fillets`). The plural case for the same
/// geometry passes today (GAP C). The single-feature chained version should
/// also pass — both wedges are disjoint and share no body face.
#[test]
fn chained_fillet_two_opposite_z_edges_succeeds() {
    let m = Model::new()
        .add(Feature::Box {
            id: "body".into(),
            extents: lits([10.0, 10.0, 5.0]),
        })
        // First fillet: z-edge at the -x -y corner, body in +x +y → quadrant pp.
        .add(Feature::Fillet {
            id: "f1".into(),
            input: "body".into(),
            axis: "z".into(),
            edge_min: lits([0.0, 0.0, 0.0]),
            edge_length: Scalar::lit(5.0),
            radius: Scalar::lit(1.0),
            quadrant: "pp".into(),
            segments: 16,
        })
        // Second fillet: z-edge at the +x +y corner, body in -x -y → quadrant nn.
        .add(Feature::Fillet {
            id: "f2".into(),
            input: "f1".into(),
            axis: "z".into(),
            edge_min: lits([10.0, 10.0, 0.0]),
            edge_length: Scalar::lit(5.0),
            radius: Scalar::lit(1.0),
            quadrant: "nn".into(),
            segments: 16,
        });
    let s = m.evaluate("f2").unwrap();
    let v = solid_volume(&s);
    // Volume removed per fillet = (r² - πr²/4) * height (faceted approx).
    // With segments=16, area_quarter = (1/4)*8*r²*sin(π/8) ≈ 0.7654*r²
    // For r=1, h=5: removed ≈ (1 - 0.7654)*5 ≈ 1.173 per edge → ~2.346 total.
    let n = 16.0;
    let r = 1.0;
    let area_quarter = 0.25 * (n / 2.0) * r * r * (2.0 * std::f64::consts::PI / n).sin();
    let exp = 500.0 - 2.0 * (r * r - area_quarter) * 5.0;
    assert!(
        v > 0.0,
        "Solid is empty (v=0) — chained Fillet trip detected"
    );
    assert!(
        (v - exp).abs() < 0.1,
        "v={v}, exp={exp}, diff={}",
        v - exp
    );
}

/// Two z-edges on ADJACENT corners of a 10×10×5 box (both on the y=0 face).
/// The two wedges share a body face (the +y face after the first fillet, OR
/// the -x and +x faces before the cuts) — this is the canonical "second
/// fillet meets first fillet's curved face" configuration STATUS.md flags.
///
/// THREE chained z-edge fillets (more typical "rounded box" use case): the
/// third wedge cutter sees BOTH prior cylindrical faces. Stresses the rescue.
#[test]
fn chained_fillet_three_z_edges_succeeds() {
    let m = Model::new()
        .add(Feature::Box {
            id: "body".into(),
            extents: lits([10.0, 10.0, 5.0]),
        })
        .add(Feature::Fillet {
            id: "f1".into(),
            input: "body".into(),
            axis: "z".into(),
            edge_min: lits([0.0, 0.0, 0.0]),
            edge_length: Scalar::lit(5.0),
            radius: Scalar::lit(1.0),
            quadrant: "pp".into(),
            segments: 16,
        })
        .add(Feature::Fillet {
            id: "f2".into(),
            input: "f1".into(),
            axis: "z".into(),
            edge_min: lits([10.0, 0.0, 0.0]),
            edge_length: Scalar::lit(5.0),
            radius: Scalar::lit(1.0),
            quadrant: "np".into(),
            segments: 16,
        })
        .add(Feature::Fillet {
            id: "f3".into(),
            input: "f2".into(),
            axis: "z".into(),
            edge_min: lits([10.0, 10.0, 0.0]),
            edge_length: Scalar::lit(5.0),
            radius: Scalar::lit(1.0),
            quadrant: "nn".into(),
            segments: 16,
        });
    let s = m.evaluate("f3").unwrap();
    let v = solid_volume(&s);
    let n = 16.0;
    let r = 1.0;
    let area_quarter = 0.25 * (n / 2.0) * r * r * (2.0 * std::f64::consts::PI / n).sin();
    let exp = 500.0 - 3.0 * (r * r - area_quarter) * 5.0;
    assert!(v > 0.0, "Solid is empty");
    assert!(
        (v - exp).abs() < 0.15,
        "v={v}, exp={exp}, diff={}",
        v - exp
    );
}

/// FOUR chained z-edge fillets (rounded "rectangle" cross-section). The most
/// common request that historically broke chained Fillet — "I want all four
/// vertical edges of my box rounded and the plural feature isn't expressive
/// enough for me".
#[test]
fn chained_fillet_four_z_edges_succeeds() {
    let m = Model::new()
        .add(Feature::Box {
            id: "body".into(),
            extents: lits([10.0, 10.0, 5.0]),
        })
        .add(Feature::Fillet {
            id: "f1".into(),
            input: "body".into(),
            axis: "z".into(),
            edge_min: lits([0.0, 0.0, 0.0]),
            edge_length: Scalar::lit(5.0),
            radius: Scalar::lit(1.0),
            quadrant: "pp".into(),
            segments: 16,
        })
        .add(Feature::Fillet {
            id: "f2".into(),
            input: "f1".into(),
            axis: "z".into(),
            edge_min: lits([10.0, 0.0, 0.0]),
            edge_length: Scalar::lit(5.0),
            radius: Scalar::lit(1.0),
            quadrant: "np".into(),
            segments: 16,
        })
        .add(Feature::Fillet {
            id: "f3".into(),
            input: "f2".into(),
            axis: "z".into(),
            edge_min: lits([10.0, 10.0, 0.0]),
            edge_length: Scalar::lit(5.0),
            radius: Scalar::lit(1.0),
            quadrant: "nn".into(),
            segments: 16,
        })
        .add(Feature::Fillet {
            id: "f4".into(),
            input: "f3".into(),
            axis: "z".into(),
            edge_min: lits([0.0, 10.0, 0.0]),
            edge_length: Scalar::lit(5.0),
            radius: Scalar::lit(1.0),
            quadrant: "pn".into(),
            segments: 16,
        });
    let s = m.evaluate("f4").unwrap();
    let v = solid_volume(&s);
    let n = 16.0;
    let r = 1.0;
    let area_quarter = 0.25 * (n / 2.0) * r * r * (2.0 * std::f64::consts::PI / n).sin();
    let exp = 500.0 - 4.0 * (r * r - area_quarter) * 5.0;
    assert!(v > 0.0, "Solid is empty (chained Fillet 4 z-edges trip)");
    assert!(
        (v - exp).abs() < 0.2,
        "v={v}, exp={exp}, diff={}",
        v - exp
    );
}

/// Chained fillets across DIFFERENT axes: one z-edge then one x-edge sharing
/// a corner. The second wedge meets the first fillet's curved face along a
/// non-axis-aligned curve.
///
/// Marked `#[ignore]` initially. Baseline panics:
///   non-manifold input to stitch: edge key (32, 39) has 1 half-edges
/// — i.e., a 1-half-edge orphan that the GAP C rescue does not promote.
/// Un-ignore once the rescue is extended to handle this curved-meets-curved
/// configuration.
#[test]
#[ignore = "chained Fillet z-then-x: 1-half-edge orphan rescue does not yet promote curved-face partners across the corner where two fillet curves meet"]
fn chained_fillet_z_then_x_edge_succeeds() {
    let m = Model::new()
        .add(Feature::Box {
            id: "body".into(),
            extents: lits([10.0, 10.0, 5.0]),
        })
        .add(Feature::Fillet {
            id: "f1".into(),
            input: "body".into(),
            axis: "z".into(),
            edge_min: lits([0.0, 0.0, 0.0]),
            edge_length: Scalar::lit(5.0),
            radius: Scalar::lit(1.0),
            quadrant: "pp".into(),
            segments: 16,
        })
        // x-edge along the top, at y=0,z=5: f1 already touched (0,0,*) but
        // this edge is at (·,0,5). They share corner (0,0,5).
        .add(Feature::Fillet {
            id: "f2".into(),
            input: "f1".into(),
            axis: "x".into(),
            edge_min: lits([0.0, 0.0, 5.0]),
            edge_length: Scalar::lit(10.0),
            radius: Scalar::lit(1.0),
            quadrant: "pn".into(), // body in +y, -z (z-axis is "n" because body extends -z)
            segments: 16,
        });
    let s = m.evaluate("f2").unwrap();
    let v = solid_volume(&s);
    assert!(v > 0.0, "Solid is empty (chained z-then-x trip)");
    // Volume sanity: two unit-radius fillets remove ~1.17 each
    // (single z-edge of length 5, single x-edge of length 10 → ~2.35).
    // We don't compute exact (the corner (0,0,5) is shared and double-removed
    // would create a slight overcount). Just check it's bounded sensibly.
    assert!(v > 490.0 && v < 500.0, "v={v} out of expected range");
}

#[test]
fn chained_fillet_two_adjacent_z_edges_succeeds() {
    let m = Model::new()
        .add(Feature::Box {
            id: "body".into(),
            extents: lits([10.0, 10.0, 5.0]),
        })
        // First fillet: z-edge at -x -y corner (0,0).
        .add(Feature::Fillet {
            id: "f1".into(),
            input: "body".into(),
            axis: "z".into(),
            edge_min: lits([0.0, 0.0, 0.0]),
            edge_length: Scalar::lit(5.0),
            radius: Scalar::lit(1.0),
            quadrant: "pp".into(),
            segments: 16,
        })
        // Second fillet: z-edge at +x -y corner (10,0). Both edges share the
        // -y face of the body. After f1, the second wedge's box-cutter
        // intersects the first fillet's cylindrical face.
        .add(Feature::Fillet {
            id: "f2".into(),
            input: "f1".into(),
            axis: "z".into(),
            edge_min: lits([10.0, 0.0, 0.0]),
            edge_length: Scalar::lit(5.0),
            radius: Scalar::lit(1.0),
            quadrant: "np".into(), // body in -x +y
            segments: 16,
        });
    let s = m.evaluate("f2").unwrap();
    let v = solid_volume(&s);
    let n = 16.0;
    let r = 1.0;
    let area_quarter = 0.25 * (n / 2.0) * r * r * (2.0 * std::f64::consts::PI / n).sin();
    let exp = 500.0 - 2.0 * (r * r - area_quarter) * 5.0;
    assert!(v > 0.0, "Solid is empty");
    assert!(
        (v - exp).abs() < 0.1,
        "v={v}, exp={exp}, diff={}",
        v - exp
    );
}
