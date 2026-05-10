//! Manufacturing batch 5: ShaftOilHole, WoodruffKey, DraftedHole,
//! HexFlange, Heatset.

use kerf_brep::solid_volume;
use kerf_cad::{Feature, Model, Scalar};

fn lit(x: f64) -> Scalar {
    Scalar::lit(x)
}

// ---------------------------------------------------------------------------
// ShaftOilHole
// ---------------------------------------------------------------------------

#[test]
fn shaft_oil_hole_reduces_volume() {
    // Build a box as the input body, then cut a radial bore through it.
    // Box: 4x4x4. Center of bore at (2,2,2) with axis=z, radius=0.3, depth=2.0
    // so the bore extends ±2.0 along x from x=2: from x=0 to x=4 (through full box).
    let m = Model::new()
        .add(Feature::Box {
            id: "body".into(),
            extents: [lit(4.0), lit(4.0), lit(4.0)],
        })
        .add(Feature::ShaftOilHole {
            id: "soh".into(),
            input: "body".into(),
            center: [lit(2.0), lit(2.0), lit(2.0)],
            axis: "z".into(),
            radius: lit(0.3),
            depth: lit(2.0),
            segments: 12,
        });
    let body_vol = solid_volume(&m.evaluate("body").unwrap());
    let soh_vol = solid_volume(&m.evaluate("soh").unwrap());
    // Bore removes material — soh volume < body volume.
    assert!(
        soh_vol < body_vol,
        "soh_vol={soh_vol} should be < body_vol={body_vol}"
    );
    // Bore runs through the full 4-unit width, so bore volume ≈ π r² * 4 = π * 0.09 * 4 ≈ 1.131.
    // The actual removed volume is > 0 (just verify some material was removed).
    assert!(
        body_vol - soh_vol > 0.5,
        "removed volume too small: removed={}",
        body_vol - soh_vol
    );
}

#[test]
fn shaft_oil_hole_roundtrip_json() {
    let m = Model::new()
        .add(Feature::Box {
            id: "shaft".into(),
            extents: [lit(2.0), lit(2.0), lit(6.0)],
        })
        .add(Feature::ShaftOilHole {
            id: "soh".into(),
            input: "shaft".into(),
            center: [lit(1.0), lit(1.0), lit(3.0)],
            axis: "z".into(),
            radius: lit(0.2),
            depth: lit(1.5),
            segments: 8,
        });
    let json = m.to_json_string().unwrap();
    let m2 = Model::from_json_str(&json).unwrap();
    let v1 = solid_volume(&m.evaluate("soh").unwrap());
    let v2 = solid_volume(&m2.evaluate("soh").unwrap());
    assert!((v1 - v2).abs() < 1e-9, "v1={v1}, v2={v2}");
}

// ---------------------------------------------------------------------------
// WoodruffKey
// ---------------------------------------------------------------------------

#[test]
fn woodruff_key_reduces_volume() {
    let m = Model::new()
        .add(Feature::Box {
            id: "shaft".into(),
            extents: [lit(2.0), lit(2.0), lit(8.0)],
        })
        .add(Feature::WoodruffKey {
            id: "wk".into(),
            input: "shaft".into(),
            center: [lit(1.0), lit(1.0), lit(4.0)],
            axis: "z".into(),
            radius: lit(0.8),
            width: lit(0.5),
            depth: lit(0.4),
            segments: 12,
        });
    let body_vol = solid_volume(&m.evaluate("shaft").unwrap());
    let wk_vol = solid_volume(&m.evaluate("wk").unwrap());
    // Keyway must remove some material.
    assert!(
        wk_vol < body_vol,
        "wk_vol={wk_vol} should be < body_vol={body_vol}"
    );
}

#[test]
fn woodruff_key_depth_exceeds_radius_errors() {
    let m = Model::new()
        .add(Feature::Box {
            id: "s".into(),
            extents: [lit(4.0), lit(4.0), lit(4.0)],
        })
        .add(Feature::WoodruffKey {
            id: "wk".into(),
            input: "s".into(),
            center: [lit(2.0), lit(2.0), lit(2.0)],
            axis: "z".into(),
            radius: lit(0.5),
            width: lit(0.4),
            depth: lit(0.8), // depth > radius — invalid
            segments: 8,
        });
    assert!(m.evaluate("wk").is_err());
}

// ---------------------------------------------------------------------------
// DraftedHole
// ---------------------------------------------------------------------------

#[test]
fn drafted_hole_reduces_volume_with_frustum_shape() {
    let m = Model::new()
        .add(Feature::Box {
            id: "plate".into(),
            extents: [lit(5.0), lit(5.0), lit(3.0)],
        })
        .add(Feature::DraftedHole {
            id: "dh".into(),
            input: "plate".into(),
            center: [lit(2.5), lit(2.5), lit(3.0)],
            axis: "z".into(),
            top_radius: lit(0.8),
            bottom_radius: lit(0.5),
            depth: lit(2.5),
            segments: 12,
        });
    let plate_vol = solid_volume(&m.evaluate("plate").unwrap());
    let dh_vol = solid_volume(&m.evaluate("dh").unwrap());
    assert!(
        dh_vol < plate_vol,
        "dh_vol={dh_vol} should be < plate_vol={plate_vol}"
    );
    // Volume removed must be between the two cylinders of same depth.
    let pi = std::f64::consts::PI;
    let min_removed = pi * 0.5 * 0.5 * 2.5;
    let max_removed = pi * 0.8 * 0.8 * 2.5 + 1.0; // +1 headroom
    let removed = plate_vol - dh_vol;
    assert!(
        removed > min_removed && removed < max_removed,
        "removed={removed}, min={min_removed}, max={max_removed}"
    );
}

#[test]
fn drafted_hole_top_radius_less_than_bottom_errors() {
    let m = Model::new()
        .add(Feature::Box {
            id: "p".into(),
            extents: [lit(4.0), lit(4.0), lit(4.0)],
        })
        .add(Feature::DraftedHole {
            id: "dh".into(),
            input: "p".into(),
            center: [lit(2.0), lit(2.0), lit(4.0)],
            axis: "z".into(),
            top_radius: lit(0.4), // less than bottom — invalid
            bottom_radius: lit(0.6),
            depth: lit(2.0),
            segments: 8,
        });
    assert!(m.evaluate("dh").is_err());
}

// ---------------------------------------------------------------------------
// HexFlange
// ---------------------------------------------------------------------------

#[test]
fn hex_flange_volume_reasonable() {
    // HexFlange uses cylinder_faceted with 6 sides where r = apothem = across_flats/2.
    // cylinder_faceted(r, h, 6) gives a regular hexagon with circumradius=r.
    // Area of such hexagon = (3√3/2) * r².
    // Volume = (3√3/2) * r² * h.
    let af = 2.0; // across_flats → apothem passed as r = af/2 = 1.0
    let h = 3.0;
    let m = Model::new().add(Feature::HexFlange {
        id: "hf".into(),
        center: [lit(0.0), lit(0.0), lit(0.0)],
        axis: "z".into(),
        across_flats: lit(af),
        height: lit(h),
    });
    let s = m.evaluate("hf").unwrap();
    let v = solid_volume(&s);
    let r = af / 2.0; // circumradius passed to cylinder_faceted
    let expected = (3.0 * 3_f64.sqrt() / 2.0) * r * r * h; // ≈ 7.794
    // Faceted (6-sided) matches exactly the analytical hex prism.
    assert!(
        (v - expected).abs() < expected * 0.01,
        "v={v}, expected={expected}"
    );
    // Volume must be positive and reasonable.
    assert!(v > 5.0 && v < 15.0, "hex flange volume={v} out of range");
}

#[test]
fn hex_flange_positive_dims_required() {
    let m = Model::new().add(Feature::HexFlange {
        id: "hf".into(),
        center: [lit(0.0), lit(0.0), lit(0.0)],
        axis: "z".into(),
        across_flats: lit(0.0), // invalid
        height: lit(2.0),
    });
    assert!(m.evaluate("hf").is_err());
}

// ---------------------------------------------------------------------------
// Heatset
// ---------------------------------------------------------------------------

#[test]
fn heatset_reduces_volume_with_lead_in() {
    let m = Model::new()
        .add(Feature::Box {
            id: "body".into(),
            extents: [lit(6.0), lit(6.0), lit(8.0)],
        })
        .add(Feature::Heatset {
            id: "hs".into(),
            input: "body".into(),
            center: [lit(3.0), lit(3.0), lit(8.0)],
            axis: "z".into(),
            insert_radius: lit(0.25),
            insert_depth: lit(4.0),
            lead_in_radius: lit(0.35),
            segments: 12,
        });
    let body_vol = solid_volume(&m.evaluate("body").unwrap());
    let hs_vol = solid_volume(&m.evaluate("hs").unwrap());
    // Heatset socket must remove material.
    assert!(
        hs_vol < body_vol,
        "hs_vol={hs_vol} should be < body_vol={body_vol}"
    );
    // Minimum volume removed ≈ π * insert_radius² * insert_depth.
    let min_removed = std::f64::consts::PI * 0.25 * 0.25 * 4.0;
    let removed = body_vol - hs_vol;
    assert!(
        removed > min_removed * 0.5, // faceted is a bit smaller than analytic
        "removed={removed}, min_removed={min_removed}"
    );
}

#[test]
fn heatset_lead_in_smaller_than_insert_errors() {
    let m = Model::new()
        .add(Feature::Box {
            id: "b".into(),
            extents: [lit(4.0), lit(4.0), lit(6.0)],
        })
        .add(Feature::Heatset {
            id: "hs".into(),
            input: "b".into(),
            center: [lit(2.0), lit(2.0), lit(6.0)],
            axis: "z".into(),
            insert_radius: lit(0.5),
            insert_depth: lit(3.0),
            lead_in_radius: lit(0.3), // less than insert_radius — invalid
            segments: 8,
        });
    assert!(m.evaluate("hs").is_err());
}
