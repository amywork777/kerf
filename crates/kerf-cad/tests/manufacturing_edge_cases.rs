//! Manufacturing edge-case batch: EndChamfer, InternalChamfer,
//! ConicalCounterbore, CrossDrilledHole, TaperedPin, FlangedNut, DowelPin,
//! BlindHole.
//!
//! Each feature gets one volume-bounded test (compared against a
//! decomposition of the expected geometry) plus participates in a
//! consolidated JSON round-trip test.

use std::f64::consts::PI;

use kerf_brep::measure::solid_volume;
use kerf_cad::{lits, Feature, Model, Scalar};

fn lit(x: f64) -> Scalar {
    Scalar::lit(x)
}

/// Faceted-cylinder volume: a regular n-gon prism inscribed in radius r,
/// extruded by height h. (Same formula kerf-brep's cylinder_faceted produces.)
fn faceted_cyl_volume(r: f64, h: f64, n: usize) -> f64 {
    0.5 * (n as f64) * r * r * (2.0 * PI / n as f64).sin() * h
}

/// Faceted-frustum volume: average-of-areas form for a prism whose top
/// and bottom are regular n-gons inscribed in r_bot and r_top. For axially
/// straight side walls between corresponding vertices (the construction
/// `frustum_faceted` uses), the exact volume is
///   h/3 * (A_bot + A_top + sqrt(A_bot * A_top))
/// where A = 0.5 * n * r^2 * sin(2π/n).
fn faceted_frustum_volume(r_bot: f64, r_top: f64, h: f64, n: usize) -> f64 {
    let a_bot = 0.5 * (n as f64) * r_bot * r_bot * (2.0 * PI / n as f64).sin();
    let a_top = 0.5 * (n as f64) * r_top * r_top * (2.0 * PI / n as f64).sin();
    h / 3.0 * (a_bot + a_top + (a_bot * a_top).sqrt())
}

#[test]
fn end_chamfer_z_axis_volume_matches() {
    // Cylinder boss of r=4, h=10. End chamfer at top (z=10) with
    // outer_radius=4 (matches body), chamfer=1.
    //
    // The cutter is a 45° outward-opening cone that clips the body's
    // rim. Removed material = the toroidal corner ring between the
    // chamfer cone surface and the cylindrical side wall, in the band
    // z=[9,10].
    //
    // Volume of corner ring (analytic, smooth-circle limit):
    //   = cylinder_band_vol - chamfered_cone_band_vol
    //   = (π * r_out² * c) - faceted_frustum_volume(r_out, r_out-c, c)
    //
    // For faceted polygons, both volumes use the same n-gon
    // approximation, so removed = poly_cyl(r_out, c) - poly_frust(r_out, r_out-c, c).
    let r_out = 4.0;
    let c = 1.0;
    let body_h = 10.0;
    let segs = 32;
    let m = Model::new()
        .add(Feature::Cylinder {
            id: "body".into(),
            radius: lit(r_out),
            height: lit(body_h),
            segments: segs,
        })
        .add(Feature::EndChamfer {
            id: "out".into(),
            input: "body".into(),
            axis: "z".into(),
            top_center: lits([0.0, 0.0, 10.0]),
            outer_radius: lit(r_out),
            chamfer: lit(c),
            segments: segs,
        });
    let s = m.evaluate("out").unwrap();
    let v = solid_volume(&s);

    let body_vol = faceted_cyl_volume(r_out, body_h, segs);
    let removed = faceted_cyl_volume(r_out, c, segs)
        - faceted_frustum_volume(r_out, r_out - c, c, segs);
    let exp = body_vol - removed;
    let rel = (v - exp).abs() / exp;
    // 2% tolerance: small discretization noise from the cutter cone's
    // n-gon vs the body's n-gon at the chamfer line.
    assert!(rel < 0.02, "v={v}, exp={exp}, rel={rel}");
    assert!(removed > 0.0);
}

#[test]
fn internal_chamfer_volume_matches_frustum() {
    // 10×10×4 plate, internal chamfer at top center (5,5,4) of a
    // notional bored hole of radius 1.5 with chamfer_width=0.5,
    // chamfer_depth=0.5 (45°). The cutter is a frustum from r=1.5 at
    // depth 0.5 to r=2.0 at the surface. Subtracted volume ≈ frustum
    // volume.
    let hr = 1.5;
    let cw = 0.5;
    let cd = 0.5;
    let segs = 32;
    let m = Model::new()
        .add(Feature::Box {
            id: "body".into(),
            extents: lits([10.0, 10.0, 4.0]),
        })
        .add(Feature::InternalChamfer {
            id: "out".into(),
            input: "body".into(),
            axis: "z".into(),
            top_center: lits([5.0, 5.0, 4.0]),
            hole_radius: lit(hr),
            chamfer_width: lit(cw),
            chamfer_depth: lit(cd),
            segments: segs,
        });
    let s = m.evaluate("out").unwrap();
    let v = solid_volume(&s);
    // Removed = frustum(r_bot=hr, r_top=hr+cw, h=cd) approximately.
    let removed_exp = faceted_frustum_volume(hr, hr + cw, cd, segs);
    let exp = 10.0 * 10.0 * 4.0 - removed_exp;
    let rel = (v - exp).abs() / exp;
    // 3% tolerance covers the eps overhang past the top surface.
    assert!(rel < 0.03, "v={v}, exp={exp}, rel={rel}");
}

#[test]
fn conical_counterbore_volume_matches_decomposition() {
    // 20×20×10 plate, conical counterbore at top (10,10,10):
    //   cbore: r=4, depth=2.
    //   drill: r=2, body_depth=8 (the drill body extends 8 below the top).
    //   tip: depth=1 (a cone tapering 2->0 below the drill).
    // Removed = cbore_disk(4, 2) + drill_body(2, 8 - 2) + tip_cone(2->0, 1)
    //         = 32π + 24π + (1/3)*π*4*1 = 56π + (4π/3)
    // Approximated by faceted polygons.
    let segs = 24;
    let cd = 2.0;
    let bd = 8.0;
    let td = 1.0;
    let dr = 2.0;
    let cr = 4.0;
    let m = Model::new()
        .add(Feature::Box {
            id: "body".into(),
            extents: lits([20.0, 20.0, 10.0]),
        })
        .add(Feature::ConicalCounterbore {
            id: "out".into(),
            input: "body".into(),
            axis: "z".into(),
            top_center: lits([10.0, 10.0, 10.0]),
            drill_radius: lit(dr),
            cbore_radius: lit(cr),
            cbore_depth: lit(cd),
            body_depth: lit(bd),
            tip_depth: lit(td),
            segments: segs,
        });
    let s = m.evaluate("out").unwrap();
    let v = solid_volume(&s);
    let removed = faceted_cyl_volume(cr, cd, segs)
        + faceted_cyl_volume(dr, bd - cd, segs)
        + faceted_frustum_volume(0.0, dr, td, segs);
    let exp = 20.0 * 20.0 * 10.0 - removed;
    let rel = (v - exp).abs() / exp;
    // 2% tolerance: small eps overhangs at section boundaries.
    assert!(rel < 0.02, "v={v}, exp={exp}, rel={rel}");
}

#[test]
fn cross_drilled_hole_volume_matches_two_minus_intersection() {
    // 20×20×20 box. Cross-drill through center (10,10,10):
    //   axis_a = x, length_a = 20 (full traversal)
    //   axis_b = y, length_b = 20
    //   radius = 2
    // Two perpendicular cylinders, full diameter intersection at center
    // is a Steinmetz-solid-shape — its volume for two cylinders of
    // radius r intersecting at right angles, both fully traversing, is
    // (16/3)*r^3 (analytic) for circular cross-sections.
    //
    // Because the kernel uses faceted cylinders (n-gon prisms), the
    // intersection is slightly smaller than the analytic Steinmetz; we
    // bound the result to a generous range.
    let segs = 24;
    let r = 2.0;
    let l = 20.0;
    let m = Model::new()
        .add(Feature::Box {
            id: "body".into(),
            extents: lits([20.0, 20.0, 20.0]),
        })
        .add(Feature::CrossDrilledHole {
            id: "out".into(),
            input: "body".into(),
            center: lits([10.0, 10.0, 10.0]),
            axis_a: "x".into(),
            axis_b: "y".into(),
            radius: lit(r),
            length_a: lit(l),
            length_b: lit(l),
            segments: segs,
        });
    let s = m.evaluate("out").unwrap();
    let v = solid_volume(&s);
    // Each cylinder ≈ faceted_cyl(r, l, segs); analytic intersection is
    // (16/3) r^3. removed = 2*cyl - intersection.
    let cyl = faceted_cyl_volume(r, l, segs);
    let intersection = (16.0 / 3.0) * r * r * r;
    let removed_exp = 2.0 * cyl - intersection;
    let exp = 20.0 * 20.0 * 20.0 - removed_exp;
    let rel = (v - exp).abs() / exp;
    // 3% tolerance: faceted vs analytic intersection mismatch.
    assert!(rel < 0.03, "v={v}, exp={exp}, rel={rel}");
}

#[test]
fn tapered_pin_volume_matches_frustum() {
    // r_large=2, r_small=1, length=5, segments=24.
    // Volume = faceted_frustum(2, 1, 5, 24).
    let segs = 24;
    let r_l = 2.0;
    let r_s = 1.0;
    let l = 5.0;
    let m = Model::new().add(Feature::TaperedPin {
        id: "p".into(),
        large_radius: lit(r_l),
        small_radius: lit(r_s),
        length: lit(l),
        segments: segs,
    });
    let s = m.evaluate("p").unwrap();
    let v = solid_volume(&s);
    let exp = faceted_frustum_volume(r_l, r_s, l, segs);
    let rel = (v - exp).abs() / exp;
    assert!(rel < 0.005, "v={v}, exp={exp}, rel={rel}");
}

#[test]
fn flanged_nut_volume_matches_decomposition() {
    // inscribed_radius=1 (across-flats=2). Hex circumradius =
    // 1 / cos(π/6) ≈ 1.1547. flange_radius=2, ft=0.5, nt=1.0,
    // bore=0.4, segs=24.
    // Volume ≈ flange_disk + hex_prism - bore_through_both - small overlap.
    let ir = 1.0;
    let fr = 2.0;
    let ft = 0.5;
    let nt = 1.0;
    let br = 0.4;
    let segs = 24;
    let m = Model::new().add(Feature::FlangedNut {
        id: "n".into(),
        inscribed_radius: lit(ir),
        flange_radius: lit(fr),
        flange_thickness: lit(ft),
        nut_thickness: lit(nt),
        bore_radius: lit(br),
        segments: segs,
    });
    let s = m.evaluate("n").unwrap();
    let v = solid_volume(&s);
    let hex_circ = ir / (PI / 6.0).cos();
    // Hex has 6 segments. Hex area = 0.5 * 6 * r² * sin(60°).
    let flange_v = faceted_cyl_volume(fr, ft, segs);
    let hex_v = faceted_cyl_volume(hex_circ, nt, 6);
    let bore_v = faceted_cyl_volume(br, ft + nt, segs);
    let exp = flange_v + hex_v - bore_v;
    let rel = (v - exp).abs() / exp;
    // 2% tolerance: small overlap at flange/hex interface.
    assert!(rel < 0.02, "v={v}, exp={exp}, rel={rel}");
}

#[test]
fn dowel_pin_volume_matches_decomposition() {
    // radius=1, length=6, chamfer=0.2, segments=24.
    // Volume = bot_frustum(0.8, 1, 0.2) + body_cyl(1, 5.6) + top_frustum(1, 0.8, 0.2).
    let r = 1.0;
    let l = 6.0;
    let c = 0.2;
    let segs = 24;
    let m = Model::new().add(Feature::DowelPin {
        id: "d".into(),
        radius: lit(r),
        length: lit(l),
        chamfer: lit(c),
        segments: segs,
    });
    let s = m.evaluate("d").unwrap();
    let v = solid_volume(&s);
    let bot_v = faceted_frustum_volume(r - c, r, c, segs);
    let body_v = faceted_cyl_volume(r, l - 2.0 * c, segs);
    let top_v = faceted_frustum_volume(r, r - c, c, segs);
    let exp = bot_v + body_v + top_v;
    let rel = (v - exp).abs() / exp;
    // 2% tolerance for eps overlaps at the two seams.
    assert!(rel < 0.02, "v={v}, exp={exp}, rel={rel}");
}

#[test]
fn blind_hole_volume_matches_cylinder() {
    // 10×10×5 plate, blind hole at top center (5,5,5): r=1, depth=3.
    // Removed = faceted_cyl(1, 3, 24).
    let r = 1.0;
    let d = 3.0;
    let segs = 24;
    let m = Model::new()
        .add(Feature::Box {
            id: "body".into(),
            extents: lits([10.0, 10.0, 5.0]),
        })
        .add(Feature::BlindHole {
            id: "out".into(),
            input: "body".into(),
            axis: "z".into(),
            top_center: lits([5.0, 5.0, 5.0]),
            radius: lit(r),
            depth: lit(d),
            segments: segs,
        });
    let s = m.evaluate("out").unwrap();
    let v = solid_volume(&s);
    let removed = faceted_cyl_volume(r, d, segs);
    let exp = 10.0 * 10.0 * 5.0 - removed;
    let rel = (v - exp).abs() / exp;
    assert!(rel < 0.01, "v={v}, exp={exp}, rel={rel}");
}

// ---------- Validation: each feature rejects bad inputs ----------

#[test]
fn end_chamfer_rejects_chamfer_geq_radius() {
    let m = Model::new()
        .add(Feature::Box {
            id: "body".into(),
            extents: lits([10.0, 10.0, 5.0]),
        })
        .add(Feature::EndChamfer {
            id: "out".into(),
            input: "body".into(),
            axis: "z".into(),
            top_center: lits([5.0, 5.0, 5.0]),
            outer_radius: lit(1.0),
            chamfer: lit(2.0),
            segments: 16,
        });
    assert!(m.evaluate("out").is_err());
}

#[test]
fn cross_drilled_hole_rejects_same_axes() {
    let m = Model::new()
        .add(Feature::Box {
            id: "body".into(),
            extents: lits([10.0, 10.0, 10.0]),
        })
        .add(Feature::CrossDrilledHole {
            id: "out".into(),
            input: "body".into(),
            center: lits([5.0, 5.0, 5.0]),
            axis_a: "x".into(),
            axis_b: "x".into(),
            radius: lit(1.0),
            length_a: lit(10.0),
            length_b: lit(10.0),
            segments: 16,
        });
    assert!(m.evaluate("out").is_err());
}

#[test]
fn tapered_pin_rejects_inverted_radii() {
    let m = Model::new().add(Feature::TaperedPin {
        id: "p".into(),
        large_radius: lit(1.0),
        small_radius: lit(2.0),
        length: lit(5.0),
        segments: 16,
    });
    assert!(m.evaluate("p").is_err());
}

#[test]
fn dowel_pin_rejects_chamfer_too_long() {
    let m = Model::new().add(Feature::DowelPin {
        id: "d".into(),
        radius: lit(1.0),
        length: lit(0.3),
        chamfer: lit(0.2),
        segments: 16,
    });
    // 2 * 0.2 = 0.4 >= 0.3 → reject.
    assert!(m.evaluate("d").is_err());
}

// ---------- Consolidated JSON round-trip ----------

#[test]
fn manufacturing_edge_cases_round_trip_via_json() {
    let m = Model::new()
        .add(Feature::Box {
            id: "body".into(),
            extents: lits([20.0, 20.0, 10.0]),
        })
        .add(Feature::EndChamfer {
            id: "ec".into(),
            input: "body".into(),
            axis: "z".into(),
            top_center: lits([10.0, 10.0, 10.0]),
            outer_radius: lit(4.0),
            chamfer: lit(1.0),
            segments: 24,
        })
        .add(Feature::InternalChamfer {
            id: "ic".into(),
            input: "ec".into(),
            axis: "z".into(),
            top_center: lits([10.0, 10.0, 10.0]),
            hole_radius: lit(1.5),
            chamfer_width: lit(0.5),
            chamfer_depth: lit(0.5),
            segments: 24,
        })
        .add(Feature::ConicalCounterbore {
            id: "ccb".into(),
            input: "body".into(),
            axis: "z".into(),
            top_center: lits([10.0, 10.0, 10.0]),
            drill_radius: lit(2.0),
            cbore_radius: lit(4.0),
            cbore_depth: lit(2.0),
            body_depth: lit(8.0),
            tip_depth: lit(1.0),
            segments: 24,
        })
        .add(Feature::CrossDrilledHole {
            id: "cdh".into(),
            input: "body".into(),
            center: lits([10.0, 10.0, 5.0]),
            axis_a: "x".into(),
            axis_b: "y".into(),
            radius: lit(1.5),
            length_a: lit(20.0),
            length_b: lit(20.0),
            segments: 24,
        })
        .add(Feature::TaperedPin {
            id: "tp".into(),
            large_radius: lit(2.0),
            small_radius: lit(1.0),
            length: lit(5.0),
            segments: 24,
        })
        .add(Feature::FlangedNut {
            id: "fn".into(),
            inscribed_radius: lit(1.0),
            flange_radius: lit(2.0),
            flange_thickness: lit(0.5),
            nut_thickness: lit(1.0),
            bore_radius: lit(0.4),
            segments: 24,
        })
        .add(Feature::DowelPin {
            id: "dp".into(),
            radius: lit(1.0),
            length: lit(6.0),
            chamfer: lit(0.2),
            segments: 24,
        })
        .add(Feature::BlindHole {
            id: "bh".into(),
            input: "body".into(),
            axis: "z".into(),
            top_center: lits([10.0, 10.0, 10.0]),
            radius: lit(1.0),
            depth: lit(3.0),
            segments: 24,
        });
    let json = m.to_json_string().unwrap();
    let parsed = Model::from_json_str(&json).unwrap();
    assert_eq!(json, parsed.to_json_string().unwrap());
}
