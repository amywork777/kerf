//! Real-mesh stress test: tessellate primitives at high resolution, write
//! to STL, re-import, run booleans. The imported solid has many small
//! triangles instead of analytic faces — the boolean engine has to handle:
//!
//! - Hundreds of coplanar triangles per "logical" face (because tessellation
//!   subdivides) — chord intersections fan out across many faces instead of
//!   one.
//! - Floating-point coordinates from sin/cos that aren't exactly aligned to
//!   integers — tolerance handling matters more.
//! - High face count → more chord intersections → exercises Phase B fixpoint
//!   harder.
//!
//! Tests are organized failure-first: each one will *probably* fail today;
//! once they pass they regression-protect the next class of bugs.

use std::io::Cursor;

use kerf_brep::booleans::BooleanOp;
use kerf_brep::primitives::{box_, box_at, cylinder_faceted};
use kerf_brep::{
    read_stl_binary_to_solid, solid_volume, tessellate, write_binary, BooleanError, Solid,
};
use kerf_geom::{Point3, Vec3};

const VOL_TOL: f64 = 1e-3; // permissive for tessellation noise

fn try_op(a: &Solid, b: &Solid, op: BooleanOp) -> Result<Solid, BooleanError> {
    match op {
        BooleanOp::Union => a.try_union(b),
        BooleanOp::Intersection => a.try_intersection(b),
        BooleanOp::Difference => a.try_difference(b),
    }
}

/// Tessellate `s` at the given segment count, write to binary STL, read back.
fn stl_roundtrip(s: &Solid, segments: usize, label: &str) -> Solid {
    let soup = tessellate(s, segments);
    let mut buf = Vec::new();
    write_binary(&soup, label, &mut buf).expect("write_binary");
    let mut reader = Cursor::new(buf);
    read_stl_binary_to_solid(&mut reader).expect("read_stl_binary_to_solid")
}

// ============================================================================
// Imported × imported
// ============================================================================

#[test]
fn imported_box_diff_imported_box_works() {
    // Tessellate two boxes at coarse resolution, re-import, then DIFF.
    let a_orig = box_(Vec3::new(2.0, 2.0, 2.0));
    let b_orig = box_at(Vec3::new(2.0, 2.0, 2.0), Point3::new(1.0, 0.0, 0.0));

    let a = stl_roundtrip(&a_orig, 8, "a");
    let b = stl_roundtrip(&b_orig, 8, "b");

    // Sanity: volumes preserved through STL.
    assert!((solid_volume(&a) - 8.0).abs() < VOL_TOL);
    assert!((solid_volume(&b) - 8.0).abs() < VOL_TOL);

    let r = try_op(&a, &b, BooleanOp::Difference)
        .expect("imported A − imported B should work");
    let v = solid_volume(&r);
    // Expected: 8 - 4 = 4
    assert!(
        (v - 4.0).abs() < VOL_TOL,
        "imported - imported diff vol = {v}, expected 4"
    );
}

#[test]
fn imported_box_union_imported_box_works() {
    let a_orig = box_(Vec3::new(2.0, 2.0, 2.0));
    let b_orig = box_at(Vec3::new(2.0, 2.0, 2.0), Point3::new(1.0, 0.0, 0.0));
    let a = stl_roundtrip(&a_orig, 8, "a");
    let b = stl_roundtrip(&b_orig, 8, "b");
    let r = try_op(&a, &b, BooleanOp::Union)
        .expect("imported A ∪ imported B should work");
    assert!(
        (solid_volume(&r) - 12.0).abs() < VOL_TOL,
        "got {}",
        solid_volume(&r)
    );
}

#[test]
fn imported_box_intersect_imported_box_works() {
    let a_orig = box_(Vec3::new(2.0, 2.0, 2.0));
    let b_orig = box_at(Vec3::new(2.0, 2.0, 2.0), Point3::new(1.0, 0.0, 0.0));
    let a = stl_roundtrip(&a_orig, 8, "a");
    let b = stl_roundtrip(&b_orig, 8, "b");
    let r = try_op(&a, &b, BooleanOp::Intersection)
        .expect("imported A ∩ imported B should work");
    assert!(
        (solid_volume(&r) - 4.0).abs() < VOL_TOL,
        "got {}",
        solid_volume(&r)
    );
}

// ============================================================================
// Imported × primitive
// ============================================================================

#[test]
fn imported_box_diff_primitive_box_works() {
    let a_orig = box_(Vec3::new(2.0, 2.0, 2.0));
    let b = box_at(Vec3::new(2.0, 2.0, 2.0), Point3::new(1.0, 0.0, 0.0));
    let a = stl_roundtrip(&a_orig, 8, "a");
    let r = try_op(&a, &b, BooleanOp::Difference).expect("imported A − primitive B");
    assert!(
        (solid_volume(&r) - 4.0).abs() < VOL_TOL,
        "got {}",
        solid_volume(&r)
    );
}

#[test]
fn primitive_box_diff_imported_box_works() {
    let a = box_(Vec3::new(2.0, 2.0, 2.0));
    let b_orig = box_at(Vec3::new(2.0, 2.0, 2.0), Point3::new(1.0, 0.0, 0.0));
    let b = stl_roundtrip(&b_orig, 8, "b");
    let r = try_op(&a, &b, BooleanOp::Difference).expect("primitive A − imported B");
    assert!(
        (solid_volume(&r) - 4.0).abs() < VOL_TOL,
        "got {}",
        solid_volume(&r)
    );
}

// ============================================================================
// Higher-resolution: cylinder
// ============================================================================

#[test]
fn imported_cylinder_diff_box_works() {
    // Cylinder pierces a box (the canonical M40-fixed case, but now from a
    // re-imported tessellated cylinder rather than the analytic primitive).
    let cyl_orig = cylinder_faceted(0.6, 3.0, 12);
    let block = box_(Vec3::new(2.0, 2.0, 2.0));

    let cyl = stl_roundtrip(&cyl_orig, 12, "cyl");
    // Expected vol: same as cyl_n12 vol.
    let expected_cyl = solid_volume(&cyl_orig);
    assert!(
        (solid_volume(&cyl) - expected_cyl).abs() < VOL_TOL,
        "imported cyl vol drift"
    );

    // Block − cyl: block volume (8) minus cylinder-inside-block portion.
    // Cylinder spans z ∈ [0, 3]; block z ∈ [0, 2]; overlap z ∈ [0, 2].
    // Cylinder cross-section: radius 0.6 quadrant (mostly-inside-block).
    // Faceted volume above block (z ∈ [0, 2] of cylinder): expected_cyl * 2/3.
    // Inside block: portion of faceted-cyl xy-area within [0,2]×[0,2]. Since
    // cylinder is centered at origin and box is at +xy, only the (+x, +y)
    // quarter of the cylinder is inside the box.
    let cyl_inside_block = expected_cyl * (2.0 / 3.0) * 0.25;
    let r = try_op(&block, &cyl, BooleanOp::Difference).expect("block − imported cyl");
    let v = solid_volume(&r);
    let expected = 8.0 - cyl_inside_block;
    assert!(
        (v - expected).abs() < VOL_TOL,
        "block − imported cyl vol = {v}, expected {expected} (cyl inside block: {cyl_inside_block})"
    );
}

// ============================================================================
// Self-boolean on imported mesh (idempotence under noisy floats)
// ============================================================================

#[test]
fn imported_self_union_is_self() {
    let orig = box_(Vec3::new(2.0, 2.0, 2.0));
    let imported = stl_roundtrip(&orig, 8, "self");
    let r = try_op(&imported, &imported, BooleanOp::Union)
        .expect("imported A ∪ imported A");
    assert!(
        (solid_volume(&r) - 8.0).abs() < VOL_TOL,
        "got {}",
        solid_volume(&r)
    );
}

#[test]
fn imported_self_intersection_is_self() {
    let orig = box_(Vec3::new(2.0, 2.0, 2.0));
    let imported = stl_roundtrip(&orig, 8, "self");
    let r = try_op(&imported, &imported, BooleanOp::Intersection)
        .expect("imported A ∩ imported A");
    assert!(
        (solid_volume(&r) - 8.0).abs() < VOL_TOL,
        "got {}",
        solid_volume(&r)
    );
}

#[test]
fn imported_self_difference_is_empty() {
    let orig = box_(Vec3::new(2.0, 2.0, 2.0));
    let imported = stl_roundtrip(&orig, 8, "self");
    let r = try_op(&imported, &imported, BooleanOp::Difference)
        .expect("imported A − imported A");
    assert!(
        solid_volume(&r).abs() < VOL_TOL,
        "got {} (expected ~0)",
        solid_volume(&r)
    );
}

// ============================================================================
// High-triangle-count input
// ============================================================================

#[test]
fn imported_high_resolution_cylinder_diff_box_works() {
    // 64-segment cylinder = 64 lateral trapezoids + 64-fan top + 64-fan bottom
    // = ~192 triangles after tessellation. Re-imported it has ~192 faces.
    // Boolean against a box exercises many chord intersections.
    let cyl_orig = cylinder_faceted(0.6, 3.0, 64);
    let block = box_(Vec3::new(2.0, 2.0, 2.0));

    let cyl = stl_roundtrip(&cyl_orig, 64, "cyl64");
    let expected_cyl = solid_volume(&cyl_orig);
    assert!((solid_volume(&cyl) - expected_cyl).abs() < VOL_TOL);

    let r = try_op(&block, &cyl, BooleanOp::Difference)
        .expect("block − 64-segment imported cyl");
    let cyl_inside_block = expected_cyl * (2.0 / 3.0) * 0.25;
    let expected = 8.0 - cyl_inside_block;
    assert!(
        (solid_volume(&r) - expected).abs() < VOL_TOL,
        "got {}, expected {}",
        solid_volume(&r),
        expected
    );
}

// ============================================================================
// Pushed harder: cyl × cyl, mixed-resolution, all 3 ops
// ============================================================================

#[test]
fn imported_cylinder_intersect_box_works() {
    let cyl_orig = cylinder_faceted(0.6, 3.0, 12);
    let block = box_(Vec3::new(2.0, 2.0, 2.0));
    let cyl = stl_roundtrip(&cyl_orig, 12, "cyl");
    let r = try_op(&block, &cyl, BooleanOp::Intersection)
        .expect("block ∩ imported cyl");
    let expected_cyl = solid_volume(&cyl_orig);
    let cyl_inside_block = expected_cyl * (2.0 / 3.0) * 0.25;
    assert!(
        (solid_volume(&r) - cyl_inside_block).abs() < VOL_TOL,
        "got {}, expected {}",
        solid_volume(&r),
        cyl_inside_block
    );
}

#[test]
fn imported_cylinder_union_box_works() {
    let cyl_orig = cylinder_faceted(0.6, 3.0, 12);
    let block = box_(Vec3::new(2.0, 2.0, 2.0));
    let cyl = stl_roundtrip(&cyl_orig, 12, "cyl");
    let r = try_op(&block, &cyl, BooleanOp::Union).expect("block ∪ imported cyl");
    let expected_cyl = solid_volume(&cyl_orig);
    let cyl_inside_block = expected_cyl * (2.0 / 3.0) * 0.25;
    let expected = 8.0 + expected_cyl - cyl_inside_block;
    assert!(
        (solid_volume(&r) - expected).abs() < VOL_TOL,
        "got {}, expected {}",
        solid_volume(&r),
        expected
    );
}

#[test]
fn imported_cyl_diff_imported_cyl_works() {
    // Two coaxial imported cylinders (same axis, different lengths). Diff
    // should give an annular shell.
    let outer_orig = cylinder_faceted(0.8, 3.0, 16);
    let inner_orig = cylinder_faceted(0.4, 3.0, 16);
    let outer = stl_roundtrip(&outer_orig, 16, "outer");
    let inner = stl_roundtrip(&inner_orig, 16, "inner");
    let r = try_op(&outer, &inner, BooleanOp::Difference)
        .expect("imported outer cyl − imported inner cyl");
    // Faceted: outer vol - inner vol (inner is fully inside outer in x,y;
    // they share the same z range so DIFF carves a hollow tube).
    let expected = solid_volume(&outer_orig) - solid_volume(&inner_orig);
    assert!(
        (solid_volume(&r) - expected).abs() < VOL_TOL,
        "annular tube vol {}, expected {}",
        solid_volume(&r),
        expected
    );
}

#[test]
fn mixed_resolution_imported_diff_works() {
    // 8-segment imported × 32-segment imported. Different resolutions exercise
    // tolerance / dedup pathways since their vertices won't coincide.
    let cyl8_orig = cylinder_faceted(0.6, 3.0, 8);
    let cyl32_orig = cylinder_faceted(0.6, 3.0, 32);
    let cyl8 = stl_roundtrip(&cyl8_orig, 8, "c8");
    let cyl32 = stl_roundtrip(&cyl32_orig, 32, "c32");

    // The two cylinders are NOT identical (different facet count → different
    // vertex positions), but they share an axis. DIFF should be a thin sliver.
    // Volume should be |vol(cyl8) − vol(cyl32 ∩ cyl8)|, which is bounded by
    // |vol(cyl8) − vol(cyl32)|.
    let r = try_op(&cyl8, &cyl32, BooleanOp::Difference)
        .expect("8-seg − 32-seg imported cylinders");
    let v = solid_volume(&r);
    // Sanity: vol must be ≤ vol(cyl8) and ≥ 0.
    let v8 = solid_volume(&cyl8_orig);
    assert!(v >= -VOL_TOL, "vol {v} < 0");
    assert!(v <= v8 + VOL_TOL, "vol {v} > vol(cyl8) = {v8}");
}

// ============================================================================
// Translated / non-axis-aligned imports
// ============================================================================

#[test]
fn imported_translated_box_diff_works() {
    let a_orig = box_at(Vec3::new(2.0, 2.0, 2.0), Point3::new(10.0, -7.5, 3.25));
    let b_orig = box_at(Vec3::new(2.0, 2.0, 2.0), Point3::new(11.0, -7.5, 3.25));
    let a = stl_roundtrip(&a_orig, 8, "a");
    let b = stl_roundtrip(&b_orig, 8, "b");
    let r = try_op(&a, &b, BooleanOp::Difference).expect("translated diff");
    assert!(
        (solid_volume(&r) - 4.0).abs() < VOL_TOL,
        "got {}",
        solid_volume(&r)
    );
}

// ============================================================================
// Volume conservation: mesh-derived booleans must satisfy inclusion-exclusion
// ============================================================================

#[test]
fn imported_pair_satisfies_inclusion_exclusion() {
    // For any two solids: vol(A∪B) = vol(A) + vol(B) - vol(A∩B).
    // Verify on imported meshes — same identity, harder pipeline.
    let a_orig = box_(Vec3::new(2.0, 2.0, 2.0));
    let b_orig = box_at(Vec3::new(2.0, 2.0, 2.0), Point3::new(1.0, 0.5, 0.0));
    let a = stl_roundtrip(&a_orig, 8, "a");
    let b = stl_roundtrip(&b_orig, 8, "b");
    let v_a = solid_volume(&a);
    let v_b = solid_volume(&b);
    let v_u = solid_volume(&try_op(&a, &b, BooleanOp::Union).expect("U"));
    let v_i = solid_volume(&try_op(&a, &b, BooleanOp::Intersection).expect("I"));
    let lhs = v_u;
    let rhs = v_a + v_b - v_i;
    assert!(
        (lhs - rhs).abs() < VOL_TOL,
        "imported pair: vol(A∪B)={v_u}, vol(A)+vol(B)-vol(A∩B)={rhs}"
    );
}

// ============================================================================
// Recursive on imported: (imported A − imported B) ∪ primitive C
// ============================================================================

#[test]
fn recursive_boolean_on_imported_diff_result_works() {
    let a_orig = box_(Vec3::new(2.0, 2.0, 2.0));
    let b_orig = box_at(Vec3::new(2.0, 2.0, 2.0), Point3::new(1.0, 0.0, 0.0));
    let c = box_at(Vec3::new(2.0, 2.0, 2.0), Point3::new(0.5, 0.5, 0.5));

    let a = stl_roundtrip(&a_orig, 8, "a");
    let b = stl_roundtrip(&b_orig, 8, "b");
    let amb = try_op(&a, &b, BooleanOp::Difference).expect("imported A - imported B");
    // amb should be [0,1]×[0,2]² = vol 4.
    assert!((solid_volume(&amb) - 4.0).abs() < VOL_TOL);

    // Re-stitched amb fed back into a boolean against primitive C.
    let r = try_op(&amb, &c, BooleanOp::Union).expect("(A-B) ∪ C");
    // (A−B) is [0,1]×[0,2]². C is [.5,2.5]³. Overlap = [.5,1]×[.5,2]×[.5,2] = .5*1.5*1.5 = 1.125.
    // Total = 4 + 8 - 1.125 = 10.875.
    assert!(
        (solid_volume(&r) - 10.875).abs() < VOL_TOL,
        "got {}",
        solid_volume(&r)
    );
}

// ============================================================================
// Pathological geometries
// ============================================================================

#[test]
fn imported_long_thin_box_diff_works() {
    // 100×0.5×0.5 imported, then DIFF half away.
    let a_orig = box_(Vec3::new(100.0, 0.5, 0.5));
    let b_orig = box_at(Vec3::new(100.0, 0.5, 0.5), Point3::new(50.0, 0.0, 0.0));
    let a = stl_roundtrip(&a_orig, 8, "a");
    let b = stl_roundtrip(&b_orig, 8, "b");
    let r = try_op(&a, &b, BooleanOp::Difference).expect("long thin diff");
    // a vol = 25; overlap = 50*0.5*0.5 = 12.5; result = 12.5.
    assert!(
        (solid_volume(&r) - 12.5).abs() < VOL_TOL,
        "got {}",
        solid_volume(&r)
    );
}

#[test]
fn imported_tiny_box_diff_huge_box_works() {
    // 0.001-side box vs 100-side box — many orders of magnitude difference.
    // Numerical pipeline must handle the disparate scales.
    let tiny = box_at(Vec3::new(0.001, 0.001, 0.001), Point3::new(0.0, 0.0, 0.0));
    let huge = box_(Vec3::new(100.0, 100.0, 100.0));
    let tiny_imp = stl_roundtrip(&tiny, 8, "tiny");
    let huge_imp = stl_roundtrip(&huge, 8, "huge");
    let r = try_op(&huge_imp, &tiny_imp, BooleanOp::Difference)
        .expect("huge - tiny");
    // 100³ - 0.001³ = 1e6 - 1e-9 ≈ 1e6.
    let v = solid_volume(&r);
    let expected = 1e6 - 1e-9;
    assert!(
        (v - expected).abs() < 1e-3,
        "tiny-vs-huge diff vol={v}, expected {expected}"
    );
}

// ============================================================================
// Deep recursion: 4 imported booleans in sequence
// ============================================================================

#[test]
fn deep_imported_boolean_chain_terminates() {
    // Carve 4 successive holes out of a big box using imported cutters.
    let block_orig = box_(Vec3::new(10.0, 10.0, 10.0));
    let mut acc = stl_roundtrip(&block_orig, 8, "block");
    let cutters = [
        box_at(Vec3::new(1.0, 1.0, 1.0), Point3::new(2.0, 2.0, 2.0)),
        box_at(Vec3::new(1.0, 1.0, 1.0), Point3::new(5.0, 5.0, 5.0)),
        box_at(Vec3::new(1.0, 1.0, 1.0), Point3::new(7.0, 7.0, 7.0)),
        box_at(Vec3::new(1.0, 1.0, 1.0), Point3::new(2.0, 7.0, 5.0)),
    ];
    for (i, cutter) in cutters.iter().enumerate() {
        let cutter_imp = stl_roundtrip(cutter, 8, &format!("cut{i}"));
        acc = try_op(&acc, &cutter_imp, BooleanOp::Difference)
            .unwrap_or_else(|e| panic!("step {i}: {}", e.message));
        kerf_topo::validate(&acc.topo).expect("step topo");
    }
    // 4 unit-box cavities carved out of a 1000-vol box.
    let v = solid_volume(&acc);
    assert!((v - 996.0).abs() < VOL_TOL, "after 4 carves vol={v}, expected 996");
    // 1 outer + 4 cavity shells = 5 shells.
    assert_eq!(acc.topo.shell_count(), 5);
}

// ============================================================================
// Non-mesh primitive ↔ imported with translation
// ============================================================================

#[test]
fn imported_at_offset_with_primitive_at_origin_works() {
    // Imported a translated box; combine with primitive at origin.
    let a_orig = box_at(Vec3::new(2.0, 2.0, 2.0), Point3::new(50.0, 0.0, 0.0));
    let a = stl_roundtrip(&a_orig, 8, "a");
    let b = box_at(Vec3::new(2.0, 2.0, 2.0), Point3::new(51.0, 0.0, 0.0));
    let r = try_op(&a, &b, BooleanOp::Union).expect("translated imported ∪ primitive");
    // 8 + 8 - 4 (1×2×2 overlap) = 12.
    assert!(
        (solid_volume(&r) - 12.0).abs() < VOL_TOL,
        "got {}",
        solid_volume(&r)
    );
}

// ============================================================================
// Many small carves stress test
// ============================================================================

#[test]
fn carve_grid_of_small_boxes_completes() {
    // Carve a 3×3 grid of 9 small boxes out of a 10×10×10 block.
    let block_orig = box_(Vec3::new(10.0, 10.0, 10.0));
    let mut acc = block_orig.clone();
    let mut total_carved = 0.0;
    for i in 0..3 {
        for j in 0..3 {
            let cutter = box_at(
                Vec3::new(0.5, 0.5, 0.5),
                Point3::new(2.0 + 3.0 * i as f64, 2.0 + 3.0 * j as f64, 4.75),
            );
            acc = acc
                .try_difference(&cutter)
                .unwrap_or_else(|e| panic!("carve ({i},{j}): {}", e.message));
            total_carved += 0.125;
        }
    }
    let v = solid_volume(&acc);
    let expected = 1000.0 - total_carved;
    assert!(
        (v - expected).abs() < VOL_TOL,
        "after 9 carves vol={v}, expected {expected}"
    );
    // 1 outer + 9 cavity shells.
    assert_eq!(acc.topo.shell_count(), 10);
}
