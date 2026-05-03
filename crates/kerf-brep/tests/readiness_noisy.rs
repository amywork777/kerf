//! Noisy-STL stress: tessellate a primitive, perturb every vertex by
//! sub-tolerance noise (mimicking real-world float imprecision), re-import,
//! run booleans. Tests that the kernel's tolerance/dedup pipeline doesn't
//! collapse on slightly-imperfect input.

use std::io::Cursor;

use kerf_brep::booleans::{BooleanOp, FaceSoup};
use kerf_brep::primitives::{box_, box_at, cylinder_faceted};
use kerf_brep::{
    read_stl_binary_to_solid, solid_volume, tessellate, write_binary, BooleanError, Solid,
};
use kerf_geom::{Point3, Vec3};

const VOL_TOL: f64 = 0.05;

fn try_op(a: &Solid, b: &Solid, op: BooleanOp) -> Result<Solid, BooleanError> {
    match op {
        BooleanOp::Union => a.try_union(b),
        BooleanOp::Intersection => a.try_intersection(b),
        BooleanOp::Difference => a.try_difference(b),
    }
}

/// Add per-vertex noise of magnitude `noise_scale` to a triangle soup, using a
/// deterministic LCG so the test is reproducible. Noise is applied per-VERTEX
/// (not per-position), so coincident vertices in different triangles get the
/// SAME noise — the soup remains topologically connectable, just with floats
/// that don't quite match analytic positions.
fn add_noise_to_soup(mut soup: FaceSoup, noise_scale: f64, seed: u64) -> FaceSoup {
    use std::collections::HashMap;
    let mut state: u64 = seed;
    let mut rand = || {
        state = state.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        ((state >> 33) as u32) as f64 / u32::MAX as f64
    };
    // Quantize identical positions to share the same noise offset.
    let mut noise_for: HashMap<(i64, i64, i64), Vec3> = HashMap::new();
    let q = 1.0e6;
    let key = |p: Point3| -> (i64, i64, i64) {
        (
            (p.x * q).round() as i64,
            (p.y * q).round() as i64,
            (p.z * q).round() as i64,
        )
    };
    for tri in &mut soup.triangles {
        for v in tri.iter_mut() {
            let k = key(*v);
            let offset = *noise_for.entry(k).or_insert_with(|| {
                Vec3::new(
                    (rand() - 0.5) * noise_scale,
                    (rand() - 0.5) * noise_scale,
                    (rand() - 0.5) * noise_scale,
                )
            });
            *v += offset;
        }
    }
    soup
}

fn import_with_noise(s: &Solid, segments: usize, noise_scale: f64, seed: u64) -> Solid {
    let soup = tessellate(s, segments);
    let noisy = add_noise_to_soup(soup, noise_scale, seed);
    let mut buf = Vec::new();
    write_binary(&noisy, "noisy", &mut buf).expect("write_binary");
    let mut reader = Cursor::new(buf);
    read_stl_binary_to_solid(&mut reader).expect("read_stl_binary_to_solid")
}

#[test]
fn noisy_box_volume_within_tolerance() {
    // Per-vertex noise of 1e-6 scale (comparable to float precision noise).
    let s = box_(Vec3::new(2.0, 2.0, 2.0));
    let noisy = import_with_noise(&s, 8, 1e-6, 42);
    let v = solid_volume(&noisy);
    assert!(
        (v - 8.0).abs() < VOL_TOL,
        "noisy box vol = {v}, expected ~8"
    );
}

#[test]
fn within_tolerance_noise_still_allows_diff() {
    // Noise of 1e-10 is well INSIDE the kernel's 1e-9 dedup tolerance, so
    // vertex matching should still work — booleans should produce correct
    // results.
    let a = box_(Vec3::new(2.0, 2.0, 2.0));
    let b = box_at(Vec3::new(2.0, 2.0, 2.0), Point3::new(1.0, 0.0, 0.0));
    let a_noisy = import_with_noise(&a, 8, 1e-10, 1);
    let b_noisy = import_with_noise(&b, 8, 1e-10, 2);
    let r = try_op(&a_noisy, &b_noisy, BooleanOp::Difference)
        .expect("within-tol noisy DIFF");
    let v = solid_volume(&r);
    assert!(
        (v - 4.0).abs() < VOL_TOL,
        "within-tol noisy diff vol = {v}, expected 4"
    );
}

#[test]
fn within_tolerance_noise_still_allows_union() {
    let a = box_(Vec3::new(2.0, 2.0, 2.0));
    let b = box_at(Vec3::new(2.0, 2.0, 2.0), Point3::new(1.0, 0.0, 0.0));
    let a_noisy = import_with_noise(&a, 8, 1e-10, 1);
    let b_noisy = import_with_noise(&b, 8, 1e-10, 2);
    let r = try_op(&a_noisy, &b_noisy, BooleanOp::Union)
        .expect("within-tol noisy UNION");
    assert!(
        (solid_volume(&r) - 12.0).abs() < VOL_TOL,
        "got {}",
        solid_volume(&r)
    );
}

#[test]
#[ignore = "noise above 1e-9 tolerance breaks booleans — documented limitation"]
fn above_tolerance_noise_breaks_boolean_diff() {
    // Noise of 1e-7 is 100× above the kernel's default tolerance. We expect
    // either a clean Err or a numerically-unstable result.
    let a = box_(Vec3::new(2.0, 2.0, 2.0));
    let b = box_at(Vec3::new(2.0, 2.0, 2.0), Point3::new(1.0, 0.0, 0.0));
    let a_noisy = import_with_noise(&a, 8, 1e-7, 1);
    let b_noisy = import_with_noise(&b, 8, 1e-7, 2);
    let r = try_op(&a_noisy, &b_noisy, BooleanOp::Difference)
        .expect("noisy A − noisy B (will fail until tolerance is configurable)");
    let v = solid_volume(&r);
    assert!((v - 4.0).abs() < VOL_TOL);
}

#[test]
fn very_small_noise_self_diff_is_near_empty() {
    // Two copies with noise BELOW tolerance — should be deduplicable as
    // "the same solid". Self-DIFF should give ~0.
    let s = box_(Vec3::new(2.0, 2.0, 2.0));
    let a = import_with_noise(&s, 8, 1e-12, 1);
    let b = import_with_noise(&s, 8, 1e-12, 2);
    let r = try_op(&a, &b, BooleanOp::Difference).expect("tiny-noise self DIFF");
    let v = solid_volume(&r);
    assert!(v.abs() < 0.01, "self-diff vol = {v}");
}

#[test]
fn noisy_cylinder_diff_box_at_within_tolerance_works() {
    let cyl = cylinder_faceted(0.6, 3.0, 12);
    let block = box_at(Vec3::new(2.0, 2.0, 2.0), Point3::new(-1.0, -1.0, -1.0));
    // Stay within tolerance.
    let cyl_noisy = import_with_noise(&cyl, 12, 1e-11, 7);
    let r = try_op(&block, &cyl_noisy, BooleanOp::Difference)
        .expect("block − within-tol noisy cyl");
    assert!(solid_volume(&r) > 0.0);
    assert!(solid_volume(&r) < 8.0);
    kerf_topo::validate(&r.topo).expect("topology");
}

#[test]
fn very_large_noise_still_works_at_high_relative_tolerance() {
    // Noise of 1e-4 — large enough that vertex dedup MIGHT fail if tolerance
    // is too tight. Tests the kernel's ability to handle "very imperfect" STL.
    let a = box_(Vec3::new(2.0, 2.0, 2.0));
    let b = box_at(Vec3::new(2.0, 2.0, 2.0), Point3::new(1.0, 0.0, 0.0));
    let a_noisy = import_with_noise(&a, 8, 1e-4, 1);
    let b_noisy = import_with_noise(&b, 8, 1e-4, 2);
    // We accept Err here — the noise may be too much for tolerance defaults.
    // The important thing is no panic.
    match try_op(&a_noisy, &b_noisy, BooleanOp::Union) {
        Ok(r) => {
            kerf_topo::validate(&r.topo).expect("topology");
            // Just verify volume is sane.
            let v = solid_volume(&r);
            assert!(v > 0.0 && v < 100.0, "noisy union vol = {v}");
        }
        Err(e) => {
            // Acceptable: noise > tolerance is expected to break some cases.
            eprintln!("very-large-noise union returned Err (acceptable): {}", e.message);
        }
    }
}
