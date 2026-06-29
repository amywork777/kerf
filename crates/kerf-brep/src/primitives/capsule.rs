//! `capsule_faceted(r, h, n, m)` — polyhedral capsule (cylinder + hemispherical end caps).
//!
//! Builds a closed triangle mesh via `from_triangles`. All faces are planar
//! triangles, so the solid works directly with the boolean pipeline (same as
//! imported STL meshes). No analytic curved-surface types used.
//!
//! ## Parameters
//! - `r` — hemisphere radius (> 0)
//! - `h` — straight-cylinder height (≥ 0; h=0 gives a sphere approximation)
//! - `n` — longitudinal segments (≥ 3)
//! - `m` — latitudinal bands per hemisphere (≥ 1; more bands → rounder caps)
//!
//! ## Triangle count
//! - Top pole fan: n triangles
//! - Top hemisphere bands: (m−1)·n·2 triangles
//! - Cylinder (h > 0): n·2 triangles
//! - Bottom hemisphere bands: (m−1)·n·2 triangles
//! - Bottom pole fan: n triangles
//! Total: 4·m·n (+ 2n when h > 0, split evenly above)
//!
//! ## Analytic volume (for testing)
//! V = (4/3)·π·r³ + π·r²·h  — exact capsule formula; the polyhedral
//! version converges to this as n→∞ and m→∞.

use std::f64::consts::{PI, TAU};

use kerf_geom::Point3;

use crate::mesh_import::from_triangles;
use crate::Solid;

/// Build a polyhedral capsule centred on the origin, cylinder axis along z.
///
/// # Panics (debug)
/// Panics if `r ≤ 0`, `h < 0`, `n < 3`, or `m < 1`.
pub fn capsule_faceted(r: f64, h: f64, n: usize, m: usize) -> Solid {
    debug_assert!(r > 0.0, "radius must be positive");
    debug_assert!(h >= 0.0, "cylinder height must be non-negative");
    debug_assert!(n >= 3, "n (longitudinal segments) must be at least 3");
    debug_assert!(m >= 1, "m (latitudinal bands) must be at least 1");

    let lon = |j: usize| TAU * j as f64 / n as f64;
    let jnext = |j: usize| (j + 1) % n;

    // Top hemisphere ring i (i=1 is near the north pole, i=m is the equator).
    // lat_angle = π/2 * i/m  (0 at pole, π/2 at equator)
    let top_ring = |i: usize, j: usize| -> Point3 {
        let lat = PI / 2.0 * i as f64 / m as f64;
        let ri = r * lat.sin();
        let zi = h / 2.0 + r * lat.cos();
        Point3::new(ri * lon(j).cos(), ri * lon(j).sin(), zi)
    };

    // Bottom hemisphere ring i (i=0 is the equator, i=m is the south pole).
    // lat_angle = π/2 * i/m  (0 at equator, π/2 at pole)
    let bot_ring = |i: usize, j: usize| -> Point3 {
        let lat = PI / 2.0 * i as f64 / m as f64;
        let ri = r * lat.cos();
        let zi = -(h / 2.0) - r * lat.sin();
        Point3::new(ri * lon(j).cos(), ri * lon(j).sin(), zi)
    };

    let top_pole = Point3::new(0.0, 0.0, h / 2.0 + r);
    let bot_pole = Point3::new(0.0, 0.0, -(h / 2.0 + r));

    let cap_tri_count = 2 * n // poles
        + 2 * (m - 1) * n * 2 // hemisphere bands (top + bot)
        + if h > 1e-12 { n * 2 } else { 0 }; // cylinder
    let mut triangles: Vec<[Point3; 3]> = Vec::with_capacity(cap_tri_count);

    // ── 1. North-pole fan ────────────────────────────────────────────────────
    // Normal at north pole is +z → CCW from above means (pole, ring[j], ring[j+1]).
    for j in 0..n {
        triangles.push([top_pole, top_ring(1, j), top_ring(1, jnext(j))]);
    }

    // ── 2. Top hemisphere bands (ring i → ring i+1, i ∈ 1..m) ──────────────
    // For quad (A=ring[i][j], B=ring[i][j+1], C=ring[i+1][j+1], D=ring[i+1][j]):
    // outward-normal (verified by cross-product) requires [A,C,B] and [A,D,C].
    for i in 1..m {
        for j in 0..n {
            let a = top_ring(i, j);
            let b = top_ring(i, jnext(j));
            let c = top_ring(i + 1, jnext(j));
            let d = top_ring(i + 1, j);
            triangles.push([a, c, b]);
            triangles.push([a, d, c]);
        }
    }

    // ── 3. Cylinder section (only when h > 0) ───────────────────────────────
    // top_ring(m, *) is the north equator (z = +h/2, radius = r).
    // bot_ring(0, *) is the south equator (z = −h/2, radius = r).
    if h > 1e-12 {
        for j in 0..n {
            let a = top_ring(m, j);
            let b = top_ring(m, jnext(j));
            let c = bot_ring(0, jnext(j));
            let d = bot_ring(0, j);
            triangles.push([a, c, b]);
            triangles.push([a, d, c]);
        }
    }

    // ── 4. Bottom hemisphere bands (ring i → ring i+1, i ∈ 0..m−1) ─────────
    // Same quad winding [A,C,B],[A,D,C] gives outward (downward) normal.
    for i in 0..m - 1 {
        for j in 0..n {
            let a = bot_ring(i, j);
            let b = bot_ring(i, jnext(j));
            let c = bot_ring(i + 1, jnext(j));
            let d = bot_ring(i + 1, j);
            triangles.push([a, c, b]);
            triangles.push([a, d, c]);
        }
    }

    // ── 5. South-pole fan ────────────────────────────────────────────────────
    // Normal at south pole is −z → CCW from below means (pole, ring[j+1], ring[j]).
    for j in 0..n {
        triangles.push([bot_pole, bot_ring(m - 1, jnext(j)), bot_ring(m - 1, j)]);
    }

    from_triangles(&triangles).expect("capsule_faceted: triangle mesh build failed")
}

/// Analytic volume of an ideal (smooth) capsule: (4/3)πr³ + πr²h.
pub fn capsule_volume_analytic(r: f64, h: f64) -> f64 {
    (4.0 / 3.0) * PI * r * r * r + PI * r * r * h
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{read_json, solid_volume, write_json};
    use kerf_topo::validate;

    // Tolerance for volume comparison: polyhedral approximation error scales
    // roughly as O(1/m²) and O(1/n²). With n=24, m=8, error is ~2%; m=16 gives < 1%.
    const VOL_TOL_LOOSE: f64 = 0.03; // 3 % — coarse meshes (m=8)
    const VOL_TOL_TIGHT: f64 = 0.01; // 1 % — fine meshes (m=16)

    // ── topology ─────────────────────────────────────────────────────────────

    #[test]
    fn capsule_n6_m2_topology_validates() {
        // n=6, m=2: smallest non-trivial configuration with distinct hemisphere bands.
        // F = 4*m*n = 48 (h=0 path). V and E follow from Euler.
        let s = capsule_faceted(1.0, 0.0, 6, 2);
        validate(&s.topo).expect("topology invalid");
        // Euler check: V - E + F = 2
        let v = s.vertex_count() as i64;
        let e = s.edge_count() as i64;
        let f = s.face_count() as i64;
        assert_eq!(v - e + f, 2, "Euler invariant violated: V={v} E={e} F={f}");
        assert!(s.face_count() > 0, "no faces");
    }

    #[test]
    fn capsule_with_cylinder_topology_validates() {
        let s = capsule_faceted(1.0, 2.0, 8, 3);
        validate(&s.topo).expect("topology invalid");
        let v = s.vertex_count() as i64;
        let e = s.edge_count() as i64;
        let f = s.face_count() as i64;
        assert_eq!(v - e + f, 2, "Euler invariant violated: V={v} E={e} F={f}");
    }

    #[test]
    fn capsule_m1_is_bicone_cylinder() {
        // m=1 → no hemisphere bands; just pole fans + optional cylinder.
        let s = capsule_faceted(1.0, 2.0, 8, 1);
        validate(&s.topo).expect("topology invalid");
        // 2 cone faces (n tri each) + 2n cylinder tri = 4n faces total? No:
        // 2*n (poles) + 0 bands + 2*n (cylinder) = 4n triangles = 32 faces.
        assert_eq!(s.face_count(), 4 * 8);
    }

    // ── volume vs analytic ────────────────────────────────────────────────────

    #[test]
    fn sphere_like_capsule_volume_within_3pct() {
        // h=0 → sphere of radius r=1. Analytic V = 4/3 π ≈ 4.1888.
        // Coarse mesh (m=8) gives ~2% error, well within 3%.
        let r = 1.0;
        let s = capsule_faceted(r, 0.0, 24, 8);
        let got = solid_volume(&s);
        let expected = capsule_volume_analytic(r, 0.0);
        let err = (got - expected).abs() / expected;
        assert!(
            err < VOL_TOL_LOOSE,
            "sphere-like capsule vol={got:.6} expected={expected:.6} err={err:.4}"
        );
    }

    #[test]
    fn sphere_like_capsule_fine_mesh_within_1pct() {
        // Fine mesh (m=16) — error < 1%.
        let r = 1.0;
        let s = capsule_faceted(r, 0.0, 32, 16);
        let got = solid_volume(&s);
        let expected = capsule_volume_analytic(r, 0.0);
        let err = (got - expected).abs() / expected;
        assert!(
            err < VOL_TOL_TIGHT,
            "fine sphere-like capsule vol={got:.6} expected={expected:.6} err={err:.4}"
        );
    }

    #[test]
    fn capsule_volume_r1_h2_within_3pct() {
        // r=1, h=2: V = 4/3π + 2π ≈ 10.4720
        let (r, h) = (1.0, 2.0);
        let s = capsule_faceted(r, h, 24, 8);
        let got = solid_volume(&s);
        let expected = capsule_volume_analytic(r, h);
        let err = (got - expected).abs() / expected;
        assert!(
            err < VOL_TOL_LOOSE,
            "capsule r={r} h={h} vol={got:.6} expected={expected:.6} err={err:.4}"
        );
    }

    #[test]
    fn capsule_volume_r2_h3_within_3pct() {
        let (r, h) = (2.0, 3.0);
        let s = capsule_faceted(r, h, 24, 8);
        let got = solid_volume(&s);
        let expected = capsule_volume_analytic(r, h);
        let err = (got - expected).abs() / expected;
        assert!(
            err < VOL_TOL_LOOSE,
            "capsule r={r} h={h} vol={got:.6} expected={expected:.6} err={err:.4}"
        );
    }

    #[test]
    fn capsule_volume_scales_with_r_cubed() {
        // Doubling radius should 8× the sphere contribution and 4× the cylinder.
        let h = 1.0;
        let v1 = solid_volume(&capsule_faceted(1.0, h, 24, 8));
        let v2 = solid_volume(&capsule_faceted(2.0, h, 24, 8));
        let ratio = v2 / v1;
        let expected_ratio = capsule_volume_analytic(2.0, h) / capsule_volume_analytic(1.0, h);
        // Ratio test is tighter since errors cancel between numerator and denominator.
        assert!(
            (ratio - expected_ratio).abs() / expected_ratio < 0.02,
            "volume scaling ratio={ratio:.4} expected={expected_ratio:.4}"
        );
    }

    // ── JSON round-trip ──────────────────────────────────────────────────────

    #[test]
    fn capsule_sphere_like_json_roundtrip() {
        let s = capsule_faceted(1.0, 0.0, 12, 4);
        let mut buf = Vec::new();
        write_json(&s, &mut buf).expect("write_json");
        let s2 = read_json(&mut buf.as_slice()).expect("read_json");
        assert_eq!(s.vertex_count(), s2.vertex_count());
        assert_eq!(s.edge_count(), s2.edge_count());
        assert_eq!(s.face_count(), s2.face_count());
        validate(&s2.topo).expect("round-tripped topology invalid");
    }

    #[test]
    fn capsule_with_cylinder_json_roundtrip() {
        let s = capsule_faceted(1.0, 3.0, 12, 4);
        let mut buf = Vec::new();
        write_json(&s, &mut buf).expect("write_json");
        let s2 = read_json(&mut buf.as_slice()).expect("read_json");
        assert_eq!(s.vertex_count(), s2.vertex_count());
        assert_eq!(s.face_count(), s2.face_count());
        validate(&s2.topo).expect("round-tripped topology invalid");
        // Volume must be preserved within float round-trip tolerance.
        let v1 = solid_volume(&s);
        let v2 = solid_volume(&s2);
        assert!(
            (v1 - v2).abs() < 1e-6,
            "JSON round-trip volume drift: {v1} vs {v2}"
        );
    }
}
