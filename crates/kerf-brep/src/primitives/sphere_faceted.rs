//! `sphere_faceted(r, n)` — lat-long triangulated approximation of a sphere.
//!
//! Unlike [`sphere`], every face is a planar triangle, so the solid feeds
//! directly into the full boolean pipeline (no curved-face limit). The
//! geometry is a classic lat-long mesh: `n` latitude bands and `n` longitude
//! slices. Higher `n` ⇒ closer to a true sphere.
//!
//! Vertex counts: `n*(n-1) + 2` (two poles + `n-1` intermediate rings of `n`
//! vertices each). Triangle counts: `2*n*(n-1)`. Euler check: V-E+F = 2.
//!
//! Winding convention: outward-normal CCW, matching every other primitive.

use std::f64::consts::PI;
use std::f64::consts::TAU;

use kerf_geom::Point3;

use crate::mesh_import::from_triangles;
use crate::Solid;

/// Build a lat-long faceted sphere of radius `r` with `n` latitude bands and
/// `n` longitude slices.
///
/// The result has `n*(n-1)+2` vertices, `3*n*(n-1)` edges, and `2*n*(n-1)`
/// triangular faces. All faces are planar, so the full boolean pipeline applies.
///
/// # Panics
/// In debug builds if `r <= 0` or `n < 3`.
pub fn sphere_faceted(r: f64, n: usize) -> Solid {
    debug_assert!(r > 0.0, "radius must be positive");
    debug_assert!(n >= 3, "n must be at least 3");

    let n_lon = n;
    let n_lat = n;

    let north_pole = Point3::new(0.0, 0.0, r);
    let south_pole = Point3::new(0.0, 0.0, -r);

    // Build n_lat-1 intermediate latitude rings.
    // ring[j] is at phi = PI*(j+1)/n_lat (j=0 is first ring below north pole).
    let rings: Vec<Vec<Point3>> = (1..n_lat)
        .map(|stack| {
            let phi = PI * stack as f64 / n_lat as f64;
            let sin_phi = phi.sin();
            let cos_phi = phi.cos();
            (0..n_lon)
                .map(|slice| {
                    let theta = TAU * slice as f64 / n_lon as f64;
                    Point3::new(
                        r * sin_phi * theta.cos(),
                        r * sin_phi * theta.sin(),
                        r * cos_phi,
                    )
                })
                .collect()
        })
        .collect();

    let mut tris: Vec<[Point3; 3]> = Vec::with_capacity(2 * n_lon * (n_lat - 1));

    // North polar cap: [north_pole, ring[0][i], ring[0][(i+1)%n_lon]]
    // Gives outward normal by right-hand rule (verified analytically).
    for i in 0..n_lon {
        tris.push([north_pole, rings[0][i], rings[0][(i + 1) % n_lon]]);
    }

    // Middle quad strips: between rings[j] and rings[j+1] (j+1 is more south).
    // Each quad splits into two triangles with outward-normal CCW winding.
    for j in 0..rings.len() - 1 {
        for i in 0..n_lon {
            let p_bot_i = rings[j + 1][i];
            let p_bot_i1 = rings[j + 1][(i + 1) % n_lon];
            let p_top_i = rings[j][i];
            let p_top_i1 = rings[j][(i + 1) % n_lon];
            tris.push([p_bot_i, p_bot_i1, p_top_i1]);
            tris.push([p_bot_i, p_top_i1, p_top_i]);
        }
    }

    // South polar cap: [south_pole, ring[last][(i+1)%n_lon], ring[last][i]]
    // Reversed order vs north cap so the outward normal points downward.
    let last = rings.len() - 1;
    for i in 0..n_lon {
        tris.push([
            south_pole,
            rings[last][(i + 1) % n_lon],
            rings[last][i],
        ]);
    }

    from_triangles(&tris).expect("sphere_faceted: mesh is always a valid closed manifold")
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::measure::solid_volume;
    use crate::serde_io::{read_json, write_json};
    use std::f64::consts::PI;

    fn analytic_vol(r: f64) -> f64 {
        (4.0 / 3.0) * PI * r * r * r
    }

    #[test]
    fn sphere_faceted_n8_topology() {
        let n = 8usize;
        let s = sphere_faceted(1.0, n);
        // V = n*(n-1)+2, E = 3*n*(n-1), F = 2*n*(n-1)
        let rings = n - 1;
        assert_eq!(s.vertex_count(), n * rings + 2, "vertex count");
        assert_eq!(s.edge_count(), 3 * n * rings, "edge count");
        assert_eq!(s.face_count(), 2 * n * rings, "face count");
        kerf_topo::validate(&s.topo).unwrap();
    }

    #[test]
    fn sphere_faceted_n16_topology() {
        let n = 16usize;
        let s = sphere_faceted(1.0, n);
        let rings = n - 1;
        assert_eq!(s.vertex_count(), n * rings + 2);
        assert_eq!(s.edge_count(), 3 * n * rings);
        assert_eq!(s.face_count(), 2 * n * rings);
        kerf_topo::validate(&s.topo).unwrap();
    }

    #[test]
    fn sphere_faceted_volume_converges_to_analytic() {
        // A lat-long inscribed sphere always has less volume than the true sphere.
        // At n=24 the lat-long mesh loses ~1.5% (O(1/n²) convergence). Threshold
        // is set conservatively at 98% to confirm the right order of magnitude.
        let r = 3.0;
        let s = sphere_faceted(r, 24);
        let got = solid_volume(&s);
        let expected = analytic_vol(r);
        let ratio = got / expected;
        assert!(
            ratio > 0.98 && ratio <= 1.0,
            "n=24 volume ratio={ratio:.6} (expected >0.98, ≤1.0)"
        );
    }

    #[test]
    fn sphere_faceted_volume_positive() {
        let s = sphere_faceted(2.0, 8);
        let v = solid_volume(&s);
        assert!(v > 0.0, "volume should be positive, got {v}");
    }

    #[test]
    fn sphere_faceted_json_roundtrip() {
        let s = sphere_faceted(1.5, 8);
        let mut buf = Vec::new();
        write_json(&s, &mut buf).unwrap();
        let s2 = read_json(&mut buf.as_slice()).unwrap();
        assert_eq!(s.vertex_count(), s2.vertex_count());
        assert_eq!(s.edge_count(), s2.edge_count());
        assert_eq!(s.face_count(), s2.face_count());
        kerf_topo::validate(&s2.topo).unwrap();
        // Volume must be preserved exactly through JSON.
        let v1 = solid_volume(&s);
        let v2 = solid_volume(&s2);
        assert!(
            (v1 - v2).abs() < 1e-9,
            "JSON round-trip volume drift: {v1} vs {v2}"
        );
    }

    #[test]
    fn sphere_faceted_radius_scales_volume() {
        // Volume ∝ r³: doubling r should give 8× volume.
        let s1 = sphere_faceted(1.0, 12);
        let s2 = sphere_faceted(2.0, 12);
        let ratio = solid_volume(&s2) / solid_volume(&s1);
        assert!(
            (ratio - 8.0).abs() < 0.01,
            "volume ratio for 2r/r should be ≈8, got {ratio}"
        );
    }
}
