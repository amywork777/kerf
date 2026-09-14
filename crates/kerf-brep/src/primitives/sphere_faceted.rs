//! `sphere_faceted(r, n_lat, n_lon)` — all-planar UV-sphere polyhedron.
//!
//! Unlike [`sphere`], every face is a flat triangle — no `SurfaceKind::Sphere`
//! surface. The result is a convex polyhedron that approximates a sphere and
//! works with the full planar boolean pipeline.
//!
//! Construction uses a UV grid:
//! - `n_lat` latitude rings between the poles (`n_lat ≥ 1`)
//! - `n_lon` longitude segments (`n_lon ≥ 3`)
//!
//! Topology: V = 2 + n_lat·n_lon, E = 3·n_lat·n_lon, F = 2·n_lat·n_lon.
//! Euler V−E+F = 2 guaranteed.
//!
//! Winding convention: CCW when viewed from outside (outward normals), matching
//! the kerf kernel standard so `solid_volume` and validation pass immediately.

use std::f64::consts::PI;

use kerf_geom::Point3;

use crate::from_triangles;
use crate::Solid;

/// Build an all-planar polyhedron approximating a sphere of radius `r`.
///
/// Uses a UV parameterization with `n_lat` latitude rings (exclusive of poles)
/// and `n_lon` longitude segments. Polar caps are triangle fans; the body is a
/// quad strip (2 triangles per quad).
///
/// Topology counts: V = 2 + n_lat·n_lon, E = 3·n_lat·n_lon,
/// F = 2·n_lat·n_lon. Euler V−E+F = 2 holds for all valid inputs.
///
/// # Panics
/// In debug builds: if `r ≤ 0`, `n_lat < 1`, or `n_lon < 3`.
pub fn sphere_faceted(r: f64, n_lat: usize, n_lon: usize) -> Solid {
    debug_assert!(r > 0.0, "radius must be positive");
    debug_assert!(n_lat >= 1, "n_lat must be at least 1");
    debug_assert!(n_lon >= 3, "n_lon must be at least 3");

    let north = Point3::new(0.0, 0.0, r);
    let south = Point3::new(0.0, 0.0, -r);

    // ring(lat, lon): vertex on the latitude ring at polar angle
    //   theta = π*(lat+1)/(n_lat+1)   (0 = north pole, π = south pole)
    // and azimuthal angle phi = 2π*lon/n_lon.
    let ring = |lat: usize, lon: usize| -> Point3 {
        let theta = PI * (lat + 1) as f64 / (n_lat + 1) as f64;
        let phi = 2.0 * PI * lon as f64 / n_lon as f64;
        Point3::new(
            r * theta.sin() * phi.cos(),
            r * theta.sin() * phi.sin(),
            r * theta.cos(),
        )
    };

    let n_tris = 2 * n_lon + 2 * n_lon * n_lat.saturating_sub(1);
    let mut tris: Vec<[Point3; 3]> = Vec::with_capacity(n_tris);

    // North cap: triangle fan from north pole to first latitude ring.
    // [N, R1, R2] → normal = (R1−N)×(R2−N); z-component = r²·sin²(θ)·sin(Δφ) > 0.
    // Outward at the north pole. ✓
    for lon in 0..n_lon {
        let r1 = ring(0, lon);
        let r2 = ring(0, (lon + 1) % n_lon);
        tris.push([north, r1, r2]);
    }

    // South cap: triangle fan from last latitude ring to south pole.
    // [S, R2, R1] reverses the R1↔R2 order so the normal flips to −z. ✓
    for lon in 0..n_lon {
        let r1 = ring(n_lat - 1, lon);
        let r2 = ring(n_lat - 1, (lon + 1) % n_lon);
        tris.push([south, r2, r1]);
    }

    // Body: quad strip between consecutive latitude rings.
    // Each quad (TL, TR above; BL, BR below) → two outward-facing triangles.
    // [TL, BR, TR] and [TL, BL, BR] — verified analytically:
    //   normal = (BR−TL)×(TR−TL) and (BL−TL)×(BR−TL); both point radially out. ✓
    for lat in 0..n_lat.saturating_sub(1) {
        for lon in 0..n_lon {
            let tl = ring(lat, lon);
            let tr = ring(lat, (lon + 1) % n_lon);
            let bl = ring(lat + 1, lon);
            let br = ring(lat + 1, (lon + 1) % n_lon);
            tris.push([tl, br, tr]);
            tris.push([tl, bl, br]);
        }
    }

    from_triangles(&tris).expect("sphere_faceted UV grid is always a valid closed manifold")
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{read_json, solid_volume, write_json};
    use std::f64::consts::PI;

    // -----------------------------------------------------------------------
    // Topology invariants
    // -----------------------------------------------------------------------

    fn check_topology(n_lat: usize, n_lon: usize) {
        let s = sphere_faceted(1.0, n_lat, n_lon);
        let expected_v = 2 + n_lat * n_lon;
        let expected_e = 3 * n_lat * n_lon;
        let expected_f = 2 * n_lat * n_lon;
        assert_eq!(s.vertex_count(), expected_v, "n_lat={n_lat} n_lon={n_lon}: V");
        assert_eq!(s.edge_count(), expected_e, "n_lat={n_lat} n_lon={n_lon}: E");
        assert_eq!(s.face_count(), expected_f, "n_lat={n_lat} n_lon={n_lon}: F");
        kerf_topo::validate(&s.topo).unwrap_or_else(|e| {
            panic!("topology invalid for n_lat={n_lat} n_lon={n_lon}: {e}")
        });
        // Euler V−E+F = 2 (genus-0 closed surface).
        let euler = expected_v as i64 - expected_e as i64 + expected_f as i64;
        assert_eq!(euler, 2, "Euler characteristic");
    }

    #[test]
    fn sphere_faceted_topology_1lat_3lon() {
        check_topology(1, 3); // octahedron-like (bipyramid)
    }

    #[test]
    fn sphere_faceted_topology_1lat_6lon() {
        check_topology(1, 6);
    }

    #[test]
    fn sphere_faceted_topology_2lat_4lon() {
        check_topology(2, 4);
    }

    #[test]
    fn sphere_faceted_topology_4lat_8lon() {
        check_topology(4, 8);
    }

    #[test]
    fn sphere_faceted_topology_8lat_16lon() {
        check_topology(8, 16);
    }

    // -----------------------------------------------------------------------
    // Volume converges to analytic 4/3 π r³
    //
    // A UV sphere is *inscribed*: all vertices lie on the sphere surface so
    // the planar-face polyhedron is always smaller than the true sphere.
    // Convergence is O(1/n²) in the grid resolution.
    // -----------------------------------------------------------------------

    #[test]
    fn sphere_faceted_volume_low_res() {
        // n=4×8 — coarse; UV sphere error is ~18-20% at this resolution.
        let r = 1.0;
        let s = sphere_faceted(r, 4, 8);
        let v = solid_volume(&s);
        let analytic = 4.0 / 3.0 * PI * r.powi(3);
        // Inscribed polyhedron: volume < analytic.
        assert!(v > 0.0, "volume must be positive; got {v}");
        assert!(
            v < analytic + 1e-9,
            "inscribed polyhedron must be ≤ sphere volume; got {v} vs {analytic}"
        );
        assert!(
            analytic - v < analytic * 0.22,
            "n=4×8 volume {v} too far from analytic {analytic}"
        );
    }

    #[test]
    fn sphere_faceted_volume_high_res() {
        // n=12×24 — medium resolution; UV sphere error is ~2.5-3%.
        let r = 2.0;
        let s = sphere_faceted(r, 12, 24);
        let v = solid_volume(&s);
        let analytic = 4.0 / 3.0 * PI * r.powi(3);
        assert!(v > 0.0);
        assert!(v < analytic + 1e-9);
        assert!(
            analytic - v < analytic * 0.04,
            "n=12×24 volume {v} too far from analytic {analytic}"
        );
    }

    #[test]
    fn sphere_faceted_volume_converges_with_resolution() {
        // Higher resolution should give a larger volume (closer to analytic).
        let r = 1.0;
        let v_coarse = solid_volume(&sphere_faceted(r, 4, 8));
        let v_medium = solid_volume(&sphere_faceted(r, 8, 16));
        let v_fine = solid_volume(&sphere_faceted(r, 12, 24));
        assert!(
            v_coarse < v_medium,
            "medium should be larger than coarse: {v_coarse} vs {v_medium}"
        );
        assert!(
            v_medium < v_fine,
            "fine should be larger than medium: {v_medium} vs {v_fine}"
        );
    }

    #[test]
    fn sphere_faceted_volume_scales_with_radius() {
        // V ∝ r³: doubling r multiplies volume by 8.
        let s1 = sphere_faceted(1.0, 8, 16);
        let s2 = sphere_faceted(2.0, 8, 16);
        let v1 = solid_volume(&s1);
        let v2 = solid_volume(&s2);
        assert!((v2 / v1 - 8.0).abs() < 1e-9, "v2/v1 = {} (expected 8)", v2 / v1);
    }

    // -----------------------------------------------------------------------
    // JSON round-trip
    // -----------------------------------------------------------------------

    #[test]
    fn sphere_faceted_json_roundtrip() {
        let s = sphere_faceted(1.5, 4, 8);
        let v_before = solid_volume(&s);
        let mut buf = Vec::new();
        write_json(&s, &mut buf).expect("write_json");
        let s2 = read_json(&mut buf.as_slice()).expect("read_json");
        // Topology counts preserved.
        assert_eq!(s.vertex_count(), s2.vertex_count(), "V after roundtrip");
        assert_eq!(s.edge_count(), s2.edge_count(), "E after roundtrip");
        assert_eq!(s.face_count(), s2.face_count(), "F after roundtrip");
        // Volume preserved.
        let v_after = solid_volume(&s2);
        assert!(
            (v_before - v_after).abs() < 1e-9,
            "volume changed after roundtrip: {v_before} → {v_after}"
        );
    }
}
