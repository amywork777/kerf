//! `sphere_faceted(r, n)` — UV-sphere polyhedron approximation.
//!
//! Unlike [`sphere`], the surface is `2n²` planar triangles instead of one
//! analytic Sphere surface. The geometry is fully planar, so the boolean
//! pipeline applies wherever intersection segments land on shared edges.
//!
//! Topology: `n² + 2` vertices, `3n²` edges, `2n²` triangular faces.
//! Euler invariant: `V − E + F = (n²+2) − 3n² + 2n² = 2`. ✓

use std::f64::consts::{PI, TAU};

use kerf_geom::Point3;

use crate::mesh_import::from_triangles;
use crate::Solid;

/// Build a UV-sphere polyhedron with `n` latitude rings and `n` longitude
/// slices, inscribed in a sphere of radius `r`.
///
/// Higher `n` ⇒ closer to a true sphere; volume approaches `(4/3)π r³`.
/// The result has `n² + 2` vertices, `3n²` edges, and `2n²` triangular faces.
///
/// # Panics
/// In debug builds: if `r <= 0` or `n < 3`.
pub fn sphere_faceted(r: f64, n: usize) -> Solid {
    debug_assert!(r > 0.0, "radius must be positive");
    debug_assert!(n >= 3, "n must be at least 3");

    let n_lon = n; // longitude slices (columns)
    let n_lat = n; // latitude stacks (rings between the two poles)

    // North and south poles.
    let pole_n = Point3::new(0.0, 0.0, r);
    let pole_s = Point3::new(0.0, 0.0, -r);

    // Ring vertices: rings[i][j], i ∈ 0..n_lat (top→bottom), j ∈ 0..n_lon.
    // Colatitude θ_i goes from π/(n_lat+1) (just below north pole) to
    // n_lat·π/(n_lat+1) (just above south pole).
    let rings: Vec<Vec<Point3>> = (0..n_lat)
        .map(|i| {
            let theta = PI * (i + 1) as f64 / (n_lat + 1) as f64;
            (0..n_lon)
                .map(|j| {
                    let phi = TAU * j as f64 / n_lon as f64;
                    Point3::new(
                        r * theta.sin() * phi.cos(),
                        r * theta.sin() * phi.sin(),
                        r * theta.cos(),
                    )
                })
                .collect()
        })
        .collect();

    // 2·n_lat·n_lon triangles total.
    let mut tris: Vec<[Point3; 3]> = Vec::with_capacity(2 * n_lat * n_lon);

    // North cap: n_lon fan triangles from pole_n to ring[0].
    // Winding [pole_n, ring[0][j], ring[0][j+1]] → outward normal ≈ +z. ✓
    for j in 0..n_lon {
        tris.push([pole_n, rings[0][j], rings[0][(j + 1) % n_lon]]);
    }

    // Body bands: between ring[i] and ring[i+1], n_lon quads → 2·n_lon triangles.
    // Let A = ring[i][j], B = ring[i][j+1], C = ring[i+1][j+1], D = ring[i+1][j].
    // Triangles [A,D,C] and [A,C,B] give outward-facing normals.
    for i in 0..n_lat - 1 {
        for j in 0..n_lon {
            let a = rings[i][j];
            let b = rings[i][(j + 1) % n_lon];
            let c = rings[i + 1][(j + 1) % n_lon];
            let d = rings[i + 1][j];
            tris.push([a, d, c]);
            tris.push([a, c, b]);
        }
    }

    // South cap: n_lon fan triangles from pole_s to ring[n_lat-1].
    // Winding [pole_s, ring[n-1][j+1], ring[n-1][j]] → outward normal ≈ -z. ✓
    for j in 0..n_lon {
        tris.push([
            pole_s,
            rings[n_lat - 1][(j + 1) % n_lon],
            rings[n_lat - 1][j],
        ]);
    }

    from_triangles(&tris).expect("sphere_faceted: mesh is not manifold")
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{read_json, solid_volume, write_json};
    use kerf_topo::validate;
    use std::f64::consts::PI;

    #[test]
    fn sphere_faceted_n3_topology() {
        let s = sphere_faceted(1.0, 3);
        assert_eq!(s.vertex_count(), 3 * 3 + 2); // n² + 2
        assert_eq!(s.face_count(), 2 * 3 * 3);   // 2n²
        validate(&s.topo).unwrap();
    }

    #[test]
    fn sphere_faceted_n8_topology() {
        let s = sphere_faceted(1.0, 8);
        assert_eq!(s.vertex_count(), 8 * 8 + 2);
        assert_eq!(s.face_count(), 2 * 8 * 8);
        validate(&s.topo).unwrap();
    }

    #[test]
    fn sphere_faceted_n16_topology() {
        let s = sphere_faceted(1.0, 16);
        assert_eq!(s.vertex_count(), 16 * 16 + 2);
        assert_eq!(s.face_count(), 2 * 16 * 16);
        validate(&s.topo).unwrap();
    }

    #[test]
    fn sphere_faceted_volume_approaches_analytic() {
        // Volume of inscribed mesh is slightly less than (4/3)π r³.
        // Relative error is O(1/n²); for n=32 expect < 2%.
        let r = 1.5;
        let s = sphere_faceted(r, 32);
        let v_mesh = solid_volume(&s);
        let v_exact = 4.0 / 3.0 * PI * r * r * r;
        let rel_err = (v_mesh - v_exact).abs() / v_exact;
        assert!(
            rel_err < 0.02,
            "volume relative error {rel_err:.4} >= 2% (mesh={v_mesh:.6}, exact={v_exact:.6})"
        );
        // Volume must be positive (correct winding).
        assert!(v_mesh > 0.0, "volume is negative — normals are flipped");
    }

    #[test]
    fn sphere_faceted_scales_with_radius() {
        let s1 = sphere_faceted(1.0, 8);
        let s2 = sphere_faceted(2.0, 8);
        let ratio = solid_volume(&s2) / solid_volume(&s1);
        // Doubling r scales volume by 2³ = 8.
        assert!(
            (ratio - 8.0).abs() < 1e-9,
            "volume ratio for 2× radius: {ratio:.6} (expected 8.0)"
        );
    }

    #[test]
    fn sphere_faceted_json_roundtrip() {
        let s = sphere_faceted(1.0, 6);
        let mut buf = Vec::new();
        write_json(&s, &mut buf).unwrap();
        let s2 = read_json(&mut buf.as_slice()).unwrap();
        assert_eq!(s.vertex_count(), s2.vertex_count());
        assert_eq!(s.edge_count(), s2.edge_count());
        assert_eq!(s.face_count(), s2.face_count());
        let v1 = solid_volume(&s);
        let v2 = solid_volume(&s2);
        assert!((v1 - v2).abs() < 1e-9, "JSON round-trip volume drift: {v1} → {v2}");
    }
}
