//! `torus_faceted(r_maj, r_min, n_maj, n_min)` — UV-torus polyhedron.
//!
//! Unlike [`torus`], the surface is `2·n_maj·n_min` planar triangles. The
//! geometry is fully planar; booleans work wherever intersection segments
//! land on shared edges.
//!
//! Topology: `n_maj·n_min` vertices, `3·n_maj·n_min` edges,
//! `2·n_maj·n_min` triangular faces.
//! Euler invariant: `V − E + F = n·m − 3n·m + 2n·m = 0` (genus-1). ✓

use std::f64::consts::TAU;

use kerf_geom::Point3;

use crate::mesh_import::from_triangles;
use crate::Solid;

/// Build a UV-torus polyhedron with `n_maj` major segments (around the z-axis)
/// and `n_min` minor segments (around the tube cross-section).
///
/// `r_maj` is the major radius (distance from z-axis to the tube centre);
/// `r_min` is the minor radius (tube cross-section radius).
/// Volume approaches `2π² r_maj r_min²` as n_maj, n_min grow.
///
/// # Panics
/// In debug builds: if either radius ≤ 0, `r_min >= r_maj`, or either
/// segment count < 3.
pub fn torus_faceted(r_maj: f64, r_min: f64, n_maj: usize, n_min: usize) -> Solid {
    debug_assert!(r_maj > 0.0, "major radius must be positive");
    debug_assert!(r_min > 0.0, "minor radius must be positive");
    debug_assert!(r_min < r_maj, "minor radius must be less than major radius");
    debug_assert!(n_maj >= 3, "n_maj must be at least 3");
    debug_assert!(n_min >= 3, "n_min must be at least 3");

    // Vertex grid: verts[j][i]
    // j ∈ 0..n_maj : major angle φ = TAU·j/n_maj (revolution around z-axis)
    // i ∈ 0..n_min : minor angle θ = TAU·i/n_min (angle around tube)
    let verts: Vec<Vec<Point3>> = (0..n_maj)
        .map(|j| {
            let phi = TAU * j as f64 / n_maj as f64;
            (0..n_min)
                .map(|i| {
                    let theta = TAU * i as f64 / n_min as f64;
                    Point3::new(
                        (r_maj + r_min * theta.cos()) * phi.cos(),
                        (r_maj + r_min * theta.cos()) * phi.sin(),
                        r_min * theta.sin(),
                    )
                })
                .collect()
        })
        .collect();

    // 2·n_maj·n_min triangles total.
    let mut tris: Vec<[Point3; 3]> = Vec::with_capacity(2 * n_maj * n_min);

    for j in 0..n_maj {
        let j1 = (j + 1) % n_maj;
        for i in 0..n_min {
            let i1 = (i + 1) % n_min;
            // Quad corners:
            //   A = (φ_j,   θ_i)    B = (φ_j,   θ_{i+1})
            //   D = (φ_{j+1}, θ_i)  C = (φ_{j+1}, θ_{i+1})
            let a = verts[j][i];
            let b = verts[j][i1];
            let c = verts[j1][i1];
            let d = verts[j1][i];
            // Triangles [A,D,C] and [A,C,B] give outward-facing normals.
            tris.push([a, d, c]);
            tris.push([a, c, b]);
        }
    }

    from_triangles(&tris).expect("torus_faceted: mesh is not manifold")
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{read_json, solid_volume, write_json};
    use kerf_topo::validate;
    use std::f64::consts::PI;

    #[test]
    fn torus_faceted_n3_topology() {
        let s = torus_faceted(2.0, 0.5, 3, 3);
        assert_eq!(s.vertex_count(), 3 * 3);      // n_maj · n_min
        assert_eq!(s.face_count(), 2 * 3 * 3);   // 2·n_maj·n_min
        validate(&s.topo).unwrap();
    }

    #[test]
    fn torus_faceted_n8_topology() {
        let s = torus_faceted(2.0, 0.5, 8, 8);
        assert_eq!(s.vertex_count(), 8 * 8);
        assert_eq!(s.face_count(), 2 * 8 * 8);
        validate(&s.topo).unwrap();
    }

    #[test]
    fn torus_faceted_n16_topology() {
        let s = torus_faceted(2.0, 0.5, 16, 12);
        assert_eq!(s.vertex_count(), 16 * 12);
        assert_eq!(s.face_count(), 2 * 16 * 12);
        validate(&s.topo).unwrap();
    }

    #[test]
    fn torus_faceted_volume_approaches_analytic() {
        // Volume of torus = 2π² R r².  For n_maj=32, n_min=32, expect < 2% error.
        let r_maj = 2.0;
        let r_min = 0.5;
        let s = torus_faceted(r_maj, r_min, 32, 32);
        let v_mesh = solid_volume(&s);
        let v_exact = 2.0 * PI * PI * r_maj * r_min * r_min;
        let rel_err = (v_mesh - v_exact).abs() / v_exact;
        assert!(
            rel_err < 0.02,
            "volume relative error {rel_err:.4} >= 2% (mesh={v_mesh:.6}, exact={v_exact:.6})"
        );
        assert!(v_mesh > 0.0, "volume is negative — normals are flipped");
    }

    #[test]
    fn torus_faceted_scales_with_minor_radius() {
        let s1 = torus_faceted(2.0, 0.5, 8, 8);
        let s2 = torus_faceted(2.0, 1.0, 8, 8);
        let ratio = solid_volume(&s2) / solid_volume(&s1);
        // Doubling r_min scales volume by 2² = 4.
        assert!(
            (ratio - 4.0).abs() < 1e-9,
            "volume ratio for 2× minor radius: {ratio:.6} (expected 4.0)"
        );
    }

    #[test]
    fn torus_faceted_json_roundtrip() {
        let s = torus_faceted(2.0, 0.5, 6, 6);
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
