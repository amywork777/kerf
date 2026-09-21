//! `sphere_faceted(r, n_stacks, n_slices)` — pure-mesh UV sphere.
//!
//! Unlike the analytic [`sphere`] primitive (1V/0E/1F with a `SurfaceKind::Sphere`
//! face), every face here is a planar triangle built from `from_triangles`.
//! This sidesteps the known analytic-sphere boolean degeneracies (axis-aligned
//! cutters at certain segment counts) and lets the solid participate in the
//! boolean pipeline without a tessellate→reimport step.
//!
//! Topology: 2 + n_stacks × n_slices vertices,
//!           n_slices × 2 × n_stacks triangular faces,
//!           matching edges = 3 × n_slices × 2 × n_stacks / 2 (each edge shared twice) + poles.

use std::f64::consts::{PI, TAU};

use kerf_geom::Point3;

use crate::mesh_import::from_triangles;
use crate::Solid;

/// Build a UV-sphere mesh with `n_stacks` latitude bands and `n_slices`
/// longitude segments. The sphere is centered at the origin with radius `r`.
///
/// Vertex count: 2 + n_stacks × n_slices (2 polar + stacked rings).
/// Triangle count: 2 × n_stacks × n_slices.
///
/// # Panics (debug)
/// Panics if `r <= 0`, `n_stacks < 2`, or `n_slices < 3`.
pub fn sphere_faceted(r: f64, n_stacks: usize, n_slices: usize) -> Solid {
    debug_assert!(r > 0.0, "radius must be positive");
    debug_assert!(n_stacks >= 2, "n_stacks must be at least 2");
    debug_assert!(n_slices >= 3, "n_slices must be at least 3");

    // Build ring vertex positions.
    // Ring 0     = (index 0)                    north pole (0, 0, r)
    // Ring s     = (index 1 + (s-1)*n_slices+k) for s in 1..=n_stacks
    // South pole = (index 1 + n_stacks*n_slices)
    let total_verts = 2 + n_stacks * n_slices;
    let mut verts: Vec<Point3> = Vec::with_capacity(total_verts);

    verts.push(Point3::new(0.0, 0.0, r)); // north pole
    for s in 1..=n_stacks {
        let theta = s as f64 * PI / (n_stacks + 1) as f64;
        let sin_t = theta.sin();
        let cos_t = theta.cos();
        for k in 0..n_slices {
            let phi = k as f64 * TAU / n_slices as f64;
            verts.push(Point3::new(r * sin_t * phi.cos(), r * sin_t * phi.sin(), r * cos_t));
        }
    }
    verts.push(Point3::new(0.0, 0.0, -r)); // south pole

    // Accessor: ring s (1..=n_stacks), slice k.
    // k is taken mod n_slices so that k = n_slices wraps to 0.
    let ring = |s: usize, k: usize| -> Point3 { verts[1 + (s - 1) * n_slices + k % n_slices] };
    let v_north = verts[0];
    let v_south = *verts.last().unwrap();

    let mut tris: Vec<[Point3; 3]> = Vec::with_capacity(2 * n_stacks * n_slices);

    // North cap — outward normal verified via cross-product: positive z near pole.
    for k in 0..n_slices {
        tris.push([v_north, ring(1, k), ring(1, k + 1)]);
    }

    // Middle bands — each quad split into two triangles with outward normals.
    for s in 1..n_stacks {
        for k in 0..n_slices {
            // Lower triangle (south-east of quad)
            tris.push([ring(s, k), ring(s + 1, k), ring(s + 1, k + 1)]);
            // Upper triangle (north-west of quad)
            tris.push([ring(s, k), ring(s + 1, k + 1), ring(s, k + 1)]);
        }
    }

    // South cap — reversed slice order to point outward (-z near pole).
    for k in 0..n_slices {
        tris.push([v_south, ring(n_stacks, k + 1), ring(n_stacks, k)]);
    }

    from_triangles(&tris).expect("sphere_faceted: mesh import failed")
}

#[cfg(test)]
mod tests {
    use std::f64::consts::PI;

    use super::*;
    use kerf_topo::validate;

    use crate::measure::solid_volume;
    use crate::serde_io::{read_json, write_json};

    #[test]
    fn sphere_faceted_topology_n4_s8() {
        let n_stacks = 4;
        let n_slices = 8;
        let s = sphere_faceted(1.0, n_stacks, n_slices);
        // V = 2 + n_stacks*n_slices, F = 2*n_stacks*n_slices (all triangles)
        // E = 3 * F / 2 (closed manifold: each edge shared by exactly 2 faces) + ...
        // Euler: V - E + F = 2
        assert_eq!(s.vertex_count(), 2 + n_stacks * n_slices);
        assert_eq!(s.face_count(), 2 * n_stacks * n_slices);
        validate(&s.topo).expect("topology invalid");
    }

    #[test]
    fn sphere_faceted_topology_n8_s16() {
        let s = sphere_faceted(2.0, 8, 16);
        assert_eq!(s.vertex_count(), 2 + 8 * 16);
        assert_eq!(s.face_count(), 2 * 8 * 16);
        validate(&s.topo).expect("topology invalid");
    }

    /// Volume should converge toward (4/3)π r³ as resolution increases.
    /// At n=16 stacks × 32 slices, UV-sphere underestimates by ~1.5% (polar
    /// caps are approximated as cones). Tolerance set to 2%.
    #[test]
    fn sphere_faceted_volume_approx() {
        let r = 3.0;
        let s = sphere_faceted(r, 16, 32);
        let analytic = (4.0 / 3.0) * PI * r * r * r;
        let computed = solid_volume(&s);
        let rel_err = ((computed - analytic) / analytic).abs();
        assert!(
            rel_err < 0.02,
            "volume error {rel_err:.4} >= 2% (computed={computed:.6}, analytic={analytic:.6})"
        );
    }

    /// Medium-resolution sphere (n=10×20) should be within 5%.
    #[test]
    fn sphere_faceted_volume_coarse() {
        let r = 1.0;
        let s = sphere_faceted(r, 10, 20);
        let analytic = (4.0 / 3.0) * PI * r * r * r;
        let computed = solid_volume(&s);
        let rel_err = ((computed - analytic) / analytic).abs();
        assert!(
            rel_err < 0.05,
            "medium volume error {rel_err:.4} >= 5% (computed={computed:.6}, analytic={analytic:.6})"
        );
    }

    #[test]
    fn sphere_faceted_volume_positive() {
        let s = sphere_faceted(1.0, 3, 6);
        assert!(solid_volume(&s) > 0.0, "volume must be positive (winding check)");
    }

    #[test]
    fn sphere_faceted_json_roundtrip() {
        let s = sphere_faceted(1.5, 6, 12);
        let v0 = s.vertex_count();
        let e0 = s.edge_count();
        let f0 = s.face_count();

        let mut buf = Vec::new();
        write_json(&s, &mut buf).expect("write_json failed");
        let s2 = read_json(&mut std::io::Cursor::new(&buf)).expect("read_json failed");

        assert_eq!(s2.vertex_count(), v0, "vertex count changed after roundtrip");
        assert_eq!(s2.edge_count(), e0, "edge count changed after roundtrip");
        assert_eq!(s2.face_count(), f0, "face count changed after roundtrip");
        validate(&s2.topo).expect("roundtrip solid invalid");
    }
}
