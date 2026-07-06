//! `torus_faceted(r_major, r_minor, n)` — polyhedral torus approximation.
//!
//! Unlike [`torus`], the surface is a grid of planar quad-pairs — no analytic
//! `Torus` surface kind. The geometry is fully planar, so it feeds directly
//! into the boolean pipeline without the curved-surface intersection path.
//!
//! The mesh uses the standard toroidal UV parametrisation:
//! - `u` ∈ [0, 2π] around the major axis (CCW from +z when viewed from above)
//! - `v` ∈ [0, 2π] around the tube cross-section
//! - `point(u, v) = ((R + r*cos(v))*cos(u), (R + r*cos(v))*sin(u), r*sin(v))`
//!
//! Triangle winding is CCW from the outward normal (same convention as
//! `tessellate` for `Torus` and `sphere_faceted`).
//!
//! The result is a **genus-1** solid: one shell, Euler characteristic 0
//! (V − E + F = 0).

use std::f64::consts::TAU;

use kerf_geom::Point3;

use crate::mesh_import::from_triangles;
use crate::Solid;

/// Build a polyhedral (all-planar-face) approximation of a torus.
///
/// - `r_major` — distance from the torus center to the center of the tube.
/// - `r_minor` — radius of the tube cross-section (`r_minor < r_major`).
/// - `n` — number of longitude segments around the major axis.
///   The minor cross-section uses `(n / 2).max(4)` segments.
///
/// Higher `n` gives a smoother approximation. At `n = 16` the torus is
/// visually round; at `n = 32` it is nearly indistinguishable from analytic.
///
/// The result has `n * (n/2).max(4)` quad faces (each split into 2 triangles)
/// and `SurfaceKind::Plane` on every face.
///
/// # Panics
/// Debug builds: if `r_major <= 0`, `r_minor <= 0`, `r_minor >= r_major`,
/// or `n < 4`.
pub fn torus_faceted(r_major: f64, r_minor: f64, n: usize) -> Solid {
    debug_assert!(r_major > 0.0, "r_major must be positive");
    debug_assert!(r_minor > 0.0, "r_minor must be positive");
    debug_assert!(
        r_minor < r_major,
        "r_minor must be < r_major (no self-intersecting torus)"
    );
    debug_assert!(n >= 4, "n must be at least 4");

    let n_maj = n;
    let n_min = (n / 2).max(4);

    let du = TAU / n_maj as f64;
    let dv = TAU / n_min as f64;

    // Point on the torus at major angle u, minor angle v.
    let pt = |u: f64, v: f64| -> Point3 {
        let rho = r_major + r_minor * v.cos();
        Point3::new(rho * u.cos(), rho * u.sin(), r_minor * v.sin())
    };

    let mut tris: Vec<[Point3; 3]> = Vec::new();

    for i in 0..n_maj {
        let u0 = i as f64 * du;
        let u1 = ((i + 1) % n_maj) as f64 * du;
        for j in 0..n_min {
            let v0 = j as f64 * dv;
            let v1 = ((j + 1) % n_min) as f64 * dv;
            // Four corners of the quad.
            let p00 = pt(u0, v0); // current-major, current-minor
            let p10 = pt(u1, v0); // next-major, current-minor
            let p11 = pt(u1, v1); // next-major, next-minor
            let p01 = pt(u0, v1); // current-major, next-minor
            // Winding matches tessellate's torus branch: CCW from outward normal.
            tris.push([p00, p10, p11]);
            tris.push([p00, p11, p01]);
        }
    }

    from_triangles(&tris).expect("torus_faceted: produced non-manifold mesh")
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    fn analytic_torus_vol(r_maj: f64, r_min: f64) -> f64 {
        2.0 * PI * PI * r_maj * r_min * r_min
    }

    #[test]
    fn torus_faceted_topology_validates() {
        let t = torus_faceted(1.0, 0.25, 16);
        kerf_topo::validate(&t.topo).unwrap();
    }

    #[test]
    fn torus_faceted_is_genus_one() {
        // Torus: 1 shell, genus 1 → V - E + F = 0 (Euler char = 0).
        let t = torus_faceted(1.0, 0.25, 16);
        let v = t.vertex_count() as i64;
        let e = t.edge_count() as i64;
        let f = t.face_count() as i64;
        assert_eq!(
            v - e + f,
            0,
            "torus Euler char must be 0 (genus 1): V={v} E={e} F={f}"
        );
        assert_eq!(t.topo.shell_count(), 1, "torus is one shell");
    }

    #[test]
    fn torus_faceted_euler_counts_match_formula() {
        // n_maj=16, n_min=max(8,4)=8: 16*8=128 quads → 256 triangular faces.
        // Each quad shares all 4 edges with neighbours: E = 2 * n_maj * n_min.
        // Vertices: V = n_maj * n_min.
        // So V=128, E=256, F=256 → V-E+F = 128-256+256 = 128... wait, that's not right.
        //
        // Actually for a triangulated toroidal grid with n_maj*n_min quads, each split into 2:
        // V = n_maj * n_min (shared vertices, wraparound)
        // F = 2 * n_maj * n_min (triangles)
        // E = 3 * n_maj * n_min (each quad has 4 edges, shared: 2*n_maj*n_min quad-edges
        //     + n_maj*n_min diagonals; shared edges: 4 per quad but each shared between 2).
        //     Actually: E_horizontal = n_maj*n_min, E_vertical = n_maj*n_min,
        //               E_diagonal   = n_maj*n_min. Total = 3*n_maj*n_min.
        // V-E+F = n_maj*n_min - 3*n_maj*n_min + 2*n_maj*n_min = 0. ✓
        let n_maj = 16usize;
        let n_min = (n_maj / 2).max(4); // = 8
        let t = torus_faceted(1.0, 0.25, n_maj);
        let expected_v = n_maj * n_min;
        let expected_f = 2 * n_maj * n_min;
        let expected_e = 3 * n_maj * n_min;
        assert_eq!(t.vertex_count(), expected_v, "vertex count");
        assert_eq!(t.face_count(), expected_f, "face count");
        assert_eq!(t.edge_count(), expected_e, "edge count");
    }

    #[test]
    fn torus_faceted_volume_approaches_analytic() {
        let (r_maj, r_min) = (1.0, 0.25);
        let analytic = analytic_torus_vol(r_maj, r_min);
        let v16 = crate::measure::solid_volume(&torus_faceted(r_maj, r_min, 16));
        let v32 = crate::measure::solid_volume(&torus_faceted(r_maj, r_min, 32));
        // Polyhedral approx always underestimates.
        assert!(v16 < analytic, "n=16 vol {v16} >= analytic {analytic}");
        assert!(v32 < analytic, "n=32 vol {v32} >= analytic {analytic}");
        // n=16 (8 minor segs) within ~15%, n=32 (16 minor segs) within ~4%.
        assert!(
            analytic - v16 < analytic * 0.15,
            "n=16 too far: vol={v16} analytic={analytic}"
        );
        assert!(
            analytic - v32 < analytic * 0.04,
            "n=32 too far: vol={v32} analytic={analytic}"
        );
        assert!(v32 > v16, "n=32 should be closer to analytic than n=16");
    }

    #[test]
    fn torus_faceted_json_roundtrip() {
        use crate::serde_io::{read_json, write_json};
        let t = torus_faceted(1.0, 0.25, 16);
        let mut buf = Vec::new();
        write_json(&t, &mut buf).unwrap();
        let t2 = read_json(&mut buf.as_slice()).unwrap();
        assert_eq!(t.vertex_count(), t2.vertex_count());
        assert_eq!(t.edge_count(), t2.edge_count());
        assert_eq!(t.face_count(), t2.face_count());
        kerf_topo::validate(&t2.topo).unwrap();
        let v1 = crate::measure::solid_volume(&t);
        let v2 = crate::measure::solid_volume(&t2);
        assert!(
            (v1 - v2).abs() < 1e-9,
            "JSON roundtrip volume drift: {v1} → {v2}"
        );
    }

    #[test]
    fn torus_faceted_boolean_chip_diff() {
        use crate::primitives::box_at;
        use kerf_geom::{Point3, Vec3};
        // Carve a small chip from the outer rim.  The cutter is fully inside
        // the torus body (outer rim at x ≈ 1.0 + 0.25 = 1.25) so the result
        // must be strictly smaller.
        let t = torus_faceted(1.0, 0.25, 16);
        let v_before = crate::measure::solid_volume(&t);
        let cutter = box_at(Vec3::new(0.2, 0.2, 0.2), Point3::new(0.9, 0.05, 0.05));
        let result = t.try_difference(&cutter).expect("torus_faceted − box chip");
        let v_after = crate::measure::solid_volume(&result);
        assert!(v_after < v_before, "diff must remove material");
        assert!(v_after > 0.0, "diff must leave positive volume");
        kerf_topo::validate(&result.topo).unwrap();
    }
}
