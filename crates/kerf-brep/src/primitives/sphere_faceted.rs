//! `sphere_faceted(r, n)` — polyhedral UV-sphere approximation.
//!
//! Unlike [`sphere`], the surface is a fan of planar triangular faces — no
//! analytic `Sphere` surface kind. The geometry is fully planar, so it feeds
//! directly into the boolean pipeline without needing the curved-surface
//! intersection path.
//!
//! The mesh uses the same UV parameterisation as `tessellate`'s sphere branch:
//! `u` = longitude (0 → 2π CCW around +z), `v` = colatitude (0 = north pole,
//! π = south pole). Triangle winding is CCW from the outward normal (the
//! divergence-theorem convention used throughout kerf).

use std::f64::consts::PI;

use kerf_geom::Point3;

use crate::mesh_import::from_triangles;
use crate::Solid;

/// Build a polyhedral UV-sphere of radius `r` with `n` longitude segments.
///
/// The number of latitude stacks is `(n / 2).max(2)`. Higher `n` gives a
/// closer approximation to the analytic sphere; `n = 4` produces a bicone,
/// `n = 16` is already visually round.
///
/// The result has `(n/2 - 1) * n + 2` vertices, all-triangular faces, and
/// `SurfaceKind::Plane` on every face, so it plays well with the planar
/// boolean pipeline.
///
/// # Panics
/// Debug builds: if `r <= 0` or `n < 4`.
pub fn sphere_faceted(r: f64, n: usize) -> Solid {
    debug_assert!(r > 0.0, "radius must be positive");
    debug_assert!(n >= 4, "n must be at least 4");

    let n_lon = n;
    let n_lat = (n / 2).max(2);

    let dlon = 2.0 * PI / n_lon as f64;
    let dlat = PI / n_lat as f64;

    // vertex on the unit sphere at colatitude v, longitude u, scaled by r.
    let pt = |u: f64, v: f64| -> Point3 {
        Point3::new(r * v.sin() * u.cos(), r * v.sin() * u.sin(), r * v.cos())
    };

    let mut tris: Vec<[Point3; 3]> = Vec::new();

    for j in 0..n_lat {
        let v0 = j as f64 * dlat;
        let v1 = (j + 1) as f64 * dlat;
        let north = j == 0;
        let south = j == n_lat - 1;

        for i in 0..n_lon {
            let u0 = i as f64 * dlon;
            let u1 = ((i + 1) % n_lon) as f64 * dlon;
            let p00 = pt(u0, v0); // upper-left  (or north pole for j==0)
            let p10 = pt(u1, v0); // upper-right (same pole, different u)
            let p11 = pt(u1, v1); // lower-right
            let p01 = pt(u0, v1); // lower-left  (or south pole for j==n_lat-1)

            if north {
                // p00 == p10 == north pole; emit one triangle per longitude slice.
                tris.push([p00, p01, p11]);
            } else if south {
                // p01 == p11 == south pole; emit one triangle per longitude slice.
                tris.push([p00, p01, p10]);
            } else {
                // Interior quad → two triangles (CCW outward winding).
                tris.push([p00, p01, p11]);
                tris.push([p00, p11, p10]);
            }
        }
    }

    from_triangles(&tris).expect("sphere_faceted: produced non-manifold mesh")
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    fn analytic_sphere_vol(r: f64) -> f64 {
        4.0 / 3.0 * PI * r * r * r
    }

    #[test]
    fn sphere_faceted_topology_validates() {
        let s = sphere_faceted(1.0, 8);
        kerf_topo::validate(&s.topo).unwrap();
    }

    #[test]
    fn sphere_faceted_euler_holds() {
        // For n=8: n_lat=4, n_lon=8.
        // V = 2 + (n_lat-1)*n_lon = 2 + 3*8 = 26
        // F = 2*n_lon*(n_lat-1) = 2*8*3 = 48
        // E = V + F - 2 = 72
        let s = sphere_faceted(1.0, 8);
        let v = s.vertex_count() as i64;
        let e = s.edge_count() as i64;
        let f = s.face_count() as i64;
        assert_eq!(v - e + f, 2, "V={v} E={e} F={f}");
        assert_eq!(v, 26);
        assert_eq!(f, 48);
    }

    #[test]
    fn sphere_faceted_volume_approaches_analytic() {
        let r = 1.0;
        let analytic = analytic_sphere_vol(r);
        // Polyhedral approx always underestimates the sphere.
        let v16 = crate::measure::solid_volume(&sphere_faceted(r, 16));
        let v32 = crate::measure::solid_volume(&sphere_faceted(r, 32));
        assert!(v16 < analytic, "n=16 vol {v16} >= analytic {analytic}");
        assert!(v32 < analytic, "n=32 vol {v32} >= analytic {analytic}");
        // n=16 (8 stacks) approximates to within ~8%, n=32 to within ~2%.
        // UV spheres inscribe the analytic sphere; each face is a chord and the
        // error is O(1/n²). These bounds are empirically verified.
        assert!(
            analytic - v16 < analytic * 0.08,
            "n=16 too far from analytic: vol={v16} analytic={analytic}"
        );
        assert!(
            analytic - v32 < analytic * 0.03,
            "n=32 too far from analytic: vol={v32} analytic={analytic}"
        );
        // Larger n gives a better approximation (monotone convergence).
        assert!(v32 > v16, "n=32 should be closer to analytic than n=16");
    }

    #[test]
    fn sphere_faceted_volume_scales_with_radius() {
        // V ∝ r³: doubling radius → 8× volume.
        let v1 = crate::measure::solid_volume(&sphere_faceted(1.0, 16));
        let v2 = crate::measure::solid_volume(&sphere_faceted(2.0, 16));
        let ratio = v2 / v1;
        assert!(
            (ratio - 8.0).abs() < 1e-6,
            "volume ratio {ratio} ≠ 8.0"
        );
    }

    #[test]
    fn sphere_faceted_json_roundtrip() {
        use crate::serde_io::{read_json, write_json};
        let s = sphere_faceted(1.0, 12);
        let mut buf = Vec::new();
        write_json(&s, &mut buf).unwrap();
        let s2 = read_json(&mut buf.as_slice()).unwrap();
        assert_eq!(s.vertex_count(), s2.vertex_count());
        assert_eq!(s.edge_count(), s2.edge_count());
        assert_eq!(s.face_count(), s2.face_count());
        kerf_topo::validate(&s2.topo).unwrap();
        let v1 = crate::measure::solid_volume(&s);
        let v2 = crate::measure::solid_volume(&s2);
        assert!((v1 - v2).abs() < 1e-9, "JSON roundtrip volume drift: {v1} → {v2}");
    }

    #[test]
    fn sphere_faceted_boolean_with_box() {
        use crate::primitives::box_at;
        use kerf_geom::{Point3, Vec3};
        // Carve a box-shaped chunk from a faceted sphere. The sphere is fully
        // planar so the boolean pipeline handles it without curved-face paths.
        let s = sphere_faceted(1.0, 16);
        let cutter = box_at(
            Vec3::new(0.4, 0.4, 0.4),
            Point3::new(-0.2, -0.2, -0.2),
        );
        let v_before = crate::measure::solid_volume(&s);
        let cutter_vol = 0.4_f64.powi(3);
        let result = s.try_difference(&cutter).expect("sphere_faceted − box");
        let v_after = crate::measure::solid_volume(&result);
        let expected = v_before - cutter_vol;
        assert!(
            (v_after - expected).abs() < 0.05,
            "sphere_faceted − box: got {v_after}, expected {expected}"
        );
        kerf_topo::validate(&result.topo).unwrap();
    }

    #[test]
    fn sphere_faceted_minimum_n4_is_valid() {
        // n=4 → bicone (2 stacks × 4 slices): V=6, F=8, E=12.
        let s = sphere_faceted(1.0, 4);
        kerf_topo::validate(&s.topo).unwrap();
        assert_eq!(s.vertex_count(), 6);
        assert_eq!(s.face_count(), 8);
        assert_eq!(s.edge_count(), 12);
    }
}
