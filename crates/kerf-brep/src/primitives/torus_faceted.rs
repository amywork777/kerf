//! `torus_faceted(major_r, minor_r, n, m)` — fully-planar torus approximation.
//!
//! Unlike [`torus`], every face is a planar triangle (no analytic `Torus`
//! surface), so the solid participates in the planar boolean pipeline.
//!
//! The tessellation uses `n` longitude slices (around the major circle) and
//! `m` tube slices (around the minor circle). Each quad is split into 2
//! triangles via the main diagonal.
//!
//! Topology (genus-1 torus): V = n·m, E = 3·n·m, F = 2·n·m.
//! Euler-Poincaré: V - E + F = 0 for genus-1 ✓.

use std::f64::consts::TAU;

use kerf_geom::Point3;

use crate::mesh_import::from_triangles;
use crate::Solid;

/// Build a fully-planar torus approximation.
///
/// `major_r` is the distance from the torus centre to the tube centre.
/// `minor_r` is the tube radius. `n` longitude slices (≥ 4) wrap around the
/// major circle; `m` tube slices (≥ 4) wrap around the minor circle. All faces
/// are planar triangles with `Plane` surface geometry.
///
/// # Panics (debug)
/// Panics if `major_r <= minor_r`, `minor_r <= 0`, `n < 4`, or `m < 4`.
pub fn torus_faceted(major_r: f64, minor_r: f64, n: usize, m: usize) -> Solid {
    debug_assert!(minor_r > 0.0, "minor_r must be positive");
    debug_assert!(major_r > minor_r, "major_r must exceed minor_r");
    debug_assert!(n >= 4, "n (longitude slices) must be at least 4");
    debug_assert!(m >= 4, "m (tube slices) must be at least 4");

    // v[i][j]: tube angle i (theta = TAU*i/m), longitude j (phi = TAU*j/n).
    // x = (R + r*cos(theta)) * cos(phi)
    // y = (R + r*cos(theta)) * sin(phi)
    // z = r * sin(theta)
    let v: Vec<Vec<Point3>> = (0..m)
        .map(|i| {
            let theta = TAU * i as f64 / m as f64;
            let cos_t = theta.cos();
            let sin_t = theta.sin();
            (0..n)
                .map(|j| {
                    let phi = TAU * j as f64 / n as f64;
                    let rho = major_r + minor_r * cos_t;
                    Point3::new(rho * phi.cos(), rho * phi.sin(), minor_r * sin_t)
                })
                .collect()
        })
        .collect();

    let mut tris: Vec<[Point3; 3]> = Vec::with_capacity(2 * n * m);

    // Each quad (i, j): v[i][j], v[i][(j+1)%n], v[(i+1)%m][(j+1)%n], v[(i+1)%m][j].
    // Split with diagonal from v[i][j] → v[(i+1)%m][(j+1)%n]:
    //   Tri1: v[i][j] → v[i][(j+1)%n] → v[(i+1)%m][(j+1)%n]  (normal outward ✓)
    //   Tri2: v[i][j] → v[(i+1)%m][(j+1)%n] → v[(i+1)%m][j]  (normal outward ✓)
    for i in 0..m {
        let i1 = (i + 1) % m;
        for j in 0..n {
            let j1 = (j + 1) % n;
            tris.push([v[i][j], v[i][j1], v[i1][j1]]);
            tris.push([v[i][j], v[i1][j1], v[i1][j]]);
        }
    }

    from_triangles(&tris).expect("torus_faceted: triangle soup must produce valid topology")
}

#[cfg(test)]
mod tests {
    use super::*;
    use kerf_topo::validate;

    use crate::{read_json, solid_volume, write_json};

    #[test]
    fn torus_faceted_8x8_topology() {
        // n=8, m=8: V=64, E=192, F=128.
        let s = torus_faceted(2.0, 1.0, 8, 8);
        assert_eq!(s.vertex_count(), 64);
        assert_eq!(s.edge_count(), 192);
        assert_eq!(s.face_count(), 128);
        validate(&s.topo).unwrap();
    }

    #[test]
    fn torus_faceted_12x8_topology() {
        // n=12, m=8: V=96, E=288, F=192.
        let s = torus_faceted(3.0, 1.0, 12, 8);
        assert_eq!(s.vertex_count(), 96);
        assert_eq!(s.edge_count(), 288);
        assert_eq!(s.face_count(), 192);
        validate(&s.topo).unwrap();
    }

    #[test]
    fn torus_faceted_minimum_4x4_topology() {
        // n=4, m=4: V=16, E=48, F=32.
        let s = torus_faceted(2.0, 1.0, 4, 4);
        assert_eq!(s.vertex_count(), 16);
        assert_eq!(s.edge_count(), 48);
        assert_eq!(s.face_count(), 32);
        validate(&s.topo).unwrap();
    }

    #[test]
    fn torus_faceted_euler_genus1() {
        // Genus-1 torus: V - E + F = 0.
        let s = torus_faceted(2.0, 0.5, 12, 8);
        let v = s.vertex_count() as i64;
        let e = s.edge_count() as i64;
        let f = s.face_count() as i64;
        assert_eq!(v - e + f, 0, "V={v} E={e} F={f}, expected V-E+F=0");
        validate(&s.topo).unwrap();
    }

    #[test]
    fn torus_faceted_volume_converges_to_analytic() {
        // Analytic volume = 2 * pi^2 * R * r^2. The inscribed tessellation
        // underestimates; at n=24, m=16 (768 triangles) error is ~3.7%.
        // Error scales as O((1/n)^2 + (1/m)^2). Accepting < 5% here.
        let major_r = 2.0_f64;
        let minor_r = 0.5_f64;
        let analytic = 2.0 * std::f64::consts::PI.powi(2) * major_r * minor_r.powi(2);
        let s = torus_faceted(major_r, minor_r, 24, 16);
        let vol = solid_volume(&s);
        let err = (vol - analytic).abs() / analytic;
        assert!(
            vol > 0.0,
            "volume must be positive, got {vol}"
        );
        assert!(
            err < 0.05,
            "volume {vol:.6} deviates {:.2}% from analytic {analytic:.6}",
            err * 100.0
        );
    }

    #[test]
    fn torus_faceted_all_faces_are_planes() {
        use crate::geometry::SurfaceKind;
        let s = torus_faceted(2.0, 0.5, 8, 8);
        for (_, surf) in &s.face_geom {
            assert!(matches!(surf, SurfaceKind::Plane(_)), "expected Plane, got {surf:?}");
        }
    }

    #[test]
    fn torus_faceted_json_roundtrip_preserves_topology_and_volume() {
        let s = torus_faceted(2.0, 0.5, 8, 8);
        let vol_before = solid_volume(&s);

        let mut buf = Vec::new();
        write_json(&s, &mut buf).unwrap();
        let s2 = read_json(&mut buf.as_slice()).unwrap();

        assert_eq!(s.vertex_count(), s2.vertex_count());
        assert_eq!(s.edge_count(), s2.edge_count());
        assert_eq!(s.face_count(), s2.face_count());
        let vol_after = solid_volume(&s2);
        assert!((vol_before - vol_after).abs() < 1e-9, "volume changed across round-trip");
    }
}
