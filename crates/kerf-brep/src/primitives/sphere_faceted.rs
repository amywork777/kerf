//! `sphere_faceted(r, n)` — fully-planar UV-sphere approximation.
//!
//! Unlike [`sphere`], every face is a planar triangle (no analytic `Sphere`
//! surface), so the solid works with the planar boolean pipeline wherever the
//! intersection pattern avoids the M11 phase-B interior-endpoint limit.
//!
//! The tessellation uses n longitude slices and m = max(n/2, 2) latitude bands.
//! Topology: V = 2 + n·m, E = 3·n·m, F = 2·n·m.

use std::f64::consts::PI;

use kerf_geom::Point3;

use crate::mesh_import::from_triangles;
use crate::Solid;

/// Build a fully-planar sphere approximation with `n` longitudinal slices.
///
/// The sphere has `radius` and is centred at the origin. Internally uses
/// `n` longitude slices and `m = max(n/2, 2)` latitude bands, producing
/// `2·n·m` triangular faces, all with `Plane` surface geometry.
///
/// # Panics (debug)
/// Panics if `radius <= 0` or `n < 4`.
pub fn sphere_faceted(radius: f64, n: usize) -> Solid {
    debug_assert!(radius > 0.0, "radius must be positive");
    debug_assert!(n >= 4, "n must be at least 4");

    let m = (n / 2).max(2); // latitude bands; always >= 2

    let north = Point3::new(0.0, 0.0, radius);
    let south = Point3::new(0.0, 0.0, -radius);

    // ring[i][j]: latitude band i (0-indexed from top), longitude slice j.
    // theta = pi*(i+1)/(m+1) so theta avoids 0 and pi (the poles).
    let ring: Vec<Vec<Point3>> = (0..m)
        .map(|i| {
            let theta = PI * (i + 1) as f64 / (m + 1) as f64;
            let sin_t = theta.sin();
            let cos_t = theta.cos();
            (0..n)
                .map(|j| {
                    let phi = 2.0 * PI * j as f64 / n as f64;
                    Point3::new(radius * sin_t * phi.cos(), radius * sin_t * phi.sin(), radius * cos_t)
                })
                .collect()
        })
        .collect();

    let mut tris: Vec<[Point3; 3]> = Vec::with_capacity(2 * n * m);

    // North polar cap: n triangles.
    // Winding CCW from outside (normal points away from origin):
    //   north → ring[0][j] → ring[0][(j+1)%n]
    for j in 0..n {
        tris.push([north, ring[0][j], ring[0][(j + 1) % n]]);
    }

    // Equatorial bands: (m-1) bands × n quads × 2 triangles each.
    // Quad (i, j) has corners ring[i][j], ring[i+1][j], ring[i+1][(j+1)%n], ring[i][(j+1)%n].
    // Split diagonal from ring[i][j] to ring[i+1][(j+1)%n]:
    //   Tri1: ring[i][j] → ring[i+1][j] → ring[i+1][(j+1)%n]  (normal outward ✓)
    //   Tri2: ring[i][j] → ring[i+1][(j+1)%n] → ring[i][(j+1)%n]  (normal outward ✓)
    for i in 0..m - 1 {
        for j in 0..n {
            let jn = (j + 1) % n;
            tris.push([ring[i][j], ring[i + 1][j], ring[i + 1][jn]]);
            tris.push([ring[i][j], ring[i + 1][jn], ring[i][jn]]);
        }
    }

    // South polar cap: n triangles.
    // Winding CCW from outside (normal points away from origin, i.e. downward):
    //   south → ring[m-1][(j+1)%n] → ring[m-1][j]
    for j in 0..n {
        tris.push([south, ring[m - 1][(j + 1) % n], ring[m - 1][j]]);
    }

    from_triangles(&tris).expect("sphere_faceted: triangle soup must produce valid topology")
}

#[cfg(test)]
mod tests {
    use super::*;
    use kerf_topo::validate;

    use crate::{read_json, solid_volume, write_json};

    #[test]
    fn sphere_faceted_8gon_topology() {
        // n=8, m=4: V=2+8*4=34, E=3*8*4=96, F=2*8*4=64.
        let s = sphere_faceted(1.0, 8);
        assert_eq!(s.vertex_count(), 34);
        assert_eq!(s.edge_count(), 96);
        assert_eq!(s.face_count(), 64);
        validate(&s.topo).unwrap();
    }

    #[test]
    fn sphere_faceted_12gon_topology() {
        // n=12, m=6: V=2+12*6=74, E=3*12*6=216, F=2*12*6=144.
        let s = sphere_faceted(1.0, 12);
        assert_eq!(s.vertex_count(), 74);
        assert_eq!(s.edge_count(), 216);
        assert_eq!(s.face_count(), 144);
        validate(&s.topo).unwrap();
    }

    #[test]
    fn sphere_faceted_minimum_n4_topology() {
        // n=4, m=2: V=2+4*2=10, E=3*4*2=24, F=2*4*2=16.
        let s = sphere_faceted(1.0, 4);
        assert_eq!(s.vertex_count(), 10);
        assert_eq!(s.edge_count(), 24);
        assert_eq!(s.face_count(), 16);
        validate(&s.topo).unwrap();
    }

    #[test]
    fn sphere_faceted_volume_converges_to_analytic() {
        // Analytic volume = 4/3 * pi * r^3. The inscribed (vertices-on-sphere)
        // tessellation always underestimates. At n=24 (576 triangles) the error
        // is ~2.6%; it scales as O(n^-2) so n=96 gives ~0.15%.
        // We accept < 5% here: the test verifies correct orientation and formula.
        let r = 2.0_f64;
        let analytic = 4.0 / 3.0 * std::f64::consts::PI * r.powi(3);
        let s = sphere_faceted(r, 24);
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
    fn sphere_faceted_all_faces_are_planes() {
        use crate::geometry::SurfaceKind;
        let s = sphere_faceted(1.0, 8);
        for (_, surf) in &s.face_geom {
            assert!(matches!(surf, SurfaceKind::Plane(_)), "expected Plane, got {surf:?}");
        }
    }

    #[test]
    fn sphere_faceted_json_roundtrip_preserves_topology_and_volume() {
        let s = sphere_faceted(1.5, 8);
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
