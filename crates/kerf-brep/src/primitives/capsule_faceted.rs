//! `capsule_faceted(r, h, m)` — capsule primitive via `revolve_polyline`.
//!
//! A capsule is a cylinder of radius `r` and body height `h` capped by two
//! hemispherical end caps, each approximated by `m` latitude rings.
//!
//! # Geometry
//! The profile is a piecewise-linear polyline in the xz-plane:
//!
//! ```text
//!   (0, 2r+h)   ← top pole (on z-axis)
//!       ·
//!      / ← top-cap arc (m-1 interior pts, angles step..π/2-step)
//!     ·
//!     |  ← cylinder wall (only if h > 0)
//!     ·
//!      \ ← bottom-cap arc (m pts, angles step..π/2)
//!       ·
//!   (0, 0)      ← bottom pole (on z-axis)
//! ```
//!
//! Interior arc points all have x > 0 (required by `revolve_polyline`).
//!
//! # Topology (via revolve_polyline, N = profile length)
//!
//! | Case   | N       | V       | E      | F      |
//! |--------|---------|---------|--------|--------|
//! | h > 0  | 2m + 2  | 2m + 2  | 4m + 1 | 2m + 1 |
//! | h = 0  | 2m + 1  | 2m + 1  | 4m - 1 | 2m     |
//!
//! Euler check: V - E + F = (2m+2) - (4m+1) + (2m+1) = 2 ✓

use std::f64::consts::FRAC_PI_2;

use kerf_geom::Point3;

use crate::primitives::revolve_polyline;
use crate::Solid;

/// Build a capsule: a cylinder of radius `r` and body height `h` capped with
/// two hemispherical end caps, each approximated by `m` latitude divisions.
///
/// Total height of the solid = `2·r + h`. Setting `h = 0` produces a sphere
/// approximation (bicone at m=1, octahedron at m=2, …).
///
/// # Panics (debug)
/// Panics in debug mode if `r <= 0`, `h < 0`, or `m < 1`.
pub fn capsule_faceted(r: f64, h: f64, m: usize) -> Solid {
    debug_assert!(r > 0.0, "radius must be positive");
    debug_assert!(h >= 0.0, "height must be non-negative");
    debug_assert!(m >= 1, "m must be at least 1");

    let step = FRAC_PI_2 / m as f64;
    let mut profile: Vec<Point3> = Vec::new();

    // Bottom pole (on z-axis)
    profile.push(Point3::new(0.0, 0.0, 0.0));

    // Bottom hemisphere arc: m points at angles step·1 through step·m = π/2.
    // At i=m: angle = π/2, giving (r, 0, r) — the bottom-to-cylinder junction.
    for i in 1..=m {
        let angle = i as f64 * step;
        let x = r * angle.sin();
        let z = r * (1.0 - angle.cos());
        profile.push(Point3::new(x, 0.0, z));
    }

    // Cylinder top junction — only when a body is present.
    if h > 0.0 {
        profile.push(Point3::new(r, 0.0, r + h));
    }

    // Top hemisphere arc: m-1 interior points at angles step·1 through step·(m-1).
    // Angles measured from the equator upward, so x = r·cos(angle) decreases
    // toward zero, and z rises from r+h toward 2r+h.
    for i in 1..m {
        let angle = i as f64 * step;
        let x = r * angle.cos();
        let z = (r + h) + r * angle.sin();
        profile.push(Point3::new(x, 0.0, z));
    }

    // Top pole (on z-axis)
    profile.push(Point3::new(0.0, 0.0, 2.0 * r + h));

    revolve_polyline(&profile)
}

#[cfg(test)]
mod tests {
    use std::io::Cursor;
    use std::f64::consts::PI;

    use super::*;
    use kerf_topo::validate;

    use crate::serde_io::{read_json, write_json};
    use crate::{read_stl_binary_to_solid, solid_volume, tessellate, write_binary};

    // ── helpers ────────────────────────────────────────────────────────────────

    /// Expected topology counts for capsule_faceted(r, h, m):
    ///   h > 0: N = 2m+2, V = 2m+2, E = 4m+1, F = 2m+1
    ///   h = 0: N = 2m+1, V = 2m+1, E = 4m-1, F = 2m
    fn expected_vef(m: usize, has_body: bool) -> (usize, usize, usize) {
        if has_body {
            (2 * m + 2, 4 * m + 1, 2 * m + 1)
        } else {
            (2 * m + 1, 4 * m - 1, 2 * m)
        }
    }

    /// Tessellate a solid, export STL, re-import, return the imported mesh solid.
    fn import_tessellated(s: &Solid, segs: usize) -> Solid {
        let soup = tessellate(s, segs);
        let mut buf = Vec::new();
        write_binary(&soup, "capsule", &mut buf).expect("write_binary");
        let mut reader = Cursor::new(buf);
        read_stl_binary_to_solid(&mut reader).expect("read_stl_binary_to_solid")
    }

    // ── topology ───────────────────────────────────────────────────────────────

    #[test]
    fn capsule_m1_h0_is_bicone() {
        // m=1, h=0: 3-point profile → diamond (bipyramid)
        let s = capsule_faceted(1.0, 0.0, 1);
        let (v, e, f) = expected_vef(1, false);
        assert_eq!(s.vertex_count(), v, "vertex count");
        assert_eq!(s.edge_count(), e, "edge count");
        assert_eq!(s.face_count(), f, "face count");
        validate(&s.topo).unwrap();
    }

    #[test]
    fn capsule_m4_h0_topology() {
        // m=4, h=0: N=9, V=9, E=15, F=8
        let s = capsule_faceted(1.0, 0.0, 4);
        let (v, e, f) = expected_vef(4, false);
        assert_eq!(s.vertex_count(), v, "vertex count");
        assert_eq!(s.edge_count(), e, "edge count");
        assert_eq!(s.face_count(), f, "face count");
        validate(&s.topo).unwrap();
    }

    #[test]
    fn capsule_m4_h2_topology() {
        // m=4, h=2: N=10, V=10, E=17, F=9
        let s = capsule_faceted(1.0, 2.0, 4);
        let (v, e, f) = expected_vef(4, true);
        assert_eq!(s.vertex_count(), v, "vertex count");
        assert_eq!(s.edge_count(), e, "edge count");
        assert_eq!(s.face_count(), f, "face count");
        validate(&s.topo).unwrap();
    }

    #[test]
    fn capsule_m8_h3_topology() {
        let s = capsule_faceted(0.5, 3.0, 8);
        let (v, e, f) = expected_vef(8, true);
        assert_eq!(s.vertex_count(), v, "vertex count");
        assert_eq!(s.edge_count(), e, "edge count");
        assert_eq!(s.face_count(), f, "face count");
        validate(&s.topo).unwrap();
    }

    // ── volume (via tessellation + import) ─────────────────────────────────────

    #[test]
    fn capsule_volume_approximates_analytic() {
        // Capsule: r=0.5, h=1.0, m=16
        // Analytic: V = π·r²·h + (4/3)·π·r³
        //         = π·0.25·1.0 + (4/3)·π·0.125 ≈ 1.309
        let r = 0.5;
        let h = 1.0;
        let s = capsule_faceted(r, h, 16);
        let imported = import_tessellated(&s, 32);
        let v = solid_volume(&imported);
        let analytic = PI * r * r * h + (4.0 / 3.0) * PI * r.powi(3);
        assert!(v > 0.0, "volume must be positive, got {v}");
        assert!(
            (v - analytic).abs() < analytic * 0.05,
            "capsule vol {v:.4} too far from analytic {analytic:.4} (>5%)"
        );
    }

    #[test]
    fn capsule_sphere_volume_approximates_analytic() {
        // h=0 capsule → sphere approximation. r=1.0, m=12.
        // Analytic sphere: V = (4/3)·π·r³ ≈ 4.189
        let r = 1.0;
        let s = capsule_faceted(r, 0.0, 12);
        let imported = import_tessellated(&s, 32);
        let v = solid_volume(&imported);
        let analytic = (4.0 / 3.0) * PI * r.powi(3);
        assert!(v > 0.0, "volume must be positive, got {v}");
        assert!(
            (v - analytic).abs() < analytic * 0.05,
            "sphere-cap vol {v:.4} too far from analytic {analytic:.4} (>5%)"
        );
    }

    // ── JSON round-trip ────────────────────────────────────────────────────────

    #[test]
    fn capsule_json_roundtrip_preserves_topology() {
        let s = capsule_faceted(0.75, 1.5, 6);
        validate(&s.topo).unwrap();

        let mut buf = Vec::new();
        write_json(&s, &mut buf).expect("serialize");
        let s2 = read_json(&mut buf.as_slice()).expect("deserialize");

        validate(&s2.topo).unwrap();
        assert_eq!(s.vertex_count(), s2.vertex_count(), "vertex count after roundtrip");
        assert_eq!(s.edge_count(), s2.edge_count(), "edge count after roundtrip");
        assert_eq!(s.face_count(), s2.face_count(), "face count after roundtrip");
    }
}
