//! `frustum_faceted(r_bot, r_top, h, n)` — n-sided faceted frustum (truncated
//! n-gon prism with two parallel n-gons of different radii).
//!
//! Like [`cylinder_faceted`] vs [`cylinder`]: the lateral surface is `n`
//! planar trapezoids instead of one analytic `Cone` surface. Topology is
//! identical to a prism (V=2n, E=3n, F=n+2) — built via [`extrude_lofted`]
//! with two same-vertex-count n-gons of different radii.
//!
//! [`cylinder_faceted`]: super::cylinder_faceted
//! [`cylinder`]: super::cylinder
//! [`extrude_lofted`]: super::extrude::extrude_lofted

use std::f64::consts::PI;

use kerf_geom::{Point3, Vec3};

use crate::primitives::extrude::extrude_lofted;
use crate::Solid;

/// Build an n-sided faceted frustum: n-gon base of radius `r_bot` at z=0,
/// n-gon top of radius `r_top` at z=`h`. Both n-gons phase-shifted by π/n
/// so n=4 gives axis-aligned squares.
///
/// `r_top` may be larger or smaller than `r_bot` (sloped lateral). Both
/// must be > 0; for `r_top == 0` use `cone_faceted`. (`r_top == r_bot`
/// degenerates to a prism — use `cylinder_faceted`.)
///
/// # Panics
/// In debug builds: any non-positive parameter, `n < 3`.
pub fn frustum_faceted(r_bot: f64, r_top: f64, h: f64, n: usize) -> Solid {
    debug_assert!(r_bot > 0.0, "r_bot must be positive");
    debug_assert!(r_top > 0.0, "r_top must be positive (use cone_faceted for r=0)");
    debug_assert!(h > 0.0, "h must be positive");
    debug_assert!(n >= 3, "n must be at least 3");

    let phase = PI / n as f64;
    let bottom: Vec<Point3> = (0..n)
        .map(|i| {
            let theta = phase + 2.0 * PI * i as f64 / n as f64;
            Point3::new(r_bot * theta.cos(), r_bot * theta.sin(), 0.0)
        })
        .collect();
    let top: Vec<Point3> = (0..n)
        .map(|i| {
            let theta = phase + 2.0 * PI * i as f64 / n as f64;
            Point3::new(r_top * theta.cos(), r_top * theta.sin(), h)
        })
        .collect();
    let _ = Vec3::z(); // keep Vec3 in scope for compiler hints
    extrude_lofted(&bottom, &top)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::solid_volume;
    use kerf_topo::validate;

    #[test]
    fn frustum_faceted_topology_matches_prism() {
        let s = frustum_faceted(1.0, 0.5, 2.0, 8);
        // Same as cylinder_faceted topology: V=2n, E=3n, F=n+2.
        assert_eq!(s.vertex_count(), 16);
        assert_eq!(s.edge_count(), 24);
        assert_eq!(s.face_count(), 10);
        validate(&s.topo).unwrap();
    }

    #[test]
    fn frustum_faceted_volume_matches_formula() {
        // Formula for square frustum: V = h/3 * (A1 + A2 + sqrt(A1*A2))
        // where A1 = bottom area, A2 = top area.
        // For inscribed n-gon area: (n/2) r² sin(2π/n).
        let r_bot = 2.0;
        let r_top = 1.0;
        let h = 4.0;
        let n = 32;
        let s = frustum_faceted(r_bot, r_top, h, n);
        let v = solid_volume(&s);
        let area_of = |r: f64| 0.5 * n as f64 * r * r * (2.0 * std::f64::consts::PI / n as f64).sin();
        let a1 = area_of(r_bot);
        let a2 = area_of(r_top);
        let exp = h / 3.0 * (a1 + a2 + (a1 * a2).sqrt());
        let rel = (v - exp).abs() / exp;
        assert!(rel < 1e-9, "v={v}, exp={exp}, rel={rel}");
    }

    #[test]
    fn inverted_frustum_top_bigger_than_bottom_works() {
        let s = frustum_faceted(0.5, 2.0, 3.0, 16);
        validate(&s.topo).unwrap();
        let v = solid_volume(&s);
        assert!(v > 0.0);
    }
}
