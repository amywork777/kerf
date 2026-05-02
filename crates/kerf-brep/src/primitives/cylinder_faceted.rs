//! `cylinder_faceted(r, h, n)` — n-gon prism approximation of a cylinder.
//!
//! Unlike [`cylinder`], the lateral surface is a fan of `n` planar quads instead
//! of one analytic `Cylinder` surface. The geometry is fully planar, so it
//! tessellates to a clean STL without the cylinder seam-loop edge case. It also
//! gets the boolean pipeline part-way: lateral faces are quads, so a faceted
//! cylinder against a box only fails when the n-gon edges pierce the box face
//! in its interior (the M11 phase-B limit). Configurations where lateral edges
//! cross box edges, or where the prism is fully nested, do work.

use kerf_geom::{Point3, Vec3};

use crate::primitives::extrude_polygon;
use crate::Solid;

/// Build an `n`-gon prism inscribed in a cylinder of radius `r` and height `h`.
///
/// The base is a regular `n`-gon centered on the origin in the xy-plane,
/// extruded along `+z`. Higher `n` ⇒ closer to a true cylinder; the result has
/// `2n` vertices, `3n` edges, and `n + 2` faces.
///
/// # Panics
/// In debug builds: if `r <= 0`, `h <= 0`, or `n < 3`.
pub fn cylinder_faceted(r: f64, h: f64, n: usize) -> Solid {
    debug_assert!(r > 0.0, "radius must be positive");
    debug_assert!(h > 0.0, "height must be positive");
    debug_assert!(n >= 3, "n must be at least 3");

    // Start at θ = π/n so that for n = 4 the result is an axis-aligned square
    // prism (vertices at (±s, ±s)) rather than a diamond. For other n, this is
    // just a half-step phase offset.
    let phase = std::f64::consts::PI / n as f64;
    let profile: Vec<Point3> = (0..n)
        .map(|i| {
            let theta = phase + 2.0 * std::f64::consts::PI * i as f64 / n as f64;
            Point3::new(r * theta.cos(), r * theta.sin(), 0.0)
        })
        .collect();
    extrude_polygon(&profile, Vec3::new(0.0, 0.0, h))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cylinder_faceted_8gon_topology() {
        let s = cylinder_faceted(1.0, 2.0, 8);
        assert_eq!(s.vertex_count(), 16);
        assert_eq!(s.edge_count(), 24);
        assert_eq!(s.face_count(), 10);
        kerf_topo::validate(&s.topo).unwrap();
    }

    #[test]
    fn cylinder_faceted_24gon_topology() {
        // High-poly approximation: 24 lateral faces + 2 caps.
        let s = cylinder_faceted(1.0, 2.0, 24);
        assert_eq!(s.vertex_count(), 48);
        assert_eq!(s.edge_count(), 72);
        assert_eq!(s.face_count(), 26);
        kerf_topo::validate(&s.topo).unwrap();
    }
}
