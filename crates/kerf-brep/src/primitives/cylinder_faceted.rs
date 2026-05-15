//! `cylinder_faceted(r, h, n)` — n-gon prism approximation of a cylinder.
//!
//! Unlike [`cylinder`], the lateral surface is a fan of `n` planar quads instead
//! of one analytic `Cylinder` surface. The geometry is fully planar, so it
//! tessellates to a clean STL without the cylinder seam-loop edge case. It also
//! gets the boolean pipeline part-way: lateral faces are quads, so a faceted
//! cylinder against a box only fails when the n-gon edges pierce the box face
//! in its interior (the M11 phase-B limit). Configurations where lateral edges
//! cross box edges, or where the prism is fully nested, do work.
//!
//! ## Cap face tagging
//!
//! The two circular end caps (bottom at z = 0, top at z = h) are tagged with
//! `face_owner_tag = "cylinder_cap"`. This hint survives boolean operations
//! (the stitch pipeline propagates owner tags through kept faces) and lets
//! downstream code (section view, mass properties, drawing projection) identify
//! which flat faces originated as circular cylinder ends rather than as
//! intersection-produced planar cuts.

use kerf_geom::{Point3, Vec3};

use crate::geometry::SurfaceKind;
use crate::primitives::extrude_polygon;
use crate::Solid;

/// Tag applied to the top and bottom cap faces of a `cylinder_faceted` solid.
/// Downstream consumers (section view, mass properties, drawings) can pattern-
/// match on this string to distinguish circular caps from arbitrary planar cuts.
pub const CYLINDER_CAP_TAG: &str = "cylinder_cap";

/// Build an `n`-gon prism inscribed in a cylinder of radius `r` and height `h`.
///
/// The base is a regular `n`-gon centered on the origin in the xy-plane,
/// extruded along `+z`. Higher `n` ⇒ closer to a true cylinder; the result has
/// `2n` vertices, `3n` edges, and `n + 2` faces.
///
/// The two cap faces (bottom at z = 0, top at z = h) are tagged with
/// [`CYLINDER_CAP_TAG`] in `face_owner_tag` so section views and mass-property
/// integrators can identify them without polygon-vertex counting.
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
    let mut solid = extrude_polygon(&profile, Vec3::new(0.0, 0.0, h));

    // Tag cap faces: faces whose surface normal is axis-aligned (|nz| ≈ 1).
    // For a prism extruded along +z, the two cap planes have normals ±(0,0,1);
    // all lateral quad faces have normals in the xy-plane (nz ≈ 0).
    let face_ids: Vec<_> = solid.topo.face_ids().collect();
    for fid in face_ids {
        if let Some(SurfaceKind::Plane(plane)) = solid.face_geom.get(fid) {
            if plane.frame.z.z.abs() > 0.99 {
                solid.face_owner_tag.insert(fid, CYLINDER_CAP_TAG.to_string());
            }
        }
    }

    solid
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
