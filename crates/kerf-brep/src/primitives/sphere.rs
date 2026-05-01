//! `sphere(radius)` constructor.
//!
//! Topology (Euler: V=1 E=0 F=1 R=0 S=1 → 1-0+1-0 = 2 = 2*1 ✓):
//!   - 1 vertex: anchor at (radius, 0, 0).
//!   - 0 edges (sphere has no boundary).
//!   - 1 face `surface` with outer_loop whose half_edge is None.
//!   - 1 shell.
//!
//! The validator skips empty loops, so the empty outer_loop is valid.

use kerf_geom::{Point3, Sphere as SphereSurface};
use kerf_topo::validate;

use crate::geometry::SurfaceKind;
use crate::Solid;

/// Sphere centered at origin with the given radius.
///
/// Topology: 1V, 0E, 1F. The single face has an empty outer loop (no
/// boundary half-edges), which the topology validator skips gracefully.
///
/// # Panics (debug)
/// Panics in debug mode if `radius` is not positive.
pub fn sphere(radius: f64) -> Solid {
    debug_assert!(radius > 0.0);
    let mut s = Solid::new();

    // ---- 1. Anchor vertex on the sphere. ----
    let v_anchor = s.topo.build_insert_vertex();
    s.vertex_geom.insert(v_anchor, Point3::new(radius, 0.0, 0.0));

    // ---- 2. Solid + Shell. ----
    let solid_id = s.topo.build_insert_solid();
    s.topo.build_set_active_solid(Some(solid_id));
    let shell_id = s.topo.build_insert_shell(solid_id);

    // ---- 3. Empty loop + face. ----
    // build_insert_loop_placeholder creates a loop with half_edge: None.
    let loop_id = s.topo.build_insert_loop_placeholder();
    let face_id = s.topo.build_insert_face(loop_id, shell_id);
    s.topo.build_set_face_shell(face_id, shell_id);
    s.topo.build_push_shell_face(shell_id, face_id);
    s.topo.build_set_loop_face(loop_id, face_id);

    // ---- 4. Surface geometry. ----
    let surface = SphereSurface::at_origin(radius);
    s.face_geom.insert(face_id, SurfaceKind::Sphere(surface));

    validate(&s.topo).expect("sphere topology violates Euler invariant");

    s
}

#[cfg(test)]
mod tests {
    use super::*;
    use kerf_topo::validate;

    use crate::geometry::SurfaceKind;

    #[test]
    fn unit_sphere_has_correct_topology() {
        let s = sphere(1.0);
        assert_eq!(s.vertex_count(), 1);
        assert_eq!(s.edge_count(), 0);
        assert_eq!(s.face_count(), 1);
        assert_eq!(s.shell_count(), 1);
    }

    #[test]
    fn sphere_validates() {
        let s = sphere(2.5);
        validate(&s.topo).unwrap();
    }

    #[test]
    fn sphere_face_has_sphere_surface() {
        let s = sphere(1.0);
        let face_id = s.topo.face_ids().next().unwrap();
        match s.face_geom.get(face_id).unwrap() {
            SurfaceKind::Sphere(_) => {}
            other => panic!("expected Sphere surface, got {other:?}"),
        }
    }
}
