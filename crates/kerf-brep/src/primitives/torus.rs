//! `torus(major_radius, minor_radius)` constructor.
//!
//! Topology (Euler-Poincaré: V=1 E=2 F=1 R=0 S=1 → 1-2+1-0 = 0 = 2*(1-1) ✓, genus G=1):
//!   - 1 vertex v_anchor at (R+r, 0, 0) — the single point on the outer equator.
//!   - 2 edges (self-loops at v_anchor): `meridian` and `equator`.
//!   - 1 face `surface` with outer_loop = 4 half-edges: mer_a → eq_a → mer_b → eq_b.
//!   - 1 shell.

use std::f64::consts::TAU;

use kerf_geom::{Circle, Frame, Point3, Torus as TorusSurface, Vec3};
use kerf_topo::validate;

use crate::geometry::{CurveKind, CurveSegment, Sense, SurfaceKind};
use crate::Solid;

/// Torus with major radius `R` (distance from axis to tube center) and minor
/// radius `r` (tube radius). The central axis is +z; the anchor point is
/// `(R+r, 0, 0)`.
///
/// # Panics (debug)
/// Panics in debug mode if `major_radius <= minor_radius` or `minor_radius <= 0`.
pub fn torus(major_radius: f64, minor_radius: f64) -> Solid {
    debug_assert!(
        minor_radius > 0.0 && major_radius > minor_radius,
        "torus requires major_radius > minor_radius > 0"
    );
    let mut s = Solid::new();

    // ---- 1. Single anchor vertex on the outer equator. ----
    let v = s.topo.build_insert_vertex();
    let p_anchor = Point3::new(major_radius + minor_radius, 0.0, 0.0);
    s.vertex_geom.insert(v, p_anchor);

    // ---- 2. Solid + Shell. ----
    let solid_id = s.topo.build_insert_solid();
    s.topo.build_set_active_solid(Some(solid_id));
    let shell_id = s.topo.build_insert_shell(solid_id);

    // ---- 3. Loop placeholder + Face. ----
    let loop_id = s.topo.build_insert_loop_placeholder();
    let face_id = s.topo.build_insert_face(loop_id, shell_id);
    s.topo.build_set_loop_face(loop_id, face_id);
    s.topo.build_push_shell_face(shell_id, face_id);

    // ---- 4. Four half-edges, all with origin = v_anchor and loop = outer loop. ----
    let mer_a = s.topo.build_insert_half_edge(v, loop_id);
    let eq_a  = s.topo.build_insert_half_edge(v, loop_id);
    let mer_b = s.topo.build_insert_half_edge(v, loop_id);
    let eq_b  = s.topo.build_insert_half_edge(v, loop_id);

    // ---- 5. Link them into a 4-cycle: mer_a → eq_a → mer_b → eq_b → mer_a. ----
    s.topo.build_set_half_edge_next_prev(mer_a, eq_a);
    s.topo.build_set_half_edge_next_prev(eq_a,  mer_b);
    s.topo.build_set_half_edge_next_prev(mer_b, eq_b);
    s.topo.build_set_half_edge_next_prev(eq_b,  mer_a);

    // ---- 6. Edges and twin assignments. ----
    let edge_mer = s.topo.build_insert_edge([mer_a, mer_b]);
    s.topo.build_set_half_edge_twin(mer_a, mer_b);
    s.topo.build_set_half_edge_twin(mer_b, mer_a);
    s.topo.build_set_half_edge_edge(mer_a, edge_mer);
    s.topo.build_set_half_edge_edge(mer_b, edge_mer);

    let edge_eq = s.topo.build_insert_edge([eq_a, eq_b]);
    s.topo.build_set_half_edge_twin(eq_a,  eq_b);
    s.topo.build_set_half_edge_twin(eq_b,  eq_a);
    s.topo.build_set_half_edge_edge(eq_a,  edge_eq);
    s.topo.build_set_half_edge_edge(eq_b,  edge_eq);

    // ---- 7. Set loop's representative half-edge and vertex outgoing. ----
    s.topo.build_set_loop_half_edge(loop_id, Some(mer_a));
    s.topo.build_set_vertex_outgoing(v, Some(mer_a));

    // ---- 8. Edge geometry. ----
    // Meridian: small circle around the tube in the y=0 plane.
    //   center = (R, 0, 0), normal = +y, frame.x = +x so point_at(0) = (R+r, 0, 0).
    //   frame.y = +z × +x = ... we need frame.z = +y, frame.x = +x, frame.y = frame.z × frame.x
    //   Actually: point_at(t) = center + r*(cos(t)*frame.x + sin(t)*frame.y)
    //   At t=0: (R,0,0) + r*(1,0,0) = (R+r, 0, 0). ✓
    //   frame.x = +x, frame.y = -z, frame.z = +y  (right-handed: x × (-z) = x × (-z) = y ✓)
    let meridian_frame = Frame {
        origin: Point3::new(major_radius, 0.0, 0.0),
        x: Vec3::x(),
        y: -Vec3::z(),
        z: Vec3::y(),
    };
    s.edge_geom.insert(edge_mer, CurveSegment {
        curve: CurveKind::Circle(Circle::new(meridian_frame, minor_radius)),
        range: (0.0, TAU),
        sense: Sense::Forward,
    });

    // Equator: large circle around the z-axis in the z=0 plane.
    //   center = origin, radius = R+r, frame = world (x/y/z).
    //   point_at(0) = (R+r, 0, 0). ✓
    let equator_frame = Frame::world(Point3::origin());
    s.edge_geom.insert(edge_eq, CurveSegment {
        curve: CurveKind::Circle(Circle::new(equator_frame, major_radius + minor_radius)),
        range: (0.0, TAU),
        sense: Sense::Forward,
    });

    // ---- 9. Face surface. ----
    let torus_surface = TorusSurface::new(Frame::world(Point3::origin()), major_radius, minor_radius);
    s.face_geom.insert(face_id, SurfaceKind::Torus(torus_surface));

    // ---- 10. Validate. ----
    validate(&s.topo).expect("torus topology violates Euler invariant");

    s
}

#[cfg(test)]
mod tests {
    use super::*;
    use kerf_topo::validate;

    use crate::geometry::{CurveKind, SurfaceKind};

    #[test]
    fn unit_torus_has_correct_topology() {
        let s = torus(3.0, 1.0);
        assert_eq!(s.vertex_count(), 1);
        assert_eq!(s.edge_count(), 2);
        assert_eq!(s.face_count(), 1);
        assert_eq!(s.shell_count(), 1);
    }

    #[test]
    fn torus_validates_with_genus_1() {
        let s = torus(3.0, 1.0);
        validate(&s.topo).unwrap();
    }

    #[test]
    fn torus_has_two_circle_edges_one_torus_face() {
        let s = torus(3.0, 1.0);
        let mut circles = 0;
        let mut tori = 0;
        for (_, seg) in &s.edge_geom {
            if matches!(seg.curve, CurveKind::Circle(_)) {
                circles += 1;
            }
        }
        for (_, surf) in &s.face_geom {
            if matches!(surf, SurfaceKind::Torus(_)) {
                tori += 1;
            }
        }
        assert_eq!(circles, 2);
        assert_eq!(tori, 1);
    }
}
