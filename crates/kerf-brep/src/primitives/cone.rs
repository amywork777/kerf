//! `cone(base_radius, height)` constructor — cone primitive.
//!
//! Topology (Euler: V=2 E=2 F=2 R=0 S=1 → 2-2+2-0 = 2 = 2*1 ✓):
//!   - 2 vertices: apex at (0, 0, h), seam_bot at (r, 0, 0).
//!   - 2 edges:
//!     - bot_circle: self-loop at seam_bot. Circle in z=0.
//!     - seam: regular edge apex ↔ seam_bot.
//!   - 2 faces:
//!     - bot_disk: 1 half-edge loop (bot_circle_he_a). Plane normal -z.
//!     - lateral: 3 half-edge loop (seam_he_a apex→seam_bot, bot_circle_he_b at
//!       seam_bot, seam_he_b seam_bot→apex).
//!
//! Direct slotmap construction via `kerf_topo::build` module; bypasses
//! Euler-operator restrictions on self-loops.

use std::f64::consts::TAU;

use kerf_geom::{Circle, Cone as ConeSurface, Frame, Line, Plane, Point3, Vec3};
use kerf_topo::validate;

use crate::Solid;
use crate::geometry::{CurveKind, CurveSegment, Sense, SurfaceKind};

/// Construct a right circular cone with base center at the origin, apex at
/// (0, 0, `height`), and base `radius`. The seam runs from the apex to
/// (r, 0, 0).
///
/// # Panics (debug)
/// Panics in debug mode if `radius` or `height` is not positive.
pub fn cone(radius: f64, height: f64) -> Solid {
    debug_assert!(radius > 0.0 && height > 0.0);
    let mut s = Solid::new();

    // ---- 1. Vertices ----
    let v_apex = s.topo.build_insert_vertex();
    let v_bot = s.topo.build_insert_vertex();
    s.vertex_geom.insert(v_apex, Point3::new(0.0, 0.0, height));
    s.vertex_geom.insert(v_bot, Point3::new(radius, 0.0, 0.0));

    // ---- 2. Solid + Shell ----
    let solid_id = s.topo.build_insert_solid();
    s.topo.build_set_active_solid(Some(solid_id));
    let shell_id = s.topo.build_insert_shell(solid_id);

    // ---- 3. Loops + Faces ----
    // bot_disk face
    let bot_loop = s.topo.build_insert_loop_placeholder();
    let bot_face = s.topo.build_insert_face(bot_loop, shell_id);
    s.topo.build_set_loop_face(bot_loop, bot_face);
    s.topo.build_push_shell_face(shell_id, bot_face);

    // lateral face
    let lat_loop = s.topo.build_insert_loop_placeholder();
    let lat_face = s.topo.build_insert_face(lat_loop, shell_id);
    s.topo.build_set_loop_face(lat_loop, lat_face);
    s.topo.build_push_shell_face(shell_id, lat_face);

    // ---- 4. Half-edges ----
    // bot_disk loop: one self-loop half-edge, origin = v_bot.
    let bot_he_a = s.topo.build_insert_half_edge(v_bot, bot_loop);

    // lateral loop half-edges (3 in order: seam_a, bot_b, seam_b).
    // seam_he_a: origin v_apex (goes to v_bot)
    let seam_he_a = s.topo.build_insert_half_edge(v_apex, lat_loop);
    // bot_circle_he_b: origin v_bot (self-loop goes to v_bot)
    let bot_he_b = s.topo.build_insert_half_edge(v_bot, lat_loop);
    // seam_he_b: origin v_bot (goes back to v_apex)
    let seam_he_b = s.topo.build_insert_half_edge(v_bot, lat_loop);

    // ---- 5. Wire next/prev ----
    // bot_disk: self-loop (next/prev = self)
    s.topo.build_set_half_edge_next_prev(bot_he_a, bot_he_a);
    // lateral: [seam_a → bot_b → seam_b] cyclic
    s.topo.build_set_half_edge_next_prev(seam_he_a, bot_he_b);
    s.topo.build_set_half_edge_next_prev(bot_he_b, seam_he_b);
    s.topo.build_set_half_edge_next_prev(seam_he_b, seam_he_a);

    // ---- 6. Wire loop representative half-edges ----
    s.topo.build_set_loop_half_edge(bot_loop, Some(bot_he_a));
    s.topo.build_set_loop_half_edge(lat_loop, Some(seam_he_a));

    // ---- 7. Twin pairs ----
    // bot circle: he_a (in bot_disk) ↔ he_b (in lateral)
    s.topo.build_set_half_edge_twin(bot_he_a, bot_he_b);
    s.topo.build_set_half_edge_twin(bot_he_b, bot_he_a);
    // seam: he_a ↔ he_b (sticker — both in lateral)
    s.topo.build_set_half_edge_twin(seam_he_a, seam_he_b);
    s.topo.build_set_half_edge_twin(seam_he_b, seam_he_a);

    // ---- 8. Edges ----
    let edge_bot_circle = s.topo.build_insert_edge([bot_he_a, bot_he_b]);
    s.topo.build_set_half_edge_edge(bot_he_a, edge_bot_circle);
    s.topo.build_set_half_edge_edge(bot_he_b, edge_bot_circle);

    let edge_seam = s.topo.build_insert_edge([seam_he_a, seam_he_b]);
    s.topo.build_set_half_edge_edge(seam_he_a, edge_seam);
    s.topo.build_set_half_edge_edge(seam_he_b, edge_seam);

    // ---- 9. Vertex outgoing half-edges ----
    s.topo.build_set_vertex_outgoing(v_apex, Some(seam_he_a));
    s.topo.build_set_vertex_outgoing(v_bot, Some(bot_he_a));

    // ---- 10. Edge geometry ----
    // bot_circle: full circle at z=0, center origin, radius r.
    // Frame::world gives x=+x, y=+y, z=+z, so point_at(0) = (r, 0, 0) = v_bot. ✓
    let bot_circle_frame = Frame::world(Point3::origin());
    s.edge_geom.insert(
        edge_bot_circle,
        CurveSegment {
            curve: CurveKind::Circle(Circle::new(bot_circle_frame, radius)),
            range: (0.0, TAU),
            sense: Sense::Forward,
        },
    );

    // seam: line from (r, 0, 0) to apex (0, 0, h).
    // Direction vector: from seam_bot to apex = (-r, 0, h), normalized.
    let seam_origin = Point3::new(radius, 0.0, 0.0);
    let seam_vec = Vec3::new(-radius, 0.0, height);
    let seam_len = seam_vec.norm();
    let seam_dir = seam_vec / seam_len;
    let seam_line =
        Line::from_origin_dir(seam_origin, seam_dir).expect("seam line has valid direction");
    s.edge_geom.insert(
        edge_seam,
        CurveSegment::line(seam_line, 0.0, seam_len),
    );

    // ---- 11. Face geometry ----
    // bot_disk: plane at z=0, normal -z (outward from the enclosed volume).
    let bot_plane_frame = Frame {
        origin: Point3::origin(),
        x: Vec3::x(),
        y: Vec3::y(),
        z: -Vec3::z(),
    };
    s.face_geom
        .insert(bot_face, SurfaceKind::Plane(Plane::new(bot_plane_frame)));

    // lateral: cone surface.
    // frame.origin = apex, frame.z = -z (pointing into cone toward base),
    // half_angle = atan(r/h).
    let cone_frame = Frame {
        origin: Point3::new(0.0, 0.0, height),
        x: Vec3::x(),
        y: Vec3::y(),
        z: -Vec3::z(),
    };
    let half_angle = (radius / height).atan();
    s.face_geom.insert(
        lat_face,
        SurfaceKind::Cone(ConeSurface::new(cone_frame, half_angle)),
    );

    validate(&s.topo).expect("cone topology violates Euler invariant");

    s
}

#[cfg(test)]
mod tests {
    use super::*;
    use kerf_topo::validate;

    use crate::geometry::{CurveKind, SurfaceKind};

    #[test]
    fn unit_cone_has_correct_topology() {
        let s = cone(1.0, 2.0);
        assert_eq!(s.vertex_count(), 2);
        assert_eq!(s.edge_count(), 2);
        assert_eq!(s.face_count(), 2);
        assert_eq!(s.shell_count(), 1);
    }

    #[test]
    fn cone_validates() {
        let s = cone(1.5, 4.0);
        validate(&s.topo).unwrap();
    }

    #[test]
    fn cone_has_one_circle_one_line_one_plane_one_cone_surface() {
        let s = cone(1.0, 2.0);
        let mut circles = 0;
        let mut lines = 0;
        let mut planes = 0;
        let mut cones = 0;
        for (_, seg) in &s.edge_geom {
            match seg.curve {
                CurveKind::Circle(_) => circles += 1,
                CurveKind::Line(_) => lines += 1,
                _ => panic!("unexpected curve"),
            }
        }
        for (_, surf) in &s.face_geom {
            match surf {
                SurfaceKind::Plane(_) => planes += 1,
                SurfaceKind::Cone(_) => cones += 1,
                _ => panic!("unexpected surface"),
            }
        }
        assert_eq!(circles, 1);
        assert_eq!(lines, 1);
        assert_eq!(planes, 1);
        assert_eq!(cones, 1);
    }
}
