//! `cylinder(radius, height)` constructor — first curved primitive.
//!
//! Topology (Euler: V=2 E=3 F=3 R=0 S=1 → 2-3+3-0 = 2 = 2*1 ✓):
//!   - 2 vertices: v_bot at (r, 0, 0), v_top at (r, 0, h).
//!   - 3 edges:
//!     - bot_circle: self-loop at v_bot. Circle in z=0.
//!     - top_circle: self-loop at v_top. Circle in z=h.
//!     - seam: regular edge v_bot ↔ v_top.
//!   - 3 faces:
//!     - bot_disk: 1 half-edge loop (bot_circle_he_a). Plane normal -z.
//!     - top_disk: 1 half-edge loop (top_circle_he_a). Plane normal +z.
//!     - lateral: 4 half-edge loop (seam_he_a, top_circle_he_b, seam_he_b, bot_circle_he_b).
//!
//! Direct slotmap construction via `kerf_topo::build` module; bypasses
//! Euler-operator restrictions on self-loops.

use std::f64::consts::TAU;

use kerf_geom::{Circle, Cylinder as CylSurface, Frame, Line, Plane, Point3, Vec3};
use kerf_topo::validate;

use crate::Solid;
use crate::geometry::{CurveKind, CurveSegment, Sense, SurfaceKind};

/// Construct a right circular cylinder centered on the Z axis, with bottom at
/// z=0 and top at z=`height`, radius `radius`. The seam runs along (r, 0, z).
///
/// # Panics (debug)
/// Panics in debug mode if `radius` or `height` is not positive.
pub fn cylinder(radius: f64, height: f64) -> Solid {
    debug_assert!(radius > 0.0 && height > 0.0);
    let mut s = Solid::new();

    // ---- 1. Vertices ----
    let v_bot = s.topo.build_insert_vertex();
    let v_top = s.topo.build_insert_vertex();
    s.vertex_geom.insert(v_bot, Point3::new(radius, 0.0, 0.0));
    s.vertex_geom.insert(v_top, Point3::new(radius, 0.0, height));

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

    // top_disk face
    let top_loop = s.topo.build_insert_loop_placeholder();
    let top_face = s.topo.build_insert_face(top_loop, shell_id);
    s.topo.build_set_loop_face(top_loop, top_face);
    s.topo.build_push_shell_face(shell_id, top_face);

    // lateral face
    let lat_loop = s.topo.build_insert_loop_placeholder();
    let lat_face = s.topo.build_insert_face(lat_loop, shell_id);
    s.topo.build_set_loop_face(lat_loop, lat_face);
    s.topo.build_push_shell_face(shell_id, lat_face);

    // ---- 4. Half-edges ----
    // bot_disk loop: one self-loop half-edge, origin = v_bot.
    let bot_he_a = s.topo.build_insert_half_edge(v_bot, bot_loop);
    // top_disk loop: one self-loop half-edge, origin = v_top.
    let top_he_a = s.topo.build_insert_half_edge(v_top, top_loop);

    // lateral loop half-edges (4 in order: seam_a, top_b, seam_b, bot_b).
    // seam_he_a: origin v_bot (goes to v_top)
    let seam_he_a = s.topo.build_insert_half_edge(v_bot, lat_loop);
    // top_circle_he_b: origin v_top (self-loop goes to v_top)
    let top_he_b = s.topo.build_insert_half_edge(v_top, lat_loop);
    // seam_he_b: origin v_top (goes back to v_bot)
    let seam_he_b = s.topo.build_insert_half_edge(v_top, lat_loop);
    // bot_circle_he_b: origin v_bot (self-loop goes to v_bot)
    let bot_he_b = s.topo.build_insert_half_edge(v_bot, lat_loop);

    // ---- 5. Wire next/prev ----
    // bot_disk: self-loop (next/prev = self)
    s.topo.build_set_half_edge_next_prev(bot_he_a, bot_he_a);
    // top_disk: self-loop
    s.topo.build_set_half_edge_next_prev(top_he_a, top_he_a);
    // lateral: [seam_a → top_b → seam_b → bot_b] cyclic
    s.topo.build_set_half_edge_next_prev(seam_he_a, top_he_b);
    s.topo.build_set_half_edge_next_prev(top_he_b, seam_he_b);
    s.topo.build_set_half_edge_next_prev(seam_he_b, bot_he_b);
    s.topo.build_set_half_edge_next_prev(bot_he_b, seam_he_a);

    // ---- 6. Wire loop representative half-edges ----
    s.topo.build_set_loop_half_edge(bot_loop, Some(bot_he_a));
    s.topo.build_set_loop_half_edge(top_loop, Some(top_he_a));
    s.topo.build_set_loop_half_edge(lat_loop, Some(seam_he_a));

    // ---- 7. Twin pairs ----
    // bot circle: he_a (in bot_disk) ↔ he_b (in lateral)
    s.topo.build_set_half_edge_twin(bot_he_a, bot_he_b);
    s.topo.build_set_half_edge_twin(bot_he_b, bot_he_a);
    // top circle: he_a (in top_disk) ↔ he_b (in lateral)
    s.topo.build_set_half_edge_twin(top_he_a, top_he_b);
    s.topo.build_set_half_edge_twin(top_he_b, top_he_a);
    // seam: he_a ↔ he_b (sticker — both in lateral)
    s.topo.build_set_half_edge_twin(seam_he_a, seam_he_b);
    s.topo.build_set_half_edge_twin(seam_he_b, seam_he_a);

    // ---- 8. Edges ----
    let edge_bot_circle = s.topo.build_insert_edge([bot_he_a, bot_he_b]);
    s.topo.build_set_half_edge_edge(bot_he_a, edge_bot_circle);
    s.topo.build_set_half_edge_edge(bot_he_b, edge_bot_circle);

    let edge_top_circle = s.topo.build_insert_edge([top_he_a, top_he_b]);
    s.topo.build_set_half_edge_edge(top_he_a, edge_top_circle);
    s.topo.build_set_half_edge_edge(top_he_b, edge_top_circle);

    let edge_seam = s.topo.build_insert_edge([seam_he_a, seam_he_b]);
    s.topo.build_set_half_edge_edge(seam_he_a, edge_seam);
    s.topo.build_set_half_edge_edge(seam_he_b, edge_seam);

    // ---- 9. Vertex outgoing half-edges ----
    // v_bot: use bot_he_a (it's in the bot circle; seam_he_a would also work)
    s.topo.build_set_vertex_outgoing(v_bot, Some(bot_he_a));
    s.topo.build_set_vertex_outgoing(v_top, Some(top_he_a));

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

    // top_circle: full circle at z=height, center (0,0,h), radius r.
    let top_circle_frame = Frame::world(Point3::new(0.0, 0.0, height));
    s.edge_geom.insert(
        edge_top_circle,
        CurveSegment {
            curve: CurveKind::Circle(Circle::new(top_circle_frame, radius)),
            range: (0.0, TAU),
            sense: Sense::Forward,
        },
    );

    // seam: vertical line from (r, 0, 0) to (r, 0, h).
    let seam_origin = Point3::new(radius, 0.0, 0.0);
    let seam_line =
        Line::from_origin_dir(seam_origin, Vec3::z()).expect("seam line has +z direction");
    s.edge_geom.insert(
        edge_seam,
        CurveSegment::line(seam_line, 0.0, height),
    );

    // ---- 11. Face geometry ----
    // bot_disk: plane at z=0, normal -z (outward from the enclosed volume).
    // Frame x=+x, y=+y, z=-z for outward-facing bottom.
    let bot_plane_frame = Frame {
        origin: Point3::origin(),
        x: Vec3::x(),
        y: Vec3::y(),
        z: -Vec3::z(),
    };
    s.face_geom
        .insert(bot_face, SurfaceKind::Plane(Plane::new(bot_plane_frame)));

    // top_disk: plane at z=height, normal +z (outward).
    let top_plane_frame = Frame::world(Point3::new(0.0, 0.0, height));
    s.face_geom
        .insert(top_face, SurfaceKind::Plane(Plane::new(top_plane_frame)));

    // lateral: cylinder surface, axis +z, radius r, frame at origin.
    let cyl_frame = Frame::world(Point3::origin());
    s.face_geom.insert(
        lat_face,
        SurfaceKind::Cylinder(CylSurface::new(cyl_frame, radius)),
    );

    validate(&s.topo).expect("cylinder topology violates Euler invariant");

    s
}

#[cfg(test)]
mod tests {
    use super::*;
    use kerf_topo::validate;

    #[test]
    fn unit_cylinder_has_correct_topology() {
        let s = cylinder(1.0, 2.0);
        assert_eq!(s.vertex_count(), 2);
        assert_eq!(s.edge_count(), 3);
        assert_eq!(s.face_count(), 3);
        assert_eq!(s.shell_count(), 1);
    }

    #[test]
    fn cylinder_validates() {
        let s = cylinder(1.5, 4.0);
        validate(&s.topo).unwrap();
    }

    #[test]
    fn cylinder_has_one_circle_edge_per_cap_and_one_line_seam() {
        let s = cylinder(1.0, 2.0);
        let mut circles = 0;
        let mut lines = 0;
        for (_, seg) in &s.edge_geom {
            match seg.curve {
                CurveKind::Circle(_) => circles += 1,
                CurveKind::Line(_) => lines += 1,
                _ => panic!("unexpected curve kind"),
            }
        }
        assert_eq!(circles, 2);
        assert_eq!(lines, 1);
    }

    #[test]
    fn cylinder_has_two_planar_caps_and_one_cylindrical_face() {
        let s = cylinder(1.0, 2.0);
        let mut planes = 0;
        let mut cylinders = 0;
        for (_, surf) in &s.face_geom {
            match surf {
                SurfaceKind::Plane(_) => planes += 1,
                SurfaceKind::Cylinder(_) => cylinders += 1,
                _ => panic!("unexpected surface kind"),
            }
        }
        assert_eq!(planes, 2);
        assert_eq!(cylinders, 1);
    }
}
