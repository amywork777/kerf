//! `frustum(top_radius, bottom_radius, height)` constructor — truncated cone.
//!
//! A frustum is a cone with the apex cut off, producing two circular caps.
//! It generalises cylinder (top_r == bot_r, rejected here — use `cylinder()`)
//! and approaches a cone as top_r → 0 (also rejected — use `cone()`).
//!
//! Topology (Euler: V=2 E=3 F=3 R=0 S=1 → 2-3+3-0 = 2 = 2*1 ✓):
//!   - 2 vertices: v_top at (top_r, 0, height), v_bot at (bot_r, 0, 0).
//!   - 3 edges:
//!     - top_circle: self-loop at v_top. Circle in z=height.
//!     - bot_circle: self-loop at v_bot. Circle in z=0.
//!     - seam: regular edge v_bot ↔ v_top.
//!   - 3 faces:
//!     - top_disk: 1 half-edge loop (top_circle_he_a). Plane normal +z.
//!     - bot_disk: 1 half-edge loop (bot_circle_he_a). Plane normal -z.
//!     - lateral: 4 half-edge loop (seam_he_a, top_circle_he_b, seam_he_b, bot_circle_he_b).
//!       Surface: Cone with apex at (0, 0, z_apex).
//!
//! Direct slotmap construction via `kerf_topo::build` module; bypasses
//! Euler-operator restrictions on self-loops.

use std::f64::consts::TAU;

use kerf_geom::{Circle, Cone as ConeSurface, Frame, Line, Plane, Point3, Vec3};
use kerf_topo::validate;

use crate::Solid;
use crate::geometry::{CurveKind, CurveSegment, Sense, SurfaceKind};

/// Construct a frustum (truncated cone) with bottom circle at z=0 (radius
/// `bottom_radius`) and top circle at z=`height` (radius `top_radius`).
/// The seam runs along the line from (bottom_radius, 0, 0) to
/// (top_radius, 0, height).
///
/// # Panics (debug)
/// - Panics if `top_radius <= 0` or `bottom_radius <= 0` or `height <= 0`.
/// - Panics if `top_radius == bottom_radius` (use `cylinder()` instead).
pub fn frustum(top_radius: f64, bottom_radius: f64, height: f64) -> Solid {
    debug_assert!(top_radius > 0.0, "top_radius must be positive (use cone() for apex)");
    debug_assert!(bottom_radius > 0.0, "bottom_radius must be positive");
    debug_assert!(height > 0.0, "height must be positive");
    debug_assert!(
        (top_radius - bottom_radius).abs() > 1e-12,
        "use cylinder() for equal radii"
    );

    let mut s = Solid::new();

    // ---- 1. Vertices ----
    let v_top = s.topo.build_insert_vertex();
    let v_bot = s.topo.build_insert_vertex();
    s.vertex_geom.insert(v_top, Point3::new(top_radius, 0.0, height));
    s.vertex_geom.insert(v_bot, Point3::new(bottom_radius, 0.0, 0.0));

    // ---- 2. Solid + Shell ----
    let solid_id = s.topo.build_insert_solid();
    s.topo.build_set_active_solid(Some(solid_id));
    let shell_id = s.topo.build_insert_shell(solid_id);

    // ---- 3. Loops + Faces ----
    // top_disk face
    let top_loop = s.topo.build_insert_loop_placeholder();
    let top_face = s.topo.build_insert_face(top_loop, shell_id);
    s.topo.build_set_loop_face(top_loop, top_face);
    s.topo.build_push_shell_face(shell_id, top_face);

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
    // top_disk loop: one self-loop half-edge, origin = v_top.
    let top_he_a = s.topo.build_insert_half_edge(v_top, top_loop);
    // bot_disk loop: one self-loop half-edge, origin = v_bot.
    let bot_he_a = s.topo.build_insert_half_edge(v_bot, bot_loop);

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
    // top_disk: self-loop (next/prev = self)
    s.topo.build_set_half_edge_next_prev(top_he_a, top_he_a);
    // bot_disk: self-loop
    s.topo.build_set_half_edge_next_prev(bot_he_a, bot_he_a);
    // lateral: [seam_a → top_b → seam_b → bot_b] cyclic
    s.topo.build_set_half_edge_next_prev(seam_he_a, top_he_b);
    s.topo.build_set_half_edge_next_prev(top_he_b, seam_he_b);
    s.topo.build_set_half_edge_next_prev(seam_he_b, bot_he_b);
    s.topo.build_set_half_edge_next_prev(bot_he_b, seam_he_a);

    // ---- 6. Wire loop representative half-edges ----
    s.topo.build_set_loop_half_edge(top_loop, Some(top_he_a));
    s.topo.build_set_loop_half_edge(bot_loop, Some(bot_he_a));
    s.topo.build_set_loop_half_edge(lat_loop, Some(seam_he_a));

    // ---- 7. Twin pairs ----
    // top circle: he_a (in top_disk) ↔ he_b (in lateral)
    s.topo.build_set_half_edge_twin(top_he_a, top_he_b);
    s.topo.build_set_half_edge_twin(top_he_b, top_he_a);
    // bot circle: he_a (in bot_disk) ↔ he_b (in lateral)
    s.topo.build_set_half_edge_twin(bot_he_a, bot_he_b);
    s.topo.build_set_half_edge_twin(bot_he_b, bot_he_a);
    // seam: he_a ↔ he_b (sticker — both in lateral)
    s.topo.build_set_half_edge_twin(seam_he_a, seam_he_b);
    s.topo.build_set_half_edge_twin(seam_he_b, seam_he_a);

    // ---- 8. Edges ----
    let edge_top_circle = s.topo.build_insert_edge([top_he_a, top_he_b]);
    s.topo.build_set_half_edge_edge(top_he_a, edge_top_circle);
    s.topo.build_set_half_edge_edge(top_he_b, edge_top_circle);

    let edge_bot_circle = s.topo.build_insert_edge([bot_he_a, bot_he_b]);
    s.topo.build_set_half_edge_edge(bot_he_a, edge_bot_circle);
    s.topo.build_set_half_edge_edge(bot_he_b, edge_bot_circle);

    let edge_seam = s.topo.build_insert_edge([seam_he_a, seam_he_b]);
    s.topo.build_set_half_edge_edge(seam_he_a, edge_seam);
    s.topo.build_set_half_edge_edge(seam_he_b, edge_seam);

    // ---- 9. Vertex outgoing half-edges ----
    s.topo.build_set_vertex_outgoing(v_top, Some(top_he_a));
    s.topo.build_set_vertex_outgoing(v_bot, Some(bot_he_a));

    // ---- 10. Edge geometry ----
    // top_circle: full circle at z=height, center (0,0,height), radius top_radius.
    // Frame::world gives x=+x, y=+y, z=+z, so point_at(0) = (top_r, 0, height) = v_top. ✓
    let top_circle_frame = Frame::world(Point3::new(0.0, 0.0, height));
    s.edge_geom.insert(
        edge_top_circle,
        CurveSegment {
            curve: CurveKind::Circle(Circle::new(top_circle_frame, top_radius)),
            range: (0.0, TAU),
            sense: Sense::Forward,
        },
    );

    // bot_circle: full circle at z=0, center origin, radius bottom_radius.
    let bot_circle_frame = Frame::world(Point3::origin());
    s.edge_geom.insert(
        edge_bot_circle,
        CurveSegment {
            curve: CurveKind::Circle(Circle::new(bot_circle_frame, bottom_radius)),
            range: (0.0, TAU),
            sense: Sense::Forward,
        },
    );

    // seam: line from (bottom_radius, 0, 0) to (top_radius, 0, height).
    let seam_origin = Point3::new(bottom_radius, 0.0, 0.0);
    let seam_vec = Vec3::new(top_radius - bottom_radius, 0.0, height);
    let seam_len = seam_vec.norm();
    let seam_dir = seam_vec / seam_len;
    let seam_line =
        Line::from_origin_dir(seam_origin, seam_dir).expect("seam line has valid direction");
    s.edge_geom.insert(
        edge_seam,
        CurveSegment::line(seam_line, 0.0, seam_len),
    );

    // ---- 11. Face geometry ----
    // top_disk: plane at z=height, normal +z (outward).
    let top_plane_frame = Frame::world(Point3::new(0.0, 0.0, height));
    s.face_geom
        .insert(top_face, SurfaceKind::Plane(Plane::new(top_plane_frame)));

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
    //
    // The lateral face lies on an infinite cone whose apex is the point where
    // the seam line (extended) meets the z-axis (r = 0).
    //
    // Seam passes through (bottom_radius, 0, 0) and (top_radius, 0, height).
    // At z = z_apex, r = 0:  bottom_radius + (z_apex / height) * (top_radius - bottom_radius) = 0
    // → z_apex = -bottom_radius * height / (top_radius - bottom_radius)
    //
    // If top_r > bot_r: cone opens upward, apex is below z=0 (z_apex < 0),
    //   frame.z = +z (axis pointing upward, away from apex toward both circles).
    // If top_r < bot_r: cone opens downward, apex is above z=height (z_apex > height),
    //   frame.z = -z (axis pointing downward, away from apex toward both circles).
    let z_apex = -bottom_radius * height / (top_radius - bottom_radius);
    let cone_axis_unit = if top_radius > bottom_radius {
        Vec3::z()
    } else {
        -Vec3::z()
    };
    // half_angle: angle between axis and generator. Use bottom circle as reference.
    // tan(alpha) = bottom_radius / |0 - z_apex| = bottom_radius / z_apex.abs()
    let half_angle = (bottom_radius / z_apex.abs()).atan();
    let cone_frame = Frame {
        origin: Point3::new(0.0, 0.0, z_apex),
        x: Vec3::x(),
        y: Vec3::y(),
        z: cone_axis_unit,
    };
    s.face_geom.insert(
        lat_face,
        SurfaceKind::Cone(ConeSurface::new(cone_frame, half_angle)),
    );

    validate(&s.topo).expect("frustum topology violates Euler invariant");

    s
}

#[cfg(test)]
mod tests {
    use super::*;
    use kerf_topo::validate;

    use crate::geometry::SurfaceKind;

    #[test]
    fn unit_frustum_has_cylinder_topology() {
        let s = frustum(1.0, 2.0, 3.0);
        assert_eq!(s.vertex_count(), 2);
        assert_eq!(s.edge_count(), 3);
        assert_eq!(s.face_count(), 3);
    }

    #[test]
    fn frustum_validates() {
        let s = frustum(1.0, 2.0, 3.0);
        validate(&s.topo).unwrap();
    }

    #[test]
    fn frustum_lateral_face_is_cone_surface() {
        let s = frustum(1.0, 2.0, 3.0);
        let mut cones = 0;
        let mut planes = 0;
        for (_, surf) in &s.face_geom {
            match surf {
                SurfaceKind::Cone(_) => cones += 1,
                SurfaceKind::Plane(_) => planes += 1,
                _ => panic!(),
            }
        }
        assert_eq!(cones, 1);
        assert_eq!(planes, 2);
    }

    #[test]
    fn frustum_apex_for_equal_radii_panics() {
        use std::panic;
        let result = panic::catch_unwind(|| frustum(2.0, 2.0, 1.0));
        assert!(result.is_err(), "frustum with equal radii should panic");
    }

    #[test]
    fn frustum_top_larger_than_bottom() {
        // top_r > bot_r: cone opens upward, apex below z=0
        let s = frustum(2.0, 1.0, 3.0);
        assert_eq!(s.vertex_count(), 2);
        assert_eq!(s.edge_count(), 3);
        assert_eq!(s.face_count(), 3);
        validate(&s.topo).unwrap();
    }

    #[test]
    fn frustum_seam_vertex_positions() {
        let s = frustum(1.0, 2.0, 3.0);
        // Check that vertex positions contain the seam endpoints.
        let pts: Vec<_> = s.vertex_geom.values().copied().collect();
        let has_top = pts.iter().any(|p| {
            (p.x - 1.0).abs() < 1e-10 && p.y.abs() < 1e-10 && (p.z - 3.0).abs() < 1e-10
        });
        let has_bot = pts.iter().any(|p| {
            (p.x - 2.0).abs() < 1e-10 && p.y.abs() < 1e-10 && p.z.abs() < 1e-10
        });
        assert!(has_top, "missing top seam vertex at (1, 0, 3)");
        assert!(has_bot, "missing bot seam vertex at (2, 0, 0)");
    }
}
