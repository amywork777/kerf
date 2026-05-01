//! `revolve_polyline(profile)` constructor — axisymmetric solid of revolution.
//!
//! Revolves an open polyline in the xz-plane around the z-axis to produce a
//! closed solid. Both endpoints of the profile must lie on the z-axis (x = 0);
//! all interior points must have x > 0.
//!
//! # Topology
//!
//! For N profile points (2 axis endpoints + (N-2) interior seam vertices):
//!
//! | Entity          | Count  | Description                                    |
//! |-----------------|--------|------------------------------------------------|
//! | Vertices        | N      | profile[0] (top apex), profile[1..N-2] (seam), profile[N-1] (bot apex) |
//! | Edges           | 2N - 3 | (N-1) seam edges + (N-2) circle edges          |
//! | Faces           | N - 1  | one per profile edge segment                   |
//!
//! Euler check: V - E + F = N - (2N-3) + (N-1) = 2 ✓
//!
//! # Face topology
//!
//! - **face[0]** (top cone): 3 half-edge loop — `seam_a[0]`, `circle_b[1]`, `seam_b[0]`
//! - **face[i]** (middle, 1 ≤ i ≤ N-3): 4 half-edge loop — `seam_a[i]`, `circle_b[i+1]`,
//!   `seam_b[i]`, `circle_a[i]`
//! - **face[N-2]** (bot cone): 3 half-edge loop — `circle_a[N-2]`, `seam_a[N-2]`, `seam_b[N-2]`
//!
//! # Circle half-edge convention
//!
//! Each circle edge at interior vertex v[i] (1 ≤ i ≤ N-2) has two half-edges:
//! - `circle_he_a[i]`: in the face ABOVE (face[i-1])
//! - `circle_he_b[i]`: in the face BELOW (face[i])

use std::f64::consts::TAU;

use kerf_geom::{Circle, Cone as ConeSurface, Cylinder as CylSurface, Frame, Line, Point3, Vec3};
use kerf_topo::validate;

use crate::Solid;
use crate::geometry::{CurveKind, CurveSegment, Sense, SurfaceKind};

/// Revolve an open polyline in the xz-plane (y = 0) around the z-axis to
/// produce a closed axisymmetric solid.
///
/// `profile[0]` and `profile[N-1]` must be on the z-axis (x = 0, y = 0).
/// All interior points must have x > 0 and y = 0. Profile must have N ≥ 3
/// points.
///
/// The profile may be in either z-increasing or z-decreasing order; the
/// constructor handles both.
///
/// # Panics (debug)
/// Panics in debug mode if the profile constraints above are violated.
pub fn revolve_polyline(profile: &[Point3]) -> Solid {
    let n = profile.len();
    debug_assert!(n >= 3, "profile must have at least 3 points");
    debug_assert!(
        profile[0].x.abs() < 1e-12 && profile[0].y.abs() < 1e-12,
        "profile[0] must be on the z-axis"
    );
    debug_assert!(
        profile[n - 1].x.abs() < 1e-12 && profile[n - 1].y.abs() < 1e-12,
        "profile[N-1] must be on the z-axis"
    );
    for i in 1..n - 1 {
        debug_assert!(
            profile[i].x > 1e-12,
            "interior profile point {i} must have x > 0"
        );
        debug_assert!(
            profile[i].y.abs() < 1e-12,
            "interior profile point {i} must be in the xz-plane (y = 0)"
        );
    }

    let mut s = Solid::new();

    // ---- 1. Vertices ----
    // v_ids[0] = top apex, v_ids[1..N-2] = seam verts, v_ids[N-1] = bot apex.
    let v_ids: Vec<_> = (0..n).map(|_| s.topo.build_insert_vertex()).collect();
    for (i, p) in profile.iter().enumerate() {
        s.vertex_geom.insert(v_ids[i], *p);
    }

    // ---- 2. Solid + Shell ----
    let solid_id = s.topo.build_insert_solid();
    s.topo.build_set_active_solid(Some(solid_id));
    let shell_id = s.topo.build_insert_shell(solid_id);

    // ---- 3. Faces + Loops (N-1 faces) ----
    let mut face_ids = Vec::with_capacity(n - 1);
    let mut loop_ids = Vec::with_capacity(n - 1);
    for _ in 0..n - 1 {
        let lp = s.topo.build_insert_loop_placeholder();
        let face = s.topo.build_insert_face(lp, shell_id);
        s.topo.build_set_loop_face(lp, face);
        s.topo.build_push_shell_face(shell_id, face);
        loop_ids.push(lp);
        face_ids.push(face);
    }

    // ---- 4. Half-edges ----
    // Seam edges: seam[i] connects v[i] to v[i+1], for i in 0..N-1.
    // Each seam edge has two half-edges (they're stickers — both in the same face).
    //   seam_he_a[i]: origin v[i], in face[i]
    //   seam_he_b[i]: origin v[i+1], in face[i] (the return half-edge)
    //
    // Circle edges: at v[i] for i in 1..N-2 (interior only).
    //   circle_he_a[i]: in face[i-1] (face above)
    //   circle_he_b[i]: in face[i] (face below)
    // We index circle arrays by offset 1, so circle_he_a[j] means the half-edge
    // for the circle at v[j+1] that belongs to face[j].

    // Allocate seam half-edges (2 per seam edge = 2*(N-1) total).
    let mut seam_he_a: Vec<_> = Vec::with_capacity(n - 1); // origin v[i], in face[i]
    let mut seam_he_b: Vec<_> = Vec::with_capacity(n - 1); // origin v[i+1], in face[i]
    for i in 0..n - 1 {
        let ha = s.topo.build_insert_half_edge(v_ids[i], loop_ids[i]);
        let hb = s.topo.build_insert_half_edge(v_ids[i + 1], loop_ids[i]);
        seam_he_a.push(ha);
        seam_he_b.push(hb);
    }

    // Allocate circle half-edges (2 per interior vertex = 2*(N-2) total).
    // circle_he_a[j]: belongs to face[j], at circle vertex v[j+1] — this is
    //                 the "top" half-edge of face[j+1] (or the sole circle in the
    //                 bottom cone face).
    // circle_he_b[j]: belongs to face[j+1], at circle vertex v[j+1].
    //
    // We track them in arrays indexed 0..N-2, where index j refers to the circle
    // at profile vertex j+1.
    let mut circle_he_a: Vec<_> = Vec::with_capacity(n - 2); // in face[j] (above)
    let mut circle_he_b: Vec<_> = Vec::with_capacity(n - 2); // in face[j+1] (below)
    for j in 0..n - 2 {
        // Circle at v[j+1].
        // circle_he_a[j] is in the face ABOVE the circle, which is face[j].
        // circle_he_b[j] is in the face BELOW the circle, which is face[j+1].
        let ha = s.topo.build_insert_half_edge(v_ids[j + 1], loop_ids[j]);
        let hb = s.topo.build_insert_half_edge(v_ids[j + 1], loop_ids[j + 1]);
        circle_he_a.push(ha);
        circle_he_b.push(hb);
    }

    // ---- 5. Wire next/prev within each face's loop ----
    //
    // face[0] (top cone): 3 half-edge loop
    //   seam_he_a[0] → circle_he_a[0] → seam_he_b[0] → (back to seam_he_a[0])
    //
    //   circle_he_a[0] is at v[1] in face[0] (the "above" half-edge for the circle at v[1]).
    //   Manifold check: seam_a[0].next=circle_a[0] (origin v[1]), seam_a[0].twin=seam_b[0] (origin v[1]). ✓
    //   circle_a[0].next=seam_b[0] (origin v[1]), circle_a[0].twin=circle_b[0] (origin v[1]). ✓
    //   seam_b[0].next=seam_a[0] (origin v[0]), seam_b[0].twin=seam_a[0] (origin v[0]). ✓
    s.topo
        .build_set_half_edge_next_prev(seam_he_a[0], circle_he_a[0]);
    s.topo
        .build_set_half_edge_next_prev(circle_he_a[0], seam_he_b[0]);
    s.topo
        .build_set_half_edge_next_prev(seam_he_b[0], seam_he_a[0]);
    s.topo.build_set_loop_half_edge(loop_ids[0], Some(seam_he_a[0]));

    // face[N-2] (bot cone): 3 half-edge loop
    //   circle_he_a[N-3] → seam_he_a[N-2] → seam_he_b[N-2] → (back to circle_he_a[N-3])
    //   Note: circle_he_a for the bottom cone is circle_he_a[n-3], which is the
    //   "above" half-edge for the circle at v[N-2]. But wait — for the bot cone face
    //   (face[N-2]), the circle at v[N-2] is accessed as circle_he_a[N-3].
    //
    //   Actually: circle_he_a[j] is in face[j]. For the bot cone face = face[N-2],
    //   we need the circle at v[N-2] in face[N-2]. That's circle_he_b[N-3].
    //   Wait — let me re-derive.
    //
    //   circle at v[j+1]: circle_he_a[j] in face[j], circle_he_b[j] in face[j+1].
    //   For the circle at v[N-2]: j = N-3. So circle_he_a[N-3] in face[N-3], circle_he_b[N-3] in face[N-2].
    //   The bot cone is face[N-2], so it uses circle_he_b[N-3].
    let bot_cone_circle_he = circle_he_b[n - 3]; // at v[N-2], in face[N-2]
    s.topo
        .build_set_half_edge_next_prev(bot_cone_circle_he, seam_he_a[n - 2]);
    s.topo
        .build_set_half_edge_next_prev(seam_he_a[n - 2], seam_he_b[n - 2]);
    s.topo
        .build_set_half_edge_next_prev(seam_he_b[n - 2], bot_cone_circle_he);
    s.topo
        .build_set_loop_half_edge(loop_ids[n - 2], Some(bot_cone_circle_he));

    // Middle faces (face[i], 1 ≤ i ≤ N-3): 4 half-edge loop
    //   seam_he_a[i] → circle_he_b[i] → seam_he_b[i] → circle_he_a[i-1] → (back to seam_he_a[i])
    //
    //   Wait — for middle face[i]:
    //   - top circle is at v[i]: circle at v[i] is j=i-1, so circle_he_b[i-1] in face[i]. ✓
    //   - bot circle is at v[i+1]: circle at v[i+1] is j=i, so circle_he_b[i] in face[i+1]...
    //     No wait. Let me re-derive:
    //     circle_he_b[j] is in face[j+1].
    //     For face[i], the bottom circle is at v[i+1]. The circle at v[i+1] has j=i.
    //     circle_he_b[i] is in face[i+1]. That's WRONG for face[i].
    //     Actually circle_he_a[j] is in face[j]. For circle at v[i+1], j=i, so circle_he_a[i] is in face[i]. ✓
    //
    //   Re-reading: circle_he_a[j] in face[j] = ABOVE circle. circle_he_b[j] in face[j+1] = BELOW circle.
    //   "Above" and "below" here mean in terms of profile indexing, not z-height.
    //
    //   For middle face[i] (the face between profile[i] and profile[i+1]):
    //   - The "top" boundary of face[i] is the circle at v[i]: this is circle_he_b[i-1] (in face[i]).
    //   - The "bottom" boundary of face[i] is the circle at v[i+1]: this is circle_he_a[i] (in face[i]).
    //
    //   Loop order: seam_a[i] → circle_a[i] (at v[i+1]) → seam_b[i] → circle_b[i-1] (at v[i]) → seam_a[i]
    //
    //   Let me verify manifold: seam_a[i].origin=v[i]. next=circle_a[i] (origin=v[i+1]).
    //   seam_a[i].twin = seam_b[i] (origin=v[i+1]). ✓
    //   circle_a[i].origin=v[i+1] (self-loop). next=seam_b[i] (origin=v[i+1]). twin=circle_b[i] (origin=v[i+1]). ✓
    //   seam_b[i].origin=v[i+1]. next=circle_b[i-1] (origin=v[i]). twin=seam_a[i] (origin=v[i]). ✓
    //   circle_b[i-1].origin=v[i] (self-loop). next=seam_a[i] (origin=v[i]). twin=circle_a[i-1] (origin=v[i]). ✓
    for i in 1..n - 2 {
        // face[i]: seam_a[i] → circle_a[i] → seam_b[i] → circle_b[i-1] → (back to seam_a[i])
        // circle_a[i] is at v[i+1] (bottom of this face segment)
        // circle_b[i-1] is at v[i] (top of this face segment)
        s.topo
            .build_set_half_edge_next_prev(seam_he_a[i], circle_he_a[i]);
        s.topo
            .build_set_half_edge_next_prev(circle_he_a[i], seam_he_b[i]);
        s.topo
            .build_set_half_edge_next_prev(seam_he_b[i], circle_he_b[i - 1]);
        s.topo
            .build_set_half_edge_next_prev(circle_he_b[i - 1], seam_he_a[i]);
        s.topo
            .build_set_loop_half_edge(loop_ids[i], Some(seam_he_a[i]));
    }

    // ---- 6. Twin pairs ----
    // Seam edges: seam_he_a[i] ↔ seam_he_b[i] (sticker — both in same face)
    for i in 0..n - 1 {
        s.topo
            .build_set_half_edge_twin(seam_he_a[i], seam_he_b[i]);
        s.topo
            .build_set_half_edge_twin(seam_he_b[i], seam_he_a[i]);
    }
    // Circle edges: circle_he_a[j] ↔ circle_he_b[j]
    for j in 0..n - 2 {
        s.topo
            .build_set_half_edge_twin(circle_he_a[j], circle_he_b[j]);
        s.topo
            .build_set_half_edge_twin(circle_he_b[j], circle_he_a[j]);
    }

    // ---- 7. Edges ----
    // (N-1) seam edges
    let mut seam_edge_ids = Vec::with_capacity(n - 1);
    for i in 0..n - 1 {
        let e = s.topo.build_insert_edge([seam_he_a[i], seam_he_b[i]]);
        s.topo.build_set_half_edge_edge(seam_he_a[i], e);
        s.topo.build_set_half_edge_edge(seam_he_b[i], e);
        seam_edge_ids.push(e);
    }
    // (N-2) circle edges
    let mut circle_edge_ids = Vec::with_capacity(n - 2);
    for j in 0..n - 2 {
        let e = s.topo.build_insert_edge([circle_he_a[j], circle_he_b[j]]);
        s.topo.build_set_half_edge_edge(circle_he_a[j], e);
        s.topo.build_set_half_edge_edge(circle_he_b[j], e);
        circle_edge_ids.push(e);
    }

    // ---- 8. Vertex outgoing half-edges ----
    // Apex vertices: use seam_he_a[0] for v[0], seam_he_b[N-2] for v[N-1].
    s.topo
        .build_set_vertex_outgoing(v_ids[0], Some(seam_he_a[0]));
    s.topo
        .build_set_vertex_outgoing(v_ids[n - 1], Some(seam_he_b[n - 2]));
    // Interior seam vertices: use circle_he_a[j] for v[j+1] (j in 0..N-2).
    for j in 0..n - 2 {
        s.topo
            .build_set_vertex_outgoing(v_ids[j + 1], Some(circle_he_a[j]));
    }

    // ---- 9. Edge geometry ----
    // Seam edges: line from profile[i] to profile[i+1].
    for i in 0..n - 1 {
        let p0 = profile[i];
        let p1 = profile[i + 1];
        let seam_vec = Vec3::new(p1.x - p0.x, 0.0, p1.z - p0.z);
        let seam_len = seam_vec.norm();
        let seam_dir = seam_vec / seam_len;
        let seam_line = Line::from_origin_dir(Point3::new(p0.x, 0.0, p0.z), seam_dir)
            .expect("seam line has valid direction");
        s.edge_geom.insert(
            seam_edge_ids[i],
            CurveSegment::line(seam_line, 0.0, seam_len),
        );
    }
    // Circle edges: full circles at each interior profile vertex.
    // circle j is at profile[j+1], radius = profile[j+1].x, z = profile[j+1].z.
    for j in 0..n - 2 {
        let p = profile[j + 1];
        let circle_frame = Frame::world(Point3::new(0.0, 0.0, p.z));
        s.edge_geom.insert(
            circle_edge_ids[j],
            CurveSegment {
                curve: CurveKind::Circle(Circle::new(circle_frame, p.x)),
                range: (0.0, TAU),
                sense: Sense::Forward,
            },
        );
    }

    // ---- 10. Face geometry ----
    // face[0]: top cone. Apex at profile[0] (on z-axis), base circle at profile[1].
    // The cone's frame.z should point from apex toward the base (downward if profile
    // is z-decreasing). Apex at profile[0], frame.z = direction from profile[0] to
    // profile[1] projected onto z-axis (since apex is on z-axis and base is on the
    // circle, we use the axis direction).
    let apex_top = profile[0];
    let base_top = profile[1];
    let cone_axis_top = Vec3::new(0.0, 0.0, base_top.z - apex_top.z);
    let cone_axis_top_unit = cone_axis_top
        .try_normalize(0.0)
        .unwrap_or(-Vec3::z());
    let half_angle_top = (base_top.x / (base_top.z - apex_top.z).abs()).atan();
    let cone_frame_top = Frame {
        origin: apex_top,
        x: Vec3::x(),
        y: Vec3::y(),
        z: cone_axis_top_unit,
    };
    s.face_geom
        .insert(face_ids[0], SurfaceKind::Cone(ConeSurface::new(cone_frame_top, half_angle_top)));

    // face[N-2]: bot cone. Apex at profile[N-1], base circle at profile[N-2].
    let apex_bot = profile[n - 1];
    let base_bot = profile[n - 2];
    let cone_axis_bot = Vec3::new(0.0, 0.0, base_bot.z - apex_bot.z);
    let cone_axis_bot_unit = cone_axis_bot
        .try_normalize(0.0)
        .unwrap_or(Vec3::z());
    let half_angle_bot = (base_bot.x / (base_bot.z - apex_bot.z).abs()).atan();
    let cone_frame_bot = Frame {
        origin: apex_bot,
        x: Vec3::x(),
        y: Vec3::y(),
        z: cone_axis_bot_unit,
    };
    s.face_geom
        .insert(face_ids[n - 2], SurfaceKind::Cone(ConeSurface::new(cone_frame_bot, half_angle_bot)));

    // Middle faces (1 ≤ i ≤ N-3).
    // face[i] spans profile[i] to profile[i+1], both with x > 0.
    // If profile[i].x == profile[i+1].x → Cylinder.
    // Otherwise → Cone (frustum section).
    for i in 1..n - 2 {
        let p_top = profile[i];
        let p_bot = profile[i + 1];
        let surf = if (p_top.x - p_bot.x).abs() < 1e-12 {
            // Vertical edge → Cylinder.
            let cyl_frame = Frame::world(Point3::new(0.0, 0.0, p_bot.z.min(p_top.z)));
            SurfaceKind::Cylinder(CylSurface::new(cyl_frame, p_top.x))
        } else {
            // Tilted edge → Cone (frustum section).
            // Find apex: seam line from p_top to p_bot extended to x = 0.
            // Parametric: (x, z) = (p_top.x, p_top.z) + t*(p_bot.x - p_top.x, p_bot.z - p_top.z)
            // x = 0 when t = -p_top.x / (p_bot.x - p_top.x)
            let t_apex = -p_top.x / (p_bot.x - p_top.x);
            let z_apex = p_top.z + t_apex * (p_bot.z - p_top.z);
            // Cone axis direction: from apex toward the circles (away from apex).
            // If apex is above both circles (z_apex > p_top.z > p_bot.z after normalization),
            // axis points downward (-z). Otherwise +z.
            let cone_axis_dir = if z_apex > p_top.z.max(p_bot.z) {
                -Vec3::z() // apex above, axis points down
            } else {
                Vec3::z() // apex below, axis points up
            };
            let half_angle = (p_top.x / (p_top.z - z_apex).abs()).atan();
            let cone_frame = Frame {
                origin: Point3::new(0.0, 0.0, z_apex),
                x: Vec3::x(),
                y: Vec3::y(),
                z: cone_axis_dir,
            };
            SurfaceKind::Cone(ConeSurface::new(cone_frame, half_angle))
        };
        s.face_geom.insert(face_ids[i], surf);
    }

    validate(&s.topo).expect("revolve_polyline topology violates Euler invariant");

    s
}

#[cfg(test)]
mod tests {
    use super::*;
    use kerf_geom::Point3;
    use kerf_topo::validate;

    use crate::geometry::SurfaceKind;

    #[test]
    fn revolve_3_point_profile_makes_bipyramid() {
        // Profile: top apex (0,0,1), mid (1,0,0.5), bot apex (0,0,0).
        // Topology: N=3, V=3, E=2*3-3=3, F=3-1=2.
        let profile = vec![
            Point3::new(0.0, 0.0, 1.0),
            Point3::new(1.0, 0.0, 0.5),
            Point3::new(0.0, 0.0, 0.0),
        ];
        let s = revolve_polyline(&profile);
        assert_eq!(s.vertex_count(), 3);
        assert_eq!(s.edge_count(), 3);
        assert_eq!(s.face_count(), 2);
        validate(&s.topo).unwrap();
    }

    #[test]
    fn revolve_4_point_profile_correct_counts() {
        // Profile: top apex, mid 1, mid 2, bot apex. N=4, V=4, E=5, F=3.
        let profile = vec![
            Point3::new(0.0, 0.0, 2.0),
            Point3::new(1.0, 0.0, 1.5),
            Point3::new(1.0, 0.0, 0.5), // same x → cylinder middle
            Point3::new(0.0, 0.0, 0.0),
        ];
        let s = revolve_polyline(&profile);
        assert_eq!(s.vertex_count(), 4);
        assert_eq!(s.edge_count(), 5);
        assert_eq!(s.face_count(), 3);
        validate(&s.topo).unwrap();
    }

    #[test]
    fn revolve_4_point_makes_cone_cylinder_cone() {
        let profile = vec![
            Point3::new(0.0, 0.0, 2.0),
            Point3::new(1.0, 0.0, 1.5),
            Point3::new(1.0, 0.0, 0.5),
            Point3::new(0.0, 0.0, 0.0),
        ];
        let s = revolve_polyline(&profile);
        let mut cones = 0;
        let mut cylinders = 0;
        for (_, surf) in &s.face_geom {
            match surf {
                SurfaceKind::Cone(_) => cones += 1,
                SurfaceKind::Cylinder(_) => cylinders += 1,
                _ => panic!("unexpected surface kind"),
            }
        }
        assert_eq!(cones, 2);
        assert_eq!(cylinders, 1);
    }

    #[test]
    fn revolve_5_point_profile_correct_counts() {
        // N=5: V=5, E=7, F=4.
        let profile = vec![
            Point3::new(0.0, 0.0, 3.0),
            Point3::new(1.0, 0.0, 2.5),
            Point3::new(1.5, 0.0, 1.5),
            Point3::new(0.8, 0.0, 0.5),
            Point3::new(0.0, 0.0, 0.0),
        ];
        let s = revolve_polyline(&profile);
        assert_eq!(s.vertex_count(), 5);
        assert_eq!(s.edge_count(), 7);
        assert_eq!(s.face_count(), 4);
        validate(&s.topo).unwrap();
    }

    #[test]
    fn revolve_all_cone_frustum_profile() {
        // N=4 with varying radii — all three faces should be cones.
        let profile = vec![
            Point3::new(0.0, 0.0, 3.0),
            Point3::new(1.0, 0.0, 2.0),
            Point3::new(2.0, 0.0, 1.0),
            Point3::new(0.0, 0.0, 0.0),
        ];
        let s = revolve_polyline(&profile);
        assert_eq!(s.vertex_count(), 4);
        assert_eq!(s.edge_count(), 5);
        assert_eq!(s.face_count(), 3);
        for (_, surf) in &s.face_geom {
            match surf {
                SurfaceKind::Cone(_) => {}
                _ => panic!("expected all cone surfaces"),
            }
        }
        validate(&s.topo).unwrap();
    }
}
