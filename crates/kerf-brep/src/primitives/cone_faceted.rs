//! `cone_faceted(radius, height, n)` — n-gon pyramid approximation of a cone.
//!
//! Unlike [`cone`], the lateral surface is a fan of `n` planar triangles instead
//! of one analytic `Cone` surface. The geometry is fully planar, so it
//! tessellates to a clean STL without any seam-edge special cases.
//!
//! # Topology (Euler: V = n+1, E = 2n, F = n+1, S = 1)
//! (n+1) - 2n + (n+1) - 0 = 2 ✓
//!
//! - 1 base n-gon face (outward normal = -z)
//! - n lateral triangle faces (outward normals point away from z-axis)
//! - Base half-edge loop is CW from +z (CCW from outward -z).
//! - Lateral loops wind: b_k → b_{k+1} → apex → b_k (CCW from outward).

use std::f64::consts::PI;

use kerf_geom::{Frame, Line, Plane, Point3, Vec3};
use kerf_topo::validate;

use crate::geometry::{CurveSegment, SurfaceKind};
use crate::Solid;

/// Build an n-gon pyramid (faceted cone) with base radius `r`, height `h`,
/// and `n` lateral triangular faces.
///
/// The base is a regular n-gon centered at the origin in the z = 0 plane,
/// with a phase offset of π/n so that for n = 4 the base is an axis-aligned
/// square. The apex is at (0, 0, h). All faces are planar.
///
/// The solid has 2n + 1 faces (1 base + n lateral), 2n edges, and n + 1
/// vertices.
///
/// # Panics (debug)
/// Panics if `radius <= 0`, `height <= 0`, or `n < 3`.
pub fn cone_faceted(radius: f64, height: f64, n: usize) -> Solid {
    debug_assert!(radius > 0.0, "radius must be positive");
    debug_assert!(height > 0.0, "height must be positive");
    debug_assert!(n >= 3, "n must be at least 3");

    let mut s = Solid::new();

    // ---- 1. Vertices ----
    // Base ring: n vertices evenly spaced, phased by π/n.
    let phase = PI / n as f64;
    let base_verts: Vec<_> = (0..n)
        .map(|i| {
            let v = s.topo.build_insert_vertex();
            let theta = phase + 2.0 * PI * i as f64 / n as f64;
            s.vertex_geom
                .insert(v, Point3::new(radius * theta.cos(), radius * theta.sin(), 0.0));
            v
        })
        .collect();

    // Apex vertex.
    let v_apex = s.topo.build_insert_vertex();
    s.vertex_geom.insert(v_apex, Point3::new(0.0, 0.0, height));

    // ---- 2. Solid + Shell ----
    let solid_id = s.topo.build_insert_solid();
    s.topo.build_set_active_solid(Some(solid_id));
    let shell_id = s.topo.build_insert_shell(solid_id);

    // ---- 3. Faces + Loops ----
    // Base face (1 n-gon).
    let base_loop = s.topo.build_insert_loop_placeholder();
    let base_face = s.topo.build_insert_face(base_loop, shell_id);
    s.topo.build_set_loop_face(base_loop, base_face);
    s.topo.build_push_shell_face(shell_id, base_face);

    // Lateral triangular faces f_0 … f_{n-1}.
    let mut lat_loops = Vec::with_capacity(n);
    let mut lat_faces = Vec::with_capacity(n);
    for _ in 0..n {
        let lp = s.topo.build_insert_loop_placeholder();
        let face = s.topo.build_insert_face(lp, shell_id);
        s.topo.build_set_loop_face(lp, face);
        s.topo.build_push_shell_face(shell_id, face);
        lat_loops.push(lp);
        lat_faces.push(face);
    }

    // ---- 4. Half-edges ----
    // Base loop half-edges: he_base[k] origin = b_k.
    // Loop order: b_0 → b_{n-1} → b_{n-2} → … → b_1 → b_0
    // (CW from +z = CCW from outward normal -z).
    let he_base: Vec<_> = (0..n)
        .map(|k| s.topo.build_insert_half_edge(base_verts[k], base_loop))
        .collect();

    // Lateral face f_k spans {b_k, b_{k+1 mod n}, apex}.
    // Loop wind: b_k → b_{k+1} → apex → b_k (CCW from outward normal).
    //
    //   he_lat_bot[k]  : origin b_k          (base edge of the triangle)
    //   he_lat_right[k]: origin b_{k+1 mod n} (right lateral edge)
    //   he_lat_left[k] : origin apex          (left  lateral edge)
    let he_lat_bot: Vec<_> = (0..n)
        .map(|k| s.topo.build_insert_half_edge(base_verts[k], lat_loops[k]))
        .collect();
    let he_lat_right: Vec<_> = (0..n)
        .map(|k| s.topo.build_insert_half_edge(base_verts[(k + 1) % n], lat_loops[k]))
        .collect();
    let he_lat_left: Vec<_> = (0..n)
        .map(|k| s.topo.build_insert_half_edge(v_apex, lat_loops[k]))
        .collect();

    // ---- 5. Wire next/prev ----
    // Base loop: he_base[k].next = he_base[(k-1+n) % n]
    // i.e. b_0 → b_{n-1} → b_{n-2} → … → b_1 → b_0.
    for k in 0..n {
        let next_k = if k == 0 { n - 1 } else { k - 1 };
        s.topo.build_set_half_edge_next_prev(he_base[k], he_base[next_k]);
    }
    s.topo.build_set_loop_half_edge(base_loop, Some(he_base[0]));

    // Lateral face f_k: bot → right → left → bot.
    for k in 0..n {
        s.topo
            .build_set_half_edge_next_prev(he_lat_bot[k], he_lat_right[k]);
        s.topo
            .build_set_half_edge_next_prev(he_lat_right[k], he_lat_left[k]);
        s.topo
            .build_set_half_edge_next_prev(he_lat_left[k], he_lat_bot[k]);
        s.topo
            .build_set_loop_half_edge(lat_loops[k], Some(he_lat_bot[k]));
    }

    // ---- 6. Twin pairs ----
    // Base edge {b_k, b_{k+1}}:
    //   in base loop:     he_base[(k+1) % n]  (origin b_{k+1}, goes toward b_k)
    //   in lateral f_k:   he_lat_bot[k]        (origin b_k, goes toward b_{k+1})
    for k in 0..n {
        let k1 = (k + 1) % n;
        s.topo
            .build_set_half_edge_twin(he_base[k1], he_lat_bot[k]);
        s.topo
            .build_set_half_edge_twin(he_lat_bot[k], he_base[k1]);
    }

    // Lateral edge l_k = {b_k, apex}:
    //   in face f_k:              he_lat_left[k]              (origin apex)
    //   in face f_{k-1 mod n}:    he_lat_right[(k+n-1) % n]  (origin b_k)
    for k in 0..n {
        let k_prev = (k + n - 1) % n;
        s.topo
            .build_set_half_edge_twin(he_lat_right[k_prev], he_lat_left[k]);
        s.topo
            .build_set_half_edge_twin(he_lat_left[k], he_lat_right[k_prev]);
    }

    // ---- 7. Edges ----
    // n base edges: edge between b_k and b_{k+1}.
    for k in 0..n {
        let k1 = (k + 1) % n;
        let e = s.topo.build_insert_edge([he_base[k1], he_lat_bot[k]]);
        s.topo.build_set_half_edge_edge(he_base[k1], e);
        s.topo.build_set_half_edge_edge(he_lat_bot[k], e);
    }
    // n lateral edges: edge l_k between b_k and apex.
    for k in 0..n {
        let k_prev = (k + n - 1) % n;
        let e = s
            .topo
            .build_insert_edge([he_lat_right[k_prev], he_lat_left[k]]);
        s.topo.build_set_half_edge_edge(he_lat_right[k_prev], e);
        s.topo.build_set_half_edge_edge(he_lat_left[k], e);
    }

    // ---- 8. Vertex outgoing half-edges ----
    for k in 0..n {
        s.topo
            .build_set_vertex_outgoing(base_verts[k], Some(he_base[k]));
    }
    s.topo
        .build_set_vertex_outgoing(v_apex, Some(he_lat_left[0]));

    validate(&s.topo).expect("cone_faceted topology violates Euler invariant");

    // ---- 9. Edge geometry (line segments) ----
    let edge_ids: Vec<_> = s.topo.edge_ids().collect();
    for eid in &edge_ids {
        let edge = s.topo.edge(*eid).unwrap();
        let [he_a, _] = edge.half_edges();
        let v0 = s.topo.half_edge(he_a).unwrap().origin();
        let twin = s.topo.half_edge(he_a).unwrap().twin();
        let v1 = s.topo.half_edge(twin).unwrap().origin();
        let p0 = *s.vertex_geom.get(v0).unwrap();
        let p1 = *s.vertex_geom.get(v1).unwrap();
        let line = Line::through(p0, p1).expect("edge endpoints must be distinct");
        let seg = CurveSegment::line(line, 0.0, (p1 - p0).norm());
        s.edge_geom.insert(*eid, seg);
    }

    // ---- 10. Face geometry (planes) ----
    // Base face: outward normal = -z.
    let base_p = *s.vertex_geom.get(base_verts[0]).unwrap();
    let base_frame = Frame {
        origin: base_p,
        x: Vec3::x(),
        y: Vec3::y(),
        z: -Vec3::z(),
    };
    s.face_geom
        .insert(base_face, SurfaceKind::Plane(Plane::new(base_frame)));

    // Lateral face f_k: outward normal = (b_{k+1} - b_k) × (apex - b_k), normalized.
    let p_apex = *s.vertex_geom.get(v_apex).unwrap();
    for k in 0..n {
        let pk = *s.vertex_geom.get(base_verts[k]).unwrap();
        let pk1 = *s.vertex_geom.get(base_verts[(k + 1) % n]).unwrap();
        let edge_vec = pk1 - pk;
        let to_apex = p_apex - pk;
        let normal = edge_vec.cross(&to_apex).normalize();
        let x = edge_vec.normalize();
        let y = normal.cross(&x);
        let frame = Frame { origin: pk, x, y, z: normal };
        s.face_geom
            .insert(lat_faces[k], SurfaceKind::Plane(Plane::new(frame)));
    }

    s
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::{PI, TAU};

    use crate::geometry::{CurveKind, SurfaceKind};
    use crate::measure::solid_volume;
    use crate::serde_io::{read_json, write_json};
    use kerf_topo::validate;

    /// Analytic volume of a regular n-gon pyramid with circumradius `r` and height `h`:
    ///   V = (n · r² · h / 6) · sin(2π / n)
    fn pyramid_volume(r: f64, h: f64, n: usize) -> f64 {
        (n as f64 * r * r * h / 6.0) * (TAU / n as f64).sin()
    }

    #[test]
    fn triangle_pyramid_topology() {
        let s = cone_faceted(1.0, 1.0, 3);
        // V = 4, E = 6, F = 4
        assert_eq!(s.vertex_count(), 4);
        assert_eq!(s.edge_count(), 6);
        assert_eq!(s.face_count(), 4);
        assert_eq!(s.shell_count(), 1);
        validate(&s.topo).unwrap();
    }

    #[test]
    fn square_pyramid_topology() {
        let s = cone_faceted(1.0, 2.0, 4);
        // V = 5, E = 8, F = 5
        assert_eq!(s.vertex_count(), 5);
        assert_eq!(s.edge_count(), 8);
        assert_eq!(s.face_count(), 5);
        assert_eq!(s.shell_count(), 1);
        validate(&s.topo).unwrap();
    }

    #[test]
    fn hex_pyramid_topology() {
        let s = cone_faceted(1.0, 1.5, 6);
        // V = 7, E = 12, F = 7
        assert_eq!(s.vertex_count(), 7);
        assert_eq!(s.edge_count(), 12);
        assert_eq!(s.face_count(), 7);
        validate(&s.topo).unwrap();
    }

    #[test]
    fn high_poly_pyramid_topology() {
        let s = cone_faceted(2.0, 3.0, 24);
        assert_eq!(s.vertex_count(), 25);
        assert_eq!(s.edge_count(), 48);
        assert_eq!(s.face_count(), 25);
        validate(&s.topo).unwrap();
    }

    #[test]
    fn all_faces_plane_all_edges_line() {
        let s = cone_faceted(1.0, 2.0, 6);
        for (_, surf) in &s.face_geom {
            assert!(
                matches!(surf, SurfaceKind::Plane(_)),
                "expected Plane, got {surf:?}"
            );
        }
        for (_, seg) in &s.edge_geom {
            assert!(
                matches!(seg.curve, CurveKind::Line(_)),
                "expected Line, got {:?}",
                seg.curve
            );
        }
    }

    #[test]
    fn triangle_pyramid_volume_matches_analytic() {
        let r = 1.0_f64;
        let h = 1.0_f64;
        let n = 3;
        let s = cone_faceted(r, h, n);
        let got = solid_volume(&s);
        let expected = pyramid_volume(r, h, n);
        assert!(
            (got - expected).abs() < 1e-12,
            "n=3: got {got}, expected {expected}"
        );
    }

    #[test]
    fn square_pyramid_volume_matches_analytic() {
        let r = 2.0_f64;
        let h = 3.0_f64;
        let n = 4;
        let s = cone_faceted(r, h, n);
        let got = solid_volume(&s);
        let expected = pyramid_volume(r, h, n);
        assert!(
            (got - expected).abs() < 1e-9,
            "n=4 r=2 h=3: got {got}, expected {expected}"
        );
    }

    #[test]
    fn high_poly_volume_converges_to_cone() {
        // For large n: V_pyramid → π r² h / 3 (the analytic cone volume).
        let r = 1.0_f64;
        let h = 2.0_f64;
        let n = 256;
        let s = cone_faceted(r, h, n);
        let got = solid_volume(&s);
        let analytic_cone = PI * r * r * h / 3.0;
        // Inscribed n-gon underestimates π·r² by O(π³/(3n²)); at n=256 the gap is ~2e-4.
        assert!(
            (got - analytic_cone).abs() < 1e-3,
            "n=256: got {got}, cone analytic={analytic_cone}"
        );
    }

    #[test]
    fn json_round_trip_triangle_pyramid() {
        let s = cone_faceted(1.0, 2.0, 3);
        let mut buf = Vec::new();
        write_json(&s, &mut buf).unwrap();
        let s2 = read_json(&mut buf.as_slice()).unwrap();
        assert_eq!(s.vertex_count(), s2.vertex_count());
        assert_eq!(s.edge_count(), s2.edge_count());
        assert_eq!(s.face_count(), s2.face_count());
        validate(&s2.topo).unwrap();
    }

    #[test]
    fn json_round_trip_hex_pyramid() {
        let s = cone_faceted(3.0, 1.5, 6);
        let mut buf = Vec::new();
        write_json(&s, &mut buf).unwrap();
        let s2 = read_json(&mut buf.as_slice()).unwrap();
        assert_eq!(s.vertex_count(), s2.vertex_count());
        assert_eq!(s.edge_count(), s2.edge_count());
        assert_eq!(s.face_count(), s2.face_count());
        validate(&s2.topo).unwrap();
    }

    #[test]
    fn apex_is_at_origin_plus_height() {
        let h = 3.7_f64;
        let s = cone_faceted(1.0, h, 5);
        let positions: Vec<_> = s.vertex_geom.values().copied().collect();
        let apex_found = positions
            .iter()
            .any(|p| p.x.abs() < 1e-12 && p.y.abs() < 1e-12 && (p.z - h).abs() < 1e-12);
        assert!(apex_found, "apex at (0, 0, {h}) not found in vertex positions");
    }

    #[test]
    fn base_vertices_lie_on_circle() {
        let r = 2.5_f64;
        let s = cone_faceted(r, 1.0, 8);
        let positions: Vec<_> = s.vertex_geom.values().copied().collect();
        let base_verts: Vec<_> = positions.iter().filter(|p| p.z.abs() < 1e-12).collect();
        assert_eq!(base_verts.len(), 8, "expected 8 base vertices at z=0");
        for p in &base_verts {
            let dist = (p.x * p.x + p.y * p.y).sqrt();
            assert!(
                (dist - r).abs() < 1e-12,
                "base vertex at distance {dist} from z-axis, expected {r}"
            );
        }
    }
}
