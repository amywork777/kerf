//! `cone_faceted(r, h, n)` — n-gon pyramid approximation of a cone.
//!
//! Unlike [`super::cone`], every face is a flat plane: one base n-gon and
//! n triangular lateral faces. The all-planar topology means the solid
//! tessellates exactly and integrates with the planar boolean pipeline.
//!
//! Topology (Euler: V − E + F = (n+1) − 2n + (n+1) = 2 ✓):
//!   - V = n+1  (n base ring vertices + 1 apex)
//!   - E = 2n   (n base ring edges + n lateral edges from apex to ring)
//!   - F = n+1  (1 base n-gon + n triangular lateral faces)
//!
//! Analytic volume: V_cone = (n · r² · sin(2π/n) · h) / 6
//!                         = (1/3) · A_n · h
//! where A_n = (n · r² · sin(2π/n)) / 2 is the area of the inscribed n-gon.

use std::f64::consts::TAU;

use kerf_geom::{Frame, Line, Plane, Point3, Vec3};
use kerf_topo::validate;

use crate::geometry::{CurveSegment, SurfaceKind};
use crate::Solid;

/// Build an n-gon pyramid inscribed in a cone of base radius `r` and height `h`.
///
/// The base is a regular n-gon in the z = 0 plane (CCW from +z), with a single
/// apex vertex at (0, 0, h). All n+1 faces are planar. Higher n → closer to a
/// true cone; the result has n+1 vertices, 2n edges, and n+1 faces.
///
/// # Panics (debug)
/// Panics in debug mode if `r ≤ 0`, `h ≤ 0`, or `n < 3`.
pub fn cone_faceted(r: f64, h: f64, n: usize) -> Solid {
    debug_assert!(r > 0.0, "radius must be positive");
    debug_assert!(h > 0.0, "height must be positive");
    debug_assert!(n >= 3, "n must be at least 3");

    let mut s = Solid::new();

    // ── 1. Vertices ──────────────────────────────────────────────────────────
    let apex = s.topo.build_insert_vertex();
    s.vertex_geom.insert(apex, Point3::new(0.0, 0.0, h));

    // Base ring: b[i] at angle 2πi/n in the z=0 plane (CCW from +z).
    let bv: Vec<_> = (0..n)
        .map(|i| {
            let theta = TAU * i as f64 / n as f64;
            let vid = s.topo.build_insert_vertex();
            s.vertex_geom
                .insert(vid, Point3::new(r * theta.cos(), r * theta.sin(), 0.0));
            vid
        })
        .collect();

    // ── 2. Solid + Shell ─────────────────────────────────────────────────────
    let solid_id = s.topo.build_insert_solid();
    s.topo.build_set_active_solid(Some(solid_id));
    let shell = s.topo.build_insert_shell(solid_id);

    // ── 3. Faces + Loops ─────────────────────────────────────────────────────
    let base_lp = s.topo.build_insert_loop_placeholder();
    let base_face = s.topo.build_insert_face(base_lp, shell);
    s.topo.build_set_loop_face(base_lp, base_face);
    s.topo.build_push_shell_face(shell, base_face);

    let lat_lps: Vec<_> = (0..n)
        .map(|_| s.topo.build_insert_loop_placeholder())
        .collect();
    let lat_faces: Vec<_> = (0..n)
        .map(|i| {
            let lp = lat_lps[i];
            let f = s.topo.build_insert_face(lp, shell);
            s.topo.build_set_loop_face(lp, f);
            s.topo.build_push_shell_face(shell, f);
            f
        })
        .collect();

    // ── 4. Base half-edges ───────────────────────────────────────────────────
    // Outward normal of the base face is −z (pointing away from the enclosed
    // volume). CCW from −z = CW from +z = reverse ring order.
    //   base_he[j] origin: b[(n−j) % n]
    //   j=0: b[0], j=1: b[n-1], j=2: b[n-2], ..., j=n-1: b[1]
    let base_he: Vec<_> = (0..n)
        .map(|j| s.topo.build_insert_half_edge(bv[(n - j) % n], base_lp))
        .collect();
    for j in 0..n {
        s.topo
            .build_set_half_edge_next_prev(base_he[j], base_he[(j + 1) % n]);
    }
    s.topo.build_set_loop_half_edge(base_lp, Some(base_he[0]));

    // ── 5. Lateral half-edges ────────────────────────────────────────────────
    // Lateral face i covers triangle (apex, b[i], b[(i+1)%n]).
    // Winding apex→b[i]→b[(i+1)%n] gives outward normal (verified via volume).
    //   lat_he[i][0]: origin apex,       goes to b[i]           (lateral edge i)
    //   lat_he[i][1]: origin b[i],       goes to b[(i+1)%n]     (base ring edge i)
    //   lat_he[i][2]: origin b[(i+1)%n], goes to apex           (lateral edge (i+1)%n rev)
    let lat_he: Vec<[_; 3]> = (0..n)
        .map(|i| {
            let lp = lat_lps[i];
            let he0 = s.topo.build_insert_half_edge(apex, lp);
            let he1 = s.topo.build_insert_half_edge(bv[i], lp);
            let he2 = s.topo.build_insert_half_edge(bv[(i + 1) % n], lp);
            [he0, he1, he2]
        })
        .collect();
    for i in 0..n {
        s.topo
            .build_set_half_edge_next_prev(lat_he[i][0], lat_he[i][1]);
        s.topo
            .build_set_half_edge_next_prev(lat_he[i][1], lat_he[i][2]);
        s.topo
            .build_set_half_edge_next_prev(lat_he[i][2], lat_he[i][0]);
        s.topo.build_set_loop_half_edge(lat_lps[i], Some(lat_he[i][0]));
    }

    // ── 6. Twin pairs ────────────────────────────────────────────────────────
    // Base ring edge i: b[i]→b[(i+1)%n] ↔ b[(i+1)%n]→b[i]
    //   forward:  lat_he[i][1]        (in lateral face i)
    //   backward: base_he[n−1−i]      (in base face; origin b[(i+1)%n])
    for i in 0..n {
        s.topo
            .build_set_half_edge_twin(lat_he[i][1], base_he[n - 1 - i]);
        s.topo
            .build_set_half_edge_twin(base_he[n - 1 - i], lat_he[i][1]);
    }
    // Lateral edge i: apex→b[i] ↔ b[i]→apex
    //   forward:  lat_he[i][0]           (origin apex, in face i)
    //   backward: lat_he[(i-1+n)%n][2]   (origin b[i], in face i-1)
    for i in 0..n {
        let prev = (i + n - 1) % n;
        s.topo
            .build_set_half_edge_twin(lat_he[i][0], lat_he[prev][2]);
        s.topo
            .build_set_half_edge_twin(lat_he[prev][2], lat_he[i][0]);
    }

    // ── 7. Edges ─────────────────────────────────────────────────────────────
    // n base ring edges
    for i in 0..n {
        let e = s
            .topo
            .build_insert_edge([lat_he[i][1], base_he[n - 1 - i]]);
        s.topo.build_set_half_edge_edge(lat_he[i][1], e);
        s.topo.build_set_half_edge_edge(base_he[n - 1 - i], e);
    }
    // n lateral edges (apex ↔ b[i])
    for i in 0..n {
        let prev = (i + n - 1) % n;
        let e = s.topo.build_insert_edge([lat_he[i][0], lat_he[prev][2]]);
        s.topo.build_set_half_edge_edge(lat_he[i][0], e);
        s.topo.build_set_half_edge_edge(lat_he[prev][2], e);
    }

    // ── 8. Vertex outgoing half-edges ────────────────────────────────────────
    s.topo.build_set_vertex_outgoing(apex, Some(lat_he[0][0]));
    for i in 0..n {
        s.topo.build_set_vertex_outgoing(bv[i], Some(lat_he[i][1]));
    }

    validate(&s.topo).expect("cone_faceted topology violates Euler invariant");

    // ── 9. Edge geometry (line segments) ─────────────────────────────────────
    let edge_ids: Vec<_> = s.topo.edge_ids().collect();
    for eid in edge_ids {
        let edge = s.topo.edge(eid).unwrap();
        let [he_a, _] = edge.half_edges();
        let v0 = s.topo.half_edge(he_a).unwrap().origin();
        let twin_he = s.topo.half_edge(he_a).unwrap().twin();
        let v1 = s.topo.half_edge(twin_he).unwrap().origin();
        let p0 = *s.vertex_geom.get(v0).unwrap();
        let p1 = *s.vertex_geom.get(v1).unwrap();
        let line = Line::through(p0, p1).unwrap();
        let length = (p1 - p0).norm();
        s.edge_geom.insert(eid, CurveSegment::line(line, 0.0, length));
    }

    // ── 10. Face geometry (planes) ───────────────────────────────────────────
    // Base face: outward normal = −z (away from enclosed volume).
    {
        let frame = Frame {
            origin: Point3::origin(),
            x: Vec3::x(),
            y: Vec3::y(),
            z: -Vec3::z(),
        };
        s.face_geom
            .insert(base_face, SurfaceKind::Plane(Plane::new(frame)));
    }
    // Lateral face i: triangle apex, b[i], b[(i+1)%n]. Outward normal from
    // Frame::from_x_yhint(apex, e1=b[i]−apex, e2=b[i+1]−apex) where
    // e1 × e2 has positive z and positive radial component → outward. ✓
    for i in 0..n {
        let apex_pt = *s.vertex_geom.get(apex).unwrap();
        let bi_pt = *s.vertex_geom.get(bv[i]).unwrap();
        let bi1_pt = *s.vertex_geom.get(bv[(i + 1) % n]).unwrap();
        let e1 = bi_pt - apex_pt;
        let e2 = bi1_pt - apex_pt;
        let frame = Frame::from_x_yhint(apex_pt, e1, e2)
            .expect("non-degenerate lateral triangle");
        s.face_geom
            .insert(lat_faces[i], SurfaceKind::Plane(Plane::new(frame)));
    }

    s
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::TAU;

    use crate::serde_io::{read_json, write_json};
    use crate::solid_volume;

    fn analytic_volume(r: f64, h: f64, n: usize) -> f64 {
        (n as f64 * r * r * (TAU / n as f64).sin() * h) / 6.0
    }

    #[test]
    fn cone_faceted_tetrahedron_topology() {
        // n=3 gives a triangular pyramid (tetrahedron-like): V=4, E=6, F=4.
        let s = cone_faceted(1.0, 2.0, 3);
        assert_eq!(s.vertex_count(), 4);
        assert_eq!(s.edge_count(), 6);
        assert_eq!(s.face_count(), 4);
        assert_eq!(s.shell_count(), 1);
        validate(&s.topo).unwrap();
    }

    #[test]
    fn cone_faceted_square_pyramid_topology() {
        // n=4: V=5, E=8, F=5.
        let s = cone_faceted(1.0, 2.0, 4);
        assert_eq!(s.vertex_count(), 5);
        assert_eq!(s.edge_count(), 8);
        assert_eq!(s.face_count(), 5);
        validate(&s.topo).unwrap();
    }

    #[test]
    fn cone_faceted_8gon_topology() {
        // n=8: V=9, E=16, F=9.
        let s = cone_faceted(1.0, 2.0, 8);
        assert_eq!(s.vertex_count(), 9);
        assert_eq!(s.edge_count(), 16);
        assert_eq!(s.face_count(), 9);
        validate(&s.topo).unwrap();
    }

    #[test]
    fn cone_faceted_volume_matches_analytic_n3() {
        let s = cone_faceted(1.0, 3.0, 3);
        let got = solid_volume(&s);
        let want = analytic_volume(1.0, 3.0, 3);
        assert!(
            (got - want).abs() < 1e-10,
            "n=3: got={got} want={want}"
        );
    }

    #[test]
    fn cone_faceted_volume_matches_analytic_n6() {
        let s = cone_faceted(2.0, 4.0, 6);
        let got = solid_volume(&s);
        let want = analytic_volume(2.0, 4.0, 6);
        assert!(
            (got - want).abs() < 1e-10,
            "n=6: got={got} want={want}"
        );
    }

    #[test]
    fn cone_faceted_volume_matches_analytic_n32() {
        // n=32: very close to analytic cone volume (1/3)πr²h = (1/3)π·1²·1
        let r = 1.0_f64;
        let h = 1.0_f64;
        let n = 32_usize;
        let s = cone_faceted(r, h, n);
        let got = solid_volume(&s);
        let want = analytic_volume(r, h, n);
        // Also verify approach to π/3
        let true_cone_vol = std::f64::consts::PI * r * r * h / 3.0;
        assert!(
            (got - want).abs() < 1e-10,
            "n=32: got={got} want={want}"
        );
        // n=32 inscribed polygon is within 1% of the true cone (actual ~0.64%)
        assert!(
            (got - true_cone_vol).abs() / true_cone_vol < 0.01,
            "n=32 should be within 1% of true cone vol, got={got} true={true_cone_vol}"
        );
    }

    #[test]
    fn cone_faceted_all_faces_planar_all_edges_linear() {
        use crate::geometry::{CurveKind, SurfaceKind};
        let s = cone_faceted(1.5, 3.0, 8);
        for (_, surf) in &s.face_geom {
            assert!(matches!(surf, SurfaceKind::Plane(_)), "all faces must be planar");
        }
        for (_, seg) in &s.edge_geom {
            assert!(matches!(seg.curve, CurveKind::Line(_)), "all edges must be linear");
        }
    }

    #[test]
    fn cone_faceted_json_roundtrip() {
        let s = cone_faceted(1.0, 2.0, 6);
        let vol_before = solid_volume(&s);
        let v = s.vertex_count();
        let e = s.edge_count();
        let f = s.face_count();

        let mut buf = Vec::new();
        write_json(&s, &mut buf).expect("write_json");
        let s2 = read_json(&mut buf.as_slice()).expect("read_json");

        assert_eq!(s2.vertex_count(), v);
        assert_eq!(s2.edge_count(), e);
        assert_eq!(s2.face_count(), f);
        let vol_after = solid_volume(&s2);
        assert!(
            (vol_after - vol_before).abs() < 1e-10,
            "volume changed after JSON round-trip: {vol_before} → {vol_after}"
        );
        validate(&s2.topo).unwrap();
    }
}
