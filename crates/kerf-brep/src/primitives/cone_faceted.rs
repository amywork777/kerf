//! `cone_faceted(r, h, n)` — n-sided pyramid with a regular n-gon base.
//!
//! Unlike [`cone`], the lateral surface is `n` planar triangles instead of one
//! analytic `Cone` surface. Plays cleanly with the boolean engine for the same
//! reasons `cylinder_faceted` does — every face is planar.
//!
//! Topology (Euler: V = n+1, E = 2n, F = n+1, S = 1 → (n+1) - 2n + (n+1) = 2 ✓):
//!   - n+1 vertices: apex at (0, 0, h) + n base vertices on a circle of radius r at z=0.
//!   - 2n edges: n base edges (forming the base n-gon) + n side edges (apex ↔ base_i).
//!   - n+1 faces: 1 base n-gon + n triangular sides.
//!
//! Built via direct slotmap construction in the kerf_topo build module
//! (the Euler operator chain mev/mef would be awkward here because the
//! apex is a single shared vertex across all side faces).

use std::f64::consts::PI;

use kerf_geom::{Frame, Line, Plane, Point3, Vec3};
use kerf_topo::{validate, HalfEdgeId, VertexId};

use crate::geometry::{CurveSegment, SurfaceKind};
use crate::Solid;

/// Construct an n-sided pyramid with a regular n-gon base of radius `r` at
/// z=0 and an apex at (0, 0, h). Phase-shifted by π/n so n=4 gives a
/// square pyramid with vertices at (±s, ±s, 0) — matching cylinder_faceted's
/// convention.
///
/// # Panics
/// In debug builds: if `r <= 0`, `h <= 0`, or `n < 3`.
pub fn cone_faceted(r: f64, h: f64, n: usize) -> Solid {
    debug_assert!(r > 0.0, "radius must be positive");
    debug_assert!(h > 0.0, "height must be positive");
    debug_assert!(n >= 3, "n must be at least 3");

    let mut s = Solid::new();

    // ---- 1. Vertices ----
    let v_apex = s.topo.build_insert_vertex();
    s.vertex_geom.insert(v_apex, Point3::new(0.0, 0.0, h));
    let phase = PI / n as f64;
    let v_base: Vec<VertexId> = (0..n)
        .map(|i| {
            let theta = phase + 2.0 * PI * i as f64 / n as f64;
            let p = Point3::new(r * theta.cos(), r * theta.sin(), 0.0);
            let v = s.topo.build_insert_vertex();
            s.vertex_geom.insert(v, p);
            v
        })
        .collect();

    // ---- 2. Solid + Shell ----
    let solid_id = s.topo.build_insert_solid();
    s.topo.build_set_active_solid(Some(solid_id));
    let shell_id = s.topo.build_insert_shell(solid_id);

    // ---- 3. Loops + Faces ----
    let base_loop = s.topo.build_insert_loop_placeholder();
    let base_face = s.topo.build_insert_face(base_loop, shell_id);
    s.topo.build_set_loop_face(base_loop, base_face);
    s.topo.build_push_shell_face(shell_id, base_face);

    let mut side_loops_faces = Vec::with_capacity(n);
    for _ in 0..n {
        let lp = s.topo.build_insert_loop_placeholder();
        let fc = s.topo.build_insert_face(lp, shell_id);
        s.topo.build_set_loop_face(lp, fc);
        s.topo.build_push_shell_face(shell_id, fc);
        side_loops_faces.push((lp, fc));
    }

    // ---- 4. Half-edges ----
    // Base loop walks v_0 → v_{n-1} → v_{n-2} → ... → v_1 → v_0 (CCW from
    // outward base normal -z). walk[k] is the vertex index at position k in
    // the loop walk.
    let walk: Vec<usize> = (0..n).map(|k| if k == 0 { 0 } else { n - k }).collect();
    let base_he: Vec<HalfEdgeId> = walk
        .iter()
        .map(|&k| s.topo.build_insert_half_edge(v_base[k], base_loop))
        .collect();
    for i in 0..n {
        let nxt = (i + 1) % n;
        s.topo.build_set_half_edge_next_prev(base_he[i], base_he[nxt]);
    }
    s.topo.build_set_loop_half_edge(base_loop, Some(base_he[0]));

    // Side face_i loop walks apex → v_i → v_{i+1} → apex (CCW from outward).
    // 3 half-edges per face: he[0] origin apex, he[1] origin v_i, he[2] origin v_{i+1}.
    let mut side_he: Vec<[HalfEdgeId; 3]> = Vec::with_capacity(n);
    for i in 0..n {
        let (lp, _) = side_loops_faces[i];
        let he_apex = s.topo.build_insert_half_edge(v_apex, lp);
        let he_vi = s.topo.build_insert_half_edge(v_base[i], lp);
        let he_vip1 = s.topo.build_insert_half_edge(v_base[(i + 1) % n], lp);
        s.topo.build_set_half_edge_next_prev(he_apex, he_vi);
        s.topo.build_set_half_edge_next_prev(he_vi, he_vip1);
        s.topo.build_set_half_edge_next_prev(he_vip1, he_apex);
        s.topo.build_set_loop_half_edge(lp, Some(he_apex));
        side_he.push([he_apex, he_vi, he_vip1]);
    }

    // ---- 5. Twins ----
    // Base edges: edge between v_i and v_{i+1}. Base loop walks the BACK
    // direction (v_{i+1} → v_i), side face_i walks the FORWARD (v_i → v_{i+1}).
    // base_he[k] has origin walk[k]. For the base half-edge that goes
    // v_{i+1} → v_i, walk[k] = i+1 and walk[k+1] = i. With my walk:
    //   walk[k] = i+1: solve. If i+1 = 0 (i.e. wrap, base edge between v_{n-1} and v_0), k = 0.
    //   Otherwise k = n - (i+1).
    // Equivalently: base edge i is at base_he index (n - i - 1) mod n.
    for i in 0..n {
        let base_idx = (n - i - 1) % n;
        let base_he_id = base_he[base_idx];
        let side_he_id = side_he[i][1]; // origin v_i, going to v_{i+1}
        s.topo.build_set_half_edge_twin(base_he_id, side_he_id);
        s.topo.build_set_half_edge_twin(side_he_id, base_he_id);
    }

    // Side edges: edge (apex, v_i) is shared by face_i (he[0] = apex→v_i)
    // and face_{i-1} (he[2] = v_i→apex, since (i-1)+1 = i).
    for i in 0..n {
        let prev_i = (i + n - 1) % n;
        let he_apex_i = side_he[i][0];
        let he_back_prev = side_he[prev_i][2];
        s.topo.build_set_half_edge_twin(he_apex_i, he_back_prev);
        s.topo.build_set_half_edge_twin(he_back_prev, he_apex_i);
    }

    // ---- 6. Edges ----
    for i in 0..n {
        let base_idx = (n - i - 1) % n;
        let base_he_id = base_he[base_idx];
        let side_he_id = side_he[i][1];
        let edge = s.topo.build_insert_edge([base_he_id, side_he_id]);
        s.topo.build_set_half_edge_edge(base_he_id, edge);
        s.topo.build_set_half_edge_edge(side_he_id, edge);
        let p0 = s.vertex_geom[v_base[i]];
        let p1 = s.vertex_geom[v_base[(i + 1) % n]];
        let line = Line::through(p0, p1).expect("base edge has length");
        let length = (p1 - p0).norm();
        s.edge_geom.insert(edge, CurveSegment::line(line, 0.0, length));
    }
    for i in 0..n {
        let prev_i = (i + n - 1) % n;
        let he_apex_i = side_he[i][0];
        let he_back_prev = side_he[prev_i][2];
        let edge = s.topo.build_insert_edge([he_apex_i, he_back_prev]);
        s.topo.build_set_half_edge_edge(he_apex_i, edge);
        s.topo.build_set_half_edge_edge(he_back_prev, edge);
        let p0 = s.vertex_geom[v_apex];
        let p1 = s.vertex_geom[v_base[i]];
        let line = Line::through(p0, p1).expect("side edge has length");
        let length = (p1 - p0).norm();
        s.edge_geom.insert(edge, CurveSegment::line(line, 0.0, length));
    }

    // ---- 7. Vertex outgoing ----
    s.topo.build_set_vertex_outgoing(v_apex, Some(side_he[0][0]));
    for i in 0..n {
        s.topo
            .build_set_vertex_outgoing(v_base[i], Some(side_he[i][1]));
    }

    // ---- 8. Face geometry ----
    // Base: plane at z=0 with outward normal -z. Right-handed frame:
    // x = +x, y = -y, z = -z (so x × y = (1,0,0) × (0,-1,0) = (0,0,-1) = z).
    let base_frame = Frame {
        origin: Point3::origin(),
        x: Vec3::x(),
        y: -Vec3::y(),
        z: -Vec3::z(),
    };
    s.face_geom
        .insert(base_face, SurfaceKind::Plane(Plane::new(base_frame)));

    // Side faces: planar with outward normal pointing away from the cone axis.
    let p_apex = s.vertex_geom[v_apex];
    for i in 0..n {
        let (_, fc) = side_loops_faces[i];
        let p_vi = s.vertex_geom[v_base[i]];
        let p_vip1 = s.vertex_geom[v_base[(i + 1) % n]];
        let mut normal = (p_vi - p_apex).cross(&(p_vip1 - p_apex));
        let nn = normal.norm();
        debug_assert!(nn > 1e-12, "degenerate side face {i}");
        normal /= nn;
        let seed = if normal.dot(&Vec3::x()).abs() < 0.9 {
            Vec3::x()
        } else {
            Vec3::y()
        };
        let x = (seed - normal * seed.dot(&normal)).normalize();
        let y = normal.cross(&x);
        let frame = Frame {
            origin: p_apex,
            x,
            y,
            z: normal,
        };
        s.face_geom.insert(fc, SurfaceKind::Plane(Plane::new(frame)));
    }

    validate(&s.topo).expect("cone_faceted topology violates Euler invariant");
    s
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::solid_volume;

    #[test]
    fn square_pyramid_topology() {
        let s = cone_faceted(1.0, 2.0, 4);
        assert_eq!(s.vertex_count(), 5); // n + 1
        assert_eq!(s.edge_count(), 8); // 2n
        assert_eq!(s.face_count(), 5); // n + 1
        assert_eq!(s.shell_count(), 1);
        validate(&s.topo).unwrap();
    }

    #[test]
    fn pentagonal_pyramid_topology() {
        let s = cone_faceted(1.0, 3.0, 5);
        assert_eq!(s.vertex_count(), 6);
        assert_eq!(s.edge_count(), 10);
        assert_eq!(s.face_count(), 6);
        validate(&s.topo).unwrap();
    }

    #[test]
    fn high_n_pyramid_validates() {
        let s = cone_faceted(2.0, 5.0, 32);
        assert_eq!(s.vertex_count(), 33);
        assert_eq!(s.edge_count(), 64);
        assert_eq!(s.face_count(), 33);
        validate(&s.topo).unwrap();
    }

    #[test]
    fn pyramid_volume_matches_formula() {
        // Square pyramid (n=4) with phase π/4: vertices at
        //   (cos(π/4 + k·π/2), sin(...)) * r for k=0..3
        // For r=1: vertices at (√2/2, √2/2), (-√2/2, √2/2), etc.
        // The base is a square inscribed in a circle of radius r — side
        // length = r·√2, area = 2r².
        // Volume = (1/3) * base_area * height = (1/3) * 2 * 2 = 4/3.
        let s = cone_faceted(1.0, 2.0, 4);
        let v = solid_volume(&s);
        assert!((v - 4.0 / 3.0).abs() < 1e-9, "v={v}, exp=4/3");
    }

    #[test]
    fn high_n_pyramid_volume_approaches_cone() {
        // For large n, V → (1/3)·π·r²·h = π/3 ≈ 1.0472.
        let s = cone_faceted(1.0, 1.0, 64);
        let v = solid_volume(&s);
        let exp = std::f64::consts::PI / 3.0;
        let rel = (v - exp).abs() / exp;
        assert!(rel < 0.01, "v={v}, exp={exp}, rel={rel}");
    }
}
