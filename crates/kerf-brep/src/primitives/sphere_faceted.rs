//! `sphere_faceted(r, stacks, slices)` — UV-style faceted sphere.
//!
//! Unlike the analytic [`sphere`], this primitive is built from planar
//! triangles (at the poles) and quads (in the middle bands), so it
//! composes with the kerf boolean engine for the same reason
//! [`cylinder_faceted`] does. The trade-off is that you need enough
//! `stacks * slices` to look round.
//!
//! Topology (Euler V - E + F = 2 ✓):
//!   - V = 2 + (S - 1) * M
//!   - E = M * (2S - 1)
//!   - F = M * S
//!   - S = 1
//!
//! Where S = stacks (number of latitude BANDS, ≥ 2) and M = slices
//! (number of longitude SEGMENTS, ≥ 3). For S = 2 there are no middle
//! quad bands, just the two polar fans meeting at the equator.
//!
//! [`sphere`]: super::sphere
//! [`cylinder_faceted`]: super::cylinder_faceted

use std::collections::HashMap;
use std::f64::consts::PI;

use kerf_geom::{Frame, Line, Plane, Point3, Vec3};
use kerf_topo::{validate, HalfEdgeId, VertexId};

use crate::geometry::{CurveSegment, SurfaceKind};
use crate::Solid;

/// Build a UV sphere of radius `r`, centered on the origin, with `stacks`
/// latitude bands (≥ 2) and `slices` longitude segments (≥ 3). North pole
/// at +z, south pole at -z, ring vertices phase-shifted by 0 (vertex 0 of
/// each ring lies on the +x half-plane).
///
/// # Panics
/// In debug builds: `r <= 0`, `stacks < 2`, or `slices < 3`.
pub fn sphere_faceted(r: f64, stacks: usize, slices: usize) -> Solid {
    debug_assert!(r > 0.0);
    debug_assert!(stacks >= 2, "stacks must be at least 2");
    debug_assert!(slices >= 3, "slices must be at least 3");

    let s_st = stacks;
    let s_sl = slices;
    let mut s = Solid::new();

    // ---- 1. Vertices ----
    let v_north = s.topo.build_insert_vertex();
    s.vertex_geom.insert(v_north, Point3::new(0.0, 0.0, r));
    let v_south = s.topo.build_insert_vertex();
    s.vertex_geom.insert(v_south, Point3::new(0.0, 0.0, -r));
    // ring[k][m] for k in 0..(s_st - 1), m in 0..s_sl
    let mut ring: Vec<Vec<VertexId>> = Vec::with_capacity(s_st - 1);
    for k in 0..(s_st - 1) {
        let theta = PI * (k + 1) as f64 / s_st as f64;
        let z = r * theta.cos();
        let rk = r * theta.sin();
        let mut row = Vec::with_capacity(s_sl);
        for m in 0..s_sl {
            let phi = 2.0 * PI * m as f64 / s_sl as f64;
            let x = rk * phi.cos();
            let y = rk * phi.sin();
            let v = s.topo.build_insert_vertex();
            s.vertex_geom.insert(v, Point3::new(x, y, z));
            row.push(v);
        }
        ring.push(row);
    }

    // ---- 2. Solid + Shell ----
    let solid_id = s.topo.build_insert_solid();
    s.topo.build_set_active_solid(Some(solid_id));
    let shell_id = s.topo.build_insert_shell(solid_id);

    // ---- 3. Faces + their half-edges ----
    // We'll record each face's half-edges in a uniform structure so we can
    // pair up twins later. Each face is described by an ordered list of
    // (origin_vertex, half_edge_id).
    let mut face_walks: Vec<Vec<(VertexId, HalfEdgeId)>> = Vec::new();
    // Edge keys: canonical (lo_v, hi_v) → list of half_edge IDs that lie on
    // that edge. We'll use this to wire twins in stage 5.
    let mut edge_pairs: HashMap<(VertexId, VertexId), Vec<HalfEdgeId>> = HashMap::new();
    // Vertex → outgoing half-edge (just need any one).
    let mut vert_outgoing: HashMap<VertexId, HalfEdgeId> = HashMap::new();

    let key = |a: VertexId, b: VertexId| -> (VertexId, VertexId) {
        // Sort by Debug-formatted comparison (slotmap keys don't implement
        // Ord directly). Within one solid, this is a stable ordering.
        let astr = format!("{a:?}");
        let bstr = format!("{b:?}");
        if astr < bstr {
            (a, b)
        } else {
            (b, a)
        }
    };

    // Helper to add a face (vertices in CCW-from-outward walk order).
    let add_face = |s: &mut Solid,
                        verts: &[VertexId],
                        edge_pairs: &mut HashMap<(VertexId, VertexId), Vec<HalfEdgeId>>,
                        vert_outgoing: &mut HashMap<VertexId, HalfEdgeId>|
     -> Vec<(VertexId, HalfEdgeId)> {
        let lp = s.topo.build_insert_loop_placeholder();
        let fc = s.topo.build_insert_face(lp, shell_id);
        s.topo.build_set_loop_face(lp, fc);
        s.topo.build_push_shell_face(shell_id, fc);

        let n = verts.len();
        let he_ids: Vec<HalfEdgeId> = verts
            .iter()
            .map(|&v| s.topo.build_insert_half_edge(v, lp))
            .collect();
        for i in 0..n {
            let nxt = (i + 1) % n;
            s.topo.build_set_half_edge_next_prev(he_ids[i], he_ids[nxt]);
            // Record edge pair under canonical key (verts[i], verts[i+1])
            let k = key(verts[i], verts[nxt]);
            edge_pairs.entry(k).or_default().push(he_ids[i]);
        }
        s.topo.build_set_loop_half_edge(lp, Some(he_ids[0]));
        // Outgoing for each vertex (pick the first half-edge whose origin is that vertex).
        for (i, &v) in verts.iter().enumerate() {
            vert_outgoing.entry(v).or_insert(he_ids[i]);
        }
        verts
            .iter()
            .copied()
            .zip(he_ids.iter().copied())
            .collect()
    };

    // North fan: M triangles. Face m walks (north, ring[0][m], ring[0][(m+1)%M]).
    for m in 0..s_sl {
        let mp1 = (m + 1) % s_sl;
        let walk = add_face(
            &mut s,
            &[v_north, ring[0][m], ring[0][mp1]],
            &mut edge_pairs,
            &mut vert_outgoing,
        );
        face_walks.push(walk);
    }

    // Middle bands: (S-2) bands. Band k has M quads. Quad m walks
    // (ring[k][m], ring[k+1][m], ring[k+1][m+1], ring[k][m+1]) — verified
    // CCW from outward normal by direct cross-product check.
    for k in 0..s_st.saturating_sub(2) {
        for m in 0..s_sl {
            let mp1 = (m + 1) % s_sl;
            let walk = add_face(
                &mut s,
                &[ring[k][m], ring[k + 1][m], ring[k + 1][mp1], ring[k][mp1]],
                &mut edge_pairs,
                &mut vert_outgoing,
            );
            face_walks.push(walk);
        }
    }

    // South fan: M triangles. Face m walks (south, ring[S-2][(m+1)%M], ring[S-2][m]).
    let last_ring = s_st - 2;
    for m in 0..s_sl {
        let mp1 = (m + 1) % s_sl;
        let walk = add_face(
            &mut s,
            &[v_south, ring[last_ring][mp1], ring[last_ring][m]],
            &mut edge_pairs,
            &mut vert_outgoing,
        );
        face_walks.push(walk);
    }

    // ---- 4. Edges + twin wiring ----
    for ((u, v), half_edges) in &edge_pairs {
        debug_assert_eq!(
            half_edges.len(),
            2,
            "edge {:?} has {} half-edges (expected 2)",
            (u, v),
            half_edges.len()
        );
        let he_a = half_edges[0];
        let he_b = half_edges[1];
        s.topo.build_set_half_edge_twin(he_a, he_b);
        s.topo.build_set_half_edge_twin(he_b, he_a);
        let edge = s.topo.build_insert_edge([he_a, he_b]);
        s.topo.build_set_half_edge_edge(he_a, edge);
        s.topo.build_set_half_edge_edge(he_b, edge);

        // Edge geometry: line between the two endpoints.
        let v_a = s
            .topo
            .half_edge(he_a)
            .expect("half-edge present")
            .origin();
        let v_b = s
            .topo
            .half_edge(he_b)
            .expect("half-edge present")
            .origin();
        let p_a = s.vertex_geom[v_a];
        let p_b = s.vertex_geom[v_b];
        if let Some(line) = Line::through(p_a, p_b) {
            let length = (p_b - p_a).norm();
            s.edge_geom
                .insert(edge, CurveSegment::line(line, 0.0, length));
        }
    }

    // ---- 5. Vertex outgoing ----
    for (v, he) in &vert_outgoing {
        s.topo.build_set_vertex_outgoing(*v, Some(*he));
    }

    // ---- 6. Face geometry: planar best-fit, outward normal away from origin ----
    let face_ids: Vec<_> = s.topo.face_ids().collect();
    for (idx, fid) in face_ids.iter().enumerate() {
        let walk = &face_walks[idx];
        let p0 = s.vertex_geom[walk[0].0];
        let p1 = s.vertex_geom[walk[1].0];
        let p2 = s.vertex_geom[walk[2].0];
        let mut normal = (p1 - p0).cross(&(p2 - p0));
        let nn = normal.norm();
        debug_assert!(nn > 1e-12, "degenerate sphere face {idx}");
        normal /= nn;
        // Sphere centroid is the origin; outward = normal direction matches
        // dot(normal, centroid_face) > 0.
        let centroid = walk
            .iter()
            .fold(Vec3::zeros(), |acc, (v, _)| acc + s.vertex_geom[*v].coords)
            / walk.len() as f64;
        if normal.dot(&centroid) < 0.0 {
            normal = -normal;
        }
        let seed = if normal.dot(&Vec3::x()).abs() < 0.9 {
            Vec3::x()
        } else {
            Vec3::y()
        };
        let x = (seed - normal * seed.dot(&normal)).normalize();
        let y = normal.cross(&x);
        let frame = Frame {
            origin: p0,
            x,
            y,
            z: normal,
        };
        s.face_geom.insert(*fid, SurfaceKind::Plane(Plane::new(frame)));
    }

    validate(&s.topo).expect("sphere_faceted topology violates Euler invariant");
    s
}


#[cfg(test)]
mod tests {
    use super::*;
    use crate::solid_volume;

    #[test]
    fn icosphere_minimal_topology() {
        // S=2, M=8: V = 2 + (2-1)*8 = 10. E = 8*(2*2-1) = 24. F = 8*2 = 16.
        let s = sphere_faceted(1.0, 2, 8);
        assert_eq!(s.vertex_count(), 10);
        assert_eq!(s.edge_count(), 24);
        assert_eq!(s.face_count(), 16);
        validate(&s.topo).unwrap();
    }

    #[test]
    fn medium_uv_sphere_topology() {
        // S=4, M=8: V = 2 + 3*8 = 26. E = 8*7 = 56. F = 8*4 = 32.
        let s = sphere_faceted(1.0, 4, 8);
        assert_eq!(s.vertex_count(), 26);
        assert_eq!(s.edge_count(), 56);
        assert_eq!(s.face_count(), 32);
        validate(&s.topo).unwrap();
    }

    #[test]
    fn fine_uv_sphere_topology() {
        // S=8, M=16: V = 2 + 7*16 = 114. E = 16*15 = 240. F = 16*8 = 128.
        let s = sphere_faceted(1.0, 8, 16);
        assert_eq!(s.vertex_count(), 114);
        assert_eq!(s.edge_count(), 240);
        assert_eq!(s.face_count(), 128);
        validate(&s.topo).unwrap();
    }

    #[test]
    fn sphere_faceted_volume_approaches_analytic() {
        // For a fine UV sphere (S=20, M=20), volume should be within ~5% of
        // the analytic 4/3 π r³.
        let s = sphere_faceted(1.0, 20, 20);
        let v = solid_volume(&s);
        let exp = (4.0 / 3.0) * PI;
        let rel = (v - exp).abs() / exp;
        assert!(rel < 0.05, "v={v}, exp={exp}, rel={rel}");
    }

    #[test]
    fn sphere_faceted_low_n_volume_is_inscribed() {
        // S=4, M=8 sphere of radius 1: volume should be POSITIVE and less
        // than the analytic 4π/3 ≈ 4.19 (inscribed).
        let s = sphere_faceted(1.0, 4, 8);
        let v = solid_volume(&s);
        assert!(v > 0.0, "v should be positive (got {v})");
        assert!(v < (4.0 / 3.0) * PI, "v should be inscribed");
    }
}
