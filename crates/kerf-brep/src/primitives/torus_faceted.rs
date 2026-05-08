//! `torus_faceted(major_radius, minor_radius, major_segs, minor_segs)` —
//! faceted torus built from planar quad faces, suitable for boolean ops.
//!
//! Unlike the analytic [`torus`], this primitive's faces are planar best-fit
//! quads — the price for being able to compose with the kerf boolean engine.
//! A torus has genus 1, so its Euler invariant is V - E + F = 0 (not 2).
//!
//! Topology:
//!   - V = M * N
//!   - E = 2 * M * N (each face contributes 4 edges, each edge shared)
//!   - F = M * N
//!   - S = 1
//!
//! Where M = `major_segs` (number of toroidal divisions, ≥ 3) and N =
//! `minor_segs` (number of poloidal divisions, ≥ 3). Vertex (i, j) lives at
//! (cos(u)(R + r cos(v)), sin(u)(R + r cos(v)), r sin(v)) for u = 2πi/M,
//! v = 2πj/N.
//!
//! [`torus`]: super::torus

use std::collections::HashMap;
use std::f64::consts::PI;

use kerf_geom::{Frame, Line, Plane, Point3, Vec3};
use kerf_topo::{HalfEdgeId, VertexId};

use crate::geometry::{CurveSegment, SurfaceKind};
use crate::Solid;

/// Build a faceted torus with major radius `r_major` (≥ minor + a small
/// margin), minor radius `r_minor`, axis along +z, centered on the origin.
/// `major_segs` and `minor_segs` are the toroidal and poloidal subdivisions
/// respectively (both ≥ 3).
///
/// # Panics
/// In debug builds: any radius ≤ 0, segment count < 3, or
/// `r_major <= r_minor` (the torus would self-intersect).
pub fn torus_faceted(
    r_major: f64,
    r_minor: f64,
    major_segs: usize,
    minor_segs: usize,
) -> Solid {
    debug_assert!(r_major > 0.0);
    debug_assert!(r_minor > 0.0);
    debug_assert!(r_major > r_minor, "torus_faceted: major must exceed minor");
    debug_assert!(major_segs >= 3, "major_segs must be at least 3");
    debug_assert!(minor_segs >= 3, "minor_segs must be at least 3");

    let m = major_segs;
    let n = minor_segs;
    let mut s = Solid::new();

    // ---- 1. Vertices: ring[i][j] for i in 0..M, j in 0..N. ----
    let mut ring: Vec<Vec<VertexId>> = Vec::with_capacity(m);
    for i in 0..m {
        let u = 2.0 * PI * i as f64 / m as f64;
        let cu = u.cos();
        let su = u.sin();
        let mut row = Vec::with_capacity(n);
        for j in 0..n {
            let v = 2.0 * PI * j as f64 / n as f64;
            let cv = v.cos();
            let sv = v.sin();
            let radial = r_major + r_minor * cv;
            let p = Point3::new(cu * radial, su * radial, r_minor * sv);
            let vid = s.topo.build_insert_vertex();
            s.vertex_geom.insert(vid, p);
            row.push(vid);
        }
        ring.push(row);
    }

    // ---- 2. Solid + Shell ----
    let solid_id = s.topo.build_insert_solid();
    s.topo.build_set_active_solid(Some(solid_id));
    let shell_id = s.topo.build_insert_shell(solid_id);

    // ---- 3. Faces (M * N quads) ----
    let mut face_walks: Vec<Vec<(VertexId, HalfEdgeId)>> = Vec::new();
    let mut edge_pairs: HashMap<(VertexId, VertexId), Vec<HalfEdgeId>> = HashMap::new();
    let mut vert_outgoing: HashMap<VertexId, HalfEdgeId> = HashMap::new();

    let key = |a: VertexId, b: VertexId| -> (VertexId, VertexId) {
        // slotmap keys lack Ord, so canonicalise via Debug.
        let astr = format!("{a:?}");
        let bstr = format!("{b:?}");
        if astr < bstr { (a, b) } else { (b, a) }
    };

    let add_face = |s: &mut Solid,
                    verts: &[VertexId],
                    edge_pairs: &mut HashMap<(VertexId, VertexId), Vec<HalfEdgeId>>,
                    vert_outgoing: &mut HashMap<VertexId, HalfEdgeId>|
     -> Vec<(VertexId, HalfEdgeId)> {
        let lp = s.topo.build_insert_loop_placeholder();
        let fc = s.topo.build_insert_face(lp, shell_id);
        s.topo.build_set_loop_face(lp, fc);
        s.topo.build_push_shell_face(shell_id, fc);
        let count = verts.len();
        let he_ids: Vec<HalfEdgeId> = verts
            .iter()
            .map(|&v| s.topo.build_insert_half_edge(v, lp))
            .collect();
        for k in 0..count {
            let nxt = (k + 1) % count;
            s.topo.build_set_half_edge_next_prev(he_ids[k], he_ids[nxt]);
            let eid = key(verts[k], verts[nxt]);
            edge_pairs.entry(eid).or_default().push(he_ids[k]);
        }
        s.topo.build_set_loop_half_edge(lp, Some(he_ids[0]));
        for (k, &v) in verts.iter().enumerate() {
            vert_outgoing.entry(v).or_insert(he_ids[k]);
        }
        verts.iter().copied().zip(he_ids.iter().copied()).collect()
    };

    // Walk: (ring[i][j], ring[i+1][j], ring[i+1][j+1], ring[i][j+1]).
    // Indices wrap modulo M (toroidal) and N (poloidal). This walk is CCW
    // when viewed from outside the torus (the outward normal direction).
    for i in 0..m {
        let ip1 = (i + 1) % m;
        for j in 0..n {
            let jp1 = (j + 1) % n;
            let walk = add_face(
                &mut s,
                &[ring[i][j], ring[ip1][j], ring[ip1][jp1], ring[i][jp1]],
                &mut edge_pairs,
                &mut vert_outgoing,
            );
            face_walks.push(walk);
        }
    }

    // ---- 4. Edges + twins ----
    for ((u, v), half_edges) in &edge_pairs {
        debug_assert_eq!(
            half_edges.len(),
            2,
            "torus_faceted edge {:?} has {} half-edges (expected 2)",
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
        let v_a = s.topo.half_edge(he_a).expect("present").origin();
        let v_b = s.topo.half_edge(he_b).expect("present").origin();
        let p_a = s.vertex_geom[v_a];
        let p_b = s.vertex_geom[v_b];
        if let Some(line) = Line::through(p_a, p_b) {
            let length = (p_b - p_a).norm();
            s.edge_geom.insert(edge, CurveSegment::line(line, 0.0, length));
        }
    }

    // ---- 5. Vertex outgoing ----
    for (v, he) in &vert_outgoing {
        s.topo.build_set_vertex_outgoing(*v, Some(*he));
    }

    // ---- 6. Face geometry: planar best-fit, outward normal points away from
    //         the tube's center axis (toroidal axis). ----
    let face_ids: Vec<_> = s.topo.face_ids().collect();
    for (idx, fid) in face_ids.iter().enumerate() {
        let walk = &face_walks[idx];
        let p0 = s.vertex_geom[walk[0].0];
        let p1 = s.vertex_geom[walk[1].0];
        let p2 = s.vertex_geom[walk[2].0];
        let mut normal = (p1 - p0).cross(&(p2 - p0));
        let nn = normal.norm();
        debug_assert!(nn > 1e-12, "degenerate torus face {idx}");
        normal /= nn;
        // Tube-center for this face's centroid: project the centroid onto
        // the major circle (z=0 ring of radius r_major centered on origin).
        // Outward normal points from tube-center toward face centroid.
        let centroid = walk
            .iter()
            .fold(Vec3::zeros(), |acc, (v, _)| acc + s.vertex_geom[*v].coords)
            / walk.len() as f64;
        let toroidal_xy = Vec3::new(centroid.x, centroid.y, 0.0);
        let toroidal_norm = toroidal_xy.norm();
        let tube_center = if toroidal_norm > 1e-12 {
            toroidal_xy * (r_major / toroidal_norm)
        } else {
            Vec3::zeros()
        };
        let outward = centroid - tube_center;
        if normal.dot(&outward) < 0.0 {
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

    // Note: we deliberately skip `validate(&s.topo)` here. validate's Euler
    // check assumes genus 0 (V - E + F = 2 per shell), but a torus has
    // genus 1 (V - E + F = 0). The other invariants — manifold edge pairing
    // and twin symmetry — are guaranteed by construction above.
    s
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::solid_volume;

    #[test]
    fn small_torus_topology() {
        // M=4, N=3: V = 12, E = 24, F = 12. Euler V - E + F = 0 (genus 1). ✓
        let t = torus_faceted(3.0, 1.0, 4, 3);
        assert_eq!(t.vertex_count(), 12);
        assert_eq!(t.edge_count(), 24);
        assert_eq!(t.face_count(), 12);
        assert_eq!(t.shell_count(), 1);
    }

    #[test]
    fn medium_torus_topology() {
        // M=8, N=6: V = 48, E = 96, F = 48. Euler 0. ✓
        let t = torus_faceted(3.0, 1.0, 8, 6);
        assert_eq!(t.vertex_count(), 48);
        assert_eq!(t.edge_count(), 96);
        assert_eq!(t.face_count(), 48);
    }

    #[test]
    fn fine_torus_volume_approaches_analytic() {
        // Analytic torus volume: 2π² R r².
        // For M=N=24, the faceted approximation should be within ~5%.
        let r_maj = 3.0;
        let r_min = 1.0;
        let t = torus_faceted(r_maj, r_min, 24, 24);
        let v = solid_volume(&t);
        let exp = 2.0 * PI * PI * r_maj * r_min * r_min;
        let rel = (v - exp).abs() / exp;
        assert!(rel < 0.06, "v={v}, exp={exp}, rel={rel}");
    }

    #[test]
    fn torus_volume_is_positive_for_low_n() {
        // Inscribed faceted torus must still have positive volume even at
        // the minimum subdivision counts.
        let t = torus_faceted(3.0, 1.0, 4, 3);
        let v = solid_volume(&t);
        assert!(v > 0.0, "low-n faceted torus has positive volume, got {v}");
    }
}
