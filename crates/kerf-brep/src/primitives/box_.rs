//! `box_(extents)` — axis-aligned box at the origin built via Euler operators.
//!
//! Construction strategy:
//!   1. `mvfs` to seed the solid with corner v0.
//!   2. Six `mev` calls walk a Hamiltonian path over the cube graph
//!      v0 -> v1 -> v2 -> v3 -> v7 -> v4 -> v5 -> v6, growing 7 sticker
//!      edges. Anchor convention: the half-edge ENDING AT the latest tip
//!      (i.e. `m_prev.half_edges.0`).
//!   3. Five `mef` calls split the resulting 14-half-edge loop into 6 quads,
//!      adding the missing cube edges (v3,v0), (v6,v7), (v0,v4), (v1,v5),
//!      (v2,v6).
//!   4. Attach geometry: a `Point3` per vertex, a line `CurveSegment` per
//!      edge, and a `Plane` per face (frame oriented outward).

use kerf_geom::{Frame, Line, Plane, Point3, Vec3};
use kerf_topo::validate;

use crate::Solid;
use crate::geometry::{CurveSegment, SurfaceKind};

/// Construct an axis-aligned box at the origin with the given extents.
/// `extents.x/y/z` must all be positive.
pub fn box_(extents: Vec3) -> Solid {
    debug_assert!(extents.x > 0.0 && extents.y > 0.0 && extents.z > 0.0);
    let mut s = Solid::new();
    let ex = extents.x;
    let ey = extents.y;
    let ez = extents.z;

    // Eight corner positions.
    let p = [
        Point3::new(0.0, 0.0, 0.0), // 0
        Point3::new(ex, 0.0, 0.0),  // 1
        Point3::new(ex, ey, 0.0),   // 2
        Point3::new(0.0, ey, 0.0),  // 3
        Point3::new(0.0, 0.0, ez),  // 4
        Point3::new(ex, 0.0, ez),   // 5
        Point3::new(ex, ey, ez),    // 6
        Point3::new(0.0, ey, ez),   // 7
    ];

    // ---- Stage 1: mvfs to seed v0. ----
    let r = s.topo.mvfs();
    let outer_loop = r.loop_;
    let mut vids = [r.vertex; 8];
    s.vertex_geom.insert(vids[0], p[0]);

    // ---- Stage 2: 7 mev calls along the Hamiltonian path. ----
    // After each mev, the new tip's "outgoing-along-path" half-edge is
    // m.half_edges.0 (origin = previous tip, dest = new tip), so the next
    // mev anchors on .0.
    let m1 = s.topo.mev_at_lone_vertex(outer_loop, vids[0]); // (v0, v1)
    vids[1] = m1.vertex;
    s.vertex_geom.insert(vids[1], p[1]);

    let m2 = s.topo.mev(outer_loop, m1.half_edges.0); // (v1, v2)
    vids[2] = m2.vertex;
    s.vertex_geom.insert(vids[2], p[2]);

    let m3 = s.topo.mev(outer_loop, m2.half_edges.0); // (v2, v3)
    vids[3] = m3.vertex;
    s.vertex_geom.insert(vids[3], p[3]);

    let m4 = s.topo.mev(outer_loop, m3.half_edges.0); // (v3, v7)
    vids[7] = m4.vertex;
    s.vertex_geom.insert(vids[7], p[7]);

    let m5 = s.topo.mev(outer_loop, m4.half_edges.0); // (v7, v4)
    vids[4] = m5.vertex;
    s.vertex_geom.insert(vids[4], p[4]);

    let m6 = s.topo.mev(outer_loop, m5.half_edges.0); // (v4, v5)
    vids[5] = m6.vertex;
    s.vertex_geom.insert(vids[5], p[5]);

    let m7 = s.topo.mev(outer_loop, m6.half_edges.0); // (v5, v6)
    vids[6] = m7.vertex;
    s.vertex_geom.insert(vids[6], p[6]);

    validate(&s.topo).expect("topology violated after mev tree");

    // ---- Stage 3: 5 mef calls close the loop into 6 quads. ----
    // The 14-half-edge outer loop walks (forward then back):
    //   m1_a v0->v1, m2_a v1->v2, m3_a v2->v3, m4_a v3->v7,
    //   m5_a v7->v4, m6_a v4->v5, m7_a v5->v6,
    //   m7_b v6->v5, m6_b v5->v4, m5_b v4->v7, m4_b v7->v3,
    //   m3_b v3->v2, m2_b v2->v1, m1_b v1->v0.
    // Each mef passes (h1, h2) such that the new face's loop becomes
    //     he_b(v_h2 -> v_h1) -> h1 -> h1.next -> ... -> h2.prev -> he_b
    // — i.e. h1 is the start of the face's existing-edge chain and
    // h2.prev is its end.

    // mef 1: bottom face v0,v1,v2,v3 — new edge (v0, v3).
    //   h1 = m1_a (origin v0), h2 = m3_b (origin v3).
    let _ = s.topo.mef(m1.half_edges.0, m3.half_edges.1);

    // mef 2: top face v7,v4,v5,v6 — new edge (v7, v6).
    //   h1 = m5_a (origin v7), h2 = m7_b (origin v6).
    let _ = s.topo.mef(m5.half_edges.0, m7.half_edges.1);

    // mef 3: left face v0,v4,v7,v3 — new edge (v4, v0).
    //   h1 = m5_b (origin v4), h2 = m1_a (origin v0).
    let _ = s.topo.mef(m5.half_edges.1, m1.half_edges.0);

    // mef 4: front face v1,v5,v4,v0 — new edge (v5, v1).
    //   h1 = m6_b (origin v5), h2 = m2_a (origin v1).
    let _ = s.topo.mef(m6.half_edges.1, m2.half_edges.0);

    // mef 5: back face v6,v2,v3,v7 (new) and right face v2,v6,v5,v1 (remainder)
    // — new edge (v2, v6).  h1 = m3_a (origin v2), h2 = m7_b (origin v6).
    let _ = s.topo.mef(m3.half_edges.0, m7.half_edges.1);

    validate(&s.topo).expect("topology violated after mef closures");

    // ---- Stage 4: attach edge geometry (line segments). ----
    // For every edge in the topology, build a CurveSegment::line between
    // the two endpoint positions.
    let edge_ids: Vec<_> = s.topo.edge_ids().collect();
    for eid in edge_ids {
        let edge = s.topo.edge(eid).unwrap();
        let [he_a, _he_b] = edge.half_edges();
        let he = s.topo.half_edge(he_a).unwrap();
        let v_from = he.origin();
        let v_to = s.topo.half_edge(he.twin()).unwrap().origin();
        let p_from = s.vertex_geom[v_from];
        let p_to = s.vertex_geom[v_to];
        let line = Line::through(p_from, p_to).expect("box edge has zero length");
        let len = (p_to - p_from).norm();
        let seg = CurveSegment::line(line, 0.0, len);
        s.edge_geom.insert(eid, seg);
    }

    // ---- Stage 5: attach face geometry (planes). ----
    // For each face, walk the outer loop, collect 4 vertices, compute the
    // plane normal as the cross product of the first two edge directions.
    // The half-edge loop direction encodes the face's outward orientation
    // (CCW from outside).
    let face_ids: Vec<_> = s.topo.face_ids().collect();
    for fid in face_ids {
        let face = s.topo.face(fid).unwrap();
        let outer = face.outer_loop();
        let lp = s.topo.loop_(outer).unwrap();
        let start = lp.half_edge().expect("box face must have outer loop");

        // Collect vertex positions walking the loop.
        let mut vs: Vec<Point3> = Vec::with_capacity(4);
        let mut cur = start;
        loop {
            let he = s.topo.half_edge(cur).unwrap();
            vs.push(s.vertex_geom[he.origin()]);
            cur = he.next();
            if cur == start {
                break;
            }
        }
        debug_assert_eq!(vs.len(), 4, "box face should be a quad");

        // Loop is CCW as seen from outside, so e1 × e2 is the outward normal.
        // Frame::from_x_yhint reprojects the y-hint orthogonal to x, then
        // sets z = x × y_hint — which matches the desired outward orientation.
        let e1 = vs[1] - vs[0];
        let e2 = vs[2] - vs[0];
        let frame = Frame::from_x_yhint(vs[0], e1, e2)
            .expect("degenerate quad — should not happen for a non-degenerate box");
        s.face_geom
            .insert(fid, SurfaceKind::Plane(Plane::new(frame)));
    }

    s
}

/// Construct an axis-aligned box with the given extents, with its minimum corner
/// placed at `origin` instead of the world origin.
/// `extents.x/y/z` must all be positive.
pub fn box_at(extents: Vec3, origin: Point3) -> Solid {
    use crate::geometry::{CurveKind, SurfaceKind};
    let mut s = box_(extents);
    let offset = origin - Point3::origin();
    for (_, p) in s.vertex_geom.iter_mut() {
        *p += offset;
    }
    for (_, surf) in s.face_geom.iter_mut() {
        if let SurfaceKind::Plane(plane) = surf {
            plane.frame.origin += offset;
        }
    }
    for (_, seg) in s.edge_geom.iter_mut() {
        if let CurveKind::Line(line) = &mut seg.curve {
            line.origin += offset;
        }
    }
    s
}

#[cfg(test)]
mod tests {
    use super::*;
    use kerf_topo::validate;

    #[test]
    fn unit_box_has_8_v_12_e_6_f() {
        let s = box_(Vec3::new(1.0, 1.0, 1.0));
        assert_eq!(s.vertex_count(), 8);
        assert_eq!(s.edge_count(), 12);
        assert_eq!(s.face_count(), 6);
        assert_eq!(s.shell_count(), 1);
    }

    #[test]
    fn box_passes_topology_validation() {
        let s = box_(Vec3::new(2.0, 3.0, 4.0));
        validate(&s.topo).unwrap();
    }

    #[test]
    fn box_corners_are_at_expected_positions() {
        let s = box_(Vec3::new(1.0, 2.0, 3.0));
        let positions: Vec<Point3> = s.vertex_geom.iter().map(|(_, p)| *p).collect();
        let expected = [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, 2.0, 0.0),
            Point3::new(0.0, 2.0, 0.0),
            Point3::new(0.0, 0.0, 3.0),
            Point3::new(1.0, 0.0, 3.0),
            Point3::new(1.0, 2.0, 3.0),
            Point3::new(0.0, 2.0, 3.0),
        ];
        for e in &expected {
            assert!(
                positions.iter().any(|p| (p - e).norm() < 1e-12),
                "corner {e:?} not found"
            );
        }
    }

    #[test]
    fn every_face_has_a_planar_surface() {
        let s = box_(Vec3::new(1.0, 1.0, 1.0));
        for (_, surf) in &s.face_geom {
            assert!(matches!(surf, SurfaceKind::Plane(_)));
        }
    }

    #[test]
    fn every_edge_has_a_line_segment() {
        let s = box_(Vec3::new(1.0, 1.0, 1.0));
        for (_, seg) in &s.edge_geom {
            assert!(matches!(seg.curve, crate::CurveKind::Line(_)));
        }
    }
}
