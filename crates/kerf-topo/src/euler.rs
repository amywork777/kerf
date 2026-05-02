//! Mäntylä Euler operators. Each operator preserves V - E + F - R = 2(S - G).

use slotmap::Key;

use crate::entity::{Edge, Face, HalfEdge, Loop, Shell, Vertex};
use crate::id::{EdgeId, FaceId, HalfEdgeId, LoopId, ShellId, SolidId, VertexId};
use crate::solid::Solid;

#[derive(Clone, Copy, Debug)]
pub struct MvfsResult {
    pub solid: SolidId,
    pub shell: ShellId,
    pub face: FaceId,
    pub loop_: LoopId,
    pub vertex: VertexId,
}

impl Solid {
    /// Make Vertex-Face-Solid: bootstrap a minimal solid (1 vertex, 1 empty-loop
    /// face, 1 shell, 1 solid). Euler: V=1, E=0, F=1, R=0, S=1 → 1-0+1-0 = 2 = 2*(1-0). ✓
    pub fn mvfs(&mut self) -> MvfsResult {
        let solid_id = self.solids.insert(true);
        if self.solid_id.is_none() {
            self.solid_id = Some(solid_id);
        }
        let shell_id = self.shells.insert(Shell {
            faces: Vec::new(),
            solid: solid_id,
        });
        let face_id = self.faces.insert(Face {
            outer_loop: LoopId::null(),
            inner_loops: Vec::new(),
            shell: shell_id,
        });
        let loop_id = self.loops.insert(Loop {
            half_edge: None,
            face: face_id,
        });
        self.faces[face_id].outer_loop = loop_id;
        self.shells[shell_id].faces.push(face_id);
        let vertex_id = self.vertices.insert(Vertex { outgoing: None });

        MvfsResult {
            solid: solid_id,
            shell: shell_id,
            face: face_id,
            loop_: loop_id,
            vertex: vertex_id,
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct MevResult {
    pub edge: EdgeId,
    pub vertex: VertexId,
    pub half_edges: (HalfEdgeId, HalfEdgeId),
}

impl Solid {
    /// Make Edge-Vertex on a non-empty loop: extend the loop by inserting a
    /// "sticker" edge (the new edge plus its tip vertex) immediately AFTER
    /// `anchor` in the loop traversal. The new edge's twin pair becomes a
    /// degenerate 2-cycle inside the existing loop.
    ///
    /// `anchor` is the half-edge ENDING AT the growth vertex (its destination
    /// is `v_old`, the vertex from which the new sticker edge sprouts). This
    /// is required to preserve the manifold invariant `h.next.origin == h.twin.origin`.
    pub fn mev(&mut self, loop_id: LoopId, anchor: HalfEdgeId) -> MevResult {
        let v_old = self.half_edges[self.half_edges[anchor].twin].origin;
        let v_new = self.vertices.insert(Vertex { outgoing: None });

        let e = self.edges.insert(Edge {
            half_edges: [HalfEdgeId::null(), HalfEdgeId::null()],
        });
        let he_a = self.half_edges.insert_with_key(|hid| HalfEdge {
            origin: v_old,
            twin: HalfEdgeId::null(),
            next: hid,
            prev: hid,
            loop_: loop_id,
            edge: e,
        });
        let he_b = self.half_edges.insert_with_key(|hid| HalfEdge {
            origin: v_new,
            twin: he_a,
            next: hid,
            prev: hid,
            loop_: loop_id,
            edge: e,
        });
        self.half_edges[he_a].twin = he_b;
        self.edges[e].half_edges = [he_a, he_b];

        // Splice he_a, he_b into the loop after `anchor`:
        //   anchor -> he_a -> he_b -> after
        let after = self.half_edges[anchor].next;
        self.half_edges[anchor].next = he_a;
        self.half_edges[he_a].prev = anchor;
        self.half_edges[he_a].next = he_b;
        self.half_edges[he_b].prev = he_a;
        self.half_edges[he_b].next = after;
        self.half_edges[after].prev = he_b;

        self.vertices[v_new].outgoing = Some(he_b);
        if self.vertices[v_old].outgoing.is_none() {
            self.vertices[v_old].outgoing = Some(he_a);
        }
        MevResult {
            edge: e,
            vertex: v_new,
            half_edges: (he_a, he_b),
        }
    }

    /// Bootstrap variant: mev on an empty loop (right after mvfs). Caller passes
    /// the lone vertex `v_old` sitting in the loop's face.
    pub fn mev_at_lone_vertex(&mut self, loop_id: LoopId, v_old: VertexId) -> MevResult {
        let v_new = self.vertices.insert(Vertex { outgoing: None });

        let e = self.edges.insert(Edge {
            half_edges: [HalfEdgeId::null(), HalfEdgeId::null()],
        });
        let he_a = self.half_edges.insert_with_key(|hid| HalfEdge {
            origin: v_old,
            twin: HalfEdgeId::null(),
            next: hid,
            prev: hid,
            loop_: loop_id,
            edge: e,
        });
        let he_b = self.half_edges.insert_with_key(|_hid| HalfEdge {
            origin: v_new,
            twin: he_a,
            next: he_a,
            prev: he_a,
            loop_: loop_id,
            edge: e,
        });
        // Patch he_a's twin and form the cyclic 2-list: he_a <-> he_b <-> he_a.
        self.half_edges[he_a].twin = he_b;
        self.half_edges[he_a].next = he_b;
        self.half_edges[he_a].prev = he_b;
        self.edges[e].half_edges = [he_a, he_b];

        self.loops[loop_id].half_edge = Some(he_a);
        self.vertices[v_old].outgoing = Some(he_a);
        self.vertices[v_new].outgoing = Some(he_b);

        MevResult {
            edge: e,
            vertex: v_new,
            half_edges: (he_a, he_b),
        }
    }

    /// Kill Edge-Vertex: inverse of `mev` for a degenerate sticker edge. Removes
    /// the edge, both half-edges, and the tip vertex.
    pub fn kev(&mut self, edge: EdgeId) {
        let [he_a, he_b] = self.edges[edge].half_edges;
        let v_tip = self.half_edges[he_b].origin;
        let v_root = self.half_edges[he_a].origin;
        let loop_id = self.half_edges[he_a].loop_;

        let before = self.half_edges[he_a].prev;
        let after = self.half_edges[he_b].next;
        if before == he_b && after == he_a {
            // The sticker was the entire loop.
            self.loops[loop_id].half_edge = None;
        } else {
            self.half_edges[before].next = after;
            self.half_edges[after].prev = before;
            if self.loops[loop_id].half_edge == Some(he_a)
                || self.loops[loop_id].half_edge == Some(he_b)
            {
                self.loops[loop_id].half_edge = Some(before);
            }
        }
        self.half_edges.remove(he_a);
        self.half_edges.remove(he_b);
        self.edges.remove(edge);
        self.vertices.remove(v_tip);
        if self.vertices[v_root].outgoing == Some(he_a) {
            let any = self
                .half_edges
                .iter()
                .find(|(_, he)| he.origin == v_root)
                .map(|(id, _)| id);
            self.vertices[v_root].outgoing = any;
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct MefResult {
    pub edge: EdgeId,
    pub face: FaceId,
    pub loop_: LoopId,
    pub half_edges: (HalfEdgeId, HalfEdgeId),
}

impl Solid {
    /// Make Edge-Face: connect two half-edges `h1` and `h2` in the same loop with
    /// a new edge; split the loop into two, creating a new face. Both half-edges
    /// must currently be in the same loop and have different origins.
    pub fn mef(&mut self, h1: HalfEdgeId, h2: HalfEdgeId) -> MefResult {
        let v1 = self.half_edges[h1].origin;
        let v2 = self.half_edges[h2].origin;
        debug_assert_ne!(v1, v2, "mef requires distinct origins");
        let l_old = self.half_edges[h1].loop_;
        debug_assert_eq!(l_old, self.half_edges[h2].loop_, "mef requires same loop");

        let e_new = self.edges.insert(Edge {
            half_edges: [HalfEdgeId::null(), HalfEdgeId::null()],
        });
        let he_a = self.half_edges.insert_with_key(|hid| HalfEdge {
            origin: v1,
            twin: HalfEdgeId::null(),
            next: hid,
            prev: hid,
            loop_: l_old,
            edge: e_new,
        });
        let he_b = self.half_edges.insert_with_key(|hid| HalfEdge {
            origin: v2,
            twin: he_a,
            next: hid,
            prev: hid,
            loop_: l_old,
            edge: e_new,
        });
        self.half_edges[he_a].twin = he_b;
        self.edges[e_new].half_edges = [he_a, he_b];

        // Splice. he_a: h1.prev -> he_a -> h2.  he_b: h2.prev -> he_b -> h1.
        let h1_prev = self.half_edges[h1].prev;
        let h2_prev = self.half_edges[h2].prev;

        self.half_edges[h1_prev].next = he_a;
        self.half_edges[he_a].prev = h1_prev;
        self.half_edges[he_a].next = h2;
        self.half_edges[h2].prev = he_a;

        self.half_edges[h2_prev].next = he_b;
        self.half_edges[he_b].prev = h2_prev;
        self.half_edges[he_b].next = h1;
        self.half_edges[h1].prev = he_b;

        // New loop + new face.
        let f_old = self.loops[l_old].face;
        let shell_id = self.faces[f_old].shell;

        let l_new = self.loops.insert(Loop {
            half_edge: Some(he_b),
            face: FaceId::null(),
        });
        let f_new = self.faces.insert(Face {
            outer_loop: l_new,
            inner_loops: Vec::new(),
            shell: shell_id,
        });
        self.loops[l_new].face = f_new;
        self.shells[shell_id].faces.push(f_new);

        // Re-tag the new-loop half via walking from he_b.
        let mut cur = he_b;
        loop {
            self.half_edges[cur].loop_ = l_new;
            cur = self.half_edges[cur].next;
            if cur == he_b {
                break;
            }
        }

        // Update old loop's half_edge to point at he_a (which is now in old loop).
        self.loops[l_old].half_edge = Some(he_a);

        MefResult {
            edge: e_new,
            face: f_new,
            loop_: l_new,
            half_edges: (he_a, he_b),
        }
    }

    /// Kill Edge-Face: inverse of mef. Removes the new edge and merges the
    /// two loops + faces back into one. `edge` must be a non-bridge edge whose
    /// two half-edges are in different loops of different faces in the same shell.
    pub fn kef(&mut self, edge: EdgeId) {
        let [he_a, he_b] = self.edges[edge].half_edges;
        let l_a = self.half_edges[he_a].loop_;
        let l_b = self.half_edges[he_b].loop_;
        debug_assert_ne!(
            l_a, l_b,
            "kef requires the two half-edges to be in different loops"
        );

        let f_b = self.loops[l_b].face;
        let f_a = self.loops[l_a].face;
        let shell = self.faces[f_a].shell;

        // Re-tag the half-edges of l_b to belong to l_a.
        if let Some(start) = self.loops[l_b].half_edge {
            let mut cur = start;
            loop {
                if cur == he_b {
                    cur = self.half_edges[cur].next;
                    continue;
                }
                self.half_edges[cur].loop_ = l_a;
                cur = self.half_edges[cur].next;
                if cur == start {
                    break;
                }
            }
        }

        // Splice he_a, he_b out of the loops.
        let he_a_prev = self.half_edges[he_a].prev;
        let he_a_next = self.half_edges[he_a].next;
        let he_b_prev = self.half_edges[he_b].prev;
        let he_b_next = self.half_edges[he_b].next;

        self.half_edges[he_a_prev].next = he_b_next;
        self.half_edges[he_b_next].prev = he_a_prev;
        self.half_edges[he_b_prev].next = he_a_next;
        self.half_edges[he_a_next].prev = he_b_prev;

        // l_a's half_edge — point it at the survivor.
        self.loops[l_a].half_edge = Some(he_a_prev);

        // Remove face_b, loop_b, the half-edges, the edge.
        self.shells[shell].faces.retain(|&f| f != f_b);
        self.faces.remove(f_b);
        self.loops.remove(l_b);
        self.half_edges.remove(he_a);
        self.half_edges.remove(he_b);
        self.edges.remove(edge);
    }
}

#[derive(Clone, Copy, Debug)]
pub struct SplitEdgeResult {
    pub new_edge: EdgeId,
    pub new_vertex: VertexId,
}

impl Solid {
    /// Split an edge in two by inserting a new vertex in its middle.
    /// V+1, E+1, F+0; preserves the Euler invariant. Manifold-preserving.
    pub fn split_edge(&mut self, edge: EdgeId) -> SplitEdgeResult {
        let [he_a, he_b] = self.edges[edge].half_edges;
        let next_a = self.half_edges[he_a].next;
        let next_b = self.half_edges[he_b].next;
        let loop_a = self.half_edges[he_a].loop_;
        let loop_b = self.half_edges[he_b].loop_;

        let v_new = self.vertices.insert(Vertex { outgoing: None });

        // Allocate the new edge first as a placeholder so the new half-edges can reference it.
        let new_edge = self.edges.insert(Edge {
            half_edges: [HalfEdgeId::null(), HalfEdgeId::null()],
        });

        // new_he_a2: origin v_new, twin he_b, splice after he_a in loop_a.
        let new_he_a2 = self.half_edges.insert(HalfEdge {
            origin: v_new,
            twin: he_b,
            next: next_a,
            prev: he_a,
            loop_: loop_a,
            edge: new_edge,
        });
        // new_he_b2: origin v_new, twin he_a, splice after he_b in loop_b.
        let new_he_b2 = self.half_edges.insert(HalfEdge {
            origin: v_new,
            twin: he_a,
            next: next_b,
            prev: he_b,
            loop_: loop_b,
            edge,
        });

        // Wire pointers.
        self.half_edges[he_a].next = new_he_a2;
        self.half_edges[he_a].twin = new_he_b2;
        self.half_edges[next_a].prev = new_he_a2;

        self.half_edges[he_b].next = new_he_b2;
        self.half_edges[he_b].twin = new_he_a2;
        self.half_edges[next_b].prev = new_he_b2;
        // he_b's edge changes from `edge` to `new_edge`.
        self.half_edges[he_b].edge = new_edge;

        // Update edge endpoints.
        self.edges[edge].half_edges = [he_a, new_he_b2];
        self.edges[new_edge].half_edges = [new_he_a2, he_b];

        // v_new's outgoing.
        self.vertices[v_new].outgoing = Some(new_he_a2);

        SplitEdgeResult {
            new_edge,
            new_vertex: v_new,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::validate::validate;

    #[test]
    fn mvfs_creates_one_of_each_top_level_entity() {
        let mut s = Solid::new();
        let r = s.mvfs();
        assert_eq!(s.vertex_count(), 1);
        assert_eq!(s.edge_count(), 0);
        assert_eq!(s.face_count(), 1);
        assert_eq!(s.shell_count(), 1);
        assert_eq!(s.face(r.face).unwrap().outer_loop, r.loop_);
        assert_eq!(s.face(r.face).unwrap().shell, r.shell);
        assert_eq!(s.shell(r.shell).unwrap().faces, vec![r.face]);
        assert_eq!(s.loop_(r.loop_).unwrap().half_edge, None);
        assert_eq!(s.vertex(r.vertex).unwrap().outgoing, None);
    }

    #[test]
    fn euler_invariant_holds_after_mvfs() {
        let mut s = Solid::new();
        s.mvfs();
        let v = s.vertex_count() as i64;
        let e = s.edge_count() as i64;
        let f = s.face_count() as i64;
        let s_count = s.shell_count() as i64;
        assert_eq!(v - e + f, 2 * s_count);
    }

    #[test]
    fn mev_at_lone_vertex_after_mvfs_grows_v_and_e() {
        let mut s = Solid::new();
        let r = s.mvfs();
        let _m = s.mev_at_lone_vertex(r.loop_, r.vertex);
        assert_eq!(s.vertex_count(), 2);
        assert_eq!(s.edge_count(), 1);
        assert_eq!(s.face_count(), 1);
        assert_eq!(s.shell_count(), 1);
        let euler = s.vertex_count() as i64 - s.edge_count() as i64 + s.face_count() as i64;
        assert_eq!(euler, 2);
    }

    #[test]
    fn mev_then_kev_round_trips() {
        let mut s = Solid::new();
        let r = s.mvfs();
        let m = s.mev_at_lone_vertex(r.loop_, r.vertex);
        s.kev(m.edge);
        assert_eq!(s.vertex_count(), 1);
        assert_eq!(s.edge_count(), 0);
        assert_eq!(s.face_count(), 1);
        assert_eq!(s.shell_count(), 1);
    }

    #[test]
    fn mef_splits_loop_and_creates_new_face() {
        // Build a closed quad loop using 4x mev plus 1x mef.
        // Easier: build a triangle by mev->mev->mev (sticker chain) then mef across two non-adjacent half-edges.
        // For a clean test: mvfs, mev_at_lone_vertex, then mev twice (forming 3 sticker edges from one vertex),
        // then mef from the first sticker's outgoing to the second's outgoing.
        // The exact half-edge selection takes care.
        // Minimal test: just verify F count goes from 1 to 2 and loop count from 1 to 2.
        let mut s = Solid::new();
        let r = s.mvfs();
        let m1 = s.mev_at_lone_vertex(r.loop_, r.vertex);
        // mev anchor is the half-edge ending at the growth vertex; he_a (m1.half_edges.0)
        // has dest = v1 (the latest tip), so we anchor on it.
        let m2 = s.mev(r.loop_, m1.half_edges.0);
        // After m1, m2: we have 3 vertices, 2 edges (each a sticker), 1 face, 1 loop with 4 half-edges total.
        // Let's pick two distinct half-edges in this loop with distinct origins.
        let h_a = m1.half_edges.0; // origin = root vertex (r.vertex)
        let h_b = m2.half_edges.1; // origin = far tip of m2 sticker
        // mef across these two.
        let mef_r = s.mef(h_a, h_b);
        assert_eq!(s.face_count(), 2);
        assert_eq!(s.loop_count(), 2);
        assert_eq!(s.edge_count(), 3);
        validate(&s).unwrap();
        let _ = mef_r;
    }

    #[test]
    fn mef_then_kef_round_trips_face_count() {
        let mut s = Solid::new();
        let r = s.mvfs();
        let m1 = s.mev_at_lone_vertex(r.loop_, r.vertex);
        let m2 = s.mev(r.loop_, m1.half_edges.0);
        let h_a = m1.half_edges.0;
        let h_b = m2.half_edges.1;
        let mef_r = s.mef(h_a, h_b);
        assert_eq!(s.face_count(), 2);
        s.kef(mef_r.edge);
        assert_eq!(s.face_count(), 1);
        assert_eq!(s.edge_count(), 2);
    }

    #[test]
    fn split_edge_creates_new_v_and_e_preserving_invariant() {
        let mut s = Solid::new();
        let r = s.mvfs();
        let m1 = s.mev_at_lone_vertex(r.loop_, r.vertex);
        // s now: 2V, 1E, 1F, 1S; outer loop has the sticker.
        crate::validate(&s).unwrap();
        let split = s.split_edge(m1.edge);
        // After split: 3V, 2E, 1F, 1S.
        assert_eq!(s.vertex_count(), 3);
        assert_eq!(s.edge_count(), 2);
        assert_eq!(s.face_count(), 1);
        crate::validate(&s).unwrap();
        let _ = split;
    }

    #[test]
    fn split_edge_in_box_topology_passes_validation() {
        // Build a box-like topology then split one of its edges. Use the test from m4
        // as a starting topology builder.
        let mut s = Solid::new();
        let r = s.mvfs();
        let m1 = s.mev_at_lone_vertex(r.loop_, r.vertex);
        let m2 = s.mev(r.loop_, m1.half_edges.0);
        let h_a = m1.half_edges.0;
        let h_b = m2.half_edges.1; // origin = m2.vertex
        let _mef_r = s.mef(h_a, h_b);
        crate::validate(&s).unwrap();
        let _split = s.split_edge(m1.edge);
        crate::validate(&s).unwrap();
        assert_eq!(s.vertex_count(), 4);
        assert_eq!(s.edge_count(), 4);
        assert_eq!(s.face_count(), 2);
    }
}
