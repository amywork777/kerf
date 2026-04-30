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
        let solid_id = self.solids.insert(());
        if self.solid_id.is_none() {
            self.solid_id = Some(solid_id);
        }
        let shell_id = self.shells.insert(Shell { faces: Vec::new(), solid: solid_id });
        let face_id = self.faces.insert(Face {
            outer_loop: LoopId::null(),
            inner_loops: Vec::new(),
            shell: shell_id,
        });
        let loop_id = self.loops.insert(Loop { half_edge: None, face: face_id });
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
    pub fn mev(&mut self, loop_id: LoopId, anchor: HalfEdgeId) -> MevResult {
        let v_old = self.half_edges[anchor].origin;
        let v_new = self.vertices.insert(Vertex { outgoing: None });

        let e = self.edges.insert(Edge { half_edges: [HalfEdgeId::null(), HalfEdgeId::null()] });
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
        MevResult { edge: e, vertex: v_new, half_edges: (he_a, he_b) }
    }

    /// Bootstrap variant: mev on an empty loop (right after mvfs). Caller passes
    /// the lone vertex `v_old` sitting in the loop's face.
    pub fn mev_at_lone_vertex(&mut self, loop_id: LoopId, v_old: VertexId) -> MevResult {
        let v_new = self.vertices.insert(Vertex { outgoing: None });

        let e = self.edges.insert(Edge { half_edges: [HalfEdgeId::null(), HalfEdgeId::null()] });
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

        MevResult { edge: e, vertex: v_new, half_edges: (he_a, he_b) }
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

#[cfg(test)]
mod tests {
    use super::*;

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
}
