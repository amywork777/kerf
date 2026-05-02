//! Direct slotmap construction API for callers that build a `Solid` from a
//! known target topology (e.g., the M8 boolean stitcher).
//!
//! Most callers should use the Euler operators in `crate::euler`. This module
//! is the escape hatch for building a solid in one shot when the full topology
//! is known up-front and applying Euler operators in reverse would be more
//! tedious than direct construction. Validation via [`crate::validate`] is the
//! safety net.

use slotmap::Key;

use crate::entity::{Edge, Face, HalfEdge, Loop, Shell, Vertex};
use crate::id::{EdgeId, FaceId, HalfEdgeId, LoopId, ShellId, SolidId, VertexId};
use crate::solid::Solid;

impl Solid {
    /// Insert a fresh vertex with no outgoing half-edge.
    pub fn build_insert_vertex(&mut self) -> VertexId {
        self.vertices.insert(Vertex { outgoing: None })
    }

    /// Set a vertex's outgoing half-edge.
    pub fn build_set_vertex_outgoing(&mut self, v: VertexId, he: Option<HalfEdgeId>) {
        self.vertices[v].outgoing = he;
    }

    /// Insert a fresh solid id; returns the id. The caller decides whether to
    /// also assign it to `Solid.solid_id`.
    pub fn build_insert_solid(&mut self) -> SolidId {
        self.solids.insert(true)
    }

    /// Set the active solid id (one per `Solid` container).
    pub fn build_set_active_solid(&mut self, id: Option<SolidId>) {
        self.solid_id = id;
    }

    /// Insert an empty shell attached to a solid.
    pub fn build_insert_shell(&mut self, solid: SolidId) -> ShellId {
        self.shells.insert(Shell {
            faces: Vec::new(),
            solid,
        })
    }

    /// Append a face to a shell's face list.
    pub fn build_push_shell_face(&mut self, shell: ShellId, face: FaceId) {
        self.shells[shell].faces.push(face);
    }

    /// Patch a face's shell pointer. Use when the shell assignment is only
    /// known after connected-component analysis (e.g. multi-shell stitch).
    pub fn build_set_face_shell(&mut self, face: FaceId, shell: ShellId) {
        self.faces[face].shell = shell;
    }

    /// Insert a placeholder loop (no half-edge, face id is `default()`/null).
    /// Caller patches the face afterward via [`Self::build_set_loop_face`].
    pub fn build_insert_loop_placeholder(&mut self) -> LoopId {
        self.loops.insert(Loop {
            half_edge: None,
            face: FaceId::null(),
        })
    }

    /// Patch the face id on a loop.
    pub fn build_set_loop_face(&mut self, lp: LoopId, face: FaceId) {
        self.loops[lp].face = face;
    }

    /// Set a loop's representative half-edge.
    pub fn build_set_loop_half_edge(&mut self, lp: LoopId, he: Option<HalfEdgeId>) {
        self.loops[lp].half_edge = he;
    }

    /// Insert a face with the given outer loop and shell.
    pub fn build_insert_face(&mut self, outer_loop: LoopId, shell: ShellId) -> FaceId {
        self.faces.insert(Face {
            outer_loop,
            inner_loops: Vec::new(),
            shell,
        })
    }

    /// Insert a half-edge whose `twin`, `next`, `prev`, `edge` are placeholders.
    /// Caller patches them via [`Self::build_set_half_edge_twin`],
    /// [`Self::build_set_half_edge_next_prev`], and [`Self::build_set_half_edge_edge`].
    pub fn build_insert_half_edge(
        &mut self,
        origin: VertexId,
        loop_: LoopId,
    ) -> HalfEdgeId {
        self.half_edges.insert(HalfEdge {
            origin,
            twin: HalfEdgeId::null(),
            next: HalfEdgeId::null(),
            prev: HalfEdgeId::null(),
            loop_,
            edge: EdgeId::null(),
        })
    }

    /// Patch a half-edge's twin pointer.
    pub fn build_set_half_edge_twin(&mut self, he: HalfEdgeId, twin: HalfEdgeId) {
        self.half_edges[he].twin = twin;
    }

    /// Patch a half-edge's next pointer (and the next's prev to point back).
    pub fn build_set_half_edge_next_prev(&mut self, cur: HalfEdgeId, next: HalfEdgeId) {
        self.half_edges[cur].next = next;
        self.half_edges[next].prev = cur;
    }

    /// Patch a half-edge's edge id.
    pub fn build_set_half_edge_edge(&mut self, he: HalfEdgeId, edge: EdgeId) {
        self.half_edges[he].edge = edge;
    }

    /// Insert an edge with two half-edges already known.
    pub fn build_insert_edge(&mut self, half_edges: [HalfEdgeId; 2]) -> EdgeId {
        self.edges.insert(Edge { half_edges })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::validate::validate;

    #[test]
    fn build_a_minimal_two_face_sliver_validates() {
        // Two triangle faces sharing all three edges (a degenerate "lens"-like
        // closed surface): two faces, three edges, three vertices. Topologically
        // a sphere (V=3, E=3, F=2 → 3-3+2 = 2 = 2*1). This exercises every
        // builder hook.
        let mut s = Solid::new();
        let v0 = s.build_insert_vertex();
        let v1 = s.build_insert_vertex();
        let v2 = s.build_insert_vertex();
        let solid = s.build_insert_solid();
        s.build_set_active_solid(Some(solid));
        let shell = s.build_insert_shell(solid);

        // Face A: v0 -> v1 -> v2.
        let lp_a = s.build_insert_loop_placeholder();
        let face_a = s.build_insert_face(lp_a, shell);
        s.build_set_loop_face(lp_a, face_a);
        s.build_push_shell_face(shell, face_a);
        let a0 = s.build_insert_half_edge(v0, lp_a);
        let a1 = s.build_insert_half_edge(v1, lp_a);
        let a2 = s.build_insert_half_edge(v2, lp_a);
        s.build_set_half_edge_next_prev(a0, a1);
        s.build_set_half_edge_next_prev(a1, a2);
        s.build_set_half_edge_next_prev(a2, a0);
        s.build_set_loop_half_edge(lp_a, Some(a0));

        // Face B: v0 -> v2 -> v1 (opposite winding).
        let lp_b = s.build_insert_loop_placeholder();
        let face_b = s.build_insert_face(lp_b, shell);
        s.build_set_loop_face(lp_b, face_b);
        s.build_push_shell_face(shell, face_b);
        let b0 = s.build_insert_half_edge(v0, lp_b);
        let b1 = s.build_insert_half_edge(v2, lp_b);
        let b2 = s.build_insert_half_edge(v1, lp_b);
        s.build_set_half_edge_next_prev(b0, b1);
        s.build_set_half_edge_next_prev(b1, b2);
        s.build_set_half_edge_next_prev(b2, b0);
        s.build_set_loop_half_edge(lp_b, Some(b0));

        // Edges: (v0,v1) shared by a0 and b2; (v1,v2) by a1 and b1; (v2,v0) by a2 and b0.
        let e01 = s.build_insert_edge([a0, b2]);
        s.build_set_half_edge_twin(a0, b2);
        s.build_set_half_edge_twin(b2, a0);
        s.build_set_half_edge_edge(a0, e01);
        s.build_set_half_edge_edge(b2, e01);

        let e12 = s.build_insert_edge([a1, b1]);
        s.build_set_half_edge_twin(a1, b1);
        s.build_set_half_edge_twin(b1, a1);
        s.build_set_half_edge_edge(a1, e12);
        s.build_set_half_edge_edge(b1, e12);

        let e20 = s.build_insert_edge([a2, b0]);
        s.build_set_half_edge_twin(a2, b0);
        s.build_set_half_edge_twin(b0, a2);
        s.build_set_half_edge_edge(a2, e20);
        s.build_set_half_edge_edge(b0, e20);

        s.build_set_vertex_outgoing(v0, Some(a0));
        s.build_set_vertex_outgoing(v1, Some(a1));
        s.build_set_vertex_outgoing(v2, Some(a2));

        validate(&s).unwrap();
    }
}
