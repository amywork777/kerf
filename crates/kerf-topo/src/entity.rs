//! Half-edge entities. Fields are `pub(crate)` — mutation only via Euler operators.

use crate::id::{EdgeId, FaceId, HalfEdgeId, LoopId, ShellId, SolidId, VertexId};
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Vertex {
    /// Some incident half-edge whose origin is this vertex. None on a freshly
    /// mvfs'd vertex with no edges yet.
    pub(crate) outgoing: Option<HalfEdgeId>,
}

impl Vertex {
    pub fn outgoing(&self) -> Option<HalfEdgeId> {
        self.outgoing
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct HalfEdge {
    pub(crate) origin: VertexId,
    pub(crate) twin: HalfEdgeId,
    pub(crate) next: HalfEdgeId,
    pub(crate) prev: HalfEdgeId,
    pub(crate) loop_: LoopId,
    // Read by the validator (T6); not yet read by T4 operators.
    #[allow(dead_code)]
    pub(crate) edge: EdgeId,
}

impl HalfEdge {
    pub fn origin(&self) -> VertexId {
        self.origin
    }
    pub fn twin(&self) -> HalfEdgeId {
        self.twin
    }
    pub fn next(&self) -> HalfEdgeId {
        self.next
    }
    pub fn prev(&self) -> HalfEdgeId {
        self.prev
    }
    pub fn loop_(&self) -> LoopId {
        self.loop_
    }
    pub fn edge(&self) -> EdgeId {
        self.edge
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Edge {
    pub(crate) half_edges: [HalfEdgeId; 2],
}

impl Edge {
    pub fn half_edges(&self) -> [HalfEdgeId; 2] {
        self.half_edges
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Loop {
    /// Some half-edge in this loop. None for an empty loop (e.g., right after mvfs).
    pub(crate) half_edge: Option<HalfEdgeId>,
    pub(crate) face: FaceId,
}

impl Loop {
    pub fn half_edge(&self) -> Option<HalfEdgeId> {
        self.half_edge
    }
    pub fn face(&self) -> FaceId {
        self.face
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Face {
    pub(crate) outer_loop: LoopId,
    // Used by kemr/mfkrh (T5) and validator (T6).
    #[allow(dead_code)]
    pub(crate) inner_loops: Vec<LoopId>,
    pub(crate) shell: ShellId,
}

impl Face {
    pub fn outer_loop(&self) -> LoopId {
        self.outer_loop
    }
    pub fn inner_loops(&self) -> &[LoopId] {
        &self.inner_loops
    }
    pub fn shell(&self) -> ShellId {
        self.shell
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Shell {
    pub(crate) faces: Vec<FaceId>,
    // Used by kemr/mfkrh (T5).
    #[allow(dead_code)]
    pub(crate) solid: SolidId,
}

impl Shell {
    pub fn faces(&self) -> &[FaceId] {
        &self.faces
    }
}
