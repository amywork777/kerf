//! Half-edge entities. Fields are `pub(crate)` — mutation only via Euler operators.

use crate::id::{EdgeId, FaceId, HalfEdgeId, LoopId, ShellId, SolidId, VertexId};

#[derive(Clone, Debug)]
pub struct Vertex {
    /// Some incident half-edge whose origin is this vertex. None on a freshly
    /// mvfs'd vertex with no edges yet.
    pub(crate) outgoing: Option<HalfEdgeId>,
}

#[derive(Clone, Debug)]
pub struct HalfEdge {
    pub(crate) origin: VertexId,
    pub(crate) twin: HalfEdgeId,
    pub(crate) next: HalfEdgeId,
    pub(crate) prev: HalfEdgeId,
    pub(crate) loop_: LoopId,
    pub(crate) edge: EdgeId,
}

#[derive(Clone, Debug)]
pub struct Edge {
    pub(crate) half_edges: [HalfEdgeId; 2],
}

#[derive(Clone, Debug)]
pub struct Loop {
    /// Some half-edge in this loop. None for an empty loop (e.g., right after mvfs).
    pub(crate) half_edge: Option<HalfEdgeId>,
    pub(crate) face: FaceId,
}

#[derive(Clone, Debug)]
pub struct Face {
    pub(crate) outer_loop: LoopId,
    pub(crate) inner_loops: Vec<LoopId>,
    pub(crate) shell: ShellId,
}

#[derive(Clone, Debug)]
pub struct Shell {
    pub(crate) faces: Vec<FaceId>,
    pub(crate) solid: SolidId,
}
