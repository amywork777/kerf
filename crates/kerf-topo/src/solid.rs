//! `Solid`: container for one connected B-rep solid.

use slotmap::SlotMap;

use crate::entity::{Edge, Face, HalfEdge, Loop, Shell, Vertex};
use crate::id::{EdgeId, FaceId, HalfEdgeId, LoopId, ShellId, SolidId, VertexId};
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct Solid {
    pub(crate) vertices: SlotMap<VertexId, Vertex>,
    pub(crate) half_edges: SlotMap<HalfEdgeId, HalfEdge>,
    pub(crate) edges: SlotMap<EdgeId, Edge>,
    pub(crate) loops: SlotMap<LoopId, Loop>,
    pub(crate) faces: SlotMap<FaceId, Face>,
    pub(crate) shells: SlotMap<ShellId, Shell>,
    pub(crate) solid_id: Option<SolidId>,
    pub(crate) solids: SlotMap<SolidId, bool>,
}

impl Solid {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn vertex(&self, id: VertexId) -> Option<&Vertex> {
        self.vertices.get(id)
    }
    pub fn half_edge(&self, id: HalfEdgeId) -> Option<&HalfEdge> {
        self.half_edges.get(id)
    }
    pub fn edge(&self, id: EdgeId) -> Option<&Edge> {
        self.edges.get(id)
    }
    pub fn loop_(&self, id: LoopId) -> Option<&Loop> {
        self.loops.get(id)
    }
    pub fn face(&self, id: FaceId) -> Option<&Face> {
        self.faces.get(id)
    }
    pub fn shell(&self, id: ShellId) -> Option<&Shell> {
        self.shells.get(id)
    }

    pub fn vertex_count(&self) -> usize {
        self.vertices.len()
    }
    pub fn edge_count(&self) -> usize {
        self.edges.len()
    }
    pub fn face_count(&self) -> usize {
        self.faces.len()
    }
    pub fn shell_count(&self) -> usize {
        self.shells.len()
    }
    pub fn loop_count(&self) -> usize {
        self.loops.len()
    }

    /// Walk a loop's half-edges via `next`, terminating when we return to start.
    pub fn iter_loop_half_edges(&self, start: HalfEdgeId) -> LoopWalker<'_> {
        LoopWalker {
            solid: self,
            start,
            current: Some(start),
        }
    }

    /// Iterate over all vertex IDs.
    pub fn vertex_ids(&self) -> impl Iterator<Item = VertexId> + '_ {
        self.vertices.iter().map(|(id, _)| id)
    }

    /// Iterate over all edge IDs.
    pub fn edge_ids(&self) -> impl Iterator<Item = EdgeId> + '_ {
        self.edges.iter().map(|(id, _)| id)
    }

    /// Iterate over all face IDs.
    pub fn face_ids(&self) -> impl Iterator<Item = FaceId> + '_ {
        self.faces.iter().map(|(id, _)| id)
    }

    /// Iterate over all shell IDs.
    pub fn shell_ids(&self) -> impl Iterator<Item = crate::id::ShellId> + '_ {
        self.shells.iter().map(|(id, _)| id)
    }

    /// Iterate over (edge_id, &Edge) pairs.
    pub fn edges_iter(&self) -> impl Iterator<Item = (EdgeId, &crate::entity::Edge)> + '_ {
        self.edges.iter()
    }
}

pub struct LoopWalker<'a> {
    solid: &'a Solid,
    start: HalfEdgeId,
    current: Option<HalfEdgeId>,
}

impl Iterator for LoopWalker<'_> {
    type Item = HalfEdgeId;
    fn next(&mut self) -> Option<HalfEdgeId> {
        let cur = self.current?;
        let nxt = self.solid.half_edges[cur].next;
        self.current = if nxt == self.start { None } else { Some(nxt) };
        Some(cur)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn empty_solid_has_no_entities() {
        let s = Solid::new();
        assert_eq!(s.vertex_count(), 0);
        assert_eq!(s.edge_count(), 0);
        assert_eq!(s.face_count(), 0);
        assert_eq!(s.shell_count(), 0);
    }
}
