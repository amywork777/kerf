//! `brep::Solid` — topology + geometry.

use slotmap::SecondaryMap;

use kerf_geom::Point3;
use kerf_topo::{EdgeId, FaceId, Solid as TopoSolid, VertexId};

use crate::geometry::{CurveSegment, SurfaceKind};

#[derive(Clone, Debug, Default)]
pub struct Solid {
    pub topo: TopoSolid,
    pub vertex_geom: SecondaryMap<VertexId, Point3>,
    pub edge_geom: SecondaryMap<EdgeId, CurveSegment>,
    pub face_geom: SecondaryMap<FaceId, SurfaceKind>,
}

impl Solid {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn vertex_count(&self) -> usize {
        self.topo.vertex_count()
    }
    pub fn edge_count(&self) -> usize {
        self.topo.edge_count()
    }
    pub fn face_count(&self) -> usize {
        self.topo.face_count()
    }
    pub fn shell_count(&self) -> usize {
        self.topo.shell_count()
    }

    pub fn point_at(&self, v: VertexId) -> Option<Point3> {
        self.vertex_geom.get(v).copied()
    }

    pub fn surface_at(&self, f: FaceId) -> Option<&SurfaceKind> {
        self.face_geom.get(f)
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
    }
}
