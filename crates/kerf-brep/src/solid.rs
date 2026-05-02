//! `brep::Solid` — topology + geometry.

use slotmap::SecondaryMap;

use kerf_geom::Point3;
use kerf_topo::{EdgeId, FaceId, Solid as TopoSolid, VertexId};

use crate::geometry::{CurveSegment, SurfaceKind};
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
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

    /// Compute the union of `self` and `other` using the default tolerance.
    pub fn union(&self, other: &Solid) -> Solid {
        crate::booleans::boolean_solid(
            self,
            other,
            crate::booleans::BooleanOp::Union,
            &kerf_geom::Tolerance::default(),
        )
    }

    /// Compute the intersection of `self` and `other` using the default tolerance.
    pub fn intersection(&self, other: &Solid) -> Solid {
        crate::booleans::boolean_solid(
            self,
            other,
            crate::booleans::BooleanOp::Intersection,
            &kerf_geom::Tolerance::default(),
        )
    }

    /// Compute the difference `self` minus `other` using the default tolerance.
    pub fn difference(&self, other: &Solid) -> Solid {
        crate::booleans::boolean_solid(
            self,
            other,
            crate::booleans::BooleanOp::Difference,
            &kerf_geom::Tolerance::default(),
        )
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

#[cfg(test)]
mod method_tests {
    use crate::primitives::{box_, box_at};
    use kerf_geom::{Point3, Vec3};
    use kerf_topo::validate;

    #[test]
    fn solid_union_method_works() {
        let a = box_(Vec3::new(10.0, 10.0, 10.0));
        let b = box_at(Vec3::new(2.0, 2.0, 2.0), Point3::new(4.0, 4.0, 4.0));
        let result = a.union(&b);
        // Nested → result is just the big box.
        assert_eq!(result.vertex_count(), 8);
        validate(&result.topo).unwrap();
    }

    #[test]
    fn solid_difference_method_works() {
        let a = box_(Vec3::new(10.0, 10.0, 10.0));
        let b = box_at(Vec3::new(2.0, 2.0, 2.0), Point3::new(4.0, 4.0, 4.0));
        let result = a.difference(&b);
        // Two disconnected shells: 16V, 24E, 12F, 2S.
        assert_eq!(result.vertex_count(), 16, "expected 16 vertices (8 outer + 8 inner)");
        assert_eq!(result.edge_count(), 24, "expected 24 edges (12 per shell)");
        assert_eq!(result.face_count(), 12, "expected 12 faces (6 per shell)");
        assert_eq!(result.shell_count(), 2, "expected 2 disconnected shells");
        validate(&result.topo).unwrap();
    }

    #[test]
    fn box_at_places_origin_correctly() {
        let s = box_at(Vec3::new(1.0, 2.0, 3.0), Point3::new(10.0, 20.0, 30.0));
        // Corner closest to (10,20,30) should be exactly that.
        let positions: Vec<Point3> = s.vertex_geom.iter().map(|(_, p)| *p).collect();
        assert!(positions.iter().any(|p| (*p - Point3::new(10.0, 20.0, 30.0)).norm() < 1e-12));
        assert!(positions.iter().any(|p| (*p - Point3::new(11.0, 22.0, 33.0)).norm() < 1e-12));
    }
}
