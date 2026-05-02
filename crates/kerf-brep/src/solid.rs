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

    /// Non-panicking variant of [`Self::union`]. Returns `Err` if the boolean
    /// pipeline hits an unsupported configuration (e.g. interior endpoints
    /// without a boundary anchor, non-manifold stitch input, etc.) instead of
    /// crashing the caller.
    pub fn try_union(&self, other: &Solid) -> Result<Solid, BooleanError> {
        try_boolean_solid(self, other, crate::booleans::BooleanOp::Union)
    }

    /// Non-panicking variant of [`Self::intersection`].
    pub fn try_intersection(&self, other: &Solid) -> Result<Solid, BooleanError> {
        try_boolean_solid(self, other, crate::booleans::BooleanOp::Intersection)
    }

    /// Non-panicking variant of [`Self::difference`].
    pub fn try_difference(&self, other: &Solid) -> Result<Solid, BooleanError> {
        try_boolean_solid(self, other, crate::booleans::BooleanOp::Difference)
    }
}

/// Error returned by the non-panicking boolean variants. Carries the panic
/// payload as a string so callers can log a meaningful diagnostic without
/// caring about which specific invariant tripped.
#[derive(Debug)]
pub struct BooleanError {
    pub message: String,
}

impl std::fmt::Display for BooleanError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "boolean failed: {}", self.message)
    }
}

impl std::error::Error for BooleanError {}

/// Run `boolean_solid` inside `catch_unwind` and convert any panic into a
/// `BooleanError`. The kernel's internal panics are invariant violations from
/// the boolean pipeline (e.g. M11 phase B interior-endpoint limits) — none
/// corrupt heap state, so unwinding is safe.
pub fn try_boolean_solid(
    a: &Solid,
    b: &Solid,
    op: crate::booleans::BooleanOp,
) -> Result<Solid, BooleanError> {
    let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        crate::booleans::boolean_solid(a, b, op, &kerf_geom::Tolerance::default())
    }));
    result.map_err(|payload| {
        let message = if let Some(s) = payload.downcast_ref::<String>() {
            s.clone()
        } else if let Some(s) = payload.downcast_ref::<&'static str>() {
            (*s).to_string()
        } else {
            "unknown panic in boolean pipeline".to_string()
        };
        BooleanError { message }
    })
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
    use crate::primitives::{box_, box_at, cylinder_faceted};
    use kerf_geom::{Point3, Vec3};
    use kerf_topo::validate;

    #[test]
    fn try_union_returns_err_for_unsupported_curved_case() {
        // Cylinder piercing through a box face hits the M11 phase-B interior
        // endpoint limit. With panic-based booleans this would crash; with
        // try_union we get a structured error.
        let block = box_at(Vec3::new(2.0, 2.0, 2.0), Point3::new(-1.0, -1.0, -1.0));
        let cyl = cylinder_faceted(0.6, 3.0, 12);
        let r = block.try_union(&cyl);
        assert!(r.is_err(), "expected curved-piercing union to fail cleanly");
        let err = r.unwrap_err();
        assert!(
            err.message.contains("interior endpoint"),
            "panic message should mention interior endpoint: {}",
            err.message
        );
    }

    #[test]
    fn try_union_returns_ok_for_supported_case() {
        let a = box_(Vec3::new(2.0, 2.0, 2.0));
        let b = box_at(Vec3::new(2.0, 2.0, 2.0), Point3::new(1.0, 0.0, 0.0));
        let r = a.try_union(&b);
        assert!(r.is_ok(), "supported case should not error");
    }

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
