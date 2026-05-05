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
    /// Maps each face to its "ancestor" — the original face it descended from
    /// via splitter mef calls. Faces that haven't been split point to
    /// themselves. Used by the M38b chord-merge to restrict merges to
    /// same-original-face sibling pairs. Lookup `face_ancestor(f)` returns
    /// `f` itself if no entry exists.
    #[serde(default)]
    pub face_provenance: SecondaryMap<FaceId, FaceId>,
    /// Picking provenance: maps each face to a free-form owner tag
    /// (typically a kerf-cad Feature id like "drill" or "body"). Populated
    /// by the cad evaluator after each Feature builds its solid; propagated
    /// through booleans by `stitch` from each kept face's source KeptFace.
    /// Faces with no entry have no recorded owner.
    #[serde(default)]
    pub face_owner_tag: SecondaryMap<FaceId, String>,
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

    /// Return the original face `f` descended from. Faces that haven't been
    /// split (or were never tagged) return themselves.
    pub fn face_ancestor(&self, f: FaceId) -> FaceId {
        self.face_provenance.get(f).copied().unwrap_or(f)
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
/// `BooleanError`. Tiered retry strategy:
///
/// 1. Primary: `(a, b)` as given.
/// 2. Swap: `(b, a)` for commutative ops (Union, Intersection). The OnBoundary
///    classifier is order-dependent, so the swap sometimes succeeds where
///    the primary doesn't.
/// 3. Jitter (multiple directions): shift `b` by various ~1e-6 offsets to
///    break coplanar / coincident-vertex degeneracies. The geometric error
///    is well below visualization precision but two orders of magnitude
///    larger than the default `point_eq` tolerance, so points that were ON
///    a boundary now read as just-off.
///
/// None of the kernel's internal panics corrupt heap state, so unwinding
/// is safe.
pub fn try_boolean_solid(
    a: &Solid,
    b: &Solid,
    op: crate::booleans::BooleanOp,
) -> Result<Solid, BooleanError> {
    use crate::booleans::BooleanOp;
    // Tier 1: primary.
    if let Ok(s) = run_boolean_solid_caught(a, b, op) {
        return Ok(s);
    }
    // Tier 2: swap (commutative ops only).
    if matches!(op, BooleanOp::Union | BooleanOp::Intersection) {
        if let Ok(s) = run_boolean_solid_caught(b, a, op) {
            return Ok(s);
        }
    }
    // Tier 3: try several jitter directions. Each direction is a small
    // non-axis-aligned offset; together they break different classes of
    // coplanar/coincident degeneracies. The original 7-3-5 direction
    // covered most cases; the additional 3-7-2, -5-7-3, and 5-2-7 cover
    // configurations where the wedge-meets-curved-face stitch error
    // happens (different classifier code paths trip on different offsets).
    const JITTERS: &[(f64, f64, f64)] = &[
        (7.0e-6, 3.0e-6, 5.0e-6),
        (3.0e-6, 7.0e-6, 2.0e-6),
        (-5.0e-6, 7.0e-6, -3.0e-6),
        (5.0e-6, 2.0e-6, 7.0e-6),
        // Larger-magnitude jitter for configurations that need >1µm
        // perturbation (still well below practical precision).
        (1.7e-4, 0.9e-4, 1.3e-4),
        (-1.3e-4, 1.7e-4, 0.5e-4),
    ];
    for &(jx, jy, jz) in JITTERS {
        let b_jittered = jitter_solid(b, kerf_geom::Vec3::new(jx, jy, jz));
        if let Ok(s) = run_boolean_solid_caught(a, &b_jittered, op) {
            return Ok(s);
        }
        if matches!(op, BooleanOp::Union | BooleanOp::Intersection) {
            if let Ok(s) = run_boolean_solid_caught(&b_jittered, a, op) {
                return Ok(s);
            }
        }
    }
    // Tier 4: jitter A instead of B. Some configurations break only when
    // a's vertices are slightly off-axis — the OnBoundary classifier has
    // a different code path depending on which solid's face is being
    // classified, and jittering the OTHER solid sometimes breaks the
    // degeneracy where jittering the original solid did not.
    for &(jx, jy, jz) in JITTERS {
        let a_jittered = jitter_solid(a, kerf_geom::Vec3::new(jx, jy, jz));
        if let Ok(s) = run_boolean_solid_caught(&a_jittered, b, op) {
            return Ok(s);
        }
    }
    // Tier 5: jitter BOTH a and b in opposite directions. This breaks
    // configurations where a single-sided jitter of either solid still
    // leaves one face exactly coplanar with the other (e.g., when the
    // b face is symmetric across the jitter axis).
    for &(jx, jy, jz) in JITTERS {
        let a_jittered = jitter_solid(a, kerf_geom::Vec3::new(jx * 0.7, jy * 0.7, jz * 0.7));
        let b_jittered = jitter_solid(b, kerf_geom::Vec3::new(-jx * 0.3, -jy * 0.3, -jz * 0.3));
        if let Ok(s) = run_boolean_solid_caught(&a_jittered, &b_jittered, op) {
            return Ok(s);
        }
    }
    // All attempts failed — re-run primary to capture the original message.
    let payload = run_boolean_solid_caught(a, b, op).err().unwrap();
    Err(payload)
}

/// Shift every vertex of a solid by `offset`. Used as a degeneracy-breaking
/// retry: jittering by ~1e-9 along a non-axis-aligned direction perturbs
/// coplanar / coincident-vertex configurations enough that the OnBoundary
/// classifier no longer sees them as degenerate.
fn jitter_solid(s: &Solid, offset: kerf_geom::Vec3) -> Solid {
    let mut out = s.clone();
    for v in out.vertex_geom.values_mut() {
        *v += offset;
    }
    out
}

fn run_boolean_solid_caught(
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
    fn try_union_returns_ok_for_curved_piercing() {
        // M40: cylinder piercing a box — the chord ring case that motivated
        // most of M40's work. Previously failed with "non-manifold input to
        // stitch" or fell through to a panic-recovered Err. Now produces a
        // clean closed manifold via Phase B fixpoint + closest-anchor stinger
        // + winding-normalization in face_polygon.
        let block = box_at(Vec3::new(2.0, 2.0, 2.0), Point3::new(-1.0, -1.0, -1.0));
        let cyl = cylinder_faceted(0.6, 3.0, 12);
        let r = block.try_union(&cyl);
        assert!(r.is_ok(), "curved piercing should now produce a valid solid");
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
