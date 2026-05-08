//! Face-face intersection between two solids: returns clipped 3D segments
//! per face pair.

use kerf_geom::intersect::{
    IntersectionComponent, SurfaceSurfaceIntersection, intersect_plane_plane,
};
use kerf_geom::{Plane, Point3, Tolerance};
use kerf_topo::FaceId;

use crate::Solid;
use crate::booleans::analytic_curves::{
    CylinderPlaneIntersection, cylinder_plane_intersection,
};
use crate::booleans::{
    BooleanOp, ClipResult, FaceClassification, classify_face, clip_line_to_convex_polygon,
    face_polygon, keep_a_face, keep_b_face,
};
use crate::geometry::{EllipseSegment, SurfaceKind};

/// Classification of a `FaceIntersection`'s underlying chord geometry.
///
/// Tier 2 of curved-surface stitch wiring. `Linear` is the legacy planar
/// chord that the entire boolean engine has always handled. `Arc` is the
/// newer Cylinder×Plane chord, where the closed-form intersection is an
/// ellipse (or degenerate circle) loop rather than a straight segment.
///
/// The presence of an `Arc(...)` variant in the result of `face_intersections`
/// signals to downstream consumers that this chord can't go through the
/// existing line-chord splice/split/stitch pipeline as-is. The current
/// pipeline filters them out before mef'ing — Tier 3 will route them
/// through curve-aware splice/stitch instead.
#[derive(Clone, Debug)]
pub enum FaceIntersectionKind {
    /// Straight chord between `start` and `end`. The line direction equals
    /// `(end - start).normalize()`.
    Linear,
    /// Arc chord on a curved face boundary (today: closed ellipse from
    /// Cylinder×Plane). `start == end == segment.start_point()` for closed
    /// loops, since the conic returns to itself.
    Arc(EllipseSegment),
}

#[derive(Clone, Debug)]
pub struct FaceIntersection {
    pub face_a: FaceId,
    pub face_b: FaceId,
    pub start: Point3,
    pub end: Point3,
    /// Tier 2: chord geometry kind. Defaults to `Linear` for the legacy
    /// Plane×Plane path. `Arc` for Cylinder×Plane.
    pub kind: FaceIntersectionKind,
}

impl FaceIntersection {
    /// True iff this intersection is the legacy linear-chord variant. Used
    /// by downstream consumers (splice, split, classify_chord_interiorness)
    /// that aren't yet curve-aware to skip arc chords without inspecting
    /// the kind enum directly.
    pub fn is_linear(&self) -> bool {
        matches!(self.kind, FaceIntersectionKind::Linear)
    }
}

pub fn face_intersections(a: &Solid, b: &Solid, tol: &Tolerance) -> Vec<FaceIntersection> {
    let mut results = Vec::new();
    for fa_id in a.topo.face_ids() {
        let surf_a = a
            .face_geom
            .get(fa_id)
            .cloned()
            .unwrap_or_else(|| panic!("face {fa_id:?} has no surface"));
        for fb_id in b.topo.face_ids() {
            let surf_b = b
                .face_geom
                .get(fb_id)
                .cloned()
                .unwrap_or_else(|| panic!("face {fb_id:?} has no surface"));

            match (&surf_a, &surf_b) {
                (SurfaceKind::Plane(pa), SurfaceKind::Plane(pb)) => {
                    if let Some(seg) =
                        intersect_planar_pair(a, fa_id, pa, b, fb_id, pb, tol)
                    {
                        results.push(seg);
                    }
                }
                (SurfaceKind::Cylinder(cyl), SurfaceKind::Plane(pl)) => {
                    if let Some(seg) =
                        intersect_cylinder_plane_pair(fa_id, cyl, fb_id, pl, false, tol)
                    {
                        results.push(seg);
                    }
                }
                (SurfaceKind::Plane(pl), SurfaceKind::Cylinder(cyl)) => {
                    if let Some(seg) =
                        intersect_cylinder_plane_pair(fa_id, cyl, fb_id, pl, true, tol)
                    {
                        results.push(seg);
                    }
                }
                _ => {
                    // Other curved-surface combinations (Cylinder×Cylinder,
                    // Sphere×Plane, Torus×anything) aren't yet typed at the
                    // brep layer. They fall through unchanged today —
                    // analytic-curves.rs's roadmap will pick them up.
                }
            }
        }
    }
    results
}

fn intersect_cylinder_plane_pair(
    face_cyl: FaceId,
    cyl: &kerf_geom::Cylinder,
    face_plane: FaceId,
    plane: &Plane,
    cylinder_is_b: bool,
    tol: &Tolerance,
) -> Option<FaceIntersection> {
    // Use the closed-form Cylinder×Plane intersection from analytic_curves.
    // Tier 2: emit an Arc-kind FaceIntersection for true closed-conic
    // intersections (Circle/Ellipse). The TwoLines/Tangent regimes are
    // genuinely two (or one) ruling segments along the cylinder's axis —
    // those aren't yet wired to face_intersections because clipping a
    // ruling line against a finite cylinder face's actual rectangular
    // u-v domain requires per-primitive face geometry that the brep
    // layer doesn't currently expose. We return None for those regimes,
    // matching today's behavior of skipping Cylinder×Plane entirely; the
    // arc-loop case is the new addition.
    match cylinder_plane_intersection(cyl, plane, tol) {
        CylinderPlaneIntersection::Empty => None,
        CylinderPlaneIntersection::Tangent(_) => None,
        CylinderPlaneIntersection::TwoLines(_, _) => None,
        CylinderPlaneIntersection::Circle(seg)
        | CylinderPlaneIntersection::Ellipse(seg) => {
            // Closed loop: start == end. Use the ellipse parameter t=0
            // anchor as both endpoints; downstream consumers that treat
            // this as a degenerate point intersection (because they
            // haven't been taught to walk arc kind) won't process it.
            let p = seg.start_point();
            let (face_a, face_b) = if cylinder_is_b {
                (face_plane, face_cyl)
            } else {
                (face_cyl, face_plane)
            };
            Some(FaceIntersection {
                face_a,
                face_b,
                start: p,
                end: p,
                kind: FaceIntersectionKind::Arc(seg),
            })
        }
    }
}

fn intersect_planar_pair(
    a: &Solid,
    fa_id: FaceId,
    plane_a: &Plane,
    b: &Solid,
    fb_id: FaceId,
    plane_b: &Plane,
    tol: &Tolerance,
) -> Option<FaceIntersection> {
    let sse = intersect_plane_plane(plane_a, plane_b, tol);
    let SurfaceSurfaceIntersection::Components(comps) = sse else {
        return None;
    };
    let comp = comps.into_iter().find_map(|c| match c {
        IntersectionComponent::Line(l) => Some(l),
        _ => None,
    })?;

    let poly_a = face_polygon(a, fa_id)?;
    let poly_b = face_polygon(b, fb_id)?;

    let clip_a = clip_line_to_convex_polygon(&comp, &poly_a, &plane_a.frame, tol);
    let clip_b = clip_line_to_convex_polygon(&comp, &poly_b, &plane_b.frame, tol);

    let (a_min, a_max) = match clip_a {
        ClipResult::Range(lo, hi) => (lo, hi),
        ClipResult::Empty => return None,
    };
    let (b_min, b_max) = match clip_b {
        ClipResult::Range(lo, hi) => (lo, hi),
        ClipResult::Empty => return None,
    };

    let lo = a_min.max(b_min);
    let hi = a_max.min(b_max);
    if hi - lo < tol.point_eq {
        return None;
    }

    let start = comp.origin + lo * comp.direction;
    let end = comp.origin + hi * comp.direction;
    Some(FaceIntersection {
        face_a: fa_id,
        face_b: fb_id,
        start,
        end,
        kind: FaceIntersectionKind::Linear,
    })
}

/// M39: per-chord "skip mef" flag.
///
/// For each `FaceIntersection`, classify the pre-split host faces against the
/// other solid. The chord is "interior" — and the mef can be safely skipped —
/// when both host faces would be DROPPED under the boolean op. Splitting an
/// interior chord produces two pieces of a face, both classified the same way
/// as the original (Inside stays Inside, OnBoundary's centroid often stays on
/// the boundary), so both pieces get dropped — and the chord ends up with one
/// or zero half-edges in the kept manifold. Skipping the split avoids that.
pub fn classify_chord_interiorness(
    a: &Solid,
    b: &Solid,
    intersections: &[FaceIntersection],
    op: BooleanOp,
    tol: &Tolerance,
) -> Vec<bool> {
    intersections
        .iter()
        .map(|inter| is_chord_interior(a, b, inter, op, tol))
        .collect()
}

fn is_chord_interior(
    a: &Solid,
    b: &Solid,
    inter: &FaceIntersection,
    op: BooleanOp,
    tol: &Tolerance,
) -> bool {
    let cls_a = classify_face(a, inter.face_a, b, tol);
    let cls_b = classify_face(b, inter.face_b, a, tol);
    if !chord_interior_by_classification(cls_a, cls_b, op) {
        return false;
    }
    // M39g: don't trust centroid-only Inside-Inside if the face has any
    // vertex OUTSIDE the other solid. The pre-split centroid can land
    // inside even when most of the face is outside (e.g., box-shift.x=1
    // is a 2×2 square centered on (1,1,1) — its center is inside the
    // box-nested cube [0.7,1.3]³, but its corners and most of its area
    // are outside). Skipping the chord then drops the whole face when
    // really only the interior crossing region should be dropped.
    if !face_fully_inside(a, inter.face_a, b, tol)
        || !face_fully_inside(b, inter.face_b, a, tol)
    {
        return false;
    }
    true
}

/// True when every vertex of `face` (in `solid`) lies inside `other` via a
/// ray-cast classification. Single OnBoundary or Outside vertex disqualifies
/// the face from being treated as fully interior.
fn face_fully_inside(solid: &Solid, face: FaceId, other: &Solid, tol: &Tolerance) -> bool {
    let Some(poly) = face_polygon(solid, face) else {
        return false;
    };
    if poly.is_empty() {
        return false;
    }
    use crate::booleans::classify::{point_on_solid_boundary, ray_solid_crossings};
    use kerf_geom::Vec3;
    let dirs = [
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
    ];
    for &p in &poly {
        if point_on_solid_boundary(other, p, tol).is_some() {
            return false; // OnBoundary vertex — ambiguous
        }
        let mut decided = false;
        for &dir in &dirs {
            if let Some(count) = ray_solid_crossings(other, p, dir, tol) {
                if count % 2 == 0 {
                    return false; // vertex Outside
                }
                decided = true;
                break;
            }
        }
        if !decided {
            return false; // ray cast couldn't decide
        }
    }
    true
}

/// Centralized M39 criterion: skip mef only when both hosts classify as
/// strictly `Inside` against the other solid AND would be dropped under op.
/// Now combined with `face_fully_inside` (M39g) to guard against the
/// misleading-centroid case (face's center is inside but corners are not).
fn chord_interior_by_classification(
    cls_a: FaceClassification,
    cls_b: FaceClassification,
    op: BooleanOp,
) -> bool {
    use FaceClassification::Inside;
    matches!((cls_a, cls_b), (Inside, Inside))
        && !keep_a_face(cls_a, op)
        && !keep_b_face(cls_b, op)
}

#[cfg(test)]
mod tests {
    use super::*;
    use kerf_geom::{Point3, Vec3};

    use crate::primitives::{box_, box_at, cylinder};

    #[test]
    fn disjoint_boxes_have_no_face_intersections() {
        let a = box_(Vec3::new(1.0, 1.0, 1.0));
        let b = box_at(Vec3::new(1.0, 1.0, 1.0), Point3::new(5.0, 0.0, 0.0));
        let result = face_intersections(&a, &b, &Tolerance::default());
        assert_eq!(result.len(), 0);
    }

    #[test]
    fn overlapping_boxes_produce_intersection_segments() {
        let a = box_(Vec3::new(1.0, 1.0, 1.0));
        let b = box_at(Vec3::new(1.0, 1.0, 1.0), Point3::new(0.5, 0.0, 0.0));
        let result = face_intersections(&a, &b, &Tolerance::default());
        assert!(
            result.len() >= 4,
            "expected at least 4 segments, got {}",
            result.len()
        );
        for seg in &result {
            // All planar inputs → all chords must be Linear kind.
            assert!(seg.is_linear(), "expected Linear chord");
            let len = (seg.end - seg.start).norm();
            assert!(
                len > 1e-6,
                "segment too short: {:?} -> {:?}",
                seg.start,
                seg.end
            );
        }
    }

    #[test]
    fn nested_box_inside_larger_has_no_intersection() {
        let big = box_(Vec3::new(10.0, 10.0, 10.0));
        let small = box_at(Vec3::new(2.0, 2.0, 2.0), Point3::new(4.0, 4.0, 4.0));
        let result = face_intersections(&big, &small, &Tolerance::default());
        assert_eq!(result.len(), 0);
    }

    /// Tier 2 test: Cylinder×Plane face pairs must emit `Arc(EllipseSegment)`
    /// kind FaceIntersections, not be silently skipped.
    #[test]
    fn face_intersection_returns_arc_for_cylinder_plane() {
        // Unit-radius cylinder of height 4 centered on Z axis (its lateral
        // face has SurfaceKind::Cylinder), and a box [0,4]³ centered at
        // origin so its top face's plane (z=2) cuts the cylinder
        // perpendicular to the axis. The Cylinder×Plane intersection is a
        // unit circle.
        let cyl = cylinder(1.0, 4.0);
        let bx = box_at(
            Vec3::new(4.0, 4.0, 4.0),
            Point3::new(-2.0, -2.0, 0.0), // bottom face at z=0, top at z=4
        );
        // Make box smaller on z so its top face plane is at z=2 (inside the
        // cylinder's lateral range).
        let plate = box_at(
            Vec3::new(4.0, 4.0, 2.0),
            Point3::new(-2.0, -2.0, 0.0), // top face at z=2
        );

        let result = face_intersections(&cyl, &plate, &Tolerance::default());
        // We expect AT LEAST one arc-kind chord — the cylinder's lateral
        // face vs each of the box's parallel-to-axis or perpendicular-
        // to-axis faces.
        let arc_count = result.iter().filter(|i| !i.is_linear()).count();
        assert!(
            arc_count > 0,
            "expected at least one Arc-kind chord from Cylinder×Plane, got {} total ({} linear)",
            result.len(),
            result.iter().filter(|i| i.is_linear()).count()
        );
        // And every Arc chord's payload must be a closed ellipse segment.
        for inter in &result {
            if let FaceIntersectionKind::Arc(seg) = &inter.kind {
                assert!(
                    seg.is_full(),
                    "arc segment is expected to be a full closed loop"
                );
                // Sanity: arc center lies on the cylinder axis.
                assert!(
                    seg.ellipse.frame.origin.x.abs() < 1e-9,
                    "arc center should be on Z axis (x=0), got {}",
                    seg.ellipse.frame.origin.x
                );
                assert!(
                    seg.ellipse.frame.origin.y.abs() < 1e-9,
                    "arc center should be on Z axis (y=0), got {}",
                    seg.ellipse.frame.origin.y
                );
            }
            // No `else` — Linear chords also legitimate (cylinder caps are
            // planes intersecting the box's planes).
            let _ = bx; // silence unused
        }
        let _ = bx; // silence unused-variable lint
    }

    /// Tier 2 sanity: pure planar booleans must not regress under the new
    /// kind-aware face_intersections (they should all emit Linear kind).
    #[test]
    fn planar_overlap_emits_only_linear_chords() {
        let a = box_(Vec3::new(2.0, 2.0, 2.0));
        let b = box_at(Vec3::new(2.0, 2.0, 2.0), Point3::new(1.0, 0.0, 0.0));
        let result = face_intersections(&a, &b, &Tolerance::default());
        assert!(!result.is_empty());
        for seg in &result {
            assert!(
                matches!(seg.kind, FaceIntersectionKind::Linear),
                "expected only Linear, got {:?}",
                seg.kind
            );
        }
    }
}
