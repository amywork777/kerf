//! Face-face intersection between two solids: returns clipped 3D segments
//! per face pair.

use kerf_geom::intersect::{
    IntersectionComponent, SurfaceSurfaceIntersection, intersect_plane_plane,
};
use kerf_geom::{Plane, Point3, Tolerance};
use kerf_topo::FaceId;

use crate::Solid;
use crate::booleans::{
    BooleanOp, ClipResult, FaceClassification, classify_face, clip_line_to_convex_polygon,
    face_polygon, keep_a_face, keep_b_face,
};
use crate::geometry::SurfaceKind;

#[derive(Clone, Debug)]
pub struct FaceIntersection {
    pub face_a: FaceId,
    pub face_b: FaceId,
    pub start: Point3,
    pub end: Point3,
}

pub fn face_intersections(a: &Solid, b: &Solid, tol: &Tolerance) -> Vec<FaceIntersection> {
    let mut results = Vec::new();
    for fa_id in a.topo.face_ids() {
        let SurfaceKind::Plane(plane_a) = a
            .face_geom
            .get(fa_id)
            .cloned()
            .unwrap_or_else(|| panic!("face {fa_id:?} has no surface"))
        else {
            continue;
        };
        for fb_id in b.topo.face_ids() {
            let SurfaceKind::Plane(plane_b) = b
                .face_geom
                .get(fb_id)
                .cloned()
                .unwrap_or_else(|| panic!("face {fb_id:?} has no surface"))
            else {
                continue;
            };

            if let Some(seg) = intersect_planar_pair(a, fa_id, &plane_a, b, fb_id, &plane_b, tol) {
                results.push(seg);
            }
        }
    }
    results
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
    chord_interior_by_classification(cls_a, cls_b, op)
}

/// Centralized M39 criterion: skip mef only when both hosts classify as
/// strictly `Inside` against the other solid AND would be dropped under op.
/// `Inside` (vs `OnBoundary`) means the centroid is unambiguously interior;
/// these chords' splits produce dropped-only pieces that confuse stitch.
///
/// Empirically: aggressive variants ("both dropped including OnBoundary")
/// over-skip and break half-overlap cases. The Inside/Inside gate is the
/// safe ceiling — the bulk of the M39 win comes from the area-weighted
/// centroid in `face_centroid` (which the gate complements without
/// overriding).
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

    use crate::primitives::{box_, box_at};

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
}
