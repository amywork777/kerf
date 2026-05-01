//! Face-face intersection between two solids: returns clipped 3D segments
//! per face pair.

use kerf_geom::intersect::{intersect_plane_plane, IntersectionComponent, SurfaceSurfaceIntersection};
use kerf_geom::{Plane, Point3, Tolerance};
use kerf_topo::FaceId;

use crate::booleans::{clip_line_to_convex_polygon, face_polygon, ClipResult};
use crate::geometry::SurfaceKind;
use crate::Solid;

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
        let SurfaceKind::Plane(plane_a) = a.face_geom.get(fa_id).cloned()
            .unwrap_or_else(|| panic!("face {fa_id:?} has no surface"))
        else { continue };
        for fb_id in b.topo.face_ids() {
            let SurfaceKind::Plane(plane_b) = b.face_geom.get(fb_id).cloned()
                .unwrap_or_else(|| panic!("face {fb_id:?} has no surface"))
            else { continue };

            if let Some(seg) = intersect_planar_pair(a, fa_id, &plane_a, b, fb_id, &plane_b, tol) {
                results.push(seg);
            }
        }
    }
    results
}

fn intersect_planar_pair(
    a: &Solid, fa_id: FaceId, plane_a: &Plane,
    b: &Solid, fb_id: FaceId, plane_b: &Plane,
    tol: &Tolerance,
) -> Option<FaceIntersection> {
    let sse = intersect_plane_plane(plane_a, plane_b, tol);
    let SurfaceSurfaceIntersection::Components(comps) = sse else { return None };
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
    if hi - lo < tol.point_eq { return None; }

    let start = comp.origin + lo * comp.direction;
    let end = comp.origin + hi * comp.direction;
    Some(FaceIntersection { face_a: fa_id, face_b: fb_id, start, end })
}

#[cfg(test)]
mod tests {
    use super::*;
    use kerf_geom::Vec3;

    use crate::primitives::box_;
    use crate::CurveKind;

    #[test]
    fn disjoint_boxes_have_no_face_intersections() {
        let a = box_(Vec3::new(1.0, 1.0, 1.0));
        let b = make_box_at(Vec3::new(1.0, 1.0, 1.0), Vec3::new(5.0, 0.0, 0.0));
        let result = face_intersections(&a, &b, &Tolerance::default());
        assert_eq!(result.len(), 0);
    }

    #[test]
    fn overlapping_boxes_produce_intersection_segments() {
        let a = box_(Vec3::new(1.0, 1.0, 1.0));
        let b = make_box_at(Vec3::new(1.0, 1.0, 1.0), Vec3::new(0.5, 0.0, 0.0));
        let result = face_intersections(&a, &b, &Tolerance::default());
        assert!(result.len() >= 4, "expected at least 4 segments, got {}", result.len());
        for seg in &result {
            let len = (seg.end - seg.start).norm();
            assert!(len > 1e-6, "segment too short: {:?} -> {:?}", seg.start, seg.end);
        }
    }

    #[test]
    fn nested_box_inside_larger_has_no_intersection() {
        let big = box_(Vec3::new(10.0, 10.0, 10.0));
        let small = make_box_at(Vec3::new(2.0, 2.0, 2.0), Vec3::new(4.0, 4.0, 4.0));
        let result = face_intersections(&big, &small, &Tolerance::default());
        assert_eq!(result.len(), 0);
    }

    fn make_box_at(extents: Vec3, offset: Vec3) -> Solid {
        let mut s = box_(extents);
        for (_, p) in s.vertex_geom.iter_mut() {
            *p += offset;
        }
        for (_, surf) in s.face_geom.iter_mut() {
            if let SurfaceKind::Plane(plane) = surf {
                plane.frame.origin += offset;
            }
        }
        for (_, seg) in s.edge_geom.iter_mut() {
            if let CurveKind::Line(line) = &mut seg.curve {
                line.origin += offset;
            }
        }
        s
    }
}
