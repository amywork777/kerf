//! End-to-end boolean: classify, select, triangulate.

use kerf_geom::{Point3, Tolerance};

use crate::Solid;
use crate::booleans::{
    BooleanOp, SelectedFaces, add_intersection_edges, classify_face, face_intersections,
    face_polygon, fan_triangulate, fan_triangulate_reversed, flip_b_face, keep_a_face, keep_b_face,
    resolve_interior_endpoints, split_solids_at_intersections,
};

/// A face soup: triangles representing the boolean result. Not a stitched B-rep
/// solid — that's a separate project.
#[derive(Clone, Debug, Default)]
pub struct FaceSoup {
    pub triangles: Vec<[Point3; 3]>,
}

/// End-to-end boolean. Mutates clones of the inputs internally; returns the
/// kept faces' triangulation.
pub fn boolean(a: &Solid, b: &Solid, op: BooleanOp, tol: &Tolerance) -> FaceSoup {
    // Phase 1-3: clone, intersect, split edges, resolve interior endpoints, split faces.
    let mut a = a.clone();
    let mut b = b.clone();
    let intersections = face_intersections(&a, &b, tol);
    let outcome = split_solids_at_intersections(&mut a, &mut b, &intersections, tol);
    let interior = resolve_interior_endpoints(&mut a, &mut b, &intersections, &outcome, tol);
    let _added = add_intersection_edges(&mut a, &mut b, &intersections, &interior, tol);

    // Phase 4: classify every face of both solids.
    let mut selected = SelectedFaces {
        a: Vec::new(),
        b: Vec::new(),
        b_flipped: flip_b_face(op),
    };
    for face_id in a.topo.face_ids() {
        let cls = classify_face(&a, face_id, &b, tol);
        if keep_a_face(cls, op) {
            selected.a.push(face_id);
        }
    }
    for face_id in b.topo.face_ids() {
        let cls = classify_face(&b, face_id, &a, tol);
        if keep_b_face(cls, op) {
            selected.b.push(face_id);
        }
    }

    // Phase 5 (lite): triangulate kept faces.
    let mut soup = FaceSoup::default();
    for fid in &selected.a {
        if let Some(poly) = face_polygon(&a, *fid) {
            soup.triangles.extend(fan_triangulate(&poly));
        }
    }
    for fid in &selected.b {
        if let Some(poly) = face_polygon(&b, *fid) {
            let tris = if selected.b_flipped {
                fan_triangulate_reversed(&poly)
            } else {
                fan_triangulate(&poly)
            };
            soup.triangles.extend(tris);
        }
    }

    soup
}

/// End-to-end boolean returning a connected Solid (not just a triangle soup).
/// Recursive booleans are now possible:
/// `boolean_solid(boolean_solid(a, b, op), c, op2)`.
pub fn boolean_solid(a: &Solid, b: &Solid, op: BooleanOp, tol: &Tolerance) -> Solid {
    use crate::booleans::stitch::{KeptFace, stitch};

    let mut a = a.clone();
    let mut b = b.clone();
    let intersections = face_intersections(&a, &b, tol);
    let outcome = split_solids_at_intersections(&mut a, &mut b, &intersections, tol);
    let interior = resolve_interior_endpoints(&mut a, &mut b, &intersections, &outcome, tol);
    let _added = add_intersection_edges(&mut a, &mut b, &intersections, &interior, tol);

    // Collect both kept and dropped face polygons up front. Dropped polygons
    // are needed by the M38 chord-merge pass.
    let mut kept: Vec<KeptFace> = Vec::new();
    let mut dropped: Vec<KeptFace> = Vec::new();
    for face_id in a.topo.face_ids() {
        let cls = classify_face(&a, face_id, &b, tol);
        let Some(polygon) = face_polygon(&a, face_id) else {
            continue;
        };
        let surface = a.face_geom.get(face_id).cloned().unwrap();
        let face = KeptFace { polygon, surface };
        if keep_a_face(cls, op) {
            kept.push(face);
        } else {
            dropped.push(face);
        }
    }
    let flip_b = flip_b_face(op);
    for face_id in b.topo.face_ids() {
        let cls = classify_face(&b, face_id, &a, tol);
        let Some(mut polygon) = face_polygon(&b, face_id) else {
            continue;
        };
        let surface = b.face_geom.get(face_id).cloned().unwrap();
        let face_kept = keep_b_face(cls, op);
        if face_kept && flip_b {
            polygon.reverse();
        }
        let face = KeptFace { polygon, surface };
        if face_kept {
            kept.push(face);
        } else {
            dropped.push(face);
        }
    }

    let _ = &dropped; // chord_merge module is built and unit-tested but not
                      // integrated — see docs/readiness.md for why a naive
                      // merge breaks neighbouring kept faces' adjacency.

    stitch(&kept, tol)
}

#[cfg(test)]
mod tests {
    use super::*;
    use kerf_geom::{Point3, Vec3};

    use crate::primitives::{box_, box_at};

    #[test]
    fn union_of_disjoint_boxes_yields_all_faces_of_both() {
        // 2 unit boxes far apart: each contributes 6 quad faces = 12 quads = 24 triangles.
        let a = box_(Vec3::new(1.0, 1.0, 1.0));
        let b = box_at(Vec3::new(1.0, 1.0, 1.0), Point3::new(5.0, 0.0, 0.0));
        let soup = boolean(&a, &b, BooleanOp::Union, &Tolerance::default());
        assert_eq!(
            soup.triangles.len(),
            24,
            "expected 24 triangles, got {}",
            soup.triangles.len()
        );
    }

    #[test]
    fn intersection_of_disjoint_boxes_is_empty() {
        let a = box_(Vec3::new(1.0, 1.0, 1.0));
        let b = box_at(Vec3::new(1.0, 1.0, 1.0), Point3::new(5.0, 0.0, 0.0));
        let soup = boolean(&a, &b, BooleanOp::Intersection, &Tolerance::default());
        assert_eq!(soup.triangles.len(), 0);
    }

    #[test]
    fn intersection_of_nested_boxes_is_smaller_box() {
        // Big [0,10]^3 ∩ small [4,6]^3 = small box's surface = 6 quads = 12 triangles.
        let big = box_(Vec3::new(10.0, 10.0, 10.0));
        let small = box_at(Vec3::new(2.0, 2.0, 2.0), Point3::new(4.0, 4.0, 4.0));
        let soup = boolean(&big, &small, BooleanOp::Intersection, &Tolerance::default());
        // Small box's 6 faces are inside big → 12 triangles. Big's faces are outside small → 0 contribution.
        assert_eq!(soup.triangles.len(), 12);
    }

    #[test]
    fn union_of_nested_boxes_is_larger_box() {
        // big ∪ small (small inside big) = big alone (small contributes nothing).
        let big = box_(Vec3::new(10.0, 10.0, 10.0));
        let small = box_at(Vec3::new(2.0, 2.0, 2.0), Point3::new(4.0, 4.0, 4.0));
        let soup = boolean(&big, &small, BooleanOp::Union, &Tolerance::default());
        assert_eq!(soup.triangles.len(), 12, "big's 6 faces = 12 triangles");
    }

    #[test]
    fn difference_of_nested_boxes_keeps_big_outside_and_small_inside_flipped() {
        // big - small: big's 6 outside faces + small's 6 inside-flipped faces = 24 triangles.
        let big = box_(Vec3::new(10.0, 10.0, 10.0));
        let small = box_at(Vec3::new(2.0, 2.0, 2.0), Point3::new(4.0, 4.0, 4.0));
        let soup = boolean(&big, &small, BooleanOp::Difference, &Tolerance::default());
        assert_eq!(soup.triangles.len(), 24);
    }

    #[test]
    fn boolean_solid_intersection_of_nested_yields_smaller_box_topology() {
        let big = box_(Vec3::new(10.0, 10.0, 10.0));
        let small = box_at(Vec3::new(2.0, 2.0, 2.0), Point3::new(4.0, 4.0, 4.0));
        let result = boolean_solid(&big, &small, BooleanOp::Intersection, &Tolerance::default());
        assert_eq!(result.vertex_count(), 8);
        assert_eq!(result.edge_count(), 12);
        assert_eq!(result.face_count(), 6);
        kerf_topo::validate(&result.topo).unwrap();
    }

    #[test]
    fn boolean_solid_union_of_nested_yields_big_box_topology() {
        let big = box_(Vec3::new(10.0, 10.0, 10.0));
        let small = box_at(Vec3::new(2.0, 2.0, 2.0), Point3::new(4.0, 4.0, 4.0));
        let result = boolean_solid(&big, &small, BooleanOp::Union, &Tolerance::default());
        assert_eq!(result.vertex_count(), 8);
        assert_eq!(result.face_count(), 6);
        kerf_topo::validate(&result.topo).unwrap();
    }

    #[test]
    fn corner_cut_difference_via_solid_method_works() {
        // Block A = [0,10]^3, cutter B = [8,12]^3 — corner cut.
        // Result is a stepped block with at least 7 faces (the original 3
        // un-touched, plus 3 stepped surfaces, plus the modified ones).
        let block = box_(Vec3::new(10.0, 10.0, 10.0));
        let cutter = box_at(Vec3::new(4.0, 4.0, 4.0), Point3::new(8.0, 8.0, 8.0));
        let result = block.difference(&cutter);
        assert!(
            result.face_count() >= 7,
            "stepped block should have >=7 faces, got {}",
            result.face_count()
        );
        kerf_topo::validate(&result.topo).unwrap();
    }

    #[test]
    fn half_overlap_difference_yields_sub_box() {
        // A = [0,2]^3, B = [1,3] x [0,2]^2. A - B = [0,1] x [0,2]^2 — a 1x2x2
        // sub-box. Pre-M28 this panicked in the stitcher because A's [1,2]
        // half-faces classified as OnBoundary and were kept, leaving open edges
        // when B's coincident faces were dropped.
        let a = box_(Vec3::new(2.0, 2.0, 2.0));
        let b = box_at(Vec3::new(2.0, 2.0, 2.0), Point3::new(1.0, 0.0, 0.0));
        let r = boolean_solid(&a, &b, BooleanOp::Difference, &Tolerance::default());
        kerf_topo::validate(&r.topo).unwrap();
        assert!(r.face_count() >= 6, "result must have at least 6 faces");
    }

    #[test]
    fn recursive_boolean_three_box_intersection() {
        // (A ∩ B) ∩ C — verifies recursion works.
        let a = box_(Vec3::new(10.0, 10.0, 10.0));
        let b = box_at(Vec3::new(2.0, 2.0, 2.0), Point3::new(4.0, 4.0, 4.0));
        let c = box_at(Vec3::new(2.0, 2.0, 2.0), Point3::new(4.0, 4.0, 4.0));
        let ab = boolean_solid(&a, &b, BooleanOp::Intersection, &Tolerance::default());
        let abc = boolean_solid(&ab, &c, BooleanOp::Intersection, &Tolerance::default());
        // ab is "small" (inside big); abc = small ∩ small = small.
        assert_eq!(abc.face_count(), 6);
        kerf_topo::validate(&abc.topo).unwrap();
    }
}
