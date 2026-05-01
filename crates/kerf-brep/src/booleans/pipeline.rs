//! End-to-end boolean: classify, select, triangulate.

use kerf_geom::{Point3, Tolerance};

use crate::Solid;
use crate::booleans::{
    BooleanOp, SelectedFaces, add_intersection_edges, classify_face, face_intersections,
    face_polygon, fan_triangulate, fan_triangulate_reversed, flip_b_face, keep_a_face, keep_b_face,
    split_solids_at_intersections,
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
    // Phase 1-3: clone, intersect, split edges, split faces.
    let mut a = a.clone();
    let mut b = b.clone();
    let intersections = face_intersections(&a, &b, tol);
    let outcome = split_solids_at_intersections(&mut a, &mut b, &intersections, tol);
    let _added = add_intersection_edges(&mut a, &mut b, &intersections, &outcome, tol);

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
    let _added = add_intersection_edges(&mut a, &mut b, &intersections, &outcome, tol);

    let mut kept: Vec<KeptFace> = Vec::new();
    for face_id in a.topo.face_ids() {
        let cls = classify_face(&a, face_id, &b, tol);
        if keep_a_face(cls, op)
            && let Some(polygon) = face_polygon(&a, face_id)
        {
            let surface = a.face_geom.get(face_id).cloned().unwrap();
            kept.push(KeptFace { polygon, surface });
        }
    }
    let flip_b = flip_b_face(op);
    for face_id in b.topo.face_ids() {
        let cls = classify_face(&b, face_id, &a, tol);
        if keep_b_face(cls, op)
            && let Some(mut polygon) = face_polygon(&b, face_id)
        {
            let surface = b.face_geom.get(face_id).cloned().unwrap();
            if flip_b {
                // Reverse polygon winding so the face's outward normal flips.
                // (We leave SurfaceKind::Plane unchanged; in the stitched solid
                // the polygon order alone defines face orientation.)
                polygon.reverse();
            }
            kept.push(KeptFace { polygon, surface });
        }
    }

    stitch(&kept, tol)
}

#[cfg(test)]
mod tests {
    use super::*;
    use kerf_geom::Vec3;

    use crate::geometry::{CurveKind, SurfaceKind};
    use crate::primitives::box_;

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

    #[test]
    fn union_of_disjoint_boxes_yields_all_faces_of_both() {
        // 2 unit boxes far apart: each contributes 6 quad faces = 12 quads = 24 triangles.
        let a = box_(Vec3::new(1.0, 1.0, 1.0));
        let b = make_box_at(Vec3::new(1.0, 1.0, 1.0), Vec3::new(5.0, 0.0, 0.0));
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
        let b = make_box_at(Vec3::new(1.0, 1.0, 1.0), Vec3::new(5.0, 0.0, 0.0));
        let soup = boolean(&a, &b, BooleanOp::Intersection, &Tolerance::default());
        assert_eq!(soup.triangles.len(), 0);
    }

    #[test]
    fn intersection_of_nested_boxes_is_smaller_box() {
        // Big [0,10]^3 ∩ small [4,6]^3 = small box's surface = 6 quads = 12 triangles.
        let big = box_(Vec3::new(10.0, 10.0, 10.0));
        let small = make_box_at(Vec3::new(2.0, 2.0, 2.0), Vec3::new(4.0, 4.0, 4.0));
        let soup = boolean(&big, &small, BooleanOp::Intersection, &Tolerance::default());
        // Small box's 6 faces are inside big → 12 triangles. Big's faces are outside small → 0 contribution.
        assert_eq!(soup.triangles.len(), 12);
    }

    #[test]
    fn union_of_nested_boxes_is_larger_box() {
        // big ∪ small (small inside big) = big alone (small contributes nothing).
        let big = box_(Vec3::new(10.0, 10.0, 10.0));
        let small = make_box_at(Vec3::new(2.0, 2.0, 2.0), Vec3::new(4.0, 4.0, 4.0));
        let soup = boolean(&big, &small, BooleanOp::Union, &Tolerance::default());
        assert_eq!(soup.triangles.len(), 12, "big's 6 faces = 12 triangles");
    }

    #[test]
    fn difference_of_nested_boxes_keeps_big_outside_and_small_inside_flipped() {
        // big - small: big's 6 outside faces + small's 6 inside-flipped faces = 24 triangles.
        let big = box_(Vec3::new(10.0, 10.0, 10.0));
        let small = make_box_at(Vec3::new(2.0, 2.0, 2.0), Vec3::new(4.0, 4.0, 4.0));
        let soup = boolean(&big, &small, BooleanOp::Difference, &Tolerance::default());
        assert_eq!(soup.triangles.len(), 24);
    }

    #[test]
    fn boolean_solid_intersection_of_nested_yields_smaller_box_topology() {
        let big = box_(Vec3::new(10.0, 10.0, 10.0));
        let small = make_box_at(Vec3::new(2.0, 2.0, 2.0), Vec3::new(4.0, 4.0, 4.0));
        let result = boolean_solid(&big, &small, BooleanOp::Intersection, &Tolerance::default());
        assert_eq!(result.vertex_count(), 8);
        assert_eq!(result.edge_count(), 12);
        assert_eq!(result.face_count(), 6);
        kerf_topo::validate(&result.topo).unwrap();
    }

    #[test]
    fn boolean_solid_union_of_nested_yields_big_box_topology() {
        let big = box_(Vec3::new(10.0, 10.0, 10.0));
        let small = make_box_at(Vec3::new(2.0, 2.0, 2.0), Vec3::new(4.0, 4.0, 4.0));
        let result = boolean_solid(&big, &small, BooleanOp::Union, &Tolerance::default());
        assert_eq!(result.vertex_count(), 8);
        assert_eq!(result.face_count(), 6);
        kerf_topo::validate(&result.topo).unwrap();
    }

    #[test]
    fn recursive_boolean_three_box_intersection() {
        // (A ∩ B) ∩ C — verifies recursion works.
        let a = box_(Vec3::new(10.0, 10.0, 10.0));
        let b = make_box_at(Vec3::new(2.0, 2.0, 2.0), Vec3::new(4.0, 4.0, 4.0));
        let c = make_box_at(Vec3::new(2.0, 2.0, 2.0), Vec3::new(4.0, 4.0, 4.0));
        let ab = boolean_solid(&a, &b, BooleanOp::Intersection, &Tolerance::default());
        let abc = boolean_solid(&ab, &c, BooleanOp::Intersection, &Tolerance::default());
        // ab is "small" (inside big); abc = small ∩ small = small.
        assert_eq!(abc.face_count(), 6);
        kerf_topo::validate(&abc.topo).unwrap();
    }
}
