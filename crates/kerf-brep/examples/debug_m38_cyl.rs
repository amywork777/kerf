//! M38 debug: see what classifications come up in cyl-box union, where
//! the chord-merge SHOULD trigger but isn't.

use kerf_brep::booleans::{
    BooleanOp, add_intersection_edges, classify_face, face_intersections, face_polygon,
    keep_a_face, keep_b_face, resolve_interior_endpoints, split_solids_at_intersections,
};
use kerf_brep::primitives::{box_at, cylinder_faceted};
use kerf_geom::{Point3, Tolerance, Vec3};

fn main() {
    let a = box_at(Vec3::new(2.0, 2.0, 2.0), Point3::new(-1.0, -1.0, -1.0));
    let b = cylinder_faceted(0.6, 3.0, 12);
    let op = BooleanOp::Union;
    let tol = Tolerance::default();

    let mut a = a.clone();
    let mut b = b.clone();
    let intersections = face_intersections(&a, &b, &tol);
    let outcome = split_solids_at_intersections(&mut a, &mut b, &intersections, &tol);
    let interior = resolve_interior_endpoints(&mut a, &mut b, &intersections, &outcome, &tol);
    let _added = add_intersection_edges(&mut a, &mut b, &intersections, &interior, &[], &tol);

    println!("After splits: A has {} faces, B has {} faces", a.face_count(), b.face_count());

    let mut kept_count = 0;
    let mut dropped_inside = 0;
    let mut dropped_onboundary = 0;
    let mut dropped_outside = 0;
    println!("\nA face classifications:");
    for face_id in a.topo.face_ids() {
        let cls = classify_face(&a, face_id, &b, &tol);
        let kept = keep_a_face(cls, op);
        let poly = face_polygon(&a, face_id).unwrap_or_default();
        println!("  {face_id:?} cls={cls:?} kept={kept} verts={}", poly.len());
        if kept {
            kept_count += 1;
        } else {
            match cls {
                kerf_brep::booleans::FaceClassification::Inside => dropped_inside += 1,
                kerf_brep::booleans::FaceClassification::OnBoundary => dropped_onboundary += 1,
                kerf_brep::booleans::FaceClassification::Outside => dropped_outside += 1,
            }
        }
    }
    println!("\nB face classifications:");
    for face_id in b.topo.face_ids() {
        let cls = classify_face(&b, face_id, &a, &tol);
        let kept = keep_b_face(cls, op);
        let poly = face_polygon(&b, face_id).unwrap_or_default();
        println!("  {face_id:?} cls={cls:?} kept={kept} verts={}", poly.len());
        if kept {
            kept_count += 1;
        } else {
            match cls {
                kerf_brep::booleans::FaceClassification::Inside => dropped_inside += 1,
                kerf_brep::booleans::FaceClassification::OnBoundary => dropped_onboundary += 1,
                kerf_brep::booleans::FaceClassification::Outside => dropped_outside += 1,
            }
        }
    }
    println!(
        "\nKept: {kept_count}; Dropped: Inside={dropped_inside} OnBoundary={dropped_onboundary} Outside={dropped_outside}"
    );
}
