//! Debug: union of two same-size boxes overlapping at a corner.
//!
//! A = [0,2]³, B = [1,3]³. Union should be a non-convex L-ish solid.
//! Currently fails with "edge has 1 half-edge" — classifier mismatch.
//!
//! This example prints which faces from each solid get kept and what their
//! classification is, so we can see which face is missing from the kept set.

use kerf_brep::booleans::{
    add_intersection_edges, classify_face, face_intersections, keep_a_face, keep_b_face,
    resolve_interior_endpoints, split_solids_at_intersections, BooleanOp,
};
use kerf_brep::primitives::{box_, box_at};
use kerf_geom::{Point3, Tolerance, Vec3};

fn main() {
    let a = box_(Vec3::new(2.0, 2.0, 2.0));
    let b = box_at(Vec3::new(2.0, 2.0, 2.0), Point3::new(1.0, 1.0, 1.0));
    let op = BooleanOp::Union;
    let tol = Tolerance::default();

    println!("A: {} faces", a.face_count());
    println!("B: {} faces", b.face_count());

    // Run the upstream phases manually so we can inspect classifications.
    let mut a = a.clone();
    let mut b = b.clone();
    let intersections = face_intersections(&a, &b, &tol);
    println!("intersections: {}", intersections.len());
    let outcome = split_solids_at_intersections(&mut a, &mut b, &intersections, &tol);
    let interior = resolve_interior_endpoints(&mut a, &mut b, &intersections, &outcome, &tol);
    let _added = add_intersection_edges(&mut a, &mut b, &intersections, &interior, &tol);

    println!("After splits: A has {} faces, B has {} faces", a.face_count(), b.face_count());

    println!("\nA face classifications:");
    let mut a_kept = 0;
    for face_id in a.topo.face_ids() {
        let cls = classify_face(&a, face_id, &b, &tol);
        let kept = keep_a_face(cls, op);
        println!("  {face_id:?}: {cls:?} → {}", if kept { "KEEP" } else { "drop" });
        if kept {
            a_kept += 1;
        }
    }
    println!("  total A kept: {a_kept}");

    println!("\nB face classifications:");
    let mut b_kept = 0;
    for face_id in b.topo.face_ids() {
        let cls = classify_face(&b, face_id, &a, &tol);
        let kept = keep_b_face(cls, op);
        println!("  {face_id:?}: {cls:?} → {}", if kept { "KEEP" } else { "drop" });
        if kept {
            b_kept += 1;
        }
    }
    println!("  total B kept: {b_kept}");

    println!("\nTotal kept faces: {}", a_kept + b_kept);
}
