//! Debug union(box-shift, box-nested) — currently failing with 1-half-edge.

use kerf_brep::booleans::{
    add_intersection_edges, classify_chord_interiorness, classify_face, face_intersections,
    keep_a_face, keep_b_face, resolve_interior_endpoints, split_solids_at_intersections, BooleanOp,
};
use kerf_brep::primitives::{box_, box_at};
use kerf_geom::{Point3, Tolerance, Vec3};

fn main() {
    let a = box_at(Vec3::new(0.6, 0.6, 0.6), Point3::new(0.7, 0.7, 0.7)); // box-nested
    let b = box_at(Vec3::new(2.0, 2.0, 2.0), Point3::new(1.0, 0.0, 0.0)); // box-shift
    let op = BooleanOp::Intersection;
    let tol = Tolerance::default();

    let mut a = a.clone();
    let mut b = b.clone();
    let intersections = face_intersections(&a, &b, &tol);
    println!("intersections: {}", intersections.len());
    for (i, inter) in intersections.iter().enumerate() {
        println!(
            "  {i}: face_a={:?} face_b={:?} {:?} -> {:?}",
            inter.face_a, inter.face_b, inter.start, inter.end
        );
    }

    let outcome = split_solids_at_intersections(&mut a, &mut b, &intersections, &tol);
    let interior = resolve_interior_endpoints(&mut a, &mut b, &intersections, &outcome, &tol);
    let skip_chord = classify_chord_interiorness(&a, &b, &intersections, op, &tol);
    println!("\nskip flags: {:?}", skip_chord);
    let _added = add_intersection_edges(&mut a, &mut b, &intersections, &interior, &skip_chord, &tol);

    println!("\nAfter splits: A has {} faces, B has {} faces", a.face_count(), b.face_count());

    println!("\nA face classifications:");
    for face_id in a.topo.face_ids() {
        let cls = classify_face(&a, face_id, &b, &tol);
        let kept = keep_a_face(cls, op);
        let poly = kerf_brep::booleans::face_polygon::face_polygon(&a, face_id).unwrap_or_default();
        let poly_str: Vec<String> = poly
            .iter()
            .map(|p| format!("({:.2},{:.2},{:.2})", p.x, p.y, p.z))
            .collect();
        println!(
            "  {face_id:?}: {cls:?} → {} [{}]",
            if kept { "KEEP" } else { "drop" },
            poly_str.join(", ")
        );
    }

    println!("\nB face classifications:");
    for face_id in b.topo.face_ids() {
        let cls = classify_face(&b, face_id, &a, &tol);
        let kept = keep_b_face(cls, op);
        let poly = kerf_brep::booleans::face_polygon::face_polygon(&b, face_id).unwrap_or_default();
        let centroid = kerf_brep::booleans::classify::face_centroid(&b, face_id);
        let poly_str: Vec<String> = poly
            .iter()
            .map(|p| format!("({:.2},{:.2},{:.2})", p.x, p.y, p.z))
            .collect();
        println!(
            "  {face_id:?}: {cls:?} centroid={centroid:?} → {} [{}]",
            if kept { "KEEP" } else { "drop" },
            poly_str.join(", ")
        );
    }
}
