//! M40 debug: trace cylinder-pierces-box face split. See what topology Phase C
//! produces and where the stitch failure originates.

use kerf_brep::booleans::{
    BooleanOp, add_intersection_edges, classify_face, face_intersections, face_polygon,
    keep_a_face, keep_b_face, resolve_interior_endpoints, split_solids_at_intersections,
};
use kerf_brep::primitives::{box_, cylinder_faceted};
use kerf_geom::{Point3, Tolerance, Vec3};

fn main() {
    let a = box_(Vec3::new(2.0, 2.0, 2.0));
    let b = cylinder_faceted(0.6, 3.0, 12);
    let op = BooleanOp::Union;
    let tol = Tolerance::default();

    let mut a = a.clone();
    let mut b = b.clone();
    let intersections = face_intersections(&a, &b, &tol);
    println!("Total intersections: {}", intersections.len());
    let outcome = split_solids_at_intersections(&mut a, &mut b, &intersections, &tol);
    let interior = resolve_interior_endpoints(&mut a, &mut b, &intersections, &outcome, &tol);
    let added = add_intersection_edges(&mut a, &mut b, &intersections, &interior, &[], &tol);
    println!("Phase C added {} chords", added.len());
    println!(
        "chord_already_added_a count: {}",
        interior.chord_already_added_a.iter().filter(|x| **x).count()
    );
    println!(
        "chord_already_added_b count: {}",
        interior.chord_already_added_b.iter().filter(|x| **x).count()
    );
    println!(
        "After splits: A has {}V/{}E/{}F, B has {}V/{}E/{}F",
        a.vertex_count(),
        a.edge_count(),
        a.face_count(),
        b.vertex_count(),
        b.edge_count(),
        b.face_count(),
    );

    println!("\nA faces:");
    for face_id in a.topo.face_ids() {
        let cls = classify_face(&a, face_id, &b, &tol);
        let kept = keep_a_face(cls, op);
        let poly = face_polygon(&a, face_id).unwrap_or_default();
        let centroid: Point3 = if poly.is_empty() {
            Point3::origin()
        } else {
            let mut c = Point3::origin();
            for p in &poly {
                c += p.coords / poly.len() as f64;
            }
            c
        };
        println!(
            "  {face_id:?} cls={cls:?} kept={kept} centroid=({:.2},{:.2},{:.2}) verts={}",
            centroid.x, centroid.y, centroid.z, poly.len()
        );
    }
    println!("\nB faces:");
    for face_id in b.topo.face_ids() {
        let cls = classify_face(&b, face_id, &a, &tol);
        let kept = keep_b_face(cls, op);
        let poly = face_polygon(&b, face_id).unwrap_or_default();
        let centroid: Point3 = if poly.is_empty() {
            Point3::origin()
        } else {
            let mut c = Point3::origin();
            for p in &poly {
                c += p.coords / poly.len() as f64;
            }
            c
        };
        println!(
            "  {face_id:?} cls={cls:?} kept={kept} centroid=({:.2},{:.2},{:.2}) verts={}",
            centroid.x, centroid.y, centroid.z, poly.len()
        );
    }
}
