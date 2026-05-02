//! Trace phase B mev calls for box-nested + box-shift to see why it produces 3 faces.

use kerf_brep::booleans::{
    add_intersection_edges, classify_chord_interiorness, face_intersections,
    resolve_interior_endpoints, split_solids_at_intersections, BooleanOp,
};
use kerf_brep::primitives::{box_at};
use kerf_geom::{Point3, Tolerance, Vec3};

fn main() {
    let mut a = box_at(Vec3::new(0.6, 0.6, 0.6), Point3::new(0.7, 0.7, 0.7));
    let mut b = kerf_brep::primitives::extrude_polygon(
        &[
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(2.0, 0.0, 0.0),
            Point3::new(1.0, 1.5, 0.0),
        ],
        Vec3::new(0.0, 0.0, 2.0),
    );
    let op = BooleanOp::Intersection;
    let tol = Tolerance::default();

    let face_b_x1 = b.topo.face_ids().find(|&fid| {
        let poly = kerf_brep::booleans::face_polygon(&b, fid).unwrap();
        poly.iter().all(|p| (p.x - 1.0).abs() < 1e-6)
    });
    println!("box-shift x=1 face: {:?}", face_b_x1);
    println!("Before: B has {}V/{}E/{}F", b.vertex_count(), b.edge_count(), b.face_count());

    let intersections = face_intersections(&a, &b, &tol);
    println!("Intersections: {}", intersections.len());
    for (i, inter) in intersections.iter().enumerate() {
        println!(
            "  #{i}: face_a={:?} face_b={:?} start=({:.2},{:.2},{:.2}) end=({:.2},{:.2},{:.2})",
            inter.face_a, inter.face_b, inter.start.x, inter.start.y, inter.start.z,
            inter.end.x, inter.end.y, inter.end.z,
        );
    }

    let outcome = split_solids_at_intersections(&mut a, &mut b, &intersections, &tol);
    println!("After phase A: A has {}V/{}E/{}F, B has {}V/{}E/{}F",
        a.vertex_count(), a.edge_count(), a.face_count(),
        b.vertex_count(), b.edge_count(), b.face_count());
    let resolved_count = outcome.endpoints.iter().filter(|x| x.is_some()).count();
    println!("Phase A resolved: {}/{}", resolved_count, intersections.len());

    let interior = resolve_interior_endpoints(&mut a, &mut b, &intersections, &outcome, &tol);
    println!("After phase B: A has {}V/{}E/{}F, B has {}V/{}E/{}F",
        a.vertex_count(), a.edge_count(), a.face_count(),
        b.vertex_count(), b.edge_count(), b.face_count());
    let added_a = interior.chord_already_added_a.iter().filter(|x| **x).count();
    let added_b = interior.chord_already_added_b.iter().filter(|x| **x).count();
    println!("chord_already_added: A={}, B={}", added_a, added_b);
    println!("  A flags: {:?}", interior.chord_already_added_a);
    println!("  B flags: {:?}", interior.chord_already_added_b);

    let skip_chord = classify_chord_interiorness(&a, &b, &intersections, op, &tol);
    let added = add_intersection_edges(&mut a, &mut b, &intersections, &interior, &skip_chord, &tol);
    println!("Phase C added {} chords", added.len());
    println!("After phase C: A has {}V/{}E/{}F, B has {}V/{}E/{}F",
        a.vertex_count(), a.edge_count(), a.face_count(),
        b.vertex_count(), b.edge_count(), b.face_count());

    if let Some(fid) = face_b_x1 {
        // Check if face still exists; original face_b may have been split.
        if b.topo.face(fid).is_some() {
            let poly = kerf_brep::booleans::face_polygon(&b, fid).unwrap();
            println!("Original x=1 face still has {} vertices", poly.len());
        }
    }
    // List ALL kept faces with classifications + polygons:
    use kerf_brep::booleans::{classify_face, keep_a_face, keep_b_face};
    println!("\nA all faces (kept+dropped):");
    for fid in a.topo.face_ids() {
        let cls = classify_face(&a, fid, &b, &tol);
        let kept = keep_a_face(cls, op);
        let Some(poly) = kerf_brep::booleans::face_polygon(&a, fid) else { continue };
        let centroid = kerf_brep::booleans::face_centroid(&a, fid).unwrap_or(Point3::origin());
        print!("  {fid:?} cls={cls:?} kept={kept} centroid=({:.2},{:.2},{:.2}) verts={}: ",
            centroid.x, centroid.y, centroid.z, poly.len());
        for p in &poly {
            print!("({:.2},{:.2},{:.2}) ", p.x, p.y, p.z);
        }
        println!();
    }
    println!("\nB kept faces with polygons (PRE-FLIP):");
    for fid in b.topo.face_ids() {
        let cls = classify_face(&b, fid, &a, &tol);
        let kept = keep_b_face(cls, op);
        if !kept { continue; }
        let Some(poly) = kerf_brep::booleans::face_polygon(&b, fid) else { continue };
        print!("  {fid:?}: ");
        for p in &poly {
            print!("({:.2},{:.2},{:.2}) ", p.x, p.y, p.z);
        }
        println!();
    }
}
