//! M40 debug: replicate boolean_solid pipeline up to (but excluding) stitch,
//! then dump the kept polygons in detail to see why edges become orphans.

use std::collections::HashMap;

use kerf_brep::booleans::{
    BooleanOp, FaceClassification, add_intersection_edges, classify_chord_interiorness,
    classify_face, face_intersections, face_polygon, flip_b_face, keep_a_face, keep_b_face,
    resolve_interior_endpoints, split_solids_at_intersections,
};
use kerf_brep::booleans::stitch::KeptFace;
use kerf_brep::primitives::{box_, box_at, cylinder_faceted, extrude_polygon};
use kerf_geom::{Point3, Tolerance, Vec3};

fn dump(a_name: &str, b_name: &str, op: BooleanOp, a: &kerf_brep::Solid, b: &kerf_brep::Solid) {
    println!("\n========== {a_name} {op:?} {b_name} ==========");
    let tol = Tolerance::default();
    let mut a = a.clone();
    let mut b = b.clone();
    let intersections = face_intersections(&a, &b, &tol);
    let outcome = split_solids_at_intersections(&mut a, &mut b, &intersections, &tol);
    let interior = resolve_interior_endpoints(&mut a, &mut b, &intersections, &outcome, &tol);
    let skip_chord = classify_chord_interiorness(&a, &b, &intersections, op, &tol);
    let _added =
        add_intersection_edges(&mut a, &mut b, &intersections, &interior, &skip_chord, &tol);

    let mut kept: Vec<KeptFace> = Vec::new();
    let mut kept_origin: Vec<String> = Vec::new();
    use slotmap::Key as _;
    for face_id in a.topo.face_ids() {
        let cls = classify_face(&a, face_id, &b, &tol);
        let Some(polygon) = face_polygon(&a, face_id) else {
            continue;
        };
        let surface = a.face_geom.get(face_id).cloned().unwrap();
        if keep_a_face(cls, op) {
            kept_origin.push(format!("A:{face_id:?}:{cls:?}"));
            kept.push(KeptFace { polygon, surface });
        }
    }
    let flip_b = flip_b_face(op);
    for face_id in b.topo.face_ids() {
        let cls = classify_face(&b, face_id, &a, &tol);
        let Some(mut polygon) = face_polygon(&b, face_id) else {
            continue;
        };
        let surface = b.face_geom.get(face_id).cloned().unwrap();
        let face_kept = keep_b_face(cls, op);
        if face_kept {
            if flip_b {
                polygon.reverse();
            }
            kept_origin.push(format!("B:{face_id:?}:{cls:?}"));
            kept.push(KeptFace { polygon, surface });
        }
    }

    // Dedup vertices by 3D position (mimic stitch).
    let mut positions: Vec<Point3> = Vec::new();
    let mut find_or_add = |p: Point3| -> usize {
        for (i, q) in positions.iter().enumerate() {
            if (p - *q).norm() < tol.point_eq {
                return i;
            }
        }
        positions.push(p);
        positions.len() - 1
    };
    let face_to_vidx: Vec<Vec<usize>> = kept
        .iter()
        .map(|kf| kf.polygon.iter().map(|&p| find_or_add(p)).collect())
        .collect();

    // Compute edge keys.
    let mut edge_dirs: HashMap<(usize, usize), Vec<(usize, bool)>> = HashMap::new();
    for (face_idx, vidx) in face_to_vidx.iter().enumerate() {
        let n = vidx.len();
        for i in 0..n {
            let a = vidx[i];
            let b = vidx[(i + 1) % n];
            if a == b {
                continue;
            }
            let key = if a < b { (a, b) } else { (b, a) };
            let forward = (a, b) == key;
            edge_dirs.entry(key).or_default().push((face_idx, forward));
        }
    }
    println!("Total kept faces: {}, total positions: {}", kept.len(), positions.len());
    let mut bad_keys: Vec<&(usize, usize)> = Vec::new();
    for (key, entries) in &edge_dirs {
        let total = entries.len();
        let fwd = entries.iter().filter(|(_, fw)| *fw).count();
        let bwd = total - fwd;
        if !(total == 2 && fwd == 1 && bwd == 1) {
            bad_keys.push(key);
            let p0 = positions[key.0];
            let p1 = positions[key.1];
            print!(
                "EDGE {key:?} pos=({:.2},{:.2},{:.2})→({:.2},{:.2},{:.2}) ",
                p0.x, p0.y, p0.z, p1.x, p1.y, p1.z,
            );
            print!("count={total} fwd={fwd} bwd={bwd} faces=[");
            for (i, (f, fw)) in entries.iter().enumerate() {
                if i > 0 {
                    print!(", ");
                }
                print!("{}({})", kept_origin[*f], if *fw { "fwd" } else { "bwd" });
            }
            println!("]");
        }
    }
    println!("Bad edge keys: {}", bad_keys.len());
    let one_he: Vec<_> = edge_dirs.iter().filter(|(_, e)| e.len() == 1).collect();
    println!("Single-half-edge edges: {}", one_he.len());
}

fn main() {
    let box2 = box_(Vec3::new(2.0, 2.0, 2.0));
    let cyl12 = cylinder_faceted(0.6, 3.0, 12);
    let tri = extrude_polygon(
        &[
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(2.0, 0.0, 0.0),
            Point3::new(1.0, 1.5, 0.0),
        ],
        Vec3::new(0.0, 0.0, 2.0),
    );
    let box_nested = box_at(Vec3::new(0.6, 0.6, 0.6), Point3::new(0.7, 0.7, 0.7));
    let box_shift = box_at(Vec3::new(2.0, 2.0, 2.0), Point3::new(1.0, 0.0, 0.0));

    dump("box[2³]", "tri-prism", BooleanOp::Difference, &box2, &tri);
    dump("box-nested", "tri-prism", BooleanOp::Intersection, &box_nested, &tri);
    dump("cyl_n12", "tri-prism", BooleanOp::Union, &cyl12, &tri);
}
