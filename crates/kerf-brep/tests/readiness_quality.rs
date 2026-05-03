//! Geometric quality checks for boolean results.
//!
//! - **Bounding box containment**: result vertices stay within the union of
//!   input bounding boxes (with tolerance for jittered tier-3 retries).
//! - **No degenerate triangles**: every face fan-triangulates to triangles
//!   with non-trivial area (catches stitch artifacts: collinear vertex
//!   strings that survived dedup, T-junction healing failures, etc.).

use kerf_brep::primitives::{
    box_, box_at, cylinder_faceted, extrude_polygon, revolve_polyline,
};
use kerf_brep::Solid;
use kerf_geom::{Point3, Vec3};

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Op {
    Union,
    Intersection,
    Difference,
}

fn run(a: &Solid, b: &Solid, op: Op) -> Solid {
    match op {
        Op::Union => a.try_union(b),
        Op::Intersection => a.try_intersection(b),
        Op::Difference => a.try_difference(b),
    }
    .unwrap_or_else(|e| panic!("{op:?} failed: {}", e.message))
}

fn aabb(s: &Solid) -> (Point3, Point3) {
    if s.vertex_count() == 0 {
        return (Point3::origin(), Point3::origin());
    }
    let mut min = Point3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
    let mut max = Point3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
    for vid in s.topo.vertex_ids() {
        if let Some(p) = s.vertex_geom.get(vid) {
            min.x = min.x.min(p.x);
            min.y = min.y.min(p.y);
            min.z = min.z.min(p.z);
            max.x = max.x.max(p.x);
            max.y = max.y.max(p.y);
            max.z = max.z.max(p.z);
        }
    }
    (min, max)
}

#[test]
fn every_result_aabb_contained_in_union_of_input_aabbs() {
    // Allow tier-3 jitter slop (~1e-5) plus tolerance for rounding.
    const SLACK: f64 = 1e-4;
    let cases = build_inputs();
    let ops = [Op::Union, Op::Intersection, Op::Difference];
    let mut errors: Vec<String> = Vec::new();
    for op in ops {
        for (a_name, a) in &cases {
            for (b_name, b) in &cases {
                if a_name == b_name {
                    continue;
                }
                let r = run(a, b, op);
                if r.vertex_count() == 0 {
                    continue; // empty result = nothing to check
                }
                let (a_min, a_max) = aabb(a);
                let (b_min, b_max) = aabb(b);
                let bound_min = Point3::new(
                    a_min.x.min(b_min.x) - SLACK,
                    a_min.y.min(b_min.y) - SLACK,
                    a_min.z.min(b_min.z) - SLACK,
                );
                let bound_max = Point3::new(
                    a_max.x.max(b_max.x) + SLACK,
                    a_max.y.max(b_max.y) + SLACK,
                    a_max.z.max(b_max.z) + SLACK,
                );
                let (r_min, r_max) = aabb(&r);
                if r_min.x < bound_min.x
                    || r_min.y < bound_min.y
                    || r_min.z < bound_min.z
                    || r_max.x > bound_max.x
                    || r_max.y > bound_max.y
                    || r_max.z > bound_max.z
                {
                    errors.push(format!(
                        "{op:?} {a_name} vs {b_name}: result aabb [{r_min:?}, {r_max:?}] \
                         exceeds input bound [{bound_min:?}, {bound_max:?}]"
                    ));
                }
            }
        }
    }
    assert!(
        errors.is_empty(),
        "{} of 168 cases have stray vertices outside input bounds:\n{}",
        errors.len(),
        errors.join("\n")
    );
}

#[test]
fn no_face_has_a_zero_area_triangle_in_its_fan_triangulation() {
    // Tolerance: areas below 1e-10 are flagged. This catches collinear
    // vertex strings and T-junction artifacts that pass topology validation
    // but produce visually invisible / numerically unstable triangles.
    //
    // Skip vase-involved cases — vase is a degenerate primitive (faces are
    // 360° lunes without seam vertices, so they're inherently zero-area
    // collinear strings). All vase failures are "expected" degeneracy from
    // the input, not bugs.
    const AREA_TOL: f64 = 1e-10;
    let cases = build_inputs();
    let ops = [Op::Union, Op::Intersection, Op::Difference];
    let mut errors: Vec<String> = Vec::new();
    for op in ops {
        for (a_name, a) in &cases {
            if *a_name == "vase" {
                continue;
            }
            for (b_name, b) in &cases {
                if a_name == b_name || *b_name == "vase" {
                    continue;
                }
                let r = run(a, b, op);
                for fid in r.topo.face_ids() {
                    let Some(poly) =
                        kerf_brep::booleans::face_polygon(&r, fid)
                    else {
                        continue;
                    };
                    if poly.len() < 3 {
                        continue;
                    }
                    // Fan-triangulate from poly[0]; check each triangle.
                    for i in 1..poly.len() - 1 {
                        let a_v = poly[0].coords;
                        let b_v = poly[i].coords;
                        let c_v = poly[i + 1].coords;
                        let area = 0.5 * (b_v - a_v).cross(&(c_v - a_v)).norm();
                        if area < AREA_TOL {
                            errors.push(format!(
                                "{op:?} {a_name} vs {b_name}: face {fid:?} fan tri \
                                 [{i}] area={area} below tol (poly={poly:?})"
                            ));
                            break;
                        }
                    }
                }
            }
        }
    }
    assert!(
        errors.is_empty(),
        "{} non-vase cases have degenerate triangles:\n{}",
        errors.len(),
        errors.join("\n")
    );
}

fn build_inputs() -> Vec<(&'static str, Solid)> {
    vec![
        ("box[2³]", box_(Vec3::new(2.0, 2.0, 2.0))),
        (
            "box-shift",
            box_at(Vec3::new(2.0, 2.0, 2.0), Point3::new(1.0, 0.0, 0.0)),
        ),
        (
            "box-corner",
            box_at(Vec3::new(2.0, 2.0, 2.0), Point3::new(1.0, 1.0, 1.0)),
        ),
        (
            "box-nested",
            box_at(Vec3::new(0.6, 0.6, 0.6), Point3::new(0.7, 0.7, 0.7)),
        ),
        (
            "tri-prism",
            extrude_polygon(
                &[
                    Point3::new(0.0, 0.0, 0.0),
                    Point3::new(2.0, 0.0, 0.0),
                    Point3::new(1.0, 1.5, 0.0),
                ],
                Vec3::new(0.0, 0.0, 2.0),
            ),
        ),
        ("cyl_n12", cylinder_faceted(0.6, 3.0, 12)),
        ("cyl_n4", cylinder_faceted(0.7, 2.0, 4)),
        (
            "vase",
            revolve_polyline(&[
                Point3::new(0.0, 0.0, 2.0),
                Point3::new(0.5, 0.0, 1.5),
                Point3::new(0.7, 0.0, 0.5),
                Point3::new(0.0, 0.0, 0.0),
            ]),
        ),
    ]
}
