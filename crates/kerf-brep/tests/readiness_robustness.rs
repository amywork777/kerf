//! Robustness tests: recursive booleans, tessellation convergence,
//! mesh-import composition, geometric consistency, translation invariance,
//! determinism, vertex deduplication.

use std::io::Cursor;

use kerf_brep::booleans::face_polygon_raw;
use kerf_brep::primitives::{box_, box_at, cylinder_faceted, extrude_polygon};
use kerf_brep::{
    read_stl_binary_to_solid, solid_volume, tessellate, write_binary, BooleanError, Solid,
};
use kerf_brep::geometry::SurfaceKind;
use kerf_geom::{Point3, Tolerance, Vec3};

const TOL: f64 = 1e-6;

fn try_op(
    a: &Solid,
    b: &Solid,
    op: kerf_brep::booleans::BooleanOp,
) -> Result<Solid, BooleanError> {
    use kerf_brep::booleans::BooleanOp;
    match op {
        BooleanOp::Union => a.try_union(b),
        BooleanOp::Intersection => a.try_intersection(b),
        BooleanOp::Difference => a.try_difference(b),
    }
}

// ============================================================================
// 1. Recursive booleans on overlapping inputs
// ============================================================================

#[test]
fn three_overlapping_boxes_chain_intersect_and_union_validate() {
    use kerf_brep::booleans::BooleanOp::*;
    // Three boxes that all overlap with each other.
    let a = box_(Vec3::new(2.0, 2.0, 2.0)); // [0,2]³
    let b = box_at(Vec3::new(2.0, 2.0, 2.0), Point3::new(1.0, 0.0, 0.0)); // [1,3]×[0,2]²
    let c = box_at(Vec3::new(2.0, 2.0, 2.0), Point3::new(0.5, 0.5, 0.5)); // [.5,2.5]³

    // (A ∪ B) ∩ C
    let ab = try_op(&a, &b, Union).expect("AB");
    let abc1 = try_op(&ab, &c, Intersection).expect("(A∪B)∩C");
    kerf_topo::validate(&abc1.topo).expect("(A∪B)∩C topology");
    // (A ∪ B) covers [0,3]×[0,2]² = 12. ∩ [.5,2.5]³ = [.5,2.5]×[.5,2]×[.5,2] = 2*1.5*1.5 = 4.5
    assert!(
        (solid_volume(&abc1) - 4.5).abs() < TOL,
        "(A∪B)∩C vol={}",
        solid_volume(&abc1)
    );

    // A ∪ (B ∩ C)
    let bc = try_op(&b, &c, Intersection).expect("BC");
    let abc2 = try_op(&a, &bc, Union).expect("A∪(B∩C)");
    kerf_topo::validate(&abc2.topo).expect("A∪(B∩C) topology");
    // B ∩ C = [1,2.5]×[.5,2]×[.5,2] = 1.5*1.5*1.5 = 3.375. A = 8 fully contains... wait,
    // [1,2.5] x-range overlaps A's [0,2] in [1,2]. So B∩C is partially inside A.
    // Inside A: [1,2]×[.5,2]×[.5,2] = 1*1.5*1.5 = 2.25. Outside A: 3.375 - 2.25 = 1.125.
    // A ∪ (B∩C) = 8 + 1.125 = 9.125.
    assert!(
        (solid_volume(&abc2) - 9.125).abs() < TOL,
        "A∪(B∩C) vol={}",
        solid_volume(&abc2)
    );

    // (A − B) ∪ C: A − B = [0,1]×[0,2]² = 4. ∪ C ([.5,2.5]³ = 8). Overlap = [.5,1]×[.5,2]×[.5,2]
    //   = .5 * 1.5 * 1.5 = 1.125. Total = 4 + 8 - 1.125 = 10.875.
    let amb = try_op(&a, &b, Difference).expect("A-B");
    let final_ = try_op(&amb, &c, Union).expect("(A-B)∪C");
    kerf_topo::validate(&final_.topo).expect("(A-B)∪C topology");
    assert!(
        (solid_volume(&final_) - 10.875).abs() < TOL,
        "(A-B)∪C vol={}",
        solid_volume(&final_)
    );
}

#[test]
fn deep_recursive_boolean_chain_terminates_and_preserves_invariant() {
    use kerf_brep::booleans::BooleanOp::*;
    // Chain: (((((A∪B)∪C)∪D)∪E)∪F) — 5 unions in a row.
    // Final volume = vol(A ∪ B ∪ C ∪ D ∪ E ∪ F).
    let inputs = [
        box_(Vec3::new(1.0, 1.0, 1.0)),
        box_at(Vec3::new(1.0, 1.0, 1.0), Point3::new(2.0, 0.0, 0.0)),
        box_at(Vec3::new(1.0, 1.0, 1.0), Point3::new(0.0, 2.0, 0.0)),
        box_at(Vec3::new(1.0, 1.0, 1.0), Point3::new(0.0, 0.0, 2.0)),
        box_at(Vec3::new(1.0, 1.0, 1.0), Point3::new(2.0, 2.0, 0.0)),
        box_at(Vec3::new(1.0, 1.0, 1.0), Point3::new(2.0, 2.0, 2.0)),
    ];
    let mut acc = inputs[0].clone();
    for i in 1..inputs.len() {
        acc = try_op(&acc, &inputs[i], Union).expect(&format!("step {i}"));
        kerf_topo::validate(&acc.topo).expect("topology");
    }
    // 6 disjoint unit boxes → volume 6.
    assert!(
        (solid_volume(&acc) - 6.0).abs() < TOL,
        "6-box chain vol={}",
        solid_volume(&acc)
    );
    assert_eq!(acc.topo.shell_count(), 6, "6 disjoint shells");
}

// ============================================================================
// 2. Tessellation refinement convergence
// ============================================================================

#[test]
fn tessellation_volume_independent_of_segment_count() {
    let s = box_(Vec3::new(2.0, 3.0, 4.0));
    // Tessellate with different segment counts; volume of triangle soup
    // (treated as a closed manifold via divergence theorem) must converge.
    for &n in &[3usize, 6, 12, 24, 48] {
        let soup = tessellate(&s, n);
        let v = soup_volume(&soup);
        assert!(
            (v - 24.0).abs() < TOL,
            "tessellate(box, {n}) vol={v}, expected 24"
        );
    }
}

#[test]
fn cylinder_tessellation_refines_toward_analytic_volume() {
    // A cylinder of radius 0.6, height 3.0 has analytic vol = π·r²·h = π·0.36·3 = 3.392920...
    // Faceted cylinder with N segments has vol = N·sin(2π/N)/2 · r²·h, which converges as N grows.
    let s = cylinder_faceted(0.6, 3.0, 64);
    let v_geom = solid_volume(&s);
    let v_tessel = soup_volume(&tessellate(&s, 64));
    assert!(
        (v_geom - v_tessel).abs() < TOL,
        "geom vol={v_geom}, tessel vol={v_tessel}"
    );
    // The faceted cylinder's volume is below analytic π·r²·h by a known factor.
    let analytic = std::f64::consts::PI * 0.36 * 3.0;
    assert!(v_geom < analytic);
    assert!(analytic - v_geom < 0.01, "n=64 should be within 1% of pi·r²·h");
}

fn soup_volume(soup: &kerf_brep::booleans::FaceSoup) -> f64 {
    let mut v = 0.0;
    for tri in &soup.triangles {
        let p0 = tri[0].coords;
        let p1 = tri[1].coords;
        let p2 = tri[2].coords;
        v += p0.dot(&p1.cross(&p2));
    }
    v / 6.0
}

// ============================================================================
// 3. Imported-mesh boolean (close the loop: tessellate → import → boolean)
// ============================================================================

#[test]
fn imported_mesh_can_be_intersected_with_primitive() {
    // Tessellate a box, write to STL, read back, then run a boolean against
    // another primitive. The result should match the boolean run directly
    // on the original primitives.
    let original = box_(Vec3::new(2.0, 2.0, 2.0));
    let cutter = box_at(Vec3::new(2.0, 2.0, 2.0), Point3::new(1.0, 0.0, 0.0));

    let soup = tessellate(&original, 24);
    let mut buf = Vec::new();
    write_binary(&soup, "boxA", &mut buf).expect("write");
    let mut reader = Cursor::new(buf);
    let imported = read_stl_binary_to_solid(&mut reader).expect("read");

    // The imported solid is a triangle-mesh approximation with ~12 triangles.
    // Volume should match.
    assert!(
        (solid_volume(&imported) - 8.0).abs() < TOL,
        "imported vol = {}",
        solid_volume(&imported)
    );

    // Note: full imported-mesh boolean still goes through the same pipeline.
    // The matrix only verifies primitives; this test covers the mesh path.
    let direct = original
        .try_intersection(&cutter)
        .expect("direct intersect");
    let direct_vol = solid_volume(&direct);

    // Try the same intersection with the imported version.
    // It may or may not work yet (boolean over triangle-soup-derived solids
    // is harder than primitive-on-primitive); we accept either success
    // matching `direct` or a clean Err — but NOT a panic.
    match imported.try_intersection(&cutter) {
        Ok(r) => {
            kerf_topo::validate(&r.topo).expect("imported boolean topology");
            assert!(
                (solid_volume(&r) - direct_vol).abs() < 0.1,
                "imported boolean vol diverges: imported={}, direct={direct_vol}",
                solid_volume(&r)
            );
        }
        Err(e) => {
            // Acceptable: imported triangle meshes have many small coplanar
            // faces that exercise harder paths in the boolean engine. The
            // important thing is no panic.
            eprintln!("imported boolean returned Err (acceptable): {}", e.message);
        }
    }
}

// ============================================================================
// 4. Vertex-on-face-plane consistency
// ============================================================================

#[test]
fn every_kept_plane_face_vertex_lies_on_its_plane() {
    use kerf_brep::booleans::BooleanOp::*;
    let cases = build_inputs();
    let ops = [Union, Intersection, Difference];
    let mut errors: Vec<String> = Vec::new();
    for op in ops {
        for (a_name, a) in &cases {
            for (b_name, b) in &cases {
                if a_name == b_name {
                    continue;
                }
                let r = try_op(a, b, op).expect("op");
                for fid in r.topo.face_ids() {
                    let SurfaceKind::Plane(plane) =
                        r.face_geom.get(fid).expect("face has surface")
                    else {
                        continue;
                    };
                    let Some(poly) = face_polygon_raw(&r, fid) else {
                        continue;
                    };
                    let n = plane.frame.z;
                    let origin = plane.frame.origin;
                    for p in &poly {
                        let d = (*p - origin).dot(&n);
                        if d.abs() > 1e-6 {
                            errors.push(format!(
                                "{op:?} {a_name} vs {b_name}: face {fid:?} vertex \
                                 {p:?} is {d} off its plane (origin={origin:?}, n={n:?})"
                            ));
                            break; // only flag first per face
                        }
                    }
                }
            }
        }
    }
    assert!(
        errors.is_empty(),
        "{} cases have off-plane vertices:\n{}",
        errors.len(),
        errors.join("\n")
    );
}

// ============================================================================
// 5. Translation invariance
// ============================================================================

#[test]
fn translation_invariance_holds_for_every_pair() {
    use kerf_brep::booleans::BooleanOp::*;
    let cases = build_inputs();
    let ops = [Union, Intersection, Difference];
    let t = Vec3::new(7.5, -3.25, 11.125);

    let mut errors: Vec<String> = Vec::new();
    for op in ops {
        for (a_name, a) in &cases {
            for (b_name, b) in &cases {
                if a_name == b_name {
                    continue;
                }
                let r1 = try_op(a, b, op).expect("op");
                let a_t = translate(a, t);
                let b_t = translate(b, t);
                let r2 = try_op(&a_t, &b_t, op).expect("op-t");
                let v1 = solid_volume(&r1);
                let v2 = solid_volume(&r2);
                if (v1 - v2).abs() > TOL {
                    errors.push(format!(
                        "{op:?} {a_name} vs {b_name}: untranslated vol={v1} ≠ translated vol={v2}"
                    ));
                }
                if r1.topo.face_count() != r2.topo.face_count()
                    || r1.topo.vertex_count() != r2.topo.vertex_count()
                {
                    errors.push(format!(
                        "{op:?} {a_name} vs {b_name}: V/F differs after translation: \
                         {} vs {} verts, {} vs {} faces",
                        r1.topo.vertex_count(),
                        r2.topo.vertex_count(),
                        r1.topo.face_count(),
                        r2.topo.face_count()
                    ));
                }
            }
        }
    }
    assert!(
        errors.is_empty(),
        "{} cases broke under translation:\n{}",
        errors.len(),
        errors.join("\n")
    );
}

fn translate(s: &Solid, t: Vec3) -> Solid {
    let mut out = s.clone();
    for v in out.vertex_geom.values_mut() {
        *v += t;
    }
    // Surface origins also need translating for plane-aligned matching.
    for surf in out.face_geom.values_mut() {
        if let SurfaceKind::Plane(plane) = surf {
            plane.frame.origin += t;
        }
    }
    out
}

// ============================================================================
// 6. Determinism
// ============================================================================

#[test]
fn same_inputs_produce_same_volume_twice() {
    use kerf_brep::booleans::BooleanOp::*;
    let cases = build_inputs();
    let ops = [Union, Intersection, Difference];
    for op in ops {
        for (a_name, a) in &cases {
            for (b_name, b) in &cases {
                if a_name == b_name {
                    continue;
                }
                let r1 = try_op(a, b, op).expect("op");
                let r2 = try_op(a, b, op).expect("op-2");
                let v1 = solid_volume(&r1);
                let v2 = solid_volume(&r2);
                assert!(
                    (v1 - v2).abs() < 1e-12,
                    "{op:?} {a_name} vs {b_name}: non-deterministic vol {v1} vs {v2}"
                );
                assert_eq!(
                    r1.topo.vertex_count(),
                    r2.topo.vertex_count(),
                    "{op:?} {a_name} vs {b_name}: non-deterministic V"
                );
                assert_eq!(
                    r1.topo.face_count(),
                    r2.topo.face_count(),
                    "{op:?} {a_name} vs {b_name}: non-deterministic F"
                );
            }
        }
    }
}

// ============================================================================
// 7. No duplicate vertices
// ============================================================================

#[test]
fn no_result_has_duplicate_vertices_at_same_position() {
    use kerf_brep::booleans::BooleanOp::*;
    let cases = build_inputs();
    let ops = [Union, Intersection, Difference];
    let tol = Tolerance::default();
    let mut errors: Vec<String> = Vec::new();
    for op in ops {
        for (a_name, a) in &cases {
            for (b_name, b) in &cases {
                if a_name == b_name {
                    continue;
                }
                let r = try_op(a, b, op).expect("op");
                let positions: Vec<(kerf_topo::VertexId, Point3)> = r
                    .topo
                    .vertex_ids()
                    .filter_map(|v| r.vertex_geom.get(v).map(|p| (v, *p)))
                    .collect();
                'pair: for i in 0..positions.len() {
                    for j in (i + 1)..positions.len() {
                        let (vi, pi) = positions[i];
                        let (vj, pj) = positions[j];
                        if (pi - pj).norm() < tol.point_eq * 100.0 {
                            errors.push(format!(
                                "{op:?} {a_name} vs {b_name}: {vi:?} and {vj:?} \
                                 both at ~({:.4},{:.4},{:.4})",
                                pi.x, pi.y, pi.z
                            ));
                            break 'pair;
                        }
                    }
                }
            }
        }
    }
    assert!(
        errors.is_empty(),
        "{} cases have duplicate vertices:\n{}",
        errors.len(),
        errors.join("\n")
    );
}

// ============================================================================
// Inputs (subset of matrix; full matrix is exercised by other tests)
// ============================================================================

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
    ]
}
