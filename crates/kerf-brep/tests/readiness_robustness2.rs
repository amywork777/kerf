//! Second-tier robustness: rotation invariance, scale extremes, coplanar
//! touching, performance budget, JSON round-trip, random fuzz.

use std::io::Cursor;
use std::time::Instant;

use kerf_brep::booleans::BooleanOp;
use kerf_brep::geometry::SurfaceKind;
use kerf_brep::primitives::{box_, box_at};
use kerf_brep::{read_json, solid_volume, write_json, Solid};
use kerf_geom::{Frame, Plane, Point3, Vec3};
use nalgebra::{Rotation3, Unit};

const TOL: f64 = 1e-6;

fn run(a: &Solid, b: &Solid, op: BooleanOp) -> Solid {
    match op {
        BooleanOp::Union => a.try_union(b),
        BooleanOp::Intersection => a.try_intersection(b),
        BooleanOp::Difference => a.try_difference(b),
    }
    .unwrap_or_else(|e| panic!("{op:?} failed: {}", e.message))
}

// ============================================================================
// Rotation invariance
// ============================================================================

/// Rotate every vertex, every surface frame, and every edge frame by R.
fn rotate_solid(s: &Solid, r: &Rotation3<f64>) -> Solid {
    let mut out = s.clone();
    for v in out.vertex_geom.values_mut() {
        *v = Point3::from(r * v.coords);
    }
    for surf in out.face_geom.values_mut() {
        if let SurfaceKind::Plane(plane) = surf {
            let new_origin = Point3::from(r * plane.frame.origin.coords);
            let new_x = r * plane.frame.x;
            let new_y_hint = r * plane.frame.y;
            let new_frame = Frame::from_x_yhint(new_origin, new_x, new_y_hint)
                .expect("rotated frame non-degenerate");
            *plane = Plane::new(new_frame);
        }
    }
    out
}

#[test]
fn rotation_invariance_box_union_box_shift() {
    let a = box_(Vec3::new(2.0, 2.0, 2.0));
    let b = box_at(Vec3::new(2.0, 2.0, 2.0), Point3::new(1.0, 0.0, 0.0));

    let r_orig = run(&a, &b, BooleanOp::Union);
    let v_orig = solid_volume(&r_orig);

    // Three rotations: 30° about each axis, then a non-axis-aligned 45° about (1,1,1).
    let rots = [
        Rotation3::from_axis_angle(&Vec3::x_axis(), std::f64::consts::FRAC_PI_6),
        Rotation3::from_axis_angle(&Vec3::y_axis(), std::f64::consts::FRAC_PI_4),
        Rotation3::from_axis_angle(&Vec3::z_axis(), std::f64::consts::FRAC_PI_3),
        Rotation3::from_axis_angle(
            &Unit::new_normalize(Vec3::new(1.0, 1.0, 1.0)),
            std::f64::consts::FRAC_PI_4,
        ),
    ];

    for (i, r) in rots.iter().enumerate() {
        let a_r = rotate_solid(&a, r);
        let b_r = rotate_solid(&b, r);
        let r_rot = run(&a_r, &b_r, BooleanOp::Union);
        let v_rot = solid_volume(&r_rot);
        assert!(
            (v_orig - v_rot).abs() < TOL,
            "rotation #{i}: orig vol {v_orig} ≠ rotated vol {v_rot}"
        );
        assert_eq!(
            r_orig.topo.face_count(),
            r_rot.topo.face_count(),
            "rotation #{i}: face count drift"
        );
        assert_eq!(
            r_orig.topo.vertex_count(),
            r_rot.topo.vertex_count(),
            "rotation #{i}: vertex count drift"
        );
    }
}

#[test]
fn rotation_invariance_for_diff_with_cavity() {
    let big = box_(Vec3::new(10.0, 10.0, 10.0));
    let small = box_at(Vec3::new(2.0, 2.0, 2.0), Point3::new(4.0, 4.0, 4.0));
    let r_orig = run(&big, &small, BooleanOp::Difference);
    let v_orig = solid_volume(&r_orig);
    let s_orig = r_orig.topo.shell_count();

    let rot = Rotation3::from_axis_angle(
        &Unit::new_normalize(Vec3::new(0.7, -0.3, 0.5)),
        1.234,
    );
    let big_r = rotate_solid(&big, &rot);
    let small_r = rotate_solid(&small, &rot);
    let r_rot = run(&big_r, &small_r, BooleanOp::Difference);
    let v_rot = solid_volume(&r_rot);
    let s_rot = r_rot.topo.shell_count();

    assert!((v_orig - v_rot).abs() < TOL);
    assert_eq!(s_orig, s_rot, "cavity shell count drift under rotation");
}

// ============================================================================
// Scale extremes
// ============================================================================

#[test]
fn boolean_works_at_huge_scale() {
    // 1000× scale of the canonical 2x2x2 + box-shift case.
    let a = box_(Vec3::new(2000.0, 2000.0, 2000.0));
    let b = box_at(Vec3::new(2000.0, 2000.0, 2000.0), Point3::new(1000.0, 0.0, 0.0));
    let r = run(&a, &b, BooleanOp::Union);
    // Expected: vol(A) + vol(B) - overlap = 2 * 2000³ - 1000*2000² = 16e9 - 4e9 = 12e9
    let expected = 2.0f64 * 2000.0_f64.powi(3) - 1000.0_f64 * 2000.0_f64.powi(2);
    assert!(
        (solid_volume(&r) - expected).abs() < expected * 1e-6,
        "huge-scale union vol drift"
    );
}

#[test]
fn boolean_works_at_small_scale() {
    // 1/1000 scale.
    let a = box_(Vec3::new(0.002, 0.002, 0.002));
    let b = box_at(Vec3::new(0.002, 0.002, 0.002), Point3::new(0.001, 0.0, 0.0));
    let r = run(&a, &b, BooleanOp::Union);
    let expected = 2.0 * 0.002_f64.powi(3) - 0.001 * 0.002 * 0.002;
    let abs_err = (solid_volume(&r) - expected).abs();
    assert!(
        abs_err < 1e-15,
        "small-scale union vol drift: got {}, expected {expected}, err {abs_err}",
        solid_volume(&r)
    );
}

#[test]
fn high_aspect_ratio_box_boolean_works() {
    // Long thin box.
    let a = box_(Vec3::new(100.0, 0.5, 0.5));
    let b = box_at(Vec3::new(100.0, 0.5, 0.5), Point3::new(50.0, 0.0, 0.0));
    let r = run(&a, &b, BooleanOp::Union);
    // Overlap = 50 * 0.5 * 0.5 = 12.5. vol(A) = 25. vol(B) = 25. Union = 25 + 25 - 12.5 = 37.5
    assert!((solid_volume(&r) - 37.5).abs() < TOL);
}

// ============================================================================
// Coplanar touching (boxes sharing a face but not overlapping in volume)
// ============================================================================

#[test]
fn coplanar_touching_boxes_union_is_sum() {
    // [0,2]³ and [2,4]×[0,2]² share the x=2 face but don't overlap.
    let a = box_(Vec3::new(2.0, 2.0, 2.0));
    let b = box_at(Vec3::new(2.0, 2.0, 2.0), Point3::new(2.0, 0.0, 0.0));
    let r = run(&a, &b, BooleanOp::Union);
    // vol = 8 + 8 = 16.
    assert!(
        (solid_volume(&r) - 16.0).abs() < TOL,
        "coplanar-touching union vol={}",
        solid_volume(&r)
    );
}

#[test]
fn coplanar_touching_boxes_intersect_is_empty() {
    let a = box_(Vec3::new(2.0, 2.0, 2.0));
    let b = box_at(Vec3::new(2.0, 2.0, 2.0), Point3::new(2.0, 0.0, 0.0));
    let r = run(&a, &b, BooleanOp::Intersection);
    assert!(solid_volume(&r).abs() < TOL, "intersection should be empty");
}

#[test]
fn coplanar_touching_boxes_diff_is_a_unchanged() {
    let a = box_(Vec3::new(2.0, 2.0, 2.0));
    let b = box_at(Vec3::new(2.0, 2.0, 2.0), Point3::new(2.0, 0.0, 0.0));
    let r = run(&a, &b, BooleanOp::Difference);
    assert!((solid_volume(&r) - 8.0).abs() < TOL);
}

// ============================================================================
// Performance budget
// ============================================================================

#[test]
fn every_matrix_case_completes_within_one_second() {
    use kerf_brep::primitives::{cylinder_faceted, extrude_polygon, revolve_polyline};
    let cases: Vec<(&str, Solid)> = vec![
        ("box[2³]", box_(Vec3::new(2.0, 2.0, 2.0))),
        ("box-shift", box_at(Vec3::new(2.0, 2.0, 2.0), Point3::new(1.0, 0.0, 0.0))),
        ("box-corner", box_at(Vec3::new(2.0, 2.0, 2.0), Point3::new(1.0, 1.0, 1.0))),
        ("box-nested", box_at(Vec3::new(0.6, 0.6, 0.6), Point3::new(0.7, 0.7, 0.7))),
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
    ];
    let ops = [BooleanOp::Union, BooleanOp::Intersection, BooleanOp::Difference];
    for op in ops {
        for (a_name, a) in &cases {
            for (b_name, b) in &cases {
                if a_name == b_name {
                    continue;
                }
                let t0 = Instant::now();
                let _ = run(a, b, op);
                let dt = t0.elapsed();
                assert!(
                    dt.as_secs_f64() < 1.0,
                    "{op:?} {a_name} vs {b_name} took {:?} (>1s)",
                    dt
                );
            }
        }
    }
}

// ============================================================================
// JSON round-trip
// ============================================================================

#[test]
fn json_roundtrip_preserves_box_topology_and_volume() {
    let s = box_(Vec3::new(2.0, 3.0, 4.0));
    let mut buf = Vec::new();
    write_json(&s, &mut buf).expect("write_json");
    let mut reader = Cursor::new(buf);
    let read_back = read_json(&mut reader).expect("read_json");
    assert_eq!(s.vertex_count(), read_back.vertex_count());
    assert_eq!(s.edge_count(), read_back.edge_count());
    assert_eq!(s.face_count(), read_back.face_count());
    assert!((solid_volume(&s) - solid_volume(&read_back)).abs() < TOL);
    kerf_topo::validate(&read_back.topo).expect("topology");
}

#[test]
fn json_roundtrip_preserves_boolean_result() {
    let a = box_(Vec3::new(2.0, 2.0, 2.0));
    let b = box_at(Vec3::new(2.0, 2.0, 2.0), Point3::new(1.0, 0.0, 0.0));
    let r = run(&a, &b, BooleanOp::Union);
    let mut buf = Vec::new();
    write_json(&r, &mut buf).expect("write_json");
    let mut reader = Cursor::new(buf);
    let read_back = read_json(&mut reader).expect("read_json");
    assert_eq!(r.vertex_count(), read_back.vertex_count());
    assert_eq!(r.edge_count(), read_back.edge_count());
    assert_eq!(r.face_count(), read_back.face_count());
    assert!((solid_volume(&r) - solid_volume(&read_back)).abs() < TOL);
    kerf_topo::validate(&read_back.topo).expect("topology");
}

// ============================================================================
// Random box-pair fuzz
// ============================================================================

#[test]
fn random_box_pair_fuzz_satisfies_inclusion_exclusion() {
    // Deterministic LCG so the test is reproducible without external rand.
    let mut state: u64 = 0xdeadbeef_cafebabe;
    let mut rand = || {
        state = state.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        ((state >> 33) as u32) as f64 / u32::MAX as f64
    };
    let mut errors: Vec<String> = Vec::new();
    let n_cases = 50;
    for case_idx in 0..n_cases {
        let extents_a = Vec3::new(
            0.5 + rand() * 2.0,
            0.5 + rand() * 2.0,
            0.5 + rand() * 2.0,
        );
        let origin_a = Point3::new(
            (rand() - 0.5) * 4.0,
            (rand() - 0.5) * 4.0,
            (rand() - 0.5) * 4.0,
        );
        let extents_b = Vec3::new(
            0.5 + rand() * 2.0,
            0.5 + rand() * 2.0,
            0.5 + rand() * 2.0,
        );
        let origin_b = Point3::new(
            origin_a.x + (rand() - 0.5) * 4.0,
            origin_a.y + (rand() - 0.5) * 4.0,
            origin_a.z + (rand() - 0.5) * 4.0,
        );
        let a = box_at(extents_a, origin_a);
        let b = box_at(extents_b, origin_b);

        // Analytic overlap.
        let a_max = origin_a + extents_a;
        let b_max = origin_b + extents_b;
        let overlap = Vec3::new(
            (a_max.x.min(b_max.x) - origin_a.x.max(origin_b.x)).max(0.0),
            (a_max.y.min(b_max.y) - origin_a.y.max(origin_b.y)).max(0.0),
            (a_max.z.min(b_max.z) - origin_a.z.max(origin_b.z)).max(0.0),
        );
        let vol_a = extents_a.x * extents_a.y * extents_a.z;
        let vol_b = extents_b.x * extents_b.y * extents_b.z;
        let vol_overlap = overlap.x * overlap.y * overlap.z;

        let try_check = |op: BooleanOp, expected: f64| -> Result<(), String> {
            match match op {
                BooleanOp::Union => a.try_union(&b),
                BooleanOp::Intersection => a.try_intersection(&b),
                BooleanOp::Difference => a.try_difference(&b),
            } {
                Ok(r) => {
                    let v = solid_volume(&r);
                    if (v - expected).abs() > 1e-6_f64.max(expected.abs() * 1e-4) {
                        Err(format!(
                            "case#{case_idx} {op:?}: got vol={v}, expected {expected}"
                        ))
                    } else {
                        Ok(())
                    }
                }
                Err(e) => Err(format!(
                    "case#{case_idx} {op:?}: {} (a@{origin_a:?} ext{extents_a:?}; b@{origin_b:?} ext{extents_b:?})",
                    e.message
                )),
            }
        };
        if let Err(e) = try_check(BooleanOp::Union, vol_a + vol_b - vol_overlap) {
            errors.push(e);
        }
        if let Err(e) = try_check(BooleanOp::Intersection, vol_overlap) {
            errors.push(e);
        }
        if let Err(e) = try_check(BooleanOp::Difference, vol_a - vol_overlap) {
            errors.push(e);
        }
    }
    assert!(
        errors.is_empty(),
        "{} of {} fuzz cases failed:\n{}",
        errors.len(),
        n_cases * 3,
        errors.join("\n")
    );
}
