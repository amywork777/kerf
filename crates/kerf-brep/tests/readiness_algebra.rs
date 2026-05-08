//! Boolean-algebra invariants the kernel must satisfy.
//!
//! readiness_geometry.rs verified individual results; this verifies the
//! ALGEBRAIC RELATIONSHIPS between results that hold for any valid CSG kernel:
//!
//! - **Idempotence**: A op A is a fixed point (or empty for `−`).
//! - **Commutativity**: A∪B = B∪A volumetrically; same for ∩.
//! - **Set-algebra conservation**: vol(A) = vol(A∩B) + vol(A−B).
//! - **Inclusion-exclusion**: vol(A∪B) = vol(A) + vol(B) − vol(A∩B).
//! - **Associativity**: (A∪B)∪C volumetrically equals A∪(B∪C) for disjoint C.
//!
//! These hold for ANY valid Boolean. Failure means the kernel produces a
//! result that's "valid" topologically but inconsistent with set theory.

use kerf_brep::primitives::{
    box_, box_at, cylinder_faceted, extrude_polygon, revolve_polyline,
};
use kerf_brep::{solid_volume, Solid};
use kerf_geom::{Point3, Vec3};

const TOL: f64 = 1e-6;

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

/// All primitives in the matrix EXCEPT `vase`. The vase is a 3-cone-face
/// revolved solid (V=4/E=5/F=3) with the analytic surfaces — its actual
/// volume is non-zero (~1.78), measurable via the analytic-surface
/// tessellation path in `solid_volume`. We exclude it from algebra-pair
/// tests because the boolean pipeline still produces a topologically
/// degenerate result for vase∪vase / vase∩vase (E grows from 5 to 7 but
/// the new edges aren't representable as circles, so per-face tessellation
/// returns 0 triangles). That's a separate kernel limitation tracked in
/// scenario_06_lofted_revolved_compound.
fn algebra_inputs() -> Vec<(&'static str, Solid)> {
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

/// All primitives including `vase`. Used for invariants that survive even
/// when the boolean output is degenerate (e.g., self_difference: vase∖vase
/// is empty regardless of how booleans handle revolved topology).
fn full_inputs() -> Vec<(&'static str, Solid)> {
    let mut v = algebra_inputs();
    v.push((
        "vase",
        revolve_polyline(&[
            Point3::new(0.0, 0.0, 2.0),
            Point3::new(0.5, 0.0, 1.5),
            Point3::new(0.7, 0.0, 0.5),
            Point3::new(0.0, 0.0, 0.0),
        ]),
    ));
    v
}

#[test]
fn self_union_preserves_volume() {
    // Skip vase: boolean output is degenerate (extra edges, tessellation
    // produces zero triangles). The vase's own volume is now correctly
    // ~1.78 thanks to the analytic-surface path in solid_volume, but the
    // boolean kernel doesn't yet preserve it through self-union.
    for (name, s) in algebra_inputs() {
        let r = run(&s, &s, Op::Union);
        let v_in = solid_volume(&s);
        let v_out = solid_volume(&r);
        assert!(
            (v_in - v_out).abs() < TOL,
            "{name}: vol(A∪A)={v_out} ≠ vol(A)={v_in}"
        );
    }
}

#[test]
fn self_intersection_preserves_volume() {
    // Skip vase: see self_union_preserves_volume comment.
    for (name, s) in algebra_inputs() {
        let r = run(&s, &s, Op::Intersection);
        let v_in = solid_volume(&s);
        let v_out = solid_volume(&r);
        assert!(
            (v_in - v_out).abs() < TOL,
            "{name}: vol(A∩A)={v_out} ≠ vol(A)={v_in}"
        );
    }
}

#[test]
fn self_difference_is_empty() {
    for (name, s) in full_inputs() {
        let r = run(&s, &s, Op::Difference);
        let v_out = solid_volume(&r);
        assert!(
            v_out.abs() < TOL,
            "{name}: vol(A−A)={v_out} ≠ 0"
        );
    }
}

#[test]
fn union_is_commutative() {
    let inputs = algebra_inputs();
    let mut errors: Vec<String> = Vec::new();
    for i in 0..inputs.len() {
        for j in (i + 1)..inputs.len() {
            let (a_name, a) = &inputs[i];
            let (b_name, b) = &inputs[j];
            let v_ab = solid_volume(&run(a, b, Op::Union));
            let v_ba = solid_volume(&run(b, a, Op::Union));
            if (v_ab - v_ba).abs() > TOL {
                errors.push(format!(
                    "vol({a_name}∪{b_name})={v_ab} ≠ vol({b_name}∪{a_name})={v_ba}"
                ));
            }
        }
    }
    assert!(errors.is_empty(), "commutativity failures:\n{}", errors.join("\n"));
}

#[test]
fn intersection_is_commutative() {
    let inputs = algebra_inputs();
    let mut errors: Vec<String> = Vec::new();
    for i in 0..inputs.len() {
        for j in (i + 1)..inputs.len() {
            let (a_name, a) = &inputs[i];
            let (b_name, b) = &inputs[j];
            let v_ab = solid_volume(&run(a, b, Op::Intersection));
            let v_ba = solid_volume(&run(b, a, Op::Intersection));
            if (v_ab - v_ba).abs() > TOL {
                errors.push(format!(
                    "vol({a_name}∩{b_name})={v_ab} ≠ vol({b_name}∩{a_name})={v_ba}"
                ));
            }
        }
    }
    assert!(errors.is_empty(), "commutativity failures:\n{}", errors.join("\n"));
}

#[test]
fn set_algebra_conservation_holds() {
    // vol(A) = vol(A∩B) + vol(A−B) for every (A, B) pair.
    let inputs = algebra_inputs();
    let mut errors: Vec<String> = Vec::new();
    for (a_name, a) in &inputs {
        for (b_name, b) in &inputs {
            if a_name == b_name {
                continue;
            }
            let v_a = solid_volume(a);
            let v_inter = solid_volume(&run(a, b, Op::Intersection));
            let v_diff = solid_volume(&run(a, b, Op::Difference));
            let lhs = v_a;
            let rhs = v_inter + v_diff;
            if (lhs - rhs).abs() > TOL {
                errors.push(format!(
                    "{a_name} vs {b_name}: vol(A)={v_a} ≠ vol(A∩B)+vol(A−B)={v_inter}+{v_diff}={rhs}"
                ));
            }
        }
    }
    assert!(
        errors.is_empty(),
        "set-algebra conservation failures:\n{}",
        errors.join("\n")
    );
}

#[test]
fn inclusion_exclusion_holds() {
    // vol(A∪B) = vol(A) + vol(B) − vol(A∩B) for every pair.
    let inputs = algebra_inputs();
    let mut errors: Vec<String> = Vec::new();
    for i in 0..inputs.len() {
        for j in (i + 1)..inputs.len() {
            let (a_name, a) = &inputs[i];
            let (b_name, b) = &inputs[j];
            let v_a = solid_volume(a);
            let v_b = solid_volume(b);
            let v_union = solid_volume(&run(a, b, Op::Union));
            let v_inter = solid_volume(&run(a, b, Op::Intersection));
            let lhs = v_union;
            let rhs = v_a + v_b - v_inter;
            if (lhs - rhs).abs() > TOL {
                errors.push(format!(
                    "{a_name} ∪ {b_name}: vol(A∪B)={v_union} ≠ vol(A)+vol(B)−vol(A∩B)={rhs}"
                ));
            }
        }
    }
    assert!(
        errors.is_empty(),
        "inclusion-exclusion failures:\n{}",
        errors.join("\n")
    );
}

#[test]
fn three_disjoint_box_union_is_associative_and_summed() {
    // 3 disjoint unit boxes far apart. Test (A∪B)∪C = A∪(B∪C) volumetrically.
    let a = box_(Vec3::new(1.0, 1.0, 1.0));
    let b = box_at(Vec3::new(1.0, 1.0, 1.0), Point3::new(5.0, 0.0, 0.0));
    let c = box_at(Vec3::new(1.0, 1.0, 1.0), Point3::new(0.0, 5.0, 0.0));

    let ab = run(&a, &b, Op::Union);
    let abc_left = run(&ab, &c, Op::Union); // (A∪B)∪C
    let bc = run(&b, &c, Op::Union);
    let abc_right = run(&a, &bc, Op::Union); // A∪(B∪C)

    let v_left = solid_volume(&abc_left);
    let v_right = solid_volume(&abc_right);
    assert!(
        (v_left - v_right).abs() < TOL,
        "vol((A∪B)∪C)={v_left} ≠ vol(A∪(B∪C))={v_right}"
    );
    assert!(
        (v_left - 3.0).abs() < TOL,
        "expected sum=3, got {v_left}"
    );
    // 3 disjoint solids → 3 shells.
    assert_eq!(abc_left.topo.shell_count(), 3);
    assert_eq!(abc_right.topo.shell_count(), 3);
}

#[test]
fn diff_is_anti_associative_for_disjoint() {
    // A − B − C should equal A when B, C are disjoint from A.
    let a = box_(Vec3::new(1.0, 1.0, 1.0));
    let far_b = box_at(Vec3::new(1.0, 1.0, 1.0), Point3::new(10.0, 0.0, 0.0));
    let far_c = box_at(Vec3::new(1.0, 1.0, 1.0), Point3::new(0.0, 10.0, 0.0));

    let ab = run(&a, &far_b, Op::Difference);
    let abc = run(&ab, &far_c, Op::Difference);

    let v = solid_volume(&abc);
    assert!(
        (v - 1.0).abs() < TOL,
        "A − far_B − far_C = A; expected vol=1, got {v}"
    );
}
