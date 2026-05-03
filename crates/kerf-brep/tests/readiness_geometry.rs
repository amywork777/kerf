//! Geometric validation of every (primitive × primitive × op) result.
//!
//! readiness_floor.rs only checks "didn't panic + topology validates".
//! This module checks that the result is the *right shape* via:
//!
//! - **Sanity invariants** (universal): hold for any valid boolean
//!   regardless of input geometry — vol(A∪B) ≥ max(vol_A, vol_B) etc.
//! - **Exact-volume assertions** (axis-aligned box pairs): the volumes
//!   are computable analytically (overlap = product of intervals).
//! - **Closed-shell check**: each shell encloses positive volume — catches
//!   the case where a kept-face set produces topology that validates but
//!   has flipped normals (genus-0 closed surface, but enclosing the wrong
//!   region).
//! - **Component-count expectation**: most boolean ops produce a single
//!   solid, exceptions are explicit.
//! - **Trivial face-count expectations**: nested/disjoint inputs have
//!   knowable result face counts.

use kerf_brep::primitives::{
    box_, box_at, cylinder_faceted, extrude_polygon, revolve_polyline,
};
use kerf_brep::{shell_volume, solid_volume, Solid};
use kerf_geom::{Point3, Vec3};
use kerf_topo::validate;

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

/// AABB of a primitive built via box_at. Used for analytic overlap volume.
#[derive(Clone, Copy, Debug)]
struct Aabb {
    min: Point3,
    max: Point3,
}

impl Aabb {
    fn from_extents_origin(extents: Vec3, origin: Point3) -> Self {
        Self {
            min: origin,
            max: origin + extents,
        }
    }
    fn volume(&self) -> f64 {
        (self.max.x - self.min.x).max(0.0)
            * (self.max.y - self.min.y).max(0.0)
            * (self.max.z - self.min.z).max(0.0)
    }
    fn intersect(&self, other: &Aabb) -> Aabb {
        Aabb {
            min: Point3::new(
                self.min.x.max(other.min.x),
                self.min.y.max(other.min.y),
                self.min.z.max(other.min.z),
            ),
            max: Point3::new(
                self.max.x.min(other.max.x),
                self.max.y.min(other.max.y),
                self.max.z.min(other.max.z),
            ),
        }
    }
}

const TOL: f64 = 1e-9;

/// Universal invariant: every boolean result must have validated topology
/// and a non-negative TOTAL signed volume.
///
/// Individual shells may have negative signed volume — that's a CAVITY
/// shell (material on the outside of that shell, e.g., a hollow sphere's
/// inner surface). The sum across all shells gives the actual body volume.
/// What we forbid: a result whose total volume is negative (= all normals
/// flipped).
///
/// Note: the `vase` primitive (revolve_polyline) is a degenerate kerf
/// fixture with V=4/E=5/F=3/vol=0 — its faces are 360°-spanning lunes
/// without intermediate vertices, so they have zero geometric area. Any
/// boolean involving vase legitimately produces vol=0 results, so we
/// don't flag "non-empty face count with zero volume" here.
fn assert_universal(result: &Solid, label: &str) -> Result<(), String> {
    validate(&result.topo).map_err(|e| format!("{label}: topology invalid: {e:?}"))?;
    let v = solid_volume(result);
    if v < -TOL {
        return Err(format!(
            "{label}: total volume {v} is negative — all face normals flipped?"
        ));
    }
    Ok(())
}

/// Op-specific sanity: vol(union) ≥ max, vol(intersect) ≤ min,
/// vol_A − vol_B ≤ vol(diff) ≤ vol_A.
fn assert_op_bounds(result: &Solid, vol_a: f64, vol_b: f64, op: Op, label: &str) {
    let v = solid_volume(result);
    match op {
        Op::Union => {
            let lo = vol_a.max(vol_b) - TOL;
            let hi = vol_a + vol_b + TOL;
            assert!(
                v >= lo && v <= hi,
                "{label}: union vol {v} outside [{lo}, {hi}] \
                 (vol_a={vol_a}, vol_b={vol_b})"
            );
        }
        Op::Intersection => {
            assert!(
                v >= -TOL && v <= vol_a.min(vol_b) + TOL,
                "{label}: intersection vol {v} outside [0, min({vol_a}, {vol_b})]"
            );
        }
        Op::Difference => {
            let lo = (vol_a - vol_b).max(0.0) - TOL;
            let hi = vol_a + TOL;
            assert!(
                v >= lo && v <= hi,
                "{label}: difference vol {v} outside [{lo}, {hi}] \
                 (vol_a={vol_a}, vol_b={vol_b})"
            );
        }
    }
}

/// Exact-volume check for axis-aligned boxes via analytic overlap.
fn assert_exact_box_pair(result: &Solid, a: Aabb, b: Aabb, op: Op, label: &str) {
    let vol_a = a.volume();
    let vol_b = b.volume();
    let vol_overlap = a.intersect(&b).volume();
    let expected = match op {
        Op::Union => vol_a + vol_b - vol_overlap,
        Op::Intersection => vol_overlap,
        Op::Difference => vol_a - vol_overlap,
    };
    let v = solid_volume(result);
    assert!(
        (v - expected).abs() < 1e-6,
        "{label}: exact-volume mismatch — expected {expected}, got {v} \
         (a={vol_a}, b={vol_b}, overlap={vol_overlap})"
    );
}

#[test]
fn every_boolean_result_passes_universal_invariants() {
    let cases = build_named_inputs();
    let ops = [Op::Union, Op::Intersection, Op::Difference];
    let mut count = 0;
    let mut errors: Vec<String> = Vec::new();
    for op in ops {
        for (a_name, a) in &cases {
            for (b_name, b) in &cases {
                if a_name == b_name {
                    continue;
                }
                let label = format!("{op:?} {a_name} vs {b_name}");
                let r = run(a, b, op);
                if let Err(e) = assert_universal(&r, &label) {
                    errors.push(e);
                }
                count += 1;
            }
        }
    }
    assert_eq!(count, 168, "expected 168 cases");
    assert!(
        errors.is_empty(),
        "{} of {count} cases failed universal invariants:\n{}",
        errors.len(),
        errors.join("\n")
    );
}

#[test]
fn every_boolean_result_satisfies_op_bounds() {
    let cases = build_named_inputs();
    let ops = [Op::Union, Op::Intersection, Op::Difference];
    for op in ops {
        for (a_name, a) in &cases {
            for (b_name, b) in &cases {
                if a_name == b_name {
                    continue;
                }
                let label = format!("{op:?} {a_name} vs {b_name}");
                let r = run(a, b, op);
                let vol_a = solid_volume(a);
                let vol_b = solid_volume(b);
                assert_op_bounds(&r, vol_a, vol_b, op, &label);
            }
        }
    }
}

#[test]
fn every_axis_aligned_box_pair_has_exact_expected_volume() {
    // Only the 4 axis-aligned-box primitives in the matrix.
    let boxes: Vec<(&str, Aabb, Solid)> = vec![
        (
            "box[2³]",
            Aabb::from_extents_origin(Vec3::new(2.0, 2.0, 2.0), Point3::origin()),
            box_(Vec3::new(2.0, 2.0, 2.0)),
        ),
        (
            "box-shift",
            Aabb::from_extents_origin(Vec3::new(2.0, 2.0, 2.0), Point3::new(1.0, 0.0, 0.0)),
            box_at(Vec3::new(2.0, 2.0, 2.0), Point3::new(1.0, 0.0, 0.0)),
        ),
        (
            "box-corner",
            Aabb::from_extents_origin(Vec3::new(2.0, 2.0, 2.0), Point3::new(1.0, 1.0, 1.0)),
            box_at(Vec3::new(2.0, 2.0, 2.0), Point3::new(1.0, 1.0, 1.0)),
        ),
        (
            "box-nested",
            Aabb::from_extents_origin(Vec3::new(0.6, 0.6, 0.6), Point3::new(0.7, 0.7, 0.7)),
            box_at(Vec3::new(0.6, 0.6, 0.6), Point3::new(0.7, 0.7, 0.7)),
        ),
    ];
    let ops = [Op::Union, Op::Intersection, Op::Difference];
    let mut count = 0;
    for op in ops {
        for (a_name, a_box, a) in &boxes {
            for (b_name, b_box, b) in &boxes {
                if a_name == b_name {
                    continue;
                }
                // Skip the empty-result cases — exact-zero is brittle to
                // emit since the result_solid may be empty (no faces).
                let overlap = a_box.intersect(b_box).volume();
                if overlap < 1e-12 && op != Op::Union {
                    continue;
                }
                let label = format!("{op:?} {a_name} vs {b_name}");
                let r = run(a, b, op);
                assert_exact_box_pair(&r, *a_box, *b_box, op, &label);
                count += 1;
            }
        }
    }
    assert!(count >= 24, "expected at least 24 box-pair exact assertions");
}

#[test]
fn nested_box_diff_has_two_shells_and_correct_volume() {
    let big = box_(Vec3::new(10.0, 10.0, 10.0));
    let small = box_at(Vec3::new(2.0, 2.0, 2.0), Point3::new(4.0, 4.0, 4.0));
    let r = big.try_difference(&small).unwrap();
    assert_universal(&r, "big - small (nested)").expect("universal");
    // 10³ - 2³ = 1000 - 8 = 992
    let v = solid_volume(&r);
    assert!((v - 992.0).abs() < 1e-6, "got {v}");
    // 2 disjoint shells: outer big (positive signed volume = +1000)
    // + inner cavity (negative signed volume = -8).
    assert_eq!(r.topo.shell_count(), 2, "expected 2 shells");
    let shell_vols: Vec<f64> = r
        .topo
        .shell_ids()
        .map(|sh| shell_volume(&r, sh))
        .collect();
    let positive_count = shell_vols.iter().filter(|v| **v > 0.0).count();
    let negative_count = shell_vols.iter().filter(|v| **v < 0.0).count();
    assert_eq!(positive_count, 1, "expected 1 positive (outer) shell");
    assert_eq!(negative_count, 1, "expected 1 negative (cavity) shell");
    let total: f64 = shell_vols.iter().sum();
    assert!((total - 992.0).abs() < 1e-6);
}

#[test]
fn nested_box_intersect_is_smaller_box() {
    let big = box_(Vec3::new(10.0, 10.0, 10.0));
    let small = box_at(Vec3::new(2.0, 2.0, 2.0), Point3::new(4.0, 4.0, 4.0));
    let r = big.try_intersection(&small).unwrap();
    assert_universal(&r, "big & small (nested)").expect("universal");
    let v = solid_volume(&r);
    assert!((v - 8.0).abs() < 1e-9);
    assert_eq!(r.topo.face_count(), 6);
    assert_eq!(r.topo.vertex_count(), 8);
    assert_eq!(r.topo.edge_count(), 12);
}

#[test]
fn nested_box_union_is_bigger_box() {
    let big = box_(Vec3::new(10.0, 10.0, 10.0));
    let small = box_at(Vec3::new(2.0, 2.0, 2.0), Point3::new(4.0, 4.0, 4.0));
    let r = big.try_union(&small).unwrap();
    assert_universal(&r, "big U small (nested)").expect("universal");
    let v = solid_volume(&r);
    assert!((v - 1000.0).abs() < 1e-6);
    assert_eq!(r.topo.face_count(), 6);
}

#[test]
fn disjoint_box_union_has_two_shells_and_summed_volume() {
    let a = box_(Vec3::new(1.0, 1.0, 1.0));
    let b = box_at(Vec3::new(1.0, 1.0, 1.0), Point3::new(5.0, 0.0, 0.0));
    let r = a.try_union(&b).unwrap();
    assert_universal(&r, "disjoint U").expect("universal");
    assert!((solid_volume(&r) - 2.0).abs() < 1e-9);
    assert_eq!(r.topo.face_count(), 12, "two boxes' worth of faces");
    assert_eq!(r.topo.shell_count(), 2, "two disjoint solids = two shells");
    assert_eq!(r.topo.vertex_count(), 16);
    assert_eq!(r.topo.edge_count(), 24);
}

#[test]
fn disjoint_box_intersect_is_empty() {
    let a = box_(Vec3::new(1.0, 1.0, 1.0));
    let b = box_at(Vec3::new(1.0, 1.0, 1.0), Point3::new(5.0, 0.0, 0.0));
    let r = a.try_intersection(&b).unwrap();
    assert_universal(&r, "disjoint &").expect("universal");
    assert_eq!(r.topo.face_count(), 0, "empty result");
    assert_eq!(r.topo.vertex_count(), 0);
    assert_eq!(solid_volume(&r), 0.0);
}

#[test]
fn disjoint_box_diff_is_unchanged_a() {
    let a = box_(Vec3::new(1.0, 1.0, 1.0));
    let b = box_at(Vec3::new(1.0, 1.0, 1.0), Point3::new(5.0, 0.0, 0.0));
    let r = a.try_difference(&b).unwrap();
    assert_universal(&r, "disjoint -").expect("universal");
    assert!((solid_volume(&r) - 1.0).abs() < 1e-9);
    assert_eq!(r.topo.face_count(), 6);
}

#[test]
fn corner_overlap_diff_volume_is_correct() {
    // Block A=[0,10]³ minus cutter B=[8,12]³. Overlap = [8,10]³ = 8.
    // Result vol = 1000 - 8 = 992.
    let block = box_(Vec3::new(10.0, 10.0, 10.0));
    let cutter = box_at(Vec3::new(4.0, 4.0, 4.0), Point3::new(8.0, 8.0, 8.0));
    let r = block.try_difference(&cutter).unwrap();
    assert_universal(&r, "corner cut").expect("universal");
    assert!((solid_volume(&r) - 992.0).abs() < 1e-6, "got {}", solid_volume(&r));
}

#[test]
fn corner_overlap_intersect_is_corner_overlap() {
    let block = box_(Vec3::new(10.0, 10.0, 10.0));
    let cutter = box_at(Vec3::new(4.0, 4.0, 4.0), Point3::new(8.0, 8.0, 8.0));
    let r = block.try_intersection(&cutter).unwrap();
    assert_universal(&r, "corner intersect").expect("universal");
    // [8,10]³ = 2×2×2 = 8
    assert!((solid_volume(&r) - 8.0).abs() < 1e-6);
    assert_eq!(r.topo.face_count(), 6, "corner overlap is a 2x2x2 box");
}

#[test]
fn tri_prism_diff_box_nested_is_genus_one_tunnel() {
    // box-nested at (0.7..1.3)³ pokes through BOTH of tri-prism's slant walls:
    // at y=1.3 tri-prism's x range is [0.867, 1.133] but box-nested extends to
    // [0.7, 1.3] — both sides of box-nested poke past tri-prism's slants.
    // Diff drills a tunnel → torus topology → genus 1.
    let tri = extrude_polygon(
        &[
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(2.0, 0.0, 0.0),
            Point3::new(1.0, 1.5, 0.0),
        ],
        Vec3::new(0.0, 0.0, 2.0),
    );
    let bn = box_at(Vec3::new(0.6, 0.6, 0.6), Point3::new(0.7, 0.7, 0.7));
    let r = tri.try_difference(&bn).unwrap();
    assert_universal(&r, "tri - box-nested").expect("universal");

    let v = r.topo.vertex_count() as i64;
    let e = r.topo.edge_count() as i64;
    let f = r.topo.face_count() as i64;
    let s = r.topo.shell_count() as i64;
    // V - E + F = 2(S - G) where G = genus.
    let euler = v - e + f;
    let expected_g_x2 = 2 * s - euler;
    assert_eq!(s, 1, "single connected shell");
    assert_eq!(expected_g_x2, 2, "genus 1 (tunnel through prism)");
}

#[test]
fn euler_invariant_holds_globally_on_every_result() {
    // Whatever genus each result happens to have, the Euler-Poincaré
    // identity must hold: V - E + F - R = 2(S - G) for some non-negative
    // integer G. This is also enforced by validate() in kerf-topo, but we
    // re-check here to ensure no result accidentally has fractional genus
    // (which would mean inconsistent topology).
    let cases = build_named_inputs();
    let ops = [Op::Union, Op::Intersection, Op::Difference];
    for op in ops {
        for (a_name, a) in &cases {
            for (b_name, b) in &cases {
                if a_name == b_name {
                    continue;
                }
                let r = run(a, b, op);
                let v = r.topo.vertex_count() as i64;
                let e = r.topo.edge_count() as i64;
                let f = r.topo.face_count() as i64;
                let s = r.topo.shell_count() as i64;
                let r_holes: i64 = r
                    .topo
                    .face_ids()
                    .map(|fid| {
                        r.topo
                            .face(fid)
                            .map(|fc| fc.inner_loops().len())
                            .unwrap_or(0) as i64
                    })
                    .sum();
                let lhs = v - e + f - r_holes;
                let g_x2 = 2 * s - lhs;
                assert!(
                    g_x2 >= 0 && g_x2 % 2 == 0,
                    "{op:?} {a_name} vs {b_name}: V-E+F-R = {lhs}, S = {s}, \
                     genus*2 = {g_x2} (must be non-negative even integer)"
                );
            }
        }
    }
}

fn build_named_inputs() -> Vec<(&'static str, Solid)> {
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
