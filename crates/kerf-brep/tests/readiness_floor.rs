//! Regression test for the boolean readiness percentage.
//!
//! Runs every (primitive A, primitive B, op) combination through the
//! non-panicking `try_*` API and asserts that at least `READINESS_FLOOR`
//! cases succeed. Bumping the floor as fixes land prevents future
//! regressions from sneaking past CI.
//!
//! To regenerate the human-readable table, use the `readiness_matrix`
//! example: `cargo run --example readiness_matrix -p kerf-brep`.

use kerf_brep::primitives::{
    box_, box_at, cylinder_faceted, extrude_polygon, revolve_polyline,
};
use kerf_brep::Solid;
use kerf_geom::{Point3, Vec3};

/// Minimum number of (primitive × primitive × op) combos that must succeed.
/// Bumped milestone-by-milestone in docs/readiness.md.
const READINESS_FLOOR: usize = 134;

/// Total number of combos run by the matrix (8 inputs × 7 others × 3 ops).
const READINESS_TOTAL: usize = 168;

#[derive(Clone, Copy)]
enum Op {
    Union,
    Intersection,
    Difference,
}

fn run(a: &Solid, b: &Solid, op: Op) -> Result<Solid, String> {
    match op {
        Op::Union => a.try_union(b),
        Op::Intersection => a.try_intersection(b),
        Op::Difference => a.try_difference(b),
    }
    .map_err(|e| e.message)
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

#[test]
fn readiness_floor_holds() {
    let inputs = build_inputs();
    let ops = [Op::Union, Op::Intersection, Op::Difference];

    let mut total = 0usize;
    let mut ok = 0usize;
    for op in ops {
        for (a_name, a) in &inputs {
            for (b_name, b) in &inputs {
                if a_name == b_name {
                    continue;
                }
                total += 1;
                if run(a, b, op).is_ok() {
                    ok += 1;
                }
            }
        }
    }
    assert_eq!(
        total, READINESS_TOTAL,
        "input matrix size changed; update READINESS_TOTAL"
    );
    assert!(
        ok >= READINESS_FLOOR,
        "readiness regressed: {ok}/{total} succeeded, floor is {READINESS_FLOOR}/{READINESS_TOTAL}. \
         If you intentionally raised the bar, update READINESS_FLOOR. If a fix regressed, \
         investigate via `cargo run --example readiness_matrix -p kerf-brep`."
    );
}
