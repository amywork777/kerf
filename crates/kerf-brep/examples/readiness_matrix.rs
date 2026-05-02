//! M34 readiness matrix: run every primitive × primitive × op combo via the
//! non-panicking `try_*` API and print a pass/fail table. The ground truth
//! for "what works today" — drives the CAD-readiness roadmap.
//!
//! Output format:
//!   <op>     <a>       <b>       OK | ERR(<message excerpt>)

use kerf_brep::primitives::{
    box_, box_at, cylinder_faceted, extrude_polygon, revolve_polyline,
};
use kerf_brep::Solid;
use kerf_geom::{Point3, Vec3};

#[derive(Clone, Copy)]
enum Op {
    Union,
    Intersection,
    Difference,
}

impl Op {
    fn label(&self) -> &'static str {
        match self {
            Op::Union => "union   ",
            Op::Intersection => "intersect",
            Op::Difference => "diff    ",
        }
    }
}

fn run(a: &Solid, b: &Solid, op: Op) -> Result<Solid, String> {
    match op {
        Op::Union => a.try_union(b),
        Op::Intersection => a.try_intersection(b),
        Op::Difference => a.try_difference(b),
    }
    .map_err(|e| {
        // Truncate the panic message to first 90 chars for readable table output.
        let msg = e.message;
        let short = msg.chars().take(90).collect::<String>();
        if msg.len() > 90 { format!("{short}…") } else { short }
    })
}

fn build_inputs() -> Vec<(&'static str, Solid)> {
    vec![
        (
            "box[2³]",
            box_(Vec3::new(2.0, 2.0, 2.0)),
        ),
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
        (
            "cyl_n12",
            cylinder_faceted(0.6, 3.0, 12),
        ),
        (
            "cyl_n4",
            cylinder_faceted(0.7, 2.0, 4),
        ),
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

fn main() {
    let inputs = build_inputs();
    let ops = [Op::Union, Op::Intersection, Op::Difference];

    let mut total = 0;
    let mut ok = 0;
    println!(
        "{:9}  {:11}  {:11}  result",
        "op", "A", "B"
    );
    println!("{}", "-".repeat(78));
    for op in ops {
        for (a_name, a) in &inputs {
            for (b_name, b) in &inputs {
                if a_name == b_name {
                    // Skip self-self (degenerate but uninteresting).
                    continue;
                }
                total += 1;
                match run(a, b, op) {
                    Ok(r) => {
                        ok += 1;
                        println!(
                            "{}  {:<11}  {:<11}  OK ({}V/{}E/{}F)",
                            op.label(),
                            a_name,
                            b_name,
                            r.vertex_count(),
                            r.edge_count(),
                            r.face_count(),
                        );
                    }
                    Err(msg) => {
                        println!(
                            "{}  {:<11}  {:<11}  ERR  {}",
                            op.label(),
                            a_name,
                            b_name,
                            msg
                        );
                    }
                }
            }
        }
    }
    println!("{}", "-".repeat(78));
    println!("readiness: {ok}/{total} ({:.0}%)", 100.0 * ok as f64 / total as f64);
}
