//! Boolean operation benchmarks. Runs the canonical boolean cases under
//! criterion to track per-op latency.
//!
//! Establishes a perf baseline. Run via `cargo bench -p kerf-brep`.
//! Compare runs by saving to `target/criterion/<bench>/base/` and using
//! `cargo bench -- --save-baseline before` then `--baseline before`.

use criterion::{criterion_group, criterion_main, Criterion};

use kerf_brep::primitives::{box_, box_at, cylinder_faceted, extrude_polygon};
use kerf_brep::Solid;
use kerf_geom::{Point3, Vec3};

fn box_overlap_pair() -> (Solid, Solid) {
    let a = box_(Vec3::new(2.0, 2.0, 2.0));
    let b = box_at(Vec3::new(2.0, 2.0, 2.0), Point3::new(1.0, 0.0, 0.0));
    (a, b)
}

fn nested_box_pair() -> (Solid, Solid) {
    let big = box_(Vec3::new(10.0, 10.0, 10.0));
    let small = box_at(Vec3::new(2.0, 2.0, 2.0), Point3::new(4.0, 4.0, 4.0));
    (big, small)
}

fn cyl_pierces_box() -> (Solid, Solid) {
    let block = box_at(Vec3::new(2.0, 2.0, 2.0), Point3::new(-1.0, -1.0, -1.0));
    let cyl = cylinder_faceted(0.6, 3.0, 12);
    (block, cyl)
}

fn tri_prism_pair() -> (Solid, Solid) {
    let tri = extrude_polygon(
        &[
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(2.0, 0.0, 0.0),
            Point3::new(1.0, 1.5, 0.0),
        ],
        Vec3::new(0.0, 0.0, 2.0),
    );
    let block = box_at(Vec3::new(0.6, 0.6, 0.6), Point3::new(0.7, 0.7, 0.7));
    (tri, block)
}

fn high_res_cyl_box() -> (Solid, Solid) {
    let block = box_at(Vec3::new(2.0, 2.0, 2.0), Point3::new(-1.0, -1.0, -1.0));
    let cyl = cylinder_faceted(0.6, 3.0, 64);
    (block, cyl)
}

fn box_overlap_benchmarks(c: &mut Criterion) {
    let (a, b) = box_overlap_pair();
    c.bench_function("box-overlap union", |bc| {
        bc.iter(|| a.try_union(&b).expect("union"))
    });
    c.bench_function("box-overlap intersect", |bc| {
        bc.iter(|| a.try_intersection(&b).expect("intersect"))
    });
    c.bench_function("box-overlap diff", |bc| {
        bc.iter(|| a.try_difference(&b).expect("diff"))
    });
}

fn nested_benchmarks(c: &mut Criterion) {
    let (big, small) = nested_box_pair();
    c.bench_function("nested union", |bc| {
        bc.iter(|| big.try_union(&small).expect("union"))
    });
    c.bench_function("nested intersect", |bc| {
        bc.iter(|| big.try_intersection(&small).expect("intersect"))
    });
    c.bench_function("nested diff (creates cavity)", |bc| {
        bc.iter(|| big.try_difference(&small).expect("diff"))
    });
}

fn cyl_box_benchmarks(c: &mut Criterion) {
    let (block, cyl) = cyl_pierces_box();
    c.bench_function("cyl12 pierces box union", |bc| {
        bc.iter(|| block.try_union(&cyl).expect("union"))
    });
    c.bench_function("cyl12 pierces box diff", |bc| {
        bc.iter(|| block.try_difference(&cyl).expect("diff"))
    });
}

fn tri_prism_benchmarks(c: &mut Criterion) {
    let (tri, block) = tri_prism_pair();
    c.bench_function("tri-prism diff box-nested (genus 1)", |bc| {
        bc.iter(|| tri.try_difference(&block).expect("diff"))
    });
}

fn high_res_benchmarks(c: &mut Criterion) {
    let (block, cyl) = high_res_cyl_box();
    c.bench_function("box diff cyl_n64 (high-res)", |bc| {
        bc.iter(|| block.try_difference(&cyl).expect("diff"))
    });
}

criterion_group!(
    boolean_benches,
    box_overlap_benchmarks,
    nested_benchmarks,
    cyl_box_benchmarks,
    tri_prism_benchmarks,
    high_res_benchmarks,
);
criterion_main!(boolean_benches);
