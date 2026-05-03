//! Regression suite for chained `Difference` and disjoint `Union` that
//! previously panicked in stitch with "1 half-edge". Root cause was
//! face-iteration-order dependence in the boolean pipeline (mef order
//! varied with input solid construction path), surfaced by the difference
//! between `box_` and `extrude_polygon` half-edge layouts.
//!
//! Fix shipped in two parts:
//! - `pipeline.rs`: canonicalise intersection processing by 3D-coord sort,
//!   so the mef order is geometry-determined rather than slotmap-iteration-
//!   determined.
//! - `face_polygon.rs`: rotate the returned polygon off any stinger
//!   junction (a vertex whose forward fan would emit a zero-area
//!   triangle), so downstream tessellation of fjord-style annular faces
//!   produces non-degenerate triangles regardless of where the loop
//!   happened to start.

use kerf_brep::{
    geometry::{CurveKind, SurfaceKind},
    primitives::{box_, box_at, cylinder_faceted, extrude_polygon},
    Solid,
};
use kerf_geom::{Point3, Vec3};

/// Pure translation of a Solid: rewrites every Point3 + Frame.origin in place.
fn translate(s: &Solid, offset: Vec3) -> Solid {
    let mut out = s.clone();
    for (_, p) in out.vertex_geom.iter_mut() {
        *p += offset;
    }
    for (_, surf) in out.face_geom.iter_mut() {
        match surf {
            SurfaceKind::Plane(p) => p.frame.origin += offset,
            SurfaceKind::Cylinder(c) => c.frame.origin += offset,
            SurfaceKind::Sphere(s) => s.frame.origin += offset,
            SurfaceKind::Cone(c) => c.frame.origin += offset,
            SurfaceKind::Torus(t) => t.frame.origin += offset,
        }
    }
    for (_, seg) in out.edge_geom.iter_mut() {
        match &mut seg.curve {
            CurveKind::Line(l) => l.origin += offset,
            CurveKind::Circle(c) => c.frame.origin += offset,
            CurveKind::Ellipse(e) => e.frame.origin += offset,
        }
    }
    out
}

/// Baseline (passes today): chained DIFF with box_at-built cutters.
#[test]
fn chained_diff_three_box_through_holes() {
    let block = box_(Vec3::new(30.0, 20.0, 10.0));
    let p1 = box_at(Vec3::new(2.0, 2.0, 12.0), Point3::new(7.0, 10.0, -1.0));
    let p2 = box_at(Vec3::new(2.0, 2.0, 12.0), Point3::new(15.0, 10.0, -1.0));
    let p3 = box_at(Vec3::new(2.0, 2.0, 12.0), Point3::new(24.0, 10.0, -1.0));

    let s1 = block.try_difference(&p1).expect("step 1 OK");
    let s2 = s1.try_difference(&p2).expect("step 2 OK");
    let s3 = s2.try_difference(&p3).expect("step 3 OK");
    assert_eq!(s3.shell_count(), 1, "single shell of genus 3");
}

/// chained DIFF with cylinder cutters. Fixed by intersection-order
/// canonicalization in pipeline.rs + stinger-junction polygon rotation in
/// face_polygon.rs.
#[test]
fn chained_diff_three_cylinder_through_holes() {
    let block = box_(Vec3::new(30.0, 20.0, 10.0));
    let cyl = cylinder_faceted(2.0, 12.0, 12);
    let p1 = translate(&cyl, Vec3::new(7.0, 10.0, -1.0));
    let p2 = translate(&cyl, Vec3::new(15.0, 10.0, -1.0));
    let p3 = translate(&cyl, Vec3::new(24.0, 10.0, -1.0));
    let s1 = block.try_difference(&p1).expect("step 1");
    let s2 = s1.try_difference(&p2).expect("step 2");
    s2.try_difference(&p3).expect("step 3");
}

/// chained DIFF with extrude_polygon-built axis-aligned box cutters.
/// Now passes with intersection-sort + polygon-rotate fixes.
#[test]
fn chained_diff_three_extruded_boxes_axis_aligned() {
    let block = box_(Vec3::new(30.0, 20.0, 10.0));
    let make_cutter = |cx: f64, cy: f64| {
        let half = 1.0;
        let profile = vec![
            Point3::new(cx - half, cy - half, -1.0),
            Point3::new(cx + half, cy - half, -1.0),
            Point3::new(cx + half, cy + half, -1.0),
            Point3::new(cx - half, cy + half, -1.0),
        ];
        extrude_polygon(&profile, Vec3::new(0.0, 0.0, 12.0))
    };
    let p1 = make_cutter(7.0, 10.0);
    let p2 = make_cutter(15.0, 10.0);
    let p3 = make_cutter(24.0, 10.0);
    let s1 = block.try_difference(&p1).expect("step 1");
    let s2 = s1.try_difference(&p2).expect("step 2");
    s2.try_difference(&p3).expect("step 3");
}

/// Union of disjoint extrude-built cylinders, then single DIFF.
/// Now also passes with the same fix.
#[test]
fn union_disjoint_cutters_then_single_difference() {
    let block = box_(Vec3::new(30.0, 20.0, 10.0));
    let cyl = cylinder_faceted(2.0, 12.0, 12);
    let p1 = translate(&cyl, Vec3::new(7.0, 10.0, -1.0));
    let p2 = translate(&cyl, Vec3::new(15.0, 10.0, -1.0));
    let p3 = translate(&cyl, Vec3::new(24.0, 10.0, -1.0));

    let cutters = p1.try_union(&p2).expect("p1 ∪ p2");
    let cutters = cutters.try_union(&p3).expect("p1 ∪ p2 ∪ p3");
    let result = block.try_difference(&cutters);
    assert!(result.is_ok(), "block - (∪ cutters): {result:?}");
}

/// Mixing differential. Passes today: extrude cutter at step 1, box_at
/// cutter at step 2 — proves the post-step-1 solid (s1) is not the
/// problem; the trigger is an extrude cutter at step 2+.
#[test]
fn mixed_extrude_then_box_at() {
    let block = box_(Vec3::new(30.0, 20.0, 10.0));
    let make_extrude = |cx: f64, cy: f64| {
        let half = 1.0;
        let profile = vec![
            Point3::new(cx - half, cy - half, -1.0),
            Point3::new(cx + half, cy - half, -1.0),
            Point3::new(cx + half, cy + half, -1.0),
            Point3::new(cx - half, cy + half, -1.0),
        ];
        extrude_polygon(&profile, Vec3::new(0.0, 0.0, 12.0))
    };
    let p1 = make_extrude(7.0, 10.0);
    let p2 = box_at(Vec3::new(2.0, 2.0, 12.0), Point3::new(14.0, 9.0, -1.0));
    let s1 = block.try_difference(&p1).expect("step 1 (extrude cutter) OK");
    s1.try_difference(&p2).expect("step 2 (box_at cutter) OK");
}
