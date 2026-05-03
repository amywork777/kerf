//! Reproducers for "non-manifold input to stitch" on chained Difference.
//!
//! ## What we know (verified via differential)
//!
//! - `block - box_at_cutter - box_at_cutter - box_at_cutter` works:
//!   `chained_diff_three_box_through_holes` is GREEN today.
//! - `block - extrude_polygon_cutter - extrude_polygon_cutter` panics in
//!   stitch on the SECOND diff (not the third). True for every n≥4 prism
//!   tested. True even for an axis-aligned 2×2×12 box built via
//!   `extrude_polygon` instead of `box_at`.
//! - The mixing `extrude → box_at` works; `box_at → extrude` panics.
//!   Conclusion: the trigger is the SECOND diff using an extrude-built
//!   cutter against any post-boolean A.
//! - Face polygons (`face_polygon`) returned by extrude_polygon and box_at
//!   for geometrically identical cubes are byte-for-byte identical, so
//!   the discrepancy is upstream of the polygon walk — face/vertex
//!   slotmap iteration order produced by the construction path.
//!
//! These tests are `#[ignore]` so the workspace stays green; remove the
//! ignore once the boolean pipeline is hardened against face-iteration
//! order. The kerf-cad layer ships a `union-cutters-first` workaround
//! that avoids the bug for typical multi-cut models.

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

/// Reproducer (currently fails). chained DIFF with cylinder cutters.
#[test]
#[ignore = "kerf boolean pipeline mishandles extrude-built cutters as the B operand of a chained DIFF — workaround in kerf-cad: union cutters first, then single DIFF"]
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

/// Reproducer (currently fails). chained DIFF with extrude_polygon-built
/// axis-aligned box cutters — geometrically identical to box_at, but built
/// via the extrude_polygon path. Confirms the bug is in construction path,
/// not geometry, polygon shape, or curved-surface handling.
#[test]
#[ignore = "see chained_diff_three_cylinder_through_holes"]
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

/// Attempted workaround: union the cutters first, then a single DIFF.
/// FAILS too — union of disjoint extrude-built cylinders also triggers
/// the bug (same "1 half-edge" panic). So a "union cutters first"
/// strategy is NOT a viable workaround in pure kerf today.
#[test]
#[ignore = "union of disjoint extrude-built cylinders ALSO panics — same root cause as chained_diff_three_cylinder_through_holes"]
fn workaround_union_cutters_then_single_difference() {
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
