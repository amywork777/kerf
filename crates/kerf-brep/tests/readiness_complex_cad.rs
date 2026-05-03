//! Complex-CAD stress: build multi-step CAD models via boolean chains,
//! tessellate, re-import, run further booleans. Mimics what users would
//! do with externally-sourced mechanical-part STLs.

use std::io::Cursor;

use kerf_brep::booleans::BooleanOp;
use kerf_brep::primitives::{box_, box_at, cylinder_faceted};
use kerf_brep::{
    read_stl_binary_to_solid, solid_volume, tessellate, write_binary, BooleanError, Solid,
};
use kerf_geom::{Point3, Vec3};

const VOL_TOL: f64 = 0.05;

fn try_op(a: &Solid, b: &Solid, op: BooleanOp) -> Result<Solid, BooleanError> {
    match op {
        BooleanOp::Union => a.try_union(b),
        BooleanOp::Intersection => a.try_intersection(b),
        BooleanOp::Difference => a.try_difference(b),
    }
}

fn import_tessellated(s: &Solid, segments: usize, label: &str) -> Solid {
    let soup = tessellate(s, segments);
    let mut buf = Vec::new();
    write_binary(&soup, label, &mut buf).expect("write_binary");
    let mut reader = Cursor::new(buf);
    read_stl_binary_to_solid(&mut reader).expect("read_stl_binary_to_solid")
}

// ============================================================================
// Bracket: a plate with a slot and 4 mounting holes
// ============================================================================

#[test]
#[ignore = "chained DIFF on a 3+ shell intermediate result hits a known kernel limitation"]
fn bracket_with_slot_and_holes_imports_and_diffs() {
    // Build: 100×60×8 plate, subtract central slot, subtract 4 corner holes.
    // Cutters extend slightly beyond the plate's z range — standard CAD
    // practice to avoid coplanar-cutter top/bottom face artifacts (the
    // kernel's coplanar-overlap handling is a known limitation per
    // docs/readiness.md).
    let plate = box_at(Vec3::new(100.0, 60.0, 8.0), Point3::new(0.0, 0.0, 0.0));

    // Central slot — extends z=−1 to z=9 (poking through both faces).
    let slot = cylinder_faceted(8.0, 10.0, 24);
    let slot_translated = translate_solid(&slot, Vec3::new(50.0, 30.0, -1.0));

    let plate_minus_slot = try_op(&plate, &slot_translated, BooleanOp::Difference)
        .expect("plate − slot");
    assert!(solid_volume(&plate_minus_slot) > 0.0);

    // 4 corner holes (also poking through).
    let mut acc = plate_minus_slot;
    for &(cx, cy) in &[(10.0, 10.0), (90.0, 10.0), (10.0, 50.0), (90.0, 50.0)] {
        let hole = translate_solid(
            &cylinder_faceted(2.5, 10.0, 12),
            Vec3::new(cx, cy, -1.0),
        );
        acc = try_op(&acc, &hole, BooleanOp::Difference)
            .unwrap_or_else(|e| panic!("hole at ({cx}, {cy}): {}", e.message));
    }
    let v_bracket = solid_volume(&acc);

    // Through-holes: each hole carves a tunnel, so the result is genus 5.
    // Single outer shell with 5 tunnels — verify topology validates.
    kerf_topo::validate(&acc.topo).expect("bracket topology");
    assert!(v_bracket > 0.0);
    assert!(v_bracket < 100.0 * 60.0 * 8.0); // less than the plate

    // Tessellate → re-import → run a UNION with another box.
    let imported = import_tessellated(&acc, 24, "bracket");
    // Re-imported volume should match (tolerance for tessellation).
    let v_imported = solid_volume(&imported);
    assert!(
        (v_imported - v_bracket).abs() < VOL_TOL.max(v_bracket * 0.001),
        "bracket: in-vol {v_bracket} ≠ imported vol {v_imported}"
    );

    // Run a UNION on the imported bracket.
    let extra = box_at(Vec3::new(20.0, 20.0, 8.0), Point3::new(120.0, 20.0, 0.0));
    let r = try_op(&imported, &extra, BooleanOp::Union)
        .expect("imported bracket ∪ extra block");
    let v_r = solid_volume(&r);
    // extra is disjoint from bracket → vol = vol(bracket) + vol(extra).
    let v_extra = 20.0 * 20.0 * 8.0;
    let expected = v_imported + v_extra;
    assert!(
        (v_r - expected).abs() < VOL_TOL.max(expected * 0.001),
        "bracket ∪ extra: got {v_r}, expected {expected}"
    );
}

// ============================================================================
// Stepped plate: rectangular block with stepped notches
// ============================================================================

#[test]
#[ignore = "chained DIFF on a 2-shell intermediate result with adjacent cutters hits a kernel limitation"]
fn stepped_plate_carves_correctly() {
    // Block 50×50×20. Three stepped notches subtracted to form a stair.
    // Each step's top z is just below 20 to avoid coplanar with plate top
    // (kernel coplanar-overlap is a known limitation; see readiness.md).
    let plate = box_(Vec3::new(50.0, 50.0, 20.0));
    // Steps don't share z values with each other or the plate top — avoids
    // ALL coplanar-face cases.
    let step1 = box_at(Vec3::new(50.0, 16.0, 4.0), Point3::new(0.0, 33.0, 14.0));
    let step2 = box_at(Vec3::new(50.0, 16.0, 8.0), Point3::new(0.0, 17.0, 8.5));
    let step3 = box_at(Vec3::new(50.0, 16.0, 12.0), Point3::new(0.0, 1.0, 2.7));

    let mut acc = plate;
    for (i, step) in [step1, step2, step3].iter().enumerate() {
        acc = try_op(&acc, step, BooleanOp::Difference)
            .unwrap_or_else(|e| panic!("step {i}: {}", e.message));
    }
    let v = solid_volume(&acc);
    let expected =
        50.0 * 50.0 * 20.0 - (50.0 * 16.0 * 4.0 + 50.0 * 16.0 * 8.0 + 50.0 * 16.0 * 12.0);
    assert!(
        (v - expected).abs() < 0.01,
        "stepped plate vol={v}, expected {expected}"
    );

    // Round-trip and verify volume preserved.
    let imported = import_tessellated(&acc, 24, "stepped");
    let v_imp = solid_volume(&imported);
    assert!(
        (v_imp - v).abs() < VOL_TOL.max(v * 0.001),
        "stepped roundtrip vol drift: {v} vs {v_imp}"
    );
}

// ============================================================================
// Hollow cube: cube with a smaller cube cavity, then carved holes through it
// ============================================================================

#[test]
fn hollow_cube_with_through_holes_works() {
    let outer = box_(Vec3::new(20.0, 20.0, 20.0));
    let cavity = box_at(Vec3::new(10.0, 10.0, 10.0), Point3::new(5.0, 5.0, 5.0));
    let hollow = try_op(&outer, &cavity, BooleanOp::Difference)
        .expect("outer − cavity (hollow cube)");
    assert_eq!(hollow.topo.shell_count(), 2, "outer + cavity");
    // Vol = 8000 - 1000 = 7000.
    assert!((solid_volume(&hollow) - 7000.0).abs() < 1e-6);

    // Round-trip.
    let imported = import_tessellated(&hollow, 24, "hollow");
    let v_imp = solid_volume(&imported);
    assert!((v_imp - 7000.0).abs() < VOL_TOL * 1000.0, "hollow imported vol drift");
    assert_eq!(imported.topo.shell_count(), 2, "imported still 2 shells");

    // Drill a hole through it (intersects both outer wall AND inner cavity).
    let drill = translate_solid(&cylinder_faceted(2.0, 25.0, 12), Vec3::new(10.0, 10.0, -2.5));
    // imported has the cube shell at [0,20]³ + cavity shell at [5,15]³.
    // Drill is a 2-radius cyl from z=-2.5 to z=22.5 along z-axis at xy=(10,10).
    // It pierces through outer (top + bottom) AND through cavity (top + bottom of cavity).
    let r = try_op(&imported, &drill, BooleanOp::Difference)
        .expect("hollow − drill");
    let v_r = solid_volume(&r);
    assert!(
        v_r < v_imp,
        "drill should reduce volume: was {v_imp}, after {v_r}"
    );
    assert!(
        v_r > 0.0,
        "drill shouldn't empty the result"
    );
}

// ============================================================================
// Helper
// ============================================================================

fn translate_solid(s: &Solid, t: Vec3) -> Solid {
    let mut out = s.clone();
    for v in out.vertex_geom.values_mut() {
        *v += t;
    }
    use kerf_brep::geometry::SurfaceKind;
    for surf in out.face_geom.values_mut() {
        if let SurfaceKind::Plane(plane) = surf {
            plane.frame.origin += t;
        } else if let SurfaceKind::Cylinder(cyl) = surf {
            cyl.frame.origin += t;
        }
    }
    out
}
