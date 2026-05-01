//! M9 CSG demo: extrude + recursive boolean + STL output.
//!
//! Model: "Prism intersected down to a core box"
//!
//!   Step 1: Make a large triangular prism P using extrude_polygon.
//!   Step 2: Make a box B nested inside P.
//!           boolean_solid(&P, &B, Intersection) → result_1 = B (single shell).
//!   Step 3: Make a smaller prism Q nested inside result_1.
//!           boolean_solid(&result_1, &Q, Intersection) → final = Q (recursive boolean).
//!
//! V1 kernel constraint: boolean_solid produces a valid single-shell solid when
//! one operand is entirely nested inside the other (zero face-face intersections).
//! The FaceSoup path (boolean()) is more flexible — used here for STL output.
//!
//! Separately write the FaceSoup STL of (prism P union detached_box) to produce
//! a more interesting mesh with multiple components.

use std::fs::File;
use std::io::BufWriter;

use kerf_brep::booleans::{BooleanOp, boolean, boolean_solid};
use kerf_brep::geometry::{CurveKind, SurfaceKind};
use kerf_brep::primitives::{box_, extrude_polygon};
use kerf_brep::{Solid, write_ascii, write_binary};
use kerf_geom::{Point3, Tolerance, Vec3};

/// Translate all geometry of `s` by `offset`.
fn translate(s: &mut Solid, offset: Vec3) {
    for (_, p) in s.vertex_geom.iter_mut() {
        *p += offset;
    }
    for (_, surf) in s.face_geom.iter_mut() {
        if let SurfaceKind::Plane(plane) = surf {
            plane.frame.origin += offset;
        }
    }
    for (_, seg) in s.edge_geom.iter_mut() {
        if let CurveKind::Line(line) = &mut seg.curve {
            line.origin += offset;
        }
    }
}

fn main() -> std::io::Result<()> {
    let tol = Tolerance::default();

    // ── Step 1: large triangular prism via extrude_polygon ────────────────────
    // Triangle base in the XY-plane at z=0, extruded 20 units in +Z.
    // The prism spans x∈[-10,10], y∈[0,~17], z∈[0,20] (roughly).
    let triangle_large = vec![
        Point3::new(-10.0, 0.0, 0.0),
        Point3::new(10.0, 0.0, 0.0),
        Point3::new(0.0, 17.0, 0.0),
    ];
    let prism_large = extrude_polygon(&triangle_large, Vec3::new(0.0, 0.0, 20.0));
    println!(
        "Large prism (extrude_polygon): V={}, E={}, F={}",
        prism_large.vertex_count(),
        prism_large.edge_count(),
        prism_large.face_count(),
    );

    // ── Step 2: box nested inside prism_large ────────────────────────────────
    // Box at (−2, 2, 5) with extents 4×4×4 — fully inside the large prism.
    let mut nested_box = box_(Vec3::new(4.0, 4.0, 4.0));
    translate(&mut nested_box, Vec3::new(-2.0, 2.0, 5.0));

    // First boolean_solid: Intersection selects what is inside both → nested_box.
    let result_1 = boolean_solid(&prism_large, &nested_box, BooleanOp::Intersection, &tol);
    println!(
        "After intersection (prism ∩ nested_box): V={}, E={}, F={}",
        result_1.vertex_count(),
        result_1.edge_count(),
        result_1.face_count(),
    );

    // ── Step 3: small prism nested inside result_1, recursive boolean ─────────
    // Small triangular prism, apex at (0,4,0), base width 2, height 3.
    // Sits at z∈[6,8], inside the 4×4×4 box of result_1 which spans z∈[5,9].
    let triangle_small = vec![
        Point3::new(-1.0, 2.5, 6.0),
        Point3::new(1.0, 2.5, 6.0),
        Point3::new(0.0, 4.0, 6.0),
    ];
    let prism_small = extrude_polygon(&triangle_small, Vec3::new(0.0, 0.0, 2.0));
    println!(
        "Small prism (extrude_polygon): V={}, E={}, F={}",
        prism_small.vertex_count(),
        prism_small.edge_count(),
        prism_small.face_count(),
    );

    // RECURSIVE boolean: boolean_solid of boolean_solid result.
    // Intersection picks the smaller prism (fully inside result_1).
    let final_solid = boolean_solid(&result_1, &prism_small, BooleanOp::Intersection, &tol);
    println!(
        "Final solid (recursive intersection): V={}, E={}, F={}",
        final_solid.vertex_count(),
        final_solid.edge_count(),
        final_solid.face_count(),
    );

    // ── Step 4: triangle soup for STL output ─────────────────────────────────
    // Use the FaceSoup path for a richer output: union of the large prism and a
    // detached small box (disjoint, so no intersection issues).
    let mut detached_box = box_(Vec3::new(3.0, 3.0, 3.0));
    translate(&mut detached_box, Vec3::new(15.0, 0.0, 0.0)); // clearly outside the prism
    let soup = boolean(&prism_large, &detached_box, BooleanOp::Union, &tol);
    println!("Triangle count (prism ∪ detached_box): {}", soup.triangles.len());

    // ── Step 5: write STL files ───────────────────────────────────────────────
    let ascii_path = "/tmp/claude/kerf_demo.stl";
    let bin_path = "/tmp/claude/kerf_demo.bin.stl";

    {
        let f = File::create(ascii_path)?;
        let mut w = BufWriter::new(f);
        write_ascii(&soup, "kerf_demo", &mut w)?;
    }
    {
        let f = File::create(bin_path)?;
        let mut w = BufWriter::new(f);
        write_binary(&soup, "kerf_demo", &mut w)?;
    }

    println!("Wrote {ascii_path}");
    println!("Wrote {bin_path}");

    // ── Step 6: verify binary STL size formula ────────────────────────────────
    let expected_bin_size = 80 + 4 + 50 * soup.triangles.len();
    let actual_bin_size = std::fs::metadata(bin_path)?.len() as usize;
    assert_eq!(
        actual_bin_size, expected_bin_size,
        "binary STL size mismatch: got {actual_bin_size}, expected {expected_bin_size}",
    );
    println!(
        "Binary STL: {} bytes = 80 + 4 + 50×{} ✓",
        actual_bin_size,
        soup.triangles.len()
    );

    Ok(())
}
