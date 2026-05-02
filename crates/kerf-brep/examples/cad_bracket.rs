//! M22a CAD example 3 — L-bracket with chamfered outer corner.
//!
//! A right-angle bracket formed by unioning two rectangular slabs:
//!
//!   Vertical arm:   box(1, 6, 6) at (0, 0, 0)   — upright thin slab
//!   Horizontal arm: box(6, 6, 1) at (0, 0, 0)   — wide flat base slab
//!
//! The two arms share an edge along the bottom of the vertical slab, forming
//! an L-shape in cross-section.
//!
//! Optionally a 2×6×2 cutter box at (5, 0, 5) is subtracted to chamfer the
//! upper-outer corner of the bracket.  If Solid::difference panics (which can
//! happen when cutter faces align with existing faces), the example falls back
//! to the plain L-union and documents the limitation.

use std::fs::File;
use std::io::BufWriter;

use kerf_brep::primitives::box_at;
use kerf_brep::tessellate::tessellate;
use kerf_brep::write_binary;
use kerf_geom::{Point3, Vec3};

fn main() -> std::io::Result<()> {
    let dir = std::env::var("KERF_GALLERY_DIR")
        .unwrap_or_else(|_| "/tmp/claude/kerf_gallery".to_string());
    std::fs::create_dir_all(&dir)?;

    // Build the L-shape by unioning two slabs.
    let vertical   = box_at(Vec3::new(1.0, 6.0, 6.0), Point3::new(0.0, 0.0, 0.0));
    let horizontal = box_at(Vec3::new(6.0, 6.0, 1.0), Point3::new(0.0, 0.0, 0.0));

    let l_shape = vertical.union(&horizontal);
    println!(
        "cad_bracket (L-union): V={}, E={}, F={}",
        l_shape.vertex_count(),
        l_shape.edge_count(),
        l_shape.face_count(),
    );

    // Attempt a chamfer: subtract a box from the upper-outer corner.
    // The cutter at (5, 0, 5) with extents (2, 6, 2) removes the top-right
    // corner of the vertical arm, producing a 45°-chamfered look.
    let cutter = box_at(Vec3::new(2.0, 6.0, 2.0), Point3::new(5.0, 0.0, 5.0));

    let segs = 32;
    let path = format!("{dir}/cad_bracket.bin.stl");

    let result = std::panic::catch_unwind(|| {
        let chamfered = l_shape.difference(&cutter);
        let soup = tessellate(&chamfered, segs);
        println!(
            "cad_bracket (chamfered): V={}, E={}, F={}, {} triangles",
            chamfered.vertex_count(),
            chamfered.edge_count(),
            chamfered.face_count(),
            soup.triangles.len(),
        );
        soup
    });

    let soup = match result {
        Ok(s) => s,
        Err(_) => {
            // Fallback: plain L-union without chamfer.
            // Happens if the cutter aligns with existing slab faces.
            println!("cad_bracket: difference panicked — using plain L-union (no chamfer)");
            let s = tessellate(&l_shape, segs);
            println!("cad_bracket (plain L): {} triangles", s.triangles.len());
            s
        }
    };

    let f = File::create(&path)?;
    let mut w = BufWriter::new(f);
    write_binary(&soup, "cad_bracket", &mut w)?;
    println!("  -> {path}");

    Ok(())
}
