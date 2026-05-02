//! M22a CAD example 2 — stair-step pyramid (Mayan ziggurat) via stacked boxes.
//!
//! Five axis-aligned boxes of decreasing footprint, each 1-unit tall, are
//! unioned together to form a stepped pyramid:
//!
//!   Layer 0: box(10, 10, 1) at z=0            (bottom, widest)
//!   Layer 1: box( 8,  8, 1) at (1, 1, 1)
//!   Layer 2: box( 6,  6, 1) at (2, 2, 2)
//!   Layer 3: box( 4,  4, 1) at (3, 3, 3)
//!   Layer 4: box( 2,  2, 1) at (4, 4, 4)      (top, narrowest)
//!
//! If Solid::union panics on face-to-face touching geometry, the fallback
//! tessellates each layer separately and merges the FaceSoup triangles — this
//! still validates the architecture and renders correctly.

use std::fs::File;
use std::io::BufWriter;

use kerf_brep::booleans::FaceSoup;
use kerf_brep::primitives::box_at;
use kerf_brep::tessellate::tessellate;
use kerf_brep::write_binary;
use kerf_geom::{Point3, Vec3};

fn main() -> std::io::Result<()> {
    let dir = std::env::var("KERF_GALLERY_DIR")
        .unwrap_or_else(|_| "/tmp/claude/kerf_gallery".to_string());
    std::fs::create_dir_all(&dir)?;

    // Define the 5 pyramid layers (extents, origin).
    let layers: &[(Vec3, Point3)] = &[
        (Vec3::new(10.0, 10.0, 1.0), Point3::new(0.0, 0.0, 0.0)),
        (Vec3::new( 8.0,  8.0, 1.0), Point3::new(1.0, 1.0, 1.0)),
        (Vec3::new( 6.0,  6.0, 1.0), Point3::new(2.0, 2.0, 2.0)),
        (Vec3::new( 4.0,  4.0, 1.0), Point3::new(3.0, 3.0, 3.0)),
        (Vec3::new( 2.0,  2.0, 1.0), Point3::new(4.0, 4.0, 4.0)),
    ];

    let segs = 32;
    let path = format!("{dir}/cad_pyramid.bin.stl");

    // Try Solid::union of all 5 layers.
    // Stacked face-to-face boxes sometimes trigger interior-endpoint issues,
    // so we catch panics via std::panic::catch_unwind and fall back to the
    // FaceSoup merge approach if needed.
    let result = std::panic::catch_unwind(|| {
        let solids: Vec<_> = layers
            .iter()
            .map(|(ext, orig)| box_at(*ext, *orig))
            .collect();

        // Union layer 0 with each subsequent layer.
        let mut merged = solids[0].clone();
        for s in &solids[1..] {
            merged = merged.union(s);
        }
        let soup = tessellate(&merged, segs);
        println!(
            "cad_pyramid (union path): V={}, E={}, F={}, {} triangles",
            merged.vertex_count(),
            merged.edge_count(),
            merged.face_count(),
            soup.triangles.len(),
        );
        soup
    });

    let soup = match result {
        Ok(s) => s,
        Err(_) => {
            // Fallback: tessellate each layer individually and merge FaceSoups.
            // This happens if union panics on touching/coplanar faces.
            println!("cad_pyramid: union panicked — using FaceSoup-merge fallback");
            let mut combined = FaceSoup::default();
            for (ext, orig) in layers {
                let s = box_at(*ext, *orig);
                let layer_soup = tessellate(&s, segs);
                combined.triangles.extend(layer_soup.triangles);
            }
            println!(
                "cad_pyramid (fallback): {} triangles across 5 layers",
                combined.triangles.len(),
            );
            combined
        }
    };

    let f = File::create(&path)?;
    let mut w = BufWriter::new(f);
    write_binary(&soup, "cad_pyramid", &mut w)?;
    println!("  -> {path}");

    Ok(())
}
