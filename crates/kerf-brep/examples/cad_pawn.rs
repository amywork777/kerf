//! M22a CAD example 1 — chess pawn via revolve_polyline.
//!
//! A 7-point profile in the xz-plane is revolved 360° around the z-axis to
//! produce a recognizable chess-pawn silhouette:
//!
//!   [(0,0,2.5), (0.25,0,2.4), (0.4,0,2.0), (0.25,0,1.5), (0.5,0,0.8), (0.6,0,0.2), (0,0,0)]
//!
//! Reading bottom-to-top: flat bottom apex, wide flared base, belly bulge,
//! narrow waist, curved dome, top apex. Tessellated at 32 segments.

use std::fs::File;
use std::io::BufWriter;

use kerf_brep::primitives::revolve_polyline;
use kerf_brep::tessellate::tessellate;
use kerf_brep::write_binary;
use kerf_geom::Point3;

fn main() -> std::io::Result<()> {
    let dir = std::env::var("KERF_GALLERY_DIR")
        .unwrap_or_else(|_| "/tmp/claude/kerf_gallery".to_string());
    std::fs::create_dir_all(&dir)?;

    // 7-point pawn profile in the xz-plane.
    // Endpoints are on the z-axis (x=0) — required by revolve_polyline.
    let profile = vec![
        Point3::new(0.0,  0.0, 2.5),  // top apex
        Point3::new(0.25, 0.0, 2.4),  // dome curve
        Point3::new(0.4,  0.0, 2.0),  // dome equator
        Point3::new(0.25, 0.0, 1.5),  // neck waist
        Point3::new(0.5,  0.0, 0.8),  // belly bulge
        Point3::new(0.6,  0.0, 0.2),  // base flare
        Point3::new(0.0,  0.0, 0.0),  // bottom apex
    ];

    let pawn = revolve_polyline(&profile);
    println!(
        "cad_pawn: V={}, E={}, F={}",
        pawn.vertex_count(),
        pawn.edge_count(),
        pawn.face_count(),
    );

    let segs = 32;
    let soup = tessellate(&pawn, segs);
    println!("cad_pawn: {} triangles", soup.triangles.len());

    let path = format!("{dir}/cad_pawn.bin.stl");
    let f = File::create(&path)?;
    let mut w = BufWriter::new(f);
    write_binary(&soup, "cad_pawn", &mut w)?;
    println!("  -> {path}");

    Ok(())
}
