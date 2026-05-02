//! M24 CAD example — bowling pin via revolve_polyline.
//!
//! 8-point profile: pointed top, dome shoulder, narrow neck, broad belly, flat
//! base. Revolved at 32 segments around the z-axis.

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

    let profile = vec![
        Point3::new(0.00, 0.0, 3.80), // top apex
        Point3::new(0.18, 0.0, 3.60), // shoulder
        Point3::new(0.28, 0.0, 3.20), // upper bell
        Point3::new(0.18, 0.0, 2.50), // neck
        Point3::new(0.45, 0.0, 1.70), // upper belly
        Point3::new(0.55, 0.0, 0.80), // lower belly
        Point3::new(0.45, 0.0, 0.10), // base flare
        Point3::new(0.00, 0.0, 0.00), // bottom apex
    ];

    let pin = revolve_polyline(&profile);
    println!(
        "cad_bowling_pin: V={}, E={}, F={}",
        pin.vertex_count(),
        pin.edge_count(),
        pin.face_count(),
    );

    let soup = tessellate(&pin, 32);
    println!("cad_bowling_pin: {} triangles", soup.triangles.len());

    let path = format!("{dir}/cad_bowling_pin.bin.stl");
    let mut w = BufWriter::new(File::create(&path)?);
    write_binary(&soup, "cad_bowling_pin", &mut w)?;
    println!("  -> {path}");
    Ok(())
}
