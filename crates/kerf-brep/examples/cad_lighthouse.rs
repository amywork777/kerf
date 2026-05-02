//! M24 CAD example — stepped lighthouse tower via revolve_polyline.
//!
//! Multi-frustum stack: wide foundation, two tapered tiers, narrow lamp
//! housing, conical roof. Demonstrates revolve_polyline handling many
//! interior segments — 12 points produce 22 edges and 11 lateral faces.

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
        Point3::new(0.00, 0.0, 5.50), // tip
        Point3::new(0.30, 0.0, 4.90), // roof base
        Point3::new(0.55, 0.0, 4.85), // roof rim eave
        Point3::new(0.55, 0.0, 4.30), // lamp housing top
        Point3::new(0.40, 0.0, 4.20), // lamp housing under
        Point3::new(0.40, 0.0, 3.80), // gallery deck top
        Point3::new(0.55, 0.0, 3.70), // gallery deck out
        Point3::new(0.55, 0.0, 0.80), // tower main shaft
        Point3::new(0.95, 0.0, 0.70), // foundation step out
        Point3::new(0.95, 0.0, 0.20), // foundation side
        Point3::new(0.85, 0.0, 0.05), // chamfer
        Point3::new(0.00, 0.0, 0.00), // base
    ];

    let tower = revolve_polyline(&profile);
    println!(
        "cad_lighthouse: V={}, E={}, F={}",
        tower.vertex_count(),
        tower.edge_count(),
        tower.face_count(),
    );

    let soup = tessellate(&tower, 32);
    println!("cad_lighthouse: {} triangles", soup.triangles.len());

    let path = format!("{dir}/cad_lighthouse.bin.stl");
    let mut w = BufWriter::new(File::create(&path)?);
    write_binary(&soup, "cad_lighthouse", &mut w)?;
    println!("  -> {path}");
    Ok(())
}
