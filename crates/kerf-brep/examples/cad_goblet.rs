//! M24 CAD example — wineglass / goblet via revolve_polyline.
//!
//! 9-point profile: heavy disc base, tapered narrow stem, flared bowl, open
//! rim. Note: revolve_polyline requires endpoints on the z-axis, so the rim
//! is closed with a thin top apex (not actually open — STL would otherwise
//! be non-manifold). The "open" appearance comes from the very thin upper lip.

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
        Point3::new(0.00, 0.0, 4.40), // top apex (closes rim)
        Point3::new(0.85, 0.0, 4.30), // outer rim lip
        Point3::new(0.95, 0.0, 3.50), // bowl widest
        Point3::new(0.70, 0.0, 2.60), // bowl base curve
        Point3::new(0.18, 0.0, 2.20), // bowl-stem joint
        Point3::new(0.12, 0.0, 1.20), // stem
        Point3::new(0.18, 0.0, 0.30), // base joint
        Point3::new(0.80, 0.0, 0.10), // disc base
        Point3::new(0.00, 0.0, 0.00), // bottom apex
    ];

    let goblet = revolve_polyline(&profile);
    println!(
        "cad_goblet: V={}, E={}, F={}",
        goblet.vertex_count(),
        goblet.edge_count(),
        goblet.face_count(),
    );

    let soup = tessellate(&goblet, 48);
    println!("cad_goblet: {} triangles", soup.triangles.len());

    let path = format!("{dir}/cad_goblet.bin.stl");
    let mut w = BufWriter::new(File::create(&path)?);
    write_binary(&soup, "cad_goblet", &mut w)?;
    println!("  -> {path}");
    Ok(())
}
