//! Helper: emit curved-primitive STL inputs for CLI testing.

use std::fs::File;
use std::io::BufWriter;

use kerf_brep::primitives::{box_at, cylinder_faceted};
use kerf_brep::tessellate::tessellate;
use kerf_brep::write_binary;
use kerf_geom::{Point3, Vec3};

fn main() -> std::io::Result<()> {
    let dir = std::env::var("KERF_GALLERY_DIR")
        .unwrap_or_else(|_| "/tmp/claude/kerf_gallery".to_string());
    std::fs::create_dir_all(&dir)?;

    let block = box_at(Vec3::new(2.0, 2.0, 2.0), Point3::new(-1.0, -1.0, -1.0));
    let cyl = cylinder_faceted(0.6, 3.0, 12);
    for (name, solid) in [("cli_block", &block), ("cli_cyl12", &cyl)] {
        let soup = tessellate(solid, 8);
        let path = format!("{dir}/{name}.bin.stl");
        let mut w = BufWriter::new(File::create(&path)?);
        write_binary(&soup, name, &mut w)?;
        println!("wrote {path}");
    }
    Ok(())
}
