//! Write corner-overlapping boxes for CLI testing.

use std::fs::File;
use std::io::BufWriter;

use kerf_brep::primitives::{box_, box_at};
use kerf_brep::tessellate::tessellate;
use kerf_brep::write_binary;
use kerf_geom::{Point3, Vec3};

fn main() -> std::io::Result<()> {
    let dir = std::env::var("KERF_GALLERY_DIR")
        .unwrap_or_else(|_| "/tmp/claude/kerf_gallery".to_string());
    std::fs::create_dir_all(&dir)?;

    let a = box_(Vec3::new(2.0, 2.0, 2.0));
    let b = box_at(Vec3::new(2.0, 2.0, 2.0), Point3::new(1.0, 1.0, 1.0));
    for (name, solid) in [("cli_corner_a", &a), ("cli_corner_b", &b)] {
        let soup = tessellate(solid, 8);
        let path = format!("{dir}/{name}.bin.stl");
        let mut w = BufWriter::new(File::create(&path)?);
        write_binary(&soup, name, &mut w)?;
        println!("wrote {path}");
    }
    Ok(())
}
