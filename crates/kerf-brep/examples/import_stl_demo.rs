//! M25 demo: write a box to STL, read it back into a Solid, re-export to OBJ.
//!
//! Demonstrates the round-trip box → STL bytes → Solid (12 triangle faces,
//! 18 edges, 8 deduped vertices) → OBJ. Imports any closed orientable
//! triangle mesh, not just kerf's own primitives.

use std::fs::File;
use std::io::BufWriter;

use kerf_brep::primitives::box_;
use kerf_brep::tessellate::tessellate;
use kerf_brep::{read_stl_binary_to_solid, write_binary, write_obj};
use kerf_geom::Vec3;

fn main() -> std::io::Result<()> {
    let dir = std::env::var("KERF_GALLERY_DIR")
        .unwrap_or_else(|_| "/tmp/claude/kerf_gallery".to_string());
    std::fs::create_dir_all(&dir)?;

    let s = box_(Vec3::new(2.0, 3.0, 4.0));
    let soup = tessellate(&s, 8);
    let mut stl_bytes = Vec::new();
    write_binary(&soup, "import_demo", &mut stl_bytes)?;
    println!(
        "wrote {} STL bytes ({} triangles)",
        stl_bytes.len(),
        soup.triangles.len()
    );

    let imported =
        read_stl_binary_to_solid(&mut stl_bytes.as_slice()).expect("import succeeds for closed box");
    println!(
        "imported solid: V={}, E={}, F={}",
        imported.vertex_count(),
        imported.edge_count(),
        imported.face_count()
    );

    // Re-tessellate the imported solid and write it back as OBJ via FaceSoup.
    let imported_soup = tessellate(&imported, 8);
    let obj_path = format!("{dir}/import_stl_demo.obj");
    let mut w = BufWriter::new(File::create(&obj_path)?);
    write_obj(&imported_soup, "import_demo", &mut w)?;
    println!("re-exported {} triangles to {obj_path}", imported_soup.triangles.len());

    Ok(())
}
