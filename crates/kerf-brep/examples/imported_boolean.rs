//! M26 demo: round-trip two boxes through STL, then run a boolean.
//!
//! Validates the kernel as a complete loop:
//!   build → tessellate → STL bytes → import → boolean → STL.
//!
//! Output: cad_imported_union.bin.stl, exact-same shape as the native
//! union demo but built from triangulated input rather than analytic
//! primitives.

use std::fs::File;
use std::io::BufWriter;

use kerf_brep::primitives::{box_, box_at};
use kerf_brep::tessellate::tessellate;
use kerf_brep::{read_stl_binary_to_solid, write_binary};
use kerf_geom::{Point3, Vec3};

fn main() -> std::io::Result<()> {
    let dir = std::env::var("KERF_GALLERY_DIR")
        .unwrap_or_else(|_| "/tmp/claude/kerf_gallery".to_string());
    std::fs::create_dir_all(&dir)?;

    // Build two native boxes and tessellate to STL bytes.
    let a_native = box_(Vec3::new(2.0, 2.0, 2.0));
    let b_native = box_at(Vec3::new(2.0, 2.0, 2.0), Point3::new(1.0, 0.0, 0.0));
    let mut a_bytes = Vec::new();
    let mut b_bytes = Vec::new();
    write_binary(&tessellate(&a_native, 8), "a", &mut a_bytes)?;
    write_binary(&tessellate(&b_native, 8), "b", &mut b_bytes)?;

    // Import each as a triangle-mesh B-rep.
    let a = read_stl_binary_to_solid(&mut a_bytes.as_slice())
        .expect("imported box A is closed manifold");
    let b = read_stl_binary_to_solid(&mut b_bytes.as_slice())
        .expect("imported box B is closed manifold");
    println!(
        "imported A: V={}, E={}, F={}",
        a.vertex_count(),
        a.edge_count(),
        a.face_count()
    );
    println!(
        "imported B: V={}, E={}, F={}",
        b.vertex_count(),
        b.edge_count(),
        b.face_count()
    );

    // Boolean on imported solids — same API as native primitives.
    let u = a.union(&b);
    kerf_topo::validate(&u.topo).expect("union output is valid");
    println!(
        "union result: V={}, E={}, F={}",
        u.vertex_count(),
        u.edge_count(),
        u.face_count()
    );

    // Write the result to STL for the gallery.
    let soup = tessellate(&u, 8);
    let path = format!("{dir}/cad_imported_union.bin.stl");
    let mut w = BufWriter::new(File::create(&path)?);
    write_binary(&soup, "imported_union", &mut w)?;
    println!("wrote {} triangles → {path}", soup.triangles.len());

    Ok(())
}
