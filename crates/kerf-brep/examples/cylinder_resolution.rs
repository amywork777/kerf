//! M23: faceted-cylinder mesh-resolution sweep.
//!
//! Builds n-gon prism approximations at n ∈ {4, 8, 16, 32}, lays them out in
//! a row, and emits an STL. Demonstrates how `cylinder_faceted` converges to
//! a true cylinder as n grows — and how every output stays purely planar
//! (so it cooperates with the planar-only boolean pipeline where edges line
//! up).

use std::fs::File;
use std::io::BufWriter;

use kerf_brep::booleans::FaceSoup;
use kerf_brep::primitives::cylinder_faceted;
use kerf_brep::tessellate::tessellate;
use kerf_brep::{write_ascii, write_binary};
use kerf_geom::Vec3;

fn translate_soup(mut soup: FaceSoup, offset: Vec3) -> FaceSoup {
    for tri in &mut soup.triangles {
        tri[0] += offset;
        tri[1] += offset;
        tri[2] += offset;
    }
    soup
}

fn main() -> std::io::Result<()> {
    let mut combined = FaceSoup::default();
    let spacing = 3.0;
    let resolutions = [4usize, 8, 16, 32];

    for (i, n) in resolutions.iter().enumerate() {
        let s = cylinder_faceted(1.0, 2.0, *n);
        let soup = translate_soup(tessellate(&s, 8), Vec3::new(spacing * i as f64, 0.0, 0.0));
        let tri_count = soup.triangles.len();
        combined.triangles.extend(soup.triangles);
        println!(
            "n={n:2}  V={:3} E={:3} F={:3}  triangles={}",
            s.vertex_count(),
            s.edge_count(),
            s.face_count(),
            tri_count,
        );
    }

    println!("Total triangles: {}", combined.triangles.len());

    let bin_path = "/tmp/claude/kerf_gallery/cylinder_resolution.bin.stl";
    let mut bin_writer = BufWriter::new(File::create(bin_path)?);
    write_binary(&combined, "kerf_cylinder_resolution", &mut bin_writer)?;
    println!("wrote {bin_path}");

    let ascii_path = "/tmp/claude/kerf_gallery/cylinder_resolution.stl";
    let mut ascii_writer = BufWriter::new(File::create(ascii_path)?);
    write_ascii(&combined, "kerf_cylinder_resolution", &mut ascii_writer)?;
    println!("wrote {ascii_path}");

    Ok(())
}
