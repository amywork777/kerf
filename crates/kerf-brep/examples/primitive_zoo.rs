//! M18 primitive zoo: every primitive tessellated and emitted to a single STL.
//!
//! Lay out all 7 primitives in a grid, each translated to its own (x,y) slot.
//! Tessellate each into a FaceSoup, concatenate triangles, write ASCII + binary STL.

use std::fs::File;
use std::io::BufWriter;

use kerf_brep::booleans::FaceSoup;
use kerf_brep::primitives::{box_, cone, cylinder, extrude_polygon, frustum, sphere, torus};
use kerf_brep::tessellate::tessellate;
use kerf_brep::{write_ascii, write_binary};
use kerf_geom::{Point3, Vec3};

fn translate_soup(mut soup: FaceSoup, offset: Vec3) -> FaceSoup {
    for tri in &mut soup.triangles {
        tri[0] += offset;
        tri[1] += offset;
        tri[2] += offset;
    }
    soup
}

fn main() -> std::io::Result<()> {
    let segs = 24;
    let mut zoo = FaceSoup::default();

    // 1. Box at (0, 0).
    let s = box_(Vec3::new(2.0, 2.0, 2.0));
    let soup = translate_soup(tessellate(&s, segs), Vec3::new(0.0, 0.0, 0.0));
    println!("box: {} triangles", soup.triangles.len());
    zoo.triangles.extend(soup.triangles);

    // 2. Triangular prism at (5, 0).
    let triangle = vec![
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(2.0, 0.0, 0.0),
        Point3::new(1.0, 1.7, 0.0),
    ];
    let s = extrude_polygon(&triangle, Vec3::new(0.0, 0.0, 2.0));
    let soup = translate_soup(tessellate(&s, segs), Vec3::new(5.0, 0.0, 0.0));
    println!("prism: {} triangles", soup.triangles.len());
    zoo.triangles.extend(soup.triangles);

    // 3. Cylinder at (10, 0).
    let s = cylinder(1.0, 2.0);
    let soup = translate_soup(tessellate(&s, segs), Vec3::new(10.0, 0.0, 0.0));
    println!("cylinder: {} triangles", soup.triangles.len());
    zoo.triangles.extend(soup.triangles);

    // 4. Cone at (15, 0).
    let s = cone(1.0, 2.0);
    let soup = translate_soup(tessellate(&s, segs), Vec3::new(15.0, 0.0, 0.0));
    println!("cone: {} triangles", soup.triangles.len());
    zoo.triangles.extend(soup.triangles);

    // 5. Sphere at (0, 5).
    let s = sphere(1.0);
    let soup = translate_soup(tessellate(&s, segs), Vec3::new(0.0, 5.0, 1.0));
    println!("sphere: {} triangles", soup.triangles.len());
    zoo.triangles.extend(soup.triangles);

    // 6. Torus at (5, 5).
    let s = torus(1.5, 0.4);
    let soup = translate_soup(tessellate(&s, segs), Vec3::new(5.0, 5.0, 0.5));
    println!("torus: {} triangles", soup.triangles.len());
    zoo.triangles.extend(soup.triangles);

    // 7. Frustum at (10, 5).
    let s = frustum(0.5, 1.5, 2.0);
    let soup = translate_soup(tessellate(&s, segs), Vec3::new(10.0, 5.0, 0.0));
    println!("frustum: {} triangles", soup.triangles.len());
    zoo.triangles.extend(soup.triangles);

    println!("zoo total: {} triangles", zoo.triangles.len());

    // Write STL.
    let ascii_path = "/tmp/claude/kerf_zoo.stl";
    let bin_path = "/tmp/claude/kerf_zoo.bin.stl";
    {
        let f = File::create(ascii_path)?;
        let mut w = BufWriter::new(f);
        write_ascii(&zoo, "kerf_zoo", &mut w)?;
    }
    {
        let f = File::create(bin_path)?;
        let mut w = BufWriter::new(f);
        write_binary(&zoo, "kerf_zoo", &mut w)?;
    }
    println!("Wrote {ascii_path}");
    println!("Wrote {bin_path}");

    // Verify binary STL size formula: 80 + 4 + 50 * total_triangles.
    let expected_bin_size = 80 + 4 + 50 * zoo.triangles.len();
    let actual_bin_size = std::fs::metadata(bin_path)?.len() as usize;
    assert_eq!(
        actual_bin_size, expected_bin_size,
        "binary STL size mismatch: got {actual_bin_size}, expected {expected_bin_size}",
    );
    println!(
        "Binary STL: {} bytes = 80 + 4 + 50×{} ✓",
        actual_bin_size,
        zoo.triangles.len()
    );

    Ok(())
}
