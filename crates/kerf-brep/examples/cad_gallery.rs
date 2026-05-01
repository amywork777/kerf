//! CAD gallery — produces STL files for every kernel capability so they can be
//! rendered to images for visual verification.

use std::fs::File;
use std::io::BufWriter;
use std::path::Path;

use kerf_brep::booleans::{boolean, BooleanOp, FaceSoup};
use kerf_brep::primitives::{box_, box_at, cone, cylinder, extrude_polygon, frustum, revolve_polyline, sphere, torus};
use kerf_brep::tessellate::tessellate;
use kerf_brep::{write_binary, Solid};
use kerf_geom::{Point3, Tolerance, Vec3};

fn write_stl(soup: &FaceSoup, name: &str, dir: &str) -> std::io::Result<()> {
    let path = format!("{dir}/{name}.bin.stl");
    let f = File::create(&path)?;
    let mut w = BufWriter::new(f);
    write_binary(soup, name, &mut w)?;
    println!("  → {path} ({} triangles)", soup.triangles.len());
    Ok(())
}

fn make_box_at(extents: Vec3, origin: Point3) -> Solid {
    box_at(extents, origin)
}

fn main() -> std::io::Result<()> {
    let dir = std::env::var("KERF_GALLERY_DIR").unwrap_or_else(|_| "/tmp/claude/kerf_gallery".to_string());
    std::fs::create_dir_all(&dir)?;
    let segs = 32;
    println!("Writing CAD gallery STL files to {dir}");

    // ── Primitives ──
    println!("\n[primitives]");
    write_stl(&tessellate(&box_(Vec3::new(2.0, 2.0, 2.0)), segs), "01_box", &dir)?;

    let triangle = vec![
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(2.0, 0.0, 0.0),
        Point3::new(1.0, 1.7, 0.0),
    ];
    write_stl(&tessellate(&extrude_polygon(&triangle, Vec3::new(0.0, 0.0, 2.0)), segs), "02_prism", &dir)?;

    write_stl(&tessellate(&cylinder(1.0, 2.0), segs), "03_cylinder", &dir)?;
    write_stl(&tessellate(&cone(1.0, 2.0), segs), "04_cone", &dir)?;
    write_stl(&tessellate(&sphere(1.0), segs), "05_sphere", &dir)?;
    write_stl(&tessellate(&torus(1.5, 0.4), segs), "06_torus", &dir)?;
    write_stl(&tessellate(&frustum(0.5, 1.5, 2.0), segs), "07_frustum", &dir)?;

    // Polygon revolve — vase-like shape.
    let vase_profile = vec![
        Point3::new(0.0, 0.0, 3.0),
        Point3::new(0.4, 0.0, 2.8),
        Point3::new(1.0, 0.0, 2.0),
        Point3::new(0.6, 0.0, 1.0),
        Point3::new(1.2, 0.0, 0.4),
        Point3::new(0.0, 0.0, 0.0),
    ];
    write_stl(&tessellate(&revolve_polyline(&vase_profile), segs), "08_revolve_vase", &dir)?;

    // ── Booleans ──
    println!("\n[booleans]");

    // Union of two prisms (M9 demo's spirit).
    let a = box_(Vec3::new(2.0, 2.0, 2.0));
    let b = make_box_at(Vec3::new(2.0, 2.0, 2.0), Point3::new(1.0, 0.0, 0.0));
    write_stl(&boolean(&a, &b, BooleanOp::Union, &Tolerance::default()), "10_union_two_boxes", &dir)?;

    // Intersection of two boxes.
    write_stl(&boolean(&a, &b, BooleanOp::Intersection, &Tolerance::default()), "11_intersection_two_boxes", &dir)?;

    // Corner-cut Difference (M11).
    let block = box_(Vec3::new(4.0, 4.0, 4.0));
    let cutter = make_box_at(Vec3::new(2.0, 2.0, 2.0), Point3::new(3.0, 3.0, 3.0));
    let stepped = block.difference(&cutter);
    write_stl(&tessellate(&stepped, segs), "12_corner_cut_step", &dir)?;

    // Hollow nested Difference (M12).
    let big = box_(Vec3::new(4.0, 4.0, 4.0));
    let small_inside = make_box_at(Vec3::new(2.0, 2.0, 2.0), Point3::new(1.0, 1.0, 1.0));
    let hollow = big.difference(&small_inside);
    write_stl(&tessellate(&hollow, segs), "13_hollow_box", &dir)?;

    // Recursive boolean (M8) — chain two simple shifted-overlap intersections.
    // Using shifted boxes that overlap cleanly (no interior endpoints).
    let a = box_(Vec3::new(4.0, 4.0, 4.0));
    let b = make_box_at(Vec3::new(4.0, 4.0, 4.0), Point3::new(2.0, 0.0, 0.0));
    let c = make_box_at(Vec3::new(4.0, 4.0, 4.0), Point3::new(0.0, 2.0, 0.0));
    let ab = a.intersection(&b);
    let abc = ab.intersection(&c);
    write_stl(&tessellate(&abc, segs), "14_recursive_intersection", &dir)?;

    println!("\nDone.");
    let _ = Path::new(&dir);
    Ok(())
}
