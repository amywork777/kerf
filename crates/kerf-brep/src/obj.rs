//! Wavefront OBJ output for FaceSoup. Mesh format with vertex deduplication.

use std::io::{self, Write};

use kerf_geom::{Point3, Tolerance};

use crate::booleans::FaceSoup;

/// Write `soup` as Wavefront OBJ to `w`. Vertices are deduplicated within
/// `tol.point_eq` so the file is compact even for highly-shared meshes.
pub fn write_obj(soup: &FaceSoup, name: &str, w: &mut impl Write) -> io::Result<()> {
    writeln!(w, "# kerf OBJ output")?;
    writeln!(w, "o {name}")?;

    let tol = Tolerance::default();
    let mut positions: Vec<Point3> = Vec::new();
    let mut tri_indices: Vec<[usize; 3]> = Vec::with_capacity(soup.triangles.len());

    for tri in &soup.triangles {
        let mut idx = [0usize; 3];
        for i in 0..3 {
            let p = tri[i];
            let found = positions
                .iter()
                .position(|q| (p - *q).norm() < tol.point_eq);
            idx[i] = match found {
                Some(j) => j + 1, // OBJ is 1-indexed
                None => {
                    positions.push(p);
                    positions.len()
                }
            };
        }
        tri_indices.push(idx);
    }

    for p in &positions {
        writeln!(w, "v {} {} {}", p.x, p.y, p.z)?;
    }

    for idx in &tri_indices {
        writeln!(w, "f {} {} {}", idx[0], idx[1], idx[2])?;
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use kerf_geom::Vec3;

    use crate::primitives::box_;
    use crate::tessellate::tessellate;

    #[test]
    fn obj_writes_header_and_vertices() {
        let s = box_(Vec3::new(1.0, 1.0, 1.0));
        let soup = tessellate(&s, 4);
        let mut buf = Vec::new();
        write_obj(&soup, "test_box", &mut buf).unwrap();
        let s = String::from_utf8(buf).unwrap();
        assert!(s.starts_with("# kerf OBJ output"));
        assert!(s.contains("o test_box"));
        // Vertex lines start with "v ".
        let v_count = s.lines().filter(|l| l.starts_with("v ")).count();
        // Face lines start with "f ".
        let f_count = s.lines().filter(|l| l.starts_with("f ")).count();
        // A box has 8 unique vertices and 12 triangles.
        assert_eq!(v_count, 8, "expected 8 unique vertices, got {v_count}");
        assert_eq!(f_count, 12, "expected 12 face triangles, got {f_count}");
    }

    #[test]
    fn obj_dedupes_vertices_correctly() {
        // Two adjacent triangles sharing an edge → 4 unique vertices, 2 triangles.
        let soup = FaceSoup {
            triangles: vec![
                [
                    Point3::new(0.0, 0.0, 0.0),
                    Point3::new(1.0, 0.0, 0.0),
                    Point3::new(0.0, 1.0, 0.0),
                ],
                [
                    Point3::new(1.0, 0.0, 0.0),
                    Point3::new(1.0, 1.0, 0.0),
                    Point3::new(0.0, 1.0, 0.0),
                ],
            ],
        };
        let mut buf = Vec::new();
        write_obj(&soup, "quad", &mut buf).unwrap();
        let out = String::from_utf8(buf).unwrap();
        let v_count = out.lines().filter(|l| l.starts_with("v ")).count();
        let f_count = out.lines().filter(|l| l.starts_with("f ")).count();
        assert_eq!(v_count, 4, "expected 4 deduped vertices, got {v_count}");
        assert_eq!(f_count, 2);
    }

    #[test]
    fn obj_indices_are_1_indexed() {
        let soup = FaceSoup {
            triangles: vec![[
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(0.0, 1.0, 0.0),
            ]],
        };
        let mut buf = Vec::new();
        write_obj(&soup, "tri", &mut buf).unwrap();
        let out = String::from_utf8(buf).unwrap();
        // Should have "f 1 2 3" line.
        assert!(out.contains("f 1 2 3"), "expected 1-indexed face line, got:\n{out}");
    }
}
