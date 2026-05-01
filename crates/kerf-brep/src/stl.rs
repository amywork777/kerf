//! STL export for `FaceSoup`. Supports ASCII (.stl) and binary (.stl) formats.

use std::io::{self, Write};

use kerf_geom::{Point3, Vec3};

use crate::booleans::FaceSoup;

/// Compute the unit normal of a triangle via cross product.
/// Returns Vec3::zeros() for degenerate triangles.
fn triangle_normal(tri: &[Point3; 3]) -> Vec3 {
    let e1 = tri[1] - tri[0];
    let e2 = tri[2] - tri[0];
    e1.cross(&e2).try_normalize(1e-15).unwrap_or_else(Vec3::zeros)
}

/// Write `soup` as ASCII STL to `w`. `name` is the solid identifier in the file header.
pub fn write_ascii(soup: &FaceSoup, name: &str, w: &mut impl Write) -> io::Result<()> {
    writeln!(w, "solid {name}")?;
    for tri in &soup.triangles {
        let n = triangle_normal(tri);
        writeln!(w, "  facet normal {} {} {}", n.x, n.y, n.z)?;
        writeln!(w, "    outer loop")?;
        for v in tri {
            writeln!(w, "      vertex {} {} {}", v.x, v.y, v.z)?;
        }
        writeln!(w, "    endloop")?;
        writeln!(w, "  endfacet")?;
    }
    writeln!(w, "endsolid {name}")?;
    Ok(())
}

/// Write `soup` as binary STL to `w`. The 80-byte header carries `name`
/// (truncated/padded as needed).
pub fn write_binary(soup: &FaceSoup, name: &str, w: &mut impl Write) -> io::Result<()> {
    // 80-byte header.
    let mut header = [0u8; 80];
    let name_bytes = name.as_bytes();
    let n = name_bytes.len().min(80);
    header[..n].copy_from_slice(&name_bytes[..n]);
    w.write_all(&header)?;

    // Triangle count (u32 LE).
    let count = u32::try_from(soup.triangles.len()).expect("too many triangles for STL");
    w.write_all(&count.to_le_bytes())?;

    for tri in &soup.triangles {
        let n = triangle_normal(tri);
        // Normal: 3 floats LE.
        for &c in &[n.x as f32, n.y as f32, n.z as f32] {
            w.write_all(&c.to_le_bytes())?;
        }
        // 3 vertices, each 3 floats LE.
        for v in tri {
            for &c in &[v.x as f32, v.y as f32, v.z as f32] {
                w.write_all(&c.to_le_bytes())?;
            }
        }
        // Attribute byte count: 2 bytes, always 0.
        w.write_all(&[0u8, 0u8])?;
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::booleans::{boolean, BooleanOp};
    use crate::geometry::{CurveKind, SurfaceKind};
    use crate::primitives::box_;
    use crate::Solid;
    use kerf_geom::{Tolerance, Vec3};

    fn unit_tri_soup() -> FaceSoup {
        FaceSoup {
            triangles: vec![[
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(0.0, 1.0, 0.0),
            ]],
        }
    }

    fn make_box_at(extents: Vec3, offset: Vec3) -> Solid {
        let mut s = box_(extents);
        for (_, p) in s.vertex_geom.iter_mut() {
            *p += offset;
        }
        for (_, surf) in s.face_geom.iter_mut() {
            if let SurfaceKind::Plane(plane) = surf {
                plane.frame.origin += offset;
            }
        }
        for (_, seg) in s.edge_geom.iter_mut() {
            if let CurveKind::Line(line) = &mut seg.curve {
                line.origin += offset;
            }
        }
        s
    }

    #[test]
    fn ascii_writer_emits_solid_header_and_endsolid() {
        let soup = unit_tri_soup();
        let mut buf = Vec::new();
        write_ascii(&soup, "test", &mut buf).unwrap();
        let s = String::from_utf8(buf).unwrap();
        assert!(s.starts_with("solid test"), "got: {s:?}");
        assert!(s.contains("endsolid test"));
        assert!(s.contains("facet normal"));
        // Three vertex lines per triangle.
        assert_eq!(s.matches("vertex").count(), 3);
    }

    #[test]
    fn binary_writer_size_matches_count() {
        let soup = unit_tri_soup();
        let mut buf = Vec::new();
        write_binary(&soup, "test", &mut buf).unwrap();
        // 80 (header) + 4 (count) + 50 per triangle.
        assert_eq!(buf.len(), 80 + 4 + 50);
        // Verify the count u32 is at offset 80.
        let count = u32::from_le_bytes([buf[80], buf[81], buf[82], buf[83]]);
        assert_eq!(count, 1);
    }

    #[test]
    fn boolean_to_stl_round_trip_count() {
        // Big - small (nested) → 24 triangles → 24 facets in ASCII, 80+4+24*50 bytes binary.
        let big = box_(Vec3::new(10.0, 10.0, 10.0));
        let small = make_box_at(Vec3::new(2.0, 2.0, 2.0), Vec3::new(4.0, 4.0, 4.0));
        let soup = boolean(&big, &small, BooleanOp::Difference, &Tolerance::default());
        assert_eq!(soup.triangles.len(), 24);

        let mut ascii = Vec::new();
        write_ascii(&soup, "diff", &mut ascii).unwrap();
        let ascii_s = String::from_utf8(ascii).unwrap();
        assert_eq!(ascii_s.matches("facet normal").count(), 24);

        let mut bin = Vec::new();
        write_binary(&soup, "diff", &mut bin).unwrap();
        assert_eq!(bin.len(), 80 + 4 + 24 * 50);
        let count = u32::from_le_bytes([bin[80], bin[81], bin[82], bin[83]]);
        assert_eq!(count, 24);
    }

    #[test]
    fn triangle_normal_is_unit_for_axis_aligned_triangle() {
        let tri = [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ];
        let n = triangle_normal(&tri);
        assert!((n - Vec3::z()).norm() < 1e-12, "got {:?}", n);
    }
}
