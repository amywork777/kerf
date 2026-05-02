//! Wavefront OBJ I/O for FaceSoup. Mesh format with vertex deduplication.

use std::io::{self, BufRead, BufReader, Read, Write};

use kerf_geom::{Point3, Tolerance};

use crate::booleans::FaceSoup;
use crate::mesh_import::MeshImportError;

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

/// Read a Wavefront OBJ file into a `FaceSoup`. Quads and n-gons are
/// triangulated via fan from the first vertex. Supports `f i/uv/n` slash form;
/// only the position index is used. Texture coords (`vt`), normals (`vn`),
/// groups (`g`), materials (`mtllib`/`usemtl`), and other entities are ignored.
pub fn read_obj(r: &mut impl Read) -> Result<FaceSoup, MeshImportError> {
    let reader = BufReader::new(r);
    let mut positions: Vec<Point3> = Vec::new();
    let mut tris: Vec<[Point3; 3]> = Vec::new();

    for line in reader.lines() {
        let line = line.map_err(MeshImportError::Io)?;
        let trimmed = line.trim();
        if trimmed.is_empty() || trimmed.starts_with('#') {
            continue;
        }
        let mut parts = trimmed.split_whitespace();
        let head = match parts.next() {
            Some(h) => h,
            None => continue,
        };
        match head {
            "v" => {
                let coords: Vec<f64> = parts
                    .take(3)
                    .map(|s| {
                        s.parse::<f64>()
                            .map_err(|_| MeshImportError::AsciiParse(format!("bad v float {s:?}")))
                    })
                    .collect::<Result<_, _>>()?;
                if coords.len() != 3 {
                    return Err(MeshImportError::AsciiParse(format!(
                        "vertex needs 3 floats: {trimmed:?}"
                    )));
                }
                positions.push(Point3::new(coords[0], coords[1], coords[2]));
            }
            "f" => {
                // Each token is `i`, `i/uv`, `i/uv/n`, or `i//n`. We want the
                // first slash-separated component (1-indexed position).
                let face_indices: Vec<usize> = parts
                    .map(|tok| {
                        let pos_idx = tok.split('/').next().unwrap_or("");
                        let i = pos_idx.parse::<i64>().map_err(|_| {
                            MeshImportError::AsciiParse(format!("bad face index {tok:?}"))
                        })?;
                        // OBJ supports negative (relative) indices: -1 means the
                        // last vertex defined so far.
                        let resolved = if i > 0 {
                            (i as usize) - 1
                        } else if i < 0 {
                            let n = positions.len() as i64;
                            let abs = -i;
                            if abs > n {
                                return Err(MeshImportError::AsciiParse(format!(
                                    "negative face index {i} out of range"
                                )));
                            }
                            (n - abs) as usize
                        } else {
                            return Err(MeshImportError::AsciiParse(
                                "face index 0 is invalid in OBJ (1-indexed)".into(),
                            ));
                        };
                        if resolved >= positions.len() {
                            return Err(MeshImportError::AsciiParse(format!(
                                "face index {} out of range (have {} verts)",
                                resolved + 1,
                                positions.len()
                            )));
                        }
                        Ok(resolved)
                    })
                    .collect::<Result<_, _>>()?;
                if face_indices.len() < 3 {
                    return Err(MeshImportError::AsciiParse(format!(
                        "face needs ≥ 3 vertices, got {}",
                        face_indices.len()
                    )));
                }
                // Fan-triangulate from face_indices[0].
                for k in 1..face_indices.len() - 1 {
                    tris.push([
                        positions[face_indices[0]],
                        positions[face_indices[k]],
                        positions[face_indices[k + 1]],
                    ]);
                }
            }
            // Ignore all other directives.
            _ => {}
        }
    }
    Ok(FaceSoup { triangles: tris })
}

/// Read an OBJ stream directly into a `Solid`.
pub fn read_obj_to_solid(r: &mut impl Read) -> Result<crate::Solid, MeshImportError> {
    let soup = read_obj(r)?;
    crate::mesh_import::from_triangles(&soup.triangles)
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
    fn round_trip_box_through_obj() {
        let s = box_(Vec3::new(1.0, 2.0, 3.0));
        let soup = tessellate(&s, 8);
        let mut buf = Vec::new();
        write_obj(&soup, "round_trip", &mut buf).unwrap();
        let s2 = read_obj_to_solid(&mut buf.as_slice()).unwrap();
        assert_eq!(s2.vertex_count(), 8);
        assert_eq!(s2.edge_count(), 18);
        assert_eq!(s2.face_count(), 12);
    }

    #[test]
    fn read_obj_quad_face_triangulates() {
        // Hand-written OBJ: a unit square as a single quad face.
        let obj = "v 0 0 0\nv 1 0 0\nv 1 1 0\nv 0 1 0\nf 1 2 3 4\n";
        let soup = read_obj(&mut obj.as_bytes()).unwrap();
        assert_eq!(soup.triangles.len(), 2, "quad → 2 triangles via fan");
    }

    #[test]
    fn read_obj_handles_slash_form_and_negative_indices() {
        let obj = "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1/1/1 2/2/1 3/3/1\n";
        let soup = read_obj(&mut obj.as_bytes()).unwrap();
        assert_eq!(soup.triangles.len(), 1);

        let obj = "v 0 0 0\nv 1 0 0\nv 0 1 0\nf -3 -2 -1\n";
        let soup = read_obj(&mut obj.as_bytes()).unwrap();
        assert_eq!(soup.triangles.len(), 1);
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
