//! 3MF (3D Manufacturing Format) export for `FaceSoup`.
//!
//! Produces a minimal, spec-conformant 3MF ZIP archive containing:
//!   - `_rels/.rels` — OPC relationship pointing to the 3D model
//!   - `[Content_Types].xml` — OPC content-type manifest
//!   - `3D/3dmodel.model` — mesh as vertices + triangles in 3MF XML schema
//!
//! The output is a `Vec<u8>` (ZIP archive). Vertices are deduplicated within
//! the default point-equality tolerance so the mesh is compact.

use std::io::Write;

use kerf_geom::{Point3, Tolerance};

use crate::booleans::FaceSoup;

/// Error type for 3MF export failures (IO errors from the zip writer).
#[derive(Debug)]
pub struct Export3mfError(pub String);

impl std::fmt::Display for Export3mfError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "3MF export error: {}", self.0)
    }
}

impl std::error::Error for Export3mfError {}

impl From<zip::result::ZipError> for Export3mfError {
    fn from(e: zip::result::ZipError) -> Self {
        Export3mfError(e.to_string())
    }
}

impl From<std::io::Error> for Export3mfError {
    fn from(e: std::io::Error) -> Self {
        Export3mfError(e.to_string())
    }
}

/// Write `soup` as a 3MF ZIP archive. Returns the raw bytes of the archive.
/// `name` is embedded as the mesh object name inside the .model XML.
pub fn write_3mf(soup: &FaceSoup, name: &str) -> Result<Vec<u8>, Export3mfError> {
    let tol = Tolerance::default();

    // Deduplicate vertices.
    let mut positions: Vec<Point3> = Vec::new();
    let mut triangles: Vec<[usize; 3]> = Vec::with_capacity(soup.triangles.len());

    for tri in &soup.triangles {
        let mut idx = [0usize; 3];
        for i in 0..3 {
            let p = tri[i];
            let found = positions
                .iter()
                .position(|q| (p - *q).norm() < tol.point_eq);
            idx[i] = match found {
                Some(j) => j,
                None => {
                    positions.push(p);
                    positions.len() - 1
                }
            };
        }
        triangles.push(idx);
    }

    // Build 3MF XML for 3D/3dmodel.model
    let model_xml = build_model_xml(name, &positions, &triangles);

    // OPC _rels/.rels
    let rels_xml = r#"<?xml version="1.0" encoding="UTF-8"?>
<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">
  <Relationship Target="/3D/3dmodel.model" Id="rel0" Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/>
</Relationships>"#;

    // OPC [Content_Types].xml
    let content_types_xml = r#"<?xml version="1.0" encoding="UTF-8"?>
<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">
  <Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>
  <Default Extension="model" ContentType="application/vnd.ms-package.3dmanufacturing-3dmodel+xml"/>
</Types>"#;

    // Write ZIP archive.
    let buf = Vec::new();
    let cursor = std::io::Cursor::new(buf);
    let mut zip = zip::ZipWriter::new(cursor);
    let options = zip::write::SimpleFileOptions::default()
        .compression_method(zip::CompressionMethod::Deflated);

    zip.start_file("[Content_Types].xml", options)?;
    zip.write_all(content_types_xml.as_bytes())?;

    zip.start_file("_rels/.rels", options)?;
    zip.write_all(rels_xml.as_bytes())?;

    zip.start_file("3D/3dmodel.model", options)?;
    zip.write_all(model_xml.as_bytes())?;

    let cursor = zip.finish()?;
    Ok(cursor.into_inner())
}

fn build_model_xml(name: &str, positions: &[Point3], triangles: &[[usize; 3]]) -> String {
    let mut xml = String::with_capacity(512 + positions.len() * 60 + triangles.len() * 40);
    xml.push_str("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
    xml.push_str("<model unit=\"millimeter\" xml:lang=\"en-US\" xmlns=\"http://schemas.microsoft.com/3dmanufacturing/core/2015/02\">\n");
    xml.push_str("  <resources>\n");
    xml.push_str(&format!("    <object id=\"1\" name=\"{name}\" type=\"model\">\n"));
    xml.push_str("      <mesh>\n");
    xml.push_str("        <vertices>\n");
    for p in positions {
        xml.push_str(&format!(
            "          <vertex x=\"{:.6}\" y=\"{:.6}\" z=\"{:.6}\"/>\n",
            p.x, p.y, p.z
        ));
    }
    xml.push_str("        </vertices>\n");
    xml.push_str("        <triangles>\n");
    for t in triangles {
        xml.push_str(&format!(
            "          <triangle v1=\"{}\" v2=\"{}\" v3=\"{}\"/>\n",
            t[0], t[1], t[2]
        ));
    }
    xml.push_str("        </triangles>\n");
    xml.push_str("      </mesh>\n");
    xml.push_str("    </object>\n");
    xml.push_str("  </resources>\n");
    xml.push_str("  <build>\n");
    xml.push_str("    <item objectid=\"1\"/>\n");
    xml.push_str("  </build>\n");
    xml.push_str("</model>\n");
    xml
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::booleans::FaceSoup;
    use kerf_geom::Point3;

    fn unit_tri_soup() -> FaceSoup {
        FaceSoup {
            triangles: vec![[
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(0.0, 1.0, 0.0),
            ]],
        }
    }

    #[test]
    fn write_3mf_produces_pk_magic() {
        let soup = unit_tri_soup();
        let bytes = write_3mf(&soup, "test").unwrap();
        // ZIP local file header magic: PK\x03\x04
        assert!(
            bytes.starts_with(b"PK\x03\x04"),
            "expected ZIP magic PK\\x03\\x04, got {:?}",
            &bytes[..4.min(bytes.len())]
        );
    }

    #[test]
    fn write_3mf_contains_model_xml() {
        let soup = unit_tri_soup();
        let bytes = write_3mf(&soup, "mypart").unwrap();
        // Read it back as a ZIP and check the model XML exists.
        let cursor = std::io::Cursor::new(&bytes);
        let mut zip = zip::ZipArchive::new(cursor).expect("valid ZIP");
        let mut model_file = zip.by_name("3D/3dmodel.model").expect("3dmodel.model present");
        let mut content = String::new();
        std::io::Read::read_to_string(&mut model_file, &mut content).unwrap();
        assert!(content.contains("<vertex"), "expected vertex elements");
        assert!(content.contains("<triangle"), "expected triangle elements");
        assert!(content.contains("mypart"), "expected part name");
    }

    #[test]
    fn write_3mf_vertex_count_matches_soup() {
        // Two disjoint triangles → 6 unique vertices (no shared).
        let soup = FaceSoup {
            triangles: vec![
                [
                    Point3::new(0.0, 0.0, 0.0),
                    Point3::new(1.0, 0.0, 0.0),
                    Point3::new(0.0, 1.0, 0.0),
                ],
                [
                    Point3::new(5.0, 0.0, 0.0),
                    Point3::new(6.0, 0.0, 0.0),
                    Point3::new(5.0, 1.0, 0.0),
                ],
            ],
        };
        let bytes = write_3mf(&soup, "two_tris").unwrap();
        let cursor = std::io::Cursor::new(&bytes);
        let mut zip = zip::ZipArchive::new(cursor).unwrap();
        let mut model_file = zip.by_name("3D/3dmodel.model").unwrap();
        let mut content = String::new();
        std::io::Read::read_to_string(&mut model_file, &mut content).unwrap();
        let vertex_count = content.matches("<vertex").count();
        assert_eq!(vertex_count, 6, "expected 6 unique vertices, got {vertex_count}");
    }

    #[test]
    fn write_3mf_deduplicates_shared_vertices() {
        // Two triangles sharing an edge → 4 unique vertices.
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
        let bytes = write_3mf(&soup, "quad").unwrap();
        let cursor = std::io::Cursor::new(&bytes);
        let mut zip = zip::ZipArchive::new(cursor).unwrap();
        let mut model_file = zip.by_name("3D/3dmodel.model").unwrap();
        let mut content = String::new();
        std::io::Read::read_to_string(&mut model_file, &mut content).unwrap();
        let vertex_count = content.matches("<vertex").count();
        assert_eq!(vertex_count, 4, "expected 4 deduped vertices, got {vertex_count}");
    }
}
