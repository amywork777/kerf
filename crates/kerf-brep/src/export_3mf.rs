//! 3MF (3D Manufacturing Format) export for `FaceSoup`.
//!
//! Produces a minimal, spec-conformant 3MF ZIP archive containing:
//!   - `_rels/.rels` — OPC relationship pointing to the 3D model
//!   - `[Content_Types].xml` — OPC content-type manifest
//!   - `3D/3dmodel.model` — mesh as vertices + triangles in 3MF XML schema
//!
//! The output is a `Vec<u8>` (ZIP archive). Vertices are deduplicated within
//! the default point-equality tolerance so the mesh is compact.
//!
//! # kerf_analytic_edges metadata
//!
//! When analytic-edge data is provided (see [`write_3mf_with_analytics`]),
//! a `kerf_analytic_edges` `<metadata>` element is injected into the `<object>`
//! element. The value is a JSON array of objects, one per analytic edge:
//!
//! ```xml
//! <object id="1" type="model">
//!   <mesh>...</mesh>
//!   <metadata name="kerf_analytic_edges">
//!     [{"face_index":12,"kind":"circle","center":[0,0,10],"radius":10,...}]
//!   </metadata>
//! </object>
//! ```
//!
//! The JSON schema is identical to the GLTF extras payload from PR #96.

use std::io::Write;

use kerf_geom::{Point3, Tolerance};

use crate::analytic_edge::AnalyticEdge;
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
///
/// No analytic-edge metadata is emitted. See [`write_3mf_with_analytics`] for
/// that capability.
pub fn write_3mf(soup: &FaceSoup, name: &str) -> Result<Vec<u8>, Export3mfError> {
    write_3mf_with_analytics(soup, name, &[])
}

/// Serialize one `AnalyticEdge` as a JSON object fragment for the
/// `kerf_analytic_edges` metadata array.
fn analytic_edge_to_json(face_idx: u32, ae: &AnalyticEdge) -> String {
    match ae {
        AnalyticEdge::Line { start, end } => format!(
            r#"{{"face_index":{face_idx},"kind":"line","start":[{},{},{}],"end":[{},{},{}]}}"#,
            start[0], start[1], start[2], end[0], end[1], end[2]
        ),
        AnalyticEdge::Circle { center, radius, normal, start_angle, sweep_angle } => format!(
            r#"{{"face_index":{face_idx},"kind":"circle","center":[{},{},{}],"radius":{radius},"normal":[{},{},{}],"start_angle":{start_angle},"sweep_angle":{sweep_angle}}}"#,
            center[0], center[1], center[2], normal[0], normal[1], normal[2]
        ),
        AnalyticEdge::Ellipse { center, major_axis, minor_axis, start_angle, sweep_angle } => format!(
            r#"{{"face_index":{face_idx},"kind":"ellipse","center":[{},{},{}],"major_axis":[{},{},{}],"minor_axis":[{},{},{}],"start_angle":{start_angle},"sweep_angle":{sweep_angle}}}"#,
            center[0], center[1], center[2],
            major_axis[0], major_axis[1], major_axis[2],
            minor_axis[0], minor_axis[1], minor_axis[2]
        ),
        AnalyticEdge::BSpline { control_points, is_clamped } => {
            let pts: Vec<String> = control_points.iter()
                .map(|p| format!("[{},{},{}]", p[0], p[1], p[2]))
                .collect();
            format!(
                r#"{{"face_index":{face_idx},"kind":"bspline","control_points":[{}],"is_clamped":{is_clamped}}}"#,
                pts.join(",")
            )
        }
        AnalyticEdge::Viviani { radius, frame, origin } => format!(
            r#"{{"face_index":{face_idx},"kind":"viviani","radius":{radius},"frame":[[{},{},{}],[{},{},{}],[{},{},{}]],"origin":[{},{},{}]}}"#,
            frame[0][0], frame[0][1], frame[0][2],
            frame[1][0], frame[1][1], frame[1][2],
            frame[2][0], frame[2][1], frame[2][2],
            origin[0], origin[1], origin[2]
        ),
    }
}

/// Write `soup` as a 3MF ZIP archive with `kerf_analytic_edges` metadata.
///
/// `analytic_edges` is a slice of `(face_index, edge)` pairs where
/// `face_index` matches the sequential face numbering from
/// [`tessellate_with_face_index`](crate::tessellate::tessellate_with_face_index).
/// When the slice is empty the output is identical to [`write_3mf`].
///
/// The metadata is embedded as a `<metadata name="kerf_analytic_edges">`
/// element inside the `<object>` element, containing a JSON array of edge
/// descriptors. The JSON schema is identical to the GLTF extras payload from
/// PR #96.
pub fn write_3mf_with_analytics(
    soup: &FaceSoup,
    name: &str,
    analytic_edges: &[(u32, &AnalyticEdge)],
) -> Result<Vec<u8>, Export3mfError> {
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

    // Build analytic-edges JSON (empty string if none provided).
    let analytic_metadata = if analytic_edges.is_empty() {
        None
    } else {
        let entries: Vec<String> = analytic_edges
            .iter()
            .map(|(face_idx, ae)| analytic_edge_to_json(*face_idx, ae))
            .collect();
        Some(format!("[{}]", entries.join(",")))
    };

    // Build 3MF XML for 3D/3dmodel.model
    let model_xml = build_model_xml(name, &positions, &triangles, analytic_metadata.as_deref());

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

fn build_model_xml(
    name: &str,
    positions: &[Point3],
    triangles: &[[usize; 3]],
    analytic_metadata: Option<&str>,
) -> String {
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
    if let Some(json) = analytic_metadata {
        xml.push_str(&format!(
            "      <metadata name=\"kerf_analytic_edges\">{json}</metadata>\n"
        ));
    }
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
    use crate::analytic_edge::AnalyticEdge;
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

    // -----------------------------------------------------------------------
    // Tests for write_3mf_with_analytics
    // -----------------------------------------------------------------------

    /// Empty analytic_edges slice → output identical to write_3mf.
    #[test]
    fn write_3mf_with_analytics_empty_matches_write_3mf() {
        let soup = unit_tri_soup();
        let bytes_plain = write_3mf(&soup, "test").unwrap();
        let bytes_analytic = write_3mf_with_analytics(&soup, "test", &[]).unwrap();
        assert_eq!(
            bytes_plain, bytes_analytic,
            "write_3mf_with_analytics([]) must produce identical output to write_3mf"
        );
    }

    /// A Circle edge → ZIP's 3dmodel.model contains a kerf_analytic_edges metadata element.
    #[test]
    fn write_3mf_with_analytics_circle_contains_metadata_element() {
        use std::f64::consts::TAU;
        let soup = unit_tri_soup();
        let circle = AnalyticEdge::Circle {
            center: [0.0, 0.0, 10.0],
            radius: 10.0,
            normal: [0.0, 0.0, 1.0],
            start_angle: 0.0,
            sweep_angle: TAU,
        };
        let edges = [(12u32, &circle)];
        let bytes = write_3mf_with_analytics(&soup, "part", &edges).unwrap();

        let cursor = std::io::Cursor::new(&bytes);
        let mut zip = zip::ZipArchive::new(cursor).expect("valid ZIP");
        let mut model_file = zip.by_name("3D/3dmodel.model").expect("3dmodel.model present");
        let mut content = String::new();
        std::io::Read::read_to_string(&mut model_file, &mut content).unwrap();

        assert!(
            content.contains("<metadata name=\"kerf_analytic_edges\">"),
            "expected kerf_analytic_edges metadata element, got:\n{content}"
        );
    }

    /// Embedded JSON parses back correctly and matches the input edge.
    #[test]
    fn write_3mf_with_analytics_circle_json_round_trip() {
        use std::f64::consts::TAU;
        let soup = unit_tri_soup();
        let circle = AnalyticEdge::Circle {
            center: [0.0, 0.0, 10.0],
            radius: 10.0,
            normal: [0.0, 0.0, 1.0],
            start_angle: 0.0,
            sweep_angle: TAU,
        };
        let edges = [(12u32, &circle)];
        let bytes = write_3mf_with_analytics(&soup, "part", &edges).unwrap();

        // Extract model XML from ZIP.
        let cursor = std::io::Cursor::new(&bytes);
        let mut zip = zip::ZipArchive::new(cursor).expect("valid ZIP");
        let mut model_file = zip.by_name("3D/3dmodel.model").expect("3dmodel.model present");
        let mut content = String::new();
        std::io::Read::read_to_string(&mut model_file, &mut content).unwrap();

        // Pull out the JSON value between the metadata tags.
        let open_tag = "<metadata name=\"kerf_analytic_edges\">";
        let close_tag = "</metadata>";
        let start = content.find(open_tag).expect("metadata open tag") + open_tag.len();
        let end = content[start..].find(close_tag).expect("metadata close tag");
        let json_str = &content[start..start + end];

        // Parse as a JSON array of objects.
        let parsed: serde_json::Value = serde_json::from_str(json_str)
            .expect("metadata value must be valid JSON");
        let arr = parsed.as_array().expect("JSON must be an array");
        assert_eq!(arr.len(), 1, "expected 1 edge entry, got {}", arr.len());

        let entry = &arr[0];
        assert_eq!(entry["face_index"], 12, "face_index mismatch");
        assert_eq!(entry["kind"], "circle", "kind mismatch");
        assert_eq!(entry["radius"], 10.0, "radius mismatch");

        let center = entry["center"].as_array().expect("center is array");
        assert_eq!(center.len(), 3);
        assert!((center[2].as_f64().unwrap() - 10.0).abs() < 1e-9, "center z mismatch");

        // Verify sweep_angle ≈ 2π
        let sweep = entry["sweep_angle"].as_f64().expect("sweep_angle numeric");
        assert!((sweep - TAU).abs() < 1e-9, "sweep_angle mismatch: {sweep}");
    }
}
