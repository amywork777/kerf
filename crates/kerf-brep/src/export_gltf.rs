//! GLTF/GLB export for `FaceSoup`.
//!
//! Produces a binary GLB (GL Transmission Format, binary container) with:
//!   - Chunk 0: JSON descriptor (scene, mesh, accessors, buffer views, buffer)
//!   - Chunk 1: binary buffer (vertex positions as f32 xyz + u32 indices)
//!
//! GLB magic: `glTF` (0x46546C67 LE). Version 2. Vertices are deduplicated
//! within the default point-equality tolerance.
//!
//! GLB byte layout (per spec §4.7):
//!   12-byte file header | 8-byte JSON chunk header + JSON bytes (4-aligned)
//!   | 8-byte BIN chunk header + binary bytes (4-aligned)
//!
//! # kerf_analytic_edges extras
//!
//! When analytic-edge data is provided (see [`write_gltf_with_analytics`]),
//! a `kerf_analytic_edges` key is injected into the primitive's `extras`
//! object. The value is a JSON array of objects, one per analytic edge:
//!
//! ```json
//! "extras": {
//!   "kerf_analytic_edges": [
//!     { "face_index": 12, "kind": "circle",
//!       "center": [0,0,10], "radius": 10, "normal": [0,0,1],
//!       "start_angle": 0.0, "sweep_angle": 6.283185307 },
//!     { "face_index": 18, "kind": "ellipse", ... }
//!   ]
//! }
//! ```
//!
//! `face_index` is the sequential index into the solid's face list (as
//! returned by `tessellate_with_face_index`). Vanilla GLTF viewers silently
//! ignore unknown `extras`. Future kerf tooling (viewer integration,
//! CAD-aware GLTF inspector) can consume this payload.
//!
//! Note on KHR_geometry_curves: the `KHR_geometry_curves` extension is not
//! yet ratified in the Khronos ecosystem (as of 2026). Until it achieves
//! official status, `extras` is the appropriate extensibility mechanism.
//! Migration to `KHR_geometry_curves` is a future task.

use kerf_geom::{Point3, Tolerance};

use crate::analytic_edge::AnalyticEdge;
use crate::booleans::FaceSoup;

/// Error type for GLTF export failures.
#[derive(Debug)]
pub struct ExportGltfError(pub String);

impl std::fmt::Display for ExportGltfError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "GLTF export error: {}", self.0)
    }
}

impl std::error::Error for ExportGltfError {}

/// Write `soup` as a binary GLB file. Returns the raw bytes.
/// `_name` is currently unused in the GLB but reserved for future mesh naming.
///
/// No analytic-edge extras are emitted. See [`write_gltf_with_analytics`] for
/// that capability.
pub fn write_gltf(soup: &FaceSoup, _name: &str) -> Result<Vec<u8>, ExportGltfError> {
    write_gltf_with_analytics(soup, _name, &[])
}

/// Serialize one AnalyticEdge as a JSON object fragment for the
/// `kerf_analytic_edges` extras array. The face_index field is the
/// sequential face number from `tessellate_with_face_index`.
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
    }
}

/// Write `soup` as a binary GLB file with `kerf_analytic_edges` extras.
///
/// `analytic_edges` is a slice of `(face_index, edge)` pairs where
/// `face_index` matches the sequential face numbering from
/// [`tessellate_with_face_index`](crate::tessellate::tessellate_with_face_index).
/// When the slice is empty the output is identical to [`write_gltf`].
pub fn write_gltf_with_analytics(
    soup: &FaceSoup,
    _name: &str,
    analytic_edges: &[(u32, &AnalyticEdge)],
) -> Result<Vec<u8>, ExportGltfError> {
    let tol = Tolerance::default();

    // Deduplicate vertices → unique positions list + per-triangle indices.
    let mut positions: Vec<Point3> = Vec::new();
    let mut indices: Vec<u32> = Vec::with_capacity(soup.triangles.len() * 3);

    for tri in &soup.triangles {
        for i in 0..3 {
            let p = tri[i];
            let found = positions
                .iter()
                .position(|q| (p - *q).norm() < tol.point_eq);
            let idx = match found {
                Some(j) => j as u32,
                None => {
                    positions.push(p);
                    (positions.len() - 1) as u32
                }
            };
            indices.push(idx);
        }
    }

    // Compute bounding box (required by GLTF accessor min/max).
    let (mut min_x, mut min_y, mut min_z) = (f32::INFINITY, f32::INFINITY, f32::INFINITY);
    let (mut max_x, mut max_y, mut max_z) = (f32::NEG_INFINITY, f32::NEG_INFINITY, f32::NEG_INFINITY);
    for p in &positions {
        let (x, y, z) = (p.x as f32, p.y as f32, p.z as f32);
        if x < min_x { min_x = x; }
        if y < min_y { min_y = y; }
        if z < min_z { min_z = z; }
        if x > max_x { max_x = x; }
        if y > max_y { max_y = y; }
        if z > max_z { max_z = z; }
    }
    // Guard against empty mesh.
    if positions.is_empty() {
        (min_x, min_y, min_z) = (0.0, 0.0, 0.0);
        (max_x, max_y, max_z) = (0.0, 0.0, 0.0);
    }

    // --- Build binary buffer ---
    // Layout: [vertex positions (f32 x3 each)] then [indices (u32 each)]
    // Both sections must be 4-byte aligned (they naturally are for f32/u32).
    let vertex_byte_len = positions.len() * 3 * 4; // 3 floats × 4 bytes
    let index_byte_len = indices.len() * 4;         // u32 × 4 bytes
    let bin_byte_len = vertex_byte_len + index_byte_len;

    let mut bin_buf: Vec<u8> = Vec::with_capacity(bin_byte_len);
    for p in &positions {
        bin_buf.extend_from_slice(&(p.x as f32).to_le_bytes());
        bin_buf.extend_from_slice(&(p.y as f32).to_le_bytes());
        bin_buf.extend_from_slice(&(p.z as f32).to_le_bytes());
    }
    for &idx in &indices {
        bin_buf.extend_from_slice(&idx.to_le_bytes());
    }
    // Pad binary chunk to 4-byte boundary (spec requires it).
    while bin_buf.len() % 4 != 0 {
        bin_buf.push(0x00);
    }
    let bin_chunk_padded_len = bin_buf.len();

    // --- Build analytic-edges extras JSON fragment (empty if none provided) ---
    let extras_fragment = if analytic_edges.is_empty() {
        String::new()
    } else {
        let entries: Vec<String> = analytic_edges
            .iter()
            .map(|(face_idx, ae)| analytic_edge_to_json(*face_idx, ae))
            .collect();
        format!(
            r#","extras": {{"kerf_analytic_edges": [{items}]}}"#,
            items = entries.join(",")
        )
    };

    // --- Build JSON descriptor ---
    // bufferView 0 → vertex positions
    // bufferView 1 → indices
    // accessor 0  → VEC3/FLOAT positions
    // accessor 1  → SCALAR/UNSIGNED_INT indices
    let json = format!(
        r#"{{
  "asset": {{"version": "2.0", "generator": "kerf-cad"}},
  "scene": 0,
  "scenes": [{{"nodes": [0]}}],
  "nodes": [{{"mesh": 0}}],
  "meshes": [{{
    "primitives": [{{
      "attributes": {{"POSITION": 0}},
      "indices": 1{extras}
    }}]
  }}],
  "accessors": [
    {{
      "bufferView": 0,
      "componentType": 5126,
      "count": {vcount},
      "type": "VEC3",
      "min": [{min_x}, {min_y}, {min_z}],
      "max": [{max_x}, {max_y}, {max_z}]
    }},
    {{
      "bufferView": 1,
      "componentType": 5125,
      "count": {icount},
      "type": "SCALAR"
    }}
  ],
  "bufferViews": [
    {{
      "buffer": 0,
      "byteOffset": 0,
      "byteLength": {vbytes},
      "target": 34962
    }},
    {{
      "buffer": 0,
      "byteOffset": {vbytes},
      "byteLength": {ibytes},
      "target": 34963
    }}
  ],
  "buffers": [{{"byteLength": {bin_len}}}]
}}"#,
        extras = extras_fragment,
        vcount = positions.len(),
        min_x = min_x, min_y = min_y, min_z = min_z,
        max_x = max_x, max_y = max_y, max_z = max_z,
        icount = indices.len(),
        vbytes = vertex_byte_len,
        ibytes = index_byte_len,
        bin_len = bin_byte_len,
    );

    // Pad JSON chunk to 4-byte boundary with spaces (spec §4.7.2.1).
    let mut json_bytes = json.into_bytes();
    while json_bytes.len() % 4 != 0 {
        json_bytes.push(b' ');
    }
    let json_chunk_padded_len = json_bytes.len();

    // --- Assemble GLB ---
    // File header: magic(4) + version(4) + total_length(4) = 12 bytes
    // JSON chunk:  length(4) + type(4) + data = 8 + json_chunk_padded_len
    // BIN  chunk:  length(4) + type(4) + data = 8 + bin_chunk_padded_len
    let total_len: u32 = (12 + 8 + json_chunk_padded_len + 8 + bin_chunk_padded_len) as u32;

    let mut out: Vec<u8> = Vec::with_capacity(total_len as usize);

    // File header.
    out.extend_from_slice(b"glTF");              // magic
    out.extend_from_slice(&2u32.to_le_bytes());  // version
    out.extend_from_slice(&total_len.to_le_bytes());

    // JSON chunk.
    out.extend_from_slice(&(json_chunk_padded_len as u32).to_le_bytes());
    out.extend_from_slice(&0x4E4F534Au32.to_le_bytes()); // "JSON" chunk type
    out.extend_from_slice(&json_bytes);

    // BIN chunk.
    out.extend_from_slice(&(bin_chunk_padded_len as u32).to_le_bytes());
    out.extend_from_slice(&0x004E4942u32.to_le_bytes()); // "BIN\0" chunk type
    out.extend_from_slice(&bin_buf);

    Ok(out)
}

/// Parse a GLB byte slice and return the vertex count encoded in the
/// POSITION accessor (accessor index 0). Useful for round-trip tests.
pub fn glb_vertex_count(glb: &[u8]) -> Result<usize, ExportGltfError> {
    // Validate magic.
    if glb.len() < 12 || &glb[..4] != b"glTF" {
        return Err(ExportGltfError("not a valid GLB file".into()));
    }
    // JSON chunk starts at offset 12.
    if glb.len() < 20 {
        return Err(ExportGltfError("GLB too short for JSON chunk header".into()));
    }
    let json_len = u32::from_le_bytes([glb[12], glb[13], glb[14], glb[15]]) as usize;
    let json_start = 20;
    let json_end = json_start + json_len;
    if glb.len() < json_end {
        return Err(ExportGltfError("GLB truncated in JSON chunk".into()));
    }
    let json_str = std::str::from_utf8(&glb[json_start..json_end])
        .map_err(|e| ExportGltfError(format!("JSON chunk not UTF-8: {e}")))?
        .trim_end_matches(' ');

    // Extract `"count": N` from the first accessor.
    // We do a minimal parse: find "count" after "accessors" appears.
    let acc_pos = json_str
        .find("\"accessors\"")
        .ok_or_else(|| ExportGltfError("no 'accessors' key in JSON".into()))?;
    let after_acc = &json_str[acc_pos..];
    let count_pos = after_acc
        .find("\"count\":")
        .ok_or_else(|| ExportGltfError("no 'count' in first accessor".into()))?;
    let after_count = &after_acc[count_pos + 8..].trim_start_matches(|c: char| c == ' ');
    let end = after_count
        .find(|c: char| !c.is_ascii_digit())
        .unwrap_or(after_count.len());
    let count: usize = after_count[..end]
        .parse()
        .map_err(|e| ExportGltfError(format!("bad count value: {e}")))?;
    Ok(count)
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
    fn write_gltf_produces_gltf_magic() {
        let soup = unit_tri_soup();
        let bytes = write_gltf(&soup, "test").unwrap();
        assert!(
            bytes.starts_with(b"glTF"),
            "expected GLB magic 'glTF', got {:?}",
            &bytes[..4.min(bytes.len())]
        );
    }

    #[test]
    fn write_gltf_version_is_2() {
        let soup = unit_tri_soup();
        let bytes = write_gltf(&soup, "test").unwrap();
        let version = u32::from_le_bytes([bytes[4], bytes[5], bytes[6], bytes[7]]);
        assert_eq!(version, 2, "GLB version must be 2");
    }

    #[test]
    fn write_gltf_total_length_matches_actual() {
        let soup = unit_tri_soup();
        let bytes = write_gltf(&soup, "test").unwrap();
        let declared_len = u32::from_le_bytes([bytes[8], bytes[9], bytes[10], bytes[11]]) as usize;
        assert_eq!(
            declared_len,
            bytes.len(),
            "GLB declared length {declared_len} != actual length {}",
            bytes.len()
        );
    }

    #[test]
    fn write_gltf_vertex_count_matches_soup_unique_vertices() {
        // Single triangle → 3 unique vertices.
        let soup = unit_tri_soup();
        let bytes = write_gltf(&soup, "tri").unwrap();
        let count = glb_vertex_count(&bytes).unwrap();
        assert_eq!(count, 3, "expected 3 unique vertices, got {count}");
    }

    #[test]
    fn write_gltf_deduplicates_shared_vertices() {
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
        let bytes = write_gltf(&soup, "quad").unwrap();
        let count = glb_vertex_count(&bytes).unwrap();
        assert_eq!(count, 4, "expected 4 deduped vertices, got {count}");
    }

    #[test]
    fn write_gltf_4byte_aligned_chunks() {
        let soup = unit_tri_soup();
        let bytes = write_gltf(&soup, "align_test").unwrap();
        // JSON chunk length at offset 12, must be 4-aligned.
        let json_chunk_len = u32::from_le_bytes([bytes[12], bytes[13], bytes[14], bytes[15]]) as usize;
        assert_eq!(json_chunk_len % 4, 0, "JSON chunk must be 4-aligned");
        // BIN chunk starts at 12 + 8 + json_chunk_len.
        let bin_header_off = 12 + 8 + json_chunk_len;
        assert!(bytes.len() >= bin_header_off + 8, "GLB truncated before BIN chunk");
        let bin_chunk_len = u32::from_le_bytes([
            bytes[bin_header_off],
            bytes[bin_header_off + 1],
            bytes[bin_header_off + 2],
            bytes[bin_header_off + 3],
        ]) as usize;
        assert_eq!(bin_chunk_len % 4, 0, "BIN chunk must be 4-aligned");
    }
}
