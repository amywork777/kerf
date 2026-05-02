//! Triangle-mesh → `Solid` import.
//!
//! Builds a half-edge B-rep solid from a flat list of 3-vertex triangles by
//! deduping vertices on quantized position and pairing directed edges (u, v)
//! with their reverse (v, u) as half-edge twins. Each triangle becomes one
//! triangular face with a `SurfaceKind::Plane` computed from its corners.
//!
//! The mesh must be a closed orientable 2-manifold: every directed edge must
//! have exactly one matching reverse-direction half-edge from a neighboring
//! triangle. Non-manifold input (boundary edges, T-junctions, edges shared by
//! more than two triangles) returns an error.

use std::collections::HashMap;
use std::io::{self, BufRead, BufReader, Read};

use kerf_geom::{Frame, Plane, Point3};
use kerf_topo::validate;

use crate::booleans::FaceSoup;
use crate::geometry::SurfaceKind;
use crate::Solid;

#[derive(Debug)]
pub enum MeshImportError {
    /// A directed edge (u, v) appeared in more than one triangle on the same
    /// side — the mesh is non-manifold or has flipped winding.
    DuplicateDirectedEdge(usize, usize),
    /// A directed edge (u, v) had no matching reverse-direction half-edge —
    /// the mesh has an open boundary or asymmetric winding.
    UnmatchedDirectedEdge(usize, usize),
    /// A triangle has a degenerate area (collinear or coincident vertices).
    DegenerateTriangle(usize),
    /// Topology validation rejected the assembled solid.
    InvalidTopology(String),
    /// ASCII STL parse failed at this line / token.
    AsciiParse(String),
    /// I/O error while reading the input stream.
    Io(io::Error),
}

impl From<io::Error> for MeshImportError {
    fn from(e: io::Error) -> Self {
        MeshImportError::Io(e)
    }
}

impl std::fmt::Display for MeshImportError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::DuplicateDirectedEdge(u, v) => {
                write!(f, "directed edge ({u}, {v}) appears in more than one triangle")
            }
            Self::UnmatchedDirectedEdge(u, v) => {
                write!(f, "directed edge ({u}, {v}) has no matching reverse half-edge")
            }
            Self::DegenerateTriangle(i) => write!(f, "triangle {i} has zero area"),
            Self::InvalidTopology(s) => write!(f, "invalid topology: {s}"),
            Self::AsciiParse(s) => write!(f, "ascii parse error: {s}"),
            Self::Io(e) => write!(f, "io error: {e}"),
        }
    }
}

impl std::error::Error for MeshImportError {}

/// Quantize a position to integer keys for hash lookup. The default 1e6 grid
/// matches the kernel's working tolerance.
fn quantize(p: Point3, grid: f64) -> (i64, i64, i64) {
    (
        (p.x * grid).round() as i64,
        (p.y * grid).round() as i64,
        (p.z * grid).round() as i64,
    )
}

/// Build a `Solid` from a triangle soup. Vertices are deduped on a 1 µm grid.
pub fn from_triangles(tris: &[[Point3; 3]]) -> Result<Solid, MeshImportError> {
    if tris.is_empty() {
        return Ok(Solid::new());
    }
    let grid = 1.0e6;

    let mut s = Solid::new();
    let solid_id = s.topo.build_insert_solid();
    s.topo.build_set_active_solid(Some(solid_id));
    let shell = s.topo.build_insert_shell(solid_id);

    // Vertex dedup.
    let mut v_map: HashMap<(i64, i64, i64), kerf_topo::VertexId> = HashMap::new();
    for tri in tris {
        for p in tri {
            let key = quantize(*p, grid);
            v_map.entry(key).or_insert_with(|| {
                let v = s.topo.build_insert_vertex();
                s.vertex_geom.insert(v, *p);
                v
            });
        }
    }

    // For each triangle, allocate face + loop + 3 half-edges and link them.
    // Track directed edges (origin, destination) → HalfEdgeId for twin pairing.
    let mut directed: HashMap<(kerf_topo::VertexId, kerf_topo::VertexId), kerf_topo::HalfEdgeId> =
        HashMap::with_capacity(tris.len() * 3);

    for (i, tri) in tris.iter().enumerate() {
        let v: [kerf_topo::VertexId; 3] = [
            v_map[&quantize(tri[0], grid)],
            v_map[&quantize(tri[1], grid)],
            v_map[&quantize(tri[2], grid)],
        ];
        if v[0] == v[1] || v[1] == v[2] || v[2] == v[0] {
            return Err(MeshImportError::DegenerateTriangle(i));
        }

        let lp = s.topo.build_insert_loop_placeholder();
        let face = s.topo.build_insert_face(lp, shell);
        s.topo.build_set_loop_face(lp, face);
        s.topo.build_push_shell_face(shell, face);

        let h: [kerf_topo::HalfEdgeId; 3] = [
            s.topo.build_insert_half_edge(v[0], lp),
            s.topo.build_insert_half_edge(v[1], lp),
            s.topo.build_insert_half_edge(v[2], lp),
        ];
        s.topo.build_set_half_edge_next_prev(h[0], h[1]);
        s.topo.build_set_half_edge_next_prev(h[1], h[2]);
        s.topo.build_set_half_edge_next_prev(h[2], h[0]);
        s.topo.build_set_loop_half_edge(lp, Some(h[0]));

        for k in 0..3 {
            let directed_key = (v[k], v[(k + 1) % 3]);
            if directed.insert(directed_key, h[k]).is_some() {
                return Err(MeshImportError::DuplicateDirectedEdge(i, k));
            }
        }

        // Plane geometry from triangle corners.
        let e1 = tri[1] - tri[0];
        let e2 = tri[2] - tri[0];
        match Frame::from_x_yhint(tri[0], e1, e2) {
            Some(frame) => {
                s.face_geom.insert(face, SurfaceKind::Plane(Plane::new(frame)));
            }
            None => return Err(MeshImportError::DegenerateTriangle(i)),
        }
    }

    // Pair twins. Each (u, v) directed edge must match exactly one (v, u).
    let keys: Vec<_> = directed.keys().copied().collect();
    let mut paired = HashMap::with_capacity(keys.len() / 2);
    for (u, v) in keys {
        if paired.contains_key(&(u, v)) || paired.contains_key(&(v, u)) {
            continue;
        }
        let h_uv = directed[&(u, v)];
        let h_vu = match directed.get(&(v, u)).copied() {
            Some(h) => h,
            None => return Err(MeshImportError::UnmatchedDirectedEdge(0, 0)),
        };

        let edge = s.topo.build_insert_edge([h_uv, h_vu]);
        s.topo.build_set_half_edge_twin(h_uv, h_vu);
        s.topo.build_set_half_edge_twin(h_vu, h_uv);
        s.topo.build_set_half_edge_edge(h_uv, edge);
        s.topo.build_set_half_edge_edge(h_vu, edge);
        paired.insert((u, v), ());
        paired.insert((v, u), ());
    }

    // Set each vertex's outgoing half-edge to any incident half-edge.
    for ((u, _v), h) in &directed {
        if s.topo.vertex(*u).and_then(|vx| vx.outgoing()).is_none() {
            s.topo.build_set_vertex_outgoing(*u, Some(*h));
        }
    }

    validate(&s.topo).map_err(|e| MeshImportError::InvalidTopology(format!("{e:?}")))?;
    Ok(s)
}

/// Read an ASCII STL stream and return a `FaceSoup`.
///
/// Only the `vertex x y z` lines are parsed — `facet normal` is recomputed
/// from the triangle corners on import. Whitespace-tolerant; rejects files
/// with non-3-vertex loops.
pub fn read_ascii(r: &mut impl Read) -> Result<FaceSoup, MeshImportError> {
    let reader = BufReader::new(r);
    let mut tris: Vec<[Point3; 3]> = Vec::new();
    let mut current: Vec<Point3> = Vec::with_capacity(3);
    for line in reader.lines() {
        let line = line?;
        let trimmed = line.trim();
        if trimmed.starts_with("vertex") {
            let parts: Vec<&str> = trimmed.split_whitespace().collect();
            if parts.len() != 4 {
                return Err(MeshImportError::AsciiParse(format!(
                    "vertex line needs 3 floats, got {trimmed:?}"
                )));
            }
            let parse_f = |s: &str| {
                s.parse::<f64>()
                    .map_err(|_| MeshImportError::AsciiParse(format!("bad float {s:?}")))
            };
            let x = parse_f(parts[1])?;
            let y = parse_f(parts[2])?;
            let z = parse_f(parts[3])?;
            current.push(Point3::new(x, y, z));
        } else if trimmed.starts_with("endloop") {
            if current.len() != 3 {
                return Err(MeshImportError::AsciiParse(format!(
                    "facet has {} vertices (expected 3)",
                    current.len()
                )));
            }
            tris.push([current[0], current[1], current[2]]);
            current.clear();
        }
    }
    Ok(FaceSoup { triangles: tris })
}

/// Sniff the first 5 bytes to choose between ASCII and binary STL, then parse.
///
/// Some binary STL files start with the literal text "solid ", which can
/// fool a naive sniffer. The fallback heuristic: if the input begins with
/// "solid" but ASCII parsing returns zero triangles, retry as binary.
pub fn read_stl_auto(buf: &[u8]) -> Result<FaceSoup, MeshImportError> {
    let looks_ascii = buf.starts_with(b"solid");
    if looks_ascii {
        if let Ok(soup) = read_ascii(&mut &buf[..]) {
            if !soup.triangles.is_empty() {
                return Ok(soup);
            }
        }
        // Misleading "solid" header but actually binary: fall through.
    }
    read_binary(&mut &buf[..])
}

/// Read a binary STL stream and return a `FaceSoup`.
pub fn read_binary(r: &mut impl Read) -> Result<FaceSoup, MeshImportError> {
    let mut header = [0u8; 80];
    r.read_exact(&mut header)?;

    let mut count_bytes = [0u8; 4];
    r.read_exact(&mut count_bytes)?;
    let count = u32::from_le_bytes(count_bytes) as usize;

    let mut tris = Vec::with_capacity(count);
    for _ in 0..count {
        let mut buf = [0u8; 50];
        r.read_exact(&mut buf)?;
        // Bytes 0..12 are normal (ignored — we re-derive). 12..48 are 3 vertices.
        let mut tri = [Point3::origin(); 3];
        for (j, t) in tri.iter_mut().enumerate() {
            let off = 12 + j * 12;
            let x = f32::from_le_bytes(buf[off..off + 4].try_into().unwrap()) as f64;
            let y = f32::from_le_bytes(buf[off + 4..off + 8].try_into().unwrap()) as f64;
            let z = f32::from_le_bytes(buf[off + 8..off + 12].try_into().unwrap()) as f64;
            *t = Point3::new(x, y, z);
        }
        tris.push(tri);
    }
    Ok(FaceSoup { triangles: tris })
}

/// Read a binary STL stream directly into a `Solid` (combines `read_binary` +
/// `from_triangles`).
pub fn read_stl_binary_to_solid(r: &mut impl Read) -> Result<Solid, MeshImportError> {
    let soup = read_binary(r)?;
    from_triangles(&soup.triangles)
}

/// Read any STL stream (ASCII or binary, sniffed automatically) into a `Solid`.
pub fn read_stl_to_solid(r: &mut impl Read) -> Result<Solid, MeshImportError> {
    let mut buf = Vec::new();
    r.read_to_end(&mut buf)?;
    let soup = read_stl_auto(&buf)?;
    from_triangles(&soup.triangles)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::primitives::box_;
    use crate::tessellate::tessellate;
    use crate::write_binary;
    use kerf_geom::Vec3;

    #[test]
    fn round_trip_box_through_stl_binary() {
        let s = box_(Vec3::new(1.0, 2.0, 3.0));
        let soup = tessellate(&s, 8);
        let mut buf = Vec::new();
        write_binary(&soup, "round_trip", &mut buf).unwrap();

        let s2 = read_stl_binary_to_solid(&mut buf.as_slice()).unwrap();
        // Box has 12 triangles. Vertex dedup → 8 verts; each edge shared by
        // 2 triangles → 18 directed edges / 2 = 18 edges; F = 12 triangles.
        // Euler: 8 - 18 + 12 = 2 ✓.
        assert_eq!(s2.vertex_count(), 8);
        assert_eq!(s2.edge_count(), 18);
        assert_eq!(s2.face_count(), 12);
    }

    #[test]
    fn round_trip_box_through_ascii_stl() {
        use crate::write_ascii;
        let s = box_(Vec3::new(1.0, 2.0, 3.0));
        let soup = tessellate(&s, 8);
        let mut buf = Vec::new();
        write_ascii(&soup, "round_trip", &mut buf).unwrap();

        let s2 = read_stl_to_solid(&mut buf.as_slice()).unwrap();
        assert_eq!(s2.vertex_count(), 8);
        assert_eq!(s2.edge_count(), 18);
        assert_eq!(s2.face_count(), 12);
    }

    #[test]
    fn auto_sniff_picks_correct_format() {
        let s = box_(Vec3::new(1.0, 1.0, 1.0));
        let soup = tessellate(&s, 8);
        // Binary path
        let mut bin = Vec::new();
        crate::write_binary(&soup, "x", &mut bin).unwrap();
        let soup_bin = read_stl_auto(&bin).unwrap();
        assert_eq!(soup_bin.triangles.len(), 12);
        // ASCII path
        let mut ascii = Vec::new();
        crate::write_ascii(&soup, "x", &mut ascii).unwrap();
        let soup_ascii = read_stl_auto(&ascii).unwrap();
        assert_eq!(soup_ascii.triangles.len(), 12);
    }

    #[test]
    fn from_triangles_empty_returns_empty_solid() {
        let s = from_triangles(&[]).unwrap();
        assert_eq!(s.vertex_count(), 0);
        assert_eq!(s.face_count(), 0);
    }

    #[test]
    fn from_triangles_open_mesh_rejects() {
        // A single triangle has unmatched directed edges (no neighbor).
        let tri = [[
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ]];
        let r = from_triangles(&tri);
        assert!(matches!(r, Err(MeshImportError::UnmatchedDirectedEdge(_, _))));
    }

    #[test]
    fn two_imported_boxes_intersection() {
        // Round-trip two boxes through STL, then intersect them. This is the
        // pure-imported path: every face on both inputs is a triangulated
        // Plane, and the boolean pipeline must handle them like native faces.
        let a_native = box_(Vec3::new(2.0, 2.0, 2.0));
        let b_native = crate::primitives::box_at(
            Vec3::new(2.0, 2.0, 2.0),
            kerf_geom::Point3::new(1.0, 0.0, 0.0),
        );
        let mut a_buf = Vec::new();
        let mut b_buf = Vec::new();
        write_binary(&tessellate(&a_native, 8), "a", &mut a_buf).unwrap();
        write_binary(&tessellate(&b_native, 8), "b", &mut b_buf).unwrap();
        let a = read_stl_binary_to_solid(&mut a_buf.as_slice()).unwrap();
        let b = read_stl_binary_to_solid(&mut b_buf.as_slice()).unwrap();

        let r = a.intersection(&b);
        assert!(r.face_count() > 0);
        kerf_topo::validate(&r.topo).unwrap();
    }

    #[test]
    fn imported_box_supports_union_with_native_box() {
        // Round-trip box A through STL → Solid. Then union with a native box B
        // shifted in x. Verifies that the imported B-rep plays nicely with
        // the boolean pipeline.
        let a_native = box_(Vec3::new(2.0, 2.0, 2.0));
        let soup = tessellate(&a_native, 8);
        let mut buf = Vec::new();
        write_binary(&soup, "round_trip", &mut buf).unwrap();
        let a_imported = read_stl_binary_to_solid(&mut buf.as_slice()).unwrap();

        let b = crate::primitives::box_at(
            Vec3::new(2.0, 2.0, 2.0),
            kerf_geom::Point3::new(1.0, 0.0, 0.0),
        );
        let u = a_imported.union(&b);
        // Result should be a connected solid; precise face/vertex counts depend
        // on triangulation, but the topology must validate.
        assert!(u.face_count() > 0);
        kerf_topo::validate(&u.topo).unwrap();
    }

    #[test]
    fn from_triangles_tetrahedron_validates() {
        // A regular tet: 4 vertices, 6 edges, 4 triangular faces. V-E+F = 2 ✓.
        let v = [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
            Point3::new(0.0, 0.0, 1.0),
        ];
        // Outward-normal winding (CCW seen from outside).
        let tris = [
            [v[1], v[2], v[3]], // far face
            [v[0], v[3], v[2]],
            [v[0], v[1], v[3]],
            [v[0], v[2], v[1]],
        ];
        let s = from_triangles(&tris).unwrap();
        assert_eq!(s.vertex_count(), 4);
        assert_eq!(s.edge_count(), 6);
        assert_eq!(s.face_count(), 4);
    }
}
