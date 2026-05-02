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
use std::io::{self, Read};

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
