//! Add intersection-segment edges into both solids by calling `mef` per chord.

use kerf_geom::{Line, Tolerance};
use kerf_topo::{FaceId, HalfEdgeId, LoopId, VertexId};

use crate::Solid;
use crate::booleans::intersect::FaceIntersection;
use crate::booleans::split::SplitOutcome;
use crate::geometry::{CurveSegment, SurfaceKind};

/// Find a half-edge in `loop_id` whose origin is `target_vertex`. Returns None if absent.
pub fn find_he_in_loop(
    solid: &Solid,
    loop_id: LoopId,
    target_vertex: VertexId,
) -> Option<HalfEdgeId> {
    let lp = solid.topo.loop_(loop_id)?;
    let start = lp.half_edge()?;
    let mut cur = start;
    loop {
        let he = solid.topo.half_edge(cur)?;
        if he.origin() == target_vertex {
            return Some(cur);
        }
        cur = he.next();
        if cur == start {
            break;
        }
    }
    None
}

/// Check whether two vertices are connected by any existing edge in the solid.
pub fn vertices_connected(solid: &Solid, v1: VertexId, v2: VertexId) -> bool {
    // Brute-force scan over all edges — correct for any topology.
    for (_eid, edge) in solid.topo.edges_iter() {
        let [he_a, he_b] = edge.half_edges();
        let oa = solid.topo.half_edge(he_a).map(|h| h.origin());
        let ob = solid.topo.half_edge(he_b).map(|h| h.origin());
        if (oa == Some(v1) && ob == Some(v2)) || (oa == Some(v2) && ob == Some(v1)) {
            return true;
        }
    }
    false
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SolidIndex {
    A,
    B,
}

#[derive(Clone, Debug)]
pub struct AddedEdge {
    pub solid_index: SolidIndex,
    pub edge: kerf_topo::EdgeId,
    pub face_new: FaceId,
}

/// For each FaceIntersection, attempt to add an edge in solid A's `face_a` (if not
/// already present) and in solid B's `face_b` (if not already present). Each new
/// edge is a chord splitting the relevant face via mef.
pub fn add_intersection_edges(
    a: &mut Solid,
    b: &mut Solid,
    intersections: &[FaceIntersection],
    split_outcome: &SplitOutcome,
    _tol: &Tolerance,
) -> Vec<AddedEdge> {
    let mut added = Vec::new();
    for (i, inter) in intersections.iter().enumerate() {
        let (start_v, end_v) = split_outcome.endpoints[i];

        // Solid A: split face_a if endpoints aren't already connected.
        if !vertices_connected(a, start_v.vertex_a, end_v.vertex_a)
            && let Some(edge_info) = add_chord(
                a,
                inter.face_a,
                start_v.vertex_a,
                end_v.vertex_a,
                inter.start,
                inter.end,
            )
        {
            added.push(AddedEdge {
                solid_index: SolidIndex::A,
                edge: edge_info.0,
                face_new: edge_info.1,
            });
        }

        // Solid B: split face_b similarly.
        if !vertices_connected(b, start_v.vertex_b, end_v.vertex_b)
            && let Some(edge_info) = add_chord(
                b,
                inter.face_b,
                start_v.vertex_b,
                end_v.vertex_b,
                inter.start,
                inter.end,
            )
        {
            added.push(AddedEdge {
                solid_index: SolidIndex::B,
                edge: edge_info.0,
                face_new: edge_info.1,
            });
        }
    }
    added
}

/// Add a chord edge from `v_start` to `v_end` on the face's outer loop, splitting
/// the face via mef. Returns (EdgeId, new FaceId) on success, None if either
/// vertex is not found in the outer loop or if the half-edges have the same origin.
fn add_chord(
    solid: &mut Solid,
    face: FaceId,
    v_start: VertexId,
    v_end: VertexId,
    p_start: kerf_geom::Point3,
    p_end: kerf_geom::Point3,
) -> Option<(kerf_topo::EdgeId, FaceId)> {
    let outer_loop = solid.topo.face(face)?.outer_loop();

    let h_start = find_he_in_loop(solid, outer_loop, v_start)?;
    let h_end = find_he_in_loop(solid, outer_loop, v_end)?;

    // mef requires distinct half-edges (different origins).
    if h_start == h_end {
        return None;
    }

    let mef = solid.topo.mef(h_start, h_end);

    // Attach geometry: line edge.
    if let Some(line) = Line::through(p_start, p_end) {
        let length = (p_end - p_start).norm();
        let seg = CurveSegment::line(line, 0.0, length);
        solid.edge_geom.insert(mef.edge, seg);
    }

    // New face inherits the parent's plane.
    if let Some(parent_surface) = solid.face_geom.get(face).cloned()
        && let SurfaceKind::Plane(_) = &parent_surface
    {
        solid.face_geom.insert(mef.face, parent_surface);
    }

    Some((mef.edge, mef.face))
}

#[cfg(test)]
mod tests {
    use super::*;
    use kerf_geom::{Point3, Vec3};

    use crate::booleans::{face_intersections, split_solids_at_intersections};
    use crate::primitives::{box_, box_at};

    #[test]
    fn overlapping_boxes_get_split_faces() {
        let mut a = box_(Vec3::new(1.0, 1.0, 1.0));
        let mut b = box_at(Vec3::new(1.0, 1.0, 1.0), Point3::new(0.5, 0.0, 0.0));

        let f_b_before = b.face_count();

        let intersections = face_intersections(&a, &b, &Tolerance::default());
        let outcome =
            split_solids_at_intersections(&mut a, &mut b, &intersections, &Tolerance::default());

        let added = add_intersection_edges(
            &mut a,
            &mut b,
            &intersections,
            &outcome,
            &Tolerance::default(),
        );

        // Expect at least some new edges.
        assert!(!added.is_empty(), "expected some new edges");

        // B face count should have increased.
        assert!(
            b.face_count() >= f_b_before,
            "b face count: {} -> {}",
            f_b_before,
            b.face_count()
        );

        kerf_topo::validate(&a.topo).unwrap();
        kerf_topo::validate(&b.topo).unwrap();
    }

    #[test]
    fn nested_boxes_have_no_chord_to_add() {
        let mut big = box_(Vec3::new(10.0, 10.0, 10.0));
        let mut small = box_at(Vec3::new(2.0, 2.0, 2.0), Point3::new(4.0, 4.0, 4.0));

        let intersections = face_intersections(&big, &small, &Tolerance::default());
        let outcome = split_solids_at_intersections(
            &mut big,
            &mut small,
            &intersections,
            &Tolerance::default(),
        );
        let added = add_intersection_edges(
            &mut big,
            &mut small,
            &intersections,
            &outcome,
            &Tolerance::default(),
        );

        assert_eq!(added.len(), 0);
        kerf_topo::validate(&big.topo).unwrap();
        kerf_topo::validate(&small.topo).unwrap();
    }
}
