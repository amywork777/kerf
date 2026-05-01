//! Split edges in two solids at every intersection-segment endpoint.

use kerf_geom::Tolerance;
use kerf_topo::VertexId;

use crate::Solid;
use crate::booleans::{
    edge_lookup::{PointLocation, locate_point_on_face},
    intersect::FaceIntersection,
};

/// For one FaceIntersection endpoint: which vertex it now corresponds to in
/// each solid (after any necessary edge splits).
#[derive(Clone, Copy, Debug)]
pub struct EndpointVertices {
    pub vertex_a: VertexId,
    pub vertex_b: VertexId,
}

/// Result of splitting both solids at all intersection endpoints.
///
/// `endpoints[i]` is `None` if intersection `i` had any Interior endpoint that
/// needs phase B (interior-endpoint resolution via mev tails).
#[derive(Clone, Debug)]
pub struct SplitOutcome {
    /// `endpoints[i] = Some((start, end))` if both endpoints of intersection `i`
    /// resolved cleanly to a boundary vertex (OnVertex or OnEdge after split).
    /// `endpoints[i] = None` if any endpoint of intersection `i` was Interior
    /// in either solid — those are deferred to phase B.
    pub endpoints: Vec<Option<(EndpointVertices, EndpointVertices)>>,
}

/// Mutate both solids in place, splitting edges as needed so that every
/// FaceIntersection endpoint that lies on a boundary edge becomes a vertex.
/// Endpoints in face interior are deferred (recorded as `None`) for phase B.
pub fn split_solids_at_intersections(
    a: &mut Solid,
    b: &mut Solid,
    intersections: &[FaceIntersection],
    tol: &Tolerance,
) -> SplitOutcome {
    let mut endpoints = Vec::with_capacity(intersections.len());

    for inter in intersections {
        let start = endpoint_to_vertices(a, b, inter, true, tol);
        let end = endpoint_to_vertices(a, b, inter, false, tol);
        match (start, end) {
            (Some(s), Some(e)) => endpoints.push(Some((s, e))),
            _ => endpoints.push(None),
        }
    }

    SplitOutcome { endpoints }
}

fn endpoint_to_vertices(
    a: &mut Solid,
    b: &mut Solid,
    inter: &FaceIntersection,
    is_start: bool,
    tol: &Tolerance,
) -> Option<EndpointVertices> {
    let p = if is_start { inter.start } else { inter.end };

    let vertex_a = ensure_vertex_at(a, inter.face_a, p, tol)?;
    let vertex_b = ensure_vertex_at(b, inter.face_b, p, tol)?;
    Some(EndpointVertices { vertex_a, vertex_b })
}

/// If `p` is on a face vertex, return that vertex. If on an edge, split the
/// edge and return the new vertex. If interior, return None — phase B
/// (interior.rs) will handle it via an mev tail.
pub(crate) fn ensure_vertex_at(
    solid: &mut Solid,
    face: kerf_topo::FaceId,
    p: kerf_geom::Point3,
    tol: &Tolerance,
) -> Option<VertexId> {
    match locate_point_on_face(solid, face, p, tol) {
        PointLocation::OnVertex(v) => Some(v),
        PointLocation::OnEdge { edge, t: _ } => {
            let split = solid.topo.split_edge(edge);
            solid.vertex_geom.insert(split.new_vertex, p);
            // Edge geometry: re-derive from the topology. The original edge
            // segment becomes two segments; for v1 box-box every edge is a
            // line, so reconstruct the line segments from the endpoint Point3s.
            update_edge_geom_after_split(solid, edge, split.new_edge);
            Some(split.new_vertex)
        }
        PointLocation::Interior => None,
    }
}

fn update_edge_geom_after_split(
    solid: &mut Solid,
    old_edge: kerf_topo::EdgeId,
    new_edge: kerf_topo::EdgeId,
) {
    use crate::geometry::CurveSegment;
    use kerf_geom::Line;

    for &eid in &[old_edge, new_edge] {
        let edge = solid.topo.edge(eid).unwrap();
        let [he_a, _he_b] = edge.half_edges();
        let v0 = solid.topo.half_edge(he_a).unwrap().origin();
        let twin = solid.topo.half_edge(he_a).unwrap().twin();
        let v1 = solid.topo.half_edge(twin).unwrap().origin();
        let p0 = *solid.vertex_geom.get(v0).unwrap();
        let p1 = *solid.vertex_geom.get(v1).unwrap();
        let line = Line::through(p0, p1).unwrap();
        let length = (p1 - p0).norm();
        let seg = CurveSegment::line(line, 0.0, length);
        solid.edge_geom.insert(eid, seg);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use kerf_geom::{Point3, Vec3};

    use crate::booleans::face_intersections;
    use crate::primitives::{box_, box_at};

    #[test]
    fn overlapping_boxes_get_new_vertices() {
        let mut a = box_(Vec3::new(1.0, 1.0, 1.0));
        let mut b = box_at(Vec3::new(1.0, 1.0, 1.0), Point3::new(0.5, 0.0, 0.0));
        let v_a_before = a.vertex_count();
        let v_b_before = b.vertex_count();

        let intersections = face_intersections(&a, &b, &Tolerance::default());
        assert!(intersections.len() >= 4);

        let outcome =
            split_solids_at_intersections(&mut a, &mut b, &intersections, &Tolerance::default());

        assert_eq!(outcome.endpoints.len(), intersections.len());
        // Both solids should have gained vertices.
        assert!(
            a.vertex_count() > v_a_before,
            "a vertex count: {} -> {}",
            v_a_before,
            a.vertex_count()
        );
        assert!(
            b.vertex_count() > v_b_before,
            "b vertex count: {} -> {}",
            v_b_before,
            b.vertex_count()
        );

        // Topology validation must still pass.
        kerf_topo::validate(&a.topo).unwrap();
        kerf_topo::validate(&b.topo).unwrap();
    }

    #[test]
    fn nested_boxes_have_no_split_to_perform() {
        let mut big = box_(Vec3::new(10.0, 10.0, 10.0));
        let mut small = box_at(Vec3::new(2.0, 2.0, 2.0), Point3::new(4.0, 4.0, 4.0));
        let intersections = face_intersections(&big, &small, &Tolerance::default());
        assert_eq!(intersections.len(), 0);

        let outcome = split_solids_at_intersections(
            &mut big,
            &mut small,
            &intersections,
            &Tolerance::default(),
        );
        assert_eq!(outcome.endpoints.len(), 0);
        assert_eq!(big.vertex_count(), 8);
        assert_eq!(small.vertex_count(), 8);
    }
}
