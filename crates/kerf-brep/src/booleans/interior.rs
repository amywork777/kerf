//! Phase B: resolve interior intersection endpoints by adding mev tails.
//!
//! After `split_solids_at_intersections` (phase A), some FaceIntersections have
//! one or both endpoints in the interior of a face (rather than on a boundary
//! edge). Phase A returns `None` for those entries in `SplitOutcome.endpoints`.
//!
//! Phase B walks those deferred intersections and adds the missing interior
//! vertices via `mev` tails anchored on a SIBLING intersection's already-resolved
//! boundary endpoint. Crucially, the mev's tail edge IS the chord connecting the
//! boundary vertex to the new interior vertex — so that intersection's chord is
//! already in the topology and phase C should skip it.
//!
//! Algorithm (per face, per solid):
//!   1. Group "pending" intersections (those needing this face's interior point)
//!      by quantized 3D point.
//!   2. For each interior point, find a SIBLING intersection that:
//!        - touches the same face,
//!        - has the same interior point as one endpoint,
//!        - has its OTHER endpoint already resolved as a boundary vertex.
//!   3. Find a half-edge in the face's outer loop whose DESTINATION is that
//!      boundary vertex. That's the `mev` anchor (mev grows from anchor's dest).
//!   4. Call `mev`. Attach the new vertex's Point3. Mark the sibling's chord as
//!      already added.
//!   5. Subsequent siblings sharing this interior point will, in phase C, find
//!      both their endpoints as resolved vertices and add their chord via mef.
//!
//! ## v1 limitations
//!
//! - Assumes every interior endpoint shares a face with at least one sibling
//!   intersection that has a boundary endpoint at that same point. The corner-cut
//!   prism case satisfies this; pathological cases (e.g., a chord with BOTH
//!   endpoints interior) will panic with diagnostic output.
//! - Quantizes 3D points using `tol.point_eq`; collisions assumed not to matter
//!   at v1 scale.

use std::collections::HashMap;

use kerf_geom::{Line, Point3, Tolerance};
use kerf_topo::{FaceId, HalfEdgeId, LoopId, VertexId};

use crate::Solid;
use crate::booleans::edge_lookup::{PointLocation, locate_point_on_face};
use crate::booleans::intersect::FaceIntersection;
use crate::booleans::split::{EndpointVertices, SplitOutcome, ensure_vertex_at};
use crate::geometry::CurveSegment;

/// Result of phase B: per-intersection vertex resolution + which chords are
/// already in the topology (added as the mev tail).
#[derive(Clone, Debug)]
pub struct InteriorResolution {
    /// `endpoints[i]` is the resolved (start, end) vertex pair for intersection
    /// `i` after phase B. Always `Some` if phase B succeeded.
    pub endpoints: Vec<Option<(EndpointVertices, EndpointVertices)>>,
    /// `chord_already_added_a[i]` is true if the chord on `face_a` for
    /// intersection `i` was added by phase B's mev tail (so phase C must skip).
    pub chord_already_added_a: Vec<bool>,
    /// Same for solid B's `face_b`.
    pub chord_already_added_b: Vec<bool>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Side {
    A,
    B,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum EndpointSide {
    Start,
    End,
}

type Key = [i64; 3];

fn quantize(p: Point3, tol: &Tolerance) -> Key {
    let q = (1.0_f64 / tol.point_eq.max(1e-15)).max(1.0);
    [
        (p.x * q).round() as i64,
        (p.y * q).round() as i64,
        (p.z * q).round() as i64,
    ]
}

/// Resolve all deferred (Interior) intersection endpoints.
///
/// For each intersection where `split_outcome.endpoints[i] = None`, this
/// classifies which endpoint is interior in which solid, then uses an mev tail
/// from a sibling intersection's boundary endpoint to grow the interior vertex.
/// Subsequent siblings sharing the same interior point reuse the now-existing
/// vertex.
pub fn resolve_interior_endpoints(
    a: &mut Solid,
    b: &mut Solid,
    intersections: &[FaceIntersection],
    split_outcome: &SplitOutcome,
    tol: &Tolerance,
) -> InteriorResolution {
    let n = intersections.len();
    let mut endpoints: Vec<Option<(EndpointVertices, EndpointVertices)>> = vec![None; n];
    let mut chord_already_added_a = vec![false; n];
    let mut chord_already_added_b = vec![false; n];

    // Pre-classify each intersection's endpoints in both solids. We refresh
    // boundary classifications lazily — split_outcome already did the OnEdge
    // splits in phase A, so locate_point_on_face will return OnVertex for those.
    //
    // Per-solid map from (face, quantized_interior_point) to the VertexId we
    // create via mev.
    let mut interior_vertex_a: HashMap<(FaceId, Key), VertexId> = HashMap::new();
    let mut interior_vertex_b: HashMap<(FaceId, Key), VertexId> = HashMap::new();

    // First pass: copy resolutions for intersections that fully resolved in phase A.
    for (i, slot) in split_outcome.endpoints.iter().enumerate() {
        if let Some(pair) = slot {
            endpoints[i] = Some(*pair);
        }
        let _ = i;
    }

    // Second pass: handle deferred ones.
    for i in 0..n {
        if endpoints[i].is_some() {
            continue;
        }

        let inter = &intersections[i];

        // Resolve start / end for solid A (face_a).
        let start_va = resolve_one_endpoint(
            a,
            inter.face_a,
            inter.start,
            tol,
            Side::A,
            EndpointSide::Start,
            i,
            intersections,
            split_outcome,
            &mut interior_vertex_a,
            &mut chord_already_added_a,
        );
        let end_va = resolve_one_endpoint(
            a,
            inter.face_a,
            inter.end,
            tol,
            Side::A,
            EndpointSide::End,
            i,
            intersections,
            split_outcome,
            &mut interior_vertex_a,
            &mut chord_already_added_a,
        );
        // Solid B (face_b).
        let start_vb = resolve_one_endpoint(
            b,
            inter.face_b,
            inter.start,
            tol,
            Side::B,
            EndpointSide::Start,
            i,
            intersections,
            split_outcome,
            &mut interior_vertex_b,
            &mut chord_already_added_b,
        );
        let end_vb = resolve_one_endpoint(
            b,
            inter.face_b,
            inter.end,
            tol,
            Side::B,
            EndpointSide::End,
            i,
            intersections,
            split_outcome,
            &mut interior_vertex_b,
            &mut chord_already_added_b,
        );

        endpoints[i] = Some((
            EndpointVertices {
                vertex_a: start_va,
                vertex_b: start_vb,
            },
            EndpointVertices {
                vertex_a: end_va,
                vertex_b: end_vb,
            },
        ));
    }

    InteriorResolution {
        endpoints,
        chord_already_added_a,
        chord_already_added_b,
    }
}

/// Resolve one endpoint (start or end) of intersection `i` against `solid`'s `face`.
/// If it's OnVertex / OnEdge, just return the vertex (after splitting if needed).
/// If it's Interior, find a sibling intersection that has this point as ITS
/// endpoint AND a boundary endpoint we can anchor mev on.
#[allow(clippy::too_many_arguments)]
fn resolve_one_endpoint(
    solid: &mut Solid,
    face: FaceId,
    p: Point3,
    tol: &Tolerance,
    side: Side,
    _endpoint_side: EndpointSide,
    intersection_idx: usize,
    intersections: &[FaceIntersection],
    split_outcome: &SplitOutcome,
    interior_map: &mut HashMap<(FaceId, Key), VertexId>,
    chord_already_added: &mut [bool],
) -> VertexId {
    // Fast path: existing interior vertex from a previous mev call.
    let key = (face, quantize(p, tol));
    if let Some(&v) = interior_map.get(&key) {
        return v;
    }

    // Try the simple boundary path first: point is on a vertex / edge.
    if let Some(v) = ensure_vertex_at(solid, face, p, tol) {
        return v;
    }

    // It's interior. Find a sibling intersection that:
    //   - touches the same face on this side,
    //   - has THIS point as one of its endpoints,
    //   - has its OTHER endpoint already resolved as a boundary vertex on this face.
    // The sibling's chord (boundary vertex → this interior point) is what mev produces.
    let sibling = find_sibling_with_boundary(
        solid,
        face,
        p,
        tol,
        side,
        intersection_idx,
        intersections,
        split_outcome,
    );

    // Find face's outer loop once — needed by both the sibling and stinger paths.
    let outer_loop = solid
        .topo
        .face(face)
        .expect("face exists")
        .outer_loop();

    let (sibling_idx, boundary_vertex_opt) = match sibling {
        Some((i, b)) => (Some(i), Some(b)),
        None => (None, None),
    };

    // Pick a mev anchor:
    //   - If we have a sibling with a boundary endpoint, anchor on that vertex
    //     (so the mev tail IS the sibling's chord).
    //   - Otherwise (M36 stinger path), use ANY half-edge in the outer loop;
    //     the mev tail becomes a non-chord stinger edge from a boundary vertex
    //     to the orphan interior point. This handles closed-polygon
    //     intersections (cylinder ∪ box family) where no chord touches the
    //     boundary. The face's outer loop ends up with a fjord — topologically
    //     valid, geometrically a spike.
    let anchor = if let Some(b) = boundary_vertex_opt {
        find_he_ending_at(solid, outer_loop, b).unwrap_or_else(|| {
            panic!(
                "M11 phase B: face {face:?} outer loop has no half-edge ending at boundary vertex \
                 {b:?}. (interior point {p:?})"
            )
        })
    } else {
        // Stinger path: pick the loop's representative half-edge.
        let lp = solid
            .topo
            .loop_(outer_loop)
            .expect("face's outer loop exists");
        lp.half_edge()
            .expect("outer loop has at least one half-edge")
    };

    // mev: grows a sticker edge from anchor.dest to a new vertex at p.
    let mev = solid.topo.mev(outer_loop, anchor);
    solid.vertex_geom.insert(mev.vertex, p);

    // Edge geometry: line from anchor.dest to interior point.
    // anchor.dest is the destination vertex of the anchor half-edge, which is
    // the same as the twin's origin.
    let anchor_he = solid
        .topo
        .half_edge(anchor)
        .expect("anchor half-edge exists");
    let v_dest = solid
        .topo
        .half_edge(anchor_he.twin())
        .expect("twin exists")
        .origin();
    let p_dest = *solid
        .vertex_geom
        .get(v_dest)
        .expect("anchor destination has a position");
    if let Some(line) = Line::through(p_dest, p) {
        let length = (p - p_dest).norm();
        let seg = CurveSegment::line(line, 0.0, length);
        solid.edge_geom.insert(mev.edge, seg);
    }

    interior_map.insert(key, mev.vertex);

    // If the mev tail IS a sibling's chord, mark it as added so phase C skips.
    // For the stinger path, the tail is artificial (not a chord) — don't mark.
    if let Some(sib) = sibling_idx {
        chord_already_added[sib] = true;
    }

    mev.vertex
}

/// Look for a sibling FaceIntersection (different index) whose:
/// - face on `side` is the same as `face`,
/// - one endpoint equals `p` (the interior point we're trying to resolve),
/// - the OTHER endpoint is a boundary vertex on `face` (so we can anchor mev).
///
/// Returns `(sibling_idx, boundary_vertex_on_face)` if found.
#[allow(clippy::too_many_arguments)]
fn find_sibling_with_boundary(
    solid: &Solid,
    face: FaceId,
    p: Point3,
    tol: &Tolerance,
    side: Side,
    self_idx: usize,
    intersections: &[FaceIntersection],
    split_outcome: &SplitOutcome,
) -> Option<(usize, VertexId)> {
    let p_key = quantize(p, tol);
    for (j, sib) in intersections.iter().enumerate() {
        if j == self_idx {
            continue;
        }
        let sib_face = match side {
            Side::A => sib.face_a,
            Side::B => sib.face_b,
        };
        if sib_face != face {
            continue;
        }
        // Which sibling endpoint matches `p`?
        let sib_start_matches = quantize(sib.start, tol) == p_key;
        let sib_end_matches = quantize(sib.end, tol) == p_key;
        if !sib_start_matches && !sib_end_matches {
            continue;
        }

        // The sibling's OTHER endpoint must be on the boundary of `face` (in `solid`).
        let other_p = if sib_start_matches { sib.end } else { sib.start };
        match locate_point_on_face(solid, face, other_p, tol) {
            PointLocation::OnVertex(v) => {
                // Verify sibling's split_outcome resolved successfully (so it's
                // safe to use as anchor).
                if split_outcome.endpoints[j].is_some() {
                    return Some((j, v));
                }
                // Even if the sibling's other side has an interior, this side
                // is fully boundary-resolvable, which is fine for our use.
                return Some((j, v));
            }
            _ => {
                // Sibling's other endpoint isn't directly OnVertex. It might be
                // OnEdge — but phase A would have split that already, so it
                // should be OnVertex now. If not, skip.
                continue;
            }
        }
    }
    None
}

/// Walk `loop_id` and return the first half-edge whose `next.origin == target`.
/// (Equivalent: half-edge whose destination is `target`.)
fn find_he_ending_at(
    solid: &Solid,
    loop_id: LoopId,
    target: VertexId,
) -> Option<HalfEdgeId> {
    let lp = solid.topo.loop_(loop_id)?;
    let start = lp.half_edge()?;
    let mut cur = start;
    loop {
        let he = solid.topo.half_edge(cur)?;
        let nxt = solid.topo.half_edge(he.next())?;
        if nxt.origin() == target {
            return Some(cur);
        }
        cur = he.next();
        if cur == start {
            break;
        }
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;
    use kerf_geom::{Tolerance, Vec3};

    use crate::booleans::{face_intersections, split_solids_at_intersections};
    use crate::primitives::{box_, box_at};

    #[test]
    fn corner_cut_box_difference_does_not_panic() {
        // A = [0,10]^3, B = [8,12]^3 — they share a [8,10]^3 corner overlap.
        // On A's z=10 face, the boundary intersections produce two chords
        // sharing the interior point (8,8,10).
        let block = box_(Vec3::new(10.0, 10.0, 10.0));
        let cutter = box_at(Vec3::new(4.0, 4.0, 4.0), Point3::new(8.0, 8.0, 8.0));
        let mut a = block.clone();
        let mut b = cutter.clone();

        let tol = Tolerance::default();
        let intersections = face_intersections(&a, &b, &tol);
        let outcome = split_solids_at_intersections(&mut a, &mut b, &intersections, &tol);
        let _interior = resolve_interior_endpoints(&mut a, &mut b, &intersections, &outcome, &tol);
        // Just verify it doesn't panic and topology is still valid.
        kerf_topo::validate(&a.topo).unwrap();
        kerf_topo::validate(&b.topo).unwrap();
    }

    #[test]
    fn corner_cut_marks_some_chords_already_added() {
        // The corner-cut box has 6 face intersections; phase B should mark
        // half of each face's chords as already added (mev tails).
        let block = box_(Vec3::new(10.0, 10.0, 10.0));
        let cutter = box_at(Vec3::new(4.0, 4.0, 4.0), Point3::new(8.0, 8.0, 8.0));
        let mut a = block.clone();
        let mut b = cutter.clone();
        let tol = Tolerance::default();
        let intersections = face_intersections(&a, &b, &tol);
        let outcome = split_solids_at_intersections(&mut a, &mut b, &intersections, &tol);
        let interior =
            resolve_interior_endpoints(&mut a, &mut b, &intersections, &outcome, &tol);

        // Across both solids, at least three chords should be added by the mev
        // tail (one per face that has a shared interior point).
        let added_a = interior.chord_already_added_a.iter().filter(|x| **x).count();
        let added_b = interior.chord_already_added_b.iter().filter(|x| **x).count();
        assert!(
            added_a >= 3 && added_b >= 3,
            "expected >=3 mev-tail chords per side, got A={added_a} B={added_b}"
        );
    }
}
