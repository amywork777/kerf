//! Build a connected `Solid` from kept face polygons via direct slotmap construction.
//!
//! Direct construction (rather than driving Euler operators backward) is the
//! right tool when we already know the full target topology — as we do after
//! the boolean classifier picks which faces to keep. We dedupe vertices by 3D
//! position, build all half-edges per face polygon, then pair them up by
//! canonical edge-key to wire twins. `validate()` is the safety net.

use std::collections::HashMap;

use kerf_geom::{Line, Point3, Tolerance};
use kerf_topo::{validate, FaceId};

use crate::Solid;
use crate::geometry::{CurveSegment, SurfaceKind};

/// One kept face's contribution: its polygon (3D vertex positions in CCW order
/// when viewed from the face's outward normal) and its surface geometry.
#[derive(Clone, Debug)]
pub struct KeptFace {
    pub polygon: Vec<Point3>,
    pub surface: SurfaceKind,
}

/// Build a connected `Solid` from a set of kept faces. Faces that share an edge
/// (within tolerance) become adjacent in the resulting solid.
///
/// Panics if the input is non-manifold (any canonical edge has != 2 incident
/// half-edges) or if `validate()` rejects the resulting topology.
pub fn stitch(kept: &[KeptFace], tol: &Tolerance) -> Solid {
    let mut new_solid = Solid::new();

    if kept.is_empty() {
        return new_solid;
    }

    // Stage 1: vertex dedup. positions[i] is the 3D point for the i-th unique vertex.
    let mut positions: Vec<Point3> = Vec::new();
    let mut face_to_vidx: Vec<Vec<usize>> = Vec::with_capacity(kept.len());
    for kf in kept {
        let mut indices = Vec::with_capacity(kf.polygon.len());
        for p in &kf.polygon {
            let idx = find_or_add(&mut positions, *p, tol);
            indices.push(idx);
        }
        face_to_vidx.push(indices);
    }

    // Stage 2: create vertices in the kerf-topo solid.
    let vids: Vec<_> = (0..positions.len())
        .map(|_| new_solid.topo.build_insert_vertex())
        .collect();
    for (i, p) in positions.iter().enumerate() {
        new_solid.vertex_geom.insert(vids[i], *p);
    }

    // Stage 3: solid bookkeeping (shells allocated after component analysis).
    let solid_id = new_solid.topo.build_insert_solid();
    new_solid.topo.build_set_active_solid(Some(solid_id));

    // Stage 4: per-face — create face/loop, then half-edges per polygon edge.
    // We use a temporary placeholder shell (index 0) for now; it is patched
    // after connected-component analysis in Stage 5b.
    let placeholder_shell = new_solid.topo.build_insert_shell(solid_id);

    struct HalfEdgeRecord {
        id: kerf_topo::HalfEdgeId,
        v_start: usize,
        v_end: usize,
        face_idx: usize,
    }
    let mut half_edge_records: Vec<HalfEdgeRecord> = Vec::new();
    let mut edge_pairs: HashMap<(usize, usize), Vec<usize>> = HashMap::new();
    let mut face_ids: Vec<FaceId> = Vec::with_capacity(kept.len());

    for (face_idx, kf) in kept.iter().enumerate() {
        let n = kf.polygon.len();
        debug_assert!(n >= 3, "face polygon must have at least 3 vertices");

        let loop_id = new_solid.topo.build_insert_loop_placeholder();
        // Use placeholder_shell temporarily; corrected in Stage 5b.
        let face_id = new_solid
            .topo
            .build_insert_face(loop_id, placeholder_shell);
        new_solid.topo.build_set_loop_face(loop_id, face_id);
        new_solid.face_geom.insert(face_id, kf.surface.clone());
        face_ids.push(face_id);

        // Allocate half-edges for each polygon edge.
        let mut he_ids = Vec::with_capacity(n);
        for i in 0..n {
            let v_start_idx = face_to_vidx[face_idx][i];
            let v_end_idx = face_to_vidx[face_idx][(i + 1) % n];
            let he_id = new_solid
                .topo
                .build_insert_half_edge(vids[v_start_idx], loop_id);
            he_ids.push(he_id);
            half_edge_records.push(HalfEdgeRecord {
                id: he_id,
                v_start: v_start_idx,
                v_end: v_end_idx,
                face_idx,
            });
            let key = canonical_edge_key(v_start_idx, v_end_idx);
            edge_pairs
                .entry(key)
                .or_default()
                .push(half_edge_records.len() - 1);
        }
        // Wire next/prev within this loop's cycle.
        for i in 0..n {
            let cur = he_ids[i];
            let nxt = he_ids[(i + 1) % n];
            new_solid.topo.build_set_half_edge_next_prev(cur, nxt);
        }
        new_solid
            .topo
            .build_set_loop_half_edge(loop_id, Some(he_ids[0]));
    }

    // Stage 5: create edges and wire twins. Each canonical pair MUST have
    // exactly two half-edges (one per adjacent face). Anything else is
    // non-manifold input.
    for (key, indices) in &edge_pairs {
        if indices.len() != 2 {
            panic!(
                "non-manifold input to stitch: edge key {key:?} has {} half-edges (expected 2)",
                indices.len()
            );
        }
        let he_a_record_idx = indices[0];
        let he_b_record_idx = indices[1];
        let he_a = half_edge_records[he_a_record_idx].id;
        let he_b = half_edge_records[he_b_record_idx].id;

        let edge_id = new_solid.topo.build_insert_edge([he_a, he_b]);
        new_solid.topo.build_set_half_edge_twin(he_a, he_b);
        new_solid.topo.build_set_half_edge_twin(he_b, he_a);
        new_solid.topo.build_set_half_edge_edge(he_a, edge_id);
        new_solid.topo.build_set_half_edge_edge(he_b, edge_id);

        // Edge geometry: line between the two endpoints (in he_a's order).
        let v_idx_a = half_edge_records[he_a_record_idx].v_start;
        let v_idx_b = half_edge_records[he_a_record_idx].v_end;
        let p0 = positions[v_idx_a];
        let p1 = positions[v_idx_b];
        let line = Line::through(p0, p1).expect("zero-length edge in stitched solid");
        let length = (p1 - p0).norm();
        let seg = CurveSegment::line(line, 0.0, length);
        new_solid.edge_geom.insert(edge_id, seg);
    }

    // Stage 5b: connected-component analysis → allocate one Shell per component.
    //
    // Build a face-adjacency list: two faces are adjacent iff they share a
    // canonical edge (i.e., appear together in some `edge_pairs` entry).
    let n_faces = kept.len();
    let mut adj: Vec<Vec<usize>> = vec![Vec::new(); n_faces];
    for indices in edge_pairs.values() {
        // indices has exactly 2 entries (validated above).
        let fi = half_edge_records[indices[0]].face_idx;
        let fj = half_edge_records[indices[1]].face_idx;
        if fi != fj {
            adj[fi].push(fj);
            adj[fj].push(fi);
        }
    }

    // BFS to assign component IDs.
    let mut component: Vec<Option<usize>> = vec![None; n_faces];
    let mut n_components: usize = 0;
    for start in 0..n_faces {
        if component[start].is_some() {
            continue;
        }
        let comp_id = n_components;
        n_components += 1;
        let mut queue = std::collections::VecDeque::new();
        queue.push_back(start);
        component[start] = Some(comp_id);
        while let Some(fi) = queue.pop_front() {
            for &fj in &adj[fi] {
                if component[fj].is_none() {
                    component[fj] = Some(comp_id);
                    queue.push_back(fj);
                }
            }
        }
    }

    // Allocate shells: the placeholder_shell is reused as component 0.
    // Additional components get fresh shells.
    let mut shell_ids: Vec<kerf_topo::ShellId> = Vec::with_capacity(n_components);
    shell_ids.push(placeholder_shell); // component 0 → placeholder_shell
    for _ in 1..n_components {
        shell_ids.push(new_solid.topo.build_insert_shell(solid_id));
    }

    // Assign each face to its component's shell.
    for (face_idx, face_id) in face_ids.iter().enumerate() {
        let comp_id = component[face_idx].expect("every face must belong to a component");
        let shell_id = shell_ids[comp_id];
        new_solid.topo.build_set_face_shell(*face_id, shell_id);
        new_solid.topo.build_push_shell_face(shell_id, *face_id);
    }

    // Stage 6: vertex outgoing references — first half-edge whose origin is
    // each vertex.
    for record in &half_edge_records {
        let v = vids[record.v_start];
        if new_solid
            .topo
            .vertex(v)
            .and_then(|vx| vx.outgoing())
            .is_none()
        {
            new_solid
                .topo
                .build_set_vertex_outgoing(v, Some(record.id));
        }
    }

    // Sanity check: stitched topology must satisfy all manifold + Euler invariants.
    validate(&new_solid.topo).expect("stitched topology violates invariants");

    new_solid
}

fn canonical_edge_key(a: usize, b: usize) -> (usize, usize) {
    if a < b { (a, b) } else { (b, a) }
}

fn find_or_add(positions: &mut Vec<Point3>, p: Point3, tol: &Tolerance) -> usize {
    for (i, q) in positions.iter().enumerate() {
        if (p - *q).norm() < tol.point_eq {
            return i;
        }
    }
    positions.push(p);
    positions.len() - 1
}

#[cfg(test)]
mod tests {
    use super::*;

    use kerf_geom::Vec3;

    use crate::booleans::face_polygon;
    use crate::primitives::box_;

    #[test]
    fn stitch_a_single_box_round_trips() {
        // Take an existing box's faces (each as a KeptFace) and stitch them
        // back into a new Solid. Should produce 8V/12E/6F with valid topology.
        let s = box_(Vec3::new(1.0, 1.0, 1.0));
        let mut kept = Vec::new();
        for face_id in s.topo.face_ids() {
            let polygon = face_polygon(&s, face_id).unwrap();
            let surface = s.face_geom.get(face_id).cloned().unwrap();
            kept.push(KeptFace { polygon, surface });
        }
        let new_s = stitch(&kept, &Tolerance::default());
        assert_eq!(new_s.vertex_count(), 8);
        assert_eq!(new_s.edge_count(), 12);
        assert_eq!(new_s.face_count(), 6);
        validate(&new_s.topo).unwrap();
    }
}
