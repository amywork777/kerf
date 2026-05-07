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
    /// Picking provenance carried over from the source solid's face_owner_tag.
    /// Threaded into the resulting Solid's face_owner_tag by `stitch`.
    /// `None` for faces whose source had no owner.
    pub owner: Option<String>,
}

/// Build a connected `Solid` from a set of kept faces. Faces that share an edge
/// (within tolerance) become adjacent in the resulting solid.
///
/// Panics if the input is non-manifold (any canonical edge has != 2 incident
/// half-edges) or if `validate()` rejects the resulting topology.
pub fn stitch(kept: &[KeptFace], tol: &Tolerance) -> Solid {
    stitch_with_rescue(kept, &[], tol)
}

/// Like [`stitch`], but with a "rescue from dropped" pre-pass: when the kept
/// pile would leave a canonical edge with exactly one half-edge (i.e., one
/// directed edge in the kept polygons has no twin among the kept set), search
/// the `dropped` pile for a coplanar face whose polygon contains the reverse
/// directed edge and promote it back into kept.
///
/// This is the GAP C multi-edge fillet repair: a sequential subtract chain
/// (e.g., Box - wedge1 - wedge2 - wedge3 - wedge4 for a 4-corner Fillets)
/// occasionally drops a face that shares an edge with a kept face's
/// boundary, leaving a single half-edge orphan. Promoting the dropped
/// partner closes the loop without altering volume (the dropped face was
/// already contributing surface to the boolean — it just got mis-classified
/// at the boundary).
///
/// Falls back to the same panic as [`stitch`] when no rescue partner exists.
pub fn stitch_with_rescue(
    kept: &[KeptFace],
    dropped: &[KeptFace],
    tol: &Tolerance,
) -> Solid {
    let kept_owned = rescue_one_half_edge_orphans(kept, dropped, tol);
    stitch_inner(&kept_owned, tol)
}

fn stitch_inner(kept: &[KeptFace], tol: &Tolerance) -> Solid {
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

    // Stage 1b: T-junction healing. If a polygon edge (a, b) passes through
    // a vertex c that exists in some other polygon, insert c into this
    // polygon between a and b. Synchronizes polygon vertex sets at shared
    // edges so stitch's per-edge twin pairing finds matched partners.
    // Iterates until fixpoint.
    let n_positions = positions.len();
    loop {
        let mut changed = false;
        for face_idx in 0..face_to_vidx.len() {
            let mut i = 0;
            while i < face_to_vidx[face_idx].len() {
                let n = face_to_vidx[face_idx].len();
                let v_a = face_to_vidx[face_idx][i];
                let v_b = face_to_vidx[face_idx][(i + 1) % n];
                let a = positions[v_a];
                let b = positions[v_b];
                let ab = b - a;
                let ab_len_sq = ab.norm_squared();
                if ab_len_sq < tol.point_eq * tol.point_eq {
                    i += 1;
                    continue;
                }
                let mut found: Option<usize> = None;
                for c_idx in 0..n_positions {
                    if c_idx == v_a || c_idx == v_b {
                        continue;
                    }
                    let c = positions[c_idx];
                    let ac = c - a;
                    let t = ac.dot(&ab) / ab_len_sq;
                    if !(t > 1e-6 && t < 1.0 - 1e-6) {
                        continue;
                    }
                    let proj = a + t * ab;
                    if (c - proj).norm() < tol.point_eq * 100.0 {
                        found = Some(c_idx);
                        break;
                    }
                }
                if let Some(c_idx) = found {
                    face_to_vidx[face_idx].insert(i + 1, c_idx);
                    changed = true;
                } else {
                    i += 1;
                }
            }
        }
        if !changed {
            break;
        }
    }

    // Stage 1b: coplanar-duplicate dedup. Two kept faces with the same cyclic
    // vertex sequence (after rotating to a canonical start) are the same face
    // emitted twice — usually an OnBoundary face that survived classification
    // on both A and B sides with matching orientation. Reverse-winding
    // duplicates are NOT collapsed (they're legitimate front/back pairs).
    let kept_indices: Vec<usize> = {
        let mut seen: std::collections::HashMap<Vec<usize>, usize> =
            std::collections::HashMap::new();
        let mut keep: Vec<usize> = Vec::with_capacity(kept.len());
        for (i, vidx) in face_to_vidx.iter().enumerate() {
            if vidx.len() < 3 {
                continue;
            }
            let key = canonical_cycle(vidx);
            if seen.contains_key(&key) {
                continue;
            }
            seen.insert(key, i);
            keep.push(i);
        }
        keep
    };
    let kept: Vec<&KeptFace> = kept_indices.iter().map(|&i| &kept[i]).collect();
    let face_to_vidx: Vec<Vec<usize>> = kept_indices
        .iter()
        .map(|&i| face_to_vidx[i].clone())
        .collect();

    // Stage 1c (M39c): drop orphan-contributor faces. An "orphan" arises
    // when a canonical edge has 3+ half-edges contributed by 3+ different
    // kept faces; pick_twin_pair selects 2, leaving the rest as half-edges
    // with no twin (→ AsymmetricTwin in `validate`). Detect such faces
    // ahead of stage 4 and drop them. Iterate to fixpoint because dropping
    // a face changes other edges' contribution counts.
    //
    // Heuristic for which face to drop when multiple compete: drop the face
    // with the most "conflicting" edges (edges currently with 3+ entries).
    // Ties broken by higher index (later-added faces are more likely the
    // duplicates from the second-solid pass).
    let kept: Vec<KeptFace> = kept.iter().map(|kf| (*kf).clone()).collect();
    let (kept, face_to_vidx) = drop_orphan_contributors(kept, face_to_vidx);
    let kept: Vec<&KeptFace> = kept.iter().collect();

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

    let mut half_edge_records: Vec<HalfEdgeRecord> = Vec::new();
    let mut edge_pairs: HashMap<(usize, usize), Vec<usize>> = HashMap::new();
    let mut face_ids: Vec<FaceId> = Vec::with_capacity(kept.len());
    // M39f: self-loop half-edges (v_start == v_end, e.g., vase apex after
    // global vertex dedup collapses multiple coincident apex vertices). They
    // can't be paired as twins (direction is ambiguous on a self-loop), so
    // we self-twin them after stage 5 to satisfy the AsymmetricTwin invariant
    // and keep them out of edge_pairs to avoid spurious "1 half-edge" panics.
    let mut self_loops: Vec<kerf_topo::HalfEdgeId> = Vec::new();

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
        if let Some(tag) = &kf.owner {
            new_solid.face_owner_tag.insert(face_id, tag.clone());
        }
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
            if v_start_idx == v_end_idx {
                // Self-loop: bypass edge pairing.
                self_loops.push(he_id);
            } else {
                let key = canonical_edge_key(v_start_idx, v_end_idx);
                edge_pairs
                    .entry(key)
                    .or_default()
                    .push(half_edge_records.len() - 1);
            }
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
    // exactly two half-edges (one per adjacent face) — but in practice the
    // boolean classifier sometimes keeps coplanar duplicates that produce
    // 3-or-more entries on a single edge. Partition by direction and pick
    // a proper twin pair (one (u,v), one (v,u)); drop the rest. Anything
    // else (all in one direction, or only 1 entry) really is non-manifold.
    for (key, indices) in &edge_pairs {
        let (he_a_record_idx, he_b_record_idx) = match pick_twin_pair(indices, &half_edge_records, *key) {
            Some(pair) => pair,
            None => {
                panic!(
                    "non-manifold input to stitch: edge key {key:?} has {} half-edges (expected 2)",
                    indices.len()
                );
            }
        };
        let he_a = half_edge_records[he_a_record_idx].id;
        let he_b = half_edge_records[he_b_record_idx].id;

        let edge_id = new_solid.topo.build_insert_edge([he_a, he_b]);
        new_solid.topo.build_set_half_edge_twin(he_a, he_b);
        new_solid.topo.build_set_half_edge_twin(he_b, he_a);
        new_solid.topo.build_set_half_edge_edge(he_a, edge_id);
        new_solid.topo.build_set_half_edge_edge(he_b, edge_id);

        // Edge geometry: line between the two endpoints (in he_a's order).
        // If the two endpoints coincide (zero-length edge — usually a vase
        // apex or coincident-vertex artifact in degenerate input), skip
        // assigning edge geometry; the topology stays valid (twin pointers
        // wired) and downstream tessellation degenerates gracefully.
        let v_idx_a = half_edge_records[he_a_record_idx].v_start;
        let v_idx_b = half_edge_records[he_a_record_idx].v_end;
        let p0 = positions[v_idx_a];
        let p1 = positions[v_idx_b];
        if let Some(line) = Line::through(p0, p1) {
            let length = (p1 - p0).norm();
            let seg = CurveSegment::line(line, 0.0, length);
            new_solid.edge_geom.insert(edge_id, seg);
        }
    }

    // M39f: self-twin all self-loop half-edges. Each gets its own degenerate
    // edge with both half-edge slots pointing back to the same half-edge —
    // satisfies validate's twin.twin == self check without polluting the
    // manifold edge set. No edge geometry (zero-length).
    for &he in &self_loops {
        let edge_id = new_solid.topo.build_insert_edge([he, he]);
        new_solid.topo.build_set_half_edge_twin(he, he);
        new_solid.topo.build_set_half_edge_edge(he, edge_id);
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

struct HalfEdgeRecord {
    id: kerf_topo::HalfEdgeId,
    v_start: usize,
    v_end: usize,
    face_idx: usize,
}

/// Given a list of half-edge record indices that all share one canonical
/// edge key (low, high), pick one half-edge in each direction (low→high and
/// high→low) so they twin properly. Returns None if either direction is
/// empty (truly non-manifold input). Extra half-edges in either direction
/// are simply skipped — usually a sign of coplanar duplicate kept faces.
fn pick_twin_pair(
    indices: &[usize],
    half_edge_records: &[HalfEdgeRecord],
    key: (usize, usize),
) -> Option<(usize, usize)> {
    let mut forward = None;
    let mut backward = None;
    for &i in indices {
        let r = &half_edge_records[i];
        if (r.v_start, r.v_end) == key && forward.is_none() {
            forward = Some(i);
        } else if (r.v_end, r.v_start) == key && backward.is_none() {
            backward = Some(i);
        }
        if forward.is_some() && backward.is_some() {
            break;
        }
    }
    match (forward, backward) {
        (Some(f), Some(b)) => Some((f, b)),
        _ => None,
    }
}

/// Rotate a cyclic sequence so it starts at its minimum element. Two
/// sequences are equivalent under cyclic rotation iff their canonical forms
/// are equal.
fn canonical_cycle(indices: &[usize]) -> Vec<usize> {
    let n = indices.len();
    if n == 0 {
        return vec![];
    }
    let (min_pos, _) = indices
        .iter()
        .enumerate()
        .min_by_key(|&(_, v)| *v)
        .unwrap();
    let mut out = Vec::with_capacity(n);
    for k in 0..n {
        out.push(indices[(min_pos + k) % n]);
    }
    out
}

fn canonical_edge_key(a: usize, b: usize) -> (usize, usize) {
    if a < b { (a, b) } else { (b, a) }
}

/// M39c: drop kept faces that contribute orphan half-edges. A canonical
/// edge key with 3+ entries means 3+ kept faces share that polygon edge —
/// only one forward and one backward survive `pick_twin_pair`, leaving the
/// rest as half-edges with no twin (AsymmetricTwin in validation).
///
/// Iterate to fixpoint: dropping a face changes other edges' counts and
/// may resolve them. Heuristic for which face to drop: the one with the
/// most edges currently in conflict (3+ entries), tie-broken by higher
/// index (later-added faces — usually B's faces in the boolean — are more
/// often the duplicates).
fn drop_orphan_contributors(
    kept: Vec<KeptFace>,
    face_to_vidx: Vec<Vec<usize>>,
) -> (Vec<KeptFace>, Vec<Vec<usize>>) {
    let mut keep_mask: Vec<bool> = vec![true; kept.len()];
    loop {
        // Build edge -> [(face_idx, direction)] across currently-kept faces.
        // Direction true = forward (matches canonical key order); false = backward.
        let mut edge_to_dir_faces: HashMap<(usize, usize), Vec<(usize, bool)>> = HashMap::new();
        for (face_idx, vidx) in face_to_vidx.iter().enumerate() {
            if !keep_mask[face_idx] {
                continue;
            }
            let n = vidx.len();
            for i in 0..n {
                let v_start = vidx[i];
                let v_end = vidx[(i + 1) % n];
                if v_start == v_end {
                    // Self-loop edges (e.g., M39f vase apex collapse) are
                    // routed around edge_pairs in stage 4, so they don't
                    // contribute to twin-pairing conflicts here.
                    continue;
                }
                let key = canonical_edge_key(v_start, v_end);
                let forward = (v_start, v_end) == key;
                edge_to_dir_faces.entry(key).or_default().push((face_idx, forward));
            }
        }

        // Count conflicts per face. An edge is "in conflict" if:
        //   - it has 3+ entries (orphan-contributor case, M39c), OR
        //   - it has 2+ entries in the SAME direction (M39h: same-direction
        //     duplicates that can't be paired as twins).
        let mut conflicts: Vec<usize> = vec![0; kept.len()];
        let mut any_conflict = false;
        for entries in edge_to_dir_faces.values() {
            let total = entries.len();
            let forward_count = entries.iter().filter(|(_, fw)| *fw).count();
            let backward_count = total - forward_count;
            let bad = total >= 3 || forward_count >= 2 || backward_count >= 2;
            if !bad {
                continue;
            }
            any_conflict = true;
            for &(f, _) in entries {
                conflicts[f] += 1;
            }
        }
        if !any_conflict {
            break;
        }

        // Pick the highest-conflict face (ties broken by higher index).
        let mut victim: Option<usize> = None;
        let mut victim_count = 0usize;
        for (i, &c) in conflicts.iter().enumerate() {
            if !keep_mask[i] || c == 0 {
                continue;
            }
            if c > victim_count || (c == victim_count && Some(i) > victim) {
                victim = Some(i);
                victim_count = c;
            }
        }
        let Some(v) = victim else {
            break;
        };
        keep_mask[v] = false;
    }

    let new_kept: Vec<KeptFace> = kept
        .into_iter()
        .enumerate()
        .filter_map(|(i, kf)| keep_mask[i].then_some(kf))
        .collect();
    let new_face_to_vidx: Vec<Vec<usize>> = face_to_vidx
        .into_iter()
        .enumerate()
        .filter_map(|(i, v)| keep_mask[i].then_some(v))
        .collect();
    (new_kept, new_face_to_vidx)
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

/// GAP C: rescue 1-half-edge orphans from the dropped pool.
///
/// After the boolean classifier drops some faces, a kept polygon's edge can
/// be left without a partner — typically a sequential subtract that drops a
/// boundary-coincident face that was actually still needed to close the
/// outer surface (or a partial shadowing of a body face by a cutter).
///
/// For each canonical edge (u, v) appearing exactly once in the kept set,
/// look in `dropped` for a coplanar face whose polygon contains the reverse
/// directed edge (v, u). Promote the first such match to kept and recheck.
/// Iterate until no rescue is possible.
///
/// Position-equality is computed at a relaxed tolerance (`point_eq * 1000`),
/// matching `chord_merge`'s practice (default `point_eq` is 1e-9 which is
/// tighter than typical CAD precision and would miss legitimate matches).
///
/// This pass is conservative: it never drops, never modifies polygons. It
/// only PROMOTES dropped faces. If no rescue applies, the kept pile is
/// returned unchanged.
fn rescue_one_half_edge_orphans(
    kept: &[KeptFace],
    dropped: &[KeptFace],
    tol: &Tolerance,
) -> Vec<KeptFace> {
    if dropped.is_empty() {
        return kept.to_vec();
    }
    let pt_tol = tol.point_eq * 1000.0;

    let mut kept = kept.to_vec();
    let mut available: Vec<bool> = vec![true; dropped.len()];
    // Cap iterations as a safety net — each iteration must promote a face.
    for _ in 0..(dropped.len().max(1)) {
        // Build a vertex dedup table over the current kept polygons.
        let mut positions: Vec<Point3> = Vec::new();
        let mut face_vidx: Vec<Vec<usize>> = Vec::with_capacity(kept.len());
        for kf in &kept {
            let mut indices = Vec::with_capacity(kf.polygon.len());
            for p in &kf.polygon {
                let idx = find_or_add(&mut positions, *p, tol);
                indices.push(idx);
            }
            face_vidx.push(indices);
        }
        // Edge → directed entries (forward = matches canonical key order).
        let mut edge_dir: HashMap<(usize, usize), Vec<bool>> = HashMap::new();
        for vidx in &face_vidx {
            let n = vidx.len();
            for i in 0..n {
                let a = vidx[i];
                let b = vidx[(i + 1) % n];
                if a == b {
                    continue;
                }
                let key = canonical_edge_key(a, b);
                let forward = (a, b) == key;
                edge_dir.entry(key).or_default().push(forward);
            }
        }
        // Identify canonical edges with exactly one half-edge entry. The
        // missing direction is the one we need a dropped face to provide.
        let mut singles: Vec<((usize, usize), bool)> = Vec::new();
        for (key, dirs) in &edge_dir {
            if dirs.len() == 1 {
                singles.push((*key, dirs[0]));
            }
        }
        if singles.is_empty() {
            return kept;
        }
        // Find a dropped face whose polygon walks the reverse of any single
        // edge AND whose surface is coplanar with at least one currently-kept
        // face that touches that edge (avoids promoting unrelated geometry
        // that happens to share two vertex positions).
        let mut promoted = false;
        for ((u, v), present_forward) in singles {
            // Reverse directed edge in 3D.
            let p_u = positions[u];
            let p_v = positions[v];
            let (ra, rb) = if present_forward { (p_v, p_u) } else { (p_u, p_v) };
            // Identify a kept face that owns this edge so we know the
            // coplanarity reference.
            let mut ref_surface: Option<SurfaceKind> = None;
            'outer: for (fi, vidx) in face_vidx.iter().enumerate() {
                let n = vidx.len();
                for i in 0..n {
                    let a = vidx[i];
                    let b = vidx[(i + 1) % n];
                    if (a, b) == (u, v) || (a, b) == (v, u) {
                        ref_surface = Some(kept[fi].surface.clone());
                        break 'outer;
                    }
                }
            }
            let Some(ref_surface) = ref_surface else {
                continue;
            };
            for di in 0..dropped.len() {
                if !available[di] {
                    continue;
                }
                if !surfaces_coplanar(&dropped[di].surface, &ref_surface, tol) {
                    continue;
                }
                if polygon_has_directed_edge(&dropped[di].polygon, ra, rb, pt_tol) {
                    kept.push(dropped[di].clone());
                    available[di] = false;
                    promoted = true;
                    break;
                }
            }
            if promoted {
                break;
            }
        }
        if !promoted {
            return kept;
        }
    }
    kept
}

/// Coplanar test mirroring `chord_merge::coplanar` so the rescue pass agrees
/// with chord-merge on which surfaces "are the same plane". Curved surface
/// kinds are out of scope (rescue only repairs planar 1-half-edge orphans).
fn surfaces_coplanar(s1: &SurfaceKind, s2: &SurfaceKind, tol: &Tolerance) -> bool {
    match (s1, s2) {
        (SurfaceKind::Plane(p1), SurfaceKind::Plane(p2)) => {
            let n1 = p1.frame.z;
            let n2 = p2.frame.z;
            let parallel = (n1.dot(&n2).abs() - 1.0).abs() < 1e-6;
            if !parallel {
                return false;
            }
            let d = (p2.frame.origin - p1.frame.origin).dot(&n1);
            d.abs() < tol.point_eq * 1000.0
        }
        _ => false,
    }
}

/// Does `poly` contain the directed edge (a → b) within point-tolerance?
fn polygon_has_directed_edge(poly: &[Point3], a: Point3, b: Point3, pt_tol: f64) -> bool {
    let n = poly.len();
    for i in 0..n {
        if (poly[i] - a).norm() <= pt_tol && (poly[(i + 1) % n] - b).norm() <= pt_tol {
            return true;
        }
    }
    false
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
            kept.push(KeptFace { polygon, surface, owner: None });
        }
        let new_s = stitch(&kept, &Tolerance::default());
        assert_eq!(new_s.vertex_count(), 8);
        assert_eq!(new_s.edge_count(), 12);
        assert_eq!(new_s.face_count(), 6);
        validate(&new_s.topo).unwrap();
    }
}
