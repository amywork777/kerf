//! Build a connected `Solid` from kept face polygons via direct slotmap construction.
//!
//! Direct construction (rather than driving Euler operators backward) is the
//! right tool when we already know the full target topology — as we do after
//! the boolean classifier picks which faces to keep. We dedupe vertices by 3D
//! position, build all half-edges per face polygon, then pair them up by
//! canonical edge-key to wire twins. `validate()` is the safety net.

use std::collections::HashMap;

use kerf_geom::{Frame, Line, Plane, Point3, Tolerance, Vec3};
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

    // Stage 1d (best-effort one-sided-boundary rescue): after orphan-
    // contributor pruning, some kept faces may still contribute half-edges
    // on canonical edges that have no opposite-direction partner
    // (1-direction-only). This shows up in:
    //   - LBracket + Counterbore (concave interior corner near cutter)
    //   - 4-corner z-edge Fillets (wedges share lateral body faces)
    // Both panic with "edge key (...) has 1 half-edges (expected 2)".
    //
    // The rescue greedily drops kept faces whose polygon edges land on
    // currently-unpaired canonical edges, iterating to fixpoint. It's a
    // safety-valve net: if more than `MAX_DROP_FRACTION` of kept faces
    // would have to be discarded, we abandon the rescue and let stage 5
    // panic as before (so the bug still surfaces clearly). For genuinely
    // closed kept sets (the common case) it's a no-op.
    //
    // It does NOT fully solve LBracket+Counterbore or 4-corner-fillet —
    // those are open-boundary cases where face-dropping cascades to drop
    // the whole solid; the proper fix is synthesizing patch faces from
    // the unpaired loop, which requires the original surface geometry.
    // Tracked as deferred work in the e2e_scenarios bug commentary.
    let (kept, face_to_vidx) = drop_one_sided_boundary(kept, face_to_vidx);

    // Stage 1e (synthesise-patch-face): if any one-sided directed half-edges
    // remain after 1d, walk them into closed loops and synthesise a new
    // planar patch face per loop so each canonical edge ends up with exactly
    // two half-edges. The patch face's polygon is the reverse of the loop
    // walk so its half-edges twin the existing one-sided ones.
    //
    // This is the proper fix for the failures 1d's face-dropping can't
    // close (LBracket + Counterbore, 4-corner z-edge Fillet chain). Loops
    // that aren't planar within tolerance are left alone — stage 5 will
    // still panic for those, surfacing the bug. Synthesised faces inherit
    // no picking owner.
    let (kept, face_to_vidx) = {
        let mut k = kept;
        let mut fv = face_to_vidx;
        let diag = synthesise_planar_patches_diag(&mut k, &mut fv, &positions, tol);
        if std::env::var("KERF_STITCH_SYNTH_DEBUG").is_ok() {
            eprintln!(
                "stitch synth: one_sided={} loops={} added={} rejected={}",
                diag.one_sided_directed_edges,
                diag.loops_found,
                diag.planar_patches_added,
                diag.loops_rejected
            );
        }
        (k, fv)
    };
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

/// Best-effort rescue for kept face sets with one-sided boundary edges.
///
/// Conservative behaviour: only act when the kept set already has zero
/// one-sided edges (no-op, fast path) OR when a single iteration of
/// "drop the face whose share of one-sided edges is highest" reduces the
/// count to zero. If after `MAX_PASSES` we still have one-sided edges,
/// abandon the rescue and return the original input — let stitch panic
/// so the bug remains visible.
///
/// The greedy "drop one face per pass" approach can NOT close legitimately
/// open boundaries (LBracket+Counterbore concave-corner case, 4-corner
/// fillet shared-face case) — dropping a face cascades to create new
/// one-sided edges on its partner faces and the fixpoint typically
/// collapses to the empty kept set. Both cases are tracked as deferred
/// work; they need a synthesise-patch-face approach, not face dropping.
fn drop_one_sided_boundary(
    kept: Vec<KeptFace>,
    face_to_vidx: Vec<Vec<usize>>,
) -> (Vec<KeptFace>, Vec<Vec<usize>>) {
    const MAX_PASSES: usize = 4;
    let initial_count = kept.len();
    if initial_count == 0 {
        return (kept, face_to_vidx);
    }

    let mut keep_mask: Vec<bool> = vec![true; initial_count];
    for _pass in 0..MAX_PASSES {
        // Build edge -> (forward_count, backward_count, contributing_faces).
        let mut edge_to_dirs: HashMap<(usize, usize), (usize, usize, Vec<usize>)> = HashMap::new();
        for (face_idx, vidx) in face_to_vidx.iter().enumerate() {
            if !keep_mask[face_idx] {
                continue;
            }
            let n = vidx.len();
            for i in 0..n {
                let v_start = vidx[i];
                let v_end = vidx[(i + 1) % n];
                if v_start == v_end {
                    continue;
                }
                let key = canonical_edge_key(v_start, v_end);
                let forward = (v_start, v_end) == key;
                let entry = edge_to_dirs.entry(key).or_insert((0, 0, Vec::new()));
                if forward {
                    entry.0 += 1;
                } else {
                    entry.1 += 1;
                }
                if !entry.2.contains(&face_idx) {
                    entry.2.push(face_idx);
                }
            }
        }

        let mut one_sided_count: Vec<usize> = vec![0; initial_count];
        let mut any = false;
        for (_key, (fwd, bwd, faces)) in &edge_to_dirs {
            let one_sided = (*fwd == 0 && *bwd >= 1) || (*bwd == 0 && *fwd >= 1);
            if !one_sided {
                continue;
            }
            any = true;
            for &f in faces {
                one_sided_count[f] += 1;
            }
        }
        if !any {
            // Success: kept set has no one-sided edges. Apply the mask.
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
            return (new_kept, new_face_to_vidx);
        }

        let mut victim: Option<usize> = None;
        let mut victim_count: usize = 0;
        for (i, &c) in one_sided_count.iter().enumerate() {
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

    // Did not converge within MAX_PASSES. Return original input untouched
    // so stitch's panic remains visible.
    (kept, face_to_vidx)
}

/// Outcome of a synthesise pass — returned for testing visibility, even
/// though the public `stitch` API just folds the synthesised faces into the
/// kept list.
#[derive(Debug, Default, Clone, PartialEq)]
pub struct SynthResult {
    /// How many one-sided directed half-edges were identified at entry.
    pub one_sided_directed_edges: usize,
    /// How many closed loops the one-sided edges decomposed into.
    pub loops_found: usize,
    /// How many loops were successfully closed with a planar patch face.
    pub planar_patches_added: usize,
    /// How many loops were rejected because they weren't planar within
    /// tolerance, weren't closeable into a single cycle, or had < 3
    /// distinct vertices.
    pub loops_rejected: usize,
}

/// Stage 1e core: walk unpaired (one-sided) directed half-edges into closed
/// loops and synthesise a planar patch face per loop. Loops that fit a single
/// plane get one patch; non-planar polyhedral loops (e.g. wrapping around
/// an L-shaped corner) are segmented into runs of consecutive coplanar
/// vertices and patched per run, with chord edges between adjacent runs
/// twinning each other automatically. Loops that fail both routes are left
/// alone — stage 5 will still panic for those, surfacing the bug.
///
/// `kept` and `face_to_vidx` are mutated in place: any synthesised faces
/// are appended, so vertex indices in `face_to_vidx` continue to refer
/// to entries in `positions`.
pub(super) fn synthesise_planar_patches_diag(
    kept: &mut Vec<KeptFace>,
    face_to_vidx: &mut Vec<Vec<usize>>,
    positions: &[Point3],
    tol: &Tolerance,
) -> SynthResult {
    let mut diag = SynthResult::default();
    if kept.is_empty() {
        return diag;
    }

    // Step 1: count directed half-edges. (v_start, v_end) -> count. A canonical
    // edge has 2 half-edges (one each direction); a one-sided directed edge
    // appears in (a, b) with no matching (b, a) entry. We work in DIRECTED
    // form here because the synthesised patch needs to twin specific
    // directions — its polygon goes b->a wherever the existing kept faces
    // contribute a->b.
    let mut directed: HashMap<(usize, usize), usize> = HashMap::new();
    for vidx in face_to_vidx.iter() {
        let n = vidx.len();
        for i in 0..n {
            let a = vidx[i];
            let b = vidx[(i + 1) % n];
            if a == b {
                continue;
            }
            *directed.entry((a, b)).or_insert(0) += 1;
        }
    }

    // Collect one-sided directed half-edges: those (a,b) with no (b,a) entry.
    // (We treat (a,b) with `directed[(a,b)] >= 1 && directed[(b,a)] == 0` as
    // one-sided. If both directions exist, twin pairing handles them in
    // stage 5; if either side has count >= 2 the orphan-contributor pass
    // already pruned them.)
    let mut one_sided: Vec<(usize, usize)> = Vec::new();
    for (&(a, b), &count) in &directed {
        if count >= 1 && !directed.contains_key(&(b, a)) {
            // Each occurrence is one one-sided half-edge. We only walk one
            // representative per directed key — multiplicity > 1 means
            // duplicate kept faces share the same directed edge, which the
            // earlier passes should have collapsed; we err on the side of
            // a single patch and skip the duplicates.
            one_sided.push((a, b));
        }
    }
    diag.one_sided_directed_edges = one_sided.len();
    if one_sided.is_empty() {
        return diag;
    }

    // Step 2: walk one-sided directed edges into closed loops.
    //
    // For a manifold patch, every one-sided edge ending at vertex `b` has
    // exactly one one-sided edge starting at `b`. If multiple successors
    // exist (branching), the loop is ambiguous; we abort that walk.
    let mut successors: HashMap<usize, Vec<(usize, usize)>> = HashMap::new();
    for &(a, b) in &one_sided {
        successors.entry(a).or_default().push((a, b));
    }

    let mut visited: std::collections::HashSet<(usize, usize)> = std::collections::HashSet::new();
    let mut loops: Vec<Vec<usize>> = Vec::new();

    for &start_edge in &one_sided {
        if visited.contains(&start_edge) {
            continue;
        }
        let (start_a, _) = start_edge;
        let mut chain: Vec<usize> = vec![start_a]; // vertex indices in order
        let mut cur = start_edge;
        let mut ok = true;
        let mut local_visited: Vec<(usize, usize)> = Vec::new();
        loop {
            if local_visited.contains(&cur) {
                // Cycle closed (or we hit a previously-visited node in this
                // walk that isn't the start — treat as closure failure).
                break;
            }
            local_visited.push(cur);
            chain.push(cur.1);
            // Successor: a one-sided edge starting at cur.1.
            let next_candidates = successors.get(&cur.1).cloned().unwrap_or_default();
            // Filter out the reversed (b, a) — we already know it's not in
            // the directed map. But avoid revisiting an already-walked edge.
            let unwalked: Vec<(usize, usize)> = next_candidates
                .into_iter()
                .filter(|e| !local_visited.contains(e))
                .collect();
            if unwalked.is_empty() {
                // Did we get back to the start? chain[last] == start_a means
                // we just walked an edge ending at the loop's start vertex.
                if cur.1 == start_a {
                    // Loop closes. chain currently is [start_a, ..., start_a];
                    // drop the trailing duplicate to make it a clean cycle.
                    chain.pop();
                    break;
                }
                ok = false;
                break;
            }
            if unwalked.len() > 1 {
                // Branching: ambiguous loop. Skip.
                ok = false;
                break;
            }
            cur = unwalked[0];
        }
        if ok && chain.len() >= 3 {
            // Mark all walked edges as visited globally so we don't restart
            // the same loop from a different entry point.
            for e in &local_visited {
                visited.insert(*e);
            }
            loops.push(chain);
        } else {
            // Even on failure, mark the visited edges so we don't infinite-
            // loop. This means a malformed walk consumes its edges; their
            // gap will surface as a stitch panic, which is acceptable —
            // we documented this as the visible-bug fallback.
            for e in &local_visited {
                visited.insert(*e);
            }
        }
    }
    diag.loops_found = loops.len();

    // Step 3: for each loop, try to synthesise patch faces.
    //
    // First attempt: fit a single plane (Newell's method) through the entire
    // loop. If the loop is planar within tolerance, emit one patch face whose
    // polygon is the reverse of the walk so its half-edges twin the
    // existing one-sided ones.
    //
    // Second attempt: if the loop isn't planar, segment it into runs of
    // consecutive edges that share a common plane and emit one patch per
    // run, joined by chord half-edges. The chords are by construction
    // mutual twins (sub-patch i's closing chord goes p_end → p_start, and
    // sub-patch i+1's opening edge enters at p_start; together with another
    // chord from sub-patch back, we get a closed cycle).
    //
    // Specifically: a non-planar polyhedral loop wrapping around an inner
    // L-corner (LBracket+Counterbore) is two planar quads meeting at a
    // shared chord edge. Segmenting and closing recovers the missing
    // bracket faces.
    let plan_tol = (tol.point_eq * 1.0e6).max(1e-6);
    for loop_vidx in loops {
        let n = loop_vidx.len();
        let pts: Vec<Point3> = loop_vidx.iter().map(|&i| positions[i]).collect();

        if try_emit_single_planar(
            &loop_vidx,
            &pts,
            plan_tol,
            tol,
            kept,
            face_to_vidx,
        ) {
            diag.planar_patches_added += 1;
            continue;
        }

        // Single plane failed. Try a 2-way split: brute-force every
        // pair (i, j) and check whether splitting the loop there gives
        // two planar sub-quads. The sub-quads share an implicit chord
        // (v_i ↔ v_j) which becomes mutual twin half-edges in the two
        // patches, so no new vertex is introduced.
        //
        // This is the polyhedral L-corner case (LBracket+Counterbore):
        // a 6-vertex loop wrapping around a 90° interior step splits
        // into two 4-vertex quads on the bracket's two interior planes,
        // joined by a chord on the corner edge.
        //
        // We restrict to MIN_SEGMENT_VERTICES=4 per sub-quad (rules out
        // accidental triangulation of truly non-planar loops) and
        // total loop size up to MAX_LOOP_FOR_SPLIT (the search is O(n^2)
        // but n is typically tiny — booleans on prismatic shapes don't
        // produce huge unpaired loops).
        const MAX_LOOP_FOR_SPLIT: usize = 32;
        const MIN_SEG_VERTICES: usize = 4;
        let mut split_segments: Option<Vec<Vec<usize>>> = None;
        if n <= MAX_LOOP_FOR_SPLIT && n >= 2 * MIN_SEG_VERTICES - 2 {
            // For each (i, j) where 0 <= i < n and j = i + len_a - 1 (mod n)
            // and len_a in [MIN, n+2-MIN]:
            //   sub-loop A is the cyclic slice [v_i, v_{i+1}, ..., v_j];
            //   sub-loop B is the cyclic slice [v_j, ..., v_i].
            //   They share endpoints v_i and v_j (the implicit chord).
            //   len_a + len_b = n + 2.
            //
            // Enumerate i in 0..n and len_a in MIN..=(n+2-MIN); test each.
            'split_search: for i in 0..n {
                for len_a in MIN_SEG_VERTICES..=(n + 2 - MIN_SEG_VERTICES) {
                    let len_b = n + 2 - len_a;
                    if len_b < MIN_SEG_VERTICES {
                        continue;
                    }
                    // Sub-loop A: starts at i, len_a vertices.
                    let mut a: Vec<usize> = Vec::with_capacity(len_a);
                    for k in 0..len_a {
                        a.push((i + k) % n);
                    }
                    // Sub-loop B: starts at the last vertex of A (i + len_a - 1),
                    // walks backward: actually walks forward through vertices
                    // (j, j+1, ..., j + len_b - 1) where j = i + len_a - 1.
                    // Note: B's last vertex is i (closing the cycle).
                    let j = (i + len_a - 1) % n;
                    let mut b: Vec<usize> = Vec::with_capacity(len_b);
                    for k in 0..len_b {
                        b.push((j + k) % n);
                    }
                    let a_pts: Vec<Point3> = a.iter().map(|&k| pts[k]).collect();
                    let b_pts: Vec<Point3> = b.iter().map(|&k| pts[k]).collect();
                    if is_planar(&a_pts, plan_tol) && is_planar(&b_pts, plan_tol) {
                        split_segments = Some(vec![a, b]);
                        break 'split_search;
                    }
                }
            }
        }

        let segments = match split_segments {
            Some(s) => s,
            None => {
                if std::env::var("KERF_STITCH_SYNTH_DEBUG").is_ok() {
                    eprintln!(
                        "stitch synth: rejected non-planar loop, n={n} (no 2-way planar split)",
                    );
                    for p in &pts {
                        eprintln!("  ({:.4}, {:.4}, {:.4})", p.x, p.y, p.z);
                    }
                }
                diag.loops_rejected += 1;
                continue;
            }
        };

        // Emit one patch per segment. Each segment is a sub-loop: walk
        // through `seg` then back along the chord (last_vertex → first_vertex
        // in the sub-loop). The chord is reused between adjacent segments —
        // sub-loop A closes with chord (last_A → first_A) and sub-loop B
        // opens at first_A which equals last_A in the previous loop's
        // boundary (overlap by 1 vertex). The chord half-edges in adjacent
        // patches twin each other.
        let mut emitted_any = false;
        for seg in segments {
            // seg is a list of indices INTO `loop_vidx`. The vertices are
            // already in walk order. To twin the originals, reverse.
            let seg_vidx: Vec<usize> = seg.iter().map(|&i| loop_vidx[i]).collect();
            let seg_pts: Vec<Point3> = seg.iter().map(|&i| pts[i]).collect();
            let Some((centroid, normal)) = fit_plane_newell(&seg_pts) else {
                continue;
            };
            let Some(frame) = build_frame(centroid, normal, &seg_pts, tol) else {
                continue;
            };
            let plane = Plane::new(frame);
            let surface = SurfaceKind::Plane(plane);

            // Reverse to twin originals.
            let mut reversed: Vec<usize> = seg_vidx.iter().rev().copied().collect();
            while reversed.len() >= 2 && reversed.first() == reversed.last() {
                reversed.pop();
            }
            if reversed.len() < 3 {
                continue;
            }
            let polygon: Vec<Point3> = reversed.iter().map(|&i| positions[i]).collect();
            kept.push(KeptFace {
                polygon,
                surface,
                owner: None,
            });
            face_to_vidx.push(reversed);
            emitted_any = true;
        }
        if emitted_any {
            diag.planar_patches_added += 1;
        } else {
            diag.loops_rejected += 1;
        }
    }

    diag
}

fn is_planar(pts: &[Point3], tol: f64) -> bool {
    if pts.len() < 3 {
        return true;
    }
    let Some((centroid, normal)) = fit_plane_newell(pts) else {
        return false;
    };
    for p in pts {
        if (p - centroid).dot(&normal).abs() > tol {
            return false;
        }
    }
    true
}

/// Build an orthonormal frame anchored at `centroid` with z = normal,
/// picking some non-degenerate in-plane direction for x. None if all
/// points are too close to the centroid (degenerate).
fn build_frame(centroid: Point3, normal: Vec3, pts: &[Point3], tol: &Tolerance) -> Option<Frame> {
    let mut x_hint: Option<Vec3> = None;
    for p in pts {
        let d = p - centroid;
        if d.norm() > tol.point_eq * 10.0 {
            x_hint = Some(d);
            break;
        }
    }
    let x_hint = x_hint?;
    let x_in_plane = x_hint - x_hint.dot(&normal) * normal;
    let x = x_in_plane.try_normalize(0.0)?;
    let y = normal.cross(&x);
    Some(Frame {
        origin: centroid,
        x,
        y,
        z: normal,
    })
}

/// Try to emit a single planar patch for the loop. Returns true on success.
/// `loop_vidx` is the loop's vertex indices in walk order; `pts` is the
/// corresponding world-coordinate points (same length as `loop_vidx`).
fn try_emit_single_planar(
    loop_vidx: &[usize],
    pts: &[Point3],
    plan_tol: f64,
    tol: &Tolerance,
    kept: &mut Vec<KeptFace>,
    face_to_vidx: &mut Vec<Vec<usize>>,
) -> bool {
    let Some((centroid, normal)) = fit_plane_newell(pts) else {
        return false;
    };
    if !is_planar(pts, plan_tol) {
        return false;
    }
    let Some(frame) = build_frame(centroid, normal, pts, tol) else {
        return false;
    };
    let plane = Plane::new(frame);
    let surface = SurfaceKind::Plane(plane);
    let mut reversed_vidx: Vec<usize> = loop_vidx.iter().rev().copied().collect();
    let mut reversed_pts: Vec<Point3> = pts.iter().rev().copied().collect();
    while reversed_vidx.len() >= 2 && reversed_vidx.first() == reversed_vidx.last() {
        reversed_vidx.pop();
        reversed_pts.pop();
    }
    if reversed_vidx.len() < 3 {
        return false;
    }
    kept.push(KeptFace {
        polygon: reversed_pts,
        surface,
        owner: None,
    });
    face_to_vidx.push(reversed_vidx);
    true
}

/// Newell's method for the unit normal of a (possibly non-convex) planar
/// polygon. Returns (centroid, unit_normal) or None for degenerate inputs.
/// Robust against arbitrary winding direction; the returned normal is
/// determined by the polygon's traversal orientation.
fn fit_plane_newell(pts: &[Point3]) -> Option<(Point3, Vec3)> {
    let n = pts.len();
    if n < 3 {
        return None;
    }
    let mut nx = 0.0;
    let mut ny = 0.0;
    let mut nz = 0.0;
    let mut cx = 0.0;
    let mut cy = 0.0;
    let mut cz = 0.0;
    for i in 0..n {
        let a = pts[i];
        let b = pts[(i + 1) % n];
        nx += (a.y - b.y) * (a.z + b.z);
        ny += (a.z - b.z) * (a.x + b.x);
        nz += (a.x - b.x) * (a.y + b.y);
        cx += a.x;
        cy += a.y;
        cz += a.z;
    }
    let normal = Vec3::new(nx, ny, nz);
    let normal = normal.try_normalize(0.0)?;
    let centroid = Point3::new(cx / n as f64, cy / n as f64, cz / n as f64);
    Some((centroid, normal))
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
            kept.push(KeptFace { polygon, surface, owner: None });
        }
        let new_s = stitch(&kept, &Tolerance::default());
        assert_eq!(new_s.vertex_count(), 8);
        assert_eq!(new_s.edge_count(), 12);
        assert_eq!(new_s.face_count(), 6);
        validate(&new_s.topo).unwrap();
    }

    /// Helper: construct a single triangular kept face from three points.
    /// Carries a placeholder Plane(world) — the surface geometry doesn't
    /// matter for synthesise_planar_patches_diag tests, which only exercise
    /// the directed-edge → loop → patch logic.
    fn tri_face(a: Point3, b: Point3, c: Point3) -> KeptFace {
        KeptFace {
            polygon: vec![a, b, c],
            surface: SurfaceKind::Plane(Plane::new(kerf_geom::Frame::world(Point3::origin()))),
            owner: None,
        }
    }

    /// Helper: build (kept, face_to_vidx, positions) from a list of
    /// KeptFaces, deduping vertices by position. Mirrors stage 1's
    /// `find_or_add` so the same vertex indices appear in multiple faces.
    fn build_indices(kept: &[KeptFace]) -> (Vec<Vec<usize>>, Vec<Point3>) {
        let mut positions: Vec<Point3> = Vec::new();
        let mut face_to_vidx: Vec<Vec<usize>> = Vec::with_capacity(kept.len());
        let tol = Tolerance::default();
        for kf in kept {
            let mut indices = Vec::with_capacity(kf.polygon.len());
            for p in &kf.polygon {
                let mut found = None;
                for (i, q) in positions.iter().enumerate() {
                    if (p - q).norm() < tol.point_eq {
                        found = Some(i);
                        break;
                    }
                }
                let idx = match found {
                    Some(i) => i,
                    None => {
                        positions.push(*p);
                        positions.len() - 1
                    }
                };
                indices.push(idx);
            }
            face_to_vidx.push(indices);
        }
        (face_to_vidx, positions)
    }

    #[test]
    fn synth_no_op_when_already_2_manifold() {
        // Round-tripping a closed cube means every directed half-edge has
        // its reverse partner. The synthesise pass should add zero patches
        // and report zero one-sided edges.
        let s = box_(Vec3::new(1.0, 1.0, 1.0));
        let mut kept = Vec::new();
        for face_id in s.topo.face_ids() {
            let polygon = face_polygon(&s, face_id).unwrap();
            let surface = s.face_geom.get(face_id).cloned().unwrap();
            kept.push(KeptFace { polygon, surface, owner: None });
        }
        let (mut face_to_vidx, positions) = build_indices(&kept);
        let mut kept_owned = kept.clone();
        let diag = synthesise_planar_patches_diag(
            &mut kept_owned,
            &mut face_to_vidx,
            &positions,
            &Tolerance::default(),
        );
        assert_eq!(diag.one_sided_directed_edges, 0);
        assert_eq!(diag.loops_found, 0);
        assert_eq!(diag.planar_patches_added, 0);
        assert_eq!(diag.loops_rejected, 0);
        assert_eq!(kept_owned.len(), 6, "no patches added on closed cube");
    }

    #[test]
    fn synth_planar_patch_closes_open_box_top() {
        // A box minus its top face: the bottom + 4 walls form a half-open
        // shell. The 4 top vertices form a square loop of one-sided edges
        // (one per wall, all going around the top in the same direction).
        // Stage 1e should fit a plane through those 4 points and add a
        // single quad patch — restoring the cube's manifold-ness.
        let s = box_(Vec3::new(1.0, 1.0, 1.0));
        let mut kept = Vec::new();
        // The cube primitive's faces are in arbitrary order. Skip the face
        // whose centroid has the largest z (the top).
        let mut face_geom: Vec<(FaceId, Point3, Vec<Point3>, SurfaceKind)> = s
            .topo
            .face_ids()
            .map(|fid| {
                let poly = face_polygon(&s, fid).unwrap();
                let centroid: Point3 = {
                    let mut sx = 0.0;
                    let mut sy = 0.0;
                    let mut sz = 0.0;
                    for p in &poly {
                        sx += p.x;
                        sy += p.y;
                        sz += p.z;
                    }
                    let n = poly.len() as f64;
                    Point3::new(sx / n, sy / n, sz / n)
                };
                let surface = s.face_geom.get(fid).cloned().unwrap();
                (fid, centroid, poly, surface)
            })
            .collect();
        face_geom.sort_by(|a, b| b.1.z.partial_cmp(&a.1.z).unwrap());
        // First face is the top — drop it.
        for (_fid, _c, poly, surface) in face_geom.iter().skip(1) {
            kept.push(KeptFace {
                polygon: poly.clone(),
                surface: surface.clone(),
                owner: None,
            });
        }
        assert_eq!(kept.len(), 5);
        let (mut face_to_vidx, positions) = build_indices(&kept);
        let mut kept_owned = kept.clone();
        let diag = synthesise_planar_patches_diag(
            &mut kept_owned,
            &mut face_to_vidx,
            &positions,
            &Tolerance::default(),
        );
        assert_eq!(diag.one_sided_directed_edges, 4, "open top has 4 one-sided edges");
        assert_eq!(diag.loops_found, 1);
        assert_eq!(diag.planar_patches_added, 1);
        assert_eq!(diag.loops_rejected, 0);
        assert_eq!(kept_owned.len(), 6, "patch closes the box");

        // The patch's polygon should have 4 vertices, all at z = 1.0
        // (or whatever the top face was) — the box primitive uses extents
        // half-extents, so check a planar polygon.
        let patch = kept_owned.last().unwrap();
        assert_eq!(patch.polygon.len(), 4);
        let z0 = patch.polygon[0].z;
        for p in &patch.polygon {
            assert!((p.z - z0).abs() < 1e-9, "patch should be planar in z");
        }
    }

    #[test]
    fn synth_full_stitch_open_box_round_trips() {
        // End-to-end: feed an open-top cube into stitch() itself. The 1e
        // pass should synthesise the missing top, then stage 5 should
        // succeed without panic, yielding a valid 8V/12E/6F solid.
        let s = box_(Vec3::new(1.0, 1.0, 1.0));
        let mut kept = Vec::new();
        let mut face_geom: Vec<(FaceId, Point3, Vec<Point3>, SurfaceKind)> = s
            .topo
            .face_ids()
            .map(|fid| {
                let poly = face_polygon(&s, fid).unwrap();
                let centroid: Point3 = {
                    let mut sx = 0.0;
                    let mut sy = 0.0;
                    let mut sz = 0.0;
                    for p in &poly {
                        sx += p.x;
                        sy += p.y;
                        sz += p.z;
                    }
                    let n = poly.len() as f64;
                    Point3::new(sx / n, sy / n, sz / n)
                };
                let surface = s.face_geom.get(fid).cloned().unwrap();
                (fid, centroid, poly, surface)
            })
            .collect();
        face_geom.sort_by(|a, b| b.1.z.partial_cmp(&a.1.z).unwrap());
        for (_fid, _c, poly, surface) in face_geom.iter().skip(1) {
            kept.push(KeptFace {
                polygon: poly.clone(),
                surface: surface.clone(),
                owner: None,
            });
        }
        let new_s = stitch(&kept, &Tolerance::default());
        assert_eq!(new_s.vertex_count(), 8);
        assert_eq!(new_s.edge_count(), 12);
        assert_eq!(new_s.face_count(), 6, "synthesised patch makes it 6 faces");
        validate(&new_s.topo).unwrap();
    }

    #[test]
    fn synth_rejects_non_planar_loop() {
        // Build a deliberately twisted, non-planar loop where no contiguous
        // run of vertices is coplanar — segmentation can't recover it. Six
        // tetrahedral side faces share an apex; the base loop has every
        // other vertex offset in z (a saddle), so no 3 consecutive vertices
        // share a plane.
        let mut base: Vec<Point3> = Vec::new();
        for k in 0..6 {
            let theta = (k as f64) * std::f64::consts::FRAC_PI_3;
            // Alternate z so no 3 consecutive points are coplanar.
            let z = if k % 2 == 0 { 0.0 } else { 0.7 };
            base.push(Point3::new(theta.cos(), theta.sin(), z));
        }
        let apex = Point3::new(0.0, 0.0, 2.0);
        let mut kept = Vec::new();
        for k in 0..6 {
            kept.push(tri_face(apex, base[k], base[(k + 1) % 6]));
        }
        let (mut face_to_vidx, positions) = build_indices(&kept);
        let mut kept_owned = kept.clone();
        let diag = synthesise_planar_patches_diag(
            &mut kept_owned,
            &mut face_to_vidx,
            &positions,
            &Tolerance::default(),
        );
        assert_eq!(diag.one_sided_directed_edges, 6);
        assert_eq!(diag.loops_found, 1);
        assert_eq!(
            diag.planar_patches_added, 0,
            "saddle loop with no coplanar runs must NOT be patched"
        );
        assert_eq!(diag.loops_rejected, 1);
        assert_eq!(kept_owned.len(), 6, "no synthesised patch added");
    }

    #[test]
    fn synth_handles_two_disjoint_loops() {
        // Two cubes both with their top removed — two independent open
        // loops. Stage 1e should walk them separately and add 2 patches.
        let s1 = box_(Vec3::new(1.0, 1.0, 1.0));
        // Translate the second box far in x so positions don't collide.
        let mut kept = Vec::new();
        let mut face_geom: Vec<(FaceId, Point3, Vec<Point3>, SurfaceKind)> = s1
            .topo
            .face_ids()
            .map(|fid| {
                let poly = face_polygon(&s1, fid).unwrap();
                let centroid: Point3 = {
                    let mut sx = 0.0;
                    let mut sy = 0.0;
                    let mut sz = 0.0;
                    for p in &poly {
                        sx += p.x;
                        sy += p.y;
                        sz += p.z;
                    }
                    let n = poly.len() as f64;
                    Point3::new(sx / n, sy / n, sz / n)
                };
                let surface = s1.face_geom.get(fid).cloned().unwrap();
                (fid, centroid, poly, surface)
            })
            .collect();
        face_geom.sort_by(|a, b| b.1.z.partial_cmp(&a.1.z).unwrap());
        for (_fid, _c, poly, surface) in face_geom.iter().skip(1) {
            kept.push(KeptFace {
                polygon: poly.clone(),
                surface: surface.clone(),
                owner: None,
            });
        }
        // Second open box, shifted 10 in x.
        let shift = Vec3::new(10.0, 0.0, 0.0);
        for (_fid, _c, poly, surface) in face_geom.iter().skip(1) {
            let shifted: Vec<Point3> = poly.iter().map(|p| p + shift).collect();
            kept.push(KeptFace {
                polygon: shifted,
                surface: surface.clone(),
                owner: None,
            });
        }
        let (mut face_to_vidx, positions) = build_indices(&kept);
        let mut kept_owned = kept.clone();
        let diag = synthesise_planar_patches_diag(
            &mut kept_owned,
            &mut face_to_vidx,
            &positions,
            &Tolerance::default(),
        );
        assert_eq!(diag.one_sided_directed_edges, 8, "4 + 4 one-sided edges");
        assert_eq!(diag.loops_found, 2, "two disjoint loops");
        assert_eq!(diag.planar_patches_added, 2);
        assert_eq!(kept_owned.len(), 12, "10 walls + 2 patches");
    }
}
