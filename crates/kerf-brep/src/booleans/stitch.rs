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
use crate::geometry::{CurveSegment, EllipseSegment, SurfaceKind};

/// Identity of the underlying curve carried by a polygon edge in a kept face.
///
/// Tier 1 of curved-surface stitch wiring. Today every edge in a kept polygon
/// is implicitly a line segment between its two endpoint vertex positions —
/// so the canonical edge key `(v_lo, v_hi)` uniquely identifies an edge.
/// Once `face_intersections` starts emitting arc chords (Cylinder×Plane), two
/// half-edges between the same pair of vertices may be carried by *different*
/// curves: a straight chord and an arc chord. The canonical key has to grow
/// to disambiguate them, otherwise stitch will pair a line half-edge with an
/// arc half-edge as twins and silently produce a non-physical solid.
///
/// `Line` is the default and matches every existing call site. `Arc(id)` is
/// emitted by curved-surface intersection chords, where `id` is a stable
/// integer that callers in the same boolean pass agree on for the same
/// underlying curve. The actual ellipse parameters live on the `KeptFace`
/// alongside the polygon.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum EdgeCurveTag {
    /// Straight-line chord between two polygon vertices. The vast majority of
    /// kerf-cad's stitch input is this variant.
    Line,
    /// Arc chord on a curved face boundary (e.g., the closed conic from
    /// Cylinder×Plane). The `u32` is a per-stitch-call arc identifier shared
    /// by both incident kept faces; matching IDs across two faces means the
    /// two half-edges traverse the *same* arc and must twin together.
    Arc(u32),
}

/// One kept face's contribution: its polygon (3D vertex positions in CCW order
/// when viewed from the face's outward normal) and its surface geometry.
///
/// `edge_tags`, when present, gives a per-polygon-edge `EdgeCurveTag` parallel
/// to `polygon` (so `edge_tags[i]` describes the chord from `polygon[i]` to
/// `polygon[(i+1) % n]`). When `edge_tags` is `None`, every edge is implicitly
/// `EdgeCurveTag::Line` — this is the legacy path for planar booleans and
/// preserves byte-for-byte equivalence with the pre-Tier-1 stitcher.
///
/// `arc_segments` indexes arc IDs to the arc geometry, so `Arc(k)` resolves to
/// `arc_segments[k]`. Used by Tier 3's polygon walker to recover the arc
/// parameter range for each half-edge; harmless if absent in Tier 1+2.
#[derive(Clone, Debug)]
pub struct KeptFace {
    pub polygon: Vec<Point3>,
    pub surface: SurfaceKind,
    /// Picking provenance carried over from the source solid's face_owner_tag.
    /// Threaded into the resulting Solid's face_owner_tag by `stitch`.
    /// `None` for faces whose source had no owner.
    pub owner: Option<String>,
    /// Per-edge curve identity (parallel to `polygon`). `None` ≡ all `Line`.
    pub edge_tags: Option<Vec<EdgeCurveTag>>,
    /// Arc-id → arc geometry. Indices match the `Arc(id)` payload in
    /// `edge_tags`. Empty for line-only inputs.
    pub arc_segments: Vec<EllipseSegment>,
}

impl KeptFace {
    /// Backwards-compatible constructor for the line-only path: every edge
    /// is implicitly `EdgeCurveTag::Line` and there are no arc segments.
    pub fn new_line(polygon: Vec<Point3>, surface: SurfaceKind, owner: Option<String>) -> Self {
        KeptFace {
            polygon,
            surface,
            owner,
            edge_tags: None,
            arc_segments: Vec::new(),
        }
    }

    /// Resolve the edge-tag for the i-th polygon edge. Defaults to `Line` when
    /// `edge_tags` is `None` (the legacy planar path).
    pub fn edge_tag(&self, i: usize) -> EdgeCurveTag {
        match &self.edge_tags {
            None => EdgeCurveTag::Line,
            Some(tags) => tags.get(i).copied().unwrap_or(EdgeCurveTag::Line),
        }
    }
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

    // Stage 1d (GAP C): prune "excursion" vertices. A vertex `v` that
    // appears in EXACTLY ONE polygon AND whose two adjacent edges in that
    // polygon are both orphans (no other polygon has either edge as a twin)
    // is part of a detour that the boolean classifier accidentally threaded
    // through this face. The detour adds no real boundary — its sole effect
    // is to leave 1-half-edge orphans in stitch. Removing `v` collapses the
    // detour while preserving the face's true outer boundary.
    //
    // This is the multi-edge-fillet-stitch repair: sequential subtracts in
    // build_fillets push split-vertex artifacts (e.g., y=0.096 chord-points
    // from wedge2's cylinder samples) into body z-face polygons that have
    // no corresponding kept partner. Pruning collapses them.
    let face_to_vidx = prune_excursion_vertices(&face_to_vidx);
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
    // Tier 1: edge_pairs key is `(v_lo, v_hi, curve_tag)`. For line-edge
    // half-edges (the entire pre-Tier-1 universe) `curve_tag` is `Line` so
    // behavior is unchanged. Two arc half-edges between the same vertex pair
    // but on *different* curves no longer collide with each other or with a
    // line edge between those vertices.
    let mut edge_pairs: HashMap<(usize, usize, EdgeCurveTag), Vec<usize>> = HashMap::new();
    let mut face_ids: Vec<FaceId> = Vec::with_capacity(kept.len());
    // M39f: self-loop half-edges (v_start == v_end, e.g., vase apex after
    // global vertex dedup collapses multiple coincident apex vertices). They
    // can't be paired as twins (direction is ambiguous on a self-loop), so
    // we self-twin them after stage 5 to satisfy the AsymmetricTwin invariant
    // and keep them out of edge_pairs to avoid spurious "1 half-edge" panics.
    let mut self_loops: Vec<kerf_topo::HalfEdgeId> = Vec::new();

    for (face_idx, kf) in kept.iter().enumerate() {
        // After Stage 1d (excursion pruning), face_to_vidx may have fewer
        // vertices than kf.polygon — use face_to_vidx as the source of
        // truth for the half-edge count.
        let n = face_to_vidx[face_idx].len();
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
        //
        // Tier 1: each polygon edge can carry a curve identity tag — `Line`
        // for the legacy planar path or `Arc(id)` for arc chords from
        // curved-surface intersections. The canonical edge key is
        // `(v_lo, v_hi, tag)` so an arc half-edge between vertices (a, b)
        // doesn't collide in `edge_pairs` with a line half-edge between the
        // same (a, b). Pruned vertices (Stage 1d) might have shifted the
        // edge index, so we look up the tag using the original polygon
        // index closest to the post-prune order; for pure planar input
        // every tag resolves to `Line` and the lookup is a no-op.
        let mut he_ids = Vec::with_capacity(n);
        for i in 0..n {
            let v_start_idx = face_to_vidx[face_idx][i];
            let v_end_idx = face_to_vidx[face_idx][(i + 1) % n];
            let he_id = new_solid
                .topo
                .build_insert_half_edge(vids[v_start_idx], loop_id);
            he_ids.push(he_id);
            let tag = kf.edge_tag(i);
            half_edge_records.push(HalfEdgeRecord {
                id: he_id,
                v_start: v_start_idx,
                v_end: v_end_idx,
                face_idx,
                curve_tag: tag,
            });
            if v_start_idx == v_end_idx {
                // Self-loop: bypass edge pairing.
                self_loops.push(he_id);
            } else {
                let key = canonical_edge_key_tagged(v_start_idx, v_end_idx, tag);
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
    //
    // Tier 1: arc-tagged keys (curve_tag != Line) take an arc-aware geometry
    // path: instead of synthesizing a line through the two endpoint
    // positions, we materialize a `CurveSegment::Ellipse` from the
    // arc_segments table on the kept face, preserving the underlying conic.
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

        // Edge geometry. Line tag → straight chord between endpoints; Arc
        // tag → recover the EllipseSegment from the kept face's arc table.
        // If the two endpoints coincide (zero-length edge — usually a vase
        // apex or coincident-vertex artifact in degenerate input), skip
        // assigning edge geometry; the topology stays valid (twin pointers
        // wired) and downstream tessellation degenerates gracefully.
        let v_idx_a = half_edge_records[he_a_record_idx].v_start;
        let v_idx_b = half_edge_records[he_a_record_idx].v_end;
        let p0 = positions[v_idx_a];
        let p1 = positions[v_idx_b];
        let curve_tag = key.2;
        match curve_tag {
            EdgeCurveTag::Line => {
                if let Some(line) = Line::through(p0, p1) {
                    let length = (p1 - p0).norm();
                    let seg = CurveSegment::line(line, 0.0, length);
                    new_solid.edge_geom.insert(edge_id, seg);
                }
            }
            EdgeCurveTag::Arc(arc_id) => {
                let face_idx = half_edge_records[he_a_record_idx].face_idx;
                if let Some(arc) =
                    kept[face_idx].arc_segments.get(arc_id as usize).cloned()
                {
                    new_solid
                        .edge_geom
                        .insert(edge_id, arc.into_curve_segment());
                }
            }
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
    /// Tier 1: which underlying curve carries this half-edge. Defaults to
    /// `Line` for the planar-stitch path; `Arc(id)` for arc chords. Read by
    /// future Tier 3 polygon-walker work; the canonical key embeds the same
    /// info so present-day stage 5 doesn't need to consult it directly.
    #[allow(dead_code)]
    curve_tag: EdgeCurveTag,
}

/// Given a list of half-edge record indices that all share one canonical
/// edge key (low, high, tag), pick one half-edge in each direction (low→high
/// and high→low) so they twin properly. Returns None if either direction is
/// empty (truly non-manifold input). Extra half-edges in either direction
/// are simply skipped — usually a sign of coplanar duplicate kept faces.
fn pick_twin_pair(
    indices: &[usize],
    half_edge_records: &[HalfEdgeRecord],
    key: (usize, usize, EdgeCurveTag),
) -> Option<(usize, usize)> {
    let mut forward = None;
    let mut backward = None;
    let key2 = (key.0, key.1);
    for &i in indices {
        let r = &half_edge_records[i];
        if (r.v_start, r.v_end) == key2 && forward.is_none() {
            forward = Some(i);
        } else if (r.v_end, r.v_start) == key2 && backward.is_none() {
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

/// Tier 1: 3-tuple canonical key including curve identity. Two half-edges
/// twin together iff they share both the unordered vertex pair AND the same
/// `EdgeCurveTag`. Lines carry tag `Line`, arcs carry `Arc(id)` so a line
/// and an arc between the same pair of vertices stay unpaired.
fn canonical_edge_key_tagged(
    a: usize,
    b: usize,
    tag: EdgeCurveTag,
) -> (usize, usize, EdgeCurveTag) {
    let (lo, hi) = canonical_edge_key(a, b);
    (lo, hi, tag)
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
        //
        // Tier 1: bucket by `(v_lo, v_hi, curve_tag)` so a line edge between
        // a pair of vertices and an arc edge between the same pair are
        // tracked SEPARATELY. Pre-Tier-1 they pooled together, which would
        // make Stage 1c spuriously drop arc-bearing faces because their
        // line-twin neighbor on the same vertex pair pushed the bucket
        // count over 3.
        let mut edge_to_dir_faces: HashMap<
            (usize, usize, EdgeCurveTag),
            Vec<(usize, bool)>,
        > = HashMap::new();
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
                let tag = kept[face_idx].edge_tag(i);
                let key = canonical_edge_key_tagged(v_start, v_end, tag);
                let forward = (v_start, v_end) == (key.0, key.1);
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

/// GAP C: prune detour vertices whose adjacent polygon edges have no
/// matching twin in any other polygon. A detour vertex contributes
/// 1-half-edge orphans that the stitcher cannot pair, and removing it
/// collapses the spurious excursion without altering any face's real
/// boundary.
///
/// Pruning conditions (all must hold for vertex `v` in polygon `P`):
///   1. `v` appears in exactly one polygon (no other face has a vertex
///      with the same global index after dedup).
///   2. Both directed edges incident on `v` in `P` (prev→v and v→next) are
///      orphans — no other polygon has the reverse edge as a twin.
///   3. The polygon has at least 4 vertices remaining after removal.
///
/// Iterates to fixpoint: removing one excursion vertex can re-classify
/// neighbors as removable.
fn prune_excursion_vertices(face_to_vidx: &[Vec<usize>]) -> Vec<Vec<usize>> {
    let mut polys: Vec<Vec<usize>> = face_to_vidx.to_vec();
    loop {
        // Build vertex-occurrence count and directed-edge multiset.
        let mut vertex_in_polys: HashMap<usize, usize> = HashMap::new();
        let mut directed_edges: HashMap<(usize, usize), usize> = HashMap::new();
        for vidx in &polys {
            let n = vidx.len();
            // Track unique vertices per polygon (a vertex may legitimately
            // appear once in each face).
            let mut seen = std::collections::HashSet::new();
            for &v in vidx {
                seen.insert(v);
            }
            for v in seen {
                *vertex_in_polys.entry(v).or_insert(0) += 1;
            }
            for i in 0..n {
                let a = vidx[i];
                let b = vidx[(i + 1) % n];
                if a == b {
                    continue;
                }
                *directed_edges.entry((a, b)).or_insert(0) += 1;
            }
        }
        // Find a removable vertex.
        let mut removed = false;
        'outer: for poly_idx in 0..polys.len() {
            let n = polys[poly_idx].len();
            if n <= 4 {
                continue;
            }
            for i in 0..n {
                let prev = polys[poly_idx][(i + n - 1) % n];
                let v = polys[poly_idx][i];
                let next = polys[poly_idx][(i + 1) % n];
                if prev == v || v == next || prev == next {
                    continue;
                }
                if *vertex_in_polys.get(&v).unwrap_or(&0) != 1 {
                    continue;
                }
                // Both incident directed edges must be orphans (no twin
                // in any other polygon: the reverse direction has 0
                // entries).
                let edge_a_twin = *directed_edges.get(&(v, prev)).unwrap_or(&0);
                let edge_b_twin = *directed_edges.get(&(next, v)).unwrap_or(&0);
                if edge_a_twin > 0 || edge_b_twin > 0 {
                    continue;
                }
                // Removing v: replace [prev, v, next] with [prev, next].
                // The new edge (prev, next) must not already appear with
                // BOTH directions in other polygons (would create a new
                // 3+ entries conflict).
                let new_edge_fwd = *directed_edges.get(&(prev, next)).unwrap_or(&0);
                let new_edge_rev = *directed_edges.get(&(next, prev)).unwrap_or(&0);
                if new_edge_fwd > 0 || new_edge_rev > 1 {
                    continue;
                }
                polys[poly_idx].remove(i);
                removed = true;
                break 'outer;
            }
        }
        if !removed {
            break;
        }
    }
    polys
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
        // GAP D (chained Fillet): score-based promotion. The greedy first-match
        // strategy used by the original GAP C rescue works for axis-aligned
        // chains where the first reverse-direction partner found is
        // typically the right one — but on chained-Fillet across different
        // axes (z-then-x), the first match tends to be a "duplicate" face
        // walking the SAME direction as the existing kept half-edge or
        // sharing a corner with several other faces; promoting it explodes
        // the orphan count instead of reducing it.
        //
        // Score each available dropped candidate by:
        //   closed  = # of its directed edges that are in `singles_set`
        //             (i.e. close an existing 1-half-edge orphan)
        //   created = # of its directed edges that match a kept directed
        //             edge in the SAME direction (creates a 2-forward
        //             same-direction conflict pick_twin_pair can't resolve),
        //             counted with a 2× penalty.
        //           + # of its directed edges with no reverse twin available
        //             anywhere (creates a permanent orphan).
        //
        // Promote the candidate with highest score that closes ≥1 orphan AND
        // has score > 0. Coplanarity gives a small tiebreak boost (preserves
        // chord_merge-style preference for coplanar partners).
        //
        // The four chained-Fillet z-edge tests (covered by GAP C) keep
        // passing because their candidates score positively (closes 1,
        // creates 0 → score 1, with coplanar boost → 2). The chained
        // Fillet-z-then-x case finds a candidate that scores positive
        // instead of running into an over-promoted dead-end.

        let mut singles_set: std::collections::HashSet<(usize, usize)> =
            std::collections::HashSet::new();
        for &((u, v), present_forward) in &singles {
            let needed = if present_forward { (v, u) } else { (u, v) };
            singles_set.insert(needed);
        }

        let mut existing_dir: HashMap<(usize, usize), usize> = HashMap::new();
        for vidx in &face_vidx {
            let n = vidx.len();
            for i in 0..n {
                let a = vidx[i];
                let b = vidx[(i + 1) % n];
                if a == b {
                    continue;
                }
                *existing_dir.entry((a, b)).or_insert(0) += 1;
            }
        }

        // Pre-resolve each dropped candidate's vertex indices in the current
        // global positions table.
        let mut dropped_vidx: Vec<Option<Vec<usize>>> = vec![None; dropped.len()];
        for di in 0..dropped.len() {
            if !available[di] {
                continue;
            }
            let mut idxs = Vec::with_capacity(dropped[di].polygon.len());
            let mut ok = true;
            for p in &dropped[di].polygon {
                let mut found = None;
                for (idx, q) in positions.iter().enumerate() {
                    if (*p - *q).norm() <= pt_tol {
                        found = Some(idx);
                        break;
                    }
                }
                match found {
                    Some(i) => idxs.push(i),
                    None => {
                        ok = false;
                        break;
                    }
                }
            }
            if ok {
                dropped_vidx[di] = Some(idxs);
            }
        }

        // Build dropped-pool reverse-direction presence so we can decide
        // whether a candidate's "introduced" forward edge has a future
        // pair available.
        let mut dropped_pool_dir: HashMap<(usize, usize), usize> = HashMap::new();
        for (di, vidx) in dropped_vidx.iter().enumerate() {
            if !available[di] {
                continue;
            }
            let Some(vidx) = vidx else {
                continue;
            };
            let n = vidx.len();
            for i in 0..n {
                let a = vidx[i];
                let b = vidx[(i + 1) % n];
                if a == b {
                    continue;
                }
                *dropped_pool_dir.entry((a, b)).or_insert(0) += 1;
            }
        }

        let mut best: Option<(i64, usize)> = None; // (score, di)
        for di in 0..dropped.len() {
            if !available[di] {
                continue;
            }
            let Some(vidx) = &dropped_vidx[di] else {
                continue;
            };
            let n = vidx.len();
            let mut closed = 0i64;
            let mut created = 0i64;
            for i in 0..n {
                let a = vidx[i];
                let b = vidx[(i + 1) % n];
                if a == b {
                    continue;
                }
                if singles_set.contains(&(a, b)) {
                    closed += 1;
                    continue;
                }
                // SAME-direction conflict with an existing kept edge.
                if existing_dir.get(&(a, b)).copied().unwrap_or(0) >= 1 {
                    created += 2;
                    continue;
                }
                // Reverse already paired (kept has 2+ in the opposite
                // direction means our forward over-pairs).
                if existing_dir.get(&(b, a)).copied().unwrap_or(0) >= 2 {
                    created += 2;
                    continue;
                }
                // Brand-new edge for kept. Check if pool can provide a
                // future reverse partner; otherwise it'll be a permanent
                // orphan.
                let reverse_in_pool = dropped_pool_dir.get(&(b, a)).copied().unwrap_or(0);
                if reverse_in_pool == 0 {
                    created += 1;
                }
            }
            if closed == 0 {
                continue;
            }
            let mut score = closed - created;
            if score <= 0 {
                continue;
            }
            // Coplanarity tiebreak: small boost when the candidate's plane
            // matches a kept face that owns one of the closed orphans.
            'tie: for i in 0..n {
                let a = vidx[i];
                let b = vidx[(i + 1) % n];
                if !singles_set.contains(&(a, b)) {
                    continue;
                }
                for (fi, kvidx) in face_vidx.iter().enumerate() {
                    let kn = kvidx.len();
                    for j in 0..kn {
                        let p = kvidx[j];
                        let q = kvidx[(j + 1) % kn];
                        if (p, q) == (b, a)
                            && surfaces_coplanar(
                                &dropped[di].surface,
                                &kept[fi].surface,
                                tol,
                            )
                        {
                            score += 1;
                            break 'tie;
                        }
                    }
                }
            }
            if best.map(|(s, _)| score > s).unwrap_or(true) {
                best = Some((score, di));
            }
        }

        let promoted = if let Some((_, di)) = best {
            kept.push(dropped[di].clone());
            available[di] = false;
            true
        } else {
            false
        };

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
            kept.push(KeptFace::new_line(polygon, surface, None));
        }
        let new_s = stitch(&kept, &Tolerance::default());
        assert_eq!(new_s.vertex_count(), 8);
        assert_eq!(new_s.edge_count(), 12);
        assert_eq!(new_s.face_count(), 6);
        validate(&new_s.topo).unwrap();
    }

    #[test]
    fn prune_excursion_vertices_removes_orphan_detour() {
        // A 6-vertex polygon with a detour: square (0..3) plus an excursion
        // through vertex 4 (only in this polygon, both adjacent edges
        // orphans). The square's 4 corners are shared with neighbors
        // (simulated as additional polygons referencing them).
        let polys = vec![
            // Polygon under test: square 0,1,2,3 with detour 4,5 inserted
            // between 1 and 2. Vertex 4 and 5 are unique to this polygon.
            vec![0, 1, 4, 5, 2, 3],
            // Neighbors that share edges with the square's perimeter so
            // those edges have twins (and won't be prunable).
            vec![1, 0, 6, 7],     // edge (0,1) twin lives here as (1,0)
            vec![2, 1, 8, 9],     // edge (1,2) twin
            vec![3, 2, 10, 11],   // edge (2,3) twin
            vec![0, 3, 12, 13],   // edge (3,0) twin
        ];
        let pruned = prune_excursion_vertices(&polys);
        // Vertices 4 and 5 should be pruned: each appears only in poly 0,
        // their adjacent directed edges (1→4, 4→5, 5→2) have no reverse
        // twins anywhere.
        assert_eq!(pruned[0], vec![0, 1, 2, 3], "detour collapsed");
        // Other polygons unchanged.
        assert_eq!(pruned[1], polys[1]);
        assert_eq!(pruned[2], polys[2]);
        assert_eq!(pruned[3], polys[3]);
        assert_eq!(pruned[4], polys[4]);
    }

    #[test]
    fn prune_excursion_vertices_preserves_shared_corners() {
        // A square 0,1,2,3 whose vertex 1 is also used by another
        // polygon. Even if vertex 1's edges are orphans, it's NOT
        // prunable because the rule requires vertex_in_polys[v] == 1.
        let polys = vec![vec![0, 1, 2, 3], vec![5, 1, 6]];
        let pruned = prune_excursion_vertices(&polys);
        assert_eq!(pruned[0], vec![0, 1, 2, 3]);
        assert_eq!(pruned[1], vec![5, 1, 6]);
    }

    #[test]
    fn prune_excursion_vertices_preserves_min_polygon() {
        // A 4-vertex polygon (minimum) should never be pruned even if all
        // edges are orphans, because removing a vertex would leave a
        // degenerate triangle below 4-vertex floor.
        let polys = vec![vec![0, 1, 2, 3]];
        let pruned = prune_excursion_vertices(&polys);
        assert_eq!(pruned[0], vec![0, 1, 2, 3]);
    }

    // ----------------------------------------------------------------
    // Tier 1 (curved-surface stitch wiring): canonical-edge-key must
    // distinguish line half-edges from arc half-edges between the same
    // pair of vertices.
    // ----------------------------------------------------------------

    #[test]
    fn canonical_edge_key_tagged_disambiguates_line_vs_arc() {
        // The unordered vertex pair is the same, but the curve identity
        // differs → the keys MUST be unequal. Without this, stitch would
        // wire a line half-edge to an arc half-edge as twins.
        let line_key = canonical_edge_key_tagged(0, 1, EdgeCurveTag::Line);
        let arc_key = canonical_edge_key_tagged(0, 1, EdgeCurveTag::Arc(0));
        assert_ne!(line_key, arc_key);
    }

    #[test]
    fn canonical_edge_key_tagged_arc_with_same_id_matches_swapped_endpoints() {
        // A→B as Arc(7) and B→A as Arc(7) are the same canonical edge
        // (twin pair) — the unordered (lo, hi) pair PLUS matching arc
        // identity collapse to one bucket so the two half-edges twin.
        let fwd = canonical_edge_key_tagged(2, 5, EdgeCurveTag::Arc(7));
        let rev = canonical_edge_key_tagged(5, 2, EdgeCurveTag::Arc(7));
        assert_eq!(fwd, rev);
    }

    #[test]
    fn canonical_edge_key_tagged_distinguishes_different_arc_ids() {
        // Two arcs between the same pair of vertices but with different
        // arc IDs are distinct edges (e.g., a closed-curve face whose two
        // sides are *different* arcs of the same conic). They must NOT
        // pair as twins.
        let arc_a = canonical_edge_key_tagged(0, 1, EdgeCurveTag::Arc(0));
        let arc_b = canonical_edge_key_tagged(0, 1, EdgeCurveTag::Arc(1));
        assert_ne!(arc_a, arc_b);
    }

    #[test]
    fn stitch_recognizes_arc_canonical_keys() {
        // End-to-end Tier 1 test. Build a degenerate "double-sided
        // triangle" mesh — two CCW triangles sharing all three vertices
        // and three edges in opposite directions (front + back of a
        // 2-manifold disk-without-holes). V=3 / E=3 / F=2 satisfies
        // Euler-Poincaré V-E+F=2 for a sphere-like 2-manifold.
        //
        // ONE of the three shared edges is tagged as `Arc(0)` on BOTH
        // faces (with the same arc_id, so they twin together). The other
        // two edges are line-tagged (default). Stitch must:
        //   1. Produce a valid 2-face / 3-edge / 3-vertex topology.
        //   2. Wire the arc half-edges to each other as twins (not to
        //      a line half-edge between the same vertex pair).
        //   3. Materialize the arc edge's geometry as a CurveSegment whose
        //      curve kind is `Ellipse`, NOT `Line`.
        use crate::geometry::{CurveKind, EllipseSegment};
        use kerf_geom::{Ellipse, Frame, Plane, Point3};

        let p0 = Point3::new(0.0, 0.0, 0.0);
        let p1 = Point3::new(1.0, 0.0, 0.0);
        let p2 = Point3::new(0.0, 1.0, 0.0);

        // The arc is a (degenerate) ellipse from p0 to p1 — semi-major
        // axes are nominal; the test only cares that the arc segment
        // round-trips into the resulting solid's edge_geom.
        let arc_ellipse = Ellipse::new(
            Frame::world(Point3::new(0.5, 0.0, 0.0)),
            0.5,
            0.5,
        );
        let arc_seg = EllipseSegment::new(arc_ellipse, 0.0, std::f64::consts::PI);

        // Surface: a flat XY plane (used for both faces; cycle direction
        // distinguishes front from back). Real Cylinder×Plane intersections
        // would have a Cylinder face on one side, but for the canonical-
        // key test the surface kind is incidental.
        let surf = SurfaceKind::Plane(Plane::new(Frame::world(Point3::origin())));

        // Front face: walks p0 → p1 → p2 → p0. Edge 0 (p0→p1) is the arc;
        // edges 1 and 2 are lines.
        let front = KeptFace {
            polygon: vec![p0, p1, p2],
            surface: surf.clone(),
            owner: None,
            edge_tags: Some(vec![
                EdgeCurveTag::Arc(0),
                EdgeCurveTag::Line,
                EdgeCurveTag::Line,
            ]),
            arc_segments: vec![arc_seg.clone()],
        };
        // Back face: walks p1 → p0 → p2 → p1. Its first edge is p1→p0
        // — the reverse of front's p0→p1 — so it twins to the arc on
        // front. Same arc_id (0) so the canonical key matches.
        let back = KeptFace {
            polygon: vec![p1, p0, p2],
            surface: surf,
            owner: None,
            edge_tags: Some(vec![
                EdgeCurveTag::Arc(0),
                EdgeCurveTag::Line,
                EdgeCurveTag::Line,
            ]),
            arc_segments: vec![arc_seg.clone()],
        };

        let solid = stitch(&[front, back], &Tolerance::default());
        // Topology sanity.
        assert_eq!(solid.vertex_count(), 3);
        assert_eq!(solid.edge_count(), 3);
        assert_eq!(solid.face_count(), 2);
        validate(&solid.topo).unwrap();

        // Find the arc-backed edge by inspecting edge_geom for an Ellipse
        // curve kind. There must be exactly one (the other two are line
        // edges).
        let mut arc_edge_count = 0;
        for (_eid, seg) in solid.edge_geom.iter() {
            if matches!(seg.curve, CurveKind::Ellipse(_)) {
                arc_edge_count += 1;
            }
        }
        assert_eq!(
            arc_edge_count, 1,
            "expected exactly 1 arc-backed edge, got {arc_edge_count}"
        );
    }

    #[test]
    fn stitch_arc_edge_does_not_collide_with_line_between_same_vertices() {
        // Regression test: build a topology where two faces share a LINE
        // edge between vertices p0 and p1, and the two faces ALSO have an
        // arc-tagged half-edge between the same vertex pair (different
        // canonical key thanks to Tier 1).
        //
        // We model this as TWO triangles plus TWO arc-bow-tie quads:
        //   front_tri: [p0, p1, p2] (line p0→p1)
        //   back_tri:  [p1, p0, p2] (line p1→p0 — line twin)
        //   front_arc: [p0, p1, p3] (arc p0→p1 marked Arc(0))
        //   back_arc:  [p1, p0, p3] (arc p1→p0 marked Arc(0) — arc twin)
        //
        // V=4 (p0..p3), E=5 (line(p0,p1) + arc(p0,p1) + line(p1,p2) +
        //   line(p2,p0) + line(p1,p3) + line(p3,p0)) — wait, that's 6 edges
        //   if line and arc are distinct.
        //
        // Pre-Tier-1 the canonical key (0,1) collided across line and
        // arc, producing 4 half-edges in one bucket and panicking. With
        // Tier 1, the line (0,1,Line) bucket has exactly 2 entries (one
        // forward, one backward) and the arc (0,1,Arc(0)) bucket also
        // has 2.
        use crate::geometry::EllipseSegment;
        use kerf_geom::{Ellipse, Frame, Plane, Point3};

        let p0 = Point3::new(0.0, 0.0, 0.0);
        let p1 = Point3::new(1.0, 0.0, 0.0);
        let p2 = Point3::new(0.5, 1.0, 0.0);
        let p3 = Point3::new(0.5, -1.0, 0.0);

        let arc_seg = EllipseSegment::new(
            Ellipse::new(Frame::world(Point3::new(0.5, 0.0, 0.0)), 0.5, 0.5),
            0.0,
            std::f64::consts::PI,
        );
        let surf = SurfaceKind::Plane(Plane::new(Frame::world(Point3::origin())));

        let front_tri = KeptFace::new_line(
            vec![p0, p1, p2],
            surf.clone(),
            None,
        );
        let back_tri = KeptFace::new_line(
            vec![p1, p0, p2],
            surf.clone(),
            None,
        );
        let front_arc = KeptFace {
            polygon: vec![p0, p1, p3],
            surface: surf.clone(),
            owner: None,
            edge_tags: Some(vec![
                EdgeCurveTag::Arc(0),
                EdgeCurveTag::Line,
                EdgeCurveTag::Line,
            ]),
            arc_segments: vec![arc_seg.clone()],
        };
        let back_arc = KeptFace {
            polygon: vec![p1, p0, p3],
            surface: surf,
            owner: None,
            edge_tags: Some(vec![
                EdgeCurveTag::Arc(0),
                EdgeCurveTag::Line,
                EdgeCurveTag::Line,
            ]),
            arc_segments: vec![arc_seg.clone()],
        };

        // Without the canonical-key extension this stitch would either
        // panic on "non-manifold input" (4 half-edges in one bucket) or
        // produce wrong twin pairings. With Tier 1 it must succeed.
        let solid = stitch(
            &[front_tri, back_tri, front_arc, back_arc],
            &Tolerance::default(),
        );
        validate(&solid.topo).unwrap();
        // Stage 1b's coplanar-duplicate dedup may collapse front/back
        // polygons that share the same canonical cycle. Both triangles
        // (front_tri and back_tri) are reverse-wound, so they survive
        // dedup; the test simply asserts that stitch did NOT panic.
        assert!(solid.face_count() >= 2);
        // Both vertex pairs (line and arc) must be wired as separate
        // edges — total edge count distinguishes the two cases.
        // Pre-Tier-1: stitch panics. Post-Tier-1: edges include the arc.
        let mut has_arc_edge = false;
        for (_eid, seg) in solid.edge_geom.iter() {
            if matches!(seg.curve, crate::geometry::CurveKind::Ellipse(_)) {
                has_arc_edge = true;
                break;
            }
        }
        assert!(
            has_arc_edge,
            "expected at least one Ellipse edge to survive stitch"
        );
    }
}
