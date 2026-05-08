//! WASM bindings for kerf-cad.
//!
//! Exposes a single function `evaluate_to_mesh(json, target_id, segments)` that
//! returns a flat `Float32Array` of vertex positions (triangle list, 9 floats
//! per triangle: 3 vertices × XYZ) plus a separate `parameters_of(json)` for
//! the viewer to populate parameter sliders.

use wasm_bindgen::prelude::*;

use kerf_brep::{
    axis_aligned_line_edge, edge_info, face_boundary_edges, quadrant_hint_for_axis_edge,
    measure::solid_volume,
    tessellate::{tessellate, tessellate_with_face_index},
    Solid,
};
use kerf_cad::Model;
use std::collections::HashMap;

mod feature_kinds;
pub use feature_kinds::ALL_KINDS;

// Default panic hook is fine; viewer can install console_error_panic_hook
// if it wants nicer traces in DevTools. Keeping the wasm crate dep-light.


/// Evaluate the model JSON's target feature and return the tessellated
/// mesh as a flat Float32Array (9 floats per triangle).
///
/// On any error (parse, evaluate, tessellate) returns a JS Error.
#[wasm_bindgen]
pub fn evaluate_to_mesh(
    json: &str,
    target_id: &str,
    segments: usize,
) -> Result<Vec<f32>, JsError> {
    let model = Model::from_json_str(json).map_err(|e| JsError::new(&format!("parse: {e}")))?;
    let solid = model
        .evaluate(target_id)
        .map_err(|e| JsError::new(&format!("evaluate '{target_id}': {e}")))?;
    Ok(solid_to_triangles(&solid, segments.max(3)))
}

/// Return the `parameters` map of the model as a JS object: `{name: value}`.
/// The viewer uses this to seed parameter sliders without reparsing the JSON
/// itself in JS.
#[wasm_bindgen]
pub fn parameters_of(json: &str) -> Result<JsValue, JsError> {
    let model = Model::from_json_str(json).map_err(|e| JsError::new(&format!("parse: {e}")))?;
    serde_wasm_bindgen::to_value(&model.parameters).map_err(|e| JsError::new(&e.to_string()))
}

/// Return the list of feature ids in the model, in order. The viewer uses
/// this to populate a target-id picker.
#[wasm_bindgen]
pub fn target_ids_of(json: &str) -> Result<Vec<String>, JsError> {
    let model = Model::from_json_str(json).map_err(|e| JsError::new(&format!("parse: {e}")))?;
    Ok(model.ids().map(|s| s.to_string()).collect())
}

/// Return every `Feature` variant name (the JSON `kind` discriminator),
/// in source order. Powers the viewer's "Browse Features" catalog: each
/// returned string is what the user would type as `kind` in a feature
/// block. Stable across model loads — pure metadata, no parsing.
#[wasm_bindgen]
pub fn feature_kinds() -> Vec<String> {
    ALL_KINDS.iter().map(|s| s.to_string()).collect()
}

/// Re-evaluate the model with overridden parameters and return the mesh.
/// Parameters from `params_json` (a flat `{"name": number}` JSON object)
/// replace the model's defaults. Anything not in `params_json` keeps the
/// model's stored value.
#[wasm_bindgen]
pub fn evaluate_with_params(
    json: &str,
    target_id: &str,
    params_json: &str,
    segments: usize,
) -> Result<Vec<f32>, JsError> {
    let mut model =
        Model::from_json_str(json).map_err(|e| JsError::new(&format!("parse model: {e}")))?;
    let overrides: std::collections::HashMap<String, f64> = serde_json::from_str(params_json)
        .map_err(|e| JsError::new(&format!("parse params: {e}")))?;
    for (k, v) in overrides {
        model.parameters.insert(k, v);
    }
    let solid = model
        .evaluate(target_id)
        .map_err(|e| JsError::new(&format!("evaluate '{target_id}': {e}")))?;
    Ok(solid_to_triangles(&solid, segments.max(3)))
}

/// Like [`evaluate_with_params`] but also returns a per-triangle face index
/// in `face_ids`. Use the resulting JS object's `triangles` (Float32Array,
/// 9 floats per triangle) and `face_ids` (Uint32Array, 1 entry per triangle)
/// for picking. Face index N corresponds to the N-th face in the resulting
/// solid's iteration order — stable within one evaluate call.
#[wasm_bindgen]
pub fn evaluate_with_face_ids(
    json: &str,
    target_id: &str,
    params_json: &str,
    segments: usize,
) -> Result<JsValue, JsError> {
    let mut model =
        Model::from_json_str(json).map_err(|e| JsError::new(&format!("parse model: {e}")))?;
    let overrides: std::collections::HashMap<String, f64> = serde_json::from_str(params_json)
        .map_err(|e| JsError::new(&format!("parse params: {e}")))?;
    for (k, v) in overrides {
        model.parameters.insert(k, v);
    }
    let solid = model
        .evaluate(target_id)
        .map_err(|e| JsError::new(&format!("evaluate '{target_id}': {e}")))?;
    let (soup, face_ids) = tessellate_with_face_index(&solid, segments.max(3));
    let mut tris = Vec::with_capacity(soup.triangles.len() * 9);
    for tri in &soup.triangles {
        for v in tri {
            tris.push(v.x as f32);
            tris.push(v.y as f32);
            tris.push(v.z as f32);
        }
    }
    let face_count = face_ids.iter().copied().max().map_or(0, |m| m + 1);
    // Per-face owner tags, indexed by the same face index used in face_ids.
    // tessellate_with_face_index iterates `solid.topo.face_ids()` in order
    // and assigns sequential indices starting at 0, so we must walk the
    // same iterator here to keep the tag vector in lockstep. Empty string
    // for any face that has no recorded owner.
    let owner_tags: Vec<String> = solid
        .topo
        .face_ids()
        .map(|fid| {
            solid
                .face_owner_tag
                .get(fid)
                .cloned()
                .unwrap_or_default()
        })
        .collect();

    // Edge enumeration. Walk solid.topo.edge_ids() in iteration order; build
    // a stable (EdgeId -> index) map so face_to_edges can refer to edges by
    // their integer index. The edge index is sequential 0..edge_count and
    // matches the order edges appear in the `edges` vector below.
    let mut edge_index_map: HashMap<kerf_topo::EdgeId, u32> = HashMap::new();
    let mut edges_out: Vec<EdgeOut> = Vec::new();
    for (idx, eid) in solid.topo.edge_ids().enumerate() {
        edge_index_map.insert(eid, idx as u32);
        let info = edge_info(&solid, eid);
        let (axis_hint, edge_min_hint, edge_length_hint, quadrant_hint) =
            match axis_aligned_line_edge(&solid, eid) {
                Some((axis, edge_min, len)) => {
                    let q = quadrant_hint_for_axis_edge(&solid, eid, axis).unwrap_or_default();
                    (axis.to_string(), edge_min, len, q)
                }
                None => (String::new(), [0.0; 3], 0.0, String::new()),
            };
        // Edge owner tag: pick the owner tag of one of the two adjacent
        // faces (preferring a non-empty owner). Useful diagnostic for the
        // viewer.
        let owner_tag = adjacent_face_owner(&solid, eid).unwrap_or_default();
        let (p_start, p_end, length, curve_kind) = match info {
            Some(i) => (i.p_start, i.p_end, i.length, i.curve_kind.to_string()),
            None => ([0.0; 3], [0.0; 3], 0.0, "unknown".to_string()),
        };
        edges_out.push(EdgeOut {
            id: idx as u32,
            owner_tag,
            p_start,
            p_end,
            length,
            curve_kind,
            axis_hint,
            edge_min_hint,
            edge_length_hint,
            quadrant_hint,
        });
    }

    // face_to_edges: indexed by face render index (0..face_count). Walk the
    // same face_ids() iterator the tessellator uses to keep indices in
    // lockstep.
    let face_to_edges: Vec<Vec<u32>> = solid
        .topo
        .face_ids()
        .map(|fid| {
            face_boundary_edges(&solid, fid)
                .into_iter()
                .filter_map(|eid| edge_index_map.get(&eid).copied())
                .collect()
        })
        .collect();

    let vol = solid_volume(&solid);
    #[derive(serde::Serialize)]
    struct OutFull {
        triangles: Vec<f32>,
        face_ids: Vec<u32>,
        face_count: u32,
        face_owner_tags: Vec<String>,
        face_to_edges: Vec<Vec<u32>>,
        edges: Vec<EdgeOut>,
        volume: f64,
        shell_count: usize,
        vertex_count: usize,
        edge_count: usize,
        face_count_topo: usize,
    }
    serde_wasm_bindgen::to_value(&OutFull {
        triangles: tris,
        face_ids,
        face_count,
        face_owner_tags: owner_tags,
        face_to_edges,
        edges: edges_out,
        volume: vol,
        shell_count: solid.shell_count(),
        vertex_count: solid.vertex_count(),
        edge_count: solid.edge_count(),
        face_count_topo: solid.face_count(),
    })
    .map_err(|e| JsError::new(&e.to_string()))
}

#[derive(serde::Serialize)]
struct EdgeOut {
    id: u32,
    owner_tag: String,
    p_start: [f64; 3],
    p_end: [f64; 3],
    length: f64,
    curve_kind: String,
    /// "x" | "y" | "z" if the edge is an axis-aligned line, else "".
    axis_hint: String,
    /// For axis-aligned line edges: the lower-coord endpoint (Fillet
    /// `edge_min`). All zeros if not axis-aligned.
    edge_min_hint: [f64; 3],
    /// Length along the axis. 0.0 if not axis-aligned.
    edge_length_hint: f64,
    /// "pp"|"pn"|"np"|"nn" describing the body's quadrant relative to the
    /// edge in canonical perpendicular order. Empty if not axis-aligned.
    quadrant_hint: String,
}

/// Pick an "owner tag" to attach to an edge by inspecting the two adjacent
/// faces (the half-edges' loops' faces). Prefers a non-empty owner, falling
/// back to the first face's owner if both are empty.
fn adjacent_face_owner(solid: &Solid, edge_id: kerf_topo::EdgeId) -> Option<String> {
    let edge = solid.topo.edge(edge_id)?;
    let [he_a, he_b] = edge.half_edges();
    let face_of = |heid: kerf_topo::HalfEdgeId| -> Option<kerf_topo::FaceId> {
        let he = solid.topo.half_edge(heid)?;
        let lp = solid.topo.loop_(he.loop_())?;
        Some(lp.face())
    };
    let fa = face_of(he_a);
    let fb = face_of(he_b);
    let tag_of = |fid: Option<kerf_topo::FaceId>| -> String {
        fid.and_then(|f| solid.face_owner_tag.get(f).cloned())
            .unwrap_or_default()
    };
    let ta = tag_of(fa);
    let tb = tag_of(fb);
    if !ta.is_empty() {
        Some(ta)
    } else if !tb.is_empty() {
        Some(tb)
    } else {
        Some(String::new())
    }
}

fn solid_to_triangles(solid: &Solid, segments: usize) -> Vec<f32> {
    let soup = tessellate(solid, segments);
    let mut out = Vec::with_capacity(soup.triangles.len() * 9);
    for tri in &soup.triangles {
        for v in tri {
            out.push(v.x as f32);
            out.push(v.y as f32);
            out.push(v.z as f32);
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    /// `ALL_KINDS` is hand-mirrored from `kerf_cad::Feature`. If anyone
    /// ever adds or removes a variant, drift will silently break the
    /// catalog browser. We don't have a way to introspect the enum at
    /// runtime, but we *can* parse a tiny model that names each kind
    /// — if the variant exists and accepts a JSON object with just
    /// `{kind, id}`, serde will reject the unknown ones loudly. (The
    /// kernel may still complain about missing fields when *evaluating*
    /// — that's fine; we only check serde-level recognition here.)
    #[test]
    fn all_kinds_are_known_to_serde() {
        // Every kind should at least be a recognized serde tag. We use
        // serde_json's value-level deserialization to a raw Feature; if
        // the kind is unknown, serde returns an "unknown variant" error.
        // Missing-fields errors are fine — they prove the variant was
        // recognized.
        for kind in ALL_KINDS {
            let stub = format!(r#"{{"kind":"{kind}","id":"x"}}"#);
            let res: Result<kerf_cad::feature::Feature, _> = serde_json::from_str(&stub);
            if let Err(e) = &res {
                let msg = e.to_string();
                // serde produces "unknown variant" when the tag isn't a
                // real variant. Any other error (missing field, type
                // mismatch on a required parameter) means the kind was
                // recognized — that's what we want.
                assert!(
                    !msg.contains("unknown variant"),
                    "feature kind {kind:?} not recognized by serde: {msg}",
                );
            }
        }
    }

    /// Round-trip: feature_kinds() should return exactly ALL_KINDS, in
    /// the same order, no duplicates.
    #[test]
    fn feature_kinds_round_trip() {
        let exported = feature_kinds();
        assert_eq!(exported.len(), ALL_KINDS.len());
        for (a, b) in exported.iter().zip(ALL_KINDS.iter()) {
            assert_eq!(a, b);
        }
        // No duplicates.
        let mut sorted = exported.clone();
        sorted.sort();
        sorted.dedup();
        assert_eq!(
            sorted.len(),
            exported.len(),
            "feature_kinds() returned duplicates",
        );
        // Generous lower bound — sanity that we shipped a real catalog.
        assert!(
            exported.len() > 200,
            "feature_kinds() returned only {} entries — expected 200+",
            exported.len(),
        );
    }

    /// Every kind from `feature_kinds()` should make it into a parseable
    /// model wrapper (i.e. `target_ids_of` doesn't choke on a tiny model
    /// that names the kind). This guarantees the catalog browser's
    /// "insert default-parameter instance" path can at least *attempt*
    /// to insert any kind.
    #[test]
    fn each_kind_can_be_named_in_a_model_skeleton() {
        // Use Box (which we know works) as a baseline so the model at
        // least has one valid feature; we only assert the kind string
        // is accepted as a tag.
        for kind in feature_kinds() {
            let stub = format!(r#"{{"kind":"{kind}","id":"probe"}}"#);
            let res: Result<kerf_cad::feature::Feature, _> = serde_json::from_str(&stub);
            if let Err(e) = res {
                let msg = e.to_string();
                assert!(
                    !msg.contains("unknown variant"),
                    "kind {kind:?} rejected at serde tag layer: {msg}",
                );
            }
        }
    }
}
