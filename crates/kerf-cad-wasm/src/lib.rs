//! WASM bindings for kerf-cad.
//!
//! Exposes a single function `evaluate_to_mesh(json, target_id, segments)` that
//! returns a flat `Float32Array` of vertex positions (triangle list, 9 floats
//! per triangle: 3 vertices × XYZ) plus a separate `parameters_of(json)` for
//! the viewer to populate parameter sliders.
//!
//! ## Caching
//!
//! The viewer hits these functions on every parameter slider tick, so we
//! keep two persistent caches in module-local thread-local state:
//!
//! 1. An [`EvalCache`] (recipe-fingerprint keyed) for Solid evaluation —
//!    avoids re-running booleans when only an unrelated parameter moved.
//! 2. A [`MeshCache`] (fingerprint + segments keyed) for tessellated
//!    Float32Array meshes — avoids re-tessellating when only the camera
//!    moved or when the same solid is requested at the same resolution.
//!
//! WASM is single-threaded, so a `RefCell<...>` thread-local is safe.
//! Call `clear_cache()` from JS to drop both caches (e.g. on model swap).

use std::cell::RefCell;
use std::collections::HashMap;

use wasm_bindgen::prelude::*;

use kerf_brep::{
    mass::mass_properties,
    measure::solid_volume,
    tessellate::{tessellate, tessellate_with_face_index},
    Solid,
};
use kerf_cad::{import_step_to_model as cad_import_step_to_model, EvalCache, Fingerprint, Model};

// ---------------------------------------------------------------------------
// Module-local persistent caches.
//
// One cache instance per "session" (one wasm Module instance in the browser).
// The viewer never explicitly creates these; it just calls evaluate_*
// repeatedly and benefits from the warm cache transparently.
// ---------------------------------------------------------------------------

thread_local! {
    static EVAL_CACHE: RefCell<EvalCache> = RefCell::new(EvalCache::new());
    static MESH_CACHE: RefCell<MeshCache> = RefCell::new(MeshCache::default());
}

/// Tessellation cache: `(solid_fingerprint, segments) → triangles`.
///
/// Tessellating a 100k-triangle mesh takes ~5ms; serializing it to a
/// Float32Array adds another. When the user spins the camera, the solid
/// is unchanged and the segments count is unchanged — both cache key
/// components match — so we can hand back the previous Float32Array
/// directly, in O(1) plus a clone.
#[derive(Default)]
struct MeshCache {
    by_key: HashMap<(Fingerprint, usize), Vec<f32>>,
}

impl MeshCache {
    fn get(&self, key: (Fingerprint, usize)) -> Option<&Vec<f32>> {
        self.by_key.get(&key)
    }
    fn put(&mut self, key: (Fingerprint, usize), tris: Vec<f32>) {
        self.by_key.insert(key, tris);
    }
    fn len(&self) -> usize {
        self.by_key.len()
    }
    fn clear(&mut self) {
        self.by_key.clear();
    }
}

// ---------------------------------------------------------------------------
// Public WASM API.
//
// Existing exports (evaluate_to_mesh, parameters_of, target_ids_of,
// evaluate_with_params, evaluate_with_face_ids) keep their signatures
// exactly so the viewer's JS doesn't have to change. They now route
// through the cached evaluator internally.
// ---------------------------------------------------------------------------

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
    let segs = segments.max(3);
    evaluate_and_tessellate(&model, target_id, segs)
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
    let overrides: HashMap<String, f64> = serde_json::from_str(params_json)
        .map_err(|e| JsError::new(&format!("parse params: {e}")))?;
    for (k, v) in overrides {
        model.parameters.insert(k, v);
    }
    let segs = segments.max(3);
    evaluate_and_tessellate(&model, target_id, segs)
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
    let overrides: HashMap<String, f64> = serde_json::from_str(params_json)
        .map_err(|e| JsError::new(&format!("parse params: {e}")))?;
    for (k, v) in overrides {
        model.parameters.insert(k, v);
    }
    let solid = EVAL_CACHE.with(|cache| {
        let mut cache = cache.borrow_mut();
        model.evaluate_cached(target_id, &mut cache)
    })
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
    let vol = solid_volume(&solid);
    #[derive(serde::Serialize)]
    struct OutFull {
        triangles: Vec<f32>,
        face_ids: Vec<u32>,
        face_count: u32,
        face_owner_tags: Vec<String>,
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
        volume: vol,
        shell_count: solid.shell_count(),
        vertex_count: solid.vertex_count(),
        edge_count: solid.edge_count(),
        face_count_topo: solid.face_count(),
    })
    .map_err(|e| JsError::new(&e.to_string()))
}

/// Return the `equations` map of the model as a JS object: `{name: expression_string}`.
/// Equations are cross-parameter constraint relationships; see `evaluate_equations`
/// to actually resolve them.
#[wasm_bindgen]
pub fn list_equations(json: &str) -> Result<JsValue, JsError> {
    let model = Model::from_json_str(json).map_err(|e| JsError::new(&format!("parse: {e}")))?;
    serde_wasm_bindgen::to_value(&model.equations).map_err(|e| JsError::new(&e.to_string()))
}

/// Resolve all equations in the model JSON against the provided parameter
/// overrides (or the model's stored parameters if `params_json` is `"{}"`).
///
/// `params_json` — a flat `{"name": number}` JSON object. These override
/// the model's `parameters` before equations are resolved, mirroring the
/// behaviour of `evaluate_with_params`.
///
/// Returns `Object<name, number>` — the full resolved parameter map (base
/// parameters merged with all equation results). Errors surface as a JS
/// `Error` with a descriptive message (cycle, unknown reference, etc.).
#[wasm_bindgen]
pub fn evaluate_equations(json: &str, params_json: &str) -> Result<JsValue, JsError> {
    let mut model =
        Model::from_json_str(json).map_err(|e| JsError::new(&format!("parse model: {e}")))?;
    let overrides: HashMap<String, f64> = serde_json::from_str(params_json)
        .map_err(|e| JsError::new(&format!("parse params: {e}")))?;
    for (k, v) in overrides {
        model.parameters.insert(k, v);
    }
    let resolved = model
        .resolve_params()
        .map_err(|e| JsError::new(&format!("equations: {e}")))?;
    serde_wasm_bindgen::to_value(&resolved).map_err(|e| JsError::new(&e.to_string()))
}

/// Compute mass properties for the model's target. Assumes uniform unit
/// density. Returns a JS object with: volume, surface_area, centroid (3-tuple),
/// inertia_tensor (3×3 array), principal_moments (3-tuple),
/// principal_axes (3×3 array), aabb_min (3-tuple), aabb_max (3-tuple).
#[wasm_bindgen]
pub fn mass_properties_of(
    json: &str,
    target_id: &str,
    params_json: &str,
    _segments: usize,
) -> Result<JsValue, JsError> {
    let mut model =
        Model::from_json_str(json).map_err(|e| JsError::new(&format!("parse model: {e}")))?;
    let overrides: HashMap<String, f64> = serde_json::from_str(params_json)
        .map_err(|e| JsError::new(&format!("parse params: {e}")))?;
    for (k, v) in overrides {
        model.parameters.insert(k, v);
    }
    let solid = EVAL_CACHE.with(|cache| {
        let mut cache = cache.borrow_mut();
        model.evaluate_cached(target_id, &mut cache)
    })
    .map_err(|e| JsError::new(&format!("evaluate '{target_id}': {e}")))?;
    let mp = mass_properties(&solid);

    #[derive(serde::Serialize)]
    struct MassPropertiesOut {
        volume: f64,
        surface_area: f64,
        centroid: [f64; 3],
        inertia_tensor: [[f64; 3]; 3],
        principal_moments: [f64; 3],
        principal_axes: [[f64; 3]; 3],
        aabb_min: [f64; 3],
        aabb_max: [f64; 3],
    }
    serde_wasm_bindgen::to_value(&MassPropertiesOut {
        volume: mp.volume,
        surface_area: mp.surface_area,
        centroid: mp.centroid,
        inertia_tensor: mp.inertia_tensor,
        principal_moments: mp.principal_moments,
        principal_axes: mp.principal_axes,
        aabb_min: mp.aabb_min,
        aabb_max: mp.aabb_max,
    })
    .map_err(|e| JsError::new(&e.to_string()))
}

/// Parse a STEP (ISO 10303-21) AP203/AP214 file and return a kerf-cad
/// `Model` JSON containing a single `ImportedMesh` feature with id
/// `"imported"`.
///
/// The viewer's existing JSON-loading flow can then take this string and
/// drive the rest of the pipeline (parameter sliders, target-id picker,
/// face-id picking) unchanged. STEP geometry has no parameters, so the
/// parameters panel will simply be empty.
///
/// Only the planar-faceted polyhedral subset of STEP is supported; curved
/// surfaces (cylinders, spheres, NURBS, …) return a JS error mentioning
/// the specific entity type encountered.
#[wasm_bindgen]
pub fn import_step_to_model(step_text: &str) -> Result<String, JsError> {
    let model = import_step_to_model_internal(step_text)?;
    model
        .to_json_string()
        .map_err(|e| JsError::new(&format!("serialize imported model: {e}")))
}

fn import_step_to_model_internal(step_text: &str) -> Result<Model, JsError> {
    cad_import_step_to_model(step_text, "imported")
        .map_err(|e| JsError::new(&format!("import STEP: {e}")))
}

/// Drop both the eval cache and the tessellation cache.
///
/// Call from JS when:
/// - the user loads a different model JSON (recipes won't collide thanks
///   to fingerprint hashing, but cached entries from the old model are
///   wasted memory),
/// - the user reloads to clear stale state,
/// - the cache has grown large and you want to bound memory.
///
/// Safe to call any time. The next evaluate_* call will warm the cache.
#[wasm_bindgen]
pub fn clear_cache() {
    EVAL_CACHE.with(|c| c.borrow_mut().clear());
    MESH_CACHE.with(|c| c.borrow_mut().clear());
}

/// Return the current cache occupancy as `[eval_entries, mesh_entries]`.
/// Exposed for the viewer's "perf overlay" / debug tooling so you can
/// confirm the warm path is hitting the cache during a slider drag.
#[wasm_bindgen]
pub fn cache_stats() -> Vec<u32> {
    let eval_n = EVAL_CACHE.with(|c| c.borrow().len()) as u32;
    let mesh_n = MESH_CACHE.with(|c| c.borrow().len()) as u32;
    vec![eval_n, mesh_n]
}

// ---------------------------------------------------------------------------
// Internal helpers.
// ---------------------------------------------------------------------------

/// Evaluate `target_id` in `model` (eval-cache) and tessellate at
/// `segments` (mesh-cache). Returns the flat triangle Vec<f32>.
fn evaluate_and_tessellate(
    model: &Model,
    target_id: &str,
    segments: usize,
) -> Result<Vec<f32>, JsError> {
    let (solid, fp) = EVAL_CACHE.with(|cache| {
        let mut cache = cache.borrow_mut();
        model.evaluate_cached_with_fingerprint(target_id, &mut cache)
    })
    .map_err(|e| JsError::new(&format!("evaluate '{target_id}': {e}")))?;

    // Mesh cache lookup: same solid (= same fingerprint) + same segments
    // → return the previously tessellated triangles.
    let key = (fp, segments);
    if let Some(tris) = MESH_CACHE.with(|c| c.borrow().get(key).cloned()) {
        return Ok(tris);
    }

    let tris = solid_to_triangles(&solid, segments);
    MESH_CACHE.with(|c| c.borrow_mut().put(key, tris.clone()));
    Ok(tris)
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

// Host tests run with `cargo test -p kerf-cad-wasm` on the dev workstation.
// They don't require a wasm runtime — they exercise the cache layer
// directly through `evaluate_and_tessellate` and `clear_cache`. These
// functions are wasm_bindgen exports, but their bodies are plain Rust
// and run fine on any target.
#[cfg(all(test, not(target_arch = "wasm32")))]
mod tests {
    use super::*;
    use kerf_cad::{Feature, Scalar};

    fn box_model(w: f64) -> Model {
        Model::new()
            .with_parameter("w", w)
            .add(Feature::Box {
                id: "body".into(),
                extents: [Scalar::param("w"), Scalar::lit(1.0), Scalar::lit(1.0)],
            })
    }

    #[test]
    fn tessellation_cache_hits() {
        // Two evaluates of the same model at the same segment count must
        // share a mesh cache entry. We can't observe "did we recompute"
        // directly, but we can observe the entry count: cold + warm both
        // should leave exactly one entry.
        clear_cache();
        let m = box_model(2.0);
        let _ = evaluate_and_tessellate(&m, "body", 8).expect("cold");
        let stats_cold = cache_stats();
        let _ = evaluate_and_tessellate(&m, "body", 8).expect("warm");
        let stats_warm = cache_stats();
        // [eval_entries, mesh_entries]
        assert_eq!(stats_cold[1], 1, "one mesh entry after cold");
        assert_eq!(stats_warm[1], 1, "still one mesh entry after warm hit");
    }

    #[test]
    fn tessellation_cache_keys_on_segments() {
        // Same solid, different segment count → different cache key →
        // separate entries. Cylinder is the canonical "segments matter"
        // case but Box also re-tessellates because the API doesn't know
        // a-priori whether segments changes the output.
        clear_cache();
        let m = Model::new()
            .add(Feature::Cylinder {
                id: "c".into(),
                radius: Scalar::lit(1.0),
                height: Scalar::lit(2.0),
                segments: 16,
            });
        let _ = evaluate_and_tessellate(&m, "c", 8).expect("seg 8");
        let _ = evaluate_and_tessellate(&m, "c", 16).expect("seg 16");
        let stats = cache_stats();
        assert_eq!(stats[1], 2, "two mesh entries: one per segment count");
    }

    #[test]
    fn tessellation_cache_invalidates_on_param_change() {
        // Different param → different fingerprint → different mesh-cache key.
        // The cache adds an entry rather than evicting; that's fine because
        // the user might revisit the old value (slider scrub-back).
        clear_cache();
        let m1 = box_model(2.0);
        let m2 = box_model(3.0);
        let t1 = evaluate_and_tessellate(&m1, "body", 8).expect("m1");
        let t2 = evaluate_and_tessellate(&m2, "body", 8).expect("m2");
        // Different recipes should produce different triangle data
        // (different x extent → different x coords on half the verts).
        assert_ne!(t1, t2, "different params produce different meshes");
        assert_eq!(cache_stats()[1], 2, "two distinct mesh entries");
    }

    #[test]
    fn clear_cache_drops_both_caches() {
        clear_cache();
        let m = box_model(2.0);
        let _ = evaluate_and_tessellate(&m, "body", 8).expect("warm");
        let stats_warm = cache_stats();
        assert!(stats_warm[0] >= 1 && stats_warm[1] >= 1);
        clear_cache();
        let stats_cleared = cache_stats();
        assert_eq!(stats_cleared, vec![0, 0], "both caches empty after clear");
    }

    #[test]
    fn evaluate_and_tessellate_matches_uncached_path() {
        // Sanity: the cached path produces the same triangles as the plain
        // tessellate(evaluate(...)) path. Catches accidental divergences
        // in the cache layer.
        clear_cache();
        let m = box_model(2.0);
        let cached = evaluate_and_tessellate(&m, "body", 8).expect("cached");
        let plain = solid_to_triangles(&m.evaluate("body").expect("plain"), 8);
        assert_eq!(cached.len(), plain.len(), "same triangle count");
        // Triangles should be bit-identical (same evaluator, same tessellator).
        for (a, b) in cached.iter().zip(plain.iter()) {
            assert!((a - b).abs() < 1e-6, "mismatch: {a} vs {b}");
        }
    }
}
