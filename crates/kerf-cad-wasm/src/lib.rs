//! WASM bindings for kerf-cad.
//!
//! Exposes a single function `evaluate_to_mesh(json, target_id, segments)` that
//! returns a flat `Float32Array` of vertex positions (triangle list, 9 floats
//! per triangle: 3 vertices × XYZ) plus a separate `parameters_of(json)` for
//! the viewer to populate parameter sliders.

use wasm_bindgen::prelude::*;

use kerf_brep::{
    dimension::{
        self, render_dimensioned_view, Dimension, DimensionKind, ViewKind, ViewportSpec,
    },
    measure::solid_volume,
    tessellate::{tessellate, tessellate_with_face_index},
    Solid,
};
use kerf_cad::Model;
use kerf_geom::{Point3, Vec3};

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

// ----------------------------------------------------------------------
// Drawing-dimension helpers (back-end half of the "Drawings" capability).
//
// The viewer can drive these without re-implementing the math. All inputs
// are flat `[f64; N]` arrays so the JS side doesn't need to know about
// nalgebra Point3/Vec3.
// ----------------------------------------------------------------------

fn pt(a: &[f64]) -> Result<Point3, JsError> {
    if a.len() < 3 {
        return Err(JsError::new("expected [x, y, z]"));
    }
    Ok(Point3::new(a[0], a[1], a[2]))
}

fn vc(a: &[f64]) -> Result<Vec3, JsError> {
    if a.len() < 3 {
        return Err(JsError::new("expected [x, y, z]"));
    }
    Ok(Vec3::new(a[0], a[1], a[2]))
}

/// Euclidean distance between two world-space points.
#[wasm_bindgen]
pub fn measure_distance(p1: Vec<f64>, p2: Vec<f64>) -> Result<f64, JsError> {
    Ok(dimension::distance(pt(&p1)?, pt(&p2)?))
}

/// Project a 3D point into 2D paper coordinates for the named view.
/// `view` is one of `"top"`, `"front"`, `"side"`, `"iso"`. Returned
/// `[u, v]` is in world units.
#[wasm_bindgen]
pub fn project_to_view(p: Vec<f64>, view: &str) -> Result<Vec<f64>, JsError> {
    let v = ViewKind::from_str(view)
        .ok_or_else(|| JsError::new(&format!("unknown view '{view}' (top|front|side|iso)")))?;
    let (u, vv) = dimension::to_2d_view(pt(&p)?, v);
    Ok(vec![u, vv])
}

/// Angle (radians) at `vertex` formed by rays to `a` and `b`. Range [0, pi].
#[wasm_bindgen]
pub fn angle_at_vertex(a: Vec<f64>, vertex: Vec<f64>, b: Vec<f64>) -> Result<f64, JsError> {
    Ok(dimension::angle_at_vertex(pt(&a)?, pt(&vertex)?, pt(&b)?))
}

/// Project a point onto a plane defined by `(plane_origin, plane_normal)`.
#[wasm_bindgen]
pub fn project_point_to_plane(
    p: Vec<f64>,
    plane_normal: Vec<f64>,
    plane_origin: Vec<f64>,
) -> Result<Vec<f64>, JsError> {
    let q = dimension::project_to_plane(pt(&p)?, vc(&plane_normal)?, pt(&plane_origin)?);
    Ok(vec![q.x, q.y, q.z])
}

/// Render a model + named view + dimension list to an SVG string.
///
/// `dimensions_json` is an array of objects:
///   `{ "kind": "linear",   "from": [x,y,z], "to": [x,y,z] }`
///   `{ "kind": "radial",   "from": [x,y,z], "to": [x,y,z], "center": [x,y,z] }`
///   `{ "kind": "angular",  "from": [x,y,z], "to": [x,y,z], "vertex": [x,y,z] }`
///
/// `viewport_json` is `{ "width": f64, "height": f64, "padding": f64 }` —
/// pass an empty object `{}` to use the default A4-ish viewport.
#[wasm_bindgen]
pub fn render_drawing_svg(
    json: &str,
    target_id: &str,
    params_json: &str,
    view: &str,
    dimensions_json: &str,
    viewport_json: &str,
) -> Result<String, JsError> {
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
    let view_kind = ViewKind::from_str(view)
        .ok_or_else(|| JsError::new(&format!("unknown view '{view}' (top|front|side|iso)")))?;

    #[derive(serde::Deserialize)]
    struct DimRaw {
        kind: String,
        from: [f64; 3],
        to: [f64; 3],
        center: Option<[f64; 3]>,
        vertex: Option<[f64; 3]>,
    }
    let raws: Vec<DimRaw> = serde_json::from_str(dimensions_json)
        .map_err(|e| JsError::new(&format!("parse dimensions: {e}")))?;
    let mut dims: Vec<Dimension> = Vec::with_capacity(raws.len());
    for r in raws {
        let from = Point3::new(r.from[0], r.from[1], r.from[2]);
        let to = Point3::new(r.to[0], r.to[1], r.to[2]);
        let kind = match r.kind.to_ascii_lowercase().as_str() {
            "linear" => DimensionKind::Linear,
            "radial" | "radius" | "radialfromcenter" => {
                let c = r.center.ok_or_else(|| JsError::new("radial dim missing 'center'"))?;
                DimensionKind::RadialFromCenter {
                    center: Point3::new(c[0], c[1], c[2]),
                }
            }
            "angular" | "angle" => {
                let v = r.vertex.ok_or_else(|| JsError::new("angular dim missing 'vertex'"))?;
                DimensionKind::Angular {
                    vertex: Point3::new(v[0], v[1], v[2]),
                }
            }
            other => return Err(JsError::new(&format!("unknown dimension kind '{other}'"))),
        };
        dims.push(Dimension { from, to, kind });
    }

    #[derive(serde::Deserialize, Default)]
    struct VpRaw {
        width: Option<f64>,
        height: Option<f64>,
        padding: Option<f64>,
    }
    let vp_raw: VpRaw = serde_json::from_str(viewport_json)
        .map_err(|e| JsError::new(&format!("parse viewport: {e}")))?;
    let default_vp = ViewportSpec::default_a4_ish();
    let viewport = ViewportSpec::new(
        vp_raw.width.unwrap_or(default_vp.width),
        vp_raw.height.unwrap_or(default_vp.height),
        vp_raw.padding.unwrap_or(default_vp.padding),
    );

    Ok(render_dimensioned_view(&solid, view_kind, &dims, viewport))
}
