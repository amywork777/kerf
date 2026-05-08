# kerf-cad viewer

Browser-based viewer for kerf-cad models. Drop a `.json` model, scrub
parameter sliders, see the mesh re-evaluate live. Pure WASM — the kerf
boolean kernel runs in the browser.

## Setup

```sh
# 1. install JS deps
pnpm install

# 2. build the WASM crate + JS bindings + sync examples
./build-wasm.sh

# 3. start the dev server
pnpm dev
```

Then open the URL Vite prints (defaults to http://localhost:5174).

## How it works

- **`kerf-cad-wasm`** crate exposes `evaluate_with_params(json, target_id, params, segments)`,
  `parameters_of`, and `target_ids_of` — see `crates/kerf-cad-wasm/src/lib.rs`.
- **`viewer/src/main.ts`** uses Three.js to render the resulting triangle list
  with orbit camera, key/fill lighting, and a grid floor.
- Loading a model autodiscovers its parameters and creates a slider for each
  one. Each slider change triggers a fresh kernel evaluation in WASM and
  rebuilds the mesh.

## Dev loop

1. Change kerf / kerf-brep / kerf-cad / kerf-cad-wasm Rust code.
2. Run `./build-wasm.sh` to rebuild + regenerate JS glue.
3. Vite hot-reloads the page.

## Limitations

- Read-only viewer. Drop a JSON, scrub sliders, view; saving back a modified
  JSON is left for a future iteration.
- Tessellation is fixed at 24 segments for curved primitives — a future
  knob.
- No picking, no measurement, no sketching — this is a preview, not an
  editor.
