# WASM Size Analysis

**Branch:** `perf/wasm-size-analysis`  
**Date:** 2026-05-15  
**File:** `viewer/src/wasm/kerf_cad_wasm_bg.wasm`

## Current State

| Milestone | Size |
|-----------|------|
| Session start (pre analytic-edge) | 2.69 MB |
| After analytic-edge PRs (#91, #107) | 2.79 MB |
| Delta | +100 KB |

The build pipeline already applies all the standard size knobs:
`opt-level = "z"`, `lto = "fat"`, `codegen-units = 1`, `panic = "abort"`,
`strip = true`, followed by `wasm-opt -Oz`. There is no obvious "free" flag
left at the toolchain level.

---

## Top Contributors (by source volume)

`twiggy` was not run (network-sandboxed build environment). The estimates below
are based on source-line counts and known library footprints; exact byte splits
require a `twiggy top` or `wasm2wat` pass on a WASM with debug sections.

| Module / dependency | Source lines | Role | Estimated WASM share |
|---|---|---|---|
| `kerf-cad/src/eval.rs` | 10 922 | 311-arm match tree dispatching every `Feature` variant to its primitive constructor | largest single file; ~300–500 KB |
| `kerf-cad/src/feature.rs` | 4 706 | 311-variant `Feature` enum + serde `#[serde(tag="kind")]` derive | serde generates per-variant visitor; ~100–200 KB |
| `kerf-brep/booleans/*` | 7 009 | Full boolean pipeline (stitch, intersect, classify, chord-merge, etc.) | ~150–250 KB |
| `kerf-cad/src/solver.rs` | 1 916 | Iterative 2D sketch constraint solver (Newton / Gauss-Seidel) | ~50–80 KB |
| `kerf-brep/src/step.rs` | 1 043 | ISO 10303-21 STEP writer | ~60–100 KB |
| `kerf-brep/src/dimension.rs` | 1 325 | 2D view projection, silhouette, snap, SVG annotation | ~80–120 KB |
| `kerf-brep/src/export_3mf.rs` + `zip` crate | 417 + zip deflate | 3MF ZIP archive writer | ~40–70 KB (zip deflate encoder) |
| `kerf-brep/src/export_gltf.rs` | 415 | GLB binary export | ~25–40 KB |
| `nalgebra` | workspace dep | `Point3`/`Vec3` (via kerf-geom); `Rotation3` in `transform.rs` | ~30–60 KB (only Vector3/Point3/Rotation3 used) |
| `serde_json` | workspace dep | Model/Feature deserialization + `serde_json::Value` in `catalog.rs` | ~80–130 KB |
| **analytic-edge work** (PRs #91, #107) | 876 | `AnalyticEdge` data model + `attach_analytic_circles` wired into boolean pipeline on every call | ~100 KB (matches the measured +100 KB delta) |

### Analytic-edge delta breakdown

`attach_analytic_circles` is called unconditionally inside
`kerf-brep/src/booleans/pipeline.rs` after every boolean operation. It
compiles in `analytic_edge_detect.rs` (449 lines) and `analytic_edge.rs` (427
lines), plus their serde impls and the `SecondaryMap<FaceId, AnalyticEdge>` on
`Solid`. These were the main driver of the 2.69 → 2.79 MB jump.

---

## Lazy-Load Candidates

### 1. STEP writer (`kerf-brep/src/step.rs`) — est. 60–100 KB

**Current:** compiled into the monolithic WASM even though no `#[wasm_bindgen]`
export triggers it at run-time — LTO strips dead code, but the string-heavy STEP
emitter (entity deduplication maps, AP214 format strings) survives because
`kerf-brep` is a single compilation unit and `lib.rs` `pub use step::write_step`.

**Opportunity:** Gate behind a `feature = "step"` Cargo feature on `kerf-brep`
and `kerf-cad-wasm`. Build a second micro-WASM (`kerf_cad_step.wasm`) loaded
on demand when the user triggers "Export STEP". Estimated savings: **80–120 KB**.

### 2. 3MF + `zip` deflate encoder (`export_3mf.rs` + `zip` crate) — est. 40–70 KB

**Current:** `zip` with `features = ["deflate"]` pulls in the miniz/flate2
encoder. No viewer UI currently exposes 3MF export; the function exists only
for the CLI binary.

**Opportunity:** Same Cargo-feature gate as STEP. Move `zip` to an optional
dep on `kerf-brep` under `features = ["export-3mf"]`. Lazy-load a second WASM
chunk on "Export 3MF". Estimated savings: **40–70 KB** (mostly the deflate
encoder, ~30–40 KB compressed).

### 3. GLB/GLTF writer (`export_gltf.rs`) — est. 25–40 KB

**Current:** Compiled in but never called from the browser. Same situation as
3MF.

**Opportunity:** Pair with the 3MF feature gate. Estimated savings: **25–40 KB**.

### 4. Dimension / 2D-view / SVG module (`dimension.rs`) — est. 80–120 KB

**Current:** `kerf-brep/src/dimension.rs` (1 325 lines) covers silhouette
projection, snap picking, and SVG annotation for the dimension panel. The
viewer's `dimensions.ts` calls `render_drawing_svg` via WASM but the function
is not currently wired as a `#[wasm_bindgen]` export (the JS mock in tests
bypasses it). If it is not reachable, LTO may already strip it; if it is
reachable (e.g. via a future `render_drawing_svg` export), it's a prime lazy
candidate.

**Opportunity:** Extract a `kerf_cad_drawing.wasm` chunk loaded only when the
dimensions panel is opened. Estimated savings: **80–120 KB** if currently
reachable.

### 5. Sketch solver (`kerf-cad/src/solver.rs`) — est. 50–80 KB

**Current:** `solver.rs` (1 916 lines, Newton / Gauss-Seidel, Jacobian,
diagnostics) is always compiled in. The sketcher is not open for most model
evaluations.

**Opportunity:** Not straightforward to split from WASM (the solver is called
synchronously during evaluation of `Feature::Sketch*`). Could be deferred by
lazy-instantiating a second Worker that owns a `solver` WASM and posts results
back; the main WASM would skip the solver module. Estimated savings: **50–80 KB**
but requires a Worker-based message-passing redesign.

---

## Cheap Wins (no architectural change)

### A. `serde_json::Value` in `catalog.rs`

`kerf-cad/src/catalog.rs` uses `serde_json::Value` to build default-example
JSON objects for 231 feature variants. `catalog.rs` is reachable from
`kerf-cad` which is a dependency of `kerf-cad-wasm`. If LTO does not strip the
catalog (it may if none of the 5 WASM-exported functions call it), this is free.
If it survives, replacing `serde_json::Value` with direct `String` building
would remove the `Map<String, Value>` allocator paths.

**Action:** Run `twiggy top` on an unoptimised WASM and check for
`kerf_cad::catalog::` symbols. If present, add `#[cfg(not(target_arch = "wasm32"))]`
guards on the catalog module.

### B. `nalgebra` `serde-serialize` feature

`Cargo.toml` pulls `nalgebra` with `features = ["serde-serialize"]`. This
generates serde impls for every nalgebra type (Matrix, Quaternion, etc.),
most of which are unused in WASM. Removing the feature (or gating it behind
`cfg(not(target_arch = "wasm32"))`) could save 20–40 KB.

### C. Cargo feature gates on `kerf-brep` export modules

The minimum-footprint fix: add a `default = []` feature on `kerf-brep` and
gate `export_3mf`, `export_gltf`, `step`, `obj`, `mesh_import`, and
`dimension` behind `features = ["export-3mf", "export-gltf", "step", "obj",
"import", "dimension"]`. The `kerf-cad-wasm` `Cargo.toml` would depend on
`kerf-brep` without those features. LTO already does much of this but explicit
feature gating makes the boundary compiler-enforced and also speeds up
incremental builds.

**Expected total from A+B+C:** 80–180 KB with no viewer or API changes.

---

## Architectural Wins (medium-term)

| Change | Effort | Potential savings |
|---|---|---|
| Code-split WASM via Vite `manualChunks` (separate `step.wasm`, `export.wasm`, `drawing.wasm`) | Medium | 150–300 KB off initial load |
| Move heavy eval to a Web Worker; stream results back | High | 0 KB size savings, but removes main-thread jank (orthogonal concern) |
| Introduce `#[cfg(feature = "wasm")]` in kerf-brep to compile out analytic-edge detect | Low | ~100 KB (reverts the +100 KB delta; attach_analytic_circles is only needed for export paths today) |

### Highest-payoff next step

**Feature-gate `attach_analytic_circles` behind a non-default `kerf-brep` feature**
and disable it in `kerf-cad-wasm/Cargo.toml`. This is a one-line Cargo change
that directly reverts the measured 100 KB regression at no functional cost to
the viewer (the viewer never inspects `face_analytic_edges`).

---

## Summary

| Action | Type | Est. savings | Effort |
|---|---|---|---|
| Feature-gate `analytic_edge_detect` out of WASM build | Cheap win | ~100 KB | 1-line Cargo change |
| `nalgebra` drop `serde-serialize` feature | Cheap win | ~20–40 KB | 1-line Cargo change |
| Feature-gate STEP + 3MF + GLTF + OBJ + mesh_import | Cheap win | ~150–230 KB | ~10 Cargo + lib.rs lines |
| Lazy-load STEP as separate WASM chunk | Architectural | ~80–120 KB off initial load | 1–2 days |
| Lazy-load 3MF + GLTF as export chunk | Architectural | ~65–110 KB off initial load | 1–2 days |
| Lazy-load dimension/drawing module | Architectural | ~80–120 KB off initial load | 2–3 days |

Combined cheap wins alone could bring the binary from 2.79 MB down to
approximately **2.40–2.50 MB** with no viewer changes and minimal risk.
