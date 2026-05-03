# kerf

A minimal-but-real B-rep solid modeling kernel in Rust.

See `docs/superpowers/specs/2026-04-28-kerf-brep-kernel-design.md` (in the parent
`code/` workspace) for the design.

## Status

**Current readiness: 168/168 (100%)** of primitive × primitive × op
combinations succeed. See [`docs/readiness.md`](docs/readiness.md) for the
milestone history. Run `cargo run --example readiness_matrix -p kerf-brep`
to refresh.

**Validation: 327 tests across 9 suites:**
- `readiness_floor.rs` (1) — topology validates for all 168 matrix cases.
- `readiness_geometry.rs` (13) — universal volume invariants + analytic
  exact-volume checks for box pairs + Euler-Poincaré on every result.
- `readiness_algebra.rs` (9) — idempotence, commutativity, set-algebra
  conservation `vol(A) = vol(A∩B) + vol(A−B)`, inclusion-exclusion,
  associativity for disjoint unions.
- `readiness_quality.rs` (2) — bbox containment + no-zero-area-triangle
  per face on every matrix case.
- `readiness_roundtrip.rs` (7) — STL + OBJ + JSON serialization preserves
  V/E/F/volume.
- `readiness_robustness.rs` (9) — recursive booleans on overlapping
  inputs, tessellation refinement convergence, imported-mesh booleans,
  vertex-on-face-plane consistency, translation invariance, determinism,
  no-duplicate-vertex check.
- `readiness_robustness2.rs` (12) — rotation invariance, scale extremes
  (1e-3 to 1e3), coplanar-touching boxes, performance budget (<1s per
  matrix case), random box-pair fuzz (50 cases × 3 ops via deterministic
  LCG seed verifying inclusion-exclusion analytically).
- `readiness_stress.rs` (22) — tessellate primitives at varying
  resolutions → STL → re-import → run booleans on the imported version;
  high-res 64-segment cylinder, 9-cavity grid carving, deep recursive
  imported chains.
- `readiness_curved.rs` (10) — sphere / torus / cone via tessellation +
  re-import; analytic volume verification, genus-1 torus topology check,
  small-chip torus DIFF.
- `readiness_noisy.rs` (6) — sub-tolerance noise preserves boolean
  correctness.
- `readiness_complex_cad.rs` (1+2 ignored) — multi-step CAD models via
  boolean chains.

**Performance baseline (criterion):** see `cargo bench -p kerf-brep`.
- nested box DIFF (creates cavity): ~27 µs
- box overlap pair (any op): ~38 µs
- tri-prism − box-nested (genus 1 result): ~59 µs
- cyl12 pierces box: ~112 µs
- box − cyl_n64 (high-res, ~192 face inputs): ~1.05 ms

**Known limitations:**
- Coplanar overlapping faces between A and B (one-plane bodies sitting
  on top of each other or sharing a face exactly) — limited to
  same-direction-normals coplanar; opposite-direction touching is
  handled correctly via `OnBoundaryOpposite`.
- Cutter spanning the hole of a torus (slicing through both sides of
  the donut): non-manifold stitch input.
- Sphere tessellation at certain segment counts (e.g. n=24 places a
  vertex at exactly cos 60°) hits axis-aligned cutter degeneracies that
  tier-3 jitter doesn't always resolve. Use n=20 / n=32 or a slightly
  off-axis cutter.
- Above-tolerance noise (>1e-9) in imported meshes breaks dedup; cleanly
  fails via `try_*` rather than panicking.

Boolean output is safe: `try_*` variants never panic and return
`BooleanError` on unsupported configurations.

- [x] M1 — Geometry foundations: workspace, primitives (Line, Circle, Ellipse, Plane, Cylinder, Sphere, Cone, Torus).
- [x] M2a — Line intersections: Line vs Line, Plane, Cylinder, Sphere, Cone, Torus + polynomial solvers.
- [x] M2b — Circle intersections (closed-form): Line-Circle, Circle-Circle (coplanar), Circle-Plane, Circle-Sphere. Harder pairs deferred to numerical fallback in M3.
- [x] M3a — Surface intersections (closed-form): Plane-Plane, Plane-Sphere, Sphere-Sphere, Plane-Cylinder. Plane-Cone, Plane-Torus, Cyl-Cyl, etc. deferred to M3b/numerical in M3c.
- [ ] M3b/M3c — Remaining surface pairs + numerical curve-surface and surface-surface fallback.
- [x] M4 — Half-edge topology + core Euler operators (mvfs, mev/kev, mef/kef) + validator. Ring operators (kemr/kfmrh) deferred to M4b.
- [ ] M4b — Ring operators (kemr/mekr, kfmrh/mfkrh) for solids with holes.
- [x] M5 — `kerf-brep::Solid` (topology + geometry) and `box_` primitive constructor.
- [x] M5b — `extrude_polygon(profile, direction)` for arbitrary convex N-gon prisms. Curved primitives (cylinder/sphere/cone/torus) and revolve are still deferred.
- [ ] M5c — Curved primitives + revolve.
- [x] M6a — Face-face intersection: per face-pair, compute clipped 3D segments. Foundation for full booleans.
- [x] M6b — Edge splitting along intersection segment endpoints. Both solids gain vertices at intersection points.
- [x] M6c — Face splitting: add intersection chords as edges via mef. Faces sub-divide where boundaries cross.
- [x] M6d — Face classification: per-face inside/outside/on-boundary against the other solid via centroid ray-casting.
- [x] M6e — Boolean selection + triangulated output. `boolean(a, b, op)` returns a triangle soup. Stitched B-rep output is a follow-up.
- [x] M7 — STL export (ASCII + binary). `boolean(a, b, op)` → `FaceSoup` → `.stl` file.
- [x] M8 — Stitched B-rep boolean output: `boolean_solid(a, b, op)` returns a connected Solid; recursive booleans now work.
- [x] M9 — CSG demo example: extrude → recursive boolean → STL output, validating full kernel.
- [x] M10 — API polish: `box_at(extents, origin)` constructor + `Solid::{union,intersection,difference}` method API.
- [x] M11 — Interior endpoint support: corner-cut Difference now works via mev-tail interior vertex insertion; per-face two-pass algorithm splits boundary endpoints in phase A, grows interior vertices in phase B, splices remaining chords in phase C.
- [x] M12 — Multi-shell stitch: nested-Difference produces hollow Solids with 2 disconnected shells.
- [x] M13 — cylinder primitive (with seam topology) + tessellator. STL output for curved solids.
- [x] M14 — cone primitive (apex + base topology, 2V/2E/2F) + tessellator fan triangulation.
- [x] M15 — sphere primitive (1V/0E/1F, empty-loop topology) + lat-long tessellator with polar triangle fan and equatorial quad strip.
- [x] M16 — torus primitive (1V/2E/1F, genus-1 topology with self-loop edges) + validator extended to accept any non-negative integer genus via Euler-Poincaré; toroidal quad-strip tessellator.
- [x] M17 — frustum primitive (truncated cone, 2V/3E/3F, cylinder topology) with lateral Cone surface; apex computed from seam-line / z-axis intersection. Generalises cylinder (equal radii rejected) and cone (top_r → 0 rejected).
- [x] M18 — Primitive zoo capstone: every primitive (box, prism, cylinder, cone, sphere, torus, frustum) tessellated and emitted to a single STL.
- [x] M19a — Wavefront OBJ output (mesh export with vertex dedup).
- [x] M19b — STEP (ISO 10303-21 / AP214) B-rep export. Preserves exact geometry (planes, cylinders, spheres, cones, tori, lines, circles, ellipses) as native CAD entities. Real CAD interchange.
- [x] M20 — `revolve_polyline(profile)` constructor for axisymmetric solids. Revolves an open polyline in the xz-plane around the z-axis. Profile must have ≥ 3 points with both endpoints on the z-axis and all interior points at x > 0. Produces N vertices, (2N-3) edges, and (N-1) faces — first and last faces are Cone surfaces (apex cones), middle faces are Cylinder (vertical segment) or Cone (tilted segment). Euler V-E+F = 2 verified at construction.
- [x] M21 — Visual gallery: `cad_gallery` example renders 13 STLs covering every primitive + every working boolean. Tessellator fix for `Cone` faces with 2 boundary circles (frustum / revolve middle faces) — previously fan-from-apex which left the lateral surface invisible. Screenshots in `screenshots/`.
- [x] M22a — Real CAD example models: chess pawn via `revolve_polyline` (7-point profile, 32 segments, 320 triangles), stair-step Mayan ziggurat (5 boxes, FaceSoup-merge fallback for face-to-face touching geometry), and L-bracket with chamfered corner (`union` + `difference`). Three new STLs + rendered PNGs in `screenshots/`.
- [x] M22b — Native serde format. `Solid` round-trips through JSON via `serde_io::{write_json, read_json}` with `serde::{Serialize, Deserialize}` derives across geom/topo/brep types and `nalgebra/serde-serialize`. Slotmap parity-invariant gotcha fixed: `solids: SlotMap<SolidId, ()>` → `SlotMap<SolidId, bool>` so occupied slots no longer serialize to `null` (which slotmap's deserializer reads as vacant).
- [x] M22c — Software-rasterized shaded renderer (`scripts/render_shaded.py`): pure-Python/numpy Z-buffer + two-light Lambertian shading replaces matplotlib wireframe. Corner cuts, hollow shells, and curved surfaces (sphere, torus) render with proper occlusion. 17 screenshots re-rendered in ~14 s total via vectorized numpy rasterizer.
- [x] M23 — `cylinder_faceted(r, h, n)` n-gon prism approximation. All-planar topology, so STL is exact and the planar boolean pipeline applies wherever intersection segments meet on shared edges. Probed the curved-boolean limit (`box ∪ cylinder` panics in M11 phase B with interior-only intersection endpoints on the box top face) — full curved face-pair clipping deferred. Resolution-sweep example (n=4 → cube, n=32 → smooth) at `screenshots/cylinder_resolution.png`.
- [x] M24 — Three new revolve_polyline CAD models: bowling pin (8-pt profile, 384 tri), wineglass / goblet (9-pt profile, 672 tri), stepped lighthouse tower (12-pt profile, 11 lateral faces, 640 tri). All-revolve so they sidestep the curved-boolean limit; demonstrate revolve_polyline's expressivity for axisymmetric shapes.
- [x] M25 — Triangle mesh → Solid importer. `read_stl_binary_to_solid` and `from_triangles` build a half-edge B-rep from a flat triangle list via vertex dedup (1 µm grid) + directed-edge twin pairing. Each triangle becomes a `Plane` face. Rejects non-manifold input (boundary edges, T-junctions, duplicate directed edges). Box round-trips through STL bytes losslessly (12 triangles → 8V/18E/12F).
- [x] M26 — Booleans on imported meshes. Imported B-reps feed back into `union`/`intersection`/`difference` exactly like native primitives — every triangle face is a Plane surface. Closes the loop: build → tessellate → STL → import → boolean → STL. Tested for native↔imported and imported↔imported on overlapping boxes.
- [x] M27 — `kerf` CLI binary. `cargo run --bin kerf -- <union|intersection|difference> a.stl b.stl out.stl` runs an end-to-end boolean on any binary STL inputs the importer accepts. Screenshots: `screenshots/cli_union.png`, `screenshots/cli_intersection.png`.
- [x] M28 — Half-overlap difference fix. Two boxes shifted in x panicked in the stitcher because A's coincident-with-B face classified as OnBoundary and was kept, leaving the kept-face graph with an unmatched half-edge. Changed `keep_a_face(OnBoundary, Difference)` from `true` to `false`: the shared boundary is re-emitted via B's flipped face when needed. Adds `half_overlap_difference_yields_sub_box` regression test. CLI screenshot: `screenshots/cli_difference.png`.
- [x] M29 — ASCII STL reader + auto-sniff. `read_ascii` parses the `vertex x y z` lines (recomputing normals on import); `read_stl_auto` sniffs the first 5 bytes and dispatches to ASCII or binary, with a fallback for binary STLs that happen to start with the literal "solid" prefix. CLI uses `read_stl_to_solid` so `kerf union ascii.stl binary.stl out.stl` works regardless of input format.
- [x] M30 — `kerf` installable + pipeable. `cargo install --path crates/kerf-brep --bin kerf` produces a 625 KB release binary. `-` for any path reads stdin / writes stdout, so booleans chain via shell pipes: `kerf union a.stl b.stl - | kerf difference - c.stl out.stl`.
- [x] M31 — Wavefront OBJ importer. `read_obj` parses `v` and `f` lines, triangulates quads / n-gons via fan, supports `i/uv/n` slash-form face tokens and negative (relative) indices, ignores `vt` / `vn` / `g` / materials. `read_obj_to_solid` is the end-to-end importer. Box round-trips through OBJ losslessly (8V/18E/12F).
- [x] M32 — CLI extension-based format detection. `kerf union a.stl b.obj merged.stl` works — input/output formats are picked from the file extension (`.stl`, `.obj`); for stdin, the leading bytes are sniffed for `v ` / `f ` / `solid ` to pick OBJ vs STL.
- [x] M33 — Recoverable boolean errors. `Solid::try_{union,intersection,difference}` and `try_boolean_solid` wrap the boolean pipeline in `catch_unwind` and convert any internal panic (M11 phase B interior limit, non-manifold stitch input, etc.) into a `BooleanError`. CLI uses these so it exits 1 with a diagnostic instead of crashing on unsupported configurations like `cylinder ∪ box`. Original panicking variants kept for invariant testing.
- [x] M34 — Readiness matrix + zero-length stitch fix. `cargo run --example readiness_matrix` runs every (primitive A × primitive B × op) combo via `try_*` and prints a pass/fail table. Baseline 91/168 (54%); fixing stitch's zero-length-edge panic (now skipped instead of `expect`) brought it to 104/168 (62%). Failure buckets and the per-bucket fix roadmap are in `docs/readiness.md`.
- [x] M35 — Stitch robustness: `pick_twin_pair` handles edges with N≥3 half-edges by picking one in each direction (drops extras as coplanar duplicates) instead of panicking. Canonical-cycle face dedup catches same-orientation duplicate kept faces. No matrix improvement yet — the surviving 3-half-edge failures all have three half-edges in the same direction, which is a coplanar-face classification problem upstream of stitch.
- [x] M36 — Stinger edges for orphan interior endpoints. M11 phase B no longer panics when an interior intersection point has no boundary-anchored sibling — it mev's a "stinger" edge from any face boundary vertex to the orphan, leaving a topological fjord (face's outer loop has a spike). 104/168 → 108/168 (62% → 64%); the M11 phase-B family of failures now flows past phase B into the stitch single-half-edge bucket (recoverable). `tests/readiness_floor.rs` asserts ≥ 108/168 to prevent regressions.
- [x] M37 — Argument-swap retry. The boolean classifier is order-dependent for the OnBoundary case — `a.try_union(&b)` can fail where `b.try_union(&a)` succeeds. `try_*` now retries with arguments swapped on failure for commutative ops. 108/168 → 117/168 (64% → 70%), +9 cases unblocked. Floor bumped to 117.
- [x] M38a — Centroid dedup before averaging. M36's stinger fjords add duplicate spike-anchor vertices to the face polygon, which biases polygon-average centroids enough that the OUTER face of a cyl-pierced box top mis-classifies as Inside (centroid lands in the hole). `face_centroid` now dedups vertices before averaging. 117/168 → 119/168 (70% → 71%). Chord-merge module integrated (same-source + Inside-only restrictions) but doesn't unblock new cases yet — `cyl ∪ box` still fails downstream in stitch.

## Visual gallery

| Capability | Render |
| --- | --- |
| Primitive zoo (M18) | `screenshots/00_primitive_zoo.png` |
| Box (M5) | `screenshots/01_box.png` |
| Triangular prism via extrude (M5b) | `screenshots/02_prism.png` |
| Cylinder (M13) | `screenshots/03_cylinder.png` |
| Cone (M14) | `screenshots/04_cone.png` |
| Sphere (M15) | `screenshots/05_sphere.png` |
| Torus (M16, genus-1) | `screenshots/06_torus.png` |
| Frustum (M17) | `screenshots/07_frustum.png` |
| Revolve polyline — vase (M20) | `screenshots/08_revolve_vase.png` |
| Union of overlapping boxes | `screenshots/10_union_two_boxes.png` |
| Intersection of overlapping boxes | `screenshots/11_intersection_two_boxes.png` |
| Corner-cut Difference (M11) | `screenshots/12_corner_cut_step.png` |
| Hollow nested Difference (M12) | `screenshots/13_hollow_box.png` |
| Recursive boolean (M8) | `screenshots/14_recursive_intersection.png` |
| Chess pawn — revolve_polyline 7-pt profile (M22a) | `screenshots/cad_pawn.png` |
| Stair-step pyramid — 5-layer ziggurat (M22a) | `screenshots/cad_pyramid.png` |
| L-bracket — union + chamfer difference (M22a) | `screenshots/cad_bracket.png` |
| cylinder_faceted resolution sweep (M23) | `screenshots/cylinder_resolution.png` |
| Bowling pin (M24) | `screenshots/cad_bowling_pin.png` |
| Wineglass / goblet (M24) | `screenshots/cad_goblet.png` |
| Lighthouse tower (M24) | `screenshots/cad_lighthouse.png` |

Build the STL files with `cargo run --example cad_gallery`. Render to PNG with `scripts/render_captioned.py` (requires `numpy-stl` + `matplotlib` + `numpy`). Uses the Z-buffered shaded renderer (`scripts/render_shaded.py`, M22c).

## Crates

- `kerf-geom` — exact analytic curves and surfaces, intersection routines.
- `kerf-topo` — half-edge topology and Euler operators.
- `kerf-brep` — the kernel (geometry + topology + booleans + constructors).

## Library usage

```rust
use kerf_brep::primitives::{box_, box_at};
use kerf_brep::{solid_volume, Solid};
use kerf_geom::{Point3, Vec3};

// Build two overlapping boxes.
let a = box_(Vec3::new(2.0, 2.0, 2.0));
let b = box_at(Vec3::new(2.0, 2.0, 2.0), Point3::new(1.0, 0.0, 0.0));

// Boolean ops via the Solid methods (panics on unsupported configs)…
let union = a.union(&b);
let intersect = a.intersection(&b);
let diff = a.difference(&b);

// …or via try_* (returns BooleanError instead).
let r = a.try_union(&b)?;
println!("union vol = {:.4}", solid_volume(&r));   // 12.0000

// Validate topology.
kerf_topo::validate(&r.topo)?;

// Per-shell volume (positive for outer, negative for cavity shells).
for sh in r.topo.shell_ids() {
    let v = kerf_brep::shell_volume(&r, sh);
    println!("shell {sh:?} vol={v:.4}");
}

// Tessellate to triangle soup, then write STL.
use std::fs::File;
let soup = kerf_brep::tessellate(&r, 24);
let mut f = File::create("union.stl")?;
kerf_brep::write_binary(&soup, "union", &mut f)?;
# Ok::<_, Box<dyn std::error::Error>>(())
```

## Recursive booleans

```rust
use kerf_brep::primitives::{box_, box_at};
use kerf_geom::{Point3, Vec3};

let plate = box_(Vec3::new(20.0, 20.0, 5.0));
let hole_a = box_at(Vec3::new(2.0, 2.0, 5.0), Point3::new(2.0, 2.0, 0.0));
let hole_b = box_at(Vec3::new(2.0, 2.0, 5.0), Point3::new(15.0, 15.0, 0.0));

// Each call returns a new Solid; chain freely.
let result = plate.try_difference(&hole_a)?
    .try_difference(&hole_b)?;
# Ok::<_, kerf_brep::BooleanError>(())
```

## Importing external STL

```rust
use std::fs::File;
use kerf_brep::{read_stl_to_solid, solid_volume};

let mut f = File::open("part.stl")?;
let imported = read_stl_to_solid(&mut f)?;
println!("imported {} vol={:.4}", imported.face_count(), solid_volume(&imported));

// Booleans against imported solids work the same as primitives.
let cutter = kerf_brep::primitives::box_(kerf_geom::Vec3::new(1.0, 1.0, 5.0));
let carved = imported.try_difference(&cutter)?;
# Ok::<_, Box<dyn std::error::Error>>(())
```

## CLI

```bash
cargo install --path crates/kerf-brep --bin kerf
kerf union a.stl b.stl out.stl
kerf intersection a.stl b.stl out.stl
kerf difference a.stl b.stl out.stl
```

Inputs may be ASCII or binary STL; output is always binary STL. Use `-` for any
path to read stdin or write stdout, so ops can chain through pipes:

```bash
kerf union a.stl b.stl - | kerf difference - c.stl out.stl
cat scan.stl | kerf intersection - cutter.stl out.stl
```
