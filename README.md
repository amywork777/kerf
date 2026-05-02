# kerf

A minimal-but-real B-rep solid modeling kernel in Rust.

See `docs/superpowers/specs/2026-04-28-kerf-brep-kernel-design.md` (in the parent
`code/` workspace) for the design.

## Status

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
- [x] M22c — Software-rasterized shaded renderer (`scripts/render_shaded.py`): pure-Python/numpy Z-buffer + two-light Lambertian shading replaces matplotlib wireframe. Corner cuts, hollow shells, and curved surfaces (sphere, torus) render with proper occlusion. 17 screenshots re-rendered in ~14 s total via vectorized numpy rasterizer.

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

Build the STL files with `cargo run --example cad_gallery`. Render to PNG with `scripts/render_captioned.py` (requires `numpy-stl` + `matplotlib` + `numpy`). Uses the Z-buffered shaded renderer (`scripts/render_shaded.py`, M22c).

## Crates

- `kerf-geom` — exact analytic curves and surfaces, intersection routines.
- `kerf-topo` — half-edge topology and Euler operators.
- `kerf-brep` — the kernel (geometry + topology + booleans + constructors).
