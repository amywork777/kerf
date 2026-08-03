# kerf-cad weekly scorecard

| Metric | Value |
|---|---|
| Last updated | 2026-08-03 |
| Tests passing | 335 / 0 failed / 4 ignored |
| Readiness matrix | 168 / 168 (100%) prim × prim × op |
| Last milestone | M38b — `sphere_faceted` faceted sphere primitive |

## Primitive inventory

| Primitive | Type | Boolean-safe |
|---|---|---|
| `box_` | planar | full |
| `extrude_polygon` | planar | full |
| `cylinder_faceted` | planar | full |
| `sphere_faceted` (**new M38b**) | planar (triangulated) | full |
| `frustum` | analytic | partial (try_*) |
| `cylinder` | analytic (Cylinder surface) | partial (try_*) |
| `cone` | analytic | partial (try_*) |
| `sphere` | analytic | partial (try_*) |
| `torus` | analytic | partial (try_*) |
| `revolve_polyline` | analytic (Cone/Cylinder) | partial (try_*) |

"Full boolean-safe" = all three ops (union/intersection/difference) succeed on all axis-aligned overlapping configurations via the planar pipeline.

## Gap list (NOT achievable in a weekly agent run)

- **2D sketcher UI** — months of human-driven product work
- **Constraint solver (backward)** — research-level problem
- **Assembly + mates** — multi-week design + implementation
- **Shell offset surfaces** — requires parallel transport on NURBS
- **Multi-edge fillet** — requires edge-blend surface math
- **Full curved-surface analytic booleans** — face-pair clipping for Cylinder/Sphere/Cone/Torus pairs needed upstream of M11

## Gap list (achievable in a future weekly agent run)

- `torus_faceted(r_major, r_minor, n_major, n_minor)` — planar torus via lat-long grid, similar to sphere_faceted
- `capsule_faceted(r, h, n)` — cylinder with hemispherical end caps, useful for physics
- More jitter directions in boolean retry (currently: x, y, z axis nudges)
- `solid_volume` mesh-based fallback for analytic-only primitives (sphere, torus)
- WASM API surface improvements (face indices, owner tags)
- Additional edge-case tests (boolean chains of 5+ ops, extreme aspect ratios)

## Recommended next-week target

**`torus_faceted(r_maj, r_min, n)`** — all-planar toroidal lat-long mesh. Natural extension of M38b, same `from_triangles` pattern. Volume against analytic π²·r_maj·r_min² as test.
