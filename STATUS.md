# Kerf CAD Kernel — Progress Scorecard

## Current Status (2026-06-29)

| Metric | Value |
|---|---|
| Test suite | **352 passed / 0 failed / 9 ignored** |
| SW-tier readiness | ~65% (est.) |
| Primitives | 10 (box, box_at, cylinder, cylinder_faceted, capsule_faceted, cone, sphere, torus, frustum, extrude_polygon, revolve_polyline) |

## Primitive Library

| Primitive | Kind | Volume check | JSON roundtrip | Boolean test |
|---|---|---|---|---|
| `box_` / `box_at` | Planar | ✓ | ✓ | ✓ |
| `cylinder` | Analytic | ✓ | ✓ | ✓ |
| `cylinder_faceted` | Planar | — | — | ✓ |
| `cone` | Analytic | ✓ | — | ✓ |
| `sphere` | Analytic | ✓ | — | ✓ |
| `torus` | Analytic | ✓ | — | ✓ |
| `frustum` | Analytic | — | — | — |
| `extrude_polygon` | Planar | ✓ | — | ✓ |
| `revolve_polyline` | Analytic | — | — | — |
| `capsule_faceted` | Planar | ✓ ✓ STL | ✓ | ✓ |

## Known Limitations (Ignored Tests)

| Test | Status |
|---|---|
| `imported_torus_diff_box_through_ring_works` | Cutter spanning torus hole hits non-manifold stitch |
| `bracket_with_slot_and_holes_imports_and_diffs` | Chained DIFF on 3+ shell intermediate hits kernel limit |
| `stepped_plate_carves_correctly` | Chained DIFF on 2-shell intermediate with adjacent cutters |
| `above_tolerance_noise_breaks_boolean_diff` | Documented noise tolerance limitation |

## Architecture

- **kerf-topo**: Half-edge B-rep topology (MEV/MEF Euler operators)
- **kerf-geom**: Analytic geometry (curves, surfaces, intersections)
- **kerf-brep**: Solid constructors + boolean pipeline (classify → split → select → stitch)

## Week-by-Week Progress

| Date | Work |
|---|---|
| 2026-06-29 | Upgraded `capsule_faceted` to explicit `(r, h, n, m)` API; added `capsule_volume_analytic`; 23 tests total (10 unit + 13 integration): topology, Euler, volume vs analytic, STL round-trip, JSON round-trip, boolean interop |
| 2026-06-15 | Added `capsule_faceted` primitive (pill-shaped solid); 8 new tests |
| prior | B-rep kernel, boolean pipeline, curved-primitive stress suite, robustness suite, real-mesh import |

## Recommended Next Targets

**Good (1–2 hour agent runs):**
- Add `sphere_faceted(r, n)` via `from_triangles` (UV sphere mesh) — enables fast faceted booleans without tessellation roundtrip
- Add `capsule_faceted` to the `readiness_geometry.rs` named-inputs matrix (adds ~21 more boolean cases)
- Add `revolve_polyline` volume-against-analytic test
- Improve `solid_volume` fallback for sphere-only solids
- Fill in missing columns for `cylinder_faceted`, `frustum`, `revolve_polyline` in the primitive table

**Avoid (months of human-driven work):**
- 2D sketcher UI
- Constraint solver
- Assembly + mates
- Shell offset surfaces
- Multi-edge fillet
- Full curved-surface analytic booleans
