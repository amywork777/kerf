# kerf-brep STATUS

_Updated: 2026-10-12 (weekly automated run)_

## Scorecard

| Metric | Value |
|---|---|
| Readiness matrix | **168 / 168 (100%)** primitive × primitive × op |
| Total tests | **338 passing, 0 failing** |
| Lib unit tests (kerf-brep) | 138 |
| Integration test suites | readiness_floor / geometry / algebra / quality / roundtrip / robustness / robustness2 / stress / curved / noisy / complex\_cad |
| Ignored tests | 4 (flaky curved configs documented in README) |
| Primitives | box, extrude\_polygon, cylinder, cone, sphere, torus, frustum, revolve\_polyline, cylinder\_faceted, **sphere\_faceted** (new M38b) |
| Boolean ops | union, intersection, difference |
| Boolean API | `try_*` never panics; returns `BooleanError` on unsupported config |

## Milestone tracking

| Milestone | Description | Status |
|---|---|---|
| M1–M22 | Geometry, topology, Euler ops, primitives, STEP/STL/OBJ, revolve, serde | ✅ done |
| M23 | `cylinder_faceted` all-planar prism primitive | ✅ done |
| M24 | Three revolve CAD models | ✅ done |
| M25–M26 | Mesh importer + boolean on imported meshes | ✅ done |
| M27–M32 | CLI, formats, OBJ import | ✅ done |
| M33 | Recoverable `try_*` boolean API | ✅ done |
| M34–M37 | Readiness matrix 54% → 70% via stitch robustness | ✅ done |
| M38a | Centroid dedup 70% → 71% | ✅ done |
| **M38b** | `sphere_faceted` — all-planar lat-lon sphere, 7 unit + 4 boolean tests | ✅ **just shipped** |

## Gap list (recommended next targets)

These are achievable in ~1–2 hours each, ordered roughly by value:

1. **`torus_faceted(r_major, r_minor, n)`** — all-planar donut using lat-lon quads.
   Same pattern as sphere_faceted. Would give a genus-1 planar primitive usable
   in the boolean pipeline without tessellation round-trips.

2. **`capsule_faceted(r, h, n)`** — cylinder-with-hemi-sphere caps. Two half-sphere
   lat-lon caps + n-gon barrel. Useful for medical / robotics models.

3. **Add sphere_faceted to the readiness matrix floor** — the 4 boolean tests added
   this week pass, but the full 9×8×3 = 216 matrix sweep isn't tracked yet.
   Running readiness_matrix with sphere_faceted as an input and raising the floor
   from 168 to ≥ 200 would be honest progress.

4. **Improve volume fallback for curved primitives** — sphere/torus/cone/frustum
   volumes via shell_volume are correct only after tessellation. A direct analytic
   fallback using the surface geometry would let `solid_volume` skip tessellation.

5. **JSON round-trip test for sphere_faceted in readiness_roundtrip.rs** — the
   in-source test covers it, but adding an integration-level check alongside
   cylinder_faceted in `readiness_roundtrip.rs` would complete that suite.

## What didn't move

- The 168/168 ceiling is unchanged — sphere_faceted is not yet wired into the
  matrix, so the floor stays at 168.
- Curved-face analytic booleans remain unsupported (sphere, torus, cone primitives
  hit `try_*` errors on non-box opponents).
- STATUS.md didn't exist before this run; created now.
