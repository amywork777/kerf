# Kerf STATUS

_Last updated: 2026-06-22_

## Boolean readiness matrix

168/168 (100%) — all (primitive A × primitive B × op) combinations pass via `try_*`.

Run `cargo run --example readiness_matrix -p kerf-brep` to refresh.

## Test counts

| Suite | Tests | Notes |
|---|---|---|
| `kerf-brep` lib (unit) | 140 | primitives, booleans, serde, STL/OBJ/STEP, mesh import |
| `readiness_floor.rs` | 1 | asserts ≥ 168/168 |
| `readiness_geometry.rs` | 13 | volume invariants + Euler-Poincaré |
| `readiness_algebra.rs` | 9 | idempotence, commutativity, set conservation |
| `readiness_quality.rs` | 2 | bbox containment + no-zero-area-triangle |
| `readiness_roundtrip.rs` | 7 | STL + OBJ + JSON round-trips |
| `readiness_robustness.rs` | 9 | recursive booleans, determinism, translation invariance |
| `readiness_robustness2.rs` | 12 | rotation invariance, scale extremes, coplanar-touching, fuzz |
| `readiness_stress.rs` | 22 | tessellate→STL→import→boolean chains |
| `readiness_curved.rs` | 10 | sphere/torus/cone via tessellation+re-import |
| `readiness_noisy.rs` | 6 | sub-tolerance noise preserves correctness |
| `readiness_complex_cad.rs` | 1 (+ 2 ignored) | multi-step CAD models |
| `kerf-geom` + `kerf-topo` lib | 92 + 14 + 1 | geometry/topo unit tests |
| `proptest_euler.rs` | 1 | MEV chain invariant |
| **Total passing** | **342** | (+ 4 ignored) |

## Primitives inventory

| Primitive | Kind | Notes |
|---|---|---|
| `box_` | Planar | 6-face cuboid |
| `box_at` | Planar | translated cuboid |
| `extrude_polygon` | Planar | arbitrary N-gon prism |
| `cylinder_faceted` | Planar | n-gon prism ≈ cylinder |
| `sphere_faceted` | Planar (new M41) | UV sphere, all-triangle, from_triangles |
| `torus_faceted` | Planar (new M41) | n×m quad grid, genus-1, from_triangles |
| `cylinder` | Curved | analytic Cylinder surface |
| `cone` | Curved | analytic Cone surface |
| `sphere` | Curved | analytic Sphere surface (1V/0E/1F) |
| `torus` | Curved | analytic Torus surface (genus-1) |
| `frustum` | Curved | truncated cone |
| `revolve_polyline` | Curved | axisymmetric solid of revolution |

## Known gaps

- Curved-surface booleans: `box ∪ cylinder` panics in M11 phase-B (interior-only intersection endpoints).
- 2D sketcher / parametric constraints: not started.
- Assembly & mates: not started.
- Shell/offset surfaces: not started.
- WASM API surface: not started.

## Next recommended target

**Improve bonded boolean retry jitter** (add more jitter axis directions to `try_*`) — addresses the class of boolean failures that succeed after small perturbation. Estimated 1–2 h.

OR

**`capsule_faceted(r, h, n)` primitive** (cylinder + two hemispherical caps, all-planar) — natural follow-on to M41. Estimated 1 h.
