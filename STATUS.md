# Kerf B-rep Kernel — Progress Status

## Scorecard (updated 2026-05-11)

| Metric | Value |
|---|---|
| Readiness matrix | 119 / 168 (71%) |
| Total tests passing | 342 |
| Ignored (known limitations) | 4 |
| Primitives | 12 (box, box_at, cone, cylinder, cylinder_faceted, capsule_faceted, extrude_polygon, frustum, revolve_polyline, sphere, sphere_faceted, torus) |

## Primitives

| Primitive | Type | Boolean-pipeline ready |
|---|---|---|
| `box_` / `box_at` | Planar | ✅ full |
| `extrude_polygon` | Planar prism | ✅ full |
| `cylinder_faceted(r, h, n)` | Polyhedral prism | ✅ works when edges don't pierce faces |
| `sphere_faceted(r, n)` | Polyhedral UV sphere | ✅ full (all-planar) |
| `capsule_faceted(r, h, n)` | Polyhedral capsule | ✅ full (all-planar) |
| `cylinder(r, h)` | Analytic curved | ❌ curved boolean path incomplete |
| `cone(r, h)` | Analytic curved | ❌ curved boolean path incomplete |
| `sphere(r)` | Analytic curved | ❌ curved boolean path incomplete |
| `torus(R, r)` | Analytic curved | ❌ curved boolean path incomplete |
| `frustum(r_bot, r_top, h)` | Analytic curved | ❌ curved boolean path incomplete |
| `revolve_polyline(profile)` | Analytic curved | ❌ curved boolean path incomplete |

## Gap list (ordered by achievability)

### Quick wins (1–2 hours per run)
- [ ] **More faceted primitives** — `cylinder_faceted_capped` (hemisphere caps), `ellipsoid_faceted`, `torus_faceted`
- [ ] **Boolean jitter expansion** — add ±x, ±y, ±z offset variants to the tier-3 retry to push readiness past 71%
- [ ] **`solid_volume` mesh fallback** — for analytic-only solids (sphere, torus), tessellate + sum to get an approximate volume
- [ ] **Face-index API on Solid** — expose face normals / centroids for downstream use
- [ ] **More integration test coverage** — exercise `capsule_faceted` and `sphere_faceted` in `readiness_curved.rs`-style stress tests

### Medium (1–2 weeks, human-driven)
- [ ] **Curved face-pair clipping** — handle Cylinder vs Plane, Sphere vs Plane intersections in the boolean pipeline directly (removes the tessellate-then-import workaround)
- [ ] **M4b ring operators** — `kemr`/`mekr`, `kfmrh`/`mfkrh` for solids with holes
- [ ] **M3b/M3c surface pairs** — Plane-Cone, Plane-Torus, Cylinder-Cylinder, etc.

### Long-term (months)
- [ ] 2D sketcher UI
- [ ] Constraint solver
- [ ] Assembly + mates
- [ ] Shell offset surfaces
- [ ] Multi-edge fillet
- [ ] Full curved-surface analytic booleans

## Known limitations (ignored tests)

| Test | Limitation |
|---|---|
| `bracket_with_slot_and_holes_imports_and_diffs` | Chained DIFF on 3+-shell intermediate |
| `stepped_plate_carves_correctly` | Chained DIFF on 2-shell intermediate with adjacent cutters |
| `imported_torus_diff_box_through_ring_works` | Cutter spanning torus hole — non-manifold stitch |
| `above_tolerance_noise_breaks_boolean_diff` | Noise >1e-9 breaks vertex dedup |

## Recommended next-week target

**Add `readiness_faceted.rs` integration test suite** — wire `sphere_faceted` and `capsule_faceted` into a curved-primitive-style test suite (volume convergence, diff, intersection, union with box, STL roundtrip). Then attempt `torus_faceted` to give a genus-1 polyhedral primitive that works directly with the boolean pipeline without tessellate-then-import.
