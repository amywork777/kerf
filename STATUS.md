# Kerf B-rep Kernel — Status Scorecard

## Test counts (as of 2026-07-27)

| Suite | Passing | Failing | Ignored |
|---|---|---|---|
| kerf-brep lib (unit tests) | 135 | 0 | 0 |
| readiness_floor | 1 | 0 | 0 |
| readiness_geometry | 13 | 0 | 0 |
| readiness_algebra | 9 | 0 | 0 |
| readiness_quality | 2 | 0 | 0 |
| readiness_roundtrip | 7 | 0 | 0 |
| readiness_robustness | 9 | 0 | 0 |
| readiness_robustness2 | 12 | 0 | 0 |
| readiness_stress | 22 | 0 | 0 |
| readiness_curved | 10 | 0 | 1 |
| readiness_noisy | 6 | 0 | 1 |
| readiness_complex_cad | 1 | 0 | 2 |
| kerf-topo lib | 92 | 0 | 0 |
| kerf-topo integration | 14 | 0 | 0 |
| proptest_euler | 1 | 0 | 0 |
| kerf-geom lib | 7 | 0 | 0 |
| **TOTAL** | **337** | **0** | **4** |

**Boolean readiness: 168/168 (100%)** of primitive × primitive × op combinations.

## Primitives

| Primitive | Topology | Surfaces |
|---|---|---|
| box_ | exact | Plane ×6 |
| extrude_polygon | exact | Plane ×(n+2) |
| cylinder | analytic (1V/3E/3F seam) | Cylinder + Plane ×2 |
| cone | analytic (2V/2E/2F) | Cone + Plane |
| sphere | analytic (1V/0E/1F) | Sphere |
| torus | analytic (1V/2E/1F genus-1) | Torus |
| frustum | analytic | Cone + Plane ×2 |
| revolve_polyline | analytic | Cone/Cylinder + Plane ×2 |
| cylinder_faceted | all-planar (2n V / 3n E / n+2 F) | Plane ×(n+2) |
| **cone_faceted** | **all-planar (n+1 V / 2n E / n+1 F)** | **Plane ×(n+1)** |

## Milestone history (cumulative)

- M1–M10: geometry + topology + box + booleans (FaceSoup).
- M11: interior endpoint support (corner-cut Difference).
- M12: multi-shell stitch (hollow Solid).
- M13–M17: analytic curved primitives (cylinder, cone, sphere, torus, frustum).
- M18: primitive zoo + STL output.
- M19: OBJ + STEP export.
- M20: revolve_polyline.
- M21: visual gallery.
- M22: real CAD models + JSON serde + software renderer.
- M23: cylinder_faceted (all-planar prism).
- M24: three revolve_polyline CAD models.
- M25: triangle mesh importer.
- M26: booleans on imported meshes.
- M27: kerf CLI.
- M28: half-overlap Difference fix.
- M29: ASCII STL reader.
- M30: pipeable CLI.
- M31: OBJ importer.
- M32: extension-based format detection.
- M33: recoverable boolean errors (try_* API).
- M34: readiness matrix + zero-length stitch fix (91/168 → 104/168).
- M35: stitch robustness (pick_twin_pair N≥3).
- M36: stinger edges for orphan interior endpoints (104 → 108/168).
- M37: argument-swap retry (108 → 117/168).
- M38a: centroid dedup (117 → 119/168).
- M38b: chord-merge with ancestor gate.
- M39: skip interior chords optimization.
- M40: 100% readiness (168/168) via L-polygon centroid fix + stress/curved suites.
- **M41**: `cone_faceted(r, h, n)` all-planar n-gon pyramid. 8 new tests (topology,
  volume-vs-analytic for n=3/6/32, planar/linear geometry check, JSON round-trip).
  Test count 329 → 337.

## Known gaps (requires significant human-driven work)

- Curved surface face-pair intersection (M3b/M3c) — remaining surface pairs.
- Ring Euler operators (kemr/mekr, kfmrh/mfkrh) for through-holes (M4b).
- Curved primitive boolean pipeline (cyl/sphere/cone lateral faces against box top).
- 2D sketcher UI, constraint solver, assembly+mates, Shell offset, multi-edge fillet.

## Recommended next weekly target

1. **`sphere_faceted(r, n_lat, n_lon)`** — all-planar lat-lon sphere approximation
   (same pattern as cone_faceted). Closes the faceted primitive gap.
2. **More boolean jitter directions** — add ±45° diagonal probes to the retry
   strategy in `solid.rs` for better coverage on axis-aligned degenerate inputs.
3. **Additional edge-case tests** — revolve_polyline with high segment counts,
   deep recursive boolean chains.
