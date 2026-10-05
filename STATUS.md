# kerf STATUS

## Current scorecard

| Dimension | Value |
|-----------|-------|
| Matrix coverage | 168/168 (100%) |
| Tests passing | 336 / 0 failing / 4 ignored |
| Test suites | 11 integration + lib unit + doctests |
| Tessellator bug fix | `find_apex_in_face` now handles apex-at-z=0 (revolve_polyline) |

## Milestones shipped

See README.md for the full milestone list (M1 – M26).

| # | Short description |
|---|-------------------|
| M26 | `capsule_faceted(r, h, m)` primitive via `revolve_polyline` |
| M26 | Tessellator fix: apex-below-circle winding reversal for cone faces |

## Test inventory

| Suite | Count | Notes |
|-------|-------|-------|
| `lib` (unit) | 134 | includes 7 new capsule tests |
| `readiness_floor` | 1 | ≥168/168 matrix floor |
| `readiness_geometry` | 13 | exact volumes + Euler-Poincaré |
| `readiness_algebra` | 9 | idempotence, commutativity, conservation |
| `readiness_quality` | 2 | bbox + no-zero-area triangle |
| `readiness_roundtrip` | 7 | STL + OBJ + JSON serde |
| `readiness_robustness` | 9 | recursive, translated, deterministic |
| `readiness_robustness2` | 12 | scale extremes, coplanar, fuzz (50 pairs) |
| `readiness_stress` | 22 | tessellate → STL → import → boolean |
| `readiness_curved` | 10 (1 ignored) | sphere, torus, cone via import |
| `readiness_noisy` | 6 (1 ignored) | sub-tolerance noise |
| `readiness_complex_cad` | 1 (2 ignored) | multi-step CAD booleans |
| `kerf_geom` unit | 92 | intersection math |
| `kerf_topo` unit + proptest | 18 | Euler operators |
| Doc-tests | 1 | serde_io example |

## Known gaps (next targets)

- **Tessellator**: the `apex_below_circle` heuristic in `tessellate.rs` uses a z-comparison; only robust when the solid is axis-aligned (all current revolve_polyline use cases). A production fix would read the Cone surface normal.
- **capsule_faceted in `lib.rs`**: `capsule_faceted` is accessible via `kerf_brep::primitives::capsule_faceted` but not as a crate-level re-export. Add when cleaning up the public API.
- Remaining ignored tests (curved torus hole, chained multi-shell DIFF, above-tolerance noise).
- M27+: see README.md milestone list.

## Recommended next-week target

1. Add `capsule_faceted` to the `lib.rs` top-level re-exports and the gallery example.
2. Add 5-10 more integration tests for capsule_faceted booleans (capsule ∩ box, capsule − box via tessellate+import).
3. Consider adding a `ring_faceted(major_r, minor_r, n, m)` primitive (faceted torus via revolve_polyline with a circular arc profile).
