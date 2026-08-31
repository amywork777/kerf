# kerf-cad weekly scorecard

| Metric | Value |
|---|---|
| Last updated | 2026-08-31 |
| Tests passing (main) | 332 / 0 failed / 4 ignored |
| Readiness matrix | 168 / 168 (100%) prim × prim × op |
| Last milestone | `solid_volume(torus(...))` analytic fix — returns 2π²·R·r² via Pappus |

## Primitive inventory (on main)

| Primitive | Surface type | Boolean-safe |
|---|---|---|
| `box_` | planar | full |
| `extrude_polygon` | planar | full |
| `cylinder_faceted` | planar | full |
| `frustum` | analytic | partial (try_*) |
| `cylinder` | analytic (Cylinder) | partial (try_*) |
| `cone` | analytic | partial (try_*) |
| `sphere` | analytic | partial (try_*) |
| `torus` | analytic | partial (try_*) |
| `revolve_polyline` | analytic (Cone/Cylinder) | partial (try_*) |

Note: `sphere_faceted`, `torus_faceted`, `capsule_faceted`, `cone_faceted`, and
`frustum_faceted` exist in open PRs (#90, #158–162) but have not yet been merged to main.

## Open PR inventory (unmerged, all target main @ 6a45def)

| PR | Title | Status |
|---|---|---|
| #162 | sphere_faceted + solid_volume analytic sphere fix | open |
| #161 | cone_faceted | open |
| #160 | sphere_faceted + torus_faceted | open |
| #159 | capsule_faceted | open |
| #158 | frustum_faceted + readiness 168→216 | open |
| #90  | sphere_faceted + capsule_faceted + torus_faceted (older) | open |

## solid_volume analytic fallbacks

| Surface | Fix | Formula |
|---|---|---|
| `sphere` | ✅ in PR #162 | (4/3)πr³ |
| `torus` | ✅ this PR | 2π²·R·r² (Pappus) |
| `cylinder` | not needed (has half-edges at distinct vertices) | — |
| `cone` | not needed | — |

## Additions this run (2026-08-31)

**`solid_volume(torus(...))` analytic fix** (+3 tests, 329 → 332 passing):

The analytic torus (1V, 2E, 1F — genus-1) has all 4 half-edges in its outer
loop originating from the same anchor vertex. The divergence-theorem
walk collects 4 identical points → zero area → returns 0.

Fix: detect `SurfaceKind::Torus` in `face_signed_volume` before the polygon
walk and return `2π²·R·r²` (Pappus's centroid theorem).

New tests:
- `analytic_torus_volume_uses_pappus_formula` — exact match to 1e-9
- `analytic_torus_volume_scales_correctly` — doubling r → 4× volume (V ∝ r²)
- `analytic_torus_volume_scales_with_major_radius` — doubling R → 2× volume (V ∝ R)

## What didn't ship / honest assessment

- sphere_faceted was duplicated; dropped. PR #162 already has it with 9 tests.
- Main branch is stale (still at `6a45def`). Six open PRs have been building on it
  without merging. The real bottleneck is PR review/merge, not feature development.

## Gap list (achievable in a future weekly agent run)

- `solid_volume(cylinder(...))` sanity test — verify it returns πr²h correctly
- `solid_volume(cone(...))` sanity test — verify it returns (1/3)πr²h
- More jitter directions in boolean retry (currently only axis nudges)
- WASM API improvements (face indices, owner tags, half-edge traversal)
- Additional edge-case boolean tests (chains of 5+ ops, extreme aspect ratios)
- Investigate / rebase the oldest stale PRs (#90, #158) onto current main

## Recommended next-week target

**Rebase and consolidate the open PRs**: PRs #90, #158, #159, #160, #161, #162 all
target the same stale base. If one is merged, the others need rebasing. A short agent
run could: (1) fetch each branch, (2) check if tests still pass on the branch tip,
(3) note which ones conflict and leave a PR comment. That's a genuine unblocking action
vs. adding more unmerged features.

Alternatively: **More jitter directions in boolean retry** — the boolean retry (`try_*`)
currently nudges by axis-aligned deltas. Adding diagonal jitter directions (±δ on all 3
axes) could unblock the cases in the 0/168 failure set without touching any test infra.
