# Kerf B-rep Kernel — Status

_Last updated: 2026-09-21 (automated weekly run)_

## Test baseline

| Metric | Value |
|--------|-------|
| Total tests (cargo test --workspace) | **340** |
| Failing | 0 |
| Ignored | 4 |
| Last milestone | M38b |

## Primitives

| Primitive | File | Notes |
|-----------|------|-------|
| `box_` / `box_at` | primitives/box_.rs | AABB, planar |
| `cone` | primitives/cone.rs | analytic Cone surface |
| `cylinder` | primitives/cylinder.rs | analytic Cylinder surface + seam edge |
| `cylinder_faceted` | primitives/cylinder_faceted.rs | n-gon prism, all-planar |
| `extrude_polygon` | primitives/extrude.rs | arbitrary convex N-gon prism |
| `frustum` | primitives/frustum.rs | truncated cone (2V/3E/3F) |
| `revolve_polyline` | primitives/revolve.rs | axisymmetric from xz profile |
| `sphere` | primitives/sphere.rs | analytic Sphere surface (1V/0E/1F) |
| `sphere_faceted` | primitives/sphere_faceted.rs | UV-mesh sphere, boolean-safe (**M38b**) |
| `torus` | primitives/torus.rs | analytic Torus, genus-1 |
| `capsule_faceted` | primitives/capsule_faceted.rs | cylinder + hemispherical caps (**M38b**) |

## Boolean readiness (168-case matrix)

119/168 (71%) — see README and `docs/readiness.md` for per-bucket breakdown.

## Known gaps (not achievable in a weekly agent run)

- 2D sketcher UI / constraint solver (backward)
- Assembly + mates
- Shell offset surfaces
- Multi-edge fillet
- Full curved-surface analytic booleans (Sphere↔Cylinder, Cylinder↔Torus, etc.)
- WASM API surface (face indices, owner tags)

## Recommended next-week target

**Add 5–10 more edge-case tests to the readiness suites:** rotation-invariance for
`sphere_faceted`, boolean of `capsule_faceted ∩ box_`, and stress tests for
high-resolution capsules (n=16×32). These require no architectural changes and
directly exercise M38b under the boolean pipeline.

Alternatively: **improve solid_volume to fall back to mesh-based computation
for analytic-sphere-only solids** — the sphere primitive currently returns
volume=0 because the analytic-sphere face has no polygon vertices. Fixing this
would make `solid_volume(sphere(r))` return the expected `(4/3)πr³`.
