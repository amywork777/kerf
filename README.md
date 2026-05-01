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
- [ ] M5b — Remaining primitives (cylinder, sphere, cone, torus) + extrude/revolve.
- [x] M6a — Face-face intersection: per face-pair, compute clipped 3D segments. Foundation for full booleans.
- [x] M6b — Edge splitting along intersection segment endpoints. Both solids gain vertices at intersection points.
- [x] M6c — Face splitting: add intersection chords as edges via mef. Faces sub-divide where boundaries cross.
- [ ] M6d — Inside/outside classification.
- [ ] M6e — Stitching (assemble result solid).
- [ ] M7 — Tessellation and STL.

## Crates

- `kerf-geom` — exact analytic curves and surfaces, intersection routines.
- `kerf-topo` — half-edge topology and Euler operators.
- `kerf-brep` — the kernel (geometry + topology + booleans + constructors).
