# kerf

A minimal-but-real B-rep solid modeling kernel in Rust.

See `docs/superpowers/specs/2026-04-28-kerf-brep-kernel-design.md` (in the parent
`code/` workspace) for the design.

## Status

- [x] M1 — Geometry foundations: workspace, primitives (Line, Circle, Ellipse, Plane, Cylinder, Sphere, Cone, Torus).
- [ ] M2 — Curve–curve and curve–surface intersection.
- [ ] M3 — Surface–surface intersection.
- [ ] M4 — Half-edge topology and Euler operators.
- [ ] M5 — Primitives, extrude, revolve.
- [ ] M6 — Booleans.
- [ ] M7 — Tessellation and STL.

## Crates

- `kerf-geom` — exact analytic curves and surfaces, intersection routines.
- `kerf-topo` — half-edge topology and Euler operators.
- `kerf-brep` — the kernel (geometry + topology + booleans + constructors).
