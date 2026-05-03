# kerf-cad: where we are

A snapshot of what works today, taken after a focused build session that
turned the kerf B-rep kernel into a usable parametric CAD pipeline.

## What you can do today

```sh
# 1. headless / scripted
cargo run -p kerf-cad -- model.json out part.step
cargo run -p kerf-cad -- model.json out part.stl   --segments 32
cargo run -p kerf-cad -- model.json out part.obj

# 2. browser, with parameter sliders + face picking + 3-view PNG
cd viewer && pnpm install && ./build-wasm.sh && pnpm dev
# → http://localhost:5174
```

A typical model is ~10 lines of JSON declaring a tree of `Box`, `Cylinder`,
`Tube`, `HollowBox`, `HollowCylinder`, `Slot`, `Wedge`, `RegularPrism`,
`Revolve`, `ExtrudePolygon`, `Sphere`, `Torus`, `Cone`, `Frustum`
primitives, `Translate`/`Rotate`/`Mirror`/`LinearPattern`/`PolarPattern`/
`CornerCut`/`Fillet`/`Chamfer` transforms, and
`Union`/`Intersection`/`Difference` booleans. Every numeric field accepts
literals, `$param` references, or arbitrary arithmetic expressions
(`"$plate_x / 2 + sqrt(16)"`).

## Readiness against published CAD packages

Weighted scorecard, where Solidworks/Fusion-tier = 100% and OpenSCAD-tier is
the subset of categories OpenSCAD actually covers (~31 SW pts, mostly:
kernel + authoring + viewer + production output).

| Capability                                | SW weight | We're at | SW pts |
|-------------------------------------------|----------:|---------:|-------:|
| Planar booleans + primitives + validation | 15%       | 92%      | 13.8   |
| Authoring layer (params + expressions)    | 6%        | 95%      | 5.7    |
| 3D viewer (mesh, camera, lighting)        | 7%        | 90%      | 6.3    |
| Picking / selection                       | 5%        | 50%      | 2.5    |
| Feature tree UI                           | 5%        | 60%      | 3.0    |
| Production output (STL/STEP/OBJ)          | 3%        | 95%      | 2.85   |
| Drawings (3-view + dimensions)            | 4%        | 50%      | 2.0    |
| Constraint solver (forward expressions)   | 10%       | 30%      | 3.0    |
| Sweep / loft (Revolve only)               | 6%        | 15%      | 0.9    |
| Manufacturing features (CornerCut, Fillet, Chamfer; single-edge only) | 12% | 20%  | 2.4    |
| Reference geometry (Mirror is adjacent)   | 3%        | 5%       | 0.15   |
| Curved-surface analytic booleans          | 8%        | 0%       | 0      |
| 2D sketcher UI                            | 8%        | 0%       | 0      |
| Assembly (multi-body + mates)             | 8%        | 0%       | 0      |
| **Solidworks-tier total**                 | **100%**  |          | **~42.6%** |
| **OpenSCAD-tier (out of 31 SW pts)**      |           |          | **~91%**   |

Started this session at 18% / 57%. Ended this session at ~40% / 91%.
Current run shipped Fillet, Chamfer, Slot, HollowCylinder, Wedge, and
RegularPrism on top of that → ~42.6% / 91%. The OpenSCAD-tier number
doesn't move — the new features are SW-style operations, not the kernel
or authoring categories OpenSCAD scores against.

## What's left to reach 100% Solidworks-tier

The gap is roughly 57 SW pts and is dominated by features that are each
multi-week engineering projects in their own right:

- **2D sketcher UI + constraint solver** (18 SW pts combined). Real CAD's
  primary input mode. Sketcher needs an interactive 2D drawing surface;
  bidirectional constraint solver is its own discipline.
- **Manufacturing features — full Fillet/Chamfer (multi-edge stacking),
  Shell, Draft** (~10 SW pts). Single-edge axis-aligned Fillet/Chamfer
  ship today; stacking multiple Fillets on the same body trips the
  boolean engine where the second fillet's wedge cutter meets the first
  fillet's curved face. Shell + Draft both need true offset-surface math
  the kernel doesn't have. Each is multi-week work.
- **Curved-surface analytic booleans** (8 SW pts). kerf currently handles
  planar + faceted curves; analytic cylinder/sphere/torus booleans are a
  major kernel addition.
- **Sweep / loft** (6 SW pts after Revolve). Sweep along a curve, loft
  between profiles. Each is its own primitive with topology bookkeeping.
- **Assembly + mates** (8 SW pts). Multiple bodies with relative
  positioning constraints.
- **Picking → edit** loop (~3 SW pts). Currently we pick a face but can't
  fillet *that* face; we'd need entity-id exposure across the JSON model.

A reasonable estimate for one focused engineer to reach 80% Solidworks-
tier: 3-6 months. To reach 100% (true parity with a 30-year-old
proprietary product): years.

## What's real and what's brittle

**Real:**
- The kernel handles planar booleans for arbitrary tree depth (the
  chained-DIFF bug fixed in the prior session was a multi-day root-cause
  hunt).
- Parameters + expressions make the model authoring loop tight.
- The viewer is genuinely usable — drop a JSON, scrub sliders, see the
  mesh, export STL.
- Mirror works including downstream Union composition.
- Single-edge axis-aligned Fillet + Chamfer ship and survive into the
  STL/STEP exports.
- 413 tests pass.

**Brittle:**
- Coplanar overlapping faces still trip the boolean engine in some
  configurations. Not all multi-cylinder unions work.
- Stacking multiple `Fillet`s on the same body fails at the second
  fillet whose wedge cutter meets the first fillet's rounded face.
  Single-corner fillets work; designs that want multiple fillets must
  union pre-filleted parts.
- The boolean engine returns an empty solid when both inputs are
  analytic spheres (the sphere primitive has 1 face / 0 edges, which
  the stitch step can't reason about). That's why `HollowSphere` was
  attempted but isn't in the catalog.
- Volume measurement returns zero for solids whose faces have empty
  outer-loop walks (analytic spheres, vase-style revolved solids).
- The viewer is read-only for the JSON itself — you can delete features
  in the tree and download the modified JSON, but can't edit a feature's
  fields inline.
- No undo/redo in the viewer. (The JSON file IS your undo: git diff.)

## How to keep going

- **Next quick win** (~half a day): face-id-to-edit-id mapping in the
  WASM, so picking a face → scheduling a fillet of its edges becomes
  possible. ~+3 SW pts.
- **Next medium win** (~2 weeks): sketch-on-plane + line/circle drawing
  primitives in the viewer, written to JSON as DSL extension. ~+10 SW
  pts.
- **Next big win** (~1 month): real fillet for orthogonal planar edges
  using offset-surface construction. ~+5 SW pts.

If you want to keep grinding toward Solidworks parity, the right shape is
probably a recurring background agent (`/schedule`) that picks one of
these per week and opens a PR — not a single long session.
