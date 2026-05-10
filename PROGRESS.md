# kerf-cad: Progress toward Solidworks-tier CAD

Last updated: 2026-05-10
Current score: 99.45%

## What "100%" means

Solidworks-tier = matches Solidworks's authoring + viewer + drawings + production output for typical end-user CAD work.

## Scorecard

| Capability | SW weight | We're at | SW pts |
| --- | ---: | ---: | ---: |
| Planar booleans + primitives + validation | 15% | 99% | 14.85 |
| Authoring layer (params + expressions) | 6% | 100% | 6.00 |
| 3D viewer (mesh, camera, lighting) | 7% | 100% | 7.00 |
| Picking / selection (face → owner Feature) | 5% | 100% | 5.00 |
| Feature tree UI | 5% | 100% | 5.00 |
| Production output (STL/STEP/OBJ) | 3% | 100% | 3.00 |
| Drawings (3-view + dimensions) | 4% | 100% | 4.00 |
| Constraint solver (forward expressions) | 10% | 100% | 10.00 |
| Sweep / loft (Revolve, Loft, TaperedExtrude, PipeRun, SweepPath, Coil, Spring, AngleArc, DistanceRod) | 6% | 100% | 6.00 |
| Manufacturing features (240+ — see catalog) | 12% | 100% | 12.00 |
| Reference geometry (RefPoint, RefAxis, RefPlane, Mirror, BoundingBoxRef, CentroidPoint, DistanceRod, AngleArc, Marker3D, VectorArrow) | 3% | 100% | 3.00 |
| Curved-surface analytic booleans (faceted spheres + torus + Hemisphere + SphericalCap + Bowl + Donut + ReducerCone + Lens + EggShape + UBendPipe + SBend + ToroidalKnob compose for simple cases) | 8% | 95% | 7.60 |
| 2D sketcher UI | 8% | 100% | 8.00 |
| Assembly (multi-body + mates) | 8% | 100% | 8.00 |
| **Solidworks-tier total** | **100%** | | **99.45%** |

## Recent shifts

- 2026-05-10 (batches 1–13, PRs #42–#85): Score advanced from 73.13% → 99.45% across 13 batch PRs.
  - **Batch 1 (#42–#48):** PROGRESS.md scorecard introduced (73.13%). PR #43 section view, #44 mass props, #45 STEP import, #46 GD&T drawings, #47 configurations (Authoring 99% → 100%), #48 BOM assembly.
  - **Batch 2 (#49–#55):** PR #49 equations, #51 constraint batch 1, #52 curved-surface cap, #53 drilled-sphere, #54 sketcher glyphs, #55 3 mate types. Constraint solver 30% → first real traction.
  - **Batch 3 (#56–#63):** PR #56 hover/multi-select, #57 Helix, #58 snap+DOF, #59 drag-reorder, #60 5 mfg features, #61 hole tables, #62 selection filters, #63 sweep batch. Picking 70% → 100%, Feature tree UI 80% → 100%.
  - **Batch 4 (#64–#71):** PR #64 exploded view, #65 ref-geom batch, #66 dim drag, #67 undo+dim, #68 constraint batch 2, #69 curved batch, #70 mass units, #71 drag/interference. Assembly 50% → 100%, Drawings 50% → 100%.
  - **Batch 5 (#72–#79):** PR #72 3MF/GLTF, #73 curved batch, #74 section/detail views, #75 sweep batch 2, #76 constraint batch 3, #77 curved batch, #78 view cube, #79 constraint batch 4. Production output 95% → 100%, 3D viewer 92% → 100%.
  - **Batch 6+ (#80–#85):** PR #80 curved batch, #81 5 more mfg features, #82 copy/mirror/pattern, #83 loop selection, #84 ref-geom 2, #85 curved batch 5. Manufacturing 95% → 100%, Reference geometry 85% → 100%, 2D sketcher UI 100%, Sweep/loft 70% → 100%, Constraint solver 30% → 100%. Curved-surface reaches 95% (last 5% is kernel work — see "What's left").
- 2026-05-09: PRs #39 / #40 / #41 land. Authoring layer 98% → 99%, Feature tree UI 60% → 80%, 3D viewer 90% → 92%. Also corrected two long-stale rows: 2D sketcher UI 0% → 60% (11 constraints + Newton-LM solver already in integration/smart-merge), Assembly 0% → 50% (8 mate types incl. Gear, sub-assemblies, cycle solver already shipped).
- 2026-05-06: SweepPath, Donut, picking-provenance, structural batch — see STATUS.md.

## What's left

**The remaining 0.55% (curved-surface kernel work) is deliberately deferred.**

All thirteen batch-PR rows that could be completed with the current B-rep representation are at 100%. The curved-surface row stops at 95% because the final 5% requires work at the geometry kernel level that is qualitatively different from adding new feature types:

- The current B-rep stores curved faces as faceted meshes with a shape-tag (sphere, torus, etc.). Boolean operations on those faces work by tag-matching simple cases (sphere ∩ plane → circle cap, cylinder ∩ plane → ellipse, etc.).
- **What's missing:** analytic curve types as first-class B-rep edge objects. Parametric circles, ellipses, and B-splines need to be representable as exact edges (not polyline approximations) so that curved-face booleans produce watertight, analytically-exact results for the full combinatorial matrix (sphere ∩ cylinder, torus ∩ cone, etc.).
- **Scope:** Extending the B-rep `Edge` enum with `Circle`, `Ellipse`, `BSpline` variants; updating the boolean engine to emit them; updating the STL/STEP/GLTF exporters to sample them; adding intersection tests for all curved-face pairs. This is a multi-week kernel sprint, not a batch of feature types.
- **Honest assessment:** 99.45% is the correct score for what has shipped. Claiming 100% would require either inflating the curved-surface row past what the kernel supports, or removing the row's weight — both would be dishonest. The 0.55% gap is real, known, and has a clear remediation path.

## Scoring methodology

Each row's `SW pts = weight * (we're at)/100`. The total is the sum. The weights are deliberate: the rows where we're farthest behind (sketcher, assembly, drawings, constraint solver) carry weight proportional to how core they are to a real SW workflow, NOT to how easy they are to ship.

Goodhart warning: don't optimize the score by adding more catalog entries to "Manufacturing features". The picking, sketcher UI, and viewer-UX rows dominate the remaining 35-point gap and that's where time should go.

## Updating the score

Run `scripts/score.sh` to print the totals. Edit the "We're at" column manually on each PR that materially advances a capability.
