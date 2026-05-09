# kerf-cad: Progress toward Solidworks-tier CAD

Last updated: 2026-05-09
Current score: 73.13%

## What "100%" means

Solidworks-tier = matches Solidworks's authoring + viewer + drawings + production output for typical end-user CAD work.

## Scorecard

| Capability | SW weight | We're at | SW pts |
| --- | ---: | ---: | ---: |
| Planar booleans + primitives + validation | 15% | 99% | 14.85 |
| Authoring layer (params + expressions) | 6% | 99% | 5.94 |
| 3D viewer (mesh, camera, lighting) | 7% | 92% | 6.44 |
| Picking / selection (face → owner Feature) | 5% | 70% | 3.50 |
| Feature tree UI | 5% | 80% | 4.00 |
| Production output (STL/STEP/OBJ) | 3% | 95% | 2.85 |
| Drawings (3-view + dimensions) | 4% | 50% | 2.00 |
| Constraint solver (forward expressions) | 10% | 30% | 3.00 |
| Sweep / loft (Revolve, Loft, TaperedExtrude, PipeRun, SweepPath, Coil, Spring, AngleArc, DistanceRod) | 6% | 70% | 4.20 |
| Manufacturing features (240+ — see catalog) | 12% | 95% | 11.40 |
| Reference geometry (RefPoint, RefAxis, RefPlane, Mirror, BoundingBoxRef, CentroidPoint, DistanceRod, AngleArc, Marker3D, VectorArrow) | 3% | 85% | 2.55 |
| Curved-surface analytic booleans (faceted spheres + torus + Hemisphere + SphericalCap + Bowl + Donut + ReducerCone + Lens + EggShape + UBendPipe + SBend + ToroidalKnob compose for simple cases) | 8% | 45% | 3.60 |
| 2D sketcher UI | 8% | 60% | 4.80 |
| Assembly (multi-body + mates) | 8% | 50% | 4.00 |
| **Solidworks-tier total** | **100%** | | **73.13%** |

## Recent shifts

- 2026-05-09: PRs #39 / #40 / #41 land. Authoring layer 98% → 99%, Feature tree UI 60% → 80%, 3D viewer 90% → 92%. Also corrected two long-stale rows: 2D sketcher UI 0% → 60% (11 constraints + Newton-LM solver already in integration/smart-merge), Assembly 0% → 50% (8 mate types incl. Gear, sub-assemblies, cycle solver already shipped).
- 2026-05-06: SweepPath, Donut, picking-provenance, structural batch — see STATUS.md.

## Scoring methodology

Each row's `SW pts = weight * (we're at)/100`. The total is the sum. The weights are deliberate: the rows where we're farthest behind (sketcher, assembly, drawings, constraint solver) carry weight proportional to how core they are to a real SW workflow, NOT to how easy they are to ship.

Goodhart warning: don't optimize the score by adding more catalog entries to "Manufacturing features". The picking, sketcher UI, and viewer-UX rows dominate the remaining 35-point gap and that's where time should go.

## Updating the score

Run `scripts/score.sh` to print the totals. Edit the "We're at" column manually on each PR that materially advances a capability.
