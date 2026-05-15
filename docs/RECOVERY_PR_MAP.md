# PR Recovery Map (post-revert 2026-05-15)

Trunk was force-reset to `4ae54b1` (Viviani kernel sprint) after a bulk
`-X theirs` rebase strategy corrupted the workspace. The 38 SW-tier PRs
listed below were "merged" against trunk but the merge commits were dropped
in the revert. GitHub marks them MERGED, but their content is not in trunk.

The original (pre-clobber) PR head commits are still in local git's object
store (recovered from HEAD reflog). To re-land any PR:

```
git fetch origin --quiet
git checkout integration/smart-merge
git checkout -b recover/pr-NN
git cherry-pick <ORIGINAL-SHA>~..<ORIGINAL-SHA>   # or list each commit
# Resolve conflicts file-by-file (do NOT use -X theirs)
cargo check --workspace && cargo test --workspace --lib
git push origin recover/pr-NN
gh pr create --base integration/smart-merge --head recover/pr-NN ...
gh pr merge --squash
```

| PR  | Branch (gone)                       | Original SHA | Topic                                        |
|-----|-------------------------------------|--------------|----------------------------------------------|
| #40 | feat/sw-suppression-rollback        | d508f6e      | per-feature suppression + rollback bar       |
| #44 | feat/sw-mass-properties             | 4f44974      | mass properties (vol/area/centroid/inertia)  |
| #45 | feat/sw-step-import                 | c87447e      | STEP import (planar polyhedral)              |
| #46 | feat/sw-gdt-drawings                | fce0a4a      | GD&T annotations on drawings                 |
| #47 | feat/sw-configurations              | 1032e88      | Configuration Manager + Design Tables        |
| #48 | feat/sw-assembly-bom                | fbebf56      | BOM panel for assemblies                     |
| #51 | feat/sw-geometric-constraints       | 3513968      | part-level geometric constraints (Newton)    |
| #52 | feat/sw-cylinder-plane-cap          | ab8f193      | lock cylinder×plane cap behavior             |
| #54 | feat/sw-sketcher-glyphs             | 6f76b06      | sketcher constraint glyphs                   |
| #56 | feat/sw-picking-ux                  | 9008770      | hover preview + Shift+click multi-select     |
| #58 | feat/sw-sketcher-snap               | d5a399c      | sketcher snap engine + DOF readout           |
| #59 | feat/sw-feature-tree-ux             | 683bf96      | feature-tree drag-reorder + search           |
| #61 | feat/sw-hole-table                  | 2b91994      | hole tables on 3-view drawings               |
| #62 | feat/sw-pick-filters                | 8b85cdc      | pick-mode toggle + filter-by-feature-kind    |
| #63 | feat/sw-sweep-batch                 | bce139d      | SpinalLoft, RailedSweep, ScaledExtrude       |
| #65 | feat/sw-ref-geometry-batch          | 83d52c7      | Centerline, MidPlane, ConstructionAxis, etc. |
| #66 | feat/sw-drawing-polish              | c502a55      | drawing dim drag + leader autoplace          |
| #67 | feat/sw-sketch-dim-entry            | 582a0c7      | sketcher undo/redo + dim entry               |
| #68 | feat/sw-constraint-types-2          | a1364c2      | 5 constraint types (Coplanar, EqualLength)   |
| #69 | feat/sw-curved-batch                | b38acb8      | TruncatedSphere, Lens2, Capsule2, OvoidShell |
| #70 | feat/sw-mass-units                  | 3e37854      | metric/imperial mass-properties toggle       |
| #71 | feat/sw-assembly-drag               | 3552a95      | assembly drag + interference detection       |
| #74 | feat/sw-section-detail-views        | 7ca5618      | section + detail drawing views               |
| #75 | feat/sw-sweep-batch-2               | 30f81e5      | HelicalSweep, AxisTaperedTube, etc.          |
| #76 | feat/sw-constraint-types-3          | d66cfaf      | 5 constraint types (PointOnFace, etc.)       |
| #77 | feat/sw-curved-batch-3              | bd3d209      | PaperLanternStrips, Trefoil, DishCap, etc.   |
| #78 | feat/sw-view-cube                   | 6d2e2f1      | view cube + ortho/persp toggle               |
| #79 | feat/sw-constraint-types-4          | c3325a1      | 5 constraint types (EqualRadius, etc.)       |
| #80 | feat/sw-curved-batch-4              | 9970752      | Bagel, Pringle, Cone2, Lozenge               |
| #81 | feat/sw-mfg-saturation              | c34d521      | ShaftOilHole, WoodruffKey, etc.              |
| #82 | feat/sw-sketch-saturation           | adcd6e5      | sketcher copy/paste + mirror + rect-pattern  |
| #83 | feat/sw-pick-saturation             | f328621      | edge-loop, face-loop, smart-pick             |
| #85 | feat/sw-curved-batch-5              | c0be717      | Onion, WaspWaist, Flask, Pear                |
| #87 | feat/sw-final-polish                | 845dafa      | pick history, tree context menu, presets    |
| #88 | feat/sw-curved-batch-6              | a208c68      | PetalCluster, HeartSolid, Whisker, CrossShape|
| #89 | feat/sw-onboarding                  | f0e94d0      | mug tutorial doc + first-run overlay         |
| #95 | feat/sw-ellipse-edges               | cfa7bc0      | detect oblique-cut ellipses → Ellipse        |
| #98 | feat/sw-step-import-analytic        | 9bddbac      | STEP-import parse CIRCLE/ELLIPSE             |

Recovery order suggestion (highest semantic value first, fewest cross-PR
dependencies):

1. #95, #98 — kernel sprint completion (Ellipse emission + STEP-import)
2. #45 — STEP import (functional gain)
3. #44 — mass properties (functional gain)
4. #51 — geometric constraints (functional gain)
5. #47 — Configurations (functional gain)
6. #48 — BOM (functional gain)
7. #74, #46, #61, #66 — drawing polish batch
8. #63, #75, #80, #88, #77, #85, #69 — curved/sweep batches
9. #65, #71, #82 — sketcher & ref-geom polish
10. #54, #56, #58, #59, #62, #67, #78, #83, #87 — picking/UX polish
11. #40, #52, #68, #70, #76, #79, #81, #89 — remaining

Most have conflicts in `viewer/src/main.ts`, `viewer/package.json`,
`viewer/pnpm-lock.yaml`, and the Feature/Model definitions. Plan on
~10-20 min per PR for proper conflict resolution.
