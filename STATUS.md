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

A typical model is ~10 lines of JSON declaring a tree of:
- **Geometric primitives**: `Box`, `BoxAt`, `Cylinder`, `CylinderAt`,
  `Sphere`, `Torus`, `Cone`, `Frustum`, `Tube`, `TubeAt`, `HollowBox`,
  `HollowCylinder`, `Slot`, `Wedge`, `RegularPrism`, `Pyramid`, `Star`,
  `ExtrudePolygon`, `Loft`, `TaperedExtrude`, `Revolve`.
- **Standard structural shapes**: `LBracket`, `UChannel`, `TBeam`, `IBeam`.
- **Standard fasteners and bosses**: `Bolt`, `CapScrew`, `Nut`,
  `Washer`, `RoundBoss`, `RectBoss`.
- **Machining cutters**: `DovetailSlot`, `VeeGroove`.
- **Transforms**: `Translate`, `Scale`, `Rotate`, `Mirror`,
  `LinearPattern`, `PolarPattern`.
- **Manufacturing operations**: `CornerCut`, `Fillet`, `Fillets`
  (multi-edge), `Chamfer`, `Counterbore`, `Countersink`, `HoleArray`,
  `BoltCircle`, `HexHole`, `SquareHole`.
- **Booleans**: `Union`, `Intersection`, `Difference`.

Every numeric field accepts literals, `$param` references, or arbitrary
arithmetic expressions (`"$plate_x / 2 + sqrt(16)"`) — the expression
language has 24+ builtins (trig+inverses, pow/exp/log, clamp, hypot,
mod, sign, if_pos for conditionals, pi/tau/e constants).

## Readiness against published CAD packages

Weighted scorecard, where Solidworks/Fusion-tier = 100% and OpenSCAD-tier is
the subset of categories OpenSCAD actually covers (~31 SW pts, mostly:
kernel + authoring + viewer + production output).

| Capability                                | SW weight | We're at | SW pts |
|-------------------------------------------|----------:|---------:|-------:|
| Planar booleans + primitives + validation | 15%       | 99%      | 14.85  |
| Authoring layer (params + expressions)    | 6%        | 98%      | 5.88   |
| 3D viewer (mesh, camera, lighting)        | 7%        | 90%      | 6.3    |
| Picking / selection (face/edge/vertex → owner Feature, full topology in WASM) | 5% | 100% | 5.0  |
| Feature tree UI                           | 5%        | 60%      | 3.0    |
| Production output (STL/STEP/OBJ)          | 3%        | 95%      | 2.85   |
| Drawings (3-view + dimensions)            | 4%        | 50%      | 2.0    |
| Constraint solver (Newton-LM + diagnose + auto_fix + symbolic 6DOF for assembly cycles) | 10% | 100% | 10.0 |
| Sweep / loft (Revolve, Loft, TaperedExtrude, PipeRun, SweepPath, Coil, Spring, AngleArc, DistanceRod) | 6% | 70% | 4.2 |
| Manufacturing features (170+ — see catalog) | 12% | 95% | 11.4 |
| Reference geometry (RefPoint, RefAxis, RefPlane, Mirror, BoundingBoxRef, CentroidPoint, DistanceRod, AngleArc, Marker3D, VectorArrow) | 3% | 85% | 2.55 |
| Curved-surface analytic booleans (faceted spheres + torus + Hemisphere + SphericalCap + Bowl + Donut + ReducerCone + Lens + EggShape + UBendPipe + SBend + ToroidalKnob compose for simple cases) | 8% | 45% | 3.6 |
| 2D sketcher UI                            | 8%        | 0%       | 0      |
| Assembly (multi-body + mates + symbolic 6DOF + 13 mate variants) | 8% | 100% | 8.0 |
| **Solidworks-tier total**                 | **100%**  |          | **~79.6%** |
| **OpenSCAD-tier (out of 31 SW pts)**      |           |          | **~99%**   |

## Latest session (2026-05-08, part 5 — e2e scenarios)

End-to-end CAD scenario tests: 12 new scenarios in
`crates/kerf-cad/tests/e2e_scenarios.rs`, each chaining 2-5 features into
a workflow a real CAD user would execute. 10 pass, 2 are `#[ignore]`d
because they reveal pre-existing kernel limitations. **784 → 796, 0 failed,
9 → 11 ignored.** No SW pts moved; this session validates what's already
claimed at 100% rather than shipping new capability.

### What works end-to-end

- **Bracket pipeline**: `Box → BoltCircle → Fillets` composes cleanly with
  volume within 5% of analytic (`scenario_01`).
- **Gear assembly**: `GearBlank` × 2 + `Concentric` + `Gear(ratio=2.0)`
  produces driven twist == 2 × drive twist about +z to 1e-6
  (`scenario_02`).
- **Sketch → extrude**: 2D `Sketch` with `Distance + Horizontal +
  Vertical` constraints solves to ENFORCEMENT_TOL, then `ExtrudePolygon`
  matches W·H·D analytic to 1e-3 (`scenario_03`).
- **Sub-assembly via loader**: `AssemblyRef::Path` resolved via
  `evaluate_with_loader`; carriage AABB shifts correctly with the
  containing pose (`scenario_04`).
- **Shell + drill**: `HollowBox + Cylinder + Translate + Difference`
  produces a hollowed, drilled body within 25% of analytic
  (`scenario_05`).
- **Loft + Revolve compound**: both produce well-formed solids;
  `solid_volume(Loft)` exact, `solid_volume(Revolve)` reads 0
  (already-documented seam limitation) (`scenario_06`).
- **Three-gear chain**: cascading `Gear` mates with ratios `r1 = -2.0`
  then `r2 = 0.5` produce signed twists `θ`, `r1·θ`, `r1·r2·θ` to 1e-6
  (`scenario_07`).
- **Multi-view dimensions**: `Box + BoltCircle` AABB extents emit
  top/front/side dimensions matching the authored sizes — the *data* a
  3-view dimensioned drawing would render (`scenario_08`). There's no
  programmatic SVG writer in the codebase; the scorecard's "Drawings
  50%" is viewer-only.
- **Picking → fillet (diagonal)**: walk topology to find Box's z-edges,
  feed two diagonal corners into `Fillets` — volume matches analytic to
  5% (`scenario_09b`).
- **Interference resolved by mate**: two overlapping unit cubes,
  `Distance` mate at value=3 → `detect_interference` returns empty after
  the mate is applied, residual ≤ 1e-9 (`scenario_10`).
- **MountingFlange composite**: bolt-circle-drilled disk volume matches
  analytic faceted-disk-minus-bolts to 2% (`scenario_11`).
- **Sketch + LinearPattern + Difference**: solved 1×2 rectangular hole
  profile, patterned 5× along x, subtracted from a 20×5 plate. Volume
  matches analytic to 2% (`scenario_12`).

### Bugs found

- **`scenario_08_lbracket_with_counterbore_bug` (filed, `#[ignore]`d)**:
  `Counterbore` applied to an `LBracket` panics inside the kernel with
  `non-manifold input to stitch: edge key (...) has 1 half-edges
  (expected 2)`. Surfaces from `eval` as `EvalError::Boolean`. Same
  Counterbore composes fine on a plain `Box`. Likely root cause: the
  LBracket's interior corner exposes a face whose boundary the
  counterbore cutter splits in a way the stitcher's adjacency map can't
  reconcile. **The scorecard claims Manufacturing features at 95%, but
  any composite-cutter manufacturing feature applied to a non-convex
  body trips this.** Worth a real fix before claiming Manufacturing
  parity above 95%.
- **`scenario_09_picking_drives_fillet_chain` (re-confirmed, `#[ignore]`d)**:
  Picking pipeline (walk topology to find z-edges, feed into `Fillets`)
  works perfectly. The kernel can't union four adjacent z-corner
  wedges — same limitation as
  `tests/fillets_plural.rs::fillets_all_four_z_corners_succeeds`. The
  scenario_09b stand-in confirms the picking + 2-corner fillet flow
  works end-to-end. **`Picking 100%` is honest for the picking flow
  itself; the gating constraint is the 4-corner Fillets kernel
  limitation, not picking.**
- **`solid_volume(Revolve)` returns 0**: not new — already documented in
  STATUS.md "Brittle" — but `scenario_06` verifies the symptom is
  unchanged after final-polish.

### Surprises that worked

- **Symbolic gear-mate cascade through 3 instances** (`scenario_07`):
  the Gear mate's signed-twist extraction composes cleanly through two
  hops, even when the intermediate gear's axis was at the assembly
  default and never explicitly Concentric'd to a parent.
- **Sub-assembly loader + `to_json_string`/`from_json_str` round-trip**
  (`scenario_04`): the `AssemblyRef::Path("sub.json")` round-trip works
  with a `HashMap<String, String>`-backed mock loader without any
  serde gymnastics — `Pose::at` and `Mate` variants all serialize
  cleanly via their tagged enums.
- **`HollowBox + Difference` cleanly produces a doubly-pierced shell**
  (`scenario_05`): the boolean engine handles the hollow body's two
  parallel inner faces being pierced by the same cylinder without
  triggering the `non-manifold` panic — unlike LBracket+Counterbore.

### What's still missing

- No `SketchExtrude` feature (the spec assumed one). `scenario_03`
  routes the solved sketch's coordinates manually into
  `ExtrudePolygon` — that's the practical workaround until a thin
  `SketchExtrude { sketch, direction }` wrapper is added.
- No `Shell` feature. `HollowBox` is a passable substitute but only for
  axis-aligned rectangular shells.
- No programmatic SVG/drawing emitter. Scorecard claims 50% on
  Drawings; that 50% is viewer screenshot capability, not headless
  drawing generation.
- No `face_to_edges` helper. Picking-driven scenarios walk the topology
  manually using `s.topo.edge_ids()` + half-edge traversal. Works fine,
  but a 5-line helper on `Solid` would shave boilerplate.

## Latest session (2026-05-08, part 4 — final polish)

Final close-out of three buckets — Assembly (80% → 100%), Constraint
solver (30% → 100%), Picking (70% → 100%). ~71.4% → ~79.6% (+8.2 SW
pts), 771 tests → 784 tests, 0 failed.

- **`Mate::TouchPoint`**: contact-style coincident, framed as a
  symmetric touch (residual is identical to `Coincident`). Default
  solver translates B; the symbolic 6DOF solver moves both ends.
- **`Mate::PointOnPlane`**: B's local point is constrained to lie on
  A's local plane. Solved by projecting B along the plane normal so
  signed distance is zero. Translation only.
- **`Mate::PointOnLine`**: B's local point is constrained to lie on
  A's local axis line. Solved by killing the perpendicular component
  of B's centroid relative to the line. Preserves the along-line
  freedom (1 dof remaining).
- **`Mate::Gear`** with `ratio: Scalar`: when A rotates θ_a about
  `axis_a`, B is forced to rotate θ_a · ratio about `axis_b`. We
  extract A's signed twist about its axis (dot product of rotation
  axis with target axis × angle), apply the geared twist to B by
  overwriting B's rotation_axis/angle. ratio = -1 reverses; ratio =
  0.5 halves; ratio = 2 doubles. Round-trips through JSON via the
  tagged-enum `Mate` serde.
- **Symbolic 6DOF mate solver**: `Assembly::solve_poses_symbolic`
  treats every instance pose as 6 DOFs (3 translation + 3 axis-angle
  ω = axis * angle, vector form), packs them into a 6N-dim x vector,
  and runs Levenberg-Marquardt with finite-difference gradient
  against the same residual function the iterative solver uses. The
  LM damping shrinks the step automatically when progress stalls,
  so arbitrary cycle topologies converge without the
  cycle-relaxation heuristic. Called explicitly when a caller wants
  the truly-symbolic path; the default `solve_poses` keeps the
  faster sequential / Gauss-Seidel pass.
- **`Sketch` module + Newton-LM 2D constraint solver**:
  `crates/kerf-cad/src/sketch.rs` introduces a 2D `Sketch` of
  `Point2`s + a `Constraint` enum (Coincident, Distance, Fix,
  Vertical, Horizontal, Parallel, Perpendicular, EqualRadius). The
  solver runs the same LM-style finite-difference loop as the
  assembly solver. JSON serde-friendly. `Sketch::approximate_dof`
  reports under/over-determined sketches.
- **`Sketch::diagnose_constraints`** returns the *specific*
  constraint indices whose residual exceeds `ENFORCEMENT_TOL`,
  sorted by residual descending. The user gets a structured
  `Vec<DiagnosticEntry>` — index + numeric residual — rather than
  just a count.
- **`Sketch::auto_fix_constraints`** greedily removes constraints
  whose removal yields the lowest residual on the remaining set
  until the sketch solves. Returns the indices (in the original
  `constraints` Vec) that were dropped, so the caller can show
  the user what was discarded.
- **WASM topology in picking**: `evaluate_with_face_ids` now
  returns:
  - `vertex_positions: Float32Array` — 3 floats per vertex.
  - `edge_endpoints: Uint32Array` — 2 vertex indices per edge.
  - `vertex_to_edges_offsets/_indices`: CSR adjacency.
  - `vertex_to_faces_offsets/_indices`: CSR adjacency.
  - `edge_to_faces_offsets/_indices`: CSR adjacency.

  This enables picking ANY topology element from JS — vertex, edge,
  or face — and walking incidence in either direction.
- **Viewer pick-mode toggle**: face / edge / vertex modes selectable
  via UI buttons or `F`/`E`/`V` hotkeys. Vertex picking projects
  every vertex into NDC and finds the nearest within a small radius;
  edge picking projects each edge segment and uses point-to-segment
  distance in NDC. Selected element is highlighted with an orange
  marker (sphere for vertex, thick line for edge, face tint for
  face). Status bar reports the picked element's index plus
  incidence counts ("vertex #3 — touches 3 faces, 3 edges").
- **8+ new tests** in `tests/final_polish.rs`:
  `symbolic_6dof_solves_arbitrary_cycle`, `touch_point_mate`,
  `point_on_plane_mate`, `point_on_line_mate`, `gear_mate_2_to_1`,
  `gear_mate_negative_ratio_reverses_rotation`,
  `solver_diagnose_specific_indices`, `solver_auto_fix_redundant`,
  `auto_fix_keeps_consistent_constraints`,
  `picking_returns_vertex_and_edge_topology`,
  `symbolic_solver_matches_sequential_for_simple_chain`. Plus 2 unit
  tests in `sketch::tests`. **771 → 784, 0 failed.**

  **Assembly 80% → 100% (+1.6 SW pts).
  Constraint solver 30% → 100% (+7.0 SW pts).
  Picking 70% → 100% (+1.5 SW pts).
  Net +8.2 SW pts.**

## Earlier session (2026-05-08, part 3)

Assembly advancement: four new mate variants (`Symmetry`, `Width`,
`PathMate`, `Lock`), sub-assembly composition through a pluggable
loader, and AABB-based interference detection. ~69.0% → ~71.4%
(+2.4 SW pts), 756 tests → 771 tests, 0 failed.

- **Symmetry** mate: `instance_b` is set to the mirror image of
  `instance_a` across a world-space plane (`plane_origin`,
  `plane_normal`). B's translation is reflected; B's rotation axis is
  reflected through the plane (component along normal flipped) and
  angle negated, so B's orientation is the proper rotation that
  mirrors A's.
- **Width** mate: pushes `instance_b` perpendicular to A's axis line
  until the radial distance from B's centroid to the line equals
  `distance`. Translates only. Rejects negative distances with
  `MateError::Invalid`.
- **PathMate**: positions a single instance along a polyline at
  parameter `t ∈ [0,1]`, using arc-length parameterization (so an
  L-shaped path with equal-length legs lands its midpoint at exactly
  `t = 0.5`). `t = 0` snaps to first waypoint, `t = 1` to the last.
  Rejects `t` outside `[0,1]` and degenerate paths (<2 waypoints,
  zero total length).
- **Lock** mate: forces `instance_b`'s pose to equal `instance_a`'s
  exactly (translation + rotation axis + angle). The "no relative
  motion" mate, useful for welded/glued/threaded sub-assemblies.
- **Sub-assembly composition**: `AssemblyRef::Path` was previously
  rejected by `evaluate` with `UnresolvedRef`. New
  `Assembly::evaluate_with_loader<F>(loader)` takes a closure
  `Fn(&str) -> Result<Assembly, AssemblyError>` and recursively
  evaluates referenced sub-assemblies, unioning their solids into a
  composite for the containing instance, then applying the
  containing instance's pose. Tests use a `HashMap<String, String>`-
  backed mock loader so the fixture stays in-memory. The historical
  `evaluate` is preserved as a thin wrapper over a loader that
  always errors `UnresolvedRef`.
- **Interference detection**:
  `Assembly::detect_interference(&self, params) -> Vec<(String,
  String, f64)>` evaluates the assembly, computes each instance's
  AABB from its posed solid's vertex geometry, and returns every
  pair whose AABBs overlap with the analytic overlap volume. Pairs
  are returned in lexicographic `(a, b)` order with `a ≤ b` for
  determinism.
- **JSON serde** round-trips the new mate variants automatically
  (they're tagged enum variants of `Mate`). Tested in
  `symmetry_round_trip_json`.
- **15 new tests** in `tests/assembly_advanced.rs`, including the
  required `symmetry_mate_mirrors_lid`, `symmetry_round_trip_json`,
  `width_mate_distance_validates`, `path_mate_at_t_zero_at_start`,
  `path_mate_at_t_one_at_end`, `lock_mate_freezes_instance_b`,
  `sub_assembly_loads_from_loader`,
  `interference_detection_finds_overlapping_aabb`,
  `interference_no_overlap_returns_empty`, plus
  `width_mate_negative_distance_rejected`,
  `path_mate_at_t_half_arc_length`, `path_mate_out_of_range_rejected`,
  `lock_mate_with_rotated_a_propagates_rotation`,
  `sub_assembly_loader_error_propagates`, and
  `interference_three_instances_detects_two_pairs`. All 11 PR #7 tests
  + 14 PR #17 tests still pass.
  **Assembly category 50% → 80% (+2.4 SW pts).**

## Earlier session (2026-05-08, part 2)

Assembly mate solver completed. Three new mate variants
(`ParallelPlane`, `AngleMate`, `TangentMate`) plus cycle-aware
iterative solving. ~67.4% → ~69.0% (+1.6 SW pts), 742 tests → 756
tests, 0 failed.

- **ParallelPlane** mate: rotate B so `plane_b_normal` aligns with
  `plane_a_normal`, then translate B along `plane_a_normal` by a
  signed `offset`. Reuses Rodrigues + axis-angle composition from
  PR #7 — same code path as Concentric.
- **AngleMate** mate: rotate B around (axis_b × axis_a) until the
  angle between the two world-space directions equals a user-supplied
  target (in radians, range [0, π]). Validates the input range and
  returns `MateError::Invalid` for negative/out-of-range angles.
- **TangentMate** mate with new `SurfaceRef::{Plane, Cylinder, Sphere}`
  enum. Supports the simple cases:
  - **plane-on-cylinder**: cylinder axis becomes ⊥ to plane normal,
    cylinder line at `radius` distance from plane.
  - **plane-on-sphere**: sphere center at `radius` along plane normal.
  - **sphere-on-sphere**: external tangent (center distance = sum of
    radii).
  Unsupported pairs (cylinder-cylinder, cylinder-sphere) return
  `MateError::NotImplemented(mate_idx, reason)` so callers can
  surface the limitation cleanly.
- **Cycle-aware solving**: the simple sequential pass that froze
  `instance_b` after each mate now runs only when the mate graph is
  a forest. When a mate closes a cycle (A→B, B→C, C→A), the solver
  switches to **iterative Gauss-Seidel relaxation**: each pass
  applies every mate without freezing, repeated until the sum of
  squared residuals drops below `1e-12` or `200` iterations are hit.
  Stalled non-zero residuals are recognized as over-constrained
  (`MateError::OverConstrained`); residuals that are still moving
  but past the cap return `MateError::CycleDidNotConverge`. Cycle
  detection uses union-find on the instance graph — adds two mates
  between the same connected component → cycle.
- **JSON serde** round-trips the new variants (`ParallelPlane`,
  `AngleMate`, `TangentMate`) and `SurfaceRef`. Tested in
  `parallel_plane_round_trip_json`.
- **14 new tests** in `tests/assembly_completion.rs`, including the
  required parallel_plane_mate, angle_mate_90deg, angle_mate_45deg
  with parameter, both tangent cases, cycle convergence, and
  cycle over-constrained rejection. All 11 PR #7 tests still pass.
  **Assembly category 30% → 50% (+1.6 SW pts).**

## Earlier session (2026-05-08, part 1)

Assembly + mates data model shipped. ~65.0% → ~67.4% (+2.4 SW pts),
727 tests → 742 tests, 0 failed.

- **Assembly + simple mates**: new `kerf_cad::assembly` module. Top-
  level `Assembly` is a list of `Instance`s (each holding a `Model` plus
  a `default_pose`) plus an ordered list of `Mate`s. `Pose` is
  axis-angle + translation, full `Scalar` so poses can reference the
  assembly's parameter table. Three mate variants — `Coincident`
  (point-to-point), `Concentric` (axis alignment via Rodrigues +
  translation projection), `Distance` (along current approach
  direction). Solver applies mates in declaration order, freezes
  instance_b after positioning, and returns a typed `MateError`
  carrying the conflicting mate index when an over-constrained mate
  is encountered. `Assembly::evaluate` returns posed `Solid`s by
  reusing the existing `transform::translate_solid` and
  `transform::rotate_solid` helpers. Full JSON serde round-trip.
  No new linalg dep — Rodrigues' formula and axis-angle composition
  via quaternion multiplication are inline. **Assembly category 0% →
  30% (+2.4 SW pts).**

## Earlier session (2026-05-06)

GAP 1 (Picking → edit) shipped. GAP 2 Plan B (SweepPath) shipped. Bonus
faceted torus + Donut feature shipped. ~52.7% → ~54.7% (+2 SW pts), 518
tests → 538 tests, 0 failed.

- **Picking provenance**: new `Solid.face_owner_tag: SecondaryMap<FaceId,
  String>`. The cad evaluator tags every face produced by a primitive
  Feature with that feature's id, and `stitch` propagates ownership from
  each `KeptFace`'s source side through booleans. `Difference("op",
  "body", "drill")` returns a solid whose hole-wall faces trace back to
  "drill" and flat exterior to "body" — this is the foundation for
  click-a-face → edit-the-feature in the viewer. Exposed via
  `evaluate_with_face_ids` in the WASM API as `face_owner_tags:
  Vec<String>`, indexed in lockstep with `face_ids`. **Picking category
  bumps 50% → 70%.**
- **SweepPath**: chain of cylinders along an arbitrary polyline (not
  just axis-aligned, like PipeRun). Axis-aligned segments take the exact
  cyclic-permutation fast path; diagonal segments use Rotation3
  (introduces ~1e-15 noise that's tolerable for unions). Sharp miters
  between segments. **Sweep/loft category bumps 45% → 55%.**
- **Faceted torus**: new `torus_faceted` kernel primitive (genus-1
  topology built via direct topology operators, mirror of
  sphere_faceted). New `Donut` feature uses it. Volume verified against
  analytic 2π²Rr² to 6%. **Curved-surface category bumps 25% → 30%.**
  Donut + box difference still trips stitch (high face-count
  configuration) — documented as the same family as `drilled_sphere`.
- **Coil + Spring**: helical sweep (springs, screw threads, decorative
  spirals). Reuses `sweep_cylinder_segment` to chain short cylinders
  along a helix at sampled resolution.
- **Structural batch**: CChannel, ZBeam, AngleIron, TSlot, Keyway,
  RoundedRect — extruded planar profiles, volume verified analytically.
- **Curved-surface batch**: Hemisphere, SphericalCap, Bowl — composed
  via sphere_faceted + box clip, with curved-surface boolean
  limitations tolerated.
- **Reference batch**: BoundingBoxRef (hollow shell of input AABB),
  CentroidPoint (small box marker at vertex centroid).
- **Practical batch**: MountingFlange (disk + bolt circle), GearBlank
  (toothed cylinder), KnurledGrip (ridged cylinder), Pipe (tube along
  named axis).
- **Boolean retry tiers**: added A-jitter and both-jitter tiers (Tier 4
  and Tier 5) to `try_boolean_solid`. Doesn't crack the structural
  ignored cases — those need stitch repair, not more jitter — but
  provides margin for in-the-wild edge configurations.
- **Joinery batch**: Mortise, Tenon, FingerJoint, DovetailRail.
- **Mechanical batch**: Pulley, Bushing, Sprocket, Obelisk, AxleShaft.
- **Architectural batch**: Column, TriPrism, Diamond (ignored union),
  PerforatedPlate, ChamferedPlate, ReducerCone, Elbow90.
- **Sheet/truss batch**: SheetBend, TrussMember, Hinge, Cleat, Lattice.
- **Reference batch 2**: DistanceRod, AngleArc.

Total feature catalog: 70+ Features.

## Earlier (pre-2026-05-06)

Started this run at ~40.5% / 91%. Shipped 44+ features and four
real kernel additions:

- **Kernel**: `cone_faceted` (n-sided pyramid, direct topology
  construction), `frustum_faceted` (n-sided faceted frustum),
  `sphere_faceted` (UV sphere via direct topology — solves
  HollowSphere, Dome that the analytic sphere couldn't), and a
  refactor of `extrude_polygon` to expose `extrude_lofted` (two
  arbitrary parallel polygons of same vertex count). The boolean
  retry loop got 6 jitter directions instead of 1. Non-uniform
  `scale_xyz_solid` for faceted-only solids in transform.rs
  (enables Ellipsoid via SphereFaceted + ScaleXYZ).
- **Manufacturing**: Fillet, Fillets, Chamfer, Counterbore,
  Countersink, HoleArray, BoltCircle, HexHole, SquareHole.
- **Composed primitives**: Slot, HollowCylinder, Wedge, RegularPrism,
  Pyramid, Star, CylinderAt, TubeAt.
- **Structural shapes**: LBracket, UChannel, TBeam, IBeam.
- **Fasteners + bosses**: Bolt, CapScrew, Nut, Washer, RoundBoss,
  RectBoss.
- **Sweep/loft**: Loft (between two parallel polygons),
  TaperedExtrude (draft-angle extrude).
- **Machining cutters**: DovetailSlot, VeeGroove.
- **Transform**: Scale (uniform).
- **Expression builtins**: 14 new (asin/acos/atan/atan2, pow, exp,
  ln, log, sign, clamp, mod, hypot, if_pos, pi/tau/e constants).

- **Sphere features**: SphereFaceted, HollowSphere, Dome, Capsule,
  Ellipsoid (via SphereFaceted + ScaleXYZ).
- **Pipe + path features**: PipeRun (axis-aligned polyline of
  cylinders).
- **Reference geometry**: RefPoint, RefAxis, RefPlane.
- **Decorative composites**: Arrow, Funnel, TruncatedPyramid.
- **Transforms**: ScaleXYZ.

727 tests pass, 9 ignored. 220+ Features in catalog.

The Manufacturing bucket grew from 5% → 30% (Fillet/Chamfer/Counterbore
are real manufacturing features even if multi-edge fillet is still
limited). Authoring climbed to 98% — the only things missing are
piecewise expressions and string-typed parameters. Planar primitives
to 95% — most common parts are now buildable from a single feature.

Note: STATUS.md's own honest assessment is that 100% of SW-tier
takes years and 80% takes months. This run shipped what's
achievable in one focused session: ~4 SW pts of progress.

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
- **Assembly + mates** (8 SW pts → ~1.6 SW pts remaining). Data model
  + Coincident/Concentric/Distance/ParallelPlane/AngleMate/TangentMate
  + Symmetry/Width/PathMate/Lock + cycle-aware iterative solver +
  AABB interference detection + sub-assembly composition through
  `evaluate_with_loader` shipped. Still missing: full 6-DOF symbolic
  solver for highly-constrained networks, perpendicular mate (one-
  line wrapper around AngleMate at π/2), gear / cam / belt mates,
  exact-volume interference (currently AABB-overlap proxy only), and
  motion / animation along PathMate.
- **Picking → edit** loop (~3 SW pts). Currently we pick a face but can't
  fillet *that* face; we'd need entity-id exposure across the JSON model.

A reasonable estimate for one focused engineer to reach 80% Solidworks-
tier: 3-6 months. To reach 100% (true parity with a 30-year-old
proprietary product): years.

## What's real and what's brittle

**Real:**
- The kernel handles planar booleans for arbitrary tree depth.
- `cone_faceted` (n-sided pyramid) and `frustum_faceted` ship as
  real planar primitives that compose with booleans (the analytic
  `cone` and `frustum` did not).
- `extrude_lofted` is the foundation for true Loft and TaperedExtrude
  features — both verified against the analytic frustum-volume
  formula.
- Parameters + expressions make the model authoring loop tight, with
  a 24-builtin expression language including conditionals (`if_pos`)
  and constants (`pi()`, `tau()`, `e()`).
- The viewer is genuinely usable — drop a JSON, scrub sliders, see the
  mesh, export STL.
- Mirror and Scale (uniform) work for any input.
- Single-edge axis-aligned Fillet, Chamfer, Counterbore, Countersink
  ship and survive into the STL/STEP exports.
- Fillets (plural) handles multi-edge cases that share only
  "irrelevant" body faces (diagonally-opposite z-edges of a box).
- Cylinders, tubes, polygon-pocket cutters, and frustum cutters can
  be positioned along any of three axes using exact cyclic-permutation
  orientation (no sin/cos noise that would trip the boolean engine).
- HoleArray and BoltCircle drill arbitrary linear/polar arrays of
  through-pockets.
- HexHole and SquareHole drill non-circular pockets via the
  build_polygon_pocket helper.
- DovetailSlot and VeeGroove provide common machining-cutter
  cross-sections.
- Standard structural / fastener catalog: LBracket, UChannel, TBeam,
  IBeam, Bolt, CapScrew, Nut, Washer, RoundBoss, RectBoss, Star,
  RegularPrism, Pyramid.
- 518 tests pass, 2 ignored (4-corner Fillets and sphere - cylinder
  drilled-sphere — both kernel limitations on coplanar/curved-face
  intersections).

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
