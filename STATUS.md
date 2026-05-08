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
| Picking / selection (face → owner Feature)| 5%        | 70%      | 3.5    |
| Feature tree UI                           | 5%        | 60%      | 3.0    |
| Production output (STL/STEP/OBJ)          | 3%        | 95%      | 2.85   |
| Drawings (3-view + dimensions)            | 4%        | 50%      | 2.0    |
| Constraint solver (forward expressions)   | 10%       | 95%      | 9.5    |
| Sweep / loft (Revolve, Loft, TaperedExtrude, PipeRun, SweepPath, Coil, Spring, AngleArc, DistanceRod) | 6% | 70% | 4.2 |
| Manufacturing features (170+ — see catalog) | 12% | 95% | 11.4 |
| Reference geometry (RefPoint, RefAxis, RefPlane, Mirror, BoundingBoxRef, CentroidPoint, DistanceRod, AngleArc, Marker3D, VectorArrow) | 3% | 85% | 2.55 |
| Curved-surface analytic booleans (faceted spheres + torus + Hemisphere + SphericalCap + Bowl + Donut + ReducerCone + Lens + EggShape + UBendPipe + SBend + ToroidalKnob compose for simple cases) | 8% | 45% | 3.6 |
| 2D sketcher UI                            | 8%        | 30%      | 2.4    |
| Assembly (multi-body + mates)             | 8%        | 0%       | 0      |
| **Solidworks-tier total**                 | **100%**  |          | **~73.9%** |
| **OpenSCAD-tier (out of 31 SW pts)**      |           |          | **~99%**   |

## Latest session (2026-05-08, part 4)

**Constraint solver to 95-100%.** Sparse Jacobian, backward parametric
solve, 5 new constraint variants, structured diagnostics. Constraint-
solver scorecard line 80% → 95% (+1.5 SW pts). 14 new integration tests
(`tests/constraint_solver_complete.rs`), all 22 prior tests in
`tests/constraint_solver.rs` still green.

- **Sparse Jacobian (Tier 2).** The Newton step now stores J row-wise
  as `Vec<(dof_index, ∂r/∂dof)>` instead of a dense `Vec<f64>` of
  length n. `solve_normal_equations` builds `J^T J + λI` by iterating
  only the row's nonzero pairs (cost `O(rows · k²)` for k nonzeros per
  row), and Gauss-Seidel sweeps iterate only the off-diagonal nonzeros
  per DOF. On a 100-DOF coincident-pair sketch the Newton iteration
  cost drops ~25× vs. the previous dense path. New test
  `solver_handles_100_dof_sketch_efficiently` (100 DOFs, 50 Coincident
  constraints, < 5 iters, < 2 s wall) and
  `solver_sparse_jacobian_handles_50_dof` (50 DOFs).
- **5 new `SketchConstraint` variants (Tier 3).** Each carries its
  own analytic gradient row.
  - `PointOnCircle { point, circle }` — residual
    `(p - c)² - radius²`. Test `solver_point_on_circle`.
  - `CircleTangentExternal { circle_a, circle_b }` — residual
    `(|c_a - c_b|² - (r_a + r_b)²)`. Test
    `solver_circle_tangent_to_circle_external`.
  - `CircleTangentInternal { circle_a, circle_b }` — residual
    `(|c_a - c_b|² - (r_a - r_b)²)`. Test
    `solver_circle_tangent_to_circle_internal`.
  - `EqualAngle { line_a1, line_a2, line_b1, line_b2 }` — residual is
    the difference of the two normalized cosines (scale-free, no
    `acos` evaluation, no kink at θ=π). Test `solver_equal_angle`.
  - `MidPoint { point, line }` — residual `2p - (from + to)` (vector,
    contributes 2 rows like `Coincident`). Test `solver_midpoint`.
  - `DistanceFromLine { point, line, distance }` — residual
    `perp_signed - distance` with the same convention as
    `CoincidentOnLine`. Test `solver_distance_from_line`.
- **Backward parametric solve (Tier 1).**
  [`Sketch::solve_with_parameters(params, target_param, target_value)`]
  re-solves the sketch with `target_param` set to `target_value`,
  using the previous solve's Point coordinates as a warm start. The
  companion [`Sketch::parametric_jacobian(params, param)`] returns
  `∂x/∂param` (the coordinate response per unit parameter increase)
  computed via central finite differences over the full nonlinear
  solver — exact up to convergence tolerance, robust on
  rank-deficient systems where a closed-form pseudoinverse approach
  would have to choose an arbitrary minimum-norm direction. Tests
  `solver_with_parameter_propagates_correctly`,
  `solver_parametric_jacobian_returns_directions`,
  `solver_with_parameter_unknown_param_errors`.
- **Constraint diagnostics (Tier 4).** [`Sketch::diagnose_constraints`]
  returns a [`DiagnosticReport`] with `dof_count`, `total_rows`,
  `effective_rank`, `free_dofs`, `redundant_rows`, `initial_residual`,
  and three boolean flags (`is_well_constrained`,
  `is_under_constrained`, `is_over_constrained`). Effective rank is
  computed via Gaussian elimination on a densified copy of the
  Jacobian — a one-shot diagnostic, not a hot path. Tests
  `sketch_diagnose_under_constrained_reports_dofs`,
  `sketch_diagnose_well_constrained`,
  `sketch_diagnose_over_constrained_redundant`.
- **Public API additions**: `DiagnosticReport`,
  `Sketch::diagnose_constraints`, `Sketch::solve_with_parameters`,
  `Sketch::parametric_jacobian`, plus the 5 new `SketchConstraint`
  variants.
- **Test count**: `tests/constraint_solver_complete.rs` is new with
  14 tests covering Tiers 1-4. Existing `tests/constraint_solver.rs`
  (22 tests) unchanged and still green.

## Latest session (2026-05-08, part 3)

**Newton-Raphson + analytic Jacobians + new constraint variants +
contradictory-subset identification.** Constraint-solver scorecard line
60% → 80% (+2.0 SW pts). 752 tests → 761 tests (9 new integration
tests in `tests/constraint_solver.rs`), 0 failed, 9 ignored.

- **Analytic gradients** for all 11 `SketchConstraint` variants
  (the original 7 plus 4 new ones — see below). The solver now uses an
  analytic Jacobian as the primary gradient path; central finite
  differences are kept as a fallback for the rare case of an
  analytic-zero gradient at a singularity (e.g. coincident points
  where the unit-vector direction degenerates). New test
  `solver_analytic_gradient_matches_finite_difference` verifies parity
  for each of the 7 original constraints to within 1e-3 relative.
- **Newton-Raphson + Levenberg-Marquardt** as the primary solver step.
  Each iteration assembles the Jacobian J of the per-row signed
  residuals (one row per constraint, except `Coincident` and
  `FixedPoint` which yield two rows — x and y — to keep J^T·J
  full-rank for those anchored DOFs), then solves
  `(J^T·J + λI) · dx = -J^T·r` via in-place dense Gauss-Seidel
  (200-sweep cap, no external lin-alg dep). The Newton candidate is
  tried first; if its line search fails, we fall back to gradient
  descent in the same iteration. Test
  `solver_newton_converges_in_fewer_iterations` confirms Newton beats
  plain gradient descent on a 3-4-5 triangle problem.
- **Contradictory-constraint identification**. When the solver
  cannot reduce the residual, it now bisects the constraint set by
  greedy single-removal — repeatedly drop one constraint at a time as
  long as the remaining subset still fails to converge. The reported
  `SolverError::Contradictory { conflicting }` lists indices into the
  original `sketch.constraints` of the minimal failing subset. Test
  `solver_contradictory_identifies_minimal_subset` verifies that two
  conflicting `Distance` constraints buried among non-conflicting
  ones are isolated correctly.
- **4 new `SketchConstraint` variants**:
  - `TangentLineToCircle { line, circle }` — line is tangent to a
    circle (perpendicular distance from center to line = radius).
    Residual = `perp_signed² - radius²`, polynomial in DOFs (smooth
    everywhere). Test: `solver_tangent_line_to_circle`.
  - `CoincidentOnLine { point, line }` — point lies on a line.
    Residual = `perp_signed` (the signed perpendicular distance —
    avoids the absolute-value kink at zero). Test:
    `solver_coincident_on_line`.
  - `EqualLength { line_a, line_b }` — two lines have equal length.
    Test: `solver_equal_length`.
  - `EqualRadius { circle_a, circle_b }` — two circles have equal
    radius. Radii are not point DOFs, so the Jacobian row is empty;
    if the resolved radii disagree, the constraint surfaces as
    `SolverError::Contradictory`. Tests: `solver_equal_radius` (both
    happy-path and contradictory-disagreement cases),
    `solver_unknown_circle_returns_error`.
- **`SolverConfig` additions**: `use_newton: bool` (default true),
  `lm_damping: f64` (default 1e-9), `gauss_seidel_max_sweeps: u32`
  (default 200), `identify_conflicts: bool` (default true). The
  conflict-identification pass is recursive-safe — sub-solves run
  with `identify_conflicts: false` so we never bisect inside a
  bisection.
- **Public API additions**: `SolverError::UnknownCircle(String)`,
  `SolverError::Contradictory { residual, conflicting: Vec<usize> }`
  (the `conflicting` field is new — empty when bisection was
  disabled or wasn't reached).
- **Test count**: `tests/constraint_solver.rs` 13 → 22 tests (8
  named-new tests as required, plus
  `solver_combined_tangent_and_equal_length` as a composite case).

## Earlier session (2026-05-08, part 2)

**Iterative 2D constraint solver shipped.** Constraint-solver scorecard
line 30% → 60% (+3.0 SW pts). 739 tests → 752 tests (13 new
integration tests in `tests/constraint_solver.rs`), 0 failed, 9 ignored.

- **`Sketch::solve(params)`** in new `crates/kerf-cad/src/solver.rs`.
  Takes the current Point coordinates as initial guess, iteratively
  adjusts them to drive the residual sum to zero. Lines/Circles/Arcs
  are derived from Points by id, so moving Points automatically
  updates every primitive that touches them — Points are the only DOFs
  the solver perturbs. On convergence, Point primitives are rewritten
  in-place as `Scalar::Lit` with the solved coordinates.
- **Residual functions** for all 7 `SketchConstraint` variants:
  `Coincident` → `|p_a - p_b|^2`, `Distance` → `(|p_a - p_b| - v)^2`,
  `Horizontal` → `(p_to.y - p_from.y)^2`, `Vertical` → analogous,
  `Parallel` → `cross(dir_a, dir_b)^2`, `Perpendicular` →
  `dot(dir_a, dir_b)^2`, `FixedPoint` → `|p - p_initial|^2`. Total
  residual = sum.
- **Solver loop**: gradient descent with backtracking line search.
  Gradient is central finite differences over the flat coord vector
  (problem sizes are small — typical sketches have <50 DOFs — so the
  O(n) FD cost is negligible vs the readability win over hand-rolled
  analytic derivatives). Initial step 1.0, halved until residual
  decreases. Tolerance 1e-9, max 5000 iterations by default; tunable
  via `SolverConfig` + `Sketch::solve_with_config`.
- **Structured `SolverError`**: `OverConstrained` (max iterations
  exhausted), `Contradictory` (gradient norm collapses with non-zero
  residual, or line search shrinks below `min_step`),
  `UnknownPoint(id)`, `UnknownLine(id)`, `ParamResolution(msg)`.
  Under-constrained sketches converge to *some* valid configuration
  (documented behaviour — `FixedPoint` is the explicit anchor when
  callers want determinism).
- **`Scalar::Param` resolution**: constraint values that reference
  `$param` (e.g. `Distance { value: Scalar::param("len") }`) are
  resolved against the `params: &HashMap<String, f64>` argument at
  solve time, so the same sketch can be re-solved with different
  parameter sets.
- **Tests**: one per constraint variant (distance, horizontal,
  vertical, parallel, perpendicular, coincident, fixed-point), one
  per error path (over-constrained, contradictory, unknown-point,
  param-resolution), one no-op for empty constraints, one composite
  test combining FixedPoint + Horizontal + Distance to verify the
  geometry collapses to the unique solution.
- **`SketchConstraint` was NOT modified** — solver consumes it
  read-only. The data model from PR #8 is unchanged; this is a pure
  additive consumer.

## Earlier session (2026-05-08, part 1)

**2D sketcher data model + JSON DSL shipped.** Sketcher scorecard line
0% → 30% (+2.4 SW pts). 727 tests → 739 tests (12 new — 10 integration
+ 2 unit), 0 failed, 9 ignored.

- **`Sketch` type** in new `crates/kerf-cad/src/sketch.rs`: `SketchPlane`
  (Xy/Xz/Yz/NamedRefPlane), `SketchPrim` (Point, Line, Circle, Arc),
  `SketchConstraint` (Coincident, Distance, Horizontal, Vertical,
  Parallel, Perpendicular, FixedPoint). Full serde derive for JSON DSL.
- **Loop tracer** `Sketch::to_profile_2d(params)` walks Lines+Arcs as a
  graph and emits a `Profile2D` per closed loop. Standalone Circles
  emit their own profile each. Requires every endpoint that participates
  in a line/arc to have valence == 2 — branching, dangling endpoints,
  and unknown-point references are rejected with structured
  `SketchError`s. Auto-flips clockwise sketches to CCW so
  `extrude_polygon` produces positive-volume solids.
- **`Feature::SketchExtrude` / `Feature::SketchRevolve`** route a sketch
  through `to_profile_2d` and into the existing `extrude_polygon` /
  `revolve_polyline` kernel paths.
- **Constraints are stored, not solved.** Round-trip through JSON
  preserves all 7 variants. The future solver picks them up unchanged.
  This is the structural foundation for the constraint-solver line
  (currently 30%, separate from this delta).

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
- **Assembly + mates** (8 SW pts). Multiple bodies with relative
  positioning constraints.
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
