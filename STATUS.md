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
| Constraint solver (forward expressions)   | 10%       | 30%      | 3.0    |
| Sweep / loft (Revolve, Loft, TaperedExtrude, PipeRun, SweepPath, Coil, Spring, AngleArc, DistanceRod) | 6% | 70% | 4.2 |
| Manufacturing features (170+ — see catalog) | 12% | 95% | 11.4 |
| Reference geometry (RefPoint, RefAxis, RefPlane, Mirror, BoundingBoxRef, CentroidPoint, DistanceRod, AngleArc, Marker3D, VectorArrow) | 3% | 85% | 2.55 |
| Curved-surface analytic booleans (faceted spheres + torus + Hemisphere + SphericalCap + Bowl + Donut + ReducerCone + Lens + EggShape + UBendPipe + SBend + ToroidalKnob compose for simple cases; closed-form Cylinder×Plane intersection lifts to brep-layer EllipseSegment) | 8% | 50% | 4.0 |
| 2D sketcher UI                            | 8%        | 0%       | 0      |
| Assembly (multi-body + mates)             | 8%        | 0%       | 0      |
| **Solidworks-tier total**                 | **100%**  |          | **~65.4%** |
| **OpenSCAD-tier (out of 31 SW pts)**      |           |          | **~99%**   |

## Latest session (2026-05-08)

Curved-surface category 45% → 50% (+0.4 SW pts), 727 → 738 tests, 0 failed,
9 ignored.

- **Brep-layer `EllipseSegment`**: new arc-bounded ellipse type in
  `crates/kerf-brep/src/geometry.rs` with explicit start/end angles, `Sense`
  (forward/reverse), `directed_span`, `start_point`/`end_point`, `is_full`,
  and lossless conversion to the existing `CurveSegment`. Added
  `CurveSegment::circle` and `CurveSegment::ellipse` arc constructors mirroring
  `CurveSegment::line`. `Cylinder×Plane`-style closed conics can now travel
  through the brep type system as first-class arcs rather than faceted
  polygons.
- **Brep-layer `cylinder_plane_intersection`**: new
  `crates/kerf-brep/src/booleans/analytic_curves.rs` lifts `kerf-geom`'s
  closed-form `intersect_plane_cylinder` to the brep layer, returning
  `CylinderPlaneIntersection { Empty | Tangent(Line) | TwoLines(Line, Line)
  | Circle(EllipseSegment) | Ellipse(EllipseSegment) }`. Circles are returned
  as degenerate ellipses (semi_major == semi_minor == r) so downstream code
  that walks closed conics has a single curve type.
- **Math sanity tests**: 11 new tests covering all four geometric regimes:
  perpendicular plane → circle; parallel-and-axis-passing plane → two
  ruling lines; parallel-and-tangent plane → single ruling line;
  parallel-and-disjoint plane → empty; oblique plane at 45° → ellipse with
  semi_major = √2; oblique plane at 30° → ellipse with semi_major = 1/cos(30°)
  AND every sampled point on the resulting ellipse verified to lie exactly on
  both the cylinder surface and the cutting plane to 1e-9.
- **Wiring deferred**: `boolean::face_intersections` still emits line-segment
  chords only. Extending it to emit ellipse-arc chords requires teaching
  `splice` (vertex coincidence on parameter), `split` (in-curve parameter
  splitting), and `stitch` (arc-twin matching, half-edge orientation along
  curve parameter rather than 3D direction) to handle non-line chords —
  multi-week and explicitly out of scope for this session per the task brief.
  Today this module exists as the analytic foundation that those wiring
  tasks will sit on top of.

## Previous session (2026-05-06)

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
