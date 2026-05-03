# kerf-cad

A parametric CAD layer over the [kerf](../..) B-rep kernel: declarative model
trees, JSON save/load, CLI evaluator.

`kerf` gives you primitives, booleans, and a validated B-rep kernel. `kerf-cad`
gives you the layer above that: a way for humans (or other programs) to
describe a CAD model as a tree of operations stored as data, then evaluate it
to a `Solid`.

## Quick start (Rust)

```rust
use kerf_brep::solid_volume;
use kerf_cad::{lits, Feature, Model, Scalar};

let m = Model::new()
    .with_parameter("plate_x", 100.0)
    .add(Feature::Box {
        id: "body".into(),
        extents: [Scalar::param("plate_x"), Scalar::lit(60.0), Scalar::lit(8.0)],
    })
    .add(Feature::Cylinder {
        id: "hole".into(),
        radius: Scalar::lit(2.5),
        height: Scalar::lit(12.0),
        segments: 16,
    })
    .add(Feature::Translate {
        id: "hole_pos".into(),
        input: "hole".into(),
        offset: lits([10.0, 30.0, -2.0]),
    })
    .add(Feature::Difference {
        id: "out".into(),
        inputs: vec!["body".into(), "hole_pos".into()],
    });

let solid = m.evaluate("out").unwrap();
println!("volume = {}", solid_volume(&solid));
```

## Quick start (CLI)

```sh
$ cargo run -p kerf-cad -- examples/bracket.json out /tmp/bracket.stl
$ ls -la /tmp/bracket.stl
```

CLI usage:

```
kerf-cad <model.json> <target_id> <output.{stl,obj,step}> [--segments N]
```

Output format is chosen from the file extension:

| Ext            | Format                       |
|----------------|------------------------------|
| `.stl`         | binary STL (mesh)            |
| `.obj`         | Wavefront OBJ (mesh)         |
| `.step` / `.stp` | STEP AP203/214 (B-rep)     |

`--segments` controls tessellation resolution for curved surfaces in mesh
formats (default 24, must be ≥3). It is ignored for STEP, which writes the
exact B-rep.

## JSON schema

A `kerf-cad` model is a JSON object with two top-level fields:

```json
{
  "parameters": { "plate_x": 100.0, "plate_y": 60.0 },
  "features": [ ... ]
}
```

Both are optional. `parameters` is a flat map of name → numeric value;
`features` is an ordered list of feature objects. Each feature has a `kind`
discriminator, an `id` (must be unique), and either parameters (for
primitives) or `input`/`inputs` referencing other ids (for transforms and
booleans).

### Scalars, parameters, and expressions

Numeric fields accept three forms:

- a literal: `1.5`
- a parameter reference: `"$name"` (looked up in `parameters`)
- an expression: any string with arithmetic, parens, `$vars`, and
  builtins, e.g. `"$plate_x / 2 + 5"`, `"$r * sin($theta * 3.14159 / 180)"`,
  `"sqrt($a * $a + $b * $b)"`.

Supported operators: `+ - * /` and parentheses. Supported builtins:
trig: `sin`, `cos`, `tan`, `asin`, `acos`, `atan`, `atan2(y, x)`;
algebra: `sqrt`, `pow(a, b)`, `exp`, `ln`, `log(value, base)`,
`abs`, `sign`, `hypot(a, b)`, `mod(a, b)`;
selection: `min(a, b)`, `max(a, b)`, `clamp(v, lo, hi)`,
`if_pos(cond, t, e)` (returns `t` when `cond > 0`, else `e`);
rounding: `floor`, `ceil`, `round`;
zero-arg constants: `pi()`, `tau()`, `e()`. Missing parameters and
parse errors surface as `Parameter` evaluation errors with the
offending name.

### Feature catalog

| `kind`           | Required fields                                                    | Inputs              |
|------------------|--------------------------------------------------------------------|---------------------|
| `Box`            | `extents: [x,y,z]`                                                 | —                   |
| `BoxAt`          | `extents: [x,y,z]`, `origin: [x,y,z]`                              | —                   |
| `Cylinder`       | `radius`, `height`, `segments` (integer ≥ 3)                       | —                   |
| `Sphere`         | `radius`                                                           | —                   |
| `Torus`          | `major_radius`, `minor_radius`                                     | —                   |
| `Cone`           | `radius`, `height`                                                 | —                   |
| `Frustum`        | `top_radius`, `bottom_radius`, `height`                            | —                   |
| `ExtrudePolygon` | `profile: { points: [[x,y], ...] }`, `direction: [x,y,z]`          | —                   |
| `Loft`           | `bottom: { points }`, `top: { points }` (same length ≥3), `height` — skin between two parallel polygons | — |
| `TaperedExtrude` | `profile: { points }`, `height`, `top_scale` — extrude with the top scaled around the centroid (draft angle) | — |
| `Revolve`        | `profile: { points: [[x,z], ...] }` — revolved around z-axis        | —                   |
| `Tube`           | `outer_radius`, `inner_radius`, `height`, `segments` (≥3)          | —                   |
| `HollowBox`      | `extents: [x,y,z]`, `wall_thickness`                               | —                   |
| `HollowCylinder` | `outer_radius`, `inner_radius`, `height`, `end_thickness`, `segments` (≥3) | —           |
| `Slot`           | `length` (≥0), `radius`, `height`, `segments` (≥3) — stadium-shape extrude in xy | —     |
| `Wedge`          | `width`, `depth`, `height` — right-triangular prism (legs along x and z, extrudes along y) | — |
| `RegularPrism`   | `radius`, `height`, `segments` (≥3) — n-gon prism (alias of `cylinder_faceted`) | —      |
| `Pyramid`        | `radius` (base circumradius), `height`, `segments` (≥3) — n-sided pyramid via faceted-cone primitive | — |
| `CylinderAt`     | `base: [x,y,z]`, `axis: "x"\|"y"\|"z"`, `radius`, `height`, `segments` (≥3) — cylinder positioned at `base` with cap-axis along `axis` | — |
| `Star`           | `points` (≥3), `outer_radius`, `inner_radius`, `height` — n-pointed star prism extruded along +z | — |
| `TubeAt`         | `base: [x,y,z]`, `axis: "x"\|"y"\|"z"`, `outer_radius`, `inner_radius`, `height`, `segments` (≥3) | — |
| `LBracket`       | `width`, `height`, `thickness`, `depth` — L cross-section extruded along +z | — |
| `UChannel`       | `width`, `height`, `thickness`, `depth` — U cross-section (web at y=0, legs along +y) extruded along +z | — |
| `TBeam`          | `flange_width`, `flange_thickness`, `web_thickness`, `total_height`, `depth` — T cross-section extruded along +z | — |
| `IBeam`          | `flange_width`, `flange_thickness`, `web_thickness`, `total_height`, `depth` — I/H cross-section (symmetric flanges) extruded along +z | — |
| `Bolt`           | `head_inscribed_radius` (apothem), `head_thickness`, `shaft_radius`, `shaft_length`, `segments` (≥3) — hex-head bolt along +z | — |
| `CapScrew`       | `head_radius`, `head_thickness`, `shaft_radius`, `shaft_length`, `segments` (≥3) — socket-head cap screw along +z | — |
| `Nut`            | `inscribed_radius` (apothem), `bore_radius`, `thickness`, `segments` (≥3) — hex nut along +z | — |
| `Washer`         | `outer_radius`, `inner_radius`, `thickness`, `segments` (≥3) — flat washer along +z | — |
| `RoundBoss`      | `base: [x,y,z]`, `radius`, `height`, `segments` (≥3) — cylindrical boss positioned at `base` | — |
| `RectBoss`       | `corner: [x,y,z]`, `extents: [x,y,z]` — rectangular boss positioned at `corner` (alias of `BoxAt`) | — |
| `Translate`      | `offset: [x,y,z]`                                                  | `input: <id>`       |
| `Scale`          | `factor` (>0) — uniform scale around origin                         | `input: <id>`       |
| `Rotate`         | `axis: [x,y,z]`, `angle_deg`, `center: [x,y,z]`                    | `input: <id>`       |
| `Mirror`         | `plane_origin: [x,y,z]`, `plane_normal: [x,y,z]` (¹)                | `input: <id>`       |
| `CornerCut`      | `corner: [x,y,z]`, `extents: [x,y,z]` — subtract a box at a corner  | `input: <id>`       |
| `Fillet`         | `axis: "x"\|"y"\|"z"`, `edge_min: [x,y,z]`, `edge_length`, `radius`, `quadrant: "pp"\|"pn"\|"np"\|"nn"`, `segments` (≥3) (²) | `input: <id>` |
| `Fillets`        | `edges: [{axis, edge_min, edge_length, radius, quadrant, segments}, …]` — multi-edge Fillet via composite cutter union (²) | `input: <id>` |
| `Chamfer`        | `axis`, `edge_min`, `edge_length`, `setback`, `quadrant` — 45° flat cut (²) | `input: <id>` |
| `Counterbore`    | `axis: "x"\|"y"\|"z"`, `top_center: [x,y,z]`, `drill_radius`, `cbore_radius`, `cbore_depth`, `total_depth`, `segments` (≥3) — stepped hole for socket-head fasteners; `top_center` is the center of the cbore opening on the +axis-facing surface, hole goes INTO the body in -axis direction | `input: <id>` |
| `Countersink`    | `axis`, `top_center: [x,y,z]`, `drill_radius`, `csink_radius`, `csink_depth`, `total_depth`, `segments` (≥3) — hole with a conical chamfer at the top, for flat-head fasteners | `input: <id>` |
| `HoleArray`      | `axis`, `start: [x,y,z]`, `offset: [x,y,z]`, `count`, `radius`, `depth`, `segments` (≥3) — drill a linear array of through-pockets | `input: <id>` |
| `BoltCircle`     | `axis`, `center: [x,y,z]`, `bolt_circle_radius`, `count`, `radius`, `depth`, `segments` (≥3) — drill a polar array of through-pockets around `center` | `input: <id>` |
| `HexHole`        | `axis`, `top_center: [x,y,z]`, `inscribed_radius` (apothem), `depth` — drill a hex-shaped through-pocket | `input: <id>` |
| `SquareHole`     | `axis`, `top_center: [x,y,z]`, `side`, `depth` — drill a square through-pocket | `input: <id>` |
| `LinearPattern`  | `count` (≥1), `offset: [x,y,z]`                                    | `input: <id>`       |
| `PolarPattern`   | `count` (≥1), `axis: [x,y,z]`, `center: [x,y,z]`, `total_angle_deg` | `input: <id>`      |
| `Union`          | —                                                                  | `inputs: [<id>, …]` |
| `Intersection`   | —                                                                  | `inputs: [<id>, …]` |
| `Difference`     | —                                                                  | `inputs: [<id>, …]` |

Booleans take 2+ inputs and fold left: `Difference` with inputs `[a, b, c]`
evaluates as `(a − b) − c`.

(¹) `Mirror` reflects geometry AND reverses face loop walks so the
result is a fully-valid solid: volume comes out positive, and the
mirrored body composes cleanly with `Union` of the original to make
symmetric designs.

(²) `Fillet` and `Chamfer` operate on **a single axis-aligned 90° edge** at
a time. `axis` is the edge direction; `edge_min` is one endpoint;
`edge_length` is the distance to the other endpoint along `+axis`;
`quadrant` is two characters indicating which way the body extends from
the edge in the two perpendicular axes (canonical perp-axis order is
`(y, z)` for axis `x`; `(z, x)` for axis `y`; `(x, y)` for axis `z`).
**Stacking multiple `Fillet`s on the same body** works when the two
filleted edges don't share a face (e.g., diagonally opposite z-edges
of a box are fine — they share only the top and bottom faces but the
fillet cutters never meet there). It fails when adjacent edges share
a face that both fillets cut into. The plural `Fillets` form builds
all wedge cutters from the unmodified input and unions them into a
single composite cutter — this works for the same "diagonally
opposite" case as chained `Fillet`, and additionally for any set of
edges whose wedge cutters don't themselves trip the boolean engine.
The four-corner-of-a-box case still fails because the wedge union
hits the same coplanar-face limitation; needs kernel-side work to
fix. Workarounds: fillet only opposite corners, split the body and
union pre-filleted parts, or post-process in your slicer.

### Conventions

- `ExtrudePolygon` profile points must be **counter-clockwise** when viewed
  from the +z side, otherwise the result will have inward-facing normals
  (and `solid_volume` will return a negative value).
- The `direction` vector for `ExtrudePolygon` does not have to be axis-aligned
  but the magnitude is the extrusion length.
- `Cylinder.segments` is structural (it changes the topology), so it is not
  parameterizable. `radius`, `height`, and other measurements are.

### Example: parametric bracket

```json
{
  "parameters": {
    "plate_x":     100.0,
    "plate_y":      60.0,
    "plate_z":       8.0,
    "hole_r":        2.5,
    "hole_height":  12.0,
    "hole_z_start": -2.0,
    "edge_inset":   10.0
  },
  "features": [
    {"kind": "Box", "id": "body", "extents": ["$plate_x", "$plate_y", "$plate_z"]},
    {"kind": "Cylinder", "id": "hole",
      "radius": "$hole_r", "height": "$hole_height", "segments": 16},
    {"kind": "Translate", "id": "hole_left",  "input": "hole",
      "offset": ["$edge_inset", 30.0, "$hole_z_start"]},
    {"kind": "Translate", "id": "hole_right", "input": "hole",
      "offset": [90.0, 30.0, "$hole_z_start"]},
    {"kind": "Difference", "id": "out",
      "inputs": ["body", "hole_left", "hole_right"]}
  ]
}
```

See [examples/](./examples/) for `bracket.json`, `hollow_box.json`,
`multi_stage_carve.json`, `hex_prism.json`, `bolt_circle.json` (`PolarPattern`
driving a 6-bolt-hole circle), `bracket_filleted.json` (two diagonally-
opposite `Fillet`s on a plate), `chamfered_block.json`, `slot.json`,
`hex_nut.json` (a `RegularPrism` body with a centered `Cylinder` thread
bore), and `star.json` (a 5-pointed star).

## Limitations (inherited from kerf)

- **Cutters that span a torus's hole** are not supported.
- The kernel runs against a 1e-9 vertex tolerance; very small coplanar
  offsets may fall outside its degenerate-case handling.

(The previously-documented "chained `Difference` with extrude-built
cutters" panic was fixed in the kerf boolean pipeline — see
`crates/kerf-brep/tests/chained_diff_repro.rs` for the regression suite.)

## Stability

The JSON schema is intended to be the public contract. Once `kerf-cad`
ships a non-zero version, breaking changes will require a version bump.
The Rust API is currently `0.x` — expect minor incompatibilities while the
ergonomics settle.
