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
kerf-cad <model.json> <target_id> <output.stl> [--segments N]
```

`--segments` controls tessellation resolution for curved surfaces (default
24, must be ≥3).

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

### Scalars and parameters

Numeric fields accept either a literal `1.5` or a string `"$name"` that is
resolved against `parameters` at evaluation time. Missing parameters produce
a `Parameter` evaluation error rather than silently substituting zero.

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
| `Translate`      | `offset: [x,y,z]`                                                  | `input: <id>`       |
| `Rotate`         | `axis: [x,y,z]`, `angle_deg`, `center: [x,y,z]`                    | `input: <id>`       |
| `Union`          | —                                                                  | `inputs: [<id>, …]` |
| `Intersection`   | —                                                                  | `inputs: [<id>, …]` |
| `Difference`     | —                                                                  | `inputs: [<id>, …]` |

Booleans take 2+ inputs and fold left: `Difference` with inputs `[a, b, c]`
evaluates as `(a − b) − c`.

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
`multi_stage_carve.json`, and `hex_prism.json`.

## Limitations (inherited from kerf)

- **Chained `Difference` with 3+ inputs and `extrude_polygon`-built cutters**
  panics in the boolean pipeline (`non-manifold input to stitch`). Repro and
  diagnosis live in `crates/kerf-brep/tests/chained_diff_repro.rs` —
  briefly: `box_at`-built cutters survive arbitrary chains, but
  `extrude_polygon`-built cutters (which is everything `Cylinder` /
  `ExtrudePolygon` / `Frustum` / `Cone` produces) fail starting at the
  second chained DIFF whenever the A operand has any prior splits. Same
  geometry, different construction path. Union-the-cutters-first does NOT
  help in the current kernel — `Union` of disjoint extrude-built solids
  hits the same path.

  **Practical guidance until the kernel is fixed:**
  - Keep `Difference` to **2 inputs** when any cutter is `Cylinder` /
    `ExtrudePolygon` / `Cone` / `Frustum`.
  - For multi-cut models, model each pocket/hole as its own body
    (`block_a - hole_a`, `block_b - hole_b`, …) and assemble at the end —
    or use `BoxAt` cutters where geometry allows.
- **Cutters that span a torus's hole** are not supported.
- The kernel itself runs against a 1e-9 vertex tolerance; very small
  coplanar offsets may fall outside its degenerate-case handling.

## Stability

The JSON schema is intended to be the public contract. Once `kerf-cad`
ships a non-zero version, breaking changes will require a version bump.
The Rust API is currently `0.x` — expect minor incompatibilities while the
ergonomics settle.
