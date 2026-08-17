# Kerf B-rep Kernel — Weekly Status

## 2026-08-17 (session: feat/cone-faceted)

### Scorecard

| Metric | Before | After | Delta |
|--------|--------|-------|-------|
| kerf-brep unit tests | 127 | 139 | +12 |
| Total workspace tests | 329 | 341 | +12 |
| Test failures | 0 | 0 | — |
| Primitives | 10 | 11 | +1 |

### What shipped

**`cone_faceted(radius, height, n)`** — an n-gon pyramid (faceted cone) with
fully planar geometry.

- `V = n+1`, `E = 2n`, `F = n+1`. Euler: (n+1) − 2n + (n+1) = 2 ✓
- 1 base n-gon face (outward normal −z) + n lateral triangle faces.
- All faces: `SurfaceKind::Plane`. All edges: `CurveKind::Line`.
- Phase offset of π/n on base ring → axis-aligned square for n = 4.
- Topology built directly via `build_*` operators (same pattern as `cone.rs`
  and `torus.rs`), not Euler operators, to allow the precise twin assignments
  needed for the star-shaped vertex connectivity at the apex.

**12 new tests** (all in `primitives/cone_faceted.rs`):

| Test | What it checks |
|------|----------------|
| `triangle_pyramid_topology` | V/E/F/shell counts for n=3 + validate() |
| `square_pyramid_topology` | V/E/F/shell counts for n=4 + validate() |
| `hex_pyramid_topology` | V/E/F/shell counts for n=6 + validate() |
| `high_poly_pyramid_topology` | V/E/F/shell counts for n=24 + validate() |
| `all_faces_plane_all_edges_line` | geometry kind assertions |
| `triangle_pyramid_volume_matches_analytic` | solid_volume vs. (n·r²h/6)·sin(2π/n) |
| `square_pyramid_volume_matches_analytic` | same for n=4, r=2, h=3 |
| `high_poly_volume_converges_to_cone` | n=256 within 1e-3 of πr²h/3 |
| `json_round_trip_triangle_pyramid` | write_json → read_json preserves V/E/F |
| `json_round_trip_hex_pyramid` | same for n=6 |
| `apex_is_at_origin_plus_height` | apex vertex at (0, 0, h) |
| `base_vertices_lie_on_circle` | all z=0 vertices at distance r from z-axis |

### What didn't ship

The task prompt mentioned `sphere_faceted.rs` and `torus_faceted.rs` as model
files — these don't exist in the repo. A UV-sphere with planar faces would
require a latitude/longitude band construction with O(stacks × slices) faces
and is more complex than the 2-hour budget allows reliably.

### Delta to scorecard

+12 tests, +1 primitive. Scorecard in README.md still says "327 tests" — the
README is not updated here because the test count it lists (327) refers to the
integration test suites only, not the unit tests. The integration suite count
is unchanged.

### Recommended next-week target

**`sphere_faceted(radius, stacks, slices)`** — a UV-sphere built as a fan of
planar polygon faces (polar caps as triangle fans, middle bands as quad strips).
Required topology pattern: N = stacks×slices + 2 vertices, complex but follows
directly from the cone_faceted apex-vertex pattern. Each latitude band is a
ring of quads closed with MEF on a single shared boundary loop.

Alternatively: add **`frustum_faceted(top_r, bot_r, height, n)`** which
generalises `cylinder_faceted` (top_r == bot_r) and `cone_faceted` (bot_r → 0)
into a single n-gon truncated pyramid. The topology is a prism with n quad
lateral faces + 2 n-gon caps — very similar to `extrude_polygon` but with
different radii at top and bottom.
