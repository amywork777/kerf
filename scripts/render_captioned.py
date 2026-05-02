#!/usr/bin/env python3
"""Render every gallery STL with a descriptive 2-line caption explaining what to look for.

Uses the software-rasterized shaded renderer (Z-buffer + Lambertian shading)
instead of matplotlib's wireframe-style 3D rendering.
"""

import os

from render_shaded import render_shaded as render


GALLERY = "/tmp/claude/kerf_gallery"
OUT = "/Users/amy/code/kerf/screenshots"
os.makedirs(OUT, exist_ok=True)


# (stem, title, subtitle, elev, azim)
PLAN = [
    ("01_box",
     "M5 — box(2, 2, 2)",
     "An axis-aligned cube. 8 vertices, 12 edges, 6 quad faces. Built via mvfs+mev+mef Euler operators.",
     25, 35),
    ("02_prism",
     "M5b — extrude_polygon(triangle, +z)",
     "A triangular prism extruded from a 3-vertex profile. Generalizes box_ to any convex N-gon.",
     22, 35),
    ("03_cylinder",
     "M13 — cylinder(r=1, h=2)",
     "Top and bottom disks (planar caps) connected by a cylindrical lateral surface. Topology has self-loop circles + 1 seam edge.",
     20, 35),
    ("04_cone",
     "M14 — cone(r=1, h=2)",
     "Apex point at top tapering to a circular base. 2 vertices, 2 edges, 2 faces. Validates Euler invariant = 2.",
     22, 35),
    ("05_sphere",
     "M15 — sphere(r=1)",
     "A closed sphere with 1V/0E/1F topology — the empty-loop case. Tessellated as a lat-long grid.",
     15, 30),
    ("06_torus",
     "M16 — torus(R=1.5, r=0.4)",
     "Genus-1 surface (Euler char 0). Topology: 1 vertex + 2 self-loop edges (meridian + equator) + 1 face. Validator handles non-zero genus.",
     45, 30),
    ("07_frustum",
     "M17 — frustum(top=0.5, bot=1.5, h=2)",
     "Truncated cone — narrower at top, wider at base. Uses Cone surface with apex computed off-solid.",
     20, 35),
    ("08_revolve_vase",
     "M20 — revolve_polyline(vase profile)",
     "6-point profile revolved around the z-axis. Each lateral segment is a Cone (frustum) or Cylinder. 5 stacked surfaces.",
     12, 30),
    ("10_union_two_boxes",
     "M6e/M8 — box ∪ shifted_box (Union)",
     "Two unit cubes overlapping by 50% along x. Union output is the merged outer surface as a connected B-rep solid.",
     22, 40),
    ("11_intersection_two_boxes",
     "M6e/M8 — box ∩ shifted_box (Intersection)",
     "Two unit cubes overlapping by 50%. Intersection is the overlap region, shaped as a smaller box.",
     22, 40),
    ("12_corner_cut_step",
     "M11 — block.difference(corner_cutter)",
     "4×4×4 block with a 2×2×2 cube notched out of one corner. Required interior-endpoint support (mev-tail trick).",
     25, 50),
    ("13_hollow_box",
     "M12 — big.difference(small_inside)",
     "Outer 4×4×4 shell + flipped inner 2×2×2 shell (visible through wireframe). Multi-shell stitch produces 2 disconnected components.",
     20, 30),
    ("14_recursive_intersection",
     "M8 — (a ∩ b) ∩ c (recursive boolean)",
     "Three boxes intersected pairwise. Validates that boolean_solid output can feed back into another boolean.",
     22, 40),
    ("cad_bracket",
     "CAD — L-bracket",
     "An L-shaped bracket built from boolean operations. Classic CAD primitive composition example.",
     25, 45),
    ("cad_pawn",
     "CAD — chess pawn",
     "A chess pawn built by revolving a profile around the z-axis. Tests revolve + boolean combination.",
     15, 35),
    ("cad_pyramid",
     "CAD — square pyramid",
     "A square pyramid. Tests triangular face tessellation and apex topology.",
     25, 45),
]


for stem, title, subtitle, elev, azim in PLAN:
    stl = f"{GALLERY}/{stem}.bin.stl"
    png = f"{OUT}/{stem}.png"
    if not os.path.exists(stl):
        print(f"SKIP {stem}")
        continue
    render(stl, png, title=title, subtitle=subtitle, elev=elev, azim=azim)


# Also re-render the primitive zoo with caption.
zoo_stl = "/tmp/claude/kerf_zoo.bin.stl"
zoo_png = f"{OUT}/00_primitive_zoo.png"
if os.path.exists(zoo_stl):
    render(
        zoo_stl, zoo_png,
        title="M18 — primitive zoo (every primitive in one STL)",
        subtitle="Left→right: cone, cylinder, triangular prism, frustum (front), box, torus (front), sphere. 1364 triangles total.",
        elev=25, azim=45,
    )

print(f"\nAll captioned renders in {OUT}/")
