#!/usr/bin/env python3
"""Software-rasterized shaded STL renderer with Z-buffer + Lambertian shading.

Uses a vectorized numpy rasterizer (~10x faster than a pure Python loop).
No new dependencies beyond numpy + matplotlib (already installed).

Lighting model: two lights in camera space.
  - Key light: slightly above-left of the camera (camera-relative)
  - Fill light: from behind/below (reduces pitch-black faces)
  - Ambient floor: 0.25
"""

import argparse
import os

import numpy as np
from stl import mesh as stlmesh
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def render_shaded(stl_path, out_path, width=800, height=800, title=None, subtitle=None,
                  elev=25, azim=45, bg=(245, 245, 248), color=(170, 180, 200)):
    m = stlmesh.Mesh.from_file(stl_path)
    tris = m.vectors  # shape (N, 3, 3)

    # ── Per-triangle normals (world space) ───────────────────────────────────
    e1 = tris[:, 1] - tris[:, 0]
    e2 = tris[:, 2] - tris[:, 0]
    normals = np.cross(e1, e2)
    norms = np.linalg.norm(normals, axis=1, keepdims=True)
    norms[norms == 0] = 1.0
    normals = normals / norms

    # ── Camera / view setup ──────────────────────────────────────────────────
    pts = tris.reshape(-1, 3)
    mn = pts.min(axis=0)
    mx = pts.max(axis=0)
    center = (mn + mx) / 2.0
    span = (mx - mn).max() * 1.5  # comfortable distance from mesh

    azim_r = np.deg2rad(azim)
    elev_r = np.deg2rad(elev)
    cam_pos = center + span * np.array([
        np.cos(elev_r) * np.cos(azim_r),
        np.cos(elev_r) * np.sin(azim_r),
        np.sin(elev_r),
    ])

    # Look-at basis vectors
    forward = center - cam_pos
    forward = forward / np.linalg.norm(forward)
    world_up = np.array([0.0, 0.0, 1.0])
    right = np.cross(forward, world_up)
    if np.linalg.norm(right) < 1e-9:
        world_up = np.array([0.0, 1.0, 0.0])
        right = np.cross(forward, world_up)
    right = right / np.linalg.norm(right)
    up = np.cross(right, forward)

    # ── Camera-relative lighting ─────────────────────────────────────────────
    # Key light: in camera space, slightly up-left from forward.
    # In world space this is: forward + 0.4*up - 0.3*right (normalized).
    key_light = forward + 0.4 * up - 0.3 * right
    key_light = key_light / np.linalg.norm(key_light)
    # Fill light: from behind and below (softens back-face darkness).
    fill_light = -forward - 0.5 * up + 0.2 * right
    fill_light = fill_light / np.linalg.norm(fill_light)

    # ── Transform all vertices to camera space (vectorized) ──────────────────
    v = tris - cam_pos  # (N, 3, 3)
    cam_tris = np.stack([
        v @ right,    # x in camera space  (N, 3)
        v @ up,       # y in camera space  (N, 3)
        v @ forward,  # z = depth          (N, 3)
    ], axis=-1)  # → (N, 3, 3)

    # ── Orthographic projection to screen pixels ─────────────────────────────
    cam_pts = cam_tris.reshape(-1, 3)
    cmn = cam_pts[:, :2].min(axis=0)
    cmx = cam_pts[:, :2].max(axis=0)
    pad = (cmx - cmn).max() * 0.05
    cmn -= pad
    cmx += pad
    side = max(cmx[0] - cmn[0], cmx[1] - cmn[1])
    cctr = (cmn + cmx) / 2.0

    # screen_tris: (N, 3, 3) — (px, py, depth)
    screen_tris = np.zeros_like(cam_tris)
    screen_tris[:, :, 0] = (cam_tris[:, :, 0] - cctr[0]) / side * width + width / 2.0
    screen_tris[:, :, 1] = -(cam_tris[:, :, 1] - cctr[1]) / side * height + height / 2.0
    screen_tris[:, :, 2] = cam_tris[:, :, 2]

    # ── Lambertian shading (key + fill + ambient) ────────────────────────────
    key_intensity = 0.65
    fill_intensity = 0.25
    ambient = 0.25

    key_contrib = np.maximum(0.0, normals @ key_light) * key_intensity
    fill_contrib = np.maximum(0.0, normals @ fill_light) * fill_intensity
    shades = ambient + key_contrib + fill_contrib  # (N,)  range ~[0.25, 1.15]
    # Cap at 1.0 for realism; slight overdrive is acceptable.
    shades = np.minimum(shades, 1.0)

    base_color = np.array(color, dtype=np.float32)
    # Precompute per-triangle colors: (N, 3) uint8
    tri_colors = np.clip(base_color[None, :] * shades[:, None], 0, 255).astype(np.uint8)

    # ── Rasterize ────────────────────────────────────────────────────────────
    image = np.full((height, width, 3), bg, dtype=np.uint8)
    zbuffer = np.full((height, width), np.inf, dtype=np.float32)

    # Sort back-to-front; Z-buffer handles correctness, sorting reduces overdraw.
    avg_depth = cam_tris[:, :, 2].mean(axis=1)
    order = np.argsort(avg_depth)[::-1]  # far first

    for idx in order:
        s = screen_tris[idx]  # (3, 3): [px, py, depth]
        _rasterize_tri_vec(image, zbuffer, s, tri_colors[idx])

    # ── Output via matplotlib ─────────────────────────────────────────────────
    fig = plt.figure(figsize=(8, 8.5), dpi=120)
    ax = fig.add_subplot(111)
    ax.imshow(image, interpolation="nearest")
    ax.axis("off")
    if title:
        fig.suptitle(title, fontsize=15, y=0.96, weight="bold")
    if subtitle:
        fig.text(0.5, 0.91, subtitle, ha="center", fontsize=10.5, color="#444")
    fig.text(
        0.5, 0.04,
        f"{tris.shape[0]} triangles • {os.path.basename(stl_path)}",
        ha="center", fontsize=8.5, color="#777", family="monospace",
    )
    plt.tight_layout(rect=[0, 0.05, 1, 0.9])
    plt.savefig(out_path, bbox_inches="tight", facecolor="white")
    plt.close(fig)
    print(f"wrote {out_path}")


def _rasterize_tri_vec(image, zbuffer, screen_verts, color):
    """Rasterize a single triangle using vectorized numpy operations."""
    h, w = image.shape[:2]
    s0, s1, s2 = screen_verts

    # Bounding box clipped to image
    xmin = max(0, int(np.floor(min(s0[0], s1[0], s2[0]))))
    xmax = min(w - 1, int(np.ceil(max(s0[0], s1[0], s2[0]))))
    ymin = max(0, int(np.floor(min(s0[1], s1[1], s2[1]))))
    ymax = min(h - 1, int(np.ceil(max(s0[1], s1[1], s2[1]))))

    if xmin > xmax or ymin > ymax:
        return

    denom = (s1[1] - s2[1]) * (s0[0] - s2[0]) + (s2[0] - s1[0]) * (s0[1] - s2[1])
    if abs(denom) < 1e-9:
        return

    # Build pixel grid over the bounding box
    px = np.arange(xmin, xmax + 1, dtype=np.float32)
    py = np.arange(ymin, ymax + 1, dtype=np.float32)
    gx, gy = np.meshgrid(px, py)  # each (rows, cols)

    # Barycentric coordinates
    w0 = ((s1[1] - s2[1]) * (gx - s2[0]) + (s2[0] - s1[0]) * (gy - s2[1])) / denom
    w1 = ((s2[1] - s0[1]) * (gx - s2[0]) + (s0[0] - s2[0]) * (gy - s2[1])) / denom
    w2 = 1.0 - w0 - w1

    inside = (w0 >= 0) & (w1 >= 0) & (w2 >= 0)
    if not inside.any():
        return

    depth = w0 * s0[2] + w1 * s1[2] + w2 * s2[2]

    # Z-buffer test
    region_z = zbuffer[ymin:ymax + 1, xmin:xmax + 1]
    update = inside & (depth < region_z)
    if not update.any():
        return

    region_z[update] = depth[update]
    zbuffer[ymin:ymax + 1, xmin:xmax + 1] = region_z

    region_img = image[ymin:ymax + 1, xmin:xmax + 1]
    region_img[update] = color
    image[ymin:ymax + 1, xmin:xmax + 1] = region_img


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="Software-rasterized shaded STL renderer")
    ap.add_argument("stl_path")
    ap.add_argument("out_path")
    ap.add_argument("--title", default=None)
    ap.add_argument("--subtitle", default=None)
    ap.add_argument("--elev", type=float, default=25)
    ap.add_argument("--azim", type=float, default=45)
    ap.add_argument("--width", type=int, default=800)
    ap.add_argument("--height", type=int, default=800)
    args = ap.parse_args()
    render_shaded(
        args.stl_path, args.out_path,
        width=args.width, height=args.height,
        title=args.title, subtitle=args.subtitle,
        elev=args.elev, azim=args.azim,
    )
