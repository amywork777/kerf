#!/usr/bin/env python3
"""Render an STL file to a PNG via matplotlib."""

import argparse
import sys

from stl import mesh as stlmesh
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


def render(stl_path: str, out_path: str, title: str = None, elev: float = 30, azim: float = 45):
    m = stlmesh.Mesh.from_file(stl_path)

    fig = plt.figure(figsize=(8, 8), dpi=120)
    ax = fig.add_subplot(111, projection="3d")
    ax.view_init(elev=elev, azim=azim)

    poly = Poly3DCollection(m.vectors, alpha=0.95)
    poly.set_facecolor((0.65, 0.7, 0.8))
    poly.set_edgecolor((0.2, 0.25, 0.3))
    poly.set_linewidth(0.3)
    ax.add_collection3d(poly)

    # Auto-scale.
    pts = m.vectors.reshape(-1, 3)
    mn = pts.min(axis=0)
    mx = pts.max(axis=0)
    ctr = (mn + mx) / 2.0
    span = (mx - mn).max() / 2.0 * 1.05
    ax.set_xlim(ctr[0] - span, ctr[0] + span)
    ax.set_ylim(ctr[1] - span, ctr[1] + span)
    ax.set_zlim(ctr[2] - span, ctr[2] + span)
    ax.set_box_aspect([1, 1, 1])
    ax.set_axis_off()

    if title:
        ax.set_title(title, fontsize=14, pad=20)

    plt.tight_layout()
    plt.savefig(out_path, bbox_inches="tight", facecolor="white")
    plt.close(fig)
    print(f"wrote {out_path} (triangles: {len(m.vectors)})")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("stl_path")
    ap.add_argument("out_path")
    ap.add_argument("--title", default=None)
    ap.add_argument("--elev", type=float, default=30)
    ap.add_argument("--azim", type=float, default=45)
    args = ap.parse_args()
    render(args.stl_path, args.out_path, args.title, args.elev, args.azim)
