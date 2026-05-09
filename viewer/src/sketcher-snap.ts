/**
 * Pure snap-finding function for the 2D sketcher.
 *
 * No DOM dependencies — safe to import from Node tests.
 */

import type { Sketch, SketchPrim } from "./sketcher.js";

export type SnapKind =
  | "endpoint"
  | "existing-point"
  | "mid"
  | "center"
  | "grid"
  | "horiz"
  | "vert";

export type SnapResult = {
  kind: SnapKind;
  point: { x: number; y: number };
};

/** Priority order: lower index = higher priority. */
const PRIORITY: SnapKind[] = [
  "endpoint",
  "existing-point",
  "mid",
  "center",
  "grid",
  "horiz",
  "vert",
];

function priority(k: SnapKind): number {
  return PRIORITY.indexOf(k);
}

/** Point-to-point distance squared. */
function dist2(ax: number, ay: number, bx: number, by: number): number {
  return (ax - bx) ** 2 + (ay - by) ** 2;
}

/**
 * Find the best snap target for `cursor` given a sketch and viewport.
 *
 * @param cursor    World-space cursor position.
 * @param sketch    Current sketch (primitives + constraints).
 * @param viewport  Provides `scale` (pixels/world-unit) for pixel-space thresholds.
 *                  Also accepts `gridSpacing` and `gridVisible` for grid snap.
 */
export function findSnap(
  cursor: { x: number; y: number },
  sketch: Sketch,
  viewport: {
    scale: number;
    gridSpacing?: number;
    gridVisible?: boolean;
  },
): SnapResult | null {
  const { scale } = viewport;

  // Pixel → world thresholds.
  const snapR = 8 / scale;      // 8 px radius for point-type snaps
  const axisR = 4 / scale;      // 4 px for horizontal/vertical axis-alignment

  let best: SnapResult | null = null;
  let bestPriority = Infinity;
  let bestDist2 = Infinity;

  function consider(kind: SnapKind, px: number, py: number) {
    const d2 = dist2(cursor.x, cursor.y, px, py);
    const p = priority(kind);
    // Keep if closer (by priority first, then distance within same priority).
    if (p < bestPriority || (p === bestPriority && d2 < bestDist2)) {
      bestPriority = p;
      bestDist2 = d2;
      best = { kind, point: { x: px, y: py } };
    }
  }

  // Index existing Points by id for quick lookup.
  const pointMap = new Map<string, { x: number; y: number }>();
  for (const prim of sketch.primitives) {
    if (prim.kind === "Point") {
      pointMap.set(prim.id, { x: prim.x, y: prim.y });
    }
  }

  for (const prim of sketch.primitives) {
    switch (prim.kind) {
      case "Point": {
        const d2 = dist2(cursor.x, cursor.y, prim.x, prim.y);
        if (d2 <= snapR * snapR) {
          // "existing-point" by default; endpoints of lines are promoted to
          // "endpoint" later when we iterate Lines.
          consider("existing-point", prim.x, prim.y);
        }
        break;
      }
      case "Line": {
        const a = pointMap.get(prim.from);
        const b = pointMap.get(prim.to);
        if (!a || !b) break;

        // Endpoints (higher priority than generic existing-point).
        if (dist2(cursor.x, cursor.y, a.x, a.y) <= snapR * snapR) {
          consider("endpoint", a.x, a.y);
        }
        if (dist2(cursor.x, cursor.y, b.x, b.y) <= snapR * snapR) {
          consider("endpoint", b.x, b.y);
        }

        // Mid-point of segment.
        const mx = (a.x + b.x) / 2;
        const my = (a.y + b.y) / 2;
        if (dist2(cursor.x, cursor.y, mx, my) <= snapR * snapR) {
          consider("mid", mx, my);
        }
        break;
      }
      case "Circle":
      case "Arc": {
        const c = pointMap.get(prim.center);
        if (!c) break;
        if (dist2(cursor.x, cursor.y, c.x, c.y) <= snapR * snapR) {
          consider("center", c.x, c.y);
        }
        break;
      }
      default:
        break;
    }
  }

  // Grid snap (if visible and configured).
  if (viewport.gridVisible && viewport.gridSpacing && viewport.gridSpacing > 0) {
    const g = viewport.gridSpacing;
    const gx = Math.round(cursor.x / g) * g;
    const gy = Math.round(cursor.y / g) * g;
    if (dist2(cursor.x, cursor.y, gx, gy) <= snapR * snapR) {
      consider("grid", gx, gy);
    }
  }

  // Horizontal / vertical alignment with any existing Point (lower priority).
  // Only run these if no higher-priority snap was found, to avoid noise.
  if (bestPriority > priority("grid")) {
    for (const prim of sketch.primitives) {
      if (prim.kind !== "Point") continue;
      // Horizontal alignment: cursor is within `axisR` of prim.y → snap y.
      if (Math.abs(cursor.y - prim.y) <= axisR) {
        consider("horiz", cursor.x, prim.y);
      }
      // Vertical alignment: cursor is within `axisR` of prim.x → snap x.
      if (Math.abs(cursor.x - prim.x) <= axisR) {
        consider("vert", prim.x, cursor.y);
      }
    }
  }

  return best;
}
