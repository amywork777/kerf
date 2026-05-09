// Visual constraint glyph badges for the 2D sketcher.
//
// Exports two things:
//   glyphPlacements(sketch, viewport)  — pure function, returns placement list
//   renderConstraintGlyphs(canvas, sketch, viewport)  — draws to a canvas 2D ctx
//
// The placement function is tested in isolation (no canvas needed).  The
// render function is a thin driver on top; pixel-perfect rendering is NOT
// asserted in tests.

import type { Sketch, SketchConstraint, SketchPrim } from "./sketcher.js";

// ---- types ----

export type Viewport = {
  /** World origin in canvas pixels (after pan/zoom). */
  ox: number;
  oy: number;
  /** Pixels per world unit (+y is up in world, down in canvas). */
  scale: number;
};

export type GlyphPlacement = {
  /** The Unicode glyph or label string to render. */
  glyph: string;
  /** Canvas-space X of the badge anchor (left edge of text). */
  x: number;
  /** Canvas-space Y of the badge anchor (text baseline). */
  y: number;
  /** The constraint kind — useful for CSS classes and hover wiring. */
  kind: SketchConstraint["kind"];
  /** The primitive ids that this constraint applies to (for hover wiring). */
  refs: string[];
};

// ---- coordinate helpers ----

function worldToCanvas(wx: number, wy: number, vp: Viewport): [number, number] {
  return [vp.ox + wx * vp.scale, vp.oy - wy * vp.scale];
}

// ---- primitive lookup ----

type PrimMap = Map<string, SketchPrim>;

function buildPrimMap(sketch: Sketch): PrimMap {
  const m = new Map<string, SketchPrim>();
  for (const p of sketch.primitives) m.set(p.id, p);
  return m;
}

/** Midpoint in world space of a Line, or null if refs are missing. */
function lineMidWorld(
  lineId: string,
  prims: PrimMap,
): { x: number; y: number } | null {
  const line = prims.get(lineId);
  if (!line || line.kind !== "Line") return null;
  const a = prims.get(line.from);
  const b = prims.get(line.to);
  if (!a || a.kind !== "Point" || !b || b.kind !== "Point") return null;
  return { x: (a.x + b.x) / 2, y: (a.y + b.y) / 2 };
}

/** World coords of a Point, or null. */
function pointWorld(
  ptId: string,
  prims: PrimMap,
): { x: number; y: number } | null {
  const p = prims.get(ptId);
  if (!p || p.kind !== "Point") return null;
  return { x: p.x, y: p.y };
}

// ---- nudge helpers to avoid overlap ----

/** Push canvas-space badge position a few pixels off the primitive. */
const BADGE_OFFSET_PX = 10;

// ---- core placement function ----

/**
 * Pure function: compute canvas-space placements for all constraint glyphs
 * in `sketch`.  Returns one GlyphPlacement per constraint (except Equal,
 * which returns one per constrained line → two for a pair).
 *
 * The function does NOT modify anything; it is safe to call repeatedly.
 */
export function glyphPlacements(
  sketch: Sketch,
  viewport: Viewport,
): GlyphPlacement[] {
  const prims = buildPrimMap(sketch);
  const results: GlyphPlacement[] = [];

  for (const c of sketch.constraints) {
    switch (c.kind) {
      case "Horizontal": {
        // Glyph at midpoint of line, nudged above.
        const mid = lineMidWorld(c.line, prims);
        if (!mid) break;
        const [cx, cy] = worldToCanvas(mid.x, mid.y, viewport);
        results.push({
          glyph: "═",
          x: cx,
          y: cy - BADGE_OFFSET_PX,
          kind: "Horizontal",
          refs: [c.line],
        });
        break;
      }

      case "Vertical": {
        // Glyph at midpoint of line, nudged to the right.
        const mid = lineMidWorld(c.line, prims);
        if (!mid) break;
        const [cx, cy] = worldToCanvas(mid.x, mid.y, viewport);
        results.push({
          glyph: "║",
          x: cx + BADGE_OFFSET_PX,
          y: cy,
          kind: "Vertical",
          refs: [c.line],
        });
        break;
      }

      case "Parallel": {
        // Glyph at midpoint of line_a, nudged below.
        const mid = lineMidWorld(c.line_a, prims);
        if (!mid) break;
        const [cx, cy] = worldToCanvas(mid.x, mid.y, viewport);
        results.push({
          glyph: "∥",
          x: cx,
          y: cy + BADGE_OFFSET_PX,
          kind: "Parallel",
          refs: [c.line_a, c.line_b],
        });
        break;
      }

      case "Perpendicular": {
        // Glyph at the "corner" = average of the two line midpoints.
        const midA = lineMidWorld(c.line_a, prims);
        const midB = lineMidWorld(c.line_b, prims);
        if (!midA || !midB) break;

        // Better anchor: find the shared endpoint if the lines share one.
        const lineA = prims.get(c.line_a);
        const lineB = prims.get(c.line_b);
        let wx = (midA.x + midB.x) / 2;
        let wy = (midA.y + midB.y) / 2;
        if (lineA && lineA.kind === "Line" && lineB && lineB.kind === "Line") {
          const aEnds = [lineA.from, lineA.to];
          const bEnds = [lineB.from, lineB.to];
          const shared = aEnds.find((id) => bEnds.includes(id));
          if (shared) {
            const pt = pointWorld(shared, prims);
            if (pt) { wx = pt.x; wy = pt.y; }
          }
        }

        const [cx, cy] = worldToCanvas(wx, wy, viewport);
        results.push({
          glyph: "⊥",
          x: cx - BADGE_OFFSET_PX,
          y: cy - BADGE_OFFSET_PX,
          kind: "Perpendicular",
          refs: [c.line_a, c.line_b],
        });
        break;
      }

      case "Coincident": {
        // Dot at the shared point (prefer a itself if it's a Point).
        const pa = pointWorld(c.a, prims);
        const pb = pointWorld(c.b, prims);
        const wpt = pa ?? pb;
        if (!wpt) break;
        const [cx, cy] = worldToCanvas(wpt.x, wpt.y, viewport);
        results.push({
          glyph: "●",
          x: cx,
          y: cy,
          kind: "Coincident",
          refs: [c.a, c.b],
        });
        break;
      }

      case "Distance": {
        // Leader + dim text between the two points.
        const pa = pointWorld(c.a, prims);
        const pb = pointWorld(c.b, prims);
        if (!pa || !pb) break;
        const [cax, cay] = worldToCanvas(pa.x, pa.y, viewport);
        const [cbx, cby] = worldToCanvas(pb.x, pb.y, viewport);
        results.push({
          glyph: `${c.value.toFixed(2)}`,
          x: (cax + cbx) / 2,
          y: (cay + cby) / 2 - BADGE_OFFSET_PX,
          kind: "Distance",
          refs: [c.a, c.b],
        });
        break;
      }

      case "FixedPoint": {
        // Pin badge above the fixed point.
        const pt = pointWorld(c.point, prims);
        if (!pt) break;
        const [cx, cy] = worldToCanvas(pt.x, pt.y, viewport);
        results.push({
          glyph: "📌",
          x: cx,
          y: cy - BADGE_OFFSET_PX,
          kind: "FixedPoint",
          refs: [c.point],
        });
        break;
      }
    }
  }

  return results;
}

// ---- renderer ----

/**
 * Draw constraint glyph badges onto `canvas` for every constraint in `sketch`.
 * Must be called after the primitives are already drawn so badges appear on top.
 *
 * Each glyph is drawn as a `<text>` via canvas 2D fillText with:
 *   font-family monospace, 9 px, fill var(--accent) (#7ee787 fallback)
 *
 * Hover wiring (data-hover on primitives) is intentionally NOT done here —
 * that requires SVG or HTML overlay elements.  The placement data returned by
 * `glyphPlacements` is what the host can use to build an SVG/HTML layer for
 * richer interaction.
 */
export function renderConstraintGlyphs(
  canvas: HTMLCanvasElement,
  sketch: Sketch,
  viewport: Viewport,
): void {
  const ctx = canvas.getContext("2d");
  if (!ctx) return;

  const placements = glyphPlacements(sketch, viewport);

  ctx.save();
  ctx.font = "9px ui-monospace,Menlo,monospace";
  ctx.textBaseline = "middle";
  ctx.fillStyle = "var(--accent, #7ee787)";

  for (const p of placements) {
    ctx.fillText(p.glyph, p.x, p.y);
  }

  ctx.restore();
}
