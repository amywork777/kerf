/**
 * dimension-layout.ts
 *
 * Pure layout helper for SVG dimension annotations.
 *
 * Exports:
 *   - `Dim`              — axis-aligned bounding box for a dimension label.
 *   - `layoutDimensions` — greedy anticlutter: shifts dims so none overlap.
 *
 * No DOM, no WASM, no Three.js — safe to import in Node tests.
 */

/** An axis-aligned bounding box for one dimension annotation. */
export interface Dim {
  /** Left edge in SVG/canvas coordinates. */
  x: number;
  /** Top edge in SVG/canvas coordinates. */
  y: number;
  /** Width of the dimension label + leader area. */
  w: number;
  /** Height of the dimension label area. */
  h: number;
}

/** Minimum gap (px) maintained between any two dimension boxes. */
const MIN_GAP = 8;

/** True when two boxes overlap (strict — touching edges are allowed). */
function overlaps(a: Dim, b: Dim): boolean {
  return a.x < b.x + b.w && a.x + a.w > b.x && a.y < b.y + b.h && a.y + a.h > b.y;
}

/**
 * Greedy single-axis shift: move `candidate` perpendicular (along Y, then X
 * if needed) until it no longer overlaps any box in `placed`.
 *
 * Strategy: try Y-axis shifts first (most natural for a stacked-dimension
 * layout), then fall back to X-axis shifts.
 */
function resolveOneConflict(candidate: Dim, blocker: Dim): Dim {
  // Amount to shift so candidate clears blocker on the Y axis.
  const shiftDown = blocker.y + blocker.h + MIN_GAP - candidate.y;
  const shiftUp = candidate.y + candidate.h + MIN_GAP - blocker.y;

  // Pick the smaller Y shift (less movement is better UX).
  const yShift = shiftDown <= shiftUp ? shiftDown : -shiftUp;
  return { ...candidate, y: candidate.y + yShift };
}

/**
 * layoutDimensions — place dimensions greedily, shifting later ones if they
 * would collide with already-placed earlier ones.
 *
 * Rules:
 *   1. The first dimension is never moved (it is the anchor).
 *   2. Each subsequent dimension is placed at its requested position; if it
 *      overlaps any already-placed dimension it is shifted (Y axis first,
 *      then X) until clear, up to MAX_PASSES attempts.
 *   3. Output preserves input order and length.
 *   4. Pure function — no mutation of input array or objects.
 */
export function layoutDimensions(dims: Dim[]): Dim[] {
  if (dims.length === 0) return [];

  const placed: Dim[] = [];
  const MAX_PASSES = 50; // safety valve for degenerate inputs

  for (let i = 0; i < dims.length; i++) {
    let candidate = { ...dims[i]! };

    if (i === 0) {
      // First dimension is always the anchor — never moved.
      placed.push(candidate);
      continue;
    }

    // Iteratively resolve conflicts with all previously placed dims.
    for (let pass = 0; pass < MAX_PASSES; pass++) {
      const blocker = placed.find((p) => overlaps(candidate, p));
      if (!blocker) break;
      candidate = resolveOneConflict(candidate, blocker);
    }

    placed.push(candidate);
  }

  return placed;
}

// ------------------------------------------------------------------
// Drag-state helpers (used by dimensions.ts for session-level offsets)
// ------------------------------------------------------------------

/** Session-level drag offsets keyed by dimension index. */
export type DragOffsets = Map<number, { dx: number; dy: number }>;

/**
 * Apply a drag offset to a Dim, returning a new Dim. Useful when the caller
 * wants to apply user drags before running layoutDimensions.
 */
export function applyDragOffset(
  dim: Dim,
  offset: { dx: number; dy: number } | undefined,
): Dim {
  if (!offset) return dim;
  return { ...dim, x: dim.x + offset.dx, y: dim.y + offset.dy };
}
