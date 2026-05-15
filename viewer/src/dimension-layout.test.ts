// Tests for layoutDimensions — pure layout algorithm for dimension annotations.
// These are node-safe (no DOM, no WASM).

import { describe, it, expect } from "vitest";
import { layoutDimensions } from "./dimension-layout.js";
import type { Dim } from "./dimension-layout.js";

const MIN_GAP = 8;

function box(x: number, y: number, w = 60, h = 20): Dim {
  return { x, y, w, h };
}

/** Check all pairs of placed dims are separated by at least MIN_GAP on one axis. */
function allSeparated(dims: Dim[]): boolean {
  for (let i = 0; i < dims.length; i++) {
    for (let j = i + 1; j < dims.length; j++) {
      const a = dims[i]!;
      const b = dims[j]!;
      const overlapX = a.x < b.x + b.w && a.x + a.w > b.x;
      const overlapY = a.y < b.y + b.h && a.y + a.h > b.y;
      if (overlapX && overlapY) return false;
    }
  }
  return true;
}

describe("layoutDimensions", () => {
  it("single dimension is returned unchanged", () => {
    const d = box(10, 20, 60, 20);
    const result = layoutDimensions([d]);
    expect(result).toHaveLength(1);
    expect(result[0]).toEqual(d);
  });

  it("two non-overlapping dimensions are returned unchanged", () => {
    const a = box(0, 0, 60, 20);
    const b = box(200, 0, 60, 20); // far apart, no overlap
    const result = layoutDimensions([a, b]);
    expect(result[0]).toEqual(a);
    expect(result[1]).toEqual(b);
  });

  it("two overlapping dimensions — second is shifted by ≥ 8 px", () => {
    const a = box(0, 0, 60, 20);
    const b = box(0, 0, 60, 20); // exactly the same position → fully overlapping
    const result = layoutDimensions([a, b]);
    expect(result[0]).toEqual(a); // first is always unchanged
    // Second must be shifted enough to no longer intersect
    expect(allSeparated(result)).toBe(true);
    // Verify the shift is at least MIN_GAP in the shifted axis
    const dy = Math.abs((result[1]?.y ?? 0) - b.y);
    const dx = Math.abs((result[1]?.x ?? 0) - b.x);
    expect(Math.max(dx, dy)).toBeGreaterThanOrEqual(MIN_GAP);
  });

  it("five stacked dimensions are all separated", () => {
    // All at the same position — worst-case stack.
    const dims: Dim[] = Array.from({ length: 5 }, () => box(0, 0, 60, 20));
    const result = layoutDimensions(dims);
    expect(result).toHaveLength(5);
    expect(allSeparated(result)).toBe(true);
  });

  it("stable order — input order is preserved, first dim is anchor", () => {
    const a = box(0, 0, 60, 20);
    const b = box(5, 5, 60, 20);  // overlaps a
    const c = box(300, 300, 60, 20); // far away, no overlap
    const result = layoutDimensions([a, b, c]);
    expect(result).toHaveLength(3);
    // First is always unchanged
    expect(result[0]).toEqual(a);
    // Last (non-overlapping) should be unchanged
    expect(result[2]).toEqual(c);
    // All separated
    expect(allSeparated(result)).toBe(true);
  });

  it("deterministic — same input gives same output", () => {
    const dims: Dim[] = [box(0, 0), box(10, 0), box(20, 0), box(30, 0)];
    const r1 = layoutDimensions(dims);
    const r2 = layoutDimensions(dims);
    expect(r1).toEqual(r2);
  });

  it("already-separated dims with touching edges are not shifted", () => {
    // b starts exactly where a ends (adjacent, not overlapping)
    const a = box(0, 0, 60, 20);
    const b = box(60, 0, 60, 20); // x=60, width=60 → no overlap
    const result = layoutDimensions([a, b]);
    expect(result[0]).toEqual(a);
    expect(result[1]).toEqual(b);
  });
});
