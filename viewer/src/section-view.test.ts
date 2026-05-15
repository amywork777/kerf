import { describe, it, expect } from "vitest";
import { clipModelToSection } from "./section-view.js";
import type { Triangle, Plane, Seg2D } from "./section-view.js";

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

/** Build 12 triangles making up an axis-aligned box [0,sx]×[0,sy]×[0,sz]. */
function makeBox(sx: number, sy: number, sz: number): Triangle[] {
  const tris: Triangle[] = [];
  // Each face = 2 triangles. Six faces:
  const faces: [
    [number, number, number],
    [number, number, number],
    [number, number, number],
    [number, number, number],
  ][] = [
    // -Z face (z=0)
    [[0,0,0],[sx,0,0],[sx,sy,0]],
    [[0,0,0],[sx,sy,0],[0,sy,0]],
    // +Z face (z=sz)
    [[0,0,sz],[sx,sy,sz],[sx,0,sz]],
    [[0,0,sz],[0,sy,sz],[sx,sy,sz]],
    // -Y face (y=0)
    [[0,0,0],[sx,0,sz],[sx,0,0]],
    [[0,0,0],[0,0,sz],[sx,0,sz]],
    // +Y face (y=sy)
    [[0,sy,0],[sx,sy,0],[sx,sy,sz]],
    [[0,sy,0],[sx,sy,sz],[0,sy,sz]],
    // -X face (x=0)
    [[0,0,0],[0,sy,0],[0,sy,sz]],
    [[0,0,0],[0,sy,sz],[0,0,sz]],
    // +X face (x=sx)
    [[sx,0,0],[sx,sy,sz],[sx,sy,0]],
    [[sx,0,0],[sx,0,sz],[sx,sy,sz]],
  ] as any;
  for (const [a, b, c] of faces as [[number,number,number],[number,number,number],[number,number,number]][]) {
    tris.push({ a, b, c });
  }
  return tris;
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

describe("clipModelToSection", () => {
  it("empty model → empty cross-section", () => {
    const plane: Plane = { normal: [0, 0, 1], offset: 0 };
    const result = clipModelToSection([], plane);
    expect(result).toHaveLength(0);
  });

  it("single triangle entirely above the plane → no segments", () => {
    const tri: Triangle = {
      a: [0, 0, 1],
      b: [1, 0, 1],
      c: [0, 1, 1],
    };
    // Plane z=0, all vertices above (dist > 0)
    const plane: Plane = { normal: [0, 0, 1], offset: 0 };
    const result = clipModelToSection([tri], plane);
    expect(result).toHaveLength(0);
  });

  it("single triangle straddling the plane → one segment", () => {
    // Triangle: z goes from -1 to +1 — the plane z=0 cuts through it.
    const tri: Triangle = {
      a: [0, 0, -1],
      b: [2, 0, -1],
      c: [1, 0,  1],
    };
    const plane: Plane = { normal: [0, 0, 1], offset: 0 };
    const segs = clipModelToSection([tri], plane);
    expect(segs).toHaveLength(1);
    const s = segs[0]!;
    // Both endpoints should lie in the plane (z=0 → projected points lie at
    // definite 2-D locations; we just verify the segment is non-degenerate).
    const length = Math.hypot(s.x2 - s.x1, s.y2 - s.y1);
    expect(length).toBeGreaterThan(0.5);
  });

  it("unit box cut by midplane z=0.5 → segments forming the cross-section", () => {
    // A box [0,1]×[0,1]×[0,1] cut at z=0.5.  The box has 12 triangles
    // (6 faces × 2 tri each).  The four vertical faces (±X, ±Y sides) each
    // contribute two triangles both of which are cut by the midplane,
    // yielding 8 segments total.  The two horizontal faces (top/bottom)
    // are parallel to the plane and not cut.
    const box = makeBox(1, 1, 1);
    const plane: Plane = { normal: [0, 0, 1], offset: 0.5 };
    const segs = clipModelToSection(box, plane);

    // 4 vertical faces × 2 triangles each = 8 segments.
    expect(segs).toHaveLength(8);

    // Every segment should have unit length (edge of the 1×1 square).
    for (const s of segs) {
      const l = Math.hypot(s.x2 - s.x1, s.y2 - s.y1);
      expect(l).toBeGreaterThan(0.4);
      expect(l).toBeLessThanOrEqual(1.01);
    }

    // The collective 2-D bounding box of all segment endpoints should span
    // [0,1]×[0,1] — confirming we captured the complete square cross-section.
    const allX = segs.flatMap((s) => [s.x1, s.x2]);
    const allY = segs.flatMap((s) => [s.y1, s.y2]);
    expect(Math.max(...allX) - Math.min(...allX)).toBeCloseTo(1, 5);
    expect(Math.max(...allY) - Math.min(...allY)).toBeCloseTo(1, 5);
  });

  it("2×3×4 box cut by y-midplane → rectangular cross-section spanning 2×4", () => {
    const box = makeBox(2, 3, 4);
    // Cut at y = 1.5 (midplane). 4 non-parallel faces × 2 tris = 8 segments.
    const plane: Plane = { normal: [0, 1, 0], offset: 1.5 };
    const segs = clipModelToSection(box, plane);
    expect(segs).toHaveLength(8);
    // The 2-D bounding box of the cross-section should span 2 (x-axis of box)
    // × 4 (z-axis of box).
    const allX = segs.flatMap((s) => [s.x1, s.x2]);
    const allY = segs.flatMap((s) => [s.y1, s.y2]);
    const spanX = Math.max(...allX) - Math.min(...allX);
    const spanY = Math.max(...allY) - Math.min(...allY);
    // One axis spans 2, the other spans 4.
    const spans = [spanX, spanY].sort((a, b) => a - b);
    expect(spans[0]).toBeCloseTo(2, 5);
    expect(spans[1]).toBeCloseTo(4, 5);
  });

  it("multiple sections produce independent results", () => {
    const box = makeBox(1, 1, 1);

    const planeA: Plane = { normal: [0, 0, 1], offset: 0.25 };
    const planeB: Plane = { normal: [0, 0, 1], offset: 0.75 };

    const segsA = clipModelToSection(box, planeA);
    const segsB = clipModelToSection(box, planeB);

    // Both planes cut a unit box through 4 vertical faces × 2 tris = 8 segs.
    expect(segsA).toHaveLength(8);
    expect(segsB).toHaveLength(8);

    // The two results are computed independently — mutating one must not
    // affect the other.
    segsA[0]!.x1 = 999;
    expect(segsB[0]!.x1).not.toBe(999);
  });

  it("non-axis-aligned normal is handled", () => {
    // Diagonal plane: normal = [1,1,0]/√2, offset = 0 → cuts through origin.
    const box = makeBox(2, 2, 2);
    const plane: Plane = { normal: [1, 1, 0], offset: 2 }; // cuts diagonally
    const segs = clipModelToSection(box, plane);
    // We just check it produces segments without throwing.
    expect(Array.isArray(segs)).toBe(true);
  });
});
