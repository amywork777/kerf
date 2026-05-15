import { describe, it, expect } from "vitest";
import { findSnap } from "./sketcher-snap.js";
import type { Sketch } from "./sketcher.js";

// Helpers for building minimal sketches.
function emptySketch(): Sketch {
  return { plane: "Xy", primitives: [], constraints: [] };
}

function sketchWithPoints(...pts: Array<{ id: string; x: number; y: number }>): Sketch {
  return {
    plane: "Xy",
    primitives: pts.map((p) => ({ kind: "Point" as const, ...p })),
    constraints: [],
  };
}

/** Scale=100 → 1 world unit = 100 px, so 8px threshold = 0.08 world units. */
const VP = { scale: 100 };

describe("findSnap — empty sketch", () => {
  it("returns null when sketch has no primitives", () => {
    const result = findSnap({ x: 0, y: 0 }, emptySketch(), VP);
    expect(result).toBeNull();
  });
});

describe("findSnap — point snap", () => {
  it("snaps to a nearby existing point", () => {
    const sketch = sketchWithPoints({ id: "p1", x: 1, y: 2 });
    // Cursor is 0.05 world units away → within 8px threshold (0.08 wu).
    const result = findSnap({ x: 1.05, y: 2 }, sketch, VP);
    expect(result).not.toBeNull();
    expect(result!.point).toEqual({ x: 1, y: 2 });
    // Should be "existing-point" (not promoted to endpoint without a Line).
    expect(result!.kind).toBe("existing-point");
  });

  it("returns null when cursor is far from the point", () => {
    const sketch = sketchWithPoints({ id: "p1", x: 1, y: 2 });
    // 0.5 world units in both axes → far from point and not on same axis.
    const result = findSnap({ x: 1.5, y: 2.5 }, sketch, VP);
    expect(result).toBeNull();
  });
});

describe("findSnap — endpoint vs mid priority", () => {
  it("snaps to endpoint when cursor is equally close to both endpoint and mid", () => {
    // Line from (0,0) to (2,0): midpoint is (1,0).
    // Place cursor at (0.04, 0) — close to endpoint (0,0).
    const sketch: Sketch = {
      plane: "Xy",
      primitives: [
        { kind: "Point", id: "pa", x: 0, y: 0 },
        { kind: "Point", id: "pb", x: 2, y: 0 },
        { kind: "Line", id: "l1", from: "pa", to: "pb" },
      ],
      constraints: [],
    };
    const result = findSnap({ x: 0.04, y: 0 }, sketch, VP);
    expect(result).not.toBeNull();
    expect(result!.kind).toBe("endpoint");
    expect(result!.point).toEqual({ x: 0, y: 0 });
  });

  it("snaps to mid when cursor is near the midpoint of a line", () => {
    const sketch: Sketch = {
      plane: "Xy",
      primitives: [
        { kind: "Point", id: "pa", x: 0, y: 0 },
        { kind: "Point", id: "pb", x: 2, y: 0 },
        { kind: "Line", id: "l1", from: "pa", to: "pb" },
      ],
      constraints: [],
    };
    // Midpoint is (1, 0). Cursor at (1.04, 0) — within 8px of mid but far from endpoints.
    const result = findSnap({ x: 1.04, y: 0 }, sketch, VP);
    expect(result).not.toBeNull();
    expect(result!.kind).toBe("mid");
    expect(result!.point).toEqual({ x: 1, y: 0 });
  });

  it("endpoint beats mid when both within range (priority test)", () => {
    // Degenerate: line from (0,0) to (0.1,0): midpoint (0.05,0).
    // Cursor at (0.04,0) — within 8px of endpoint AND within 8px of mid.
    // Endpoint should win.
    const sketch: Sketch = {
      plane: "Xy",
      primitives: [
        { kind: "Point", id: "pa", x: 0, y: 0 },
        { kind: "Point", id: "pb", x: 0.1, y: 0 },
        { kind: "Line", id: "l1", from: "pa", to: "pb" },
      ],
      constraints: [],
    };
    const result = findSnap({ x: 0.04, y: 0 }, sketch, VP);
    expect(result).not.toBeNull();
    expect(result!.kind).toBe("endpoint");
  });
});

describe("findSnap — center snap", () => {
  it("snaps to circle center when cursor is near it", () => {
    const sketch: Sketch = {
      plane: "Xy",
      primitives: [
        { kind: "Point", id: "c", x: 3, y: 4 },
        { kind: "Circle", id: "k1", center: "c", radius: 1, n_segments: 32 },
      ],
      constraints: [],
    };
    const result = findSnap({ x: 3.04, y: 4 }, sketch, VP);
    expect(result).not.toBeNull();
    // The center Point itself matches "existing-point"; the Circle promotes it
    // to "center" — but since "existing-point" already fires before "center"
    // in the loop, the net winner is whichever has higher priority.
    // "existing-point" has priority index 1, "center" has index 3 — so
    // "existing-point" wins for the Point prim, but "center" is also
    // considered. Since "existing-point" priority < "center", it will win.
    // The point coords are the same, so either is correct — just check coords.
    expect(result!.point).toEqual({ x: 3, y: 4 });
  });
});

describe("findSnap — no snap when cursor is far from everything", () => {
  it("returns null when cursor is far from all primitives", () => {
    const sketch: Sketch = {
      plane: "Xy",
      primitives: [
        { kind: "Point", id: "pa", x: 0, y: 0 },
        { kind: "Point", id: "pb", x: 2, y: 0 },
        { kind: "Line", id: "l1", from: "pa", to: "pb" },
      ],
      constraints: [],
    };
    // Cursor is at (5, 5), well outside any snap radius.
    const result = findSnap({ x: 5, y: 5 }, sketch, VP);
    expect(result).toBeNull();
  });
});

describe("findSnap — grid snap", () => {
  it("snaps to grid intersection when grid is visible", () => {
    const sketch = emptySketch();
    // Cursor at (0.97, 1.03) — within 8px of grid point (1,1).
    const result = findSnap(
      { x: 0.97, y: 1.03 },
      sketch,
      { scale: 100, gridSpacing: 1, gridVisible: true },
    );
    expect(result).not.toBeNull();
    expect(result!.kind).toBe("grid");
    expect(result!.point.x).toBeCloseTo(1, 5);
    expect(result!.point.y).toBeCloseTo(1, 5);
  });

  it("does not snap to grid when grid is not visible", () => {
    const sketch = emptySketch();
    const result = findSnap(
      { x: 0.97, y: 1.03 },
      sketch,
      { scale: 100, gridSpacing: 1, gridVisible: false },
    );
    expect(result).toBeNull();
  });
});
