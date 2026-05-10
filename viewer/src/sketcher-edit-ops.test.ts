// Tests for sketcher-edit-ops.ts: copy/paste, mirror, rectangular pattern.
//
// Run with: pnpm exec vitest run src/sketcher-edit-ops.test.ts

import { describe, it, expect } from "vitest";
import {
  copySelection,
  pasteFragment,
  mirrorSelection,
  patternRect,
} from "./sketcher-edit-ops.js";
import { type Sketch } from "./sketcher.js";

// ------------------------------------------------------------------
// Helpers
// ------------------------------------------------------------------

/** A simple line sketch: p1(0,0)—p2(1,0)—l1. */
function lineSketch(): Sketch {
  return {
    plane: "Xy",
    primitives: [
      { kind: "Point", id: "p1", x: 0, y: 0 },
      { kind: "Point", id: "p2", x: 1, y: 0 },
      { kind: "Line", id: "l1", from: "p1", to: "p2" },
    ],
    constraints: [],
  };
}

/** A rectangle: p1(0,0) p2(2,0) p3(2,3) p4(0,3) + four lines. */
function rectSketch(): Sketch {
  return {
    plane: "Xy",
    primitives: [
      { kind: "Point", id: "p1", x: 0, y: 0 },
      { kind: "Point", id: "p2", x: 2, y: 0 },
      { kind: "Point", id: "p3", x: 2, y: 3 },
      { kind: "Point", id: "p4", x: 0, y: 3 },
      { kind: "Line", id: "l1", from: "p1", to: "p2" },
      { kind: "Line", id: "l2", from: "p2", to: "p3" },
      { kind: "Line", id: "l3", from: "p3", to: "p4" },
      { kind: "Line", id: "l4", from: "p4", to: "p1" },
    ],
    constraints: [],
  };
}

// ------------------------------------------------------------------
// 1. copySelection
// ------------------------------------------------------------------

describe("copySelection", () => {
  it("extracts only the selected primitives", () => {
    const sk = lineSketch();
    const frag = copySelection(sk, ["p1", "p2", "l1"]);
    expect(frag.primitives).toHaveLength(3);
    expect(frag.primitives.map(p => p.id)).toEqual(expect.arrayContaining(["p1", "p2", "l1"]));
  });

  it("extracts a subset of ids, not the full sketch", () => {
    const sk = rectSketch();
    const frag = copySelection(sk, ["p1", "p2", "l1"]);
    expect(frag.primitives).toHaveLength(3);
    expect(frag.primitives.map(p => p.id)).not.toContain("p3");
    expect(frag.primitives.map(p => p.id)).not.toContain("l2");
  });

  it("returns an empty fragment for an empty selection", () => {
    const sk = lineSketch();
    const frag = copySelection(sk, []);
    expect(frag.primitives).toHaveLength(0);
  });

  it("does not mutate the source sketch", () => {
    const sk = lineSketch();
    const before = JSON.stringify(sk);
    copySelection(sk, ["p1", "p2", "l1"]);
    expect(JSON.stringify(sk)).toBe(before);
  });
});

// ------------------------------------------------------------------
// 2. pasteFragment
// ------------------------------------------------------------------

describe("pasteFragment", () => {
  it("adds primitives with new ids (no id collisions)", () => {
    const sk = lineSketch();
    const frag = copySelection(sk, ["p1", "p2", "l1"]);
    const result = pasteFragment(sk, frag, { x: 10, y: 0 });
    expect(result.primitives).toHaveLength(6); // original 3 + pasted 3
    const ids = result.primitives.map(p => p.id);
    expect(new Set(ids).size).toBe(ids.length); // all unique
  });

  it("shifts pasted Points by the given offset", () => {
    const sk = lineSketch();
    const frag = copySelection(sk, ["p1", "p2", "l1"]);
    const result = pasteFragment(sk, frag, { x: 10, y: 5 });
    const origIds = new Set(sk.primitives.map(p => p.id));
    const newPoints = result.primitives.filter(
      p => p.kind === "Point" && !origIds.has(p.id)
    ) as Array<{ kind: "Point"; id: string; x: number; y: number }>;
    expect(newPoints).toHaveLength(2);
    const xs = newPoints.map(p => p.x).sort((a, b) => a - b);
    expect(xs[0]).toBeCloseTo(10);  // p1(0,0) + (10,5) = (10,5)
    expect(xs[1]).toBeCloseTo(11);  // p2(1,0) + (10,5) = (11,5)
    newPoints.forEach(p => expect(p.y).toBeCloseTo(5));
  });

  it("preserves original sketch unchanged", () => {
    const sk = lineSketch();
    const before = JSON.stringify(sk);
    const frag = copySelection(sk, ["p1", "p2", "l1"]);
    pasteFragment(sk, frag, { x: 10, y: 0 });
    expect(JSON.stringify(sk)).toBe(before);
  });

  it("gives new ids even when offset is zero", () => {
    const sk = lineSketch();
    const frag = copySelection(sk, ["p1", "p2", "l1"]);
    const result = pasteFragment(sk, frag, { x: 0, y: 0 });
    const ids = result.primitives.map(p => p.id);
    expect(new Set(ids).size).toBe(ids.length);
  });
});

// ------------------------------------------------------------------
// 3. mirrorSelection
// ------------------------------------------------------------------

describe("mirrorSelection", () => {
  it("reflects across x-axis: y → -y", () => {
    const sk: Sketch = {
      plane: "Xy",
      primitives: [
        { kind: "Point", id: "p1", x: 0, y: 0 },
        { kind: "Point", id: "p2", x: 1, y: 0 },
        { kind: "Line", id: "l1", from: "p1", to: "p2" },
        { kind: "Point", id: "p3", x: 0.5, y: 2 },
      ],
      constraints: [],
    };
    // x-axis mirror: axis from (0,0) to (1,0).
    const result = mirrorSelection(sk, ["p3"], { from: { x: 0, y: 0 }, to: { x: 1, y: 0 } });
    const origIds = new Set(sk.primitives.map(p => p.id));
    const newPts = result.primitives.filter(
      p => p.kind === "Point" && !origIds.has(p.id)
    ) as Array<{ kind: "Point"; id: string; x: number; y: number }>;
    expect(newPts).toHaveLength(1);
    expect(newPts[0].x).toBeCloseTo(0.5);
    expect(newPts[0].y).toBeCloseTo(-2);
  });

  it("reflects across y-axis: x → -x", () => {
    const sk = lineSketch(); // p1(0,0)→p2(1,0)
    const result = mirrorSelection(sk, ["p1", "p2", "l1"], { from: { x: 0, y: 0 }, to: { x: 0, y: 1 } });
    expect(result.primitives).toHaveLength(6); // original 3 + mirrored 3
    const origIds = new Set(sk.primitives.map(p => p.id));
    const newPts = result.primitives.filter(
      p => p.kind === "Point" && !origIds.has(p.id)
    ) as Array<{ kind: "Point"; id: string; x: number; y: number }>;
    expect(newPts).toHaveLength(2);
    const xs = newPts.map(p => p.x).sort((a, b) => a - b);
    expect(xs[0]).toBeCloseTo(-1);
    expect(xs[1]).toBeCloseTo(0);
  });

  it("produces no id collisions after mirror", () => {
    const sk = lineSketch();
    const result = mirrorSelection(sk, ["p1", "p2", "l1"], { from: { x: 0, y: 0 }, to: { x: 1, y: 0 } });
    const ids = result.primitives.map(p => p.id);
    expect(new Set(ids).size).toBe(ids.length);
  });

  it("does not mutate the source sketch", () => {
    const sk = lineSketch();
    const before = JSON.stringify(sk);
    mirrorSelection(sk, ["p1", "p2", "l1"], { from: { x: 0, y: 0 }, to: { x: 1, y: 0 } });
    expect(JSON.stringify(sk)).toBe(before);
  });
});

// ------------------------------------------------------------------
// 4. patternRect
// ------------------------------------------------------------------

describe("patternRect", () => {
  it("2×3 produces 6 copies including the original", () => {
    const sk = lineSketch(); // 3 primitives
    const result = patternRect(sk, ["p1", "p2", "l1"], 5, 5, 2, 3);
    // 2 columns × 3 rows = 6 copies × 3 primitives = 18
    expect(result.primitives).toHaveLength(18);
    const ids = result.primitives.map(p => p.id);
    expect(new Set(ids).size).toBe(ids.length);
  });

  it("1×1 is a no-op: returns exactly the original primitives", () => {
    const sk = lineSketch();
    const result = patternRect(sk, ["p1", "p2", "l1"], 5, 5, 1, 1);
    expect(result.primitives).toHaveLength(sk.primitives.length);
  });

  it("offsets copies correctly by dx and dy", () => {
    const sk: Sketch = {
      plane: "Xy",
      primitives: [{ kind: "Point", id: "p1", x: 0, y: 0 }],
      constraints: [],
    };
    const result = patternRect(sk, ["p1"], 3, 7, 3, 2);
    expect(result.primitives).toHaveLength(6); // 3×2
    const pts = result.primitives.filter(p => p.kind === "Point") as
      Array<{ kind: "Point"; id: string; x: number; y: number }>;
    const coords = pts.map(p => `${p.x.toFixed(1)},${p.y.toFixed(1)}`).sort();
    expect(coords).toEqual(["0.0,0.0", "0.0,7.0", "3.0,0.0", "3.0,7.0", "6.0,0.0", "6.0,7.0"].sort());
  });

  it("does not mutate the source sketch", () => {
    const sk = lineSketch();
    const before = JSON.stringify(sk);
    patternRect(sk, ["p1", "p2", "l1"], 5, 5, 2, 2);
    expect(JSON.stringify(sk)).toBe(before);
  });
});
