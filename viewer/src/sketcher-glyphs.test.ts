// Tests for sketcher-glyphs.ts — placement function only.
// We test glyphPlacements() (pure, no canvas) rather than pixel rendering,
// which is not deterministic across environments.

import { describe, it, expect } from "vitest";
import { glyphPlacements } from "./sketcher-glyphs.js";
import type { Sketch, SketchConstraint } from "./sketcher.js";
import type { Viewport } from "./sketcher-glyphs.js";

// ---- helpers ----

/** A simple 1:1 viewport centred at (0,0): world unit = 1 canvas pixel.
 *  worldToCanvas(x, y) → [x, -y] (y flipped). */
const VP: Viewport = { ox: 0, oy: 0, scale: 1 };

/** A viewport with scale=10 and origin at (100,100) — more realistic. */
const VP10: Viewport = { ox: 100, oy: 100, scale: 10 };

function horizontalLineSketch(constraints: SketchConstraint[] = []): Sketch {
  return {
    plane: "Xy",
    primitives: [
      { kind: "Point", id: "p1", x: 0, y: 0 },
      { kind: "Point", id: "p2", x: 4, y: 0 },
      { kind: "Line", id: "l1", from: "p1", to: "p2" },
    ],
    constraints,
  };
}

function twoLinesSketch(constraints: SketchConstraint[] = []): Sketch {
  return {
    plane: "Xy",
    primitives: [
      { kind: "Point", id: "p1", x: 0, y: 0 },
      { kind: "Point", id: "p2", x: 4, y: 0 },
      { kind: "Point", id: "p3", x: 4, y: 3 },
      { kind: "Line", id: "l1", from: "p1", to: "p2" },
      { kind: "Line", id: "l2", from: "p2", to: "p3" },
    ],
    constraints,
  };
}

// ---- tests ----

describe("glyphPlacements", () => {
  it("returns empty array for a sketch with no constraints", () => {
    const sketch: Sketch = {
      plane: "Xy",
      primitives: [
        { kind: "Point", id: "p1", x: 0, y: 0 },
        { kind: "Point", id: "p2", x: 2, y: 0 },
        { kind: "Line", id: "l1", from: "p1", to: "p2" },
      ],
      constraints: [],
    };
    expect(glyphPlacements(sketch, VP)).toEqual([]);
  });

  it("returns empty array for an entirely empty sketch", () => {
    const sketch: Sketch = { plane: "Xy", primitives: [], constraints: [] };
    expect(glyphPlacements(sketch, VP)).toEqual([]);
  });

  // ---- Horizontal ----

  it("Horizontal constraint → 1 glyph (═) at midpoint of line, above", () => {
    const sketch = horizontalLineSketch([{ kind: "Horizontal", line: "l1" }]);
    // l1 goes from (0,0) to (4,0), midpoint = (2,0).
    // VP: worldToCanvas(2,0) = [2, 0]. Badge nudged 10px above → y = -10.
    const placements = glyphPlacements(sketch, VP);
    expect(placements).toHaveLength(1);
    const p = placements[0]!;
    expect(p.glyph).toBe("═");
    expect(p.kind).toBe("Horizontal");
    expect(p.refs).toContain("l1");
    expect(p.x).toBeCloseTo(2);
    expect(p.y).toBeCloseTo(-10); // above (y decreasing = up on canvas)
  });

  it("Horizontal glyph with VP10 is at scaled midpoint", () => {
    const sketch = horizontalLineSketch([{ kind: "Horizontal", line: "l1" }]);
    // midpoint world = (2, 0). canvas = (100 + 2*10, 100 - 0*10) = (120, 100).
    // nudge above → y = 100 - 10 = 90.
    const placements = glyphPlacements(sketch, VP10);
    expect(placements).toHaveLength(1);
    const p = placements[0]!;
    expect(p.x).toBeCloseTo(120);
    expect(p.y).toBeCloseTo(90);
  });

  // ---- Vertical ----

  it("Vertical constraint → 1 glyph (║) at midpoint of line, right side", () => {
    const sketch = horizontalLineSketch([{ kind: "Vertical", line: "l1" }]);
    const placements = glyphPlacements(sketch, VP);
    expect(placements).toHaveLength(1);
    const p = placements[0]!;
    expect(p.glyph).toBe("║");
    expect(p.kind).toBe("Vertical");
    expect(p.x).toBeCloseTo(12); // 2 + 10 nudge right
    expect(p.y).toBeCloseTo(0);
  });

  // ---- Parallel ----

  it("Parallel constraint between 2 lines → 1 glyph (∥) at midpoint of line_a, below", () => {
    const sketch = twoLinesSketch([
      { kind: "Parallel", line_a: "l1", line_b: "l2" },
    ]);
    // l1 midpoint = (2, 0). Canvas(VP) = (2, 0). Nudge below → y = +10.
    const placements = glyphPlacements(sketch, VP);
    expect(placements).toHaveLength(1);
    const p = placements[0]!;
    expect(p.glyph).toBe("∥");
    expect(p.kind).toBe("Parallel");
    expect(p.refs).toContain("l1");
    expect(p.refs).toContain("l2");
    expect(p.x).toBeCloseTo(2);
    expect(p.y).toBeCloseTo(10); // below on canvas
  });

  // ---- Perpendicular ----

  it("Perpendicular constraint at shared corner → 1 glyph (⊥) at the vertex", () => {
    // l1 and l2 share p2 at (4, 0) — that is the corner.
    const sketch = twoLinesSketch([
      { kind: "Perpendicular", line_a: "l1", line_b: "l2" },
    ]);
    const placements = glyphPlacements(sketch, VP);
    expect(placements).toHaveLength(1);
    const p = placements[0]!;
    expect(p.glyph).toBe("⊥");
    expect(p.kind).toBe("Perpendicular");
    // Shared point is p2 at world (4,0) → canvas (4,0) with VP.
    // Nudged left+up: x-10, y-10.
    expect(p.x).toBeCloseTo(-6); // 4 - 10
    expect(p.y).toBeCloseTo(-10); // 0 - 10
  });

  // ---- Coincident ----

  it("Coincident constraint → 1 dot (●) at the shared point", () => {
    const sketch: Sketch = {
      plane: "Xy",
      primitives: [
        { kind: "Point", id: "p1", x: 1, y: 2 },
        { kind: "Point", id: "p2", x: 1, y: 2 },
      ],
      constraints: [{ kind: "Coincident", a: "p1", b: "p2" }],
    };
    const placements = glyphPlacements(sketch, VP);
    expect(placements).toHaveLength(1);
    const p = placements[0]!;
    expect(p.glyph).toBe("●");
    expect(p.kind).toBe("Coincident");
    expect(p.x).toBeCloseTo(1);
    expect(p.y).toBeCloseTo(-2); // VP: oy=0, scale=1 → canvas y = 0 - 2*1 = -2
  });

  // ---- Distance ----

  it("Distance constraint → 1 label between the two points", () => {
    const sketch: Sketch = {
      plane: "Xy",
      primitives: [
        { kind: "Point", id: "p1", x: 0, y: 0 },
        { kind: "Point", id: "p2", x: 4, y: 0 },
      ],
      constraints: [{ kind: "Distance", a: "p1", b: "p2", value: 4 }],
    };
    const placements = glyphPlacements(sketch, VP);
    expect(placements).toHaveLength(1);
    const p = placements[0]!;
    expect(p.glyph).toBe("4.00");
    expect(p.kind).toBe("Distance");
    expect(p.x).toBeCloseTo(2);   // midpoint of canvas-x between (0,0) and (4,0)
    expect(p.y).toBeCloseTo(-10); // above
  });

  // ---- FixedPoint ----

  it("FixedPoint constraint → 1 pin badge above the point", () => {
    const sketch: Sketch = {
      plane: "Xy",
      primitives: [{ kind: "Point", id: "p1", x: 3, y: 1 }],
      constraints: [{ kind: "FixedPoint", point: "p1" }],
    };
    const placements = glyphPlacements(sketch, VP);
    expect(placements).toHaveLength(1);
    const p = placements[0]!;
    expect(p.glyph).toBe("📌");
    expect(p.kind).toBe("FixedPoint");
    expect(p.x).toBeCloseTo(3);
    expect(p.y).toBeCloseTo(-11); // canvas y = 0 - 1*1 = -1, nudge -10 = -11
  });

  // ---- Multiple constraints → multiple glyphs, no overlapping placements ----

  it("Multiple constraints produce multiple glyphs with differing positions (≥ 6 px apart)", () => {
    const sketch = twoLinesSketch([
      { kind: "Horizontal", line: "l1" },
      { kind: "Vertical", line: "l2" },
      { kind: "Perpendicular", line_a: "l1", line_b: "l2" },
    ]);
    const placements = glyphPlacements(sketch, VP10);
    expect(placements).toHaveLength(3);

    // All pairs must differ by at least 6 px in at least one axis.
    for (let i = 0; i < placements.length; i++) {
      for (let j = i + 1; j < placements.length; j++) {
        const a = placements[i]!;
        const b = placements[j]!;
        const dx = Math.abs(a.x - b.x);
        const dy = Math.abs(a.y - b.y);
        expect(dx >= 6 || dy >= 6).toBe(true);
      }
    }
  });

  // ---- Missing primitive ids → gracefully skipped ----

  it("constraint referencing a missing primitive id is silently skipped", () => {
    const sketch: Sketch = {
      plane: "Xy",
      primitives: [
        { kind: "Point", id: "p1", x: 0, y: 0 },
      ],
      constraints: [{ kind: "Horizontal", line: "GHOST" }],
    };
    expect(glyphPlacements(sketch, VP)).toEqual([]);
  });
});
