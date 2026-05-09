// Tests for hole-table: extractHoles + renderHoleTable (hole-table.ts).
// Covers: empty model, single Counterbore, HoleArray (N holes), stable
// letter assignment.

import { describe, it, expect } from "vitest";
import { extractHoles } from "./hole-table.js";

// ---------------------------------------------------------------------------
// Helpers — minimal model JSON builders
// ---------------------------------------------------------------------------

function modelWith(features: unknown[]): string {
  return JSON.stringify({ features });
}

// ---------------------------------------------------------------------------
// Empty model → no holes
// ---------------------------------------------------------------------------

describe("extractHoles — empty model", () => {
  it("returns [] for a model with no features", () => {
    expect(extractHoles(modelWith([]))).toEqual([]);
  });

  it("returns [] when features only contain non-hole kinds (Box, Cylinder)", () => {
    const json = modelWith([
      { kind: "Box", id: "box0", extents: [10, 20, 30] },
      { kind: "Cylinder", id: "cyl0", radius: 5, height: 15, segments: 16 },
    ]);
    expect(extractHoles(json)).toEqual([]);
  });
});

// ---------------------------------------------------------------------------
// Single Counterbore → 1 hole with correct fields
// ---------------------------------------------------------------------------

describe("extractHoles — Counterbore", () => {
  it("extracts one hole with diameter = 2 × drill_radius and correct depth", () => {
    const json = modelWith([
      {
        kind: "Counterbore",
        id: "cb0",
        input: "box0",
        axis: "z",
        top_center: [5, 10, 20],
        drill_radius: 3,
        cbore_radius: 5,
        cbore_depth: 4,
        total_depth: 12,
        segments: 16,
      },
    ]);
    const holes = extractHoles(json);
    expect(holes).toHaveLength(1);
    const h = holes[0]!;
    expect(h.tag).toBe("A1");
    expect(h.type).toBe("Counterbore");
    expect(h.diameter).toBeCloseTo(6); // 2 × drill_radius
    expect(h.depth).toBeCloseTo(12);   // total_depth
    expect(h.x).toBeCloseTo(5);
    expect(h.y).toBeCloseTo(10);
    expect(h.z).toBeCloseTo(20);
  });
});

// ---------------------------------------------------------------------------
// Single Countersink → 1 hole
// ---------------------------------------------------------------------------

describe("extractHoles — Countersink", () => {
  it("extracts one hole with diameter = 2 × drill_radius", () => {
    const json = modelWith([
      {
        kind: "Countersink",
        id: "cs0",
        input: "box0",
        axis: "z",
        top_center: [1, 2, 3],
        drill_radius: 2,
        csink_radius: 4,
        csink_depth: 2,
        total_depth: 8,
        segments: 16,
      },
    ]);
    const holes = extractHoles(json);
    expect(holes).toHaveLength(1);
    expect(holes[0]!.type).toBe("Countersink");
    expect(holes[0]!.diameter).toBeCloseTo(4); // 2 × drill_radius
    expect(holes[0]!.depth).toBeCloseTo(8);
  });
});

// ---------------------------------------------------------------------------
// HoleArray → N holes (one per element)
// ---------------------------------------------------------------------------

describe("extractHoles — HoleArray", () => {
  it("expands a HoleArray with count=3 into 3 holes", () => {
    const json = modelWith([
      {
        kind: "HoleArray",
        id: "ha0",
        input: "box0",
        axis: "z",
        start: [0, 0, 10],
        offset: [5, 0, 0],
        count: 3,
        radius: 2,
        depth: 10,
        segments: 16,
      },
    ]);
    const holes = extractHoles(json);
    expect(holes).toHaveLength(3);
    // Check tags: A1, A2, A3
    expect(holes.map((h) => h.tag)).toEqual(["A1", "A2", "A3"]);
    // Check positions: start + i * offset
    expect(holes[0]!.x).toBeCloseTo(0);
    expect(holes[1]!.x).toBeCloseTo(5);
    expect(holes[2]!.x).toBeCloseTo(10);
    // diameter = 2 × radius
    holes.forEach((h) => expect(h.diameter).toBeCloseTo(4));
    holes.forEach((h) => expect(h.depth).toBeCloseTo(10));
  });
});

// ---------------------------------------------------------------------------
// BoltCircle → N holes (evenly spaced around circle)
// ---------------------------------------------------------------------------

describe("extractHoles — BoltCircle", () => {
  it("expands a BoltCircle with count=4 into 4 holes", () => {
    const json = modelWith([
      {
        kind: "BoltCircle",
        id: "bc0",
        input: "box0",
        axis: "z",
        center: [0, 0, 10],
        bolt_circle_radius: 20,
        count: 4,
        radius: 3,
        depth: 8,
        segments: 16,
      },
    ]);
    const holes = extractHoles(json);
    expect(holes).toHaveLength(4);
    expect(holes.map((h) => h.tag)).toEqual(["A1", "A2", "A3", "A4"]);
    // All holes share same diameter/depth
    holes.forEach((h) => {
      expect(h.diameter).toBeCloseTo(6);
      expect(h.depth).toBeCloseTo(8);
    });
    // Positions lie on bolt circle (distance from center = bolt_circle_radius)
    for (const h of holes) {
      const dx = h.x - 0;
      const dy = h.y - 0;
      expect(Math.hypot(dx, dy)).toBeCloseTo(20);
    }
  });
});

// ---------------------------------------------------------------------------
// HexHole → 1 hole (diameter = 2 × inscribed_radius)
// ---------------------------------------------------------------------------

describe("extractHoles — HexHole", () => {
  it("extracts a HexHole with diameter from inscribed_radius", () => {
    const json = modelWith([
      {
        kind: "HexHole",
        id: "hh0",
        input: "box0",
        axis: "z",
        top_center: [3, 4, 5],
        inscribed_radius: 6,
        depth: 15,
      },
    ]);
    const holes = extractHoles(json);
    expect(holes).toHaveLength(1);
    expect(holes[0]!.type).toBe("HexHole");
    expect(holes[0]!.diameter).toBeCloseTo(12); // 2 × inscribed_radius
    expect(holes[0]!.depth).toBeCloseTo(15);
  });
});

// ---------------------------------------------------------------------------
// SquareHole → 1 hole (diameter reported as side length)
// ---------------------------------------------------------------------------

describe("extractHoles — SquareHole", () => {
  it("extracts a SquareHole with diameter = side", () => {
    const json = modelWith([
      {
        kind: "SquareHole",
        id: "sq0",
        input: "box0",
        axis: "z",
        top_center: [0, 0, 0],
        side: 8,
        depth: 5,
      },
    ]);
    const holes = extractHoles(json);
    expect(holes).toHaveLength(1);
    expect(holes[0]!.type).toBe("SquareHole");
    expect(holes[0]!.diameter).toBeCloseTo(8);
    expect(holes[0]!.depth).toBeCloseTo(5);
  });
});

// ---------------------------------------------------------------------------
// Letter assignment is stable (alphabetical by feature id)
// ---------------------------------------------------------------------------

describe("extractHoles — stable letter assignment", () => {
  it("assigns tags in id-sorted order so A1 always belongs to the same feature", () => {
    const json = modelWith([
      // Insert in non-alphabetical order
      {
        kind: "Counterbore",
        id: "zz_last",
        input: "box0",
        axis: "z",
        top_center: [0, 0, 0],
        drill_radius: 1,
        cbore_radius: 2,
        cbore_depth: 1,
        total_depth: 5,
        segments: 16,
      },
      {
        kind: "Counterbore",
        id: "aa_first",
        input: "box0",
        axis: "z",
        top_center: [10, 0, 0],
        drill_radius: 1,
        cbore_radius: 2,
        cbore_depth: 1,
        total_depth: 5,
        segments: 16,
      },
    ]);
    const holes = extractHoles(json);
    expect(holes).toHaveLength(2);
    // aa_first sorts before zz_last → gets A1
    const a1 = holes.find((h) => h.tag === "A1")!;
    const a2 = holes.find((h) => h.tag === "A2")!;
    expect(a1.x).toBeCloseTo(10); // aa_first's x
    expect(a2.x).toBeCloseTo(0);  // zz_last's x
  });

  it("tags are stable across multiple calls with same input", () => {
    const json = modelWith([
      {
        kind: "Countersink",
        id: "cs_b",
        input: "box0",
        axis: "z",
        top_center: [0, 0, 0],
        drill_radius: 2,
        csink_radius: 4,
        csink_depth: 2,
        total_depth: 8,
        segments: 16,
      },
    ]);
    const first = extractHoles(json);
    const second = extractHoles(json);
    expect(first[0]!.tag).toBe(second[0]!.tag);
  });
});

// ---------------------------------------------------------------------------
// Mixed features — only hole-producing ones are extracted
// ---------------------------------------------------------------------------

describe("extractHoles — mixed model", () => {
  it("ignores non-hole features and correctly extracts hole features", () => {
    const json = modelWith([
      { kind: "Box", id: "base", extents: [100, 100, 20] },
      {
        kind: "Counterbore",
        id: "cb_main",
        input: "base",
        axis: "z",
        top_center: [25, 25, 20],
        drill_radius: 4,
        cbore_radius: 7,
        cbore_depth: 3,
        total_depth: 20,
        segments: 16,
      },
      { kind: "Cylinder", id: "pin", radius: 3, height: 10, segments: 16 },
      {
        kind: "HoleArray",
        id: "ha_edge",
        input: "base",
        axis: "z",
        start: [10, 10, 20],
        offset: [20, 0, 0],
        count: 2,
        radius: 2,
        depth: 20,
        segments: 16,
      },
    ]);
    const holes = extractHoles(json);
    // 1 Counterbore + 2 from HoleArray = 3 holes
    expect(holes).toHaveLength(3);
    const types = holes.map((h) => h.type).sort();
    expect(types).toEqual(["Counterbore", "HoleArray", "HoleArray"]);
  });
});
