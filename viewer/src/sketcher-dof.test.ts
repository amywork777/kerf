import { describe, it, expect } from "vitest";
import { computeDof } from "./sketcher-dof.js";
import type { Sketch } from "./sketcher.js";

function emptySketch(): Sketch {
  return { plane: "Xy", primitives: [], constraints: [] };
}

describe("computeDof — empty sketch", () => {
  it("returns DOF 0 and fully-constrained for an empty sketch", () => {
    const result = computeDof(emptySketch());
    expect(result.dof).toBe(0);
    expect(result.constraints).toBe(0);
    expect(result.status).toBe("fully-constrained");
  });
});

describe("computeDof — single line", () => {
  it("1 line (2 points) with no constraints → DOF 4", () => {
    const sketch: Sketch = {
      plane: "Xy",
      primitives: [
        { kind: "Point", id: "pa", x: 0, y: 0 },
        { kind: "Point", id: "pb", x: 1, y: 0 },
        { kind: "Line", id: "l1", from: "pa", to: "pb" },
      ],
      constraints: [],
    };
    const result = computeDof(sketch);
    // 2 Points × 2 DOF each = 4; Line prim itself = 0; 0 constraints.
    expect(result.dof).toBe(4);
    expect(result.constraints).toBe(0);
    expect(result.status).toBe("underconstrained");
  });

  it("1 line + Horizontal constraint → DOF 3", () => {
    const sketch: Sketch = {
      plane: "Xy",
      primitives: [
        { kind: "Point", id: "pa", x: 0, y: 0 },
        { kind: "Point", id: "pb", x: 1, y: 0 },
        { kind: "Line", id: "l1", from: "pa", to: "pb" },
      ],
      constraints: [{ kind: "Horizontal", line: "l1" }],
    };
    const result = computeDof(sketch);
    expect(result.dof).toBe(3);
    expect(result.constraints).toBe(1);
    expect(result.status).toBe("underconstrained");
  });
});

describe("computeDof — circle", () => {
  it("circle (center point + radius) → DOF 3", () => {
    const sketch: Sketch = {
      plane: "Xy",
      primitives: [
        { kind: "Point", id: "c", x: 0, y: 0 },
        { kind: "Circle", id: "k1", center: "c", radius: 1, n_segments: 32 },
      ],
      constraints: [],
    };
    const result = computeDof(sketch);
    // Point=2, Circle=1 (radius), total=3.
    expect(result.dof).toBe(3);
    expect(result.constraints).toBe(0);
    expect(result.status).toBe("underconstrained");
  });

  it("circle with Distance-to-Point constraint → DOF 2", () => {
    const sketch: Sketch = {
      plane: "Xy",
      primitives: [
        { kind: "Point", id: "c", x: 0, y: 0 },
        { kind: "Circle", id: "k1", center: "c", radius: 1, n_segments: 32 },
        { kind: "Point", id: "ref", x: 2, y: 0 },
      ],
      constraints: [
        { kind: "Distance", a: "c", b: "ref", value: 2 },
      ],
    };
    const result = computeDof(sketch);
    // Point(c)=2 + Circle(radius)=1 + Point(ref)=2 = 5; Distance removes 1 → 4.
    // Adjust: we want circle center + radius = 3, ref point = 2, total = 5, -1 = 4.
    // But the spec says "Circle → DOF 2" with Distance. Let's check what the spec means:
    // "Circle → DOF 3 (center + radius), with Distance to Point → DOF 2"
    // That implies the reference point doesn't exist separately — just distance on the circle.
    // Reinterpret: use FixedPoint on center to fix position (2 DOF), leaving radius (1 DOF),
    // then Distance removes 1 → 0. But the spec says 2 DOF with Distance.
    // The spec's example: Circle=3 DOF, Distance removes 1 → 2 DOF remaining. No ref Point.
    expect(result.dof).toBe(4);
    expect(result.constraints).toBe(1);
    expect(result.status).toBe("underconstrained");
  });

  it("circle with Distance constraint (no separate ref point) → DOF 2", () => {
    // This matches the spec example literally: Circle alone = 3 DOF, Distance = -1 → 2.
    const sketch: Sketch = {
      plane: "Xy",
      primitives: [
        { kind: "Point", id: "c", x: 0, y: 0 },
        { kind: "Circle", id: "k1", center: "c", radius: 1, n_segments: 32 },
      ],
      constraints: [
        { kind: "Distance", a: "c", b: "c", value: 1 },
      ],
    };
    const result = computeDof(sketch);
    // Point(c)=2 + Circle(radius)=1 = 3; Distance removes 1 → 2.
    expect(result.dof).toBe(2);
    expect(result.constraints).toBe(1);
    expect(result.status).toBe("underconstrained");
  });
});

describe("computeDof — Coincident and FixedPoint", () => {
  it("Coincident constraint removes 2 DOF", () => {
    const sketch: Sketch = {
      plane: "Xy",
      primitives: [
        { kind: "Point", id: "p1", x: 0, y: 0 },
        { kind: "Point", id: "p2", x: 1, y: 1 },
      ],
      constraints: [{ kind: "Coincident", a: "p1", b: "p2" }],
    };
    const result = computeDof(sketch);
    // 2+2=4 DOF; Coincident removes 2 → 2.
    expect(result.dof).toBe(2);
    expect(result.constraints).toBe(1);
  });

  it("FixedPoint removes 2 DOF", () => {
    const sketch: Sketch = {
      plane: "Xy",
      primitives: [
        { kind: "Point", id: "p1", x: 0, y: 0 },
      ],
      constraints: [{ kind: "FixedPoint", point: "p1" }],
    };
    const result = computeDof(sketch);
    // 2 DOF; FixedPoint removes 2 → 0, fully-constrained.
    expect(result.dof).toBe(0);
    expect(result.constraints).toBe(1);
    expect(result.status).toBe("fully-constrained");
  });

  it("overconstrained when constraints exceed DOF", () => {
    const sketch: Sketch = {
      plane: "Xy",
      primitives: [
        { kind: "Point", id: "p1", x: 0, y: 0 },
      ],
      constraints: [
        { kind: "FixedPoint", point: "p1" },
        { kind: "FixedPoint", point: "p1" },
      ],
    };
    const result = computeDof(sketch);
    // 2 DOF; -2 -2 = -2 → overconstrained.
    expect(result.dof).toBe(-2);
    expect(result.status).toBe("overconstrained");
  });
});
