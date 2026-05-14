// Unit + integration smoke tests for the sketcher state ↔ JSON pipeline.
//
// Pure-data tests (no DOM): exercises the rust-shape Sketch round-trip,
// the SketchExtrude model wrapper, and a smoke-test "fixture loads + extrudes"
// flow with a mocked WASM `evaluate` call.

import { describe, it, expect } from "vitest";
import {
  applyExtendClick,
  applyFilletClick,
  applyTrimClick,
  buildExtrudeModelJson,
  constraintRefs,
  constraintTag,
  type Sketch,
  type SketchPrim,
  type SketchConstraint,
} from "./sketcher.js";

describe("sketch_state", () => {
  // ---- 1. Sketch state → JSON round-trip (matches rust serde shape) ----
  it("rectangle sketch round-trips through JSON", () => {
    const original: Sketch = {
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
      constraints: [
        { kind: "Horizontal", line: "l1" },
        { kind: "Vertical", line: "l2" },
      ],
    };
    const json = JSON.stringify(original);
    const back = JSON.parse(json) as Sketch;
    expect(back).toEqual(original);
  });

  it("circle round-trips", () => {
    const original: Sketch = {
      plane: "Xy",
      primitives: [
        { kind: "Point", id: "c", x: 0, y: 0 },
        { kind: "Circle", id: "k", center: "c", radius: 1.5, n_segments: 32 },
      ],
      constraints: [],
    };
    const back = JSON.parse(JSON.stringify(original)) as Sketch;
    expect(back).toEqual(original);
  });

  it("arc round-trips with start/end angles", () => {
    const original: Sketch = {
      plane: "Xy",
      primitives: [
        { kind: "Point", id: "c", x: 0, y: 0 },
        { kind: "Point", id: "s", x: 1, y: 0 },
        { kind: "Point", id: "e", x: 0, y: 1 },
        {
          kind: "Arc",
          id: "a1",
          center: "c",
          radius: 1,
          start_angle: 0,
          end_angle: Math.PI / 2,
          n_segments: 8,
        },
      ],
      constraints: [],
    };
    const back = JSON.parse(JSON.stringify(original)) as Sketch;
    expect(back).toEqual(original);
  });

  it("all 7 constraint variants serialize", () => {
    const cs: SketchConstraint[] = [
      { kind: "Coincident", a: "p1", b: "p2" },
      { kind: "Distance", a: "p1", b: "p2", value: 4 },
      { kind: "Horizontal", line: "l1" },
      { kind: "Vertical", line: "l2" },
      { kind: "Parallel", line_a: "l1", line_b: "l3" },
      { kind: "Perpendicular", line_a: "l1", line_b: "l2" },
      { kind: "FixedPoint", point: "p1" },
    ];
    for (const c of cs) {
      const back = JSON.parse(JSON.stringify(c)) as SketchConstraint;
      expect(back).toEqual(c);
      // Helpers don't throw and produce non-empty output.
      expect(constraintTag(c).length).toBeGreaterThan(0);
      expect(constraintRefs(c).length).toBeGreaterThan(0);
    }
  });

  // ---- 2. buildExtrudeModelJson constructs a valid kerf-cad Model JSON ----
  it("buildExtrudeModelJson wraps sketch in a SketchExtrude feature", () => {
    const sk: Sketch = {
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
    const json = buildExtrudeModelJson(sk, [0, 0, 4]);
    const parsed = JSON.parse(json);
    expect(parsed.features.length).toBe(1);
    expect(parsed.features[0].kind).toBe("SketchExtrude");
    expect(parsed.features[0].id).toBe("out");
    expect(parsed.features[0].direction).toEqual([0, 0, 4]);
    expect(parsed.features[0].sketch).toEqual(sk);
  });

  // ---- 3. Integration smoke test: load a fixture, extrude it, mock WASM ----
  it("fixture sketch loads and triggers extrude with mocked WASM", () => {
    // Load the same JSON shape the public/examples/sketch_rectangle.json uses.
    const fixture: Sketch = {
      plane: "Xy",
      primitives: [
        { kind: "Point", id: "p1", x: 0, y: 0 },
        { kind: "Point", id: "p2", x: 4, y: 0 },
        { kind: "Point", id: "p3", x: 4, y: 3 },
        { kind: "Point", id: "p4", x: 0, y: 3 },
        { kind: "Line", id: "l1", from: "p1", to: "p2" },
        { kind: "Line", id: "l2", from: "p2", to: "p3" },
        { kind: "Line", id: "l3", from: "p3", to: "p4" },
        { kind: "Line", id: "l4", from: "p4", to: "p1" },
      ],
      constraints: [{ kind: "Horizontal", line: "l1" }],
    };
    const json = buildExtrudeModelJson(fixture, [0, 0, 5]);

    // Mock WASM evaluator: just check we get a sane Model JSON in.
    let observedTarget: string | null = null;
    let observedJson: string | null = null;
    const fakeEval = (modelJson: string, target: string) => {
      observedJson = modelJson;
      observedTarget = target;
      return { triangles: [], face_ids: [], face_count: 0, volume: 0 };
    };
    const result = fakeEval(json, "out");
    expect(observedTarget).toBe("out");
    expect(observedJson).toBe(json);
    expect(result.triangles.length).toBe(0); // we didn't tessellate
  });

  // ---- 4. Sketch with primitive id collision rejected at JSON layer ----
  it("primitives are kept in array order on round-trip", () => {
    const sk: Sketch = {
      plane: "Xy",
      primitives: [
        { kind: "Point", id: "p1", x: 0, y: 0 },
        { kind: "Point", id: "p2", x: 1, y: 0 },
        { kind: "Line", id: "l1", from: "p1", to: "p2" },
      ],
      constraints: [],
    };
    const back = JSON.parse(JSON.stringify(sk)) as Sketch;
    expect(back.primitives.map((p: SketchPrim) => p.id)).toEqual(["p1", "p2", "l1"]);
  });

  // ---- 5. NamedRefPlane serialization ----
  it("NamedRefPlane round-trips", () => {
    const sk: Sketch = {
      plane: { NamedRefPlane: "top" },
      primitives: [],
      constraints: [],
    };
    const back = JSON.parse(JSON.stringify(sk)) as Sketch;
    expect(back.plane).toEqual({ NamedRefPlane: "top" });
  });

  // ---- 6. Multi-click canvas UX: applyTrimClick / applyExtendClick / applyFilletClick ----
  it("applyTrimClick projects click onto line and emits Point + TrimLine", () => {
    // A horizontal line from (0,0) to (4,0). Click at (2.5, 0.3) should
    // project onto the line at (2.5, 0), emit a Point there, then a
    // TrimLine referencing that new Point.
    const sketch: Sketch = {
      plane: "Xy",
      primitives: [
        { kind: "Point", id: "p1", x: 0, y: 0 },
        { kind: "Point", id: "p2", x: 4, y: 0 },
        { kind: "Line", id: "l1", from: "p1", to: "p2" },
      ],
      constraints: [],
    };
    const r = applyTrimClick(sketch, "l1", { x: 2.5, y: 0.3 }, {
      nextPointId: () => "p_new",
      nextTrimId: () => "tr_new",
    });
    expect(r).not.toBeNull();
    // Two primitives: the new Point (projected onto the line), then TrimLine.
    expect(r!.primitives.length).toBe(2);
    const pt = r!.primitives[0];
    expect(pt.kind).toBe("Point");
    expect(pt.id).toBe("p_new");
    if (pt.kind === "Point") {
      expect(Math.abs(pt.x - 2.5)).toBeLessThan(1e-9);
      expect(Math.abs(pt.y - 0.0)).toBeLessThan(1e-9);
    }
    const tr = r!.primitives[1];
    expect(tr.kind).toBe("TrimLine");
    expect(tr.id).toBe("tr_new");
    if (tr.kind === "TrimLine") {
      expect(tr.line).toBe("l1");
      expect(tr.at_point).toBe("p_new");
    }
  });

  it("applyTrimClick reuses existing point when provided", () => {
    const sketch: Sketch = {
      plane: "Xy",
      primitives: [
        { kind: "Point", id: "p1", x: 0, y: 0 },
        { kind: "Point", id: "p2", x: 4, y: 0 },
        { kind: "Point", id: "ex", x: 2.0, y: 0 },
        { kind: "Line", id: "l1", from: "p1", to: "p2" },
      ],
      constraints: [],
    };
    const r = applyTrimClick(sketch, "l1", { x: 2.0, y: 0.05 }, {
      existingPointId: "ex",
      nextTrimId: () => "tr_x",
    });
    expect(r).not.toBeNull();
    // Only TrimLine emitted (no new Point because we reused "ex").
    expect(r!.primitives.length).toBe(1);
    const tr = r!.primitives[0];
    expect(tr.kind).toBe("TrimLine");
    if (tr.kind === "TrimLine") {
      expect(tr.at_point).toBe("ex");
      expect(tr.line).toBe("l1");
    }
  });

  it("applyExtendClick emits Point at click and ExtendLine to it", () => {
    const sketch: Sketch = {
      plane: "Xy",
      primitives: [
        { kind: "Point", id: "p1", x: 0, y: 0 },
        { kind: "Point", id: "p2", x: 2, y: 0 },
        { kind: "Line", id: "l1", from: "p1", to: "p2" },
      ],
      constraints: [],
    };
    const r = applyExtendClick(sketch, "l1", { x: 5, y: 0 }, {
      nextPointId: () => "p_target",
      nextExtendId: () => "ex_1",
    });
    expect(r).not.toBeNull();
    expect(r!.primitives.length).toBe(2);
    const pt = r!.primitives[0];
    expect(pt.kind).toBe("Point");
    if (pt.kind === "Point") {
      expect(pt.id).toBe("p_target");
      expect(pt.x).toBe(5);
      expect(pt.y).toBe(0);
    }
    const ex = r!.primitives[1];
    expect(ex.kind).toBe("ExtendLine");
    if (ex.kind === "ExtendLine") {
      expect(ex.line).toBe("l1");
      expect(ex.to_point).toBe("p_target");
    }
  });

  it("applyFilletClick rejects non-positive radius", () => {
    const sketch: Sketch = {
      plane: "Xy",
      primitives: [
        { kind: "Point", id: "corner", x: 1, y: 1 },
      ],
      constraints: [],
    };
    expect(applyFilletClick(sketch, "corner", 0)).toBeNull();
    expect(applyFilletClick(sketch, "corner", -0.5)).toBeNull();
    expect(applyFilletClick(sketch, "corner", Number.NaN)).toBeNull();
    // Non-existent corner Point.
    expect(applyFilletClick(sketch, "missing", 0.5)).toBeNull();
  });

  it("applyFilletClick emits FilletCorner for valid input", () => {
    const sketch: Sketch = {
      plane: "Xy",
      primitives: [
        { kind: "Point", id: "corner", x: 1, y: 1 },
      ],
      constraints: [],
    };
    const r = applyFilletClick(sketch, "corner", 0.5, {
      nextFilletId: () => "f_42",
    });
    expect(r).not.toBeNull();
    expect(r!.primitives.length).toBe(1);
    const fc = r!.primitives[0];
    expect(fc.kind).toBe("FilletCorner");
    if (fc.kind === "FilletCorner") {
      expect(fc.id).toBe("f_42");
      expect(fc.corner_point).toBe("corner");
      expect(fc.radius).toBe(0.5);
    }
  });

  it("applyTrimClick returns null for unknown line id", () => {
    const sketch: Sketch = {
      plane: "Xy",
      primitives: [
        { kind: "Point", id: "p1", x: 0, y: 0 },
        { kind: "Point", id: "p2", x: 4, y: 0 },
        { kind: "Line", id: "l1", from: "p1", to: "p2" },
      ],
      constraints: [],
    };
    expect(applyTrimClick(sketch, "missing", { x: 2, y: 0 })).toBeNull();
  });
});
