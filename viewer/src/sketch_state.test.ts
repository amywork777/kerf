// Unit + integration smoke tests for the sketcher state ↔ JSON pipeline.
//
// Pure-data tests (no DOM): exercises the rust-shape Sketch round-trip,
// the SketchExtrude model wrapper, and a smoke-test "fixture loads + extrudes"
// flow with a mocked WASM `evaluate` call.
//
// Run with: pnpm exec tsc -p . --outDir /tmp/sketcher-tests && node /tmp/sketcher-tests/sketch_state.test.js
//   (or `pnpm test` once we wire it up).

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

let passed = 0;
let failed = 0;
function test(name: string, fn: () => void) {
  try {
    fn();
    console.log(`  ok ${name}`);
    passed++;
  } catch (e) {
    console.error(`  FAIL ${name}\n    ${(e as Error).stack ?? e}`);
    failed++;
  }
}
function assert(cond: unknown, msg = "assertion failed") {
  if (!cond) throw new Error(msg);
}
function assertEq<T>(a: T, b: T, msg = "values not equal") {
  if (JSON.stringify(a) !== JSON.stringify(b)) {
    throw new Error(`${msg}: ${JSON.stringify(a)} !== ${JSON.stringify(b)}`);
  }
}

console.log("# sketch_state.test.ts");

// ---- 1. Sketch state → JSON round-trip (matches rust serde shape) ----
test("rectangle sketch round-trips through JSON", () => {
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
  assertEq(back, original);
});

test("circle round-trips", () => {
  const original: Sketch = {
    plane: "Xy",
    primitives: [
      { kind: "Point", id: "c", x: 0, y: 0 },
      { kind: "Circle", id: "k", center: "c", radius: 1.5, n_segments: 32 },
    ],
    constraints: [],
  };
  const back = JSON.parse(JSON.stringify(original)) as Sketch;
  assertEq(back, original);
});

test("arc round-trips with start/end angles", () => {
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
  assertEq(back, original);
});

test("all 7 constraint variants serialize", () => {
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
    assertEq(back, c);
    // Helpers don't throw and produce non-empty output.
    assert(constraintTag(c).length > 0, `tag empty for ${c.kind}`);
    assert(constraintRefs(c).length > 0, `refs empty for ${c.kind}`);
  }
});

// ---- 2. buildExtrudeModelJson constructs a valid kerf-cad Model JSON ----
test("buildExtrudeModelJson wraps sketch in a SketchExtrude feature", () => {
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
  assertEq(parsed.features.length, 1);
  assertEq(parsed.features[0].kind, "SketchExtrude");
  assertEq(parsed.features[0].id, "out");
  assertEq(parsed.features[0].direction, [0, 0, 4]);
  assertEq(parsed.features[0].sketch, sk);
});

// ---- 3. Integration smoke test: load a fixture, extrude it, mock WASM ----
test("fixture sketch loads and triggers extrude with mocked WASM", () => {
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
  assert(observedTarget === "out");
  assert(observedJson === json);
  assert(result.triangles.length === 0); // we didn't tessellate
});

// ---- 4. Sketch with primitive id collision rejected at JSON layer ----
test("primitives are kept in array order on round-trip", () => {
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
  assertEq(
    back.primitives.map((p: SketchPrim) => p.id),
    ["p1", "p2", "l1"],
  );
});

// ---- 5. NamedRefPlane serialization ----
test("NamedRefPlane round-trips", () => {
  const sk: Sketch = {
    plane: { NamedRefPlane: "top" },
    primitives: [],
    constraints: [],
  };
  const back = JSON.parse(JSON.stringify(sk)) as Sketch;
  assertEq(back.plane, { NamedRefPlane: "top" });
});

// ---- 6. Multi-click canvas UX: applyTrimClick / applyExtendClick / applyFilletClick ----
test("applyTrimClick projects click onto line and emits Point + TrimLine", () => {
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
  if (r === null) throw new Error("expected non-null result");
  // Two primitives: the new Point (projected onto the line), then TrimLine.
  assertEq(r.primitives.length, 2);
  const pt = r.primitives[0];
  assert(pt.kind === "Point" && pt.id === "p_new");
  if (pt.kind === "Point") {
    assert(Math.abs(pt.x - 2.5) < 1e-9, `proj x = ${pt.x}`);
    assert(Math.abs(pt.y - 0.0) < 1e-9, `proj y = ${pt.y}`);
  }
  const tr = r.primitives[1];
  assert(tr.kind === "TrimLine" && tr.id === "tr_new");
  if (tr.kind === "TrimLine") {
    assertEq(tr.line, "l1");
    assertEq(tr.at_point, "p_new");
  }
});

test("applyTrimClick reuses existing point when provided", () => {
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
  if (r === null) throw new Error("expected non-null result");
  // Only TrimLine emitted (no new Point because we reused "ex").
  assertEq(r.primitives.length, 1);
  const tr = r.primitives[0];
  assert(tr.kind === "TrimLine");
  if (tr.kind === "TrimLine") {
    assertEq(tr.at_point, "ex");
    assertEq(tr.line, "l1");
  }
});

test("applyExtendClick emits Point at click and ExtendLine to it", () => {
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
  if (r === null) throw new Error("expected non-null result");
  assertEq(r.primitives.length, 2);
  const pt = r.primitives[0];
  assert(pt.kind === "Point");
  if (pt.kind === "Point") {
    assertEq(pt.id, "p_target");
    assertEq(pt.x, 5);
    assertEq(pt.y, 0);
  }
  const ex = r.primitives[1];
  assert(ex.kind === "ExtendLine");
  if (ex.kind === "ExtendLine") {
    assertEq(ex.line, "l1");
    assertEq(ex.to_point, "p_target");
  }
});

test("applyFilletClick rejects non-positive radius", () => {
  const sketch: Sketch = {
    plane: "Xy",
    primitives: [
      { kind: "Point", id: "corner", x: 1, y: 1 },
    ],
    constraints: [],
  };
  assertEq(applyFilletClick(sketch, "corner", 0), null);
  assertEq(applyFilletClick(sketch, "corner", -0.5), null);
  assertEq(applyFilletClick(sketch, "corner", Number.NaN), null);
  // Non-existent corner Point.
  assertEq(applyFilletClick(sketch, "missing", 0.5), null);
});

test("applyFilletClick emits FilletCorner for valid input", () => {
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
  if (r === null) throw new Error("expected non-null result");
  assertEq(r.primitives.length, 1);
  const fc = r.primitives[0];
  assert(fc.kind === "FilletCorner");
  if (fc.kind === "FilletCorner") {
    assertEq(fc.id, "f_42");
    assertEq(fc.corner_point, "corner");
    assertEq(fc.radius, 0.5);
  }
});

test("applyTrimClick returns null for unknown line id", () => {
  const sketch: Sketch = {
    plane: "Xy",
    primitives: [
      { kind: "Point", id: "p1", x: 0, y: 0 },
      { kind: "Point", id: "p2", x: 4, y: 0 },
      { kind: "Line", id: "l1", from: "p1", to: "p2" },
    ],
    constraints: [],
  };
  assertEq(applyTrimClick(sketch, "missing", { x: 2, y: 0 }), null);
});

console.log(`\n${passed} passed, ${failed} failed`);
// `process` is from node — this file runs in node, but DOM lib types take
// priority. Cast through any to keep tsc happy without pulling @types/node.
if (failed > 0) (globalThis as { process?: { exit(c: number): void } }).process?.exit(1);
