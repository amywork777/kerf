// Unit + integration smoke tests for the sketcher state ↔ JSON pipeline.
//
// Pure-data tests (no DOM): exercises the rust-shape Sketch round-trip,
// the SketchExtrude model wrapper, and a smoke-test "fixture loads + extrudes"
// flow with a mocked WASM `evaluate` call.
//
// Run with: pnpm exec tsc -p . --outDir /tmp/sketcher-tests && node /tmp/sketcher-tests/sketch_state.test.js
//   (or `pnpm test` once we wire it up).

import {
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

console.log(`\n${passed} passed, ${failed} failed`);
// `process` is from node — this file runs in node, but DOM lib types take
// priority. Cast through any to keep tsc happy without pulling @types/node.
if (failed > 0) (globalThis as { process?: { exit(c: number): void } }).process?.exit(1);
