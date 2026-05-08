// Unit tests for the feature-tree suppression + rollback JSON helpers.
//
// Pure-data tests (no DOM, no WASM): exercises the round-trip between
// the rust-shape model JSON and the viewer's suppression / rollback
// state. Run via `pnpm test`.

import {
  suppressedFromJson,
  rollbackFromJson,
  setSuppressedInJson,
  setRollbackInJson,
  toggleSuppressedInJson,
  rollbackIdForBarIndex,
} from "./suppression.js";

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

console.log("# suppression.test.ts");

const baseJson = JSON.stringify(
  {
    features: [
      { kind: "Box", id: "body", extents: [10, 10, 10] },
      { kind: "Box", id: "hole", extents: [2, 2, 12] },
      {
        kind: "Difference",
        id: "out",
        inputs: ["body", "hole"],
      },
    ],
  },
  null,
  2,
);

// ---- 1. Reading ----
test("suppressedFromJson returns empty Set on a fresh model", () => {
  assertEq(Array.from(suppressedFromJson(baseJson)), []);
});

test("rollbackFromJson returns null on a fresh model", () => {
  assertEq(rollbackFromJson(baseJson), null);
});

test("suppressedFromJson reads an existing array", () => {
  const j = JSON.stringify({
    features: [],
    suppressed: ["a", "b"],
  });
  const set = suppressedFromJson(j);
  assert(set.has("a"));
  assert(set.has("b"));
  assertEq(set.size, 2);
});

test("rollbackFromJson reads a string id", () => {
  const j = JSON.stringify({ features: [], rollback_to: "f3" });
  assertEq(rollbackFromJson(j), "f3");
});

// ---- 2. Writing ----
test("setSuppressedInJson writes a sorted array", () => {
  const out = setSuppressedInJson(baseJson, ["hole", "body"]);
  const parsed = JSON.parse(out);
  assertEq(parsed.suppressed, ["body", "hole"]);
});

test("setSuppressedInJson omits an empty set", () => {
  const out = setSuppressedInJson(baseJson, []);
  const parsed = JSON.parse(out);
  assert(!("suppressed" in parsed), "empty set should remove the field");
});

test("setRollbackInJson writes the id", () => {
  const out = setRollbackInJson(baseJson, "body");
  const parsed = JSON.parse(out);
  assertEq(parsed.rollback_to, "body");
});

test("setRollbackInJson clears with null", () => {
  const j = setRollbackInJson(baseJson, "body");
  const cleared = setRollbackInJson(j, null);
  const parsed = JSON.parse(cleared);
  assert(!("rollback_to" in parsed), "null should remove the field");
});

// ---- 3. Toggle (the suppression-checkbox mutator) ----
test("toggleSuppressedInJson adds an id when suppress=true", () => {
  const out = toggleSuppressedInJson(baseJson, "hole", true);
  assertEq(suppressedFromJson(out).has("hole"), true);
});

test("toggleSuppressedInJson removes an id when suppress=false", () => {
  const start = setSuppressedInJson(baseJson, ["hole", "body"]);
  const out = toggleSuppressedInJson(start, "hole", false);
  assertEq(Array.from(suppressedFromJson(out)).sort(), ["body"]);
});

test("toggleSuppressedInJson is idempotent (suppress twice → still one entry)", () => {
  const a = toggleSuppressedInJson(baseJson, "hole", true);
  const b = toggleSuppressedInJson(a, "hole", true);
  assertEq(Array.from(suppressedFromJson(b)), ["hole"]);
});

test("toggleSuppressedInJson preserves other features field", () => {
  const out = toggleSuppressedInJson(baseJson, "hole", true);
  const parsed = JSON.parse(out);
  assertEq(parsed.features.length, 3);
  assertEq(parsed.features[0].id, "body");
});

// ---- 4. Rollback bar drag ↔ rollback_to mapping ----
test("rollbackIdForBarIndex returns null when bar is at bottom", () => {
  assertEq(rollbackIdForBarIndex(["a", "b", "c"], 3), null);
});

test("rollbackIdForBarIndex returns last active feature id by index", () => {
  assertEq(rollbackIdForBarIndex(["a", "b", "c"], 2), "b");
  assertEq(rollbackIdForBarIndex(["a", "b", "c"], 1), "a");
});

test("rollbackIdForBarIndex clamps bar at top to first feature", () => {
  // index 0 means "above feature[0]" but we always keep at least one.
  assertEq(rollbackIdForBarIndex(["a", "b", "c"], 0), "a");
});

test("rollbackIdForBarIndex with empty list yields null", () => {
  assertEq(rollbackIdForBarIndex([], 0), null);
});

// ---- 5. Round-trip parity (the persistence guarantee) ----
test("rollback + suppression survive a JSON round-trip", () => {
  let json = baseJson;
  json = toggleSuppressedInJson(json, "hole", true);
  json = setRollbackInJson(json, "body");
  const reparsed = JSON.parse(json);
  assertEq(reparsed.suppressed, ["hole"]);
  assertEq(reparsed.rollback_to, "body");
});

console.log(`\n${passed} passed, ${failed} failed`);
if (failed > 0)
  (globalThis as { process?: { exit(c: number): void } }).process?.exit(1);
