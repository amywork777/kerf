import { describe, it, expect } from "vitest";
import { reorderFeatures, filterFeatures } from "./feature-tree-ops.js";

// ---- helpers ----
function makeJson(ids: string[]): string {
  return JSON.stringify({
    features: ids.map((id) => ({ id, kind: `kind_${id}` })),
  });
}

function featureIds(json: string): string[] {
  return JSON.parse(json).features.map((f: { id: string }) => f.id);
}

// ---- reorderFeatures ----

describe("reorderFeatures", () => {
  it("moves src before dst: [a,b,c] reorder a before b → [a,b,c] (no change)", () => {
    const json = makeJson(["a", "b", "c"]);
    const result = reorderFeatures(json, "a", "b", "before");
    expect(featureIds(result)).toEqual(["a", "b", "c"]);
  });

  it("moves src after dst: [a,b,c] reorder a after c → [b,c,a]", () => {
    const json = makeJson(["a", "b", "c"]);
    const result = reorderFeatures(json, "a", "c", "after");
    expect(featureIds(result)).toEqual(["b", "c", "a"]);
  });

  it("moves src before dst: [a,b,c] reorder c before b → [a,c,b]", () => {
    const json = makeJson(["a", "b", "c"]);
    const result = reorderFeatures(json, "c", "b", "before");
    expect(featureIds(result)).toEqual(["a", "c", "b"]);
  });

  it("same src and dst → no change (returns equivalent JSON)", () => {
    const json = makeJson(["a", "b", "c"]);
    const before = featureIds(json);
    const result = reorderFeatures(json, "b", "b", "after");
    expect(featureIds(result)).toEqual(before);
  });

  it("invalid src id → throws", () => {
    const json = makeJson(["a", "b", "c"]);
    expect(() => reorderFeatures(json, "x", "a", "before")).toThrow();
  });

  it("invalid dst id → throws", () => {
    const json = makeJson(["a", "b", "c"]);
    expect(() => reorderFeatures(json, "a", "z", "after")).toThrow();
  });

  it("preserves feature data (not just ids)", () => {
    const json = JSON.stringify({
      features: [
        { id: "a", kind: "Extrude", depth: 10 },
        { id: "b", kind: "Cut", depth: 5 },
        { id: "c", kind: "Fillet", radius: 2 },
      ],
    });
    const result = reorderFeatures(json, "a", "c", "after");
    const parsed = JSON.parse(result).features;
    expect(parsed[0]).toMatchObject({ id: "b", kind: "Cut", depth: 5 });
    expect(parsed[1]).toMatchObject({ id: "c", kind: "Fillet", radius: 2 });
    expect(parsed[2]).toMatchObject({ id: "a", kind: "Extrude", depth: 10 });
  });
});

// ---- filterFeatures ----

type FeatureSummary = { id: string; kind: string };

const SAMPLE: FeatureSummary[] = [
  { id: "extrude_1", kind: "Extrude" },
  { id: "cut_top", kind: "Cut" },
  { id: "fillet_edge", kind: "Fillet" },
  { id: "revolve_axis", kind: "Revolve" },
];

describe("filterFeatures", () => {
  it("empty query → all features returned", () => {
    expect(filterFeatures(SAMPLE, "")).toEqual(SAMPLE);
  });

  it("whitespace-only query → all features returned", () => {
    expect(filterFeatures(SAMPLE, "   ")).toEqual(SAMPLE);
  });

  it("substring match on id", () => {
    const result = filterFeatures(SAMPLE, "cut");
    expect(result).toHaveLength(1);
    expect(result[0]!.id).toBe("cut_top");
  });

  it("substring match on kind", () => {
    const result = filterFeatures(SAMPLE, "Fillet");
    expect(result).toHaveLength(1);
    expect(result[0]!.kind).toBe("Fillet");
  });

  it("case-insensitive match on kind", () => {
    const result = filterFeatures(SAMPLE, "fillet");
    expect(result).toHaveLength(1);
    expect(result[0]!.kind).toBe("Fillet");
  });

  it("case-insensitive match on id", () => {
    const result = filterFeatures(SAMPLE, "EXTRUDE");
    expect(result).toHaveLength(1);
    expect(result[0]!.id).toBe("extrude_1");
  });

  it("matches features whose id OR kind contains the query", () => {
    // "e" appears in id="extrude_1", id="fillet_edge", id="revolve_axis",
    // kind="Extrude", kind="Fillet", kind="Revolve" — all 4 rows match
    const result = filterFeatures(SAMPLE, "e");
    expect(result.length).toBeGreaterThan(1);
  });

  it("no matches → empty array", () => {
    const result = filterFeatures(SAMPLE, "xyzzy_no_match_123");
    expect(result).toEqual([]);
  });

  it("empty features array → empty array regardless of query", () => {
    expect(filterFeatures([], "cut")).toEqual([]);
  });
});
