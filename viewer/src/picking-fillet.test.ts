import { describe, it, expect } from "vitest";
import {
  applyFilletToModelJson,
  buildFilletFeature,
  isEdgeFilletable,
  type EdgeOut,
} from "./picking-fillet";

const filletableEdge: EdgeOut = {
  id: 7,
  owner_tag: "body",
  p_start: [0.0, 0.0, 0.0],
  p_end: [0.0, 0.0, 4.5],
  length: 4.5,
  curve_kind: "line",
  axis_hint: "z",
  edge_min_hint: [0.0, 0.0, 0.0],
  edge_length_hint: 4.5,
  quadrant_hint: "pp",
};

const circleEdge: EdgeOut = {
  ...filletableEdge,
  id: 9,
  curve_kind: "circle",
  axis_hint: "",
  quadrant_hint: "",
};

describe("isEdgeFilletable", () => {
  it("accepts axis-aligned line edges with a quadrant hint", () => {
    expect(isEdgeFilletable(filletableEdge)).toBe(true);
  });

  it("rejects circle edges", () => {
    expect(isEdgeFilletable(circleEdge)).toBe(false);
  });

  it("rejects edges that lack an axis hint (e.g. diagonal)", () => {
    const diag: EdgeOut = { ...filletableEdge, axis_hint: "" };
    expect(isEdgeFilletable(diag)).toBe(false);
  });

  it("rejects edges with no resolvable quadrant", () => {
    const noQuad: EdgeOut = { ...filletableEdge, quadrant_hint: "" };
    expect(isEdgeFilletable(noQuad)).toBe(false);
  });
});

describe("buildFilletFeature", () => {
  it("emits a Fillet feature with the expected shape", () => {
    const f = buildFilletFeature("body", filletableEdge, 0.5, "abc");
    expect(f).toMatchObject({
      kind: "Fillet",
      id: "fillet_e7_abc",
      input: "body",
      axis: "z",
      edge_min: [0.0, 0.0, 0.0],
      edge_length: 4.5,
      radius: 0.5,
      quadrant: "pp",
      segments: 8,
    });
  });

  it("uses a random suffix when not given one", () => {
    const a = buildFilletFeature("body", filletableEdge, 0.5);
    const b = buildFilletFeature("body", filletableEdge, 0.5);
    // ids should both start with the same prefix but the suffixes should
    // virtually never coincide.
    expect(String(a.id)).toMatch(/^fillet_e7_/);
    expect(String(b.id)).toMatch(/^fillet_e7_/);
  });
});

describe("applyFilletToModelJson", () => {
  const baseModel = {
    parameters: { plate_x: 10.0 },
    features: [
      { kind: "Box", id: "body", extents: ["$plate_x", 5, 2] },
    ],
  };

  it("appends a Fillet feature pointing at the current target", () => {
    const json = JSON.stringify(baseModel);
    const result = applyFilletToModelJson(json, "body", filletableEdge, 0.5, "xy");
    expect(result).not.toBeNull();
    const parsed = JSON.parse(result!.json);
    expect(parsed.features).toHaveLength(2);
    expect(parsed.features[1]).toMatchObject({
      kind: "Fillet",
      id: "fillet_e7_xy",
      input: "body",
      axis: "z",
      edge_min: [0.0, 0.0, 0.0],
      edge_length: 4.5,
      radius: 0.5,
      quadrant: "pp",
      segments: 8,
    });
    expect(result!.newId).toBe("fillet_e7_xy");
    expect(result!.ids).toEqual(["body", "fillet_e7_xy"]);
  });

  it("returns null on a non-filletable edge", () => {
    const json = JSON.stringify(baseModel);
    expect(applyFilletToModelJson(json, "body", circleEdge, 0.5)).toBeNull();
    // The original JSON should not have been parsed-then-re-emitted into
    // anything we'd inadvertently use; the call returns null cleanly.
  });

  it("preserves the rest of the model (parameters, prior features)", () => {
    const json = JSON.stringify(baseModel);
    const result = applyFilletToModelJson(json, "body", filletableEdge, 1.0, "z");
    const parsed = JSON.parse(result!.json);
    expect(parsed.parameters).toEqual({ plate_x: 10.0 });
    expect(parsed.features[0]).toMatchObject({ kind: "Box", id: "body" });
  });

  it("chains: fillet → fillet uses the previous fillet as input", () => {
    let json = JSON.stringify(baseModel);
    let target = "body";
    const r1 = applyFilletToModelJson(json, target, filletableEdge, 0.5, "a");
    json = r1!.json;
    target = r1!.newId;
    const e2: EdgeOut = { ...filletableEdge, id: 11 };
    const r2 = applyFilletToModelJson(json, target, e2, 0.5, "b");
    const parsed = JSON.parse(r2!.json);
    expect(parsed.features).toHaveLength(3);
    expect(parsed.features[2].input).toBe("fillet_e7_a");
    expect(parsed.features[2].id).toBe("fillet_e11_b");
  });
});
