// Tests for the Dimensions panel's pure helpers and the WASM call shape.
// We intentionally avoid jsdom — the panel's state machine is exposed
// through `buildDimension` / `buildRenderArgs`, which are what the SVG
// renderer ultimately consumes. The DOM-bound DimensionsPanel class is a
// thin wrapper around them, so verifying the wiring at this level keeps
// the test fast and free of jsdom config.

// vitest types — not pulling in @types/node, so use the global describe/it.
import { describe, it, expect, vi } from "vitest";
import * as THREE from "three";

import {
  buildDimension,
  buildRenderArgs,
  describeDimension,
  picksRequired,
} from "./dimensions.js";

describe("buildDimension — panel state → JSON model", () => {
  it("packs linear picks into { kind, from, to }", () => {
    const a = new THREE.Vector3(1, 2, 3);
    const b = new THREE.Vector3(4, 5, 6);
    const d = buildDimension("linear", [a, b]);
    expect(d).toEqual({
      kind: "linear",
      from: [1, 2, 3],
      to: [4, 5, 6],
    });
  });

  it("packs radial picks: pts[0] → center & from, pts[1] → to", () => {
    const center = new THREE.Vector3(0, 0, 0);
    const onCircle = new THREE.Vector3(2, 0, 0);
    const d = buildDimension("radial", [center, onCircle]);
    expect(d).toEqual({
      kind: "radial",
      center: [0, 0, 0],
      from: [0, 0, 0],
      to: [2, 0, 0],
    });
  });

  it("packs angular picks: pts[0] → vertex, pts[1] → from, pts[2] → to", () => {
    const v = new THREE.Vector3(0, 0, 0);
    const a = new THREE.Vector3(1, 0, 0);
    const b = new THREE.Vector3(0, 1, 0);
    const d = buildDimension("angular", [v, a, b]);
    expect(d).toEqual({
      kind: "angular",
      vertex: [0, 0, 0],
      from: [1, 0, 0],
      to: [0, 1, 0],
    });
  });

  it("rejects wrong pick counts", () => {
    expect(() =>
      buildDimension("linear", [new THREE.Vector3()]),
    ).toThrowError(/needs 2 picks/);
    expect(() =>
      buildDimension("angular", [new THREE.Vector3(), new THREE.Vector3()]),
    ).toThrowError(/needs 3 picks/);
  });

  it("picksRequired matches dimension kind contract", () => {
    expect(picksRequired("linear")).toBe(2);
    expect(picksRequired("radial")).toBe(2);
    expect(picksRequired("angular")).toBe(3);
  });

  it("describeDimension formats human-readable summaries", () => {
    const lin = buildDimension("linear", [
      new THREE.Vector3(0, 0, 0),
      new THREE.Vector3(3, 4, 0),
    ]);
    expect(describeDimension(lin)).toContain("5.00"); // |(3,4,0)| = 5
    const rad = buildDimension("radial", [
      new THREE.Vector3(0, 0, 0),
      new THREE.Vector3(0, 0, 7),
    ]);
    expect(describeDimension(rad)).toContain("R=7.00");
    const ang = buildDimension("angular", [
      new THREE.Vector3(0, 0, 0),
      new THREE.Vector3(1, 0, 0),
      new THREE.Vector3(0, 1, 0),
    ]);
    // 90° between +X and +Y at origin.
    expect(describeDimension(ang)).toContain("90.00°");
  });
});

describe("render_drawing_svg smoke test", () => {
  it("buildRenderArgs serializes panel state into the WASM 6-arg shape", () => {
    const args = buildRenderArgs({
      json: '{"features":[]}',
      targetId: "out",
      parameters: { plate_x: 10 },
      view: "top",
      dimensions: [
        buildDimension("linear", [
          new THREE.Vector3(0, 0, 0),
          new THREE.Vector3(2, 0, 0),
        ]),
      ],
    });

    // Shape matches kerf_cad_wasm::render_drawing_svg(json, target_id,
    // params_json, view, dimensions_json, viewport_json).
    expect(args.json).toBe('{"features":[]}');
    expect(args.targetId).toBe("out");
    expect(JSON.parse(args.paramsJson)).toEqual({ plate_x: 10 });
    expect(args.view).toBe("top");
    const dims = JSON.parse(args.dimensionsJson);
    expect(Array.isArray(dims)).toBe(true);
    expect(dims).toHaveLength(1);
    expect(dims[0]).toMatchObject({
      kind: "linear",
      from: [0, 0, 0],
      to: [2, 0, 0],
    });
    // Default viewport is `{}` (lets the back-end pick A4-ish).
    expect(JSON.parse(args.viewportJson)).toEqual({});
  });

  it("mocked WASM render_drawing_svg is called with the 6 packed args", () => {
    // Stand-in for the real WASM export, with the same 6-arg signature.
    const fakeRenderSvg = vi.fn(
      (
        _json: string,
        _targetId: string,
        _paramsJson: string,
        _view: string,
        _dimensionsJson: string,
        _viewportJson: string,
      ) => "<svg></svg>",
    );

    const args = buildRenderArgs({
      json: '{"x":1}',
      targetId: "out",
      parameters: {},
      view: "front",
      dimensions: [
        buildDimension("radial", [
          new THREE.Vector3(0, 0, 0),
          new THREE.Vector3(5, 0, 0),
        ]),
      ],
      viewport: { width: 800, height: 600, padding: 32 },
    });

    // Mirror what DimensionsPanel.downloadSvg does: feed the args through.
    fakeRenderSvg(
      args.json,
      args.targetId,
      args.paramsJson,
      args.view,
      args.dimensionsJson,
      args.viewportJson,
    );

    expect(fakeRenderSvg).toHaveBeenCalledTimes(1);
    const call = fakeRenderSvg.mock.calls[0]!;
    expect(call[0]).toBe('{"x":1}');
    expect(call[1]).toBe("out");
    expect(JSON.parse(call[2])).toEqual({});
    expect(call[3]).toBe("front");
    const passedDims = JSON.parse(call[4]);
    expect(passedDims[0].kind).toBe("radial");
    expect(passedDims[0].center).toEqual([0, 0, 0]);
    expect(passedDims[0].to).toEqual([5, 0, 0]);
    expect(JSON.parse(call[5])).toEqual({
      width: 800,
      height: 600,
      padding: 32,
    });
  });
});
