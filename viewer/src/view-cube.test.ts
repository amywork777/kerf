/**
 * view-cube.test.ts
 *
 * Unit tests for the pure helper `viewCubeFaceFromName`.  No DOM or WebGL
 * context required.
 */

import { describe, it, expect } from "vitest";
import * as THREE from "three";
import { viewCubeFaceFromName } from "./view-cube.js";

function expectUnitVec(v: THREE.Vector3 | null, x: number, y: number, z: number) {
  expect(v).not.toBeNull();
  expect(v!.x).toBeCloseTo(x);
  expect(v!.y).toBeCloseTo(y);
  expect(v!.z).toBeCloseTo(z);
  expect(v!.length()).toBeCloseTo(1);
}

describe("viewCubeFaceFromName", () => {
  it('returns (0, 0, 1) for "front"', () => {
    expectUnitVec(viewCubeFaceFromName("front"), 0, 0, 1);
  });

  it('returns (0, 0, -1) for "back"', () => {
    expectUnitVec(viewCubeFaceFromName("back"), 0, 0, -1);
  });

  it('returns (0, 1, 0) for "top"', () => {
    expectUnitVec(viewCubeFaceFromName("top"), 0, 1, 0);
  });

  it('returns (0, -1, 0) for "bottom"', () => {
    expectUnitVec(viewCubeFaceFromName("bottom"), 0, -1, 0);
  });

  it('returns (1, 0, 0) for "right"', () => {
    expectUnitVec(viewCubeFaceFromName("right"), 1, 0, 0);
  });

  it('returns (-1, 0, 0) for "left"', () => {
    expectUnitVec(viewCubeFaceFromName("left"), -1, 0, 0);
  });

  it("is case-insensitive (FRONT, Top, BACK)", () => {
    expectUnitVec(viewCubeFaceFromName("FRONT"), 0, 0, 1);
    expectUnitVec(viewCubeFaceFromName("Top"), 0, 1, 0);
    expectUnitVec(viewCubeFaceFromName("BACK"), 0, 0, -1);
  });

  it("returns null for unknown face names", () => {
    expect(viewCubeFaceFromName("diagonal")).toBeNull();
    expect(viewCubeFaceFromName("")).toBeNull();
    expect(viewCubeFaceFromName("iso")).toBeNull();
  });
});
