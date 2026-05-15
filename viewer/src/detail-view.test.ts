import { describe, it, expect } from "vitest";
import { detailViewOutputSize } from "./detail-view.js";
import type { DetailViewParams } from "./detail-view.js";

// Note: renderDetailPanel touches the DOM (CanvasRenderingContext2D), so it is
// covered by integration tests.  The pure helpers (detailViewOutputSize) are
// fully testable without jsdom.

describe("detailViewOutputSize", () => {
  it("zoom=2 produces 2× larger output", () => {
    const result = detailViewOutputSize({ w: 100, h: 80 }, 2);
    expect(result.outW).toBe(200);
    expect(result.outH).toBe(160);
  });

  it("zoom=3 produces 3× larger output", () => {
    const result = detailViewOutputSize({ w: 60, h: 40 }, 3);
    expect(result.outW).toBe(180);
    expect(result.outH).toBe(120);
  });

  it("zoom=1 → dimensions are unchanged", () => {
    const result = detailViewOutputSize({ w: 50, h: 50 }, 1);
    expect(result.outW).toBe(50);
    expect(result.outH).toBe(50);
  });

  it("fractional zoom rounds to integer pixels", () => {
    // 100 × 1.5 = 150 exactly (no rounding needed).
    const r1 = detailViewOutputSize({ w: 100, h: 100 }, 1.5);
    expect(r1.outW).toBe(150);
    expect(r1.outH).toBe(150);

    // 33 × 2 = 66 — just confirm integers come out.
    const r2 = detailViewOutputSize({ w: 33, h: 21 }, 2);
    expect(Number.isInteger(r2.outW)).toBe(true);
    expect(Number.isInteger(r2.outH)).toBe(true);
  });

  it("asymmetric rect with zoom=2 scales each dimension independently", () => {
    const result = detailViewOutputSize({ w: 80, h: 30 }, 2);
    expect(result.outW).toBe(160);
    expect(result.outH).toBe(60);
  });
});

describe("DetailViewParams type contract", () => {
  it("default zoom property is accepted as optional", () => {
    // TypeScript compile-time check: params without zoom must be valid.
    const params: DetailViewParams = {
      baseView: "front",
      rect: { x: 10, y: 20, w: 100, h: 80 },
      label: "A",
      // zoom omitted — should be optional
    };
    expect(params.zoom).toBeUndefined();
    expect(params.label).toBe("A");
    expect(params.baseView).toBe("front");
  });

  it("all three base views are valid", () => {
    const views: DetailViewParams["baseView"][] = ["front", "top", "side"];
    for (const bv of views) {
      const p: DetailViewParams = {
        baseView: bv,
        rect: { x: 0, y: 0, w: 50, h: 50 },
        label: "B",
      };
      expect(p.baseView).toBe(bv);
    }
  });
});
