/**
 * Tests for viewer/src/gdt.ts — GD&T overlay renderer.
 *
 * @vitest-environment jsdom
 */

import { describe, it, expect, beforeEach } from "vitest";
import { renderGdtOverlay } from "./gdt.js";
import type { GdtAnnotations } from "./gdt.js";

// ---------------------------------------------------------------------------
// Test helpers
// ---------------------------------------------------------------------------

const SVG_NS = "http://www.w3.org/2000/svg";

function makeSvg(): SVGElement {
  return document.createElementNS(SVG_NS, "svg") as unknown as SVGElement;
}

/** Identity projection: model XY → screen XY (ignores Z). */
const flatView = {
  project([x, y, _z]: [number, number, number]): [number, number] {
    return [x, y];
  },
};

const emptyAnnotations: GdtAnnotations = {
  datums: [],
  frames: [],
  finishes: [],
};

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

describe("renderGdtOverlay", () => {
  let svg: SVGElement;

  beforeEach(() => {
    svg = makeSvg();
  });

  it("produces no extra DOM for empty annotations", () => {
    renderGdtOverlay(svg, emptyAnnotations, flatView);
    // No <g> layer should be appended when there is nothing to draw.
    expect(svg.children.length).toBe(0);
  });

  describe("datum reference", () => {
    it("produces a datum-box rect, a letter text, a leader line, and a flag triangle", () => {
      const annotations: GdtAnnotations = {
        ...emptyAnnotations,
        datums: [{ letter: "A", anchor: [50, 50, 0] }],
      };
      renderGdtOverlay(svg, annotations, flatView);

      // One gdt-layer > one gdt-datum group.
      const layer = svg.querySelector(".gdt-layer")!;
      expect(layer).not.toBeNull();
      const datumGroup = layer.querySelector(".gdt-datum")!;
      expect(datumGroup).not.toBeNull();

      // Must have the datum box rect.
      const box = datumGroup.querySelector<SVGRectElement>('rect[data-gdt="datum-box"]');
      expect(box).not.toBeNull();

      // Must have a text element with the letter.
      const texts = datumGroup.querySelectorAll("text");
      const letterTexts = Array.from(texts).filter((t) => t.textContent === "A");
      expect(letterTexts.length).toBeGreaterThan(0);

      // Must have a leader line.
      const lines = datumGroup.querySelectorAll("line");
      expect(lines.length).toBeGreaterThan(0);

      // Must have a triangle flag (polygon).
      const polygons = datumGroup.querySelectorAll("polygon");
      expect(polygons.length).toBeGreaterThan(0);
    });
  });

  describe("feature control frame", () => {
    it("produces 2 cells (symbol + tolerance) for a frame with no datum refs", () => {
      const annotations: GdtAnnotations = {
        ...emptyAnnotations,
        frames: [
          {
            characteristic: "flatness",
            tolerance: 0.05,
            diametric: false,
            datum_refs: [],
            anchor: [100, 100, 0],
          },
        ],
      };
      renderGdtOverlay(svg, annotations, flatView);

      const frameGroup = svg.querySelector(".gdt-frame")!;
      expect(frameGroup).not.toBeNull();

      // symbol cell + tolerance cell = 2 named cells.
      const symbolCell = frameGroup.querySelector('rect[data-gdt="frame-cell-symbol"]');
      const toleranceCell = frameGroup.querySelector('rect[data-gdt="frame-cell-tolerance"]');
      expect(symbolCell).not.toBeNull();
      expect(toleranceCell).not.toBeNull();
    });

    it("produces 5 cells (symbol, tolerance, A, B, C) for a frame with 3 datum refs", () => {
      const annotations: GdtAnnotations = {
        ...emptyAnnotations,
        frames: [
          {
            characteristic: "position",
            tolerance: 0.1,
            diametric: true,
            datum_refs: [
              { letter: "A" },
              { letter: "B" },
              { letter: "C" },
            ],
            anchor: [80, 80, 0],
          },
        ],
      };
      renderGdtOverlay(svg, annotations, flatView);

      const frameGroup = svg.querySelector(".gdt-frame")!;
      expect(frameGroup).not.toBeNull();

      // 1 symbol + 1 tolerance + 3 datum-ref = 5 named data-gdt cells total.
      // The outer border rect has no data-gdt attribute, so count named cells.
      const namedCells = frameGroup.querySelectorAll("rect[data-gdt^='frame-cell']");
      expect(namedCells.length).toBe(5);

      // All three datum letters appear as text nodes.
      const allText = Array.from(frameGroup.querySelectorAll("text")).map(
        (t) => t.textContent ?? "",
      );
      expect(allText.some((t) => t.includes("A"))).toBe(true);
      expect(allText.some((t) => t.includes("B"))).toBe(true);
      expect(allText.some((t) => t.includes("C"))).toBe(true);
    });

    it("includes ⌀ prefix in tolerance cell for diametric frames", () => {
      const annotations: GdtAnnotations = {
        ...emptyAnnotations,
        frames: [
          {
            characteristic: "circularity",
            tolerance: 0.02,
            diametric: true,
            datum_refs: [],
            anchor: [0, 0, 0],
          },
        ],
      };
      renderGdtOverlay(svg, annotations, flatView);

      const frameGroup = svg.querySelector(".gdt-frame")!;
      const allText = Array.from(frameGroup.querySelectorAll("text")).map(
        (t) => t.textContent ?? "",
      );
      expect(allText.some((t) => t.startsWith("⌀"))).toBe(true);
    });
  });

  describe("surface finish", () => {
    it("produces a checkmark path and Ra label", () => {
      const annotations: GdtAnnotations = {
        ...emptyAnnotations,
        finishes: [{ ra: 1.6, anchor: [60, 60, 0] }],
      };
      renderGdtOverlay(svg, annotations, flatView);

      const finishGroup = svg.querySelector(".gdt-finish")!;
      expect(finishGroup).not.toBeNull();

      // Checkmark path.
      const path = finishGroup.querySelector('path[data-gdt="finish-check"]');
      expect(path).not.toBeNull();

      // Ra label.
      const raEl = finishGroup.querySelector('text[data-gdt="finish-ra"]');
      expect(raEl).not.toBeNull();
      expect(raEl!.textContent).toBe("1.6");
    });

    it("adds a process indicator inside the V for non-default processes", () => {
      const annotations: GdtAnnotations = {
        ...emptyAnnotations,
        finishes: [{ ra: 0.8, process: "ground", anchor: [0, 0, 0] }],
      };
      renderGdtOverlay(svg, annotations, flatView);

      const finishGroup = svg.querySelector(".gdt-finish")!;
      const procEl = finishGroup.querySelector('text[data-gdt="finish-process"]');
      expect(procEl).not.toBeNull();
      expect(procEl!.textContent).toBe("G");
    });

    it("does not add process indicator for process='any'", () => {
      const annotations: GdtAnnotations = {
        ...emptyAnnotations,
        finishes: [{ ra: 3.2, process: "any", anchor: [0, 0, 0] }],
      };
      renderGdtOverlay(svg, annotations, flatView);

      const finishGroup = svg.querySelector(".gdt-finish")!;
      const procEl = finishGroup.querySelector('text[data-gdt="finish-process"]');
      expect(procEl).toBeNull();
    });
  });

  describe("multiple annotations per view", () => {
    it("all annotation types appear in the SVG when combined", () => {
      const annotations: GdtAnnotations = {
        datums: [
          { letter: "A", anchor: [10, 10, 0] },
          { letter: "B", anchor: [20, 20, 0] },
        ],
        frames: [
          {
            characteristic: "perpendicularity",
            tolerance: 0.03,
            diametric: false,
            datum_refs: [{ letter: "A" }],
            anchor: [40, 40, 0],
          },
        ],
        finishes: [
          { ra: 1.6, process: "machined", anchor: [60, 60, 0] },
        ],
      };
      renderGdtOverlay(svg, annotations, flatView);

      const datumGroups = svg.querySelectorAll(".gdt-datum");
      expect(datumGroups.length).toBe(2);

      const frameGroups = svg.querySelectorAll(".gdt-frame");
      expect(frameGroups.length).toBe(1);

      const finishGroups = svg.querySelectorAll(".gdt-finish");
      expect(finishGroups.length).toBe(1);
    });
  });
});
