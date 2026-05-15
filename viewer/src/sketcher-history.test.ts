// @vitest-environment node
import { describe, it, expect } from "vitest";
import { SketchHistory } from "./sketcher-history.js";
import type { Sketch } from "./sketcher.js";

function makeSketch(label: string): Sketch {
  return {
    plane: "Xy",
    primitives: [{ kind: "Point", id: label, x: 0, y: 0 }],
    constraints: [],
  };
}

describe("SketchHistory", () => {
  /**
   * Usage pattern: call push(current) BEFORE each mutation, then apply the
   * mutation. undo(current) returns the previously-pushed pre-mutation state.
   *
   * Scenario:
   *   s1 (initial) → push(s1) → mutate to s2 → push(s2) → mutate to s3
   *   Undo from s3 → returns s2; undo from s2 → returns s1
   *   Redo from s1 → returns s2
   */
  it("push 3 states, undo, redo", () => {
    const h = new SketchHistory();
    const s1 = makeSketch("s1");
    const s2 = makeSketch("s2");
    const s3 = makeSketch("s3");

    // Simulate: start at s1, push before mutating to s2, push before mutating to s3.
    h.push(s1); // pre-state before mutation to s2
    h.push(s2); // pre-state before mutation to s3
    // Current live state is s3.

    // First undo: returns s2 (the pre-mutation state before the last mutation)
    const prev1 = h.undo(s3);
    expect(prev1?.primitives[0].id).toBe("s2");

    // Second undo: returns s1
    const prev2 = h.undo(prev1!);
    expect(prev2?.primitives[0].id).toBe("s1");

    // Redo from s1: returns s2 (what was undone most recently)
    const fwd = h.redo(prev2!);
    expect(fwd?.primitives[0].id).toBe("s2");
  });

  it("undo at empty stack → no-op (returns null)", () => {
    const h = new SketchHistory();
    expect(h.canUndo).toBe(false);
    expect(h.undo(makeSketch("x"))).toBeNull();
  });

  it("redo after a new push → redo stack cleared", () => {
    const h = new SketchHistory();
    // Build some undo history.
    h.push(makeSketch("a"));
    h.push(makeSketch("b"));

    // Undo once to get something in the redo stack.
    h.undo(makeSketch("c"));
    expect(h.canRedo).toBe(true);

    // New push (new mutation) must clear the redo stack.
    h.push(makeSketch("d"));
    expect(h.canRedo).toBe(false);
  });

  it("limits undo stack to max (push 60 → only 50 kept)", () => {
    const h = new SketchHistory(50);
    for (let i = 0; i < 60; i++) {
      h.push(makeSketch(`s${i}`));
    }
    expect(h.undoDepth).toBe(50);
    expect(h.canUndo).toBe(true);
  });

  it("custom max respected", () => {
    const h = new SketchHistory(3);
    for (let i = 0; i < 10; i++) {
      h.push(makeSketch(`s${i}`));
    }
    expect(h.undoDepth).toBe(3);
  });

  it("canUndo / canRedo reflect stack emptiness", () => {
    const h = new SketchHistory();
    expect(h.canUndo).toBe(false);
    expect(h.canRedo).toBe(false);

    h.push(makeSketch("a"));
    expect(h.canUndo).toBe(true);
    expect(h.canRedo).toBe(false);

    h.undo(makeSketch("b"));
    expect(h.canUndo).toBe(false);
    expect(h.canRedo).toBe(true);
  });

  it("deep-clones states (mutations after push do not corrupt history)", () => {
    const h = new SketchHistory();
    const s = makeSketch("original");
    h.push(s);
    // Mutate the object AFTER pushing — history must be unaffected.
    (s.primitives[0] as { id: string }).id = "mutated";

    const restored = h.undo(makeSketch("current"));
    expect(restored?.primitives[0].id).toBe("original");
  });
});
