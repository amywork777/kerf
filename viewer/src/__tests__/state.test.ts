import { describe, it, expect } from "vitest";
import {
  newHistory,
  pushSnapshot,
  undo,
  redo,
  canUndo,
  canRedo,
  MAX_UNDO_DEPTH,
  type Snapshot,
} from "../state.js";

function snap(json: string, target = "out", params: Record<string, number> = {}): Snapshot {
  return { json, targetId: target, parameters: params };
}

describe("undo_redo_stack", () => {
  it("starts empty — nothing to undo or redo", () => {
    const h = newHistory();
    expect(canUndo(h)).toBe(false);
    expect(canRedo(h)).toBe(false);
    expect(undo(h, snap("a"))).toBeNull();
    expect(redo(h, snap("a"))).toBeNull();
  });

  it("undo restores the previous snapshot", () => {
    const h = newHistory();
    pushSnapshot(h, snap("v1"));
    pushSnapshot(h, snap("v2"));
    // current state is v3 — caller passes it in.
    const restored = undo(h, snap("v3"));
    expect(restored).not.toBeNull();
    expect(restored!.json).toBe("v2");
    expect(canRedo(h)).toBe(true);
  });

  it("redo replays an undone snapshot", () => {
    const h = newHistory();
    pushSnapshot(h, snap("v1"));
    const restored = undo(h, snap("v2"))!;
    expect(restored.json).toBe("v1");
    const replayed = redo(h, restored);
    expect(replayed).not.toBeNull();
    expect(replayed!.json).toBe("v2");
  });

  it("a fresh edit clears the redo stack", () => {
    const h = newHistory();
    pushSnapshot(h, snap("v1"));
    undo(h, snap("v2"));
    expect(canRedo(h)).toBe(true);
    pushSnapshot(h, snap("v3"));
    expect(canRedo(h)).toBe(false);
  });

  it("caps the undo stack at MAX_UNDO_DEPTH (50)", () => {
    const h = newHistory();
    for (let i = 0; i < MAX_UNDO_DEPTH + 10; i++) {
      pushSnapshot(h, snap(`v${i}`));
    }
    expect(h.undoStack.length).toBe(MAX_UNDO_DEPTH);
    // Oldest entries dropped — the first surviving entry is v10.
    expect(h.undoStack[0]!.json).toBe("v10");
    expect(h.undoStack[h.undoStack.length - 1]!.json).toBe(`v${MAX_UNDO_DEPTH + 10 - 1}`);
  });

  it("does not push duplicate consecutive snapshots", () => {
    const h = newHistory();
    pushSnapshot(h, snap("v1", "out", { a: 1 }));
    pushSnapshot(h, snap("v1", "out", { a: 1 }));
    pushSnapshot(h, snap("v1", "out", { a: 1 }));
    expect(h.undoStack.length).toBe(1);
  });

  it("snapshots are deep-copied (mutations to the source don't leak)", () => {
    const h = newHistory();
    const src = snap("v1", "out", { a: 1 });
    pushSnapshot(h, src);
    src.parameters.a = 999;
    expect(h.undoStack[0]!.parameters.a).toBe(1);
  });

  it("undo/redo preserves parameter overrides separately from the JSON", () => {
    const h = newHistory();
    pushSnapshot(h, snap("model", "out", { width: 10 }));
    const restored = undo(h, snap("model", "out", { width: 20 }))!;
    expect(restored.parameters.width).toBe(10);
    const re = redo(h, restored)!;
    expect(re.parameters.width).toBe(20);
  });
});
