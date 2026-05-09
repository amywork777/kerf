import { describe, it, expect } from "vitest";
import {
  createPickingState,
  handleFaceClick,
  handleVertexClick,
  handleEdgeClick,
  handleHover,
  clearHover,
  clearAll,
  primaryFace,
  faceSelectionLabel,
  highlightedFaceIds,
} from "./picking-state";

// ---------------------------------------------------------------------------
// Hover state
// ---------------------------------------------------------------------------
describe("hover state", () => {
  it("is initially -1 (no hover)", () => {
    const s = createPickingState();
    expect(s.hoveredFace).toBe(-1);
  });

  it("handleHover updates hoveredFace", () => {
    const s = createPickingState();
    const next = handleHover(s, 5);
    expect(next).not.toBeNull();
    expect(next!.hoveredFace).toBe(5);
  });

  it("handleHover returns null when face hasn't changed (no-op)", () => {
    const s = { ...createPickingState(), hoveredFace: 5 };
    expect(handleHover(s, 5)).toBeNull();
  });

  it("hover is distinct from selection — hovering a face doesn't select it", () => {
    const s = createPickingState();
    const hovered = handleHover(s, 3)!;
    expect(hovered.hoveredFace).toBe(3);
    expect(hovered.selectedFaces.size).toBe(0);
  });

  it("selected face can be hovered simultaneously (they are independent)", () => {
    let s = createPickingState();
    s = handleFaceClick(s, 3, false);
    const hovered = handleHover(s, 3)!;
    expect(hovered.selectedFaces.has(3)).toBe(true);
    expect(hovered.hoveredFace).toBe(3);
  });

  it("clearHover resets hoveredFace to -1 but keeps selection", () => {
    let s = createPickingState();
    s = handleFaceClick(s, 7, false);
    s = handleHover(s, 2)!;
    const cleared = clearHover(s);
    expect(cleared.hoveredFace).toBe(-1);
    expect(cleared.selectedFaces.has(7)).toBe(true);
  });

  it("clearHover is a no-op and returns same object if already -1", () => {
    const s = createPickingState();
    expect(clearHover(s)).toBe(s);
  });

  it("mouseleave (clearHover) keeps selection intact", () => {
    let s = createPickingState();
    s = handleFaceClick(s, 1, false);
    s = handleFaceClick(s, 2, true);
    s = { ...s, hoveredFace: 4 };
    const after = clearHover(s);
    expect(after.hoveredFace).toBe(-1);
    expect(after.selectedFaces.size).toBe(2);
    expect(after.selectedFaces.has(1)).toBe(true);
    expect(after.selectedFaces.has(2)).toBe(true);
  });
});

// ---------------------------------------------------------------------------
// Face click — plain (no Shift)
// ---------------------------------------------------------------------------
describe("plain click (no Shift) replaces selection", () => {
  it("clicking a face selects only that face", () => {
    const s = createPickingState();
    const next = handleFaceClick(s, 4, false);
    expect(next.selectedFaces).toEqual(new Set([4]));
  });

  it("clicking a different face replaces the previous selection", () => {
    let s = createPickingState();
    s = handleFaceClick(s, 1, false);
    s = handleFaceClick(s, 2, false);
    expect(s.selectedFaces).toEqual(new Set([2]));
  });

  it("clicking fid=-1 (miss) clears selection", () => {
    let s = createPickingState();
    s = handleFaceClick(s, 5, false);
    s = handleFaceClick(s, -1, false);
    expect(s.selectedFaces.size).toBe(0);
  });

  it("plain click replaces a multi-face selection", () => {
    let s = createPickingState();
    s = handleFaceClick(s, 1, false);
    s = handleFaceClick(s, 2, true);
    s = handleFaceClick(s, 3, true);
    expect(s.selectedFaces.size).toBe(3);
    // Now plain click on 9 — wipes all three.
    s = handleFaceClick(s, 9, false);
    expect(s.selectedFaces).toEqual(new Set([9]));
  });
});

// ---------------------------------------------------------------------------
// Shift+click multi-select
// ---------------------------------------------------------------------------
describe("Shift+click toggles set membership", () => {
  it("Shift+click adds a new face to the selection", () => {
    let s = handleFaceClick(createPickingState(), 1, false);
    s = handleFaceClick(s, 2, true);
    expect(s.selectedFaces).toEqual(new Set([1, 2]));
  });

  it("Shift+click removes an already-selected face (toggle)", () => {
    let s = handleFaceClick(createPickingState(), 1, false);
    s = handleFaceClick(s, 2, true);
    s = handleFaceClick(s, 1, true); // de-select 1
    expect(s.selectedFaces).toEqual(new Set([2]));
  });

  it("Shift+click on fid=-1 is ignored", () => {
    let s = handleFaceClick(createPickingState(), 5, false);
    const before = s.selectedFaces;
    s = handleFaceClick(s, -1, true);
    expect(s.selectedFaces).toEqual(before);
  });

  it("builds up a set across multiple Shift clicks", () => {
    let s = createPickingState();
    for (const fid of [10, 20, 30, 40]) {
      s = handleFaceClick(s, fid, true);
    }
    expect(s.selectedFaces).toEqual(new Set([10, 20, 30, 40]));
  });

  it("clicking the same face twice with Shift leaves it de-selected", () => {
    let s = createPickingState();
    s = handleFaceClick(s, 5, true);
    s = handleFaceClick(s, 5, true);
    expect(s.selectedFaces.size).toBe(0);
  });
});

// ---------------------------------------------------------------------------
// Vertex / edge clicks clear face selection
// ---------------------------------------------------------------------------
describe("vertex and edge clicks", () => {
  it("vertex click clears face selection and sets vertex", () => {
    let s = handleFaceClick(createPickingState(), 3, false);
    s = handleVertexClick(s, 7);
    expect(s.selectedFaces.size).toBe(0);
    expect(s.highlightedVertex).toBe(7);
    expect(s.highlightedEdge).toBe(-1);
  });

  it("edge click clears face selection and sets edge", () => {
    let s = handleFaceClick(createPickingState(), 3, false);
    s = handleEdgeClick(s, 11);
    expect(s.selectedFaces.size).toBe(0);
    expect(s.highlightedEdge).toBe(11);
    expect(s.highlightedVertex).toBe(-1);
  });
});

// ---------------------------------------------------------------------------
// clearAll
// ---------------------------------------------------------------------------
describe("clearAll", () => {
  it("resets everything to the initial state", () => {
    let s = handleFaceClick(createPickingState(), 3, false);
    s = handleFaceClick(s, 7, true);
    s = { ...s, hoveredFace: 2, highlightedVertex: 1, highlightedEdge: 5 };
    const cleared = clearAll(s);
    expect(cleared.selectedFaces.size).toBe(0);
    expect(cleared.hoveredFace).toBe(-1);
    expect(cleared.highlightedVertex).toBe(-1);
    expect(cleared.highlightedEdge).toBe(-1);
  });
});

// ---------------------------------------------------------------------------
// Status label helper
// ---------------------------------------------------------------------------
describe("faceSelectionLabel", () => {
  it("returns empty string when nothing is selected", () => {
    expect(faceSelectionLabel(createPickingState(), 10)).toBe("");
  });

  it("returns singular form for one selected face", () => {
    const s = handleFaceClick(createPickingState(), 4, false);
    expect(faceSelectionLabel(s, 12)).toBe("selected face #4 (of 12)");
  });

  it("returns count form for multiple selected faces", () => {
    let s = handleFaceClick(createPickingState(), 1, false);
    s = handleFaceClick(s, 2, true);
    s = handleFaceClick(s, 3, true);
    expect(faceSelectionLabel(s, 10)).toBe("selected 3 faces (of 10)");
  });
});

// ---------------------------------------------------------------------------
// primaryFace helper
// ---------------------------------------------------------------------------
describe("primaryFace", () => {
  it("returns -1 when no faces are selected", () => {
    expect(primaryFace(createPickingState())).toBe(-1);
  });

  it("returns the single selected face", () => {
    const s = handleFaceClick(createPickingState(), 8, false);
    expect(primaryFace(s)).toBe(8);
  });
});

// ---------------------------------------------------------------------------
// highlightedFaceIds
// ---------------------------------------------------------------------------
describe("highlightedFaceIds", () => {
  it("returns selected set and hovered face", () => {
    let s = handleFaceClick(createPickingState(), 1, false);
    s = handleFaceClick(s, 2, true);
    s = handleHover(s, 5)!;
    const { selected, hovered } = highlightedFaceIds(s);
    expect(selected).toEqual(new Set([1, 2]));
    expect(hovered).toBe(5);
  });
});
