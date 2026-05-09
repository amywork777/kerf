/**
 * Pure picking-state logic — no THREE, no DOM, testable in Node.
 *
 * Tracks:
 *  - selectedFaces: Set<number>  (multi-select via Shift+click in face mode)
 *  - hoveredFace:   number       (-1 = none)
 *  - highlightedVertex / highlightedEdge: number (single-select, unchanged)
 */

export type PickMode = "face" | "edge" | "vertex";

export interface PickingState {
  selectedFaces: Set<number>;
  hoveredFace: number;
  highlightedVertex: number;
  highlightedEdge: number;
}

/** Return a fresh zeroed state. */
export function createPickingState(): PickingState {
  return {
    selectedFaces: new Set(),
    hoveredFace: -1,
    highlightedVertex: -1,
    highlightedEdge: -1,
  };
}

/**
 * Handle a canvas click in face mode.
 *
 * - shift=false: replaces the selection with the clicked face (or clears if
 *   fid is -1).
 * - shift=true:  toggles fid in the selection set (ignored when fid === -1).
 *
 * Returns a *new* PickingState (immutable-style, easy to assert in tests).
 */
export function handleFaceClick(
  state: PickingState,
  fid: number,
  shift: boolean,
): PickingState {
  // Shift+click on a miss (fid === -1) is a no-op — keep selection intact.
  if (shift && fid < 0) {
    return {
      ...state,
      highlightedVertex: -1,
      highlightedEdge: -1,
    };
  }

  const next: PickingState = {
    ...state,
    selectedFaces: new Set(state.selectedFaces),
    highlightedVertex: -1,
    highlightedEdge: -1,
  };

  if (!shift) {
    // Plain click: replace selection.
    next.selectedFaces = fid >= 0 ? new Set([fid]) : new Set();
  } else {
    // Shift click: toggle membership.
    if (next.selectedFaces.has(fid)) {
      next.selectedFaces.delete(fid);
    } else {
      next.selectedFaces.add(fid);
    }
  }
  return next;
}

/**
 * Handle a canvas click in vertex mode (single-select, unchanged).
 */
export function handleVertexClick(
  state: PickingState,
  vid: number,
): PickingState {
  return {
    ...state,
    selectedFaces: new Set(),
    highlightedVertex: vid,
    highlightedEdge: -1,
  };
}

/**
 * Handle a canvas click in edge mode (single-select, unchanged).
 */
export function handleEdgeClick(
  state: PickingState,
  eid: number,
): PickingState {
  return {
    ...state,
    selectedFaces: new Set(),
    highlightedVertex: -1,
    highlightedEdge: eid,
  };
}

/**
 * Update the hover face. Call this on (debounced) pointermove.
 * Returns a new state only when the hovered face actually changes.
 */
export function handleHover(
  state: PickingState,
  fid: number,
): PickingState | null {
  if (fid === state.hoveredFace) return null; // no change
  return { ...state, hoveredFace: fid };
}

/**
 * Called on pointerleave / canvas blur — clears hover but keeps selection.
 */
export function clearHover(state: PickingState): PickingState {
  if (state.hoveredFace === -1) return state;
  return { ...state, hoveredFace: -1 };
}

/**
 * Called when switching pick modes or rebuilding the mesh.
 */
export function clearAll(state: PickingState): PickingState {
  return createPickingState();
}

// ---------------------------------------------------------------------------
// Helpers for callers
// ---------------------------------------------------------------------------

/** The "primary" face for backward-compat (first in insertion order, or -1). */
export function primaryFace(state: PickingState): number {
  const it = state.selectedFaces.values();
  const first = it.next();
  return first.done ? -1 : first.value;
}

/** Build the status-bar label for a face selection. */
export function faceSelectionLabel(state: PickingState, totalFaces: number): string {
  const n = state.selectedFaces.size;
  if (n === 0) return "";
  if (n === 1) return `selected face #${primaryFace(state)} (of ${totalFaces})`;
  return `selected ${n} faces (of ${totalFaces})`;
}

/** Which faces are currently highlighted (selected ∪ hovered). */
export function highlightedFaceIds(state: PickingState): { selected: Set<number>; hovered: number } {
  return { selected: state.selectedFaces, hovered: state.hoveredFace };
}
