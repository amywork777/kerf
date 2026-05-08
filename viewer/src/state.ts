/**
 * Undo/redo state-stack utilities, kept as pure functions of plain
 * JSON-serializable state so we can unit-test them in vitest without
 * dragging the whole three.js scene into JSDOM.
 *
 * The viewer treats every model edit (parameter scrub, feature add,
 * feature delete, parameter reset, etc.) as a snapshot push: we keep
 * a stack of the *previous* model JSON strings and pop one off on
 * Ctrl+Z. Redo works the symmetric way. The stack is capped at
 * `MAX_UNDO_DEPTH` so a heavy scrub session doesn't run away with
 * memory.
 */

export const MAX_UNDO_DEPTH = 50;

/**
 * One snapshot of the editable parts of a model — JSON text plus the
 * scrubbed parameter overrides plus the current target id. Storing the
 * already-stringified JSON keeps snapshot diffing trivially comparable
 * (just `===` on the string).
 */
export interface Snapshot {
  json: string;
  targetId: string;
  parameters: Record<string, number>;
}

/**
 * Mutable history container. Use {@link pushSnapshot} to record a new
 * "previous" state before applying an edit, {@link undo} to step back,
 * {@link redo} to step forward.
 *
 * Both stacks are capped at MAX_UNDO_DEPTH; the oldest entry is dropped
 * when an overflow push happens.
 */
export interface History {
  undoStack: Snapshot[];
  redoStack: Snapshot[];
}

export function newHistory(): History {
  return { undoStack: [], redoStack: [] };
}

/**
 * Record a snapshot of the *current* state (the one that's about to be
 * mutated). Always clears the redo stack — once you make a fresh edit,
 * the alternate branch is gone, just like every other editor. Skips
 * pushing duplicates of the most recent entry so a noop edit doesn't
 * pollute the stack.
 */
export function pushSnapshot(h: History, snap: Snapshot): void {
  const top = h.undoStack[h.undoStack.length - 1];
  if (top && snapshotsEqual(top, snap)) return;
  h.undoStack.push(cloneSnapshot(snap));
  if (h.undoStack.length > MAX_UNDO_DEPTH) {
    h.undoStack.shift();
  }
  h.redoStack.length = 0;
}

/**
 * Pop one entry off the undo stack. Returns the snapshot that was the
 * *previous* state, or null if there's nothing to undo. The caller must
 * pass in the current state so it can be pushed onto the redo stack.
 */
export function undo(h: History, current: Snapshot): Snapshot | null {
  const prev = h.undoStack.pop();
  if (!prev) return null;
  h.redoStack.push(cloneSnapshot(current));
  if (h.redoStack.length > MAX_UNDO_DEPTH) {
    h.redoStack.shift();
  }
  return prev;
}

/**
 * Pop one entry off the redo stack. Symmetric to {@link undo}.
 */
export function redo(h: History, current: Snapshot): Snapshot | null {
  const next = h.redoStack.pop();
  if (!next) return null;
  h.undoStack.push(cloneSnapshot(current));
  if (h.undoStack.length > MAX_UNDO_DEPTH) {
    h.undoStack.shift();
  }
  return next;
}

export function canUndo(h: History): boolean {
  return h.undoStack.length > 0;
}

export function canRedo(h: History): boolean {
  return h.redoStack.length > 0;
}

function snapshotsEqual(a: Snapshot, b: Snapshot): boolean {
  if (a.json !== b.json) return false;
  if (a.targetId !== b.targetId) return false;
  const aKeys = Object.keys(a.parameters);
  const bKeys = Object.keys(b.parameters);
  if (aKeys.length !== bKeys.length) return false;
  for (const k of aKeys) {
    if (a.parameters[k] !== b.parameters[k]) return false;
  }
  return true;
}

function cloneSnapshot(s: Snapshot): Snapshot {
  return {
    json: s.json,
    targetId: s.targetId,
    parameters: { ...s.parameters },
  };
}
