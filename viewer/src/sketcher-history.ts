/**
 * Undo/redo history stack for the 2D sketcher.
 *
 * Stores deep-cloned JSON snapshots of `Sketch` states in two stacks (undo
 * and redo). The undo stack holds prior states; the redo stack holds states
 * that were undone and can be re-applied.
 *
 * Usage pattern
 * -------------
 *   // Before each mutation, snapshot the current state:
 *   history.push(toSketch());
 *   // … apply mutation …
 *
 *   // Undo: restore the most-recently pushed snapshot.
 *   const prev = history.undo();   // returns Sketch | null
 *   if (prev) applySketch(prev);
 *
 *   // Redo: re-apply the state that was most-recently undone.
 *   // Pass the current live state so it can be pushed onto the undo stack.
 *   const next = history.redo(toSketch());
 *   if (next) applySketch(next);
 *
 * Limit: the undo stack is capped at `max` entries (default 50). Oldest
 * entries are dropped when the limit is exceeded.
 *
 * No DOM dependencies — safe to import from Node (vitest node environment).
 */

import type { Sketch } from "./sketcher.js";

const DEFAULT_MAX = 50;

export class SketchHistory {
  private undoStack: string[] = [];
  private redoStack: string[] = [];
  private readonly max: number;

  constructor(max = DEFAULT_MAX) {
    this.max = max;
  }

  /**
   * Snapshot the current sketch state before a mutation.
   * Clears the redo stack (a new branch of history is being created).
   */
  push(sketch: Sketch): void {
    this.redoStack = [];
    this.undoStack.push(JSON.stringify(sketch));
    if (this.undoStack.length > this.max) {
      this.undoStack.shift(); // drop oldest
    }
  }

  /**
   * Undo: pop the most-recently pushed pre-mutation snapshot and return it.
   * The caller must save `current` to the redo stack; pass it here.
   * Returns null if the undo stack is empty.
   */
  undo(current: Sketch): Sketch | null {
    if (this.undoStack.length === 0) return null;
    this.redoStack.push(JSON.stringify(current));
    return JSON.parse(this.undoStack.pop()!) as Sketch;
  }

  /**
   * Redo: pop the most-recently undone snapshot, push `current` back onto
   * the undo stack, and return the snapshot to be applied.
   * Returns null if the redo stack is empty.
   */
  redo(current: Sketch): Sketch | null {
    if (this.redoStack.length === 0) return null;
    this.undoStack.push(JSON.stringify(current));
    if (this.undoStack.length > this.max) {
      this.undoStack.shift();
    }
    return JSON.parse(this.redoStack.pop()!) as Sketch;
  }

  get canUndo(): boolean {
    return this.undoStack.length > 0;
  }

  get canRedo(): boolean {
    return this.redoStack.length > 0;
  }

  /** Number of entries on the undo stack (exposed for tests). */
  get undoDepth(): number {
    return this.undoStack.length;
  }
}
