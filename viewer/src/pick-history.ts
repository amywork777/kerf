/**
 * pick-history.ts
 *
 * Tiny LRU-capped history of the last N user pick selections.
 * Each entry records which picking mode was active and the selected id.
 */

export type PickKind = "face" | "edge" | "vertex";

export interface PickEntry {
  kind: PickKind;
  id: number;
}

export class PickHistory {
  private history: PickEntry[] = [];
  private cursor = -1; // points at the "current" position in history

  constructor(private readonly maxSize: number = 10) {}

  /**
   * Push a new selection. Drops any forward history (like a browser's
   * back/forward stack), then appends. Trims to maxSize oldest entries.
   */
  push(entry: PickEntry): void {
    // Drop everything after the cursor (forward history).
    this.history = this.history.slice(0, this.cursor + 1);
    this.history.push(entry);
    // Trim oldest entries so we never exceed maxSize.
    if (this.history.length > this.maxSize) {
      this.history = this.history.slice(this.history.length - this.maxSize);
    }
    this.cursor = this.history.length - 1;
  }

  /**
   * Go back one step. Returns the previous entry, or null if already at
   * the beginning of history.
   */
  back(): PickEntry | null {
    if (this.cursor <= 0) return null;
    this.cursor -= 1;
    return this.history[this.cursor] ?? null;
  }

  /**
   * Go forward one step (after one or more back() calls). Returns the
   * entry, or null if already at the end.
   */
  forward(): PickEntry | null {
    if (this.cursor >= this.history.length - 1) return null;
    this.cursor += 1;
    return this.history[this.cursor] ?? null;
  }

  /** The entry at the current cursor position (what is "selected now"). */
  current(): PickEntry | null {
    return this.history[this.cursor] ?? null;
  }

  /** Total number of entries stored (including forward history). */
  get size(): number {
    return this.history.length;
  }

  /** True when back() would return an entry. */
  get canGoBack(): boolean {
    return this.cursor > 0;
  }

  /** True when forward() would return an entry. */
  get canGoForward(): boolean {
    return this.cursor < this.history.length - 1;
  }

  /** Reset to empty. */
  clear(): void {
    this.history = [];
    this.cursor = -1;
  }
}
