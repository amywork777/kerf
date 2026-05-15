/**
 * Inline dimension-entry widget for the 2D sketcher.
 *
 * Shows a small `<input>` element near the constraint anchor so the user can
 * type a numeric value without leaving the canvas. Resolves when the user
 * presses Enter (returns the number) or Escape (returns null).
 *
 * DOM-based — tests must run under jsdom.
 */

export interface AnchorPt {
  /** Canvas-space x coordinate in pixels (relative to the viewport). */
  x: number;
  /** Canvas-space y coordinate in pixels (relative to the viewport). */
  y: number;
}

/**
 * Show a small floating input near `anchor` pre-filled with `defaultValue`.
 *
 * Returns the parsed float the user confirmed, or null if they pressed Escape
 * (or the input was otherwise dismissed without a valid number).
 */
export function promptDistance(
  anchor: AnchorPt,
  defaultValue: number,
): Promise<number | null> {
  return new Promise((resolve) => {
    const input = document.createElement("input");
    input.type = "number";
    input.step = "any";
    input.value = defaultValue.toFixed(4);
    input.className = "sk-dim-entry";

    Object.assign(input.style, {
      position: "fixed",
      left: `${anchor.x + 12}px`,
      top: `${anchor.y - 10}px`,
      width: "80px",
      padding: "2px 4px",
      fontSize: "12px",
      fontFamily: "ui-monospace, Menlo, monospace",
      background: "#1e2530",
      color: "#e6edf3",
      border: "1px solid #58a6ff",
      borderRadius: "3px",
      zIndex: "2000",
      outline: "none",
    } as CSSStyleDeclaration);

    const cleanup = () => {
      input.removeEventListener("keydown", onKey);
      input.removeEventListener("blur", onBlur);
      if (input.parentNode) input.parentNode.removeChild(input);
    };

    const confirm = () => {
      const v = parseFloat(input.value);
      cleanup();
      resolve(Number.isFinite(v) ? v : null);
    };

    const cancel = () => {
      cleanup();
      resolve(null);
    };

    const onKey = (e: KeyboardEvent) => {
      if (e.key === "Enter") {
        e.preventDefault();
        confirm();
      } else if (e.key === "Escape") {
        e.preventDefault();
        cancel();
      }
    };

    // Blur without Enter → cancel (same as Escape).
    const onBlur = () => cancel();

    input.addEventListener("keydown", onKey);
    input.addEventListener("blur", onBlur);

    document.body.appendChild(input);
    // Select all so user can immediately type a replacement value.
    input.focus();
    input.select();
  });
}
