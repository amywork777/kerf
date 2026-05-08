/**
 * Error overlay banner: a red-tinted strip across the top of the 3D
 * stage that surfaces evaluation errors more visibly than the muted
 * status bar in the side panel. Click to dismiss.
 *
 * Pure DOM, no three.js coupling — instantiate with the `<div id="stage">`
 * element and call show / hide. Kept testable in JSDOM via vitest.
 */

export interface ErrorOverlay {
  /** Show the banner with `message`. Optionally pin `featureId` so the
   * user can see which feature was the culprit. Idempotent — calling
   * `show` twice just updates the text. */
  show(message: string, featureId?: string | null): void;
  /** Hide the banner. */
  hide(): void;
  /** True if the banner is currently visible. */
  isVisible(): boolean;
  /** Underlying DOM node — exposed for tests. */
  readonly element: HTMLElement;
}

/**
 * Create the overlay node and attach it to `parent` (typically the
 * 3D stage container). The node is absolutely positioned and starts
 * hidden. Call the returned `show` / `hide` to toggle.
 */
export function mountErrorOverlay(parent: HTMLElement): ErrorOverlay {
  const el = document.createElement("div");
  el.className = "error-overlay";
  el.setAttribute("role", "alert");
  el.hidden = true;
  el.innerHTML = `
    <div class="eo-icon" aria-hidden="true">!</div>
    <div class="eo-body">
      <div class="eo-title">evaluation failed</div>
      <div class="eo-msg"></div>
      <div class="eo-feature" hidden></div>
    </div>
    <div class="eo-dismiss" title="dismiss">close</div>
  `;
  // Click anywhere on the banner dismisses (matches "click banner to
  // dismiss" in the spec).
  el.addEventListener("click", () => {
    el.hidden = true;
  });
  parent.appendChild(el);

  const msgEl = el.querySelector(".eo-msg") as HTMLElement;
  const featEl = el.querySelector(".eo-feature") as HTMLElement;

  return {
    element: el,
    show(message: string, featureId?: string | null) {
      msgEl.textContent = message;
      if (featureId) {
        featEl.hidden = false;
        featEl.textContent = `feature: ${featureId}`;
      } else {
        featEl.hidden = true;
        featEl.textContent = "";
      }
      el.hidden = false;
    },
    hide() {
      el.hidden = true;
    },
    isVisible() {
      return !el.hidden;
    },
  };
}

/**
 * Try to extract a feature id from an evaluator error message. The
 * kerf-cad evaluator wraps errors as `"evaluate '<id>': <inner>"`, so
 * we match on that prefix. Returns null on no match — the caller can
 * fall back to omitting the feature pin.
 */
export function extractFeatureIdFromError(msg: string): string | null {
  // Matches: evaluate 'foo': bar  OR  evaluate "foo": bar
  const m = msg.match(/evaluate\s+['"]([^'"]+)['"]/);
  return m ? (m[1] ?? null) : null;
}
