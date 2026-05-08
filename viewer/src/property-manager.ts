/**
 * SolidWorks-style PropertyManager panel.
 *
 * Replaces the flat range-slider list with typed controls:
 *   - Number stepper (numeric input + ▲▼ buttons + range slider)
 *   - Expression editor (ƒx toggle, text input, eval preview)
 *   - Reset-to-default button (↺)
 *
 * Exports a single `mountPropertyManager` function that returns a `{ refresh }` handle.
 */

export interface PropertyManagerOpts {
  /** Returns the current live parameter map. */
  getParams(): Record<string, number>;
  /** Returns the model's default parameter map. */
  getDefaults(): Record<string, number>;
  /** Called when the user changes a parameter value. */
  setParam(name: string, value: number): void;
  /** Called after every setParam (triggers a rebuild in the host). */
  onChange(): void;
}

export interface PropertyManagerHandle {
  /** Re-render all rows from the current getParams() snapshot. */
  refresh(): void;
}

// ---------------------------------------------------------------------------
// Tiny expression evaluator
// Supports: +, -, *, /, unary -, parens, numeric literals, $param references.
// Returns the numeric result or throws with a human-readable error message.
// ---------------------------------------------------------------------------

/**
 * Evaluate a simple arithmetic expression string.
 * @param expr  The expression, e.g. "$plate_x / 2 + 3"
 * @param params  Current parameter values for $name lookups.
 */
export function evalExpression(
  expr: string,
  params: Record<string, number>,
): number {
  // Replace $param references with their numeric values.
  const substituted = expr.replace(/\$([a-zA-Z_][a-zA-Z0-9_]*)/g, (_match, name: string) => {
    if (!(name in params)) {
      throw new Error(`Unknown parameter: $${name}`);
    }
    return String(params[name]);
  });

  // Safety: only allow digits, operators, parens, dots, spaces, and 'e'/'E' for sci notation.
  if (!/^[\d\s+\-*/().eE]+$/.test(substituted)) {
    throw new Error(`Unsupported characters in expression`);
  }

  // Use Function constructor for safe evaluation (no identifiers left after substitution).
  try {
    // eslint-disable-next-line no-new-func
    const result = new Function(`"use strict"; return (${substituted});`)() as unknown;
    if (typeof result !== "number" || !isFinite(result)) {
      throw new Error(`Expression does not evaluate to a finite number`);
    }
    return result;
  } catch (e) {
    throw new Error(`Invalid expression: ${(e as Error).message}`);
  }
}

// ---------------------------------------------------------------------------
// Smart step size
// ---------------------------------------------------------------------------

function smartStep(value: number): number {
  return Math.max(Math.abs(value) / 100, 0.01);
}

// ---------------------------------------------------------------------------
// Row builder
// ---------------------------------------------------------------------------

interface RowState {
  name: string;
  exprMode: boolean;
}

function buildRow(
  name: string,
  cur: number,
  def: number,
  opts: PropertyManagerOpts,
  allParams: () => Record<string, number>,
): HTMLElement {
  const state: RowState = { name, exprMode: false };

  const row = document.createElement("div");
  row.className = "pm-row";

  // ── header: label + fx button + reset button ──────────────────────────────
  const header = document.createElement("div");
  header.className = "pm-row-header";

  const label = document.createElement("span");
  label.className = "pm-label";
  label.textContent = name;

  const fxBtn = document.createElement("button");
  fxBtn.className = "pm-fx-btn";
  fxBtn.title = "Toggle expression editor";
  fxBtn.textContent = "ƒx";
  fxBtn.type = "button";

  const resetBtn = document.createElement("button");
  resetBtn.className = "pm-reset-btn";
  resetBtn.title = "Reset to default";
  resetBtn.textContent = "↺";
  resetBtn.type = "button";

  header.appendChild(label);
  header.appendChild(fxBtn);
  header.appendChild(resetBtn);

  // ── numeric stepper ───────────────────────────────────────────────────────
  const stepperWrap = document.createElement("div");
  stepperWrap.className = "pm-stepper";

  const numInput = document.createElement("input");
  numInput.type = "number";
  numInput.className = "pm-number-input";
  numInput.value = String(cur);
  numInput.step = String(smartStep(cur));

  const upBtn = document.createElement("button");
  upBtn.className = "pm-step-up";
  upBtn.title = "Increment";
  upBtn.textContent = "▲";
  upBtn.type = "button";

  const downBtn = document.createElement("button");
  downBtn.className = "pm-step-down";
  downBtn.title = "Decrement";
  downBtn.textContent = "▼";
  downBtn.type = "button";

  stepperWrap.appendChild(numInput);
  stepperWrap.appendChild(upBtn);
  stepperWrap.appendChild(downBtn);

  // ── range slider ──────────────────────────────────────────────────────────
  const span = Math.max(Math.abs(def) * 2, 10);
  const sliderMin = def === 0 ? -span : Math.min(def - span, def * -0.2);
  const sliderMax = def === 0 ? span : Math.max(def + span, def * 2);
  const sliderStep = Math.max((sliderMax - sliderMin) / 200, 0.001);

  const slider = document.createElement("input");
  slider.type = "range";
  slider.className = "pm-slider";
  slider.min = String(sliderMin);
  slider.max = String(sliderMax);
  slider.step = String(sliderStep);
  slider.value = String(cur);

  // ── assemble row ──────────────────────────────────────────────────────────
  row.appendChild(header);
  row.appendChild(stepperWrap);
  row.appendChild(slider);
  // exprWrap is created on-demand and removed on toggle-off

  // ── helpers ───────────────────────────────────────────────────────────────
  function applyValue(v: number) {
    opts.setParam(name, v);
    opts.onChange();
    numInput.value = String(v);
    slider.value = String(v);
    // Update step based on new magnitude.
    numInput.step = String(smartStep(v));
  }

  // ── event handlers ────────────────────────────────────────────────────────
  numInput.addEventListener("change", () => {
    const v = Number(numInput.value);
    if (isFinite(v)) applyValue(v);
  });

  upBtn.addEventListener("click", () => {
    const v = Number(numInput.value);
    applyValue(v + smartStep(v));
  });

  downBtn.addEventListener("click", () => {
    const v = Number(numInput.value);
    applyValue(v - smartStep(v));
  });

  slider.addEventListener("input", () => {
    applyValue(Number(slider.value));
  });

  fxBtn.addEventListener("click", () => {
    state.exprMode = !state.exprMode;
    if (state.exprMode) {
      // Build and attach the expression editor.
      const exprWrap = document.createElement("div");
      exprWrap.className = "pm-expr-wrap";

      const exprInput = document.createElement("input");
      exprInput.type = "text";
      exprInput.className = "pm-expr-input";
      exprInput.placeholder = "e.g. $other * 2";

      const exprPreview = document.createElement("div");
      exprPreview.className = "pm-expr-preview";

      exprWrap.appendChild(exprInput);
      exprWrap.appendChild(exprPreview);
      row.appendChild(exprWrap);

      stepperWrap.style.display = "none";
      fxBtn.classList.add("pm-fx-active");

      exprInput.addEventListener("input", () => {
        const raw = exprInput.value.trim();
        if (!raw) {
          exprPreview.textContent = "";
          exprPreview.classList.remove("pm-expr-error");
          return;
        }
        try {
          const result = evalExpression(raw, allParams());
          exprPreview.textContent = `= ${result}`;
          exprPreview.classList.remove("pm-expr-error");
          opts.setParam(name, result);
          opts.onChange();
        } catch (e) {
          exprPreview.textContent = (e as Error).message;
          exprPreview.classList.add("pm-expr-error");
        }
      });
    } else {
      // Remove the expression editor.
      const exprWrap = row.querySelector(".pm-expr-wrap");
      if (exprWrap) row.removeChild(exprWrap);
      stepperWrap.style.display = "";
      fxBtn.classList.remove("pm-fx-active");
    }
  });

  resetBtn.addEventListener("click", () => {
    applyValue(def);
    if (state.exprMode) {
      const exprInputEl = row.querySelector<HTMLInputElement>(".pm-expr-input");
      const exprPreviewEl = row.querySelector<HTMLElement>(".pm-expr-preview");
      if (exprInputEl) exprInputEl.value = "";
      if (exprPreviewEl) {
        exprPreviewEl.textContent = "";
        exprPreviewEl.classList.remove("pm-expr-error");
      }
    }
  });

  return row;
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

export function mountPropertyManager(
  host: HTMLElement,
  opts: PropertyManagerOpts,
): PropertyManagerHandle {
  function render() {
    host.innerHTML = "";
    const params = opts.getParams();
    const defaults = opts.getDefaults();
    // Use union of keys from both maps, sorted.
    const keys = Array.from(
      new Set([...Object.keys(params), ...Object.keys(defaults)]),
    ).sort();

    for (const k of keys) {
      const def = defaults[k] ?? params[k] ?? 0;
      const cur = params[k] ?? def;
      const row = buildRow(k, cur, def, opts, opts.getParams);
      host.appendChild(row);
    }
  }

  render();

  return {
    refresh() {
      render();
    },
  };
}
