/**
 * Tests for the SolidWorks-style PropertyManager panel.
 *
 * @vitest-environment happy-dom
 */
import { describe, it, expect, vi } from "vitest";
import { mountPropertyManager, type PropertyManagerOpts } from "./property-manager.js";

function makeHost(): HTMLElement {
  const el = document.createElement("div");
  document.body.appendChild(el);
  return el;
}

function makeOpts(
  params: Record<string, number> = { plate_x: 100, hole_radius: 5 },
  defaults?: Record<string, number>,
): { opts: PropertyManagerOpts; setParam: ReturnType<typeof vi.fn>; onChange: ReturnType<typeof vi.fn> } {
  const setParam = vi.fn();
  const onChange = vi.fn();
  const resolvedDefaults = defaults ?? { ...params };
  const opts: PropertyManagerOpts = {
    getParams: () => ({ ...params }),
    getDefaults: () => ({ ...resolvedDefaults }),
    setParam,
    onChange,
  };
  return { opts, setParam, onChange };
}

describe("PropertyManager — mounting", () => {
  it("renders one row per parameter", () => {
    const host = makeHost();
    const { opts } = makeOpts({ plate_x: 100, hole_radius: 5 });
    mountPropertyManager(host, opts);
    const rows = host.querySelectorAll(".pm-row");
    expect(rows).toHaveLength(2);
  });

  it("renders the parameter name as a label in each row", () => {
    const host = makeHost();
    const { opts } = makeOpts({ plate_x: 100 });
    mountPropertyManager(host, opts);
    expect(host.textContent).toContain("plate_x");
  });

  it("renders a numeric input with the current value", () => {
    const host = makeHost();
    const { opts } = makeOpts({ plate_x: 42 });
    mountPropertyManager(host, opts);
    const numInput = host.querySelector<HTMLInputElement>(".pm-number-input");
    expect(numInput).not.toBeNull();
    expect(Number(numInput!.value)).toBe(42);
  });

  it("renders a range slider beneath the number input", () => {
    const host = makeHost();
    const { opts } = makeOpts({ plate_x: 100 });
    mountPropertyManager(host, opts);
    const slider = host.querySelector<HTMLInputElement>('input[type="range"]');
    expect(slider).not.toBeNull();
  });

  it("renders a reset button per row", () => {
    const host = makeHost();
    const { opts } = makeOpts({ plate_x: 100, hole_radius: 5 });
    mountPropertyManager(host, opts);
    const resets = host.querySelectorAll(".pm-reset-btn");
    expect(resets).toHaveLength(2);
  });

  it("renders an fx toggle button per row", () => {
    const host = makeHost();
    const { opts } = makeOpts({ plate_x: 100, hole_radius: 5 });
    mountPropertyManager(host, opts);
    const fxBtns = host.querySelectorAll(".pm-fx-btn");
    expect(fxBtns).toHaveLength(2);
  });
});

describe("PropertyManager — number stepper", () => {
  it("typing a number calls setParam with parsed value", () => {
    const host = makeHost();
    const { opts, setParam } = makeOpts({ plate_x: 100 });
    mountPropertyManager(host, opts);
    const numInput = host.querySelector<HTMLInputElement>(".pm-number-input")!;
    numInput.value = "55";
    numInput.dispatchEvent(new Event("change"));
    expect(setParam).toHaveBeenCalledWith("plate_x", 55);
  });

  it("up stepper increments by smart step size", () => {
    const host = makeHost();
    const { opts, setParam } = makeOpts({ plate_x: 100 });
    mountPropertyManager(host, opts);
    const upBtn = host.querySelector<HTMLButtonElement>(".pm-step-up")!;
    upBtn.click();
    // smart step for 100 = max(100/100, 0.01) = 1
    expect(setParam).toHaveBeenCalledWith("plate_x", 101);
  });

  it("down stepper decrements by smart step size", () => {
    const host = makeHost();
    const { opts, setParam } = makeOpts({ plate_x: 100 });
    mountPropertyManager(host, opts);
    const downBtn = host.querySelector<HTMLButtonElement>(".pm-step-down")!;
    downBtn.click();
    // smart step for 100 = 1
    expect(setParam).toHaveBeenCalledWith("plate_x", 99);
  });

  it("smart step for small values uses 0.01 minimum", () => {
    const host = makeHost();
    const { opts, setParam } = makeOpts({ tiny: 0.5 });
    mountPropertyManager(host, opts);
    const upBtn = host.querySelector<HTMLButtonElement>(".pm-step-up")!;
    upBtn.click();
    // smart step for 0.5 = max(0.5/100, 0.01) = max(0.005, 0.01) = 0.01
    expect(setParam).toHaveBeenCalledWith("tiny", 0.51);
  });

  it("slider change calls setParam", () => {
    const host = makeHost();
    const { opts, setParam } = makeOpts({ plate_x: 100 });
    mountPropertyManager(host, opts);
    const slider = host.querySelector<HTMLInputElement>('input[type="range"]')!;
    slider.value = "80";
    slider.dispatchEvent(new Event("input"));
    expect(setParam).toHaveBeenCalledWith("plate_x", 80);
  });
});

describe("PropertyManager — expression mode", () => {
  it("clicking fx toggle switches to expression mode (text input visible)", () => {
    const host = makeHost();
    const { opts } = makeOpts({ plate_x: 100 });
    mountPropertyManager(host, opts);
    const fxBtn = host.querySelector<HTMLButtonElement>(".pm-fx-btn")!;
    fxBtn.click();
    const exprInput = host.querySelector<HTMLInputElement>(".pm-expr-input");
    expect(exprInput).not.toBeNull();
  });

  it("clicking fx again toggles back to numeric mode", () => {
    const host = makeHost();
    const { opts } = makeOpts({ plate_x: 100 });
    mountPropertyManager(host, opts);
    const fxBtn = host.querySelector<HTMLButtonElement>(".pm-fx-btn")!;
    fxBtn.click();
    fxBtn.click();
    const exprInput = host.querySelector<HTMLInputElement>(".pm-expr-input");
    expect(exprInput).toBeNull();
  });

  it("entering $other * 2 evaluates correctly when other=5 → preview shows = 10", () => {
    const host = makeHost();
    const params = { other: 5, plate_x: 100 };
    const { opts } = makeOpts(params);
    mountPropertyManager(host, opts);
    // Find the plate_x row fx button (rows are sorted alphabetically)
    const rows = host.querySelectorAll<HTMLElement>(".pm-row");
    // rows[0] = other, rows[1] = plate_x
    const plateRow = rows[1]!;
    const fxBtn = plateRow.querySelector<HTMLButtonElement>(".pm-fx-btn")!;
    fxBtn.click();
    const exprInput = plateRow.querySelector<HTMLInputElement>(".pm-expr-input")!;
    exprInput.value = "$other * 2";
    exprInput.dispatchEvent(new Event("input"));
    const preview = plateRow.querySelector(".pm-expr-preview");
    expect(preview).not.toBeNull();
    expect(preview!.textContent).toMatch(/^=\s*10(\.0+)?$/);
  });

  it("entering an invalid expression shows red error", () => {
    const host = makeHost();
    const { opts } = makeOpts({ plate_x: 100 });
    mountPropertyManager(host, opts);
    const fxBtn = host.querySelector<HTMLButtonElement>(".pm-fx-btn")!;
    fxBtn.click();
    const exprInput = host.querySelector<HTMLInputElement>(".pm-expr-input")!;
    exprInput.value = "$unknown_param + ??garbage";
    exprInput.dispatchEvent(new Event("input"));
    const preview = host.querySelector(".pm-expr-preview");
    expect(preview).not.toBeNull();
    expect(preview!.classList.contains("pm-expr-error")).toBe(true);
  });

  it("a valid expression calls setParam with the evaluated number", () => {
    const host = makeHost();
    const params = { base: 10, result: 0 };
    const { opts, setParam } = makeOpts(params);
    mountPropertyManager(host, opts);
    // result row (1) - alphabetically: base(0), result(1)
    const rows = host.querySelectorAll<HTMLElement>(".pm-row");
    const resultRow = rows[1]!;
    const fxBtn = resultRow.querySelector<HTMLButtonElement>(".pm-fx-btn")!;
    fxBtn.click();
    const exprInput = resultRow.querySelector<HTMLInputElement>(".pm-expr-input")!;
    exprInput.value = "$base * 3";
    exprInput.dispatchEvent(new Event("input"));
    expect(setParam).toHaveBeenCalledWith("result", 30);
  });
});

describe("PropertyManager — reset button", () => {
  it("reset button calls setParam with the default value", () => {
    const host = makeHost();
    const params = { plate_x: 55 };
    const defaults = { plate_x: 100 };
    const { opts, setParam } = makeOpts(params, defaults);
    mountPropertyManager(host, opts);
    const resetBtn = host.querySelector<HTMLButtonElement>(".pm-reset-btn")!;
    resetBtn.click();
    expect(setParam).toHaveBeenCalledWith("plate_x", 100);
  });
});

describe("PropertyManager — refresh", () => {
  it("refresh() re-renders rows to reflect updated param values", () => {
    const host = makeHost();
    let currentParams = { plate_x: 100 };
    const { setParam, onChange } = makeOpts();
    const opts: PropertyManagerOpts = {
      getParams: () => currentParams,
      getDefaults: () => ({ plate_x: 100 }),
      setParam,
      onChange,
    };
    const { refresh } = mountPropertyManager(host, opts);
    currentParams = { plate_x: 200 };
    refresh();
    const numInput = host.querySelector<HTMLInputElement>(".pm-number-input")!;
    expect(Number(numInput.value)).toBe(200);
  });

  it("refresh preserves expression-mode state when params change externally", () => {
    const host = makeHost();
    let currentParams: Record<string, number> = { plate_x: 100, other: 5 };
    const setParam = vi.fn((name: string, value: number) => {
      currentParams = { ...currentParams, [name]: value };
    });
    const onChange = vi.fn();
    const opts: PropertyManagerOpts = {
      getParams: () => ({ ...currentParams }),
      getDefaults: () => ({ plate_x: 100, other: 5 }),
      setParam,
      onChange,
    };
    const { refresh } = mountPropertyManager(host, opts);

    // Rows are sorted alphabetically: other(0), plate_x(1)
    const rows = host.querySelectorAll<HTMLElement>(".pm-row");
    const plateRow = rows[1]!;

    // Toggle fx on plate_x and type an expression
    const fxBtn = plateRow.querySelector<HTMLButtonElement>(".pm-fx-btn")!;
    fxBtn.click();
    const exprInput = plateRow.querySelector<HTMLInputElement>(".pm-expr-input")!;
    exprInput.value = "$other * 2";
    exprInput.dispatchEvent(new Event("input"));

    // Simulate an external setParam on 'other' triggering refresh
    currentParams = { ...currentParams, other: 10 };
    refresh();

    // Expression mode must still be active — expr input still present
    const exprInputAfter = plateRow.querySelector<HTMLInputElement>(".pm-expr-input");
    expect(exprInputAfter).not.toBeNull();
    // Expression text must be preserved
    expect(exprInputAfter!.value).toBe("$other * 2");
    // ƒx button still active
    expect(plateRow.querySelector(".pm-fx-btn")!.classList.contains("pm-fx-active")).toBe(true);
  });
});

describe("PropertyManager — slider range for negative defaults", () => {
  it("slider min is below the negative default and max is at or above 0", () => {
    const host = makeHost();
    const { opts } = makeOpts({ offset: -50 }, { offset: -50 });
    mountPropertyManager(host, opts);
    const slider = host.querySelector<HTMLInputElement>('input[type="range"]')!;
    expect(Number(slider.min)).toBeLessThan(-50);
    expect(Number(slider.max)).toBeGreaterThanOrEqual(0);
  });
});
