/**
 * @vitest-environment jsdom
 */
import { describe, it, expect, beforeEach, vi } from "vitest";
import { mountConfigDropdown, mountDesignTable } from "./configurations.js";

type ModelJsonShape = {
  parameters?: Record<string, number>;
  configurations?: Record<string, Record<string, number>>;
  active_configuration?: string | null;
  features?: Array<{ id: string; kind: string }>;
};

function makeModelJson(extra: Partial<ModelJsonShape> = {}): string {
  return JSON.stringify({
    parameters: { width: 10, height: 5 },
    configurations: {
      Standard: { width: 10 },
      "Heavy-duty": { width: 20, height: 12 },
    },
    features: [{ id: "body", kind: "Box" }],
    ...extra,
  });
}

function setupHost(): HTMLElement {
  document.body.innerHTML = "";
  const host = document.createElement("div");
  document.body.appendChild(host);
  return host;
}

describe("mountConfigDropdown", () => {
  let host: HTMLElement;
  beforeEach(() => {
    host = setupHost();
  });

  it("renders Default + each configuration name", () => {
    const json = makeModelJson();
    const setModel = vi.fn();
    const ctrl = mountConfigDropdown(host, {
      getModel: () => json,
      setModel,
    });
    ctrl.refresh();

    const select = host.querySelector("select") as HTMLSelectElement;
    expect(select).toBeTruthy();
    const options = Array.from(select.options).map((o) => o.value);
    expect(options).toEqual(["", "Heavy-duty", "Standard"]);
    // The visible label for the first option says "Default".
    expect(select.options[0]!.textContent).toMatch(/default/i);
  });

  it("hides itself when no configurations are defined", () => {
    const json = JSON.stringify({
      parameters: {},
      features: [{ id: "body", kind: "Box" }],
    });
    const ctrl = mountConfigDropdown(host, {
      getModel: () => json,
      setModel: vi.fn(),
    });
    ctrl.refresh();
    const root = host.querySelector(".config-dropdown") as HTMLElement;
    expect(root.hidden).toBe(true);
  });

  it("selecting a config writes active_configuration into the model", () => {
    let json = makeModelJson();
    const setModel = vi.fn((next: string) => {
      json = next;
    });
    const ctrl = mountConfigDropdown(host, {
      getModel: () => json,
      setModel,
    });
    ctrl.refresh();

    const select = host.querySelector("select") as HTMLSelectElement;
    select.value = "Heavy-duty";
    select.dispatchEvent(new Event("change"));

    expect(setModel).toHaveBeenCalledTimes(1);
    const written = JSON.parse(setModel.mock.calls[0]![0]);
    expect(written.active_configuration).toBe("Heavy-duty");
  });

  it("selecting Default clears active_configuration", () => {
    let json = makeModelJson({ active_configuration: "Standard" });
    const setModel = vi.fn((next: string) => {
      json = next;
    });
    const ctrl = mountConfigDropdown(host, {
      getModel: () => json,
      setModel,
    });
    ctrl.refresh();

    const select = host.querySelector("select") as HTMLSelectElement;
    select.value = "";
    select.dispatchEvent(new Event("change"));

    const written = JSON.parse(setModel.mock.calls[0]![0]);
    expect(written.active_configuration ?? null).toBeNull();
  });

  it("reflects the current active_configuration in the dropdown", () => {
    const json = makeModelJson({ active_configuration: "Heavy-duty" });
    const ctrl = mountConfigDropdown(host, {
      getModel: () => json,
      setModel: vi.fn(),
    });
    ctrl.refresh();

    const select = host.querySelector("select") as HTMLSelectElement;
    expect(select.value).toBe("Heavy-duty");
  });
});

describe("mountDesignTable", () => {
  let host: HTMLElement;
  beforeEach(() => {
    host = setupHost();
  });

  it("renders one row per config plus a Default row", () => {
    const json = makeModelJson();
    const ctrl = mountDesignTable(host, {
      getModel: () => json,
      setModel: vi.fn(),
    });
    ctrl.refresh();

    // Expand the panel so the table is in the DOM.
    const toggle = host.querySelector(".dt-toggle") as HTMLElement;
    toggle.click();

    const rows = host.querySelectorAll("tbody tr");
    // 1 Default row + 2 named configs = 3 data rows. The "+ Add config"
    // button lives outside <tbody> rows or in its own row — check that
    // each named config has a row distinguishable by its label.
    const labels = Array.from(rows).map(
      (r) => r.querySelector(".dt-row-label")?.textContent ?? "",
    );
    expect(labels).toContain("Default");
    expect(labels).toContain("Standard");
    expect(labels).toContain("Heavy-duty");
  });

  it("Default row is read-only", () => {
    const json = makeModelJson();
    const ctrl = mountDesignTable(host, {
      getModel: () => json,
      setModel: vi.fn(),
    });
    ctrl.refresh();
    (host.querySelector(".dt-toggle") as HTMLElement).click();

    // Find the Default row's data cells; they should not be editable
    // <input> elements.
    const rows = Array.from(host.querySelectorAll("tbody tr"));
    const defaultRow = rows.find(
      (r) => r.querySelector(".dt-row-label")?.textContent === "Default",
    );
    expect(defaultRow).toBeTruthy();
    const inputs = defaultRow!.querySelectorAll("input");
    expect(inputs.length).toBe(0);
  });

  it("editing a cell writes the new override into the model", () => {
    let json = makeModelJson();
    const setModel = vi.fn((next: string) => {
      json = next;
    });
    const ctrl = mountDesignTable(host, {
      getModel: () => json,
      setModel,
    });
    ctrl.refresh();
    (host.querySelector(".dt-toggle") as HTMLElement).click();

    // Find the Standard row's "width" cell input and change it.
    const rows = Array.from(host.querySelectorAll("tbody tr"));
    const standardRow = rows.find(
      (r) => r.querySelector(".dt-row-label")?.textContent === "Standard",
    )!;
    const widthInput = standardRow.querySelector(
      'input[data-param="width"]',
    ) as HTMLInputElement;
    expect(widthInput).toBeTruthy();
    widthInput.value = "42";
    widthInput.dispatchEvent(new Event("change"));

    expect(setModel).toHaveBeenCalled();
    const written = JSON.parse(setModel.mock.calls.at(-1)![0]);
    expect(written.configurations.Standard.width).toBe(42);
  });

  it("blanking a cell removes the override (falls back to base)", () => {
    let json = makeModelJson();
    const setModel = vi.fn((next: string) => {
      json = next;
    });
    const ctrl = mountDesignTable(host, {
      getModel: () => json,
      setModel,
    });
    ctrl.refresh();
    (host.querySelector(".dt-toggle") as HTMLElement).click();

    const rows = Array.from(host.querySelectorAll("tbody tr"));
    const standardRow = rows.find(
      (r) => r.querySelector(".dt-row-label")?.textContent === "Standard",
    )!;
    const widthInput = standardRow.querySelector(
      'input[data-param="width"]',
    ) as HTMLInputElement;
    widthInput.value = "";
    widthInput.dispatchEvent(new Event("change"));

    const written = JSON.parse(setModel.mock.calls.at(-1)![0]);
    expect(written.configurations.Standard.width).toBeUndefined();
  });

  it("Add config button appends a new empty config", () => {
    let json = makeModelJson();
    const setModel = vi.fn((next: string) => {
      json = next;
    });
    const ctrl = mountDesignTable(host, {
      getModel: () => json,
      setModel,
    });
    ctrl.refresh();
    (host.querySelector(".dt-toggle") as HTMLElement).click();

    const addBtn = host.querySelector(".dt-add-config") as HTMLElement;
    expect(addBtn).toBeTruthy();
    // Pre-set the prompt return for jsdom.
    (window as unknown as { prompt: (msg: string) => string | null }).prompt = () =>
      "Compact";
    addBtn.click();

    const written = JSON.parse(setModel.mock.calls.at(-1)![0]);
    expect(written.configurations).toHaveProperty("Compact");
    expect(written.configurations.Compact).toEqual({});
  });

  it("non-numeric input is rejected (no setModel call)", () => {
    let json = makeModelJson();
    const setModel = vi.fn((next: string) => {
      json = next;
    });
    const ctrl = mountDesignTable(host, {
      getModel: () => json,
      setModel,
    });
    ctrl.refresh();
    (host.querySelector(".dt-toggle") as HTMLElement).click();

    const rows = Array.from(host.querySelectorAll("tbody tr"));
    const standardRow = rows.find(
      (r) => r.querySelector(".dt-row-label")?.textContent === "Standard",
    )!;
    const widthInput = standardRow.querySelector(
      'input[data-param="width"]',
    ) as HTMLInputElement;
    widthInput.value = "abc";
    widthInput.dispatchEvent(new Event("change"));

    // setModel must NOT be called for invalid numeric input. The cell
    // should signal the rejection visually — we only check the negative
    // here (no model mutation).
    expect(setModel).not.toHaveBeenCalled();
  });
});
