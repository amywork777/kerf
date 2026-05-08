// @vitest-environment jsdom
import { describe, it, expect, vi, beforeEach } from "vitest";
import { mountToolbar, type ToolbarOpts } from "./toolbar.js";

// Minimal model JSON with two features
const modelWithFeatures = JSON.stringify({
  parameters: {},
  features: [
    { kind: "Box", id: "box_1", extents: [10, 10, 10] },
    { kind: "Box", id: "box_2", extents: [5, 5, 5] },
  ],
});

const modelWithOneFeature = JSON.stringify({
  parameters: {},
  features: [{ kind: "Box", id: "box_1", extents: [10, 10, 10] }],
});

function makeOpts(overrides: Partial<ToolbarOpts> = {}): ToolbarOpts {
  return {
    getModel: () => null,
    setModel: vi.fn(),
    openSketcher: vi.fn(),
    getSelection: () => null,
    triggerFillet: vi.fn(),
    triggerChamfer: vi.fn(),
    ...overrides,
  };
}

describe("mountToolbar", () => {
  let host: HTMLDivElement;

  beforeEach(() => {
    host = document.createElement("div");
    document.body.appendChild(host);
  });

  it("renders 9 toolbar buttons in the correct order", () => {
    const opts = makeOpts();
    mountToolbar(host, opts);
    const buttons = host.querySelectorAll("button[data-toolbar-action]");
    expect(buttons).toHaveLength(9);
    const actions = Array.from(buttons).map((b) => (b as HTMLElement).dataset.toolbarAction);
    expect(actions).toEqual([
      "sketch",
      "extrude",
      "revolve",
      "fillet",
      "chamfer",
      "shell",
      "boolean",
      "pattern",
      "reference",
    ]);
  });

  it("clicking Sketch calls openSketcher", () => {
    const openSketcher = vi.fn();
    const opts = makeOpts({ openSketcher });
    mountToolbar(host, opts);
    const btn = host.querySelector<HTMLButtonElement>("button[data-toolbar-action='sketch']")!;
    btn.click();
    expect(openSketcher).toHaveBeenCalledOnce();
  });

  it("clicking Extrude with a model loaded calls setModel with new feature appended", () => {
    const setModel = vi.fn();
    const opts = makeOpts({
      getModel: () => ({ json: modelWithFeatures, targetId: "box_2" }),
      setModel,
    });
    mountToolbar(host, opts);
    const btn = host.querySelector<HTMLButtonElement>("button[data-toolbar-action='extrude']")!;
    btn.click();
    expect(setModel).toHaveBeenCalledOnce();
    const [newJson, newTarget] = setModel.mock.calls[0]!;
    const parsed = JSON.parse(newJson);
    expect(parsed.features).toHaveLength(3);
    const added = parsed.features[2];
    expect(added.kind).toBe("ExtrudePolygon");
    expect(typeof added.id).toBe("string");
    expect(added.id).toMatch(/^extrude_/);
    expect(newTarget).toBe(added.id);
  });

  it("clicking Extrude with no model has no effect (button disabled)", () => {
    const setModel = vi.fn();
    const opts = makeOpts({ getModel: () => null, setModel });
    mountToolbar(host, opts);
    const btn = host.querySelector<HTMLButtonElement>("button[data-toolbar-action='extrude']")!;
    expect(btn.disabled).toBe(true);
    btn.click();
    expect(setModel).not.toHaveBeenCalled();
  });

  it("extrude auto-increments ID: extrude_1, extrude_2 etc.", () => {
    const setModel = vi.fn();
    // Model already has an extrude_1 feature
    const modelWithExtrude = JSON.stringify({
      parameters: {},
      features: [
        { kind: "Box", id: "box_1", extents: [10, 10, 10] },
        { kind: "ExtrudePolygon", id: "extrude_1", profile: [], direction: [0, 0, 1] },
      ],
    });
    let currentJson = modelWithExtrude;
    const opts = makeOpts({
      getModel: () => ({ json: currentJson, targetId: "extrude_1" }),
      setModel: (json: string) => { currentJson = json; setModel(json); },
    });
    mountToolbar(host, opts);
    const btn = host.querySelector<HTMLButtonElement>("button[data-toolbar-action='extrude']")!;
    btn.click();
    const parsed = JSON.parse(setModel.mock.calls[0]![0]);
    const added = parsed.features[2];
    expect(added.id).toBe("extrude_2");
  });

  it("clicking Boolean reveals Union/Difference/Intersection dropdown items", () => {
    const opts = makeOpts({
      getModel: () => ({ json: modelWithFeatures, targetId: "box_2" }),
    });
    mountToolbar(host, opts);
    const boolBtn = host.querySelector<HTMLButtonElement>("button[data-toolbar-action='boolean']")!;
    boolBtn.click();
    const dropdown = host.querySelector("[data-toolbar-dropdown='boolean']") as HTMLElement;
    expect(dropdown).not.toBeNull();
    expect(dropdown.style.display).not.toBe("none");
    const subBtns = dropdown.querySelectorAll("button[data-toolbar-sub]");
    const subs = Array.from(subBtns).map((b) => (b as HTMLElement).dataset.toolbarSub);
    expect(subs).toContain("union");
    expect(subs).toContain("difference");
    expect(subs).toContain("intersection");
  });

  it("clicking Union appends a Union feature with last 2 feature ids", () => {
    const setModel = vi.fn();
    const opts = makeOpts({
      getModel: () => ({ json: modelWithFeatures, targetId: "box_2" }),
      setModel,
    });
    mountToolbar(host, opts);
    // Open boolean dropdown
    const boolBtn = host.querySelector<HTMLButtonElement>("button[data-toolbar-action='boolean']")!;
    boolBtn.click();
    // Click Union sub-button
    const unionBtn = host.querySelector<HTMLButtonElement>("button[data-toolbar-sub='union']")!;
    unionBtn.click();
    expect(setModel).toHaveBeenCalledOnce();
    const [newJson, newTarget] = setModel.mock.calls[0]!;
    const parsed = JSON.parse(newJson);
    expect(parsed.features).toHaveLength(3);
    const added = parsed.features[2];
    expect(added.kind).toBe("Union");
    expect(Array.isArray(added.inputs)).toBe(true);
    expect(added.inputs).toContain("box_1");
    expect(added.inputs).toContain("box_2");
    expect(newTarget).toBe(added.id);
  });

  it("Revolve button is disabled when no model loaded", () => {
    const opts = makeOpts({ getModel: () => null });
    mountToolbar(host, opts);
    const btn = host.querySelector<HTMLButtonElement>("button[data-toolbar-action='revolve']")!;
    expect(btn.disabled).toBe(true);
  });

  it("Shell button is disabled when no model loaded", () => {
    const opts = makeOpts({ getModel: () => null });
    mountToolbar(host, opts);
    const btn = host.querySelector<HTMLButtonElement>("button[data-toolbar-action='shell']")!;
    expect(btn.disabled).toBe(true);
  });

  it("clicking Revolve with model appends a Revolve feature", () => {
    const setModel = vi.fn();
    const opts = makeOpts({
      getModel: () => ({ json: modelWithOneFeature, targetId: "box_1" }),
      setModel,
    });
    mountToolbar(host, opts);
    const btn = host.querySelector<HTMLButtonElement>("button[data-toolbar-action='revolve']")!;
    btn.click();
    expect(setModel).toHaveBeenCalledOnce();
    const parsed = JSON.parse(setModel.mock.calls[0]![0]);
    const added = parsed.features[1];
    expect(added.kind).toBe("Revolve");
    expect(added.id).toMatch(/^revolve_/);
  });

  it("clicking Shell with model appends a Shell-like feature", () => {
    const setModel = vi.fn();
    const opts = makeOpts({
      getModel: () => ({ json: modelWithOneFeature, targetId: "box_1" }),
      setModel,
    });
    mountToolbar(host, opts);
    const btn = host.querySelector<HTMLButtonElement>("button[data-toolbar-action='shell']")!;
    btn.click();
    expect(setModel).toHaveBeenCalledOnce();
    const parsed = JSON.parse(setModel.mock.calls[0]![0]);
    const added = parsed.features[1];
    expect(added.kind).toBe("Shell");
    expect(added.id).toMatch(/^shell_/);
    expect(added.input).toBe("box_1");
    expect(added.thickness).toBe(1.0);
  });

  it("refresh() updates disabled state when model is loaded after mount", () => {
    let currentModel: { json: string; targetId: string } | null = null;
    const opts = makeOpts({ getModel: () => currentModel });
    const toolbar = mountToolbar(host, opts);
    const extrudeBtn = host.querySelector<HTMLButtonElement>("button[data-toolbar-action='extrude']")!;
    expect(extrudeBtn.disabled).toBe(true);
    currentModel = { json: modelWithOneFeature, targetId: "box_1" };
    toolbar.refresh();
    expect(extrudeBtn.disabled).toBe(false);
  });

  it("Pattern dropdown shows Linear and Polar sub-buttons", () => {
    const opts = makeOpts({
      getModel: () => ({ json: modelWithFeatures, targetId: "box_2" }),
    });
    mountToolbar(host, opts);
    const btn = host.querySelector<HTMLButtonElement>("button[data-toolbar-action='pattern']")!;
    btn.click();
    const dropdown = host.querySelector("[data-toolbar-dropdown='pattern']") as HTMLElement;
    expect(dropdown).not.toBeNull();
    const subs = Array.from(dropdown.querySelectorAll("button[data-toolbar-sub]")).map(
      (b) => (b as HTMLElement).dataset.toolbarSub,
    );
    expect(subs).toContain("linear-pattern");
    expect(subs).toContain("polar-pattern");
  });

  it("Reference dropdown shows Plane, Axis, Point sub-buttons", () => {
    const opts = makeOpts();
    mountToolbar(host, opts);
    const btn = host.querySelector<HTMLButtonElement>("button[data-toolbar-action='reference']")!;
    btn.click();
    const dropdown = host.querySelector("[data-toolbar-dropdown='reference']") as HTMLElement;
    expect(dropdown).not.toBeNull();
    const subs = Array.from(dropdown.querySelectorAll("button[data-toolbar-sub]")).map(
      (b) => (b as HTMLElement).dataset.toolbarSub,
    );
    expect(subs).toContain("ref-plane");
    expect(subs).toContain("ref-axis");
    expect(subs).toContain("ref-point");
  });
});
