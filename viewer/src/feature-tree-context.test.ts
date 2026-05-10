/**
 * @vitest-environment jsdom
 */
import { describe, it, expect, vi, beforeEach, afterEach } from "vitest";
import {
  buildMenuItems,
  showContextMenu,
  dismissContextMenu,
  attachFeatureContextMenu,
  type FeatureContextCallbacks,
} from "./feature-tree-context.js";

function makeCallbacks(): FeatureContextCallbacks & {
  onRename: ReturnType<typeof vi.fn>;
  onSuppress: ReturnType<typeof vi.fn>;
  onDelete: ReturnType<typeof vi.fn>;
  onShowProperties: ReturnType<typeof vi.fn>;
} {
  return {
    onRename: vi.fn(),
    onSuppress: vi.fn(),
    onDelete: vi.fn(),
    onShowProperties: vi.fn(),
  };
}

describe("buildMenuItems", () => {
  it("shows exactly 4 menu items", () => {
    const items = buildMenuItems("feat1", makeCallbacks());
    expect(items).toHaveLength(4);
  });

  it("item labels are Rename / Suppress / Delete / Show Properties (in order)", () => {
    const items = buildMenuItems("feat1", makeCallbacks());
    expect(items.map((i) => i.label)).toEqual([
      "Rename",
      "Suppress",
      "Delete",
      "Show Properties",
    ]);
  });

  it("clicking Rename fires onRename with the feature id", () => {
    const cbs = makeCallbacks();
    const items = buildMenuItems("myFeature", cbs);
    items[0]!.action();
    expect(cbs.onRename).toHaveBeenCalledWith("myFeature");
    expect(cbs.onSuppress).not.toHaveBeenCalled();
  });

  it("clicking Suppress fires onSuppress with the feature id", () => {
    const cbs = makeCallbacks();
    const items = buildMenuItems("myFeature", cbs);
    items[1]!.action();
    expect(cbs.onSuppress).toHaveBeenCalledWith("myFeature");
  });

  it("clicking Delete fires onDelete with the feature id", () => {
    const cbs = makeCallbacks();
    const items = buildMenuItems("myFeature", cbs);
    items[2]!.action();
    expect(cbs.onDelete).toHaveBeenCalledWith("myFeature");
  });

  it("clicking Show Properties fires onShowProperties with the feature id", () => {
    const cbs = makeCallbacks();
    const items = buildMenuItems("myFeature", cbs);
    items[3]!.action();
    expect(cbs.onShowProperties).toHaveBeenCalledWith("myFeature");
  });
});

describe("showContextMenu / dismissContextMenu", () => {
  afterEach(() => {
    dismissContextMenu();
    document.body.innerHTML = "";
  });

  it("mounts a <ul> into the body", () => {
    const cbs = makeCallbacks();
    const items = buildMenuItems("f1", cbs);
    const menu = showContextMenu(100, 200, items);
    expect(document.body.contains(menu)).toBe(true);
    expect(menu.tagName).toBe("UL");
  });

  it("menu has exactly 4 <li> children", () => {
    const items = buildMenuItems("f1", makeCallbacks());
    const menu = showContextMenu(0, 0, items);
    expect(menu.querySelectorAll("li")).toHaveLength(4);
  });

  it("dismissContextMenu removes the menu from DOM", () => {
    const items = buildMenuItems("f1", makeCallbacks());
    const menu = showContextMenu(0, 0, items);
    expect(document.body.contains(menu)).toBe(true);
    dismissContextMenu();
    expect(document.body.contains(menu)).toBe(false);
  });

  it("showing a second menu removes the first", () => {
    const items = buildMenuItems("f1", makeCallbacks());
    const m1 = showContextMenu(0, 0, items);
    const m2 = showContextMenu(50, 50, buildMenuItems("f2", makeCallbacks()));
    expect(document.body.contains(m1)).toBe(false);
    expect(document.body.contains(m2)).toBe(true);
  });
});

describe("attachFeatureContextMenu", () => {
  let container: HTMLElement;

  beforeEach(() => {
    container = document.createElement("ol");
    // Add two <li data-id="..."> children.
    ["alpha", "beta"].forEach((id) => {
      const li = document.createElement("li");
      li.dataset.id = id;
      li.textContent = id;
      container.appendChild(li);
    });
    document.body.appendChild(container);
  });

  afterEach(() => {
    dismissContextMenu();
    document.body.innerHTML = "";
  });

  it("fires onDelete for the right-clicked feature via contextmenu event", () => {
    const cbs = makeCallbacks();
    attachFeatureContextMenu(container, cbs);

    // Find the "alpha" li and right-click it.
    const alphaLi = container.querySelector<HTMLElement>('[data-id="alpha"]')!;
    const evt = new MouseEvent("contextmenu", {
      bubbles: true,
      clientX: 50,
      clientY: 50,
    });
    alphaLi.dispatchEvent(evt);

    // Menu should now be visible; click the Delete item (index 2).
    const menu = document.querySelector<HTMLElement>(".feature-context-menu")!;
    expect(menu).not.toBeNull();
    const deleteItem = menu.querySelectorAll("li")[2] as HTMLElement;
    deleteItem.click();

    expect(cbs.onDelete).toHaveBeenCalledWith("alpha");
    expect(cbs.onRename).not.toHaveBeenCalled();
  });

  it("cleanup function removes the listener", () => {
    const cbs = makeCallbacks();
    const cleanup = attachFeatureContextMenu(container, cbs);
    cleanup();

    const alphaLi = container.querySelector<HTMLElement>('[data-id="alpha"]')!;
    alphaLi.dispatchEvent(
      new MouseEvent("contextmenu", { bubbles: true }),
    );
    // No menu should appear after cleanup.
    expect(document.querySelector(".feature-context-menu")).toBeNull();
  });
});
