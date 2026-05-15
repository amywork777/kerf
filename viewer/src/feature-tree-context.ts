/**
 * feature-tree-context.ts
 *
 * Context menu for the feature tree. Right-clicking a feature <li> builds
 * and shows a small absolutely-positioned menu with four actions:
 * Rename / Suppress / Delete / Show Properties.
 *
 * The menu positions itself next to the cursor and dismisses on any
 * outside click, Escape, or scroll.
 */

export interface FeatureContextCallbacks {
  onRename(featureId: string): void;
  onSuppress(featureId: string): void;
  onDelete(featureId: string): void;
  onShowProperties(featureId: string): void;
}

export interface MenuItem {
  label: string;
  action: () => void;
}

/** Build the ordered list of context menu items for a feature. */
export function buildMenuItems(
  featureId: string,
  callbacks: FeatureContextCallbacks,
): MenuItem[] {
  return [
    { label: "Rename", action: () => callbacks.onRename(featureId) },
    { label: "Suppress", action: () => callbacks.onSuppress(featureId) },
    { label: "Delete", action: () => callbacks.onDelete(featureId) },
    { label: "Show Properties", action: () => callbacks.onShowProperties(featureId) },
  ];
}

let activeMenu: HTMLElement | null = null;

/** Remove any currently-open context menu from the DOM. */
export function dismissContextMenu(): void {
  if (activeMenu) {
    activeMenu.remove();
    activeMenu = null;
  }
}

/**
 * Show a context menu at (x, y) with the given items.
 * Any previously open menu is removed first.
 */
export function showContextMenu(
  x: number,
  y: number,
  items: MenuItem[],
): HTMLElement {
  dismissContextMenu();

  const menu = document.createElement("ul");
  menu.className = "feature-context-menu";
  menu.style.cssText = [
    "position:fixed",
    `left:${x}px`,
    `top:${y}px`,
    "margin:0",
    "padding:4px 0",
    "list-style:none",
    "background:var(--panel,#161b22)",
    "border:1px solid var(--line,#30363d)",
    "border-radius:4px",
    "z-index:999",
    "min-width:150px",
    "box-shadow:0 4px 16px rgba(0,0,0,0.4)",
  ].join(";");

  for (const item of items) {
    const li = document.createElement("li");
    li.textContent = item.label;
    li.style.cssText = [
      "padding:6px 14px",
      "cursor:pointer",
      "font:13px -apple-system,BlinkMacSystemFont,'Segoe UI',system-ui,sans-serif",
      "color:var(--fg,#e6edf3)",
      "white-space:nowrap",
    ].join(";");
    li.addEventListener("mouseenter", () => {
      li.style.background = "var(--bg,#0d1116)";
      li.style.color = "var(--accent,#58a6ff)";
    });
    li.addEventListener("mouseleave", () => {
      li.style.background = "";
      li.style.color = "var(--fg,#e6edf3)";
    });
    li.addEventListener("click", (e) => {
      e.stopPropagation();
      dismissContextMenu();
      item.action();
    });
    menu.appendChild(li);
  }

  document.body.appendChild(menu);
  activeMenu = menu;

  // Dismiss on outside click.
  const onOutside = (e: MouseEvent) => {
    if (!menu.contains(e.target as Node)) {
      dismissContextMenu();
      document.removeEventListener("mousedown", onOutside, true);
      document.removeEventListener("keydown", onEscape, true);
      document.removeEventListener("scroll", onScroll, true);
    }
  };
  const onEscape = (e: KeyboardEvent) => {
    if (e.key === "Escape") {
      dismissContextMenu();
      document.removeEventListener("mousedown", onOutside, true);
      document.removeEventListener("keydown", onEscape, true);
      document.removeEventListener("scroll", onScroll, true);
    }
  };
  const onScroll = () => {
    dismissContextMenu();
    document.removeEventListener("mousedown", onOutside, true);
    document.removeEventListener("keydown", onEscape, true);
    document.removeEventListener("scroll", onScroll, true);
  };

  // Use capture so we catch clicks on the canvas / outside panel.
  document.addEventListener("mousedown", onOutside, true);
  document.addEventListener("keydown", onEscape, true);
  document.addEventListener("scroll", onScroll, true);

  return menu;
}

/**
 * Attach contextmenu listeners to all <li> children of featureListEl.
 * Returns a cleanup function that removes the listeners.
 */
export function attachFeatureContextMenu(
  featureListEl: HTMLElement,
  callbacks: FeatureContextCallbacks,
): () => void {
  const handler = (e: MouseEvent) => {
    const li = (e.target as Element).closest("li[data-id]") as HTMLElement | null;
    if (!li) return;
    e.preventDefault();
    const featureId = li.dataset.id!;
    const items = buildMenuItems(featureId, callbacks);
    showContextMenu(e.clientX, e.clientY, items);
  };

  featureListEl.addEventListener("contextmenu", handler);
  return () => featureListEl.removeEventListener("contextmenu", handler);
}
