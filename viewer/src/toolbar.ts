// SolidWorks-style feature-insertion toolbar that mounts above the 3D canvas.
//
// Exported API:
//   mountToolbar(host, opts) → { refresh() }
//
// All state mutations go through opts.setModel — the toolbar itself holds no
// model state.  opts.getModel() is called at interaction time (never cached).

export type ToolbarOpts = {
  /** Return the current model, or null if none loaded. */
  getModel(): { json: string; targetId: string } | null;
  /** Set a new model JSON string.  newTargetId is the feature to make active. */
  setModel(json: string, newTargetId?: string): void;
  /** Open the 2D sketcher panel (delegates to the existing toggle handler). */
  openSketcher(): void;
  /** Return the currently selected topology, or null. */
  getSelection(): { faceId?: number; edgeId?: number } | null;
  /** Trigger the picking-driven fillet flow. */
  triggerFillet(): void;
  /** Trigger the picking-driven chamfer flow. */
  triggerChamfer(): void;
};

export type ToolbarHandle = {
  /** Re-evaluate disabled states based on the current model. */
  refresh(): void;
};

// ---------------------------------------------------------------------------
// ID generation helpers
// ---------------------------------------------------------------------------

/** Return the next incremental id for a given prefix, e.g. "extrude_3". */
function nextId(json: string, prefix: string): string {
  try {
    const parsed = JSON.parse(json) as { features?: Array<{ id?: string }> };
    const features = parsed.features ?? [];
    let max = 0;
    const re = new RegExp(`^${prefix}_(\\d+)$`);
    for (const f of features) {
      if (typeof f.id === "string") {
        const m = f.id.match(re);
        if (m) max = Math.max(max, parseInt(m[1]!, 10));
      }
    }
    return `${prefix}_${max + 1}`;
  } catch {
    return `${prefix}_1`;
  }
}

/** Return the last N feature ids from a model JSON. */
function lastFeatureIds(json: string, n: number): string[] {
  try {
    const parsed = JSON.parse(json) as { features?: Array<{ id?: string }> };
    const ids = (parsed.features ?? [])
      .map((f) => f.id)
      .filter((id): id is string => typeof id === "string");
    return ids.slice(-n);
  } catch {
    return [];
  }
}

function featureCount(json: string): number {
  try {
    const parsed = JSON.parse(json) as { features?: unknown[] };
    return (parsed.features ?? []).length;
  } catch {
    return 0;
  }
}

/** Append a feature object to the model JSON and return the updated JSON. */
function appendFeature(
  json: string,
  feature: Record<string, unknown>,
): string {
  const parsed = JSON.parse(json) as { features?: Array<Record<string, unknown>> };
  parsed.features = [...(parsed.features ?? []), feature];
  return JSON.stringify(parsed, null, 2);
}

// ---------------------------------------------------------------------------
// Feature builders — all return { newJson, newId }
// ---------------------------------------------------------------------------

function buildExtrude(json: string, currentTargetId: string): { newJson: string; newId: string } {
  const id = nextId(json, "extrude");
  // Default: a unit square extruded 10 units in +z.
  const feature: Record<string, unknown> = {
    kind: "ExtrudePolygon",
    id,
    input: currentTargetId,
    profile: [
      [0, 0], [10, 0], [10, 10], [0, 10],
    ],
    direction: [0, 0, 10],
  };
  return { newJson: appendFeature(json, feature), newId: id };
}

function buildRevolve(json: string, currentTargetId: string): { newJson: string; newId: string } {
  const id = nextId(json, "revolve");
  // Default: a simple stepped profile suitable for a rotational solid.
  // Profile in xz-plane: first/last x=0 (on z-axis), interior x>0.
  const feature: Record<string, unknown> = {
    kind: "Revolve",
    id,
    input: currentTargetId,
    profile: [
      [0, 0], [5, 0], [5, 10], [0, 10],
    ],
  };
  return { newJson: appendFeature(json, feature), newId: id };
}

function buildShell(json: string, currentTargetId: string): { newJson: string; newId: string } {
  const id = nextId(json, "shell");
  const feature: Record<string, unknown> = {
    kind: "Shell",
    id,
    input: currentTargetId,
    thickness: 1.0,
  };
  return { newJson: appendFeature(json, feature), newId: id };
}

function buildBoolean(
  json: string,
  sub: "union" | "difference" | "intersection",
): { newJson: string; newId: string } {
  const kindMap = {
    union: "Union",
    difference: "Difference",
    intersection: "Intersection",
  } as const;
  const prefixMap = {
    union: "bool_union",
    difference: "bool_diff",
    intersection: "bool_intersect",
  } as const;
  const id = nextId(json, prefixMap[sub]);
  const inputs = lastFeatureIds(json, 2);
  const feature: Record<string, unknown> = {
    kind: kindMap[sub],
    id,
    inputs,
  };
  return { newJson: appendFeature(json, feature), newId: id };
}

function buildLinearPattern(
  json: string,
  currentTargetId: string,
): { newJson: string; newId: string } {
  const id = nextId(json, "lin_pattern");
  const feature: Record<string, unknown> = {
    kind: "HoleArray",
    id,
    input: currentTargetId,
    axis: "z",
    start: [0, 0, 0],
    offset: [10, 0, 0],
    count: 3,
    radius: 2,
    depth: 5,
    segments: 16,
  };
  return { newJson: appendFeature(json, feature), newId: id };
}

function buildPolarPattern(
  json: string,
  currentTargetId: string,
): { newJson: string; newId: string } {
  const id = nextId(json, "pol_pattern");
  const feature: Record<string, unknown> = {
    kind: "BoltCircle",
    id,
    input: currentTargetId,
    axis: "z",
    center: [0, 0, 0],
    bolt_circle_radius: 20,
    count: 6,
    radius: 2,
    depth: 5,
    segments: 16,
  };
  return { newJson: appendFeature(json, feature), newId: id };
}

function buildRefPlane(json: string): { newJson: string; newId: string } {
  const id = nextId(json, "ref_plane");
  const feature: Record<string, unknown> = {
    kind: "RefPlane",
    id,
    normal: [0, 0, 1],
    origin: [0, 0, 0],
  };
  return { newJson: appendFeature(json, feature), newId: id };
}

function buildRefAxis(json: string): { newJson: string; newId: string } {
  const id = nextId(json, "ref_axis");
  const feature: Record<string, unknown> = {
    kind: "RefAxis",
    id,
    direction: [0, 0, 1],
    origin: [0, 0, 0],
  };
  return { newJson: appendFeature(json, feature), newId: id };
}

function buildRefPoint(json: string): { newJson: string; newId: string } {
  const id = nextId(json, "ref_point");
  const feature: Record<string, unknown> = {
    kind: "RefPoint",
    id,
    position: [0, 0, 0],
  };
  return { newJson: appendFeature(json, feature), newId: id };
}

// ---------------------------------------------------------------------------
// DOM helpers
// ---------------------------------------------------------------------------

function btn(
  label: string,
  action: string,
  title?: string,
): HTMLButtonElement {
  const b = document.createElement("button");
  b.dataset.toolbarAction = action;
  b.innerHTML = label;
  b.title = title ?? label.replace(/<[^>]*>/g, "");
  return b;
}

function dropdownBtn(
  label: string,
  sub: string,
): HTMLButtonElement {
  const b = document.createElement("button");
  b.dataset.toolbarSub = sub;
  b.textContent = label;
  return b;
}

function makeDropdown(id: string, items: HTMLButtonElement[]): HTMLDivElement {
  const div = document.createElement("div");
  div.dataset.toolbarDropdown = id;
  div.style.cssText =
    "display:none;position:absolute;top:100%;left:0;z-index:200;" +
    "background:var(--panel);border:1px solid var(--line);border-radius:4px;" +
    "padding:4px;min-width:120px;box-shadow:0 4px 12px rgba(0,0,0,0.4);";
  for (const item of items) {
    item.style.cssText =
      "display:block;width:100%;text-align:left;margin-bottom:2px;flex:unset;";
    div.appendChild(item);
  }
  return div;
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

export function mountToolbar(
  host: HTMLElement,
  opts: ToolbarOpts,
): ToolbarHandle {
  // ------------------------------------------------------------------
  // Outer container
  // ------------------------------------------------------------------
  const bar = document.createElement("div");
  bar.id = "toolbar-inner";
  bar.style.cssText =
    "display:flex;align-items:center;gap:2px;height:100%;padding:0 8px;" +
    "overflow-x:auto;";

  host.style.position = "relative";
  host.appendChild(bar);

  // Track all buttons that require a model so we can disable/enable them.
  const modelButtons: HTMLButtonElement[] = [];
  // Buttons that additionally require >= 2 features (booleans).
  const multiFeatureButtons: HTMLButtonElement[] = [];
  // All open dropdowns (at most one at a time).
  let openDropdown: HTMLDivElement | null = null;

  function closeDropdowns() {
    if (openDropdown) {
      openDropdown.style.display = "none";
      openDropdown = null;
    }
  }

  const onDocClick = (e: MouseEvent) => {
    if (!bar.contains(e.target as Node)) closeDropdowns();
  };
  document.addEventListener("click", onDocClick);

  function addButton(
    label: string,
    action: string,
    needsModel: boolean,
    onClick: () => void,
    title?: string,
  ): HTMLButtonElement {
    const b = btn(label, action, title);
    if (needsModel) {
      modelButtons.push(b);
      b.disabled = opts.getModel() === null;
    }
    b.addEventListener("click", (e) => {
      e.stopPropagation();
      if (b.disabled) return;
      onClick();
    });
    bar.appendChild(b);
    return b;
  }

  function addDropdownButton(
    label: string,
    action: string,
    dropdownId: string,
    items: { label: string; sub: string; onClick: () => void }[],
    title?: string,
    minFeatures = 0,
  ): HTMLButtonElement {
    const wrapper = document.createElement("div");
    wrapper.style.cssText = "position:relative;display:flex;";
    bar.appendChild(wrapper);

    const b = btn(label, action, title);
    b.style.cssText += ";padding-right:14px;";
    wrapper.appendChild(b);

    const subBtns = items.map((item) => {
      const sb = dropdownBtn(item.label, item.sub);
      sb.addEventListener("click", (e) => {
        e.stopPropagation();
        closeDropdowns();
        item.onClick();
      });
      return sb;
    });

    const dd = makeDropdown(dropdownId, subBtns);
    wrapper.appendChild(dd);

    b.addEventListener("click", (e) => {
      e.stopPropagation();
      if (b.disabled) return;
      const wasOpen = openDropdown === dd;
      closeDropdowns();
      if (!wasOpen) {
        dd.style.display = "block";
        openDropdown = dd;
      }
    });

    if (minFeatures >= 2) {
      multiFeatureButtons.push(b);
    } else if (minFeatures >= 1) {
      modelButtons.push(b);
    }

    return b;
  }

  // ------------------------------------------------------------------
  // 1. Sketch
  // ------------------------------------------------------------------
  addButton("✏ Sketch", "sketch", false, () => {
    opts.openSketcher();
  });

  // ------------------------------------------------------------------
  // 2. Extrude
  // ------------------------------------------------------------------
  addButton("↑ Extrude", "extrude", true, () => {
    const m = opts.getModel();
    if (!m) return;
    const { newJson, newId } = buildExtrude(m.json, m.targetId);
    opts.setModel(newJson, newId);
  });

  // ------------------------------------------------------------------
  // 3. Revolve
  // ------------------------------------------------------------------
  addButton("↻ Revolve", "revolve", true, () => {
    const m = opts.getModel();
    if (!m) return;
    const { newJson, newId } = buildRevolve(m.json, m.targetId);
    opts.setModel(newJson, newId);
  });

  // ------------------------------------------------------------------
  // 4. Fillet
  // ------------------------------------------------------------------
  addButton("⌒ Fillet", "fillet", true, () => {
    const sel = opts.getSelection();
    if (sel?.edgeId !== undefined) {
      opts.triggerFillet();
    } else {
      // Show a hint in the status bar if available — no-op otherwise.
      const statusEl = document.getElementById("status");
      if (statusEl) statusEl.textContent = "Select an edge first";
    }
  });

  // ------------------------------------------------------------------
  // 5. Chamfer
  // ------------------------------------------------------------------
  addButton("◇ Chamfer", "chamfer", true, () => {
    const sel = opts.getSelection();
    if (sel?.edgeId !== undefined) {
      opts.triggerChamfer();
    } else {
      const statusEl = document.getElementById("status");
      if (statusEl) statusEl.textContent = "Select an edge first";
    }
  });

  // ------------------------------------------------------------------
  // 6. Shell
  // ------------------------------------------------------------------
  addButton("⬜ Shell", "shell", true, () => {
    const m = opts.getModel();
    if (!m) return;
    const { newJson, newId } = buildShell(m.json, m.targetId);
    opts.setModel(newJson, newId);
  });

  // ------------------------------------------------------------------
  // 7. Boolean (dropdown: Union / Difference / Intersection)
  // ------------------------------------------------------------------
  addDropdownButton(
    "⊕ Boolean ▾",
    "boolean",
    "boolean",
    [
      {
        label: "Union",
        sub: "union",
        onClick: () => {
          const m = opts.getModel();
          if (!m) return;
          const { newJson, newId } = buildBoolean(m.json, "union");
          opts.setModel(newJson, newId);
        },
      },
      {
        label: "Difference",
        sub: "difference",
        onClick: () => {
          const m = opts.getModel();
          if (!m) return;
          const { newJson, newId } = buildBoolean(m.json, "difference");
          opts.setModel(newJson, newId);
        },
      },
      {
        label: "Intersection",
        sub: "intersection",
        onClick: () => {
          const m = opts.getModel();
          if (!m) return;
          const { newJson, newId } = buildBoolean(m.json, "intersection");
          opts.setModel(newJson, newId);
        },
      },
    ],
    "Boolean operations",
    2,
  );

  // ------------------------------------------------------------------
  // 8. Pattern (dropdown: Linear / Polar)
  // ------------------------------------------------------------------
  addDropdownButton(
    "⋮⋮ Pattern ▾",
    "pattern",
    "pattern",
    [
      {
        label: "Linear",
        sub: "linear-pattern",
        onClick: () => {
          const m = opts.getModel();
          if (!m) return;
          const { newJson, newId } = buildLinearPattern(m.json, m.targetId);
          opts.setModel(newJson, newId);
        },
      },
      {
        label: "Polar",
        sub: "polar-pattern",
        onClick: () => {
          const m = opts.getModel();
          if (!m) return;
          const { newJson, newId } = buildPolarPattern(m.json, m.targetId);
          opts.setModel(newJson, newId);
        },
      },
    ],
    "Pattern features",
  );

  // ------------------------------------------------------------------
  // 9. Reference (dropdown: Plane / Axis / Point)
  // ------------------------------------------------------------------
  addDropdownButton(
    "⊞ Reference ▾",
    "reference",
    "reference",
    [
      {
        label: "Plane",
        sub: "ref-plane",
        onClick: () => {
          const m = opts.getModel();
          if (!m) return;
          const { newJson, newId } = buildRefPlane(m.json);
          opts.setModel(newJson, newId);
        },
      },
      {
        label: "Axis",
        sub: "ref-axis",
        onClick: () => {
          const m = opts.getModel();
          if (!m) return;
          const { newJson, newId } = buildRefAxis(m.json);
          opts.setModel(newJson, newId);
        },
      },
      {
        label: "Point",
        sub: "ref-point",
        onClick: () => {
          const m = opts.getModel();
          if (!m) return;
          const { newJson, newId } = buildRefPoint(m.json);
          opts.setModel(newJson, newId);
        },
      },
    ],
    "Reference geometry",
  );

  // ------------------------------------------------------------------
  // Handle
  // ------------------------------------------------------------------
  function refresh() {
    const m = opts.getModel();
    const hasModel = m !== null;
    const fc = m ? featureCount(m.json) : 0;
    for (const b of modelButtons) b.disabled = !hasModel;
    for (const b of multiFeatureButtons) b.disabled = fc < 2;
  }
  refresh();
  return {
    refresh,
    destroy() {
      document.removeEventListener("click", onDocClick);
      bar.remove();
    },
  };
}
