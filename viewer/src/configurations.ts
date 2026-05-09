/**
 * Configuration Manager + Design Table.
 *
 * Mirrors SolidWorks's "configurations" feature: a Model can ship
 * multiple named parameter overlays, plus an Excel-style grid editor
 * for them.
 *
 * Two independent mounts:
 * - `mountConfigDropdown(host, opts)` — a labelled <select> with a
 *   "Default" entry plus one per configuration. Selection writes
 *   `active_configuration` back into the model JSON via opts.setModel.
 * - `mountDesignTable(host, opts)` — a collapsible panel with a
 *   <table>: header row of params, one body row per config (plus an
 *   immutable "Default" row showing the base parameters), and an
 *   "+ Add config" button. Edits write back via opts.setModel; the
 *   parent is expected to trigger a model rebuild.
 *
 * The module is DOM-only — no three.js, no WASM. The host page wires
 * setModel to its rebuild pipeline. Both controls expose a `refresh()`
 * to re-render after the underlying JSON changes.
 *
 * # Robustness
 * - Invalid JSON in `getModel()` is treated as "no model": both
 *   controls hide themselves.
 * - Editing a cell to a non-numeric string visually marks the cell as
 *   invalid (`.invalid` class) and skips the setModel call. The user
 *   is left with the bad text in place to fix.
 * - Editing to a blank string removes the override (the cell falls
 *   back to the base `parameters` value at evaluation time).
 * - The "Default" row is rendered with read-only <span>s, never inputs.
 */

export interface ConfigPanelOpts {
  /** Return the current model JSON string. Called on every refresh and
   * on every UI mutation, so the host can keep authoritative state. */
  getModel(): string | null;
  /** Persist the updated model JSON. Called when the user mutates
   * `active_configuration` or `configurations`. The host typically
   * re-evaluates and re-renders after this. */
  setModel(json: string): void;
}

export interface ConfigPanel {
  /** Re-read the model from `getModel()` and re-render the panel. */
  refresh(): void;
}

type ModelJson = {
  parameters?: Record<string, number>;
  configurations?: Record<string, Record<string, number>>;
  active_configuration?: string | null;
  features?: Array<unknown>;
};

function parseModel(json: string | null): ModelJson | null {
  if (!json) return null;
  try {
    const parsed = JSON.parse(json) as ModelJson;
    return parsed && typeof parsed === "object" ? parsed : null;
  } catch {
    return null;
  }
}

function sortedConfigNames(model: ModelJson): string[] {
  const cs = model.configurations ?? {};
  return Object.keys(cs).sort();
}

// ---------------------------------------------------------------------------
// Config dropdown
// ---------------------------------------------------------------------------

export function mountConfigDropdown(
  host: HTMLElement,
  opts: ConfigPanelOpts,
): ConfigPanel {
  const root = document.createElement("div");
  root.className = "config-dropdown";
  root.innerHTML = `
    <label for="config-select">Configuration</label>
    <select id="config-select"></select>
  `;
  host.appendChild(root);

  const select = root.querySelector("select") as HTMLSelectElement;

  select.addEventListener("change", () => {
    const json = opts.getModel();
    const model = parseModel(json);
    if (!model) return;
    const next = select.value === "" ? null : select.value;
    if (next === null) {
      delete model.active_configuration;
    } else {
      model.active_configuration = next;
    }
    opts.setModel(JSON.stringify(model, null, 2));
  });

  function refresh() {
    const model = parseModel(opts.getModel());
    if (!model) {
      root.hidden = true;
      return;
    }
    const names = sortedConfigNames(model);
    if (names.length === 0) {
      // Hide the whole control when no configs exist — the implicit
      // default is the only choice and a static dropdown would just
      // be visual noise.
      root.hidden = true;
      return;
    }
    root.hidden = false;

    // Rebuild options. First option is always "Default" (value === "").
    select.innerHTML = "";
    const defaultOpt = document.createElement("option");
    defaultOpt.value = "";
    defaultOpt.textContent = "Default";
    select.appendChild(defaultOpt);
    for (const n of names) {
      const opt = document.createElement("option");
      opt.value = n;
      opt.textContent = n;
      select.appendChild(opt);
    }
    select.value = model.active_configuration ?? "";
  }

  refresh();
  return { refresh };
}

// ---------------------------------------------------------------------------
// Design table
// ---------------------------------------------------------------------------

export function mountDesignTable(
  host: HTMLElement,
  opts: ConfigPanelOpts,
): ConfigPanel {
  const root = document.createElement("div");
  root.className = "design-table";
  root.innerHTML = `
    <div class="dt-header">
      <span class="dt-toggle" role="button" aria-expanded="false">▶</span>
      <span class="dt-title">Design Table</span>
    </div>
    <div class="dt-body" hidden></div>
  `;
  host.appendChild(root);

  const toggle = root.querySelector(".dt-toggle") as HTMLElement;
  const body = root.querySelector(".dt-body") as HTMLElement;

  toggle.addEventListener("click", () => {
    const collapsed = body.hidden;
    body.hidden = !collapsed;
    toggle.textContent = collapsed ? "▼" : "▶";
    toggle.setAttribute("aria-expanded", collapsed ? "true" : "false");
    if (collapsed) renderTable();
  });

  function renderTable() {
    const model = parseModel(opts.getModel());
    body.innerHTML = "";
    if (!model) {
      const p = document.createElement("p");
      p.className = "dt-empty";
      p.textContent = "No model loaded.";
      body.appendChild(p);
      return;
    }

    // The column order: every parameter the model knows about, plus
    // every parameter referenced by any configuration's overlay (in
    // case a config overrides a non-default key — robustness against
    // hand-edited JSON).
    const baseParams = model.parameters ?? {};
    const configs = model.configurations ?? {};
    const paramSet = new Set<string>(Object.keys(baseParams));
    for (const overlay of Object.values(configs)) {
      for (const k of Object.keys(overlay)) paramSet.add(k);
    }
    const params = Array.from(paramSet).sort();
    const configNames = sortedConfigNames(model);

    const table = document.createElement("table");
    table.className = "dt-grid";

    // Header row.
    const thead = document.createElement("thead");
    const headerTr = document.createElement("tr");
    const cornerTh = document.createElement("th");
    cornerTh.textContent = "Configuration";
    headerTr.appendChild(cornerTh);
    for (const p of params) {
      const th = document.createElement("th");
      th.textContent = p;
      headerTr.appendChild(th);
    }
    thead.appendChild(headerTr);
    table.appendChild(thead);

    const tbody = document.createElement("tbody");

    // Default row — read-only.
    {
      const tr = document.createElement("tr");
      tr.className = "dt-default-row";
      const lbl = document.createElement("td");
      lbl.className = "dt-row-label";
      lbl.textContent = "Default";
      tr.appendChild(lbl);
      for (const p of params) {
        const td = document.createElement("td");
        td.className = "dt-cell dt-readonly";
        const v = baseParams[p];
        const span = document.createElement("span");
        span.textContent = v == null || Number.isNaN(v) ? "" : String(v);
        td.appendChild(span);
        tr.appendChild(td);
      }
      tbody.appendChild(tr);
    }

    // Per-config rows — editable.
    for (const name of configNames) {
      const overlay = configs[name] ?? {};
      const tr = document.createElement("tr");
      const lbl = document.createElement("td");
      lbl.className = "dt-row-label";
      lbl.textContent = name;
      tr.appendChild(lbl);
      for (const p of params) {
        const td = document.createElement("td");
        td.className = "dt-cell";
        const input = document.createElement("input");
        input.type = "text";
        input.dataset.param = p;
        input.dataset.config = name;
        const v = overlay[p];
        input.value = v == null || Number.isNaN(v) ? "" : String(v);
        input.placeholder = baseParams[p] != null ? String(baseParams[p]) : "";
        input.addEventListener("change", () => {
          handleCellEdit(input, name, p);
        });
        td.appendChild(input);
        tr.appendChild(td);
      }
      tbody.appendChild(tr);
    }

    // Footer row with "+ Add config" button. Spans full width.
    {
      const tr = document.createElement("tr");
      tr.className = "dt-footer-row";
      const td = document.createElement("td");
      td.colSpan = params.length + 1;
      const btn = document.createElement("button");
      btn.className = "dt-add-config";
      btn.textContent = "+ Add config";
      btn.addEventListener("click", () => {
        addConfigPrompt();
      });
      td.appendChild(btn);
      tr.appendChild(td);
      tbody.appendChild(tr);
    }

    table.appendChild(tbody);
    body.appendChild(table);
  }

  function handleCellEdit(
    input: HTMLInputElement,
    configName: string,
    param: string,
  ) {
    const raw = input.value.trim();
    const model = parseModel(opts.getModel());
    if (!model) return;
    const configs = (model.configurations = model.configurations ?? {});
    const overlay = (configs[configName] = configs[configName] ?? {});

    if (raw === "") {
      // Blank → remove the override; the param falls back to base.
      delete overlay[param];
      input.classList.remove("invalid");
    } else {
      const num = Number(raw);
      if (!Number.isFinite(num)) {
        // Non-numeric → reject. Mark visually invalid and bail out
        // without persisting; the user keeps their bad text and can
        // fix it. setModel is NOT called.
        input.classList.add("invalid");
        return;
      }
      input.classList.remove("invalid");
      overlay[param] = num;
    }
    opts.setModel(JSON.stringify(model, null, 2));
  }

  function addConfigPrompt() {
    const name = window.prompt("New configuration name:");
    if (!name) return;
    const trimmed = name.trim();
    if (!trimmed) return;
    const model = parseModel(opts.getModel());
    if (!model) return;
    const configs = (model.configurations = model.configurations ?? {});
    if (configs[trimmed] != null) {
      window.alert(`A configuration named "${trimmed}" already exists.`);
      return;
    }
    configs[trimmed] = {};
    opts.setModel(JSON.stringify(model, null, 2));
    // Re-render so the new row appears immediately.
    renderTable();
  }

  function refresh() {
    // Always re-render the (possibly hidden) body so toggling open
    // shows fresh state. Cheap because the table is small.
    if (!body.hidden) renderTable();
  }

  // Initial render is deferred until first expand. Render now if the
  // host pre-expanded by clearing `hidden` on us.
  if (!body.hidden) renderTable();
  return { refresh };
}
