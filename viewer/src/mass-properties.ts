/**
 * Mass Properties panel — SolidWorks-style readout of volume, surface area,
 * centroid, bounding box, and principal moments of inertia.
 *
 * Usage:
 *   const panel = mountMassProperties(hostElement);
 *   // After a rebuild:
 *   panel.update(massPropertiesData);   // show values
 *   panel.update(null);                 // hide / clear values
 *
 * The panel header includes a unit-system toggle ("mm / g" | "in / lb").
 * The selection is persisted in viewer-session state (no Rust changes needed).
 */

import {
  type UnitSystem,
  type ValueKind,
  formatWithUnit,
  UNIT_SYSTEM_LABELS,
} from "./mass-units.js";

export interface MassPropertiesData {
  volume: number;
  surface_area: number;
  centroid: [number, number, number];
  inertia_tensor: [[number, number, number], [number, number, number], [number, number, number]];
  principal_moments: [number, number, number];
  principal_axes: [[number, number, number], [number, number, number], [number, number, number]];
  aabb_min: [number, number, number];
  aabb_max: [number, number, number];
}

export interface MassPropertiesPanel {
  update(data: MassPropertiesData | null): void;
  /** Expose current unit system for testing / external integration. */
  getUnitSystem(): UnitSystem;
  /** Set unit system programmatically (triggers display refresh). */
  setUnitSystem(system: UnitSystem): void;
}

/** Session-scoped last unit selection (survives panel remounts). */
let _sessionUnitSystem: UnitSystem = "metric";

/**
 * Format a (x, y, z) triple with the appropriate length unit.
 */
function fmtVec3(v: [number, number, number], system: UnitSystem): string {
  const fmt = (n: number) => formatWithUnit(n, "length", system);
  return `(${fmt(v[0])}, ${fmt(v[1])}, ${fmt(v[2])})`;
}

/**
 * Mount the mass properties panel inside `host`.
 *
 * The panel is collapsible (▸/▾ toggle). It starts collapsed.
 * Call `.update(data)` after each successful rebuild to refresh values.
 */
export function mountMassProperties(host: HTMLElement): MassPropertiesPanel {
  // --- Build DOM ---
  // The host is `<div id="mass-properties">` from index.html; the inner
  // wrapper avoids duplicating the id (which would shadow getElementById
  // lookups and produce invalid HTML).
  const container = document.createElement("div");
  container.className = "mp-panel";

  const header = document.createElement("div");
  header.className = "mp-header";

  const chevron = document.createElement("span");
  chevron.className = "mp-chevron";
  chevron.textContent = "▸";

  const title = document.createElement("span");
  title.className = "mp-title";
  title.textContent = "Mass Properties";

  // --- Unit toggle dropdown ---
  const unitLabel = document.createElement("label");
  unitLabel.className = "mp-unit-label";
  unitLabel.textContent = "Units:";

  const unitSelect = document.createElement("select");
  unitSelect.className = "mp-unit-select";
  unitSelect.id = "mp-unit-select";

  for (const [value, label] of Object.entries(UNIT_SYSTEM_LABELS) as [UnitSystem, string][]) {
    const opt = document.createElement("option");
    opt.value = value;
    opt.textContent = label;
    if (value === _sessionUnitSystem) opt.selected = true;
    unitSelect.appendChild(opt);
  }

  // Stop header collapse/expand from firing when interacting with the select.
  unitSelect.addEventListener("click", (e) => e.stopPropagation());
  unitSelect.addEventListener("change", () => {
    _sessionUnitSystem = unitSelect.value as UnitSystem;
    if (lastData) renderData(lastData);
  });

  unitLabel.appendChild(unitSelect);

  header.appendChild(chevron);
  header.appendChild(title);
  header.appendChild(unitLabel);

  const body = document.createElement("div");
  body.className = "mp-body";
  body.hidden = true;

  const table = document.createElement("table");
  table.className = "mp-table";
  body.appendChild(table);

  container.appendChild(header);
  container.appendChild(body);
  host.appendChild(container);

  // --- Collapse / expand ---
  header.addEventListener("click", () => {
    const expanded = !body.hidden;
    body.hidden = expanded;
    chevron.textContent = expanded ? "▸" : "▾";
  });

  // --- Row helpers ---
  function makeRow(label: string, valueId: string): HTMLTableRowElement {
    const tr = document.createElement("tr");
    const td1 = document.createElement("td");
    td1.className = "mp-label";
    td1.textContent = label;
    const td2 = document.createElement("td");
    td2.className = "mp-value";
    td2.id = valueId;
    td2.textContent = "—";
    tr.appendChild(td1);
    tr.appendChild(td2);
    return tr;
  }

  function makeSectionRow(label: string): HTMLTableRowElement {
    const tr = document.createElement("tr");
    const td = document.createElement("td");
    td.colSpan = 2;
    td.className = "mp-section";
    td.textContent = label;
    tr.appendChild(td);
    return tr;
  }

  // Build rows.
  const rowVolume    = makeRow("Volume:", "mp-volume");
  const rowArea      = makeRow("Surface area:", "mp-area");
  const rowCentroid  = makeRow("Centroid:", "mp-centroid");
  const rowBbox      = makeRow("Bounding box:", "mp-bbox");
  const rowMomSec    = makeSectionRow("Principal moments:");
  const rowI1        = makeRow("  I₁", "mp-i1");
  const rowI2        = makeRow("  I₂", "mp-i2");
  const rowI3        = makeRow("  I₃", "mp-i3");

  table.appendChild(rowVolume);
  table.appendChild(rowArea);
  table.appendChild(rowCentroid);
  table.appendChild(rowBbox);
  table.appendChild(rowMomSec);
  table.appendChild(rowI1);
  table.appendChild(rowI2);
  table.appendChild(rowI3);

  function setValue(id: string, text: string) {
    const el = document.getElementById(id);
    if (el) el.textContent = text;
  }

  function clearValues() {
    for (const id of ["mp-volume","mp-area","mp-centroid","mp-bbox","mp-i1","mp-i2","mp-i3"]) {
      setValue(id, "—");
    }
  }

  /** Last data snapshot — kept so unit-toggle re-renders without a rebuild. */
  let lastData: MassPropertiesData | null = null;

  function renderData(data: MassPropertiesData) {
    const sys = _sessionUnitSystem;

    const [xmin, ymin, zmin] = data.aabb_min;
    const [xmax, ymax, zmax] = data.aabb_max;
    const dx = xmax - xmin;
    const dy = ymax - ymin;
    const dz = zmax - zmin;

    setValue("mp-volume",   formatWithUnit(data.volume, "volume", sys));
    setValue("mp-area",     formatWithUnit(data.surface_area, "area", sys));
    setValue("mp-centroid", fmtVec3(data.centroid, sys));
    setValue("mp-bbox",
      `Δx ${formatWithUnit(dx, "length", sys)}  Δy ${formatWithUnit(dy, "length", sys)}  Δz ${formatWithUnit(dz, "length", sys)}`);
    // Principal moments are in g·mm²; leave as raw numbers with no unit
    // (unit conversion for inertia tensors would need mass input which we
    //  don't have separately — display as-is with 3 sig figs).
    const fmtI = (n: number) => {
      if (n === 0) return "0";
      const d = Math.ceil(Math.log10(Math.abs(n)));
      const mag = Math.pow(10, 3 - d);
      return String(Math.round(n * mag) / mag);
    };
    setValue("mp-i1", fmtI(data.principal_moments[0]));
    setValue("mp-i2", fmtI(data.principal_moments[1]));
    setValue("mp-i3", fmtI(data.principal_moments[2]));
  }

  return {
    update(data: MassPropertiesData | null) {
      lastData = data;
      if (!data) {
        clearValues();
        return;
      }
      renderData(data);
    },

    getUnitSystem(): UnitSystem {
      return _sessionUnitSystem;
    },

    setUnitSystem(system: UnitSystem) {
      _sessionUnitSystem = system;
      unitSelect.value = system;
      if (lastData) renderData(lastData);
    },
  };
}
