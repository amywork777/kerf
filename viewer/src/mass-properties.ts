/**
 * Mass Properties panel — SolidWorks-style readout of volume, surface area,
 * centroid, bounding box, and principal moments of inertia.
 *
 * Usage:
 *   const panel = mountMassProperties(hostElement);
 *   // After a rebuild:
 *   panel.update(massPropertiesData);   // show values
 *   panel.update(null);                 // hide / clear values
 */

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
}

function fmt(n: number, decimals = 3): string {
  return n.toFixed(decimals);
}

function fmt3(v: [number, number, number], decimals = 3): string {
  return `(${fmt(v[0], decimals)}, ${fmt(v[1], decimals)}, ${fmt(v[2], decimals)})`;
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

  header.appendChild(chevron);
  header.appendChild(title);

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

  return {
    update(data: MassPropertiesData | null) {
      if (!data) {
        clearValues();
        return;
      }
      const [xmin, ymin, zmin] = data.aabb_min;
      const [xmax, ymax, zmax] = data.aabb_max;
      const dx = xmax - xmin;
      const dy = ymax - ymin;
      const dz = zmax - zmin;

      setValue("mp-volume",   `${fmt(data.volume)} mm³`);
      setValue("mp-area",     `${fmt(data.surface_area)} mm²`);
      setValue("mp-centroid", fmt3(data.centroid));
      setValue("mp-bbox",
        `Δx ${fmt(dx, 2)}  Δy ${fmt(dy, 2)}  Δz ${fmt(dz, 2)}`);
      setValue("mp-i1", fmt(data.principal_moments[0]));
      setValue("mp-i2", fmt(data.principal_moments[1]));
      setValue("mp-i3", fmt(data.principal_moments[2]));
    },
  };
}
