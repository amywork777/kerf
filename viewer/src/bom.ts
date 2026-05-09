/**
 * BOM (Bill of Materials) panel — SolidWorks-style part table.
 *
 * Mount once with mountBom(host). Call update(rows) after every successful
 * rebuild. Pass null to show the "no assembly loaded" placeholder.
 */

export interface BomEntry {
  name: string;
  quantity: number;
  material: string;
  volume_each: number;
  volume_total: number;
  mass_each: number;
  mass_total: number;
  depth: number;
}

interface BomPanel {
  update(rows: BomEntry[] | null): void;
}

const INDENT_PX = 12;

function fmt(n: number, digits = 1): string {
  return n.toLocaleString("en-US", {
    minimumFractionDigits: digits,
    maximumFractionDigits: digits,
  });
}

export function mountBom(host: HTMLElement): BomPanel {
  // --- header (collapsible) ---
  const header = document.createElement("div");
  header.className = "bom-header";
  header.setAttribute("aria-expanded", "false");
  header.innerHTML = `<span class="bom-chevron">&#9654;</span><span>Bill of Materials</span>`;

  const body = document.createElement("div");
  body.className = "bom-body";
  body.hidden = true;

  // placeholder shown when no assembly is loaded
  const placeholder = document.createElement("p");
  placeholder.className = "bom-placeholder";
  placeholder.textContent =
    "No assembly loaded — BOM is for multi-part files.";
  body.appendChild(placeholder);

  // table container (hidden until rows arrive)
  const tableWrap = document.createElement("div");
  tableWrap.className = "bom-table-wrap";
  tableWrap.hidden = true;
  tableWrap.innerHTML = `
    <table class="bom-table">
      <thead>
        <tr>
          <th class="bom-col-name">Part</th>
          <th class="bom-col-num">Qty</th>
          <th class="bom-col-mat">Material</th>
          <th class="bom-col-num">Volume (mm³)</th>
          <th class="bom-col-num">Mass (g)</th>
        </tr>
      </thead>
      <tbody class="bom-tbody"></tbody>
    </table>`;
  body.appendChild(tableWrap);

  host.appendChild(header);
  host.appendChild(body);

  // toggle collapse on header click
  header.addEventListener("click", () => {
    const expanded = body.hidden;
    body.hidden = !expanded;
    header.setAttribute("aria-expanded", String(expanded));
    const chevron = header.querySelector(".bom-chevron") as HTMLElement;
    chevron.innerHTML = expanded ? "&#9660;" : "&#9654;";
  });

  function update(rows: BomEntry[] | null): void {
    const tbody = tableWrap.querySelector(
      ".bom-tbody"
    ) as HTMLTableSectionElement;
    tbody.innerHTML = "";

    if (!rows || rows.length === 0) {
      placeholder.hidden = false;
      tableWrap.hidden = true;
      return;
    }

    placeholder.hidden = true;
    tableWrap.hidden = false;

    for (const row of rows) {
      const tr = document.createElement("tr");
      tr.className = "bom-row";

      const nameCell = document.createElement("td");
      nameCell.className = "bom-col-name";
      nameCell.style.paddingLeft = `${8 + row.depth * INDENT_PX}px`;
      const prefix = row.depth > 0 ? "↳ " : "";
      nameCell.textContent = prefix + row.name;

      const qtyCell = document.createElement("td");
      qtyCell.className = "bom-col-num";
      qtyCell.textContent = String(row.quantity);

      const matCell = document.createElement("td");
      matCell.className = "bom-col-mat";
      matCell.textContent = row.material;

      const volCell = document.createElement("td");
      volCell.className = "bom-col-num";
      volCell.textContent = fmt(row.volume_total, 1);

      const massCell = document.createElement("td");
      massCell.className = "bom-col-num";
      massCell.textContent = fmt(row.mass_total, 2);

      tr.appendChild(nameCell);
      tr.appendChild(qtyCell);
      tr.appendChild(matCell);
      tr.appendChild(volCell);
      tr.appendChild(massCell);
      tbody.appendChild(tr);
    }
  }

  return { update };
}
