/**
 * Unit tests for the BOM panel (bom.ts).
 *
 * @vitest-environment jsdom
 */
import { describe, it, expect, beforeEach } from "vitest";
import { mountBom, type BomEntry } from "./bom.js";

function makeEntry(overrides: Partial<BomEntry> = {}): BomEntry {
  return {
    name: "Part A",
    quantity: 1,
    material: "Steel",
    volume_each: 1000.0,
    volume_total: 1000.0,
    mass_each: 1.0,
    mass_total: 1.0,
    depth: 0,
    ...overrides,
  };
}

describe("BOM panel", () => {
  let host: HTMLDivElement;

  beforeEach(() => {
    host = document.createElement("div");
    document.body.appendChild(host);
  });

  it("renders a header on mount", () => {
    mountBom(host);
    const header = host.querySelector(".bom-header");
    expect(header).not.toBeNull();
    expect(header!.textContent).toContain("Bill of Materials");
  });

  it("update with 3 rows renders 3 table rows", () => {
    const panel = mountBom(host);

    // Expand the panel first (update works regardless)
    const header = host.querySelector(".bom-header") as HTMLElement;
    header.click();

    panel.update([
      makeEntry({ name: "Bracket", quantity: 2 }),
      makeEntry({ name: "Bolt M6", quantity: 16, depth: 0 }),
      makeEntry({ name: "Subassy", quantity: 1, depth: 1 }),
    ]);

    const rows = host.querySelectorAll(".bom-tbody tr");
    expect(rows.length).toBe(3);
  });

  it("update with null shows no-assembly placeholder", () => {
    const panel = mountBom(host);
    panel.update(null);

    const placeholder = host.querySelector(".bom-placeholder") as HTMLElement;
    expect(placeholder).not.toBeNull();
    expect(placeholder.hidden).toBe(false);

    const tableWrap = host.querySelector(".bom-table-wrap") as HTMLElement;
    expect(tableWrap.hidden).toBe(true);
  });

  it("depth > 0 applies indentation greater than depth 0", () => {
    const panel = mountBom(host);
    const header = host.querySelector(".bom-header") as HTMLElement;
    header.click();

    panel.update([
      makeEntry({ name: "Root Part", depth: 0 }),
      makeEntry({ name: "Sub Part", depth: 1 }),
    ]);

    const cells = host.querySelectorAll(".bom-col-name");
    // thead th + 2 tbody tds
    const rootCell = cells[1] as HTMLElement;
    const subCell = cells[2] as HTMLElement;

    const rootPadding = parseInt(rootCell.style.paddingLeft, 10);
    const subPadding = parseInt(subCell.style.paddingLeft, 10);
    expect(subPadding).toBeGreaterThan(rootPadding);
  });

  it("update with empty array shows no-assembly placeholder", () => {
    const panel = mountBom(host);
    panel.update([]);

    const placeholder = host.querySelector(".bom-placeholder") as HTMLElement;
    expect(placeholder.hidden).toBe(false);
  });
});
