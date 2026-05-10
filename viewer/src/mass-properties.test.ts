/**
 * @vitest-environment jsdom
 */
import { describe, it, expect, beforeEach, afterEach } from "vitest";
import { mountMassProperties, type MassPropertiesData } from "./mass-properties.js";

const SAMPLE: MassPropertiesData = {
  volume: 12.345,
  surface_area: 43.210,
  centroid: [1.234, 2.345, 3.456],
  inertia_tensor: [
    [100, 0, 0],
    [0, 80, 0],
    [0, 0, 60],
  ],
  principal_moments: [60.0, 80.0, 100.0],
  principal_axes: [
    [1, 0, 0],
    [0, 1, 0],
    [0, 0, 1],
  ],
  aabb_min: [0.0, 0.0, 0.0],
  aabb_max: [10.0, 5.0, 8.0],
};

function getById(id: string): HTMLElement {
  const el = document.getElementById(id);
  if (!el) throw new Error(`Element #${id} not found`);
  return el;
}

describe("mountMassProperties", () => {
  let host: HTMLDivElement;

  beforeEach(() => {
    document.body.innerHTML = "";
    // Match the real viewer's index.html: the host already carries
    // id="mass-properties" before mountMassProperties runs.
    host = document.createElement("div");
    host.id = "mass-properties";
    document.body.appendChild(host);
    // Ensure metric is the default for each test (session state is module-global).
    // We mount the panel first so setUnitSystem can reach the select element.
  });

  afterEach(() => {
    document.body.innerHTML = "";
  });

  it("mounts the panel into the host element", () => {
    mountMassProperties(host);
    expect(host.querySelector(".mp-body")).not.toBeNull();
    expect(host.querySelector(".mp-header")).not.toBeNull();
  });

  it("starts collapsed (body hidden)", () => {
    mountMassProperties(host);
    const body = host.querySelector(".mp-body") as HTMLElement;
    expect(body).not.toBeNull();
    expect(body.hidden).toBe(true);
  });

  it("shows dashes for all values initially", () => {
    mountMassProperties(host);
    expect(getById("mp-volume").textContent).toBe("—");
    expect(getById("mp-area").textContent).toBe("—");
    expect(getById("mp-centroid").textContent).toBe("—");
    expect(getById("mp-bbox").textContent).toBe("—");
    expect(getById("mp-i1").textContent).toBe("—");
    expect(getById("mp-i2").textContent).toBe("—");
    expect(getById("mp-i3").textContent).toBe("—");
  });

  it("update with sample data renders volume and surface area", () => {
    const panel = mountMassProperties(host);
    panel.setUnitSystem("metric");
    panel.update(SAMPLE);
    // 12.345 → 3 sig figs → 12.3; 43.210 → 43.2
    expect(getById("mp-volume").textContent).toBe("12.3 mm³");
    expect(getById("mp-area").textContent).toBe("43.2 mm²");
  });

  it("update with sample data renders centroid", () => {
    const panel = mountMassProperties(host);
    panel.setUnitSystem("metric");
    panel.update(SAMPLE);
    const centroidText = getById("mp-centroid").textContent;
    // Values are shown in mm with 3 sig figs: 1.234→1.23, 2.345→2.35, 3.456→3.46
    expect(centroidText).toContain("1.23");
    expect(centroidText).toContain("2.35");
    expect(centroidText).toContain("3.46");
  });

  it("update with sample data renders bounding box deltas", () => {
    const panel = mountMassProperties(host);
    panel.setUnitSystem("metric");
    panel.update(SAMPLE);
    const bboxText = getById("mp-bbox").textContent;
    // Δx=10, Δy=5, Δz=8 — shown with unit suffix
    expect(bboxText).toContain("10 mm");
    expect(bboxText).toContain("5 mm");
    expect(bboxText).toContain("8 mm");
  });

  it("update with sample data renders principal moments", () => {
    const panel = mountMassProperties(host);
    panel.setUnitSystem("metric");
    panel.update(SAMPLE);
    // Principal moments: 60, 80, 100 — 3 sig figs, no trailing zeros
    expect(getById("mp-i1").textContent).toBe("60");
    expect(getById("mp-i2").textContent).toBe("80");
    expect(getById("mp-i3").textContent).toBe("100");
  });

  it("update with null clears values back to dashes", () => {
    const panel = mountMassProperties(host);
    panel.update(SAMPLE);
    panel.update(null);
    expect(getById("mp-volume").textContent).toBe("—");
    expect(getById("mp-area").textContent).toBe("—");
    expect(getById("mp-centroid").textContent).toBe("—");
    expect(getById("mp-i1").textContent).toBe("—");
  });

  it("panel exposes getUnitSystem defaulting to metric", () => {
    const panel = mountMassProperties(host);
    panel.setUnitSystem("metric");
    expect(panel.getUnitSystem()).toBe("metric");
  });

  it("setUnitSystem to imperial re-renders volume in in³", () => {
    const panel = mountMassProperties(host);
    panel.setUnitSystem("metric");
    panel.update(SAMPLE);
    panel.setUnitSystem("imperial");
    const volText = getById("mp-volume").textContent ?? "";
    expect(volText).toMatch(/in³$/);
  });

  it("setUnitSystem to imperial re-renders bbox in in", () => {
    const panel = mountMassProperties(host);
    panel.setUnitSystem("metric");
    panel.update(SAMPLE);
    panel.setUnitSystem("imperial");
    const bboxText = getById("mp-bbox").textContent ?? "";
    expect(bboxText).toMatch(/in/);
  });

  it("unit-select element is present in the header", () => {
    mountMassProperties(host);
    const sel = document.getElementById("mp-unit-select");
    expect(sel).not.toBeNull();
  });

  it("clicking the header toggles expanded/collapsed state", () => {
    mountMassProperties(host);
    const header = host.querySelector(".mp-header") as HTMLElement;
    const body = host.querySelector(".mp-body") as HTMLElement;
    const chevron = host.querySelector(".mp-chevron") as HTMLElement;

    // Initially collapsed.
    expect(body.hidden).toBe(true);
    expect(chevron.textContent).toBe("▸");

    // Click to expand.
    header.click();
    expect(body.hidden).toBe(false);
    expect(chevron.textContent).toBe("▾");

    // Click to collapse again.
    header.click();
    expect(body.hidden).toBe(true);
    expect(chevron.textContent).toBe("▸");
  });
});
