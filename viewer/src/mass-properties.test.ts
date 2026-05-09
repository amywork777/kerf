/**
 * @vitest-environment jsdom
 */
import { describe, it, expect, beforeEach } from "vitest";
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
    host = document.createElement("div");
    document.body.appendChild(host);
  });

  it("mounts the panel into the host element", () => {
    mountMassProperties(host);
    const panel = document.getElementById("mass-properties");
    expect(panel).not.toBeNull();
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
    panel.update(SAMPLE);
    expect(getById("mp-volume").textContent).toBe("12.345 mm³");
    expect(getById("mp-area").textContent).toBe("43.210 mm²");
  });

  it("update with sample data renders centroid", () => {
    const panel = mountMassProperties(host);
    panel.update(SAMPLE);
    const centroidText = getById("mp-centroid").textContent;
    expect(centroidText).toContain("1.234");
    expect(centroidText).toContain("2.345");
    expect(centroidText).toContain("3.456");
  });

  it("update with sample data renders bounding box deltas", () => {
    const panel = mountMassProperties(host);
    panel.update(SAMPLE);
    const bboxText = getById("mp-bbox").textContent;
    // Δx=10, Δy=5, Δz=8
    expect(bboxText).toContain("10.00");
    expect(bboxText).toContain("5.00");
    expect(bboxText).toContain("8.00");
  });

  it("update with sample data renders principal moments", () => {
    const panel = mountMassProperties(host);
    panel.update(SAMPLE);
    expect(getById("mp-i1").textContent).toBe("60.000");
    expect(getById("mp-i2").textContent).toBe("80.000");
    expect(getById("mp-i3").textContent).toBe("100.000");
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
