/**
 * @vitest-environment jsdom
 */
import { describe, it, expect, beforeEach, vi } from "vitest";
import * as THREE from "three";

// We import the module under test after mocking THREE.WebGLRenderer so
// that constructing a renderer in jsdom doesn't blow up on WebGL.
// The module itself only reads renderer.localClippingEnabled (set by
// main.ts) and calls material.clippingPlanes — both are plain properties.

import { mountSectionView } from "./section-view.js";

// ---------------------------------------------------------------------------
// Minimal stub for THREE.WebGLRenderer — only the properties section-view
// reads/writes are needed.
// ---------------------------------------------------------------------------
function makeFakeRenderer() {
  return {
    localClippingEnabled: false,
  } as unknown as THREE.WebGLRenderer;
}

function makeFakeMaterial() {
  const mat = new THREE.MeshStandardMaterial();
  return mat;
}

// ---------------------------------------------------------------------------
// Test helpers
// ---------------------------------------------------------------------------
function makeHost() {
  const el = document.createElement("div");
  document.body.appendChild(el);
  return el;
}

function makeOpts(onChange = vi.fn()) {
  const renderer = makeFakeRenderer();
  const material = makeFakeMaterial();
  return {
    opts: {
      getRenderer: () => renderer,
      getMeshMaterial: () => material,
      onChange,
    },
    renderer,
    material,
    onChange,
  };
}

// ---------------------------------------------------------------------------

describe("mountSectionView — DOM structure", () => {
  let host: HTMLElement;
  beforeEach(() => {
    document.body.innerHTML = "";
    host = makeHost();
  });

  it("mounts a toggle button inside the host", () => {
    const { opts } = makeOpts();
    mountSectionView(host, opts);
    const btn = host.querySelector<HTMLButtonElement>("#section-toggle");
    expect(btn).not.toBeNull();
    expect(btn!.textContent).toContain("Section");
  });

  it("mounts a slider that is initially hidden", () => {
    const { opts } = makeOpts();
    mountSectionView(host, opts);
    const slider = host.querySelector<HTMLInputElement>("#section-slider");
    expect(slider).not.toBeNull();
    expect(slider!.hidden).toBe(true);
  });

  it("mounts a flip-direction button that is initially hidden", () => {
    const { opts } = makeOpts();
    mountSectionView(host, opts);
    const flipBtn = host.querySelector<HTMLButtonElement>("#section-flip");
    expect(flipBtn).not.toBeNull();
    expect(flipBtn!.hidden).toBe(true);
  });
});

// ---------------------------------------------------------------------------

describe("mountSectionView — toggle cycles through modes", () => {
  let host: HTMLElement;
  beforeEach(() => {
    document.body.innerHTML = "";
    host = makeHost();
  });

  it("starts in 'off' mode (label shows 'off')", () => {
    const { opts } = makeOpts();
    mountSectionView(host, opts);
    const btn = host.querySelector<HTMLButtonElement>("#section-toggle")!;
    expect(btn.textContent!.toLowerCase()).toContain("off");
  });

  it("first click changes mode to X", () => {
    const { opts } = makeOpts();
    mountSectionView(host, opts);
    const btn = host.querySelector<HTMLButtonElement>("#section-toggle")!;
    btn.click();
    expect(btn.textContent).toContain("X");
  });

  it("second click changes mode to Y", () => {
    const { opts } = makeOpts();
    mountSectionView(host, opts);
    const btn = host.querySelector<HTMLButtonElement>("#section-toggle")!;
    btn.click(); // off → X
    btn.click(); // X → Y
    expect(btn.textContent).toContain("Y");
  });

  it("third click changes mode to Z", () => {
    const { opts } = makeOpts();
    mountSectionView(host, opts);
    const btn = host.querySelector<HTMLButtonElement>("#section-toggle")!;
    btn.click(); btn.click(); btn.click(); // off → X → Y → Z
    expect(btn.textContent).toContain("Z");
  });

  it("fourth click wraps back to off", () => {
    const { opts } = makeOpts();
    mountSectionView(host, opts);
    const btn = host.querySelector<HTMLButtonElement>("#section-toggle")!;
    btn.click(); btn.click(); btn.click(); btn.click(); // off→X→Y→Z→off
    expect(btn.textContent!.toLowerCase()).toContain("off");
  });

  it("slider becomes visible when section is enabled", () => {
    const { opts } = makeOpts();
    mountSectionView(host, opts);
    const btn = host.querySelector<HTMLButtonElement>("#section-toggle")!;
    const slider = host.querySelector<HTMLInputElement>("#section-slider")!;
    btn.click(); // enable X
    expect(slider.hidden).toBe(false);
  });

  it("slider is hidden again when section is turned off", () => {
    const { opts } = makeOpts();
    mountSectionView(host, opts);
    const btn = host.querySelector<HTMLButtonElement>("#section-toggle")!;
    const slider = host.querySelector<HTMLInputElement>("#section-slider")!;
    btn.click(); btn.click(); btn.click(); btn.click(); // full cycle → off
    expect(slider.hidden).toBe(true);
  });
});

// ---------------------------------------------------------------------------

describe("mountSectionView — getActivePlane()", () => {
  let host: HTMLElement;
  beforeEach(() => {
    document.body.innerHTML = "";
    host = makeHost();
  });

  it("returns null when section is off", () => {
    const { opts } = makeOpts();
    const sv = mountSectionView(host, opts);
    expect(sv.getActivePlane()).toBeNull();
  });

  it("returns a plane with normal (1,0,0) for X mode", () => {
    const { opts } = makeOpts();
    const sv = mountSectionView(host, opts);
    const btn = host.querySelector<HTMLButtonElement>("#section-toggle")!;
    btn.click(); // → X
    const plane = sv.getActivePlane();
    expect(plane).not.toBeNull();
    expect(plane!.normal.x).toBeCloseTo(1);
    expect(plane!.normal.y).toBeCloseTo(0);
    expect(plane!.normal.z).toBeCloseTo(0);
  });

  it("returns a plane with normal (0,1,0) for Y mode", () => {
    const { opts } = makeOpts();
    const sv = mountSectionView(host, opts);
    const btn = host.querySelector<HTMLButtonElement>("#section-toggle")!;
    btn.click(); btn.click(); // → Y
    const plane = sv.getActivePlane();
    expect(plane!.normal.y).toBeCloseTo(1);
  });

  it("returns a plane with normal (0,0,1) for Z mode", () => {
    const { opts } = makeOpts();
    const sv = mountSectionView(host, opts);
    const btn = host.querySelector<HTMLButtonElement>("#section-toggle")!;
    btn.click(); btn.click(); btn.click(); // → Z
    const plane = sv.getActivePlane();
    expect(plane!.normal.z).toBeCloseTo(1);
  });

  it("sets the clipping plane on the material when enabled", () => {
    const { opts, material } = makeOpts();
    const sv = mountSectionView(host, opts);
    const btn = host.querySelector<HTMLButtonElement>("#section-toggle")!;
    btn.click(); // → X
    const plane = sv.getActivePlane();
    expect(material.clippingPlanes).toContain(plane);
  });

  it("clears clipping planes from material when turned off", () => {
    const { opts, material } = makeOpts();
    const sv = mountSectionView(host, opts);
    const btn = host.querySelector<HTMLButtonElement>("#section-toggle")!;
    btn.click(); // → X
    btn.click(); btn.click(); btn.click(); // → off
    expect(material.clippingPlanes).toHaveLength(0);
  });
});

// ---------------------------------------------------------------------------

describe("mountSectionView — slider updates plane constant", () => {
  let host: HTMLElement;
  beforeEach(() => {
    document.body.innerHTML = "";
    host = makeHost();
  });

  it("slider value changes update the plane constant", () => {
    const { opts } = makeOpts();
    const sv = mountSectionView(host, opts);
    const btn = host.querySelector<HTMLButtonElement>("#section-toggle")!;
    btn.click(); // → X
    const slider = host.querySelector<HTMLInputElement>("#section-slider")!;
    // Set slider to a specific value and fire input event.
    slider.value = "25";
    slider.dispatchEvent(new Event("input"));
    const plane = sv.getActivePlane();
    expect(plane).not.toBeNull();
    // constant should reflect the slider position (negated because THREE
    // Plane uses -constant in the equation dot(n,x) + constant = 0).
    expect(plane!.constant).toBeCloseTo(-25);
  });

  it("onChange is called when slider changes", () => {
    const { opts, onChange } = makeOpts();
    mountSectionView(host, opts);
    const btn = host.querySelector<HTMLButtonElement>("#section-toggle")!;
    btn.click(); // → X
    const callsBefore = onChange.mock.calls.length;
    const slider = host.querySelector<HTMLInputElement>("#section-slider")!;
    slider.value = "10";
    slider.dispatchEvent(new Event("input"));
    expect(onChange.mock.calls.length).toBeGreaterThan(callsBefore);
  });
});

// ---------------------------------------------------------------------------

describe("mountSectionView — direction flip button", () => {
  let host: HTMLElement;
  beforeEach(() => {
    document.body.innerHTML = "";
    host = makeHost();
  });

  it("flip button negates the plane normal for X", () => {
    const { opts } = makeOpts();
    const sv = mountSectionView(host, opts);
    const btn = host.querySelector<HTMLButtonElement>("#section-toggle")!;
    btn.click(); // → X
    const flipBtn = host.querySelector<HTMLButtonElement>("#section-flip")!;
    flipBtn.click();
    const plane = sv.getActivePlane()!;
    expect(plane.normal.x).toBeCloseTo(-1);
  });

  it("flip button is visible when section is enabled", () => {
    const { opts } = makeOpts();
    mountSectionView(host, opts);
    const btn = host.querySelector<HTMLButtonElement>("#section-toggle")!;
    btn.click(); // → X
    const flipBtn = host.querySelector<HTMLButtonElement>("#section-flip")!;
    expect(flipBtn.hidden).toBe(false);
  });

  it("flipping twice restores original normal", () => {
    const { opts } = makeOpts();
    const sv = mountSectionView(host, opts);
    const btn = host.querySelector<HTMLButtonElement>("#section-toggle")!;
    btn.click(); // → X
    const flipBtn = host.querySelector<HTMLButtonElement>("#section-flip")!;
    flipBtn.click();
    flipBtn.click();
    expect(sv.getActivePlane()!.normal.x).toBeCloseTo(1);
  });
});

// ---------------------------------------------------------------------------

describe("mountSectionView — refresh(aabb)", () => {
  let host: HTMLElement;
  beforeEach(() => {
    document.body.innerHTML = "";
    host = makeHost();
  });

  it("refresh(null) hides the slider even if section is enabled", () => {
    const { opts } = makeOpts();
    const sv = mountSectionView(host, opts);
    const btn = host.querySelector<HTMLButtonElement>("#section-toggle")!;
    btn.click(); // enable X
    sv.refresh(null);
    const slider = host.querySelector<HTMLInputElement>("#section-slider")!;
    expect(slider.hidden).toBe(true);
  });

  it("refresh(aabb) shows the slider when section is on", () => {
    const { opts } = makeOpts();
    const sv = mountSectionView(host, opts);
    const btn = host.querySelector<HTMLButtonElement>("#section-toggle")!;
    btn.click(); // enable X
    sv.refresh({ min: [-10, -5, -2], max: [10, 5, 2] });
    const slider = host.querySelector<HTMLInputElement>("#section-slider")!;
    expect(slider.hidden).toBe(false);
  });

  it("refresh(aabb) sets slider min/max to AABB range of active axis (X)", () => {
    const { opts } = makeOpts();
    const sv = mountSectionView(host, opts);
    const btn = host.querySelector<HTMLButtonElement>("#section-toggle")!;
    btn.click(); // → X
    sv.refresh({ min: [-10, -5, -2], max: [10, 5, 2] });
    const slider = host.querySelector<HTMLInputElement>("#section-slider")!;
    expect(Number(slider.min)).toBeCloseTo(-10);
    expect(Number(slider.max)).toBeCloseTo(10);
  });

  it("refresh(aabb) sets slider min/max to AABB range of active axis (Y)", () => {
    const { opts } = makeOpts();
    const sv = mountSectionView(host, opts);
    const btn = host.querySelector<HTMLButtonElement>("#section-toggle")!;
    btn.click(); btn.click(); // → Y
    sv.refresh({ min: [-10, -5, -2], max: [10, 5, 2] });
    const slider = host.querySelector<HTMLInputElement>("#section-slider")!;
    expect(Number(slider.min)).toBeCloseTo(-5);
    expect(Number(slider.max)).toBeCloseTo(5);
  });

  it("refresh(aabb) step is ~1% of range", () => {
    const { opts } = makeOpts();
    const sv = mountSectionView(host, opts);
    const btn = host.querySelector<HTMLButtonElement>("#section-toggle")!;
    btn.click(); // → X (range = 20)
    sv.refresh({ min: [-10, 0, 0], max: [10, 0, 0] });
    const slider = host.querySelector<HTMLInputElement>("#section-slider")!;
    const step = Number(slider.step);
    expect(step).toBeCloseTo(0.2, 1); // 1% of 20 = 0.2
  });

  it("refresh(null) keeps slider hidden when section is off", () => {
    const { opts } = makeOpts();
    const sv = mountSectionView(host, opts);
    sv.refresh(null);
    const slider = host.querySelector<HTMLInputElement>("#section-slider")!;
    expect(slider.hidden).toBe(true);
  });
});
