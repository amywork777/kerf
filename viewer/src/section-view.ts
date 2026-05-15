/**
 * section-view.ts — SolidWorks-style section view (clipping plane).
 *
 * Known limitation: the cut face is left visually hollow (no cap surface).
 * Implementing a filled cap requires CSG or stencil-buffer techniques that
 * are out of scope for this iteration.
 */

import * as THREE from "three";

export interface SectionViewOpts {
  getRenderer(): THREE.WebGLRenderer;
  getMeshMaterial(): THREE.Material;
  onChange(): void;
}

export interface ModelAabb {
  min: [number, number, number];
  max: [number, number, number];
}

export interface SectionViewHandle {
  /** Call after every mesh rebuild with the new AABB (or null if no model). */
  refresh(modelAabb: ModelAabb | null): void;
  /** Returns the active THREE.Plane, or null when section is off. */
  getActivePlane(): THREE.Plane | null;
}

type SectionMode = "off" | "X" | "Y" | "Z";
const MODES: SectionMode[] = ["off", "X", "Y", "Z"];

const AXIS_INDEX: Record<Exclude<SectionMode, "off">, 0 | 1 | 2> = {
  X: 0,
  Y: 1,
  Z: 2,
};

const AXIS_NORMAL: Record<Exclude<SectionMode, "off">, [number, number, number]> = {
  X: [1, 0, 0],
  Y: [0, 1, 0],
  Z: [0, 0, 1],
};

export function mountSectionView(
  host: HTMLElement,
  opts: SectionViewOpts,
): SectionViewHandle {
  // --- state ---
  let mode: SectionMode = "off";
  let direction: 1 | -1 = 1; // +1 clips positive half-space, -1 clips negative
  let currentAabb: ModelAabb | null = null;
  const plane = new THREE.Plane(new THREE.Vector3(1, 0, 0), 0);

  // --- DOM ---
  const container = document.createElement("div");
  container.id = "section-view";

  const label = document.createElement("label");
  label.htmlFor = "section-toggle";
  label.textContent = "Section view";

  const row = document.createElement("div");
  row.className = "sv-row";

  const toggleBtn = document.createElement("button");
  toggleBtn.id = "section-toggle";
  toggleBtn.textContent = "Section: off";

  const flipBtn = document.createElement("button");
  flipBtn.id = "section-flip";
  flipBtn.textContent = "▲▼";
  flipBtn.title = "Flip clip direction";
  flipBtn.hidden = true;

  row.appendChild(toggleBtn);
  row.appendChild(flipBtn);

  const slider = document.createElement("input");
  slider.id = "section-slider";
  slider.type = "range";
  slider.hidden = true;

  container.appendChild(label);
  container.appendChild(row);
  container.appendChild(slider);
  host.appendChild(container);

  // --- helpers ---
  function applyToMaterial() {
    const mat = opts.getMeshMaterial() as THREE.MeshStandardMaterial;
    if (mode === "off") {
      mat.clippingPlanes = [];
    } else {
      mat.clippingPlanes = [plane];
    }
    opts.onChange();
  }

  function updatePlaneFromSlider() {
    const offset = Number(slider.value);
    const [nx, ny, nz] = AXIS_NORMAL[mode as Exclude<SectionMode, "off">];
    plane.normal.set(nx * direction, ny * direction, nz * direction);
    plane.constant = -offset * direction;
  }

  function updateSliderRange() {
    if (!currentAabb || mode === "off") {
      slider.hidden = true;
      flipBtn.hidden = true;
      return;
    }
    slider.hidden = false;
    flipBtn.hidden = false;
    const axisIdx = AXIS_INDEX[mode as Exclude<SectionMode, "off">];
    const lo = currentAabb.min[axisIdx];
    const hi = currentAabb.max[axisIdx];
    const range = Math.abs(hi - lo) || 1;
    const step = range / 100;
    slider.min = String(lo);
    slider.max = String(hi);
    slider.step = String(step);
    // Position at midpoint by default.
    const mid = (lo + hi) / 2;
    if (Number(slider.value) < lo || Number(slider.value) > hi) {
      slider.value = String(mid);
    }
    updatePlaneFromSlider();
  }

  function syncUI() {
    if (mode === "off") {
      toggleBtn.textContent = "Section: off";
      slider.hidden = true;
      flipBtn.hidden = true;
    } else {
      toggleBtn.textContent = `Section: ${mode}`;
      slider.hidden = false;
      flipBtn.hidden = false;
      if (currentAabb !== null) {
        updateSliderRange();
      }
    }
  }

  // --- event listeners ---
  toggleBtn.addEventListener("click", () => {
    const idx = MODES.indexOf(mode);
    mode = MODES[(idx + 1) % MODES.length]!;
    direction = 1; // reset direction on mode change
    if (mode !== "off") {
      const [nx, ny, nz] = AXIS_NORMAL[mode];
      plane.normal.set(nx, ny, nz);
      plane.constant = 0;
    }
    syncUI();
    applyToMaterial();
  });

  flipBtn.addEventListener("click", () => {
    direction = direction === 1 ? -1 : 1;
    if (mode !== "off") {
      updatePlaneFromSlider();
    }
    applyToMaterial();
  });

  slider.addEventListener("input", () => {
    if (mode !== "off") {
      updatePlaneFromSlider();
    }
    applyToMaterial();
  });

  // --- public API ---
  return {
    refresh(modelAabb: ModelAabb | null) {
      currentAabb = modelAabb;
      updateSliderRange();
    },

    getActivePlane(): THREE.Plane | null {
      if (mode === "off") return null;
      return plane;
    },
  };
}
