// Dimensions panel: lets the user pick world-space points in the 3D scene
// and accumulate Linear / Radial / Angular dimension annotations, then
// download an annotated SVG via the kerf-cad-wasm `render_drawing_svg`
// export.
//
// Vanilla TS, no framework. The panel keeps its own state machine for the
// "pick points" mode; main.ts wires its onScenePick callback into the
// existing raycaster click handler so we receive the WORLD position of
// each click, not just a face id.

import * as THREE from "three";

export type DimKind = "linear" | "radial" | "angular";
export type ViewKind = "top" | "front" | "side" | "iso";

export interface LinearDim {
  kind: "linear";
  from: [number, number, number];
  to: [number, number, number];
}
export interface RadialDim {
  kind: "radial";
  from: [number, number, number];
  to: [number, number, number];
  center: [number, number, number];
}
export interface AngularDim {
  kind: "angular";
  from: [number, number, number];
  to: [number, number, number];
  vertex: [number, number, number];
}
export type Dimension = LinearDim | RadialDim | AngularDim;

/** How many world-space picks each dimension kind needs. */
export function picksRequired(kind: DimKind): number {
  return kind === "angular" ? 3 : 2;
}

/**
 * Build a wire-format Dimension object from a list of picked points and a
 * dimension kind. Convention:
 *   linear  — pts[0] → from, pts[1] → to
 *   radial  — pts[0] → center, pts[1] → to (point on circle); from := center
 *   angular — pts[0] → vertex, pts[1] → from, pts[2] → to
 *
 * Throws if the wrong number of points was supplied. Pure function — used
 * directly in tests so the picking flow's state machine can be verified
 * without a DOM.
 */
export function buildDimension(kind: DimKind, pts: THREE.Vector3[]): Dimension {
  const need = picksRequired(kind);
  if (pts.length !== need) {
    throw new Error(`${kind} dimension needs ${need} picks, got ${pts.length}`);
  }
  const v = (p: THREE.Vector3): [number, number, number] => [p.x, p.y, p.z];
  switch (kind) {
    case "linear":
      return { kind, from: v(pts[0]!), to: v(pts[1]!) };
    case "radial":
      return { kind, center: v(pts[0]!), from: v(pts[0]!), to: v(pts[1]!) };
    case "angular":
      return { kind, vertex: v(pts[0]!), from: v(pts[1]!), to: v(pts[2]!) };
  }
}

/**
 * Format a dimension for the live list. Pure function.
 */
export function describeDimension(d: Dimension): string {
  const f = (n: number) => n.toFixed(2);
  const p = (a: [number, number, number]) => `(${f(a[0])}, ${f(a[1])}, ${f(a[2])})`;
  switch (d.kind) {
    case "linear": {
      const dx = d.to[0] - d.from[0];
      const dy = d.to[1] - d.from[1];
      const dz = d.to[2] - d.from[2];
      const len = Math.hypot(dx, dy, dz);
      return `Linear  ${p(d.from)} → ${p(d.to)}  =  ${f(len)}`;
    }
    case "radial": {
      const dx = d.to[0] - d.center[0];
      const dy = d.to[1] - d.center[1];
      const dz = d.to[2] - d.center[2];
      const r = Math.hypot(dx, dy, dz);
      return `Radial  c=${p(d.center)}  R=${f(r)}`;
    }
    case "angular": {
      const ax = d.from[0] - d.vertex[0];
      const ay = d.from[1] - d.vertex[1];
      const az = d.from[2] - d.vertex[2];
      const bx = d.to[0] - d.vertex[0];
      const by = d.to[1] - d.vertex[1];
      const bz = d.to[2] - d.vertex[2];
      const an = Math.hypot(ax, ay, az);
      const bn = Math.hypot(bx, by, bz);
      let deg = 0;
      if (an > 1e-12 && bn > 1e-12) {
        const cos = Math.max(-1, Math.min(1, (ax * bx + ay * by + az * bz) / (an * bn)));
        deg = (Math.acos(cos) * 180) / Math.PI;
      }
      return `Angular v=${p(d.vertex)}  =  ${deg.toFixed(2)}°`;
    }
  }
}

/** Args passed to the WASM render_drawing_svg call. Exported for tests. */
export interface RenderArgs {
  json: string;
  targetId: string;
  paramsJson: string;
  view: ViewKind;
  dimensionsJson: string;
  viewportJson: string;
}

/**
 * Pure helper: pack the panel state into the exact arg tuple
 * `render_drawing_svg(json, target_id, params_json, view, dimensions_json, viewport_json)`
 * expects. Tested directly so the smoke test can mock the WASM call and
 * verify we're feeding it well-formed JSON.
 */
export function buildRenderArgs(opts: {
  json: string;
  targetId: string;
  parameters: Record<string, number>;
  view: ViewKind;
  dimensions: Dimension[];
  viewport?: { width?: number; height?: number; padding?: number };
}): RenderArgs {
  return {
    json: opts.json,
    targetId: opts.targetId,
    paramsJson: JSON.stringify(opts.parameters),
    view: opts.view,
    dimensionsJson: JSON.stringify(opts.dimensions),
    viewportJson: JSON.stringify(opts.viewport ?? {}),
  };
}

// ------------------------------------------------------------------
// Browser-side controller
// ------------------------------------------------------------------

export interface DimensionsPanelDeps {
  /** Returns the current model state, or null if no model loaded. */
  getModel: () => {
    json: string;
    targetId: string;
    parameters: Record<string, number>;
  } | null;
  /** WASM render fn — injected so tests can mock it. */
  renderSvg: (args: RenderArgs) => string;
  /** three.js scene to host point markers. */
  scene: THREE.Scene;
  /** Status setter (re-uses main.ts's ok/err). */
  setStatus: (msg: string, isError?: boolean) => void;
}

const MARKER_COLOR_PENDING = 0xffb84d;
const MARKER_COLOR_DONE = 0x58a6ff;

export class DimensionsPanel {
  private dims: Dimension[] = [];
  private picking = false;
  private pendingPicks: THREE.Vector3[] = [];
  private pendingMarkers: THREE.Object3D[] = [];
  private allMarkers: THREE.Object3D[] = [];

  private kindSel: HTMLSelectElement;
  private viewSel: HTMLSelectElement;
  private pickBtn: HTMLButtonElement;
  private clearBtn: HTMLButtonElement;
  private downloadBtn: HTMLButtonElement;
  private listEl: HTMLOListElement;
  private hintEl: HTMLElement;

  constructor(
    private root: HTMLElement,
    private deps: DimensionsPanelDeps,
  ) {
    this.kindSel = root.querySelector("#dim-kind") as HTMLSelectElement;
    this.viewSel = root.querySelector("#dim-view") as HTMLSelectElement;
    this.pickBtn = root.querySelector("#dim-pick-btn") as HTMLButtonElement;
    this.clearBtn = root.querySelector("#dim-clear-btn") as HTMLButtonElement;
    this.downloadBtn = root.querySelector("#dim-download-btn") as HTMLButtonElement;
    this.listEl = root.querySelector("#dim-list") as HTMLOListElement;
    this.hintEl = root.querySelector("#dim-hint") as HTMLElement;

    this.pickBtn.addEventListener("click", () => this.togglePicking());
    this.clearBtn.addEventListener("click", () => this.clear());
    this.downloadBtn.addEventListener("click", () => this.downloadSvg());
    this.kindSel.addEventListener("change", () => {
      // Reset partial picks if user switches modes mid-flight.
      if (this.picking) this.cancelPending();
      this.refreshHint();
    });
    this.refreshHint();
    this.refreshList();
  }

  /** Whether to swallow the next scene click (caller checks this). */
  isPicking(): boolean {
    return this.picking;
  }

  /**
   * main.ts forwards a 3D-scene click here when picking mode is on.
   * Pass the WORLD-space hit point (raycaster hits[0].point).
   */
  onScenePick(point: THREE.Vector3): void {
    if (!this.picking) return;
    const kind = this.kindSel.value as DimKind;
    const need = picksRequired(kind);
    this.pendingPicks.push(point.clone());
    this.addMarker(point, MARKER_COLOR_PENDING, this.pendingMarkers);
    this.refreshHint();
    if (this.pendingPicks.length >= need) {
      this.commitPending();
    }
  }

  private togglePicking() {
    if (this.picking) {
      this.cancelPending();
    } else {
      this.picking = true;
      this.pickBtn.textContent = "Cancel pick";
      this.pickBtn.classList.add("active");
    }
    this.refreshHint();
  }

  private cancelPending() {
    this.picking = false;
    this.pendingPicks = [];
    for (const m of this.pendingMarkers) this.deps.scene.remove(m);
    this.pendingMarkers = [];
    this.pickBtn.textContent = "Pick points";
    this.pickBtn.classList.remove("active");
    this.refreshHint();
  }

  private commitPending() {
    const kind = this.kindSel.value as DimKind;
    try {
      const dim = buildDimension(kind, this.pendingPicks);
      this.dims.push(dim);
      // Promote pending markers to permanent.
      for (const m of this.pendingMarkers) {
        ((m as THREE.Mesh).material as THREE.MeshBasicMaterial).color.setHex(
          MARKER_COLOR_DONE,
        );
        this.allMarkers.push(m);
      }
      this.pendingMarkers = [];
      this.pendingPicks = [];
      this.picking = false;
      this.pickBtn.textContent = "Pick points";
      this.pickBtn.classList.remove("active");
      this.refreshHint();
      this.refreshList();
      this.deps.setStatus(`added ${kind} dimension (${this.dims.length} total)`);
    } catch (e) {
      this.deps.setStatus(String(e), true);
      this.cancelPending();
    }
  }

  private addMarker(point: THREE.Vector3, color: number, bucket: THREE.Object3D[]) {
    const geom = new THREE.SphereGeometry(0.6, 12, 8);
    const mat = new THREE.MeshBasicMaterial({ color });
    const m = new THREE.Mesh(geom, mat);
    m.position.copy(point);
    this.deps.scene.add(m);
    bucket.push(m);
  }

  private removeAllMarkers() {
    for (const m of this.allMarkers) {
      this.deps.scene.remove(m);
      const mesh = m as THREE.Mesh;
      mesh.geometry.dispose();
      (mesh.material as THREE.Material).dispose();
    }
    this.allMarkers = [];
    for (const m of this.pendingMarkers) {
      this.deps.scene.remove(m);
      const mesh = m as THREE.Mesh;
      mesh.geometry.dispose();
      (mesh.material as THREE.Material).dispose();
    }
    this.pendingMarkers = [];
  }

  private clear() {
    this.dims = [];
    this.pendingPicks = [];
    this.removeAllMarkers();
    this.picking = false;
    this.pickBtn.textContent = "Pick points";
    this.pickBtn.classList.remove("active");
    this.refreshHint();
    this.refreshList();
    this.deps.setStatus("dimensions cleared");
  }

  private refreshHint() {
    const kind = this.kindSel.value as DimKind;
    const need = picksRequired(kind);
    if (this.picking) {
      const remaining = need - this.pendingPicks.length;
      this.hintEl.textContent = `Click ${remaining} more point${remaining === 1 ? "" : "s"} on the model.`;
    } else {
      this.hintEl.textContent = `Pick ${need} ${kind} point${need === 1 ? "" : "s"}.`;
    }
  }

  private refreshList() {
    this.listEl.innerHTML = "";
    this.dims.forEach((d, i) => {
      const li = document.createElement("li");
      const label = document.createElement("span");
      label.className = "dim-label";
      label.textContent = describeDimension(d);
      const del = document.createElement("span");
      del.className = "dim-del";
      del.title = "Remove this dimension";
      del.textContent = "✕";
      del.addEventListener("click", () => this.removeAt(i));
      li.appendChild(label);
      li.appendChild(del);
      this.listEl.appendChild(li);
    });
    this.downloadBtn.disabled = this.dims.length === 0;
  }

  private removeAt(idx: number) {
    this.dims.splice(idx, 1);
    this.refreshList();
    this.deps.setStatus(`removed dimension (${this.dims.length} remaining)`);
  }

  private downloadSvg() {
    const m = this.deps.getModel();
    if (!m) {
      this.deps.setStatus("no model loaded", true);
      return;
    }
    if (this.dims.length === 0) {
      this.deps.setStatus("add at least one dimension first", true);
      return;
    }
    const args = buildRenderArgs({
      json: m.json,
      targetId: m.targetId,
      parameters: m.parameters,
      view: this.viewSel.value as ViewKind,
      dimensions: this.dims,
    });
    try {
      const svg = this.deps.renderSvg(args);
      const blob = new Blob([svg], { type: "image/svg+xml" });
      const url = URL.createObjectURL(blob);
      const a = document.createElement("a");
      a.href = url;
      a.download = `${m.targetId}.${args.view}.dimensioned.svg`;
      document.body.appendChild(a);
      a.click();
      a.remove();
      URL.revokeObjectURL(url);
      this.deps.setStatus(`downloaded ${a.download}`);
    } catch (e) {
      this.deps.setStatus(`SVG render failed: ${e}`, true);
    }
  }

  /** Test/inspection hook: current dimension list. */
  getDimensions(): Dimension[] {
    return this.dims.slice();
  }
}
