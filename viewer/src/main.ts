import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";

import init, {
  evaluate_to_mesh,
  evaluate_with_params,
  evaluate_with_face_ids,
  parameters_of,
  target_ids_of,
  feature_kinds,
} from "./wasm/kerf_cad_wasm.js";
import { exportThreeViewPng } from "./drawings.js";
import {
  newHistory,
  pushSnapshot,
  undo as historyUndo,
  redo as historyRedo,
  canUndo,
  canRedo,
  type Snapshot,
} from "./state.js";
import { mountErrorOverlay, extractFeatureIdFromError } from "./overlay.js";
import {
  groupByCategory,
  searchKinds,
  defaultInstance,
  type Category,
} from "./catalog.js";

await init();

const stage = document.getElementById("stage")!;
const status = document.getElementById("status")!;
const fileInput = document.getElementById("file") as HTMLInputElement;
const dropZone = document.getElementById("drop")!;
const paramsEl = document.getElementById("params")!;
const targetsEl = document.getElementById("targets")!;
const targetSelect = document.getElementById("target-select") as HTMLSelectElement;
const actionsEl = document.getElementById("actions")!;
const viewsEl = document.getElementById("views")!;
const downloadBtn = document.getElementById("download-btn")!;
const resetBtn = document.getElementById("reset-btn")!;
const wireframeToggle = document.getElementById("wireframe-toggle") as HTMLInputElement;
const drawingBtn = document.getElementById("drawing-btn")!;
const actions2El = document.getElementById("actions2")!;
const featureTreeEl = document.getElementById("feature-tree")!;
const featureListEl = document.getElementById("feature-list")!;
const featureCountEl = document.getElementById("feature-count")!;
const selectedFeatureEl = document.getElementById("selected-feature")!;
const selectedFeatureBodyEl = document.getElementById("selected-feature-body")!;
const selectedFeatureTitleEl = document.getElementById("selected-feature-title")!;
const undoBtn = document.getElementById("undo-btn") as HTMLButtonElement;
const redoBtn = document.getElementById("redo-btn") as HTMLButtonElement;
const historyActionsEl = document.getElementById("history-actions")!;
const catalogEl = document.getElementById("catalog")!;
const catalogToggleEl = document.getElementById("catalog-toggle")!;
const catalogToggleLabel = document.getElementById("catalog-toggle-label")!;
const catalogBodyEl = document.getElementById("catalog-body")!;
const catalogSearchEl = document.getElementById("catalog-search") as HTMLInputElement;
const catalogListEl = document.getElementById("catalog-list")!;
selectedFeatureEl.querySelector(".sf-close")?.addEventListener("click", () => {
  selectedFeatureId = null;
  highlightedFace = -1;
  refreshFaceColors();
  renderSelectedFeaturePanel();
});

// --- three.js scene ---
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x0d1116);

const camera = new THREE.PerspectiveCamera(45, 1, 0.1, 5000);
camera.position.set(80, 60, 100);

const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(window.devicePixelRatio);
stage.appendChild(renderer.domElement);

const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.dampingFactor = 0.08;

scene.add(new THREE.AmbientLight(0x404448, 1.0));
const key = new THREE.DirectionalLight(0xffffff, 1.6);
key.position.set(60, 100, 80);
scene.add(key);
const fill = new THREE.DirectionalLight(0xa8c5ff, 0.5);
fill.position.set(-80, -40, -60);
scene.add(fill);

const grid = new THREE.GridHelper(200, 20, 0x222933, 0x1a1f26);
(grid.material as THREE.Material).transparent = true;
(grid.material as THREE.Material).opacity = 0.6;
scene.add(grid);

const meshMaterial = new THREE.MeshStandardMaterial({
  color: 0xffffff,
  metalness: 0.2,
  roughness: 0.55,
  flatShading: true,
  side: THREE.DoubleSide,
  vertexColors: true,
});

let currentMesh: THREE.Mesh | null = null;
let currentWireframe: THREE.LineSegments | null = null;
let currentFaceIds: Uint32Array | null = null;
let currentFaceOwnerTags: string[] | null = null;
let currentFaceCount = 0;
let highlightedFace = -1;
let hoveredFace = -1;
let selectedFeatureId: string | null = null;

function fitToView(box: THREE.Box3) {
  const center = box.getCenter(new THREE.Vector3());
  const size = box.getSize(new THREE.Vector3()).length() || 50;
  controls.target.copy(center);
  const dir = new THREE.Vector3(1, 0.7, 1.2).normalize();
  camera.position.copy(center.clone().add(dir.multiplyScalar(size * 1.5)));
  camera.near = Math.max(0.01, size / 1000);
  camera.far = size * 200;
  camera.updateProjectionMatrix();
  controls.update();
}

function setMesh(
  triangles: Float32Array,
  faceIds: Uint32Array,
  faceOwnerTags: string[],
  faceCount: number,
  fit: boolean,
) {
  if (currentMesh) {
    scene.remove(currentMesh);
    currentMesh.geometry.dispose();
    currentMesh = null;
  }
  if (currentWireframe) {
    scene.remove(currentWireframe);
    currentWireframe.geometry.dispose();
    currentWireframe = null;
  }
  currentFaceIds = faceIds;
  currentFaceOwnerTags = faceOwnerTags;
  currentFaceCount = faceCount;
  highlightedFace = -1;
  hoveredFace = -1;

  if (triangles.length === 0) return;

  const geom = new THREE.BufferGeometry();
  geom.setAttribute("position", new THREE.BufferAttribute(triangles, 3));
  geom.computeVertexNormals();
  geom.computeBoundingBox();

  // Per-vertex color buffer used to highlight picked faces by tinting their
  // triangles. Three.js chooses the vertex shader path that multiplies
  // vertexColor with material.color.
  const colors = new Float32Array(triangles.length); // 3 floats per vertex
  resetColors(colors, faceIds);
  geom.setAttribute("color", new THREE.BufferAttribute(colors, 3));

  currentMesh = new THREE.Mesh(geom, meshMaterial);
  scene.add(currentMesh);
  refreshWireframe();
  if (fit && geom.boundingBox) fitToView(geom.boundingBox);
}

const BASE_COLOR = new THREE.Color(0x6ea8ff);
const HIGHLIGHT_COLOR = new THREE.Color(0xffb84d);
const HOVER_COLOR = new THREE.Color(0xa8c7ff);

function resetColors(colors: Float32Array, faceIds: Uint32Array) {
  for (let tri = 0; tri < faceIds.length; tri++) {
    const c = colorForFace(faceIds[tri]!);
    for (let v = 0; v < 3; v++) {
      const off = (tri * 3 + v) * 3;
      colors[off] = c.r;
      colors[off + 1] = c.g;
      colors[off + 2] = c.b;
    }
  }
}

function colorForFace(faceId: number): THREE.Color {
  if (faceId === highlightedFace) return HIGHLIGHT_COLOR;
  if (faceId === hoveredFace) return HOVER_COLOR;
  return BASE_COLOR;
}

function refreshFaceColors() {
  if (!currentMesh || !currentFaceIds) return;
  const attr = currentMesh.geometry.getAttribute("color") as THREE.BufferAttribute;
  resetColors(attr.array as Float32Array, currentFaceIds);
  attr.needsUpdate = true;
}

function refreshWireframe() {
  if (currentWireframe) {
    scene.remove(currentWireframe);
    currentWireframe.geometry.dispose();
    currentWireframe = null;
  }
  if (!currentMesh || !wireframeToggle.checked) return;
  const wf = new THREE.WireframeGeometry(currentMesh.geometry);
  const mat = new THREE.LineBasicMaterial({ color: 0xffffff, transparent: true, opacity: 0.25 });
  currentWireframe = new THREE.LineSegments(wf, mat);
  scene.add(currentWireframe);
}

wireframeToggle.addEventListener("change", refreshWireframe);

function resize() {
  const r = stage.getBoundingClientRect();
  renderer.setSize(r.width, r.height, false);
  camera.aspect = r.width / Math.max(1, r.height);
  camera.updateProjectionMatrix();
}
resize();
window.addEventListener("resize", resize);

// --- raycast picking ---
const raycaster = new THREE.Raycaster();
const ndc = new THREE.Vector2();

function pickFaceAt(clientX: number, clientY: number): number {
  if (!currentMesh || !currentFaceIds) return -1;
  const rect = renderer.domElement.getBoundingClientRect();
  ndc.x = ((clientX - rect.left) / rect.width) * 2 - 1;
  ndc.y = -((clientY - rect.top) / rect.height) * 2 + 1;
  raycaster.setFromCamera(ndc, camera);
  const hits = raycaster.intersectObject(currentMesh, false);
  if (hits.length === 0 || hits[0]!.faceIndex === undefined) return -1;
  const tri = hits[0]!.faceIndex!;
  return tri < currentFaceIds.length ? currentFaceIds[tri]! : -1;
}

renderer.domElement.addEventListener("click", (e) => {
  const fid = pickFaceAt(e.clientX, e.clientY);
  highlightedFace = fid;
  refreshFaceColors();
  if (fid >= 0) {
    const owner = currentFaceOwnerTags?.[fid] ?? "";
    if (owner) {
      selectedFeatureId = owner;
      renderSelectedFeaturePanel();
      ok(`selected face #${fid} → owner '${owner}'`);
    } else {
      ok(`selected face #${fid} (no owner tag)`);
    }
  } else {
    selectedFeatureId = null;
    renderSelectedFeaturePanel();
  }
});

renderer.domElement.addEventListener("pointermove", (e) => {
  // Throttle hover detection by skipping when no model.
  if (!currentMesh) return;
  const fid = pickFaceAt(e.clientX, e.clientY);
  if (fid !== hoveredFace) {
    hoveredFace = fid;
    refreshFaceColors();
  }
});

function tick() {
  controls.update();
  renderer.render(scene, camera);
  requestAnimationFrame(tick);
}
tick();

// --- model state ---
type ModelState = {
  json: string;
  targetId: string;
  parameters: Record<string, number>;
  defaults: Record<string, number>;
};
let model: ModelState | null = null;
const history = newHistory();
const errorOverlay = mountErrorOverlay(stage);

const SEGMENTS = 24;

function snapshotOf(m: ModelState): Snapshot {
  return {
    json: m.json,
    targetId: m.targetId,
    parameters: { ...m.parameters },
  };
}

/**
 * Capture the current model state on the undo stack *before* mutating
 * `model`. Caller must call this *before* changing any field. No-op if
 * we have no model loaded yet.
 */
function recordHistory() {
  if (!model) return;
  pushSnapshot(history, snapshotOf(model));
  refreshHistoryButtons();
}

function refreshHistoryButtons() {
  undoBtn.disabled = !canUndo(history);
  redoBtn.disabled = !canRedo(history);
}

function applySnapshot(snap: Snapshot) {
  if (!model) return;
  model.json = snap.json;
  model.targetId = snap.targetId;
  model.parameters = { ...snap.parameters };
  // `defaults` is keyed by the originally-loaded parameters, so we
  // intentionally don't overwrite them — they survive undo/redo.
  const ids = parsedFeatureIds(model.json);
  renderTargets(ids);
  renderParams();
  renderFeatureTree();
  renderSelectedFeaturePanel();
  rebuild();
}

function parsedFeatureIds(json: string): string[] {
  try {
    const parsed = JSON.parse(json);
    return ((parsed?.features ?? []) as Array<{ id?: string }>)
      .map((f) => f?.id)
      .filter((x): x is string => typeof x === "string");
  } catch {
    return [];
  }
}

function ok(msg: string) {
  status.classList.remove("error");
  status.textContent = msg;
  errorOverlay.hide();
}
function err(msg: string) {
  status.classList.add("error");
  status.textContent = msg;
  // Promote evaluator errors to a visible overlay banner so the user
  // can't miss them when they're scrubbing a slider that just blew up
  // the boolean engine.
  const featId = extractFeatureIdFromError(msg);
  errorOverlay.show(msg, featId);
}

function rebuild(fit: boolean = false) {
  if (!model) return;
  try {
    const t0 = performance.now();
    const result = evaluate_with_face_ids(
      model.json,
      model.targetId,
      JSON.stringify(model.parameters),
      SEGMENTS,
    ) as {
      triangles: number[];
      face_ids: number[];
      face_owner_tags: string[];
      face_count: number;
      volume: number;
      shell_count: number;
      vertex_count: number;
      edge_count: number;
      face_count_topo: number;
    };
    const tris = new Float32Array(result.triangles);
    const faceIds = new Uint32Array(result.face_ids);
    const ownerTags = result.face_owner_tags ?? [];
    const dt = performance.now() - t0;
    setMesh(tris, faceIds, ownerTags, result.face_count, fit);
    ok(
      `target='${model.targetId}'  V/E/F/S=${result.vertex_count}/${result.edge_count}/${result.face_count_topo}/${result.shell_count}` +
        `  vol=${result.volume.toFixed(3)}  tris=${tris.length / 9}  eval=${dt.toFixed(1)}ms`,
    );
  } catch (e) {
    err(String(e));
  }
}

function renderParams() {
  paramsEl.innerHTML = "";
  if (!model) return;
  const keys = Object.keys(model.defaults).sort();
  for (const k of keys) {
    const def = model.defaults[k]!;
    const cur = model.parameters[k] ?? def;
    // Slider range: ±2x of |def| (or ±10 if def is 0)
    const span = Math.max(Math.abs(def) * 2, 10);
    const min = def === 0 ? -span : Math.min(def - span, def * -0.2);
    const max = def === 0 ? span : Math.max(def + span, def * 2);
    const step = Math.max((max - min) / 200, 0.001);

    const row = document.createElement("div");
    row.className = "row";
    row.innerHTML = `
      <label><span>${k}</span><span class="val" data-name="${k}">${cur.toFixed(3)}</span></label>
      <input type="range" min="${min}" max="${max}" step="${step}" value="${cur}" data-name="${k}" />`;
    paramsEl.appendChild(row);
    const input = row.querySelector("input")!;
    const valSpan = row.querySelector(".val") as HTMLElement;
    let pristine = true;
    input.addEventListener("input", () => {
      // Snapshot once at the *start* of a slider drag, not on every
      // intermediate `input` event — otherwise the undo stack fills up
      // with one entry per pixel of mouse movement.
      if (pristine) {
        recordHistory();
        pristine = false;
      }
      const v = Number(input.value);
      model!.parameters[k] = v;
      valSpan.textContent = v.toFixed(3);
      rebuild();
    });
    input.addEventListener("change", () => {
      pristine = true;
    });
  }
}

function renderTargets(ids: string[]) {
  targetsEl.hidden = ids.length === 0;
  targetSelect.innerHTML = "";
  for (const id of ids) {
    const opt = document.createElement("option");
    opt.value = id;
    opt.textContent = id;
    targetSelect.appendChild(opt);
  }
  if (model) targetSelect.value = model.targetId;
  targetSelect.onchange = () => {
    if (!model) return;
    recordHistory();
    model.targetId = targetSelect.value;
    refreshFeatureTreeSelection();
    rebuild();
  };
}

type FeatureSummary = { id: string; kind: string };

function summarizeFeatures(json: string): FeatureSummary[] {
  try {
    const parsed = JSON.parse(json);
    const features = (parsed?.features ?? []) as Array<Record<string, unknown>>;
    return features
      .filter((f): f is Record<string, unknown> => typeof f === "object" && f !== null)
      .map((f) => ({
        id: typeof f.id === "string" ? f.id : "?",
        kind: typeof f.kind === "string" ? f.kind : "?",
      }));
  } catch {
    return [];
  }
}

function renderFeatureTree() {
  if (!model) {
    featureTreeEl.hidden = true;
    return;
  }
  const features = summarizeFeatures(model.json);
  featureTreeEl.hidden = features.length === 0;
  featureCountEl.textContent = features.length ? `(${features.length})` : "";
  featureListEl.innerHTML = "";
  for (const f of features) {
    const li = document.createElement("li");
    li.dataset.id = f.id;
    li.innerHTML = `<span class="kind">${f.kind}</span><span class="id">${f.id}</span><span class="del" title="Delete this feature">✕</span>`;
    (li.querySelector(".id") as HTMLElement).title = `Click to set as target — current: '${f.id}'`;
    li.querySelector(".id")!.addEventListener("click", () => {
      if (!model) return;
      model.targetId = f.id;
      targetSelect.value = f.id;
      refreshFeatureTreeSelection();
      rebuild();
    });
    li.querySelector(".del")!.addEventListener("click", (e) => {
      e.stopPropagation();
      deleteFeature(f.id);
    });
    featureListEl.appendChild(li);
  }
  refreshFeatureTreeSelection();
}

function deleteFeature(id: string) {
  if (!model) return;
  const parsed = JSON.parse(model.json);
  const before = (parsed.features ?? []).length;
  const next = (parsed.features ?? []).filter(
    (f: { id?: string }) => f?.id !== id,
  );
  const removed = before - next.length;
  if (removed === 0) return;
  recordHistory();
  parsed.features = next;
  // If we just deleted the current target, fall back to the last remaining feature.
  const remainingIds = parsed.features
    .map((f: { id?: string }) => f.id)
    .filter((x: unknown): x is string => typeof x === "string");
  if (remainingIds.length === 0) {
    err(`would delete last feature — refusing`);
    return;
  }
  const newJson = JSON.stringify(parsed, null, 2);
  const nextTarget =
    remainingIds.includes(model.targetId)
      ? model.targetId
      : remainingIds.includes("out")
        ? "out"
        : remainingIds[remainingIds.length - 1];
  model = {
    ...model,
    json: newJson,
    targetId: nextTarget,
  };
  renderTargets(remainingIds);
  renderFeatureTree();
  rebuild();
}

function renderSelectedFeaturePanel() {
  if (!model || !selectedFeatureId) {
    selectedFeatureEl.hidden = true;
    return;
  }
  const parsed = JSON.parse(model.json);
  const features = (parsed.features ?? []) as Array<Record<string, unknown>>;
  const feat = features.find((f) => f?.id === selectedFeatureId);
  if (!feat) {
    selectedFeatureEl.hidden = true;
    return;
  }
  selectedFeatureEl.hidden = false;
  selectedFeatureTitleEl.textContent = `${feat.kind} • ${feat.id}`;
  selectedFeatureBodyEl.innerHTML = "";

  const SKIP = new Set(["kind", "id", "input", "inputs"]);
  for (const [k, v] of Object.entries(feat)) {
    if (SKIP.has(k)) continue;
    if (typeof v === "number") {
      addNumericRow(k, v, [k]);
    } else if (Array.isArray(v) && v.every((x) => typeof x === "number" || typeof x === "string")) {
      for (let i = 0; i < v.length; i++) {
        const elem = v[i];
        if (typeof elem === "number") {
          addNumericRow(`${k}[${i}]`, elem, [k, i]);
        } else {
          addExpressionRow(`${k}[${i}]`, String(elem), [k, i]);
        }
      }
    } else if (typeof v === "string") {
      addExpressionRow(k, v, [k]);
    }
  }

  if (selectedFeatureBodyEl.children.length === 0) {
    const empty = document.createElement("div");
    empty.className = "sf-empty";
    empty.textContent = "no editable fields";
    selectedFeatureBodyEl.appendChild(empty);
  }
}

function addNumericRow(label: string, value: number, path: (string | number)[]) {
  const row = document.createElement("div");
  row.className = "sf-row";
  const span = Math.max(Math.abs(value) * 2, 10);
  const min = value === 0 ? -span : Math.min(value - span, value * -0.2);
  const max = value === 0 ? span : Math.max(value + span, value * 2);
  const step = Math.max((max - min) / 200, 0.001);
  row.innerHTML = `
    <label><span>${label}</span><span class="sf-val">${value.toFixed(3)}</span></label>
    <input type="range" min="${min}" max="${max}" step="${step}" value="${value}" />`;
  selectedFeatureBodyEl.appendChild(row);
  const input = row.querySelector("input")!;
  const valSpan = row.querySelector(".sf-val") as HTMLElement;
  let pristine = true;
  input.addEventListener("input", () => {
    if (pristine) {
      recordHistory();
      pristine = false;
    }
    const n = Number(input.value);
    valSpan.textContent = n.toFixed(3);
    setFeatureFieldAtPath(path, n);
  });
  input.addEventListener("change", () => {
    pristine = true;
  });
}

function addExpressionRow(label: string, value: string, path: (string | number)[]) {
  const row = document.createElement("div");
  row.className = "sf-row";
  row.innerHTML = `
    <label><span>${label}</span><span class="sf-muted">expr</span></label>
    <input type="text" value="${value.replace(/"/g, "&quot;")}" />`;
  selectedFeatureBodyEl.appendChild(row);
  const input = row.querySelector("input")!;
  input.addEventListener("change", () => {
    recordHistory();
    const raw = input.value;
    const asNum = Number(raw);
    setFeatureFieldAtPath(path, Number.isFinite(asNum) && raw.trim() !== "" && !raw.startsWith("$") && !/[a-zA-Z]/.test(raw) ? asNum : raw);
  });
}

function setFeatureFieldAtPath(path: (string | number)[], value: number | string) {
  if (!model || !selectedFeatureId) return;
  const parsed = JSON.parse(model.json);
  const features = (parsed.features ?? []) as Array<Record<string, unknown>>;
  const feat = features.find((f) => f?.id === selectedFeatureId);
  if (!feat) return;
  let cursor: unknown = feat;
  for (let i = 0; i < path.length - 1; i++) {
    cursor = (cursor as Record<string | number, unknown>)[path[i]!];
  }
  const last = path[path.length - 1]!;
  (cursor as Record<string | number, unknown>)[last] = value;
  model.json = JSON.stringify(parsed, null, 2);
  rebuild();
}

function refreshFeatureTreeSelection() {
  if (!model) return;
  for (const li of featureListEl.children as HTMLCollectionOf<HTMLLIElement>) {
    li.classList.toggle("target", li.dataset.id === model.targetId);
  }
}

function loadJson(json: string) {
  try {
    const ids = target_ids_of(json) as string[];
    if (ids.length === 0) {
      err("model has no features");
      return;
    }
    const params = (parameters_of(json) ?? {}) as Record<string, number>;
    // Prefer a target named "out" if present, else last id.
    const targetId = ids.includes("out") ? "out" : ids[ids.length - 1]!;
    model = {
      json,
      targetId,
      parameters: { ...params },
      defaults: { ...params },
    };
    selectedFeatureId = null;
    // A fresh load clears history — undo across model loads would be
    // confusing (the snapshot's parameter map wouldn't match the new
    // model's defaults).
    history.undoStack.length = 0;
    history.redoStack.length = 0;
    refreshHistoryButtons();
    renderTargets(ids);
    renderParams();
    renderFeatureTree();
    renderSelectedFeaturePanel();
    actionsEl.hidden = false;
    actions2El.hidden = false;
    viewsEl.hidden = false;
    historyActionsEl.hidden = false;
    catalogEl.hidden = false;
    rebuild(true);
  } catch (e) {
    err(String(e));
  }
}

// --- save back to JSON ---
downloadBtn.addEventListener("click", () => {
  if (!model) return;
  // Re-emit JSON with the slider-modified parameters baked in.
  const parsed = JSON.parse(model.json);
  parsed.parameters = { ...model.parameters };
  const out = JSON.stringify(parsed, null, 2);
  const blob = new Blob([out], { type: "application/json" });
  const url = URL.createObjectURL(blob);
  const a = document.createElement("a");
  a.href = url;
  a.download = "model.kerf-cad.json";
  document.body.appendChild(a);
  a.click();
  a.remove();
  URL.revokeObjectURL(url);
});

resetBtn.addEventListener("click", () => {
  if (!model) return;
  recordHistory();
  model.parameters = { ...model.defaults };
  renderParams();
  rebuild();
});

drawingBtn.addEventListener("click", () => {
  if (!model || !currentMesh) return;
  exportThreeViewPng(currentMesh, model.targetId);
});

// --- view presets ---
function setView(kind: "iso" | "front" | "top" | "side") {
  if (!currentMesh?.geometry.boundingBox) return;
  const box = currentMesh.geometry.boundingBox;
  const center = box.getCenter(new THREE.Vector3());
  const size = box.getSize(new THREE.Vector3()).length() || 50;
  const dist = size * 1.6;
  const dir = {
    iso: new THREE.Vector3(1, 0.7, 1.2).normalize(),
    front: new THREE.Vector3(0, 0, 1),
    top: new THREE.Vector3(0, 1, 0.001), // tiny z so OrbitControls doesn't lock up
    side: new THREE.Vector3(1, 0, 0),
  }[kind];
  controls.target.copy(center);
  camera.position.copy(center.clone().add(dir.multiplyScalar(dist)));
  controls.update();
}
viewsEl.querySelectorAll<HTMLButtonElement>("button[data-view]").forEach((b) => {
  b.addEventListener("click", () => setView(b.dataset.view as any));
});

// --- undo / redo ---
function performUndo() {
  if (!model || !canUndo(history)) return;
  const restored = historyUndo(history, snapshotOf(model));
  if (!restored) return;
  applySnapshot(restored);
  refreshHistoryButtons();
  ok(`undo (${history.undoStack.length} earlier · ${history.redoStack.length} ahead)`);
}

function performRedo() {
  if (!model || !canRedo(history)) return;
  const restored = historyRedo(history, snapshotOf(model));
  if (!restored) return;
  applySnapshot(restored);
  refreshHistoryButtons();
  ok(`redo (${history.undoStack.length} earlier · ${history.redoStack.length} ahead)`);
}

undoBtn.addEventListener("click", performUndo);
redoBtn.addEventListener("click", performRedo);

// --- keyboard shortcuts ---
window.addEventListener("keydown", (e) => {
  // Undo / redo: meta on macOS, ctrl elsewhere. Capture *before* the
  // input-focused early-return so it works while a slider has focus.
  const mod = e.metaKey || e.ctrlKey;
  if (mod && (e.key === "z" || e.key === "Z")) {
    e.preventDefault();
    if (e.shiftKey) performRedo();
    else performUndo();
    return;
  }
  if (mod && (e.key === "y" || e.key === "Y")) {
    e.preventDefault();
    performRedo();
    return;
  }
  // Don't intercept while typing in a slider/input.
  const t = e.target as HTMLElement | null;
  if (t && (t.tagName === "INPUT" || t.tagName === "SELECT" || t.tagName === "TEXTAREA")) {
    return;
  }
  switch (e.key) {
    case "1": setView("iso"); break;
    case "2": setView("front"); break;
    case "3": setView("top"); break;
    case "4": setView("side"); break;
    case "w":
    case "W":
      wireframeToggle.checked = !wireframeToggle.checked;
      refreshWireframe();
      break;
    case "r":
    case "R":
      resetBtn.click();
      break;
    case "d":
    case "D":
      drawingBtn.click();
      break;
    case "s":
    case "S":
      downloadBtn.click();
      break;
    case "Escape":
      highlightedFace = -1;
      selectedFeatureId = null;
      refreshFaceColors();
      renderSelectedFeaturePanel();
      break;
  }
});

// --- file drop / picker ---
fileInput.addEventListener("change", () => {
  const f = fileInput.files?.[0];
  if (!f) return;
  f.text().then(loadJson);
});
["dragenter", "dragover"].forEach((evt) =>
  dropZone.addEventListener(evt, (e) => {
    e.preventDefault();
    (e as DragEvent).dataTransfer!.dropEffect = "copy";
    dropZone.classList.add("over");
  }),
);
["dragleave", "drop"].forEach((evt) =>
  dropZone.addEventListener(evt, (e) => {
    e.preventDefault();
    dropZone.classList.remove("over");
  }),
);
dropZone.addEventListener("drop", (e) => {
  const f = (e as DragEvent).dataTransfer?.files?.[0];
  if (!f) return;
  f.text().then(loadJson);
});

// --- example loader ---
document.querySelectorAll("[data-example]").forEach((a) => {
  a.addEventListener("click", async () => {
    const name = (a as HTMLElement).dataset.example!;
    try {
      const r = await fetch(`/examples/${name}.json`);
      const json = await r.text();
      loadJson(json);
    } catch (e) {
      err(`could not fetch example: ${e}`);
    }
  });
});

// --- feature catalog browser ---
const ALL_FEATURE_KINDS: string[] = (() => {
  try {
    return feature_kinds() as string[];
  } catch {
    return [];
  }
})();

let catalogOpen = false;

function refreshCatalog() {
  const query = catalogSearchEl.value;
  const filtered = searchKinds(ALL_FEATURE_KINDS, query);
  const grouped = groupByCategory(filtered);
  catalogListEl.innerHTML = "";
  if (filtered.length === 0) {
    const empty = document.createElement("div");
    empty.id = "catalog-empty";
    empty.textContent = `no kinds match “${query}”`;
    catalogListEl.appendChild(empty);
    return;
  }
  // Stable category order — primitives first because that's most of
  // what people want; transforms/booleans last because they need
  // wired-up inputs.
  const ORDER: Category[] = [
    "Primitives",
    "Manufacturing",
    "Sweep & Loft",
    "Structural",
    "Fasteners",
    "Joinery",
    "Reference",
    "Transforms",
    "Patterns & Booleans",
    "Other",
  ];
  for (const cat of ORDER) {
    const list = grouped.get(cat);
    if (!list || list.length === 0) continue;
    const section = document.createElement("div");
    section.className = "cat-section";
    const h = document.createElement("h3");
    h.textContent = `${cat} (${list.length})`;
    section.appendChild(h);
    const ul = document.createElement("ul");
    for (const kind of list) {
      const li = document.createElement("li");
      li.textContent = kind;
      li.title = `insert a default-parameter ${kind} into the model`;
      li.addEventListener("click", () => insertFeatureKind(kind));
      ul.appendChild(li);
    }
    section.appendChild(ul);
    catalogListEl.appendChild(section);
  }
}

/**
 * Splice a default-parameter instance of `kind` into the current model
 * and re-evaluate. The new feature gets a unique id derived from the
 * kind name (`cylinder_3` if `cylinder_2` already exists, etc.) so it
 * doesn't collide with existing features.
 */
function insertFeatureKind(kind: string) {
  if (!model) {
    err("load a model first, then add features from the catalog");
    return;
  }
  recordHistory();
  const parsed = JSON.parse(model.json);
  const existing: Array<Record<string, unknown>> = parsed.features ?? [];
  const existingIds = new Set(
    existing
      .map((f) => f.id)
      .filter((x): x is string => typeof x === "string"),
  );
  const baseId = kind.toLowerCase();
  let id = baseId;
  let n = 1;
  while (existingIds.has(id)) {
    n += 1;
    id = `${baseId}_${n}`;
  }
  const inst = defaultInstance(kind, id);
  // Wire the new feature to the previous target where possible — it
  // gives the user a working chain on insert for transforms/booleans
  // instead of an immediate "_ not found" error. Skip for primitives
  // (which don't have an `input` field).
  if (model.targetId && existing.length > 0) {
    if (inst.input === "_") inst.input = model.targetId;
    if (Array.isArray(inst.inputs) && inst.inputs.every((x) => x === "_a" || x === "_b")) {
      inst.inputs = [model.targetId, model.targetId];
    }
  }
  existing.push(inst);
  parsed.features = existing;
  model.json = JSON.stringify(parsed, null, 2);
  // Make the new feature the target so the user immediately sees what
  // they inserted.
  model.targetId = id;
  const ids = parsedFeatureIds(model.json);
  renderTargets(ids);
  renderFeatureTree();
  rebuild();
  ok(`inserted ${kind} as '${id}' — ${existing.length} features total`);
}

catalogToggleEl.addEventListener("click", () => {
  catalogOpen = !catalogOpen;
  catalogBodyEl.hidden = !catalogOpen;
  catalogToggleLabel.textContent = catalogOpen
    ? `▼ Browse Features (${ALL_FEATURE_KINDS.length})`
    : `▶ Browse Features (${ALL_FEATURE_KINDS.length})`;
  if (catalogOpen) {
    refreshCatalog();
    catalogSearchEl.focus();
  }
});

catalogSearchEl.addEventListener("input", refreshCatalog);

// Initialise the toggle label with the count, so it reads
// "▶ Browse Features (231)" before the user opens it.
catalogToggleLabel.textContent = `▶ Browse Features (${ALL_FEATURE_KINDS.length})`;

ok("waiting for a model — drop a JSON or click an example");
