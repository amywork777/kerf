import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";

import init, {
  evaluate_to_mesh,
  evaluate_with_params,
  evaluate_with_face_ids,
  parameters_of,
  target_ids_of,
} from "./wasm/kerf_cad_wasm.js";
import { exportThreeViewPng } from "./drawings.js";

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
let currentFaceCount = 0;
let highlightedFace = -1;
let hoveredFace = -1;

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
    ok(`selected face #${fid} (of ${currentFaceCount})`);
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

const SEGMENTS = 24;

function ok(msg: string) {
  status.classList.remove("error");
  status.textContent = msg;
}
function err(msg: string) {
  status.classList.add("error");
  status.textContent = msg;
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
      face_count: number;
      volume: number;
      shell_count: number;
      vertex_count: number;
      edge_count: number;
      face_count_topo: number;
    };
    const tris = new Float32Array(result.triangles);
    const faceIds = new Uint32Array(result.face_ids);
    const dt = performance.now() - t0;
    setMesh(tris, faceIds, result.face_count, fit);
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
    input.addEventListener("input", () => {
      const v = Number(input.value);
      model!.parameters[k] = v;
      valSpan.textContent = v.toFixed(3);
      rebuild();
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
  parsed.features = (parsed.features ?? []).filter(
    (f: { id?: string }) => f?.id !== id,
  );
  const removed = before - parsed.features.length;
  if (removed === 0) return;
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
    renderTargets(ids);
    renderParams();
    renderFeatureTree();
    actionsEl.hidden = false;
    actions2El.hidden = false;
    viewsEl.hidden = false;
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

// --- keyboard shortcuts ---
window.addEventListener("keydown", (e) => {
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
      refreshFaceColors();
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

ok("waiting for a model — drop a JSON or click an example");
