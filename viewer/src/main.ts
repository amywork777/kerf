import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";

import init, {
  evaluate_to_mesh,
  evaluate_with_params,
  parameters_of,
  target_ids_of,
} from "./wasm/kerf_cad_wasm.js";

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
  color: 0x6ea8ff,
  metalness: 0.2,
  roughness: 0.55,
  flatShading: true,
  side: THREE.DoubleSide,
});

let currentMesh: THREE.Mesh | null = null;
let currentWireframe: THREE.LineSegments | null = null;

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

function setMesh(triangles: Float32Array, fit: boolean) {
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
  if (triangles.length === 0) return;

  const geom = new THREE.BufferGeometry();
  geom.setAttribute("position", new THREE.BufferAttribute(triangles, 3));
  geom.computeVertexNormals();
  geom.computeBoundingBox();
  currentMesh = new THREE.Mesh(geom, meshMaterial);
  scene.add(currentMesh);
  refreshWireframe();
  if (fit && geom.boundingBox) fitToView(geom.boundingBox);
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
    const tris = evaluate_with_params(
      model.json,
      model.targetId,
      JSON.stringify(model.parameters),
      SEGMENTS,
    );
    const dt = performance.now() - t0;
    setMesh(new Float32Array(tris), fit);
    ok(
      `target='${model.targetId}'  triangles=${tris.length / 9}  eval=${dt.toFixed(1)}ms`,
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
    rebuild();
  };
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
    actionsEl.hidden = false;
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
