import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";

import init, {
  evaluate_to_mesh,
  evaluate_with_params,
  evaluate_with_face_ids,
  mass_properties_of,
  parameters_of,
  target_ids_of,
  assembly_bom_of,
} from "./wasm/kerf_cad_wasm.js";
import { exportThreeViewPng } from "./drawings.js";
import { mountSketcher, buildExtrudeModelJson, type Sketch } from "./sketcher.js";
import { findEdgeLoop, findFaceLoop } from "./pick-loops.js";

await init();

// --- BOM panel ---
const bomEl = document.getElementById("bom")!;
const bomPanel = mountBom(bomEl);

const stage = document.getElementById("stage")!;
const status = document.getElementById("status")!;
const fileInput = document.getElementById("file") as HTMLInputElement;
const dropZone = document.getElementById("drop")!;
const paramsEl = document.getElementById("params")!;
const targetsEl = document.getElementById("targets")!;
const targetSelect = document.getElementById("target-select") as HTMLSelectElement;
const actionsEl = document.getElementById("actions")!;
const viewsEl = document.getElementById("views")!;
const openBtn = document.getElementById("open-btn")!;
const saveBtn = document.getElementById("save-btn")!;
const saveAsBtn = document.getElementById("save-as-btn")!;
const saveStatusEl = document.getElementById("save-status")!;
const resetBtn = document.getElementById("reset-btn")!;
const wireframeToggle = document.getElementById("wireframe-toggle") as HTMLInputElement;
const drawingBtn = document.getElementById("drawing-btn")!;
const actions2El = document.getElementById("actions2")!;
const gdtBtn = document.getElementById("gdt-btn")!;
const gdtPanel = document.getElementById("gdt-panel")!;
const gdtJson = document.getElementById("gdt-json") as HTMLTextAreaElement;
const gdtExportBtn = document.getElementById("gdt-export-btn")!;
const gdtClearBtn = document.getElementById("gdt-clear-btn")!;
const gdtError = document.getElementById("gdt-error")!;
const gdtHelpLink = document.getElementById("gdt-help-link")!;
const featureTreeEl = document.getElementById("feature-tree")!;
const featureListEl = document.getElementById("feature-list")!;
const featureCountEl = document.getElementById("feature-count")!;
const massPropHost = document.getElementById("mass-properties")!;
const massPropertiesPanel = mountMassProperties(massPropHost);

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

// Camera toggle state — declared early so resize() can reference them.
let isOrtho = false;
let orthoCamera: THREE.OrthographicCamera | null = null;

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
// Required for per-material clipping planes to take effect.
renderer.localClippingEnabled = true;
meshMaterial.clippingPlanes = [];

let currentMesh: THREE.Mesh | null = null;
let currentWireframe: THREE.LineSegments | null = null;
let currentFaceIds: Uint32Array | null = null;
let currentFaceOwnerTags: string[] | null = null;
let currentFaceCount = 0;

// Picking state — owns selectedFaces (Set), hoveredFace, highlightedVertex/Edge.
let picking: PickingState = createPickingState();

// Assigned by the section-view mount block below. setMesh() guards with
// `?.` because it can run during the first-rebuild path before the mount
// statement has executed.
let sectionView: SectionViewHandle | undefined;

// Topology data for vertex / edge picking. Set on every rebuild from
// the WASM `evaluate_with_face_ids` response.
let currentVertexPositions: Float32Array | null = null;
let currentEdgeEndpoints: Uint32Array | null = null;
let currentVertexToFaces: { offsets: Uint32Array; indices: Uint32Array } | null = null;
let currentVertexToEdges: { offsets: Uint32Array; indices: Uint32Array } | null = null;
let currentEdgeToFaces: { offsets: Uint32Array; indices: Uint32Array } | null = null;
let currentFaceToFaces: { offsets: Uint32Array; indices: Uint32Array } | null = null;
let highlightedVertex = -1;
let highlightedEdge = -1;

// Loop selection state: when Alt+click triggers an edge/face loop, we store
// all ids in the loop so they can be highlighted together.
let highlightedEdgeLoop: number[] = [];
let highlightedFaceLoop: number[] = [];

// Picking mode: which kind of topology element does a click select?
type PickMode = "face" | "edge" | "vertex";
let pickMode: PickMode = "face";

// Feature-kind filter for face picking. "All" means no filtering.
let filterKind: FilterKind = "All";
// Map from face-id → owner feature id, populated from WASM face_owner_tags.
let faceOwnerTags: Map<number, string> = new Map();
// Parsed feature summaries from the loaded model JSON, used by applyFilter.
let parsedFeaturesForFilter: ParsedFeature[] = [];

// Display helpers for vertex/edge picking — small markers/highlighters
// that mount under the mesh while a non-face mode is active.
let vertexHelper: THREE.Points | null = null;
let edgeHelper: THREE.LineSegments | null = null;
let vertexHighlightDot: THREE.Mesh | null = null;
let edgeHighlightLine: THREE.LineSegments | null = null;

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

/// Pick the nearest vertex (in world-space distance from the cursor
/// ray) to a click. Returns -1 if no vertex is within a small NDC
/// pixel radius.
function pickVertexAt(clientX: number, clientY: number): number {
  if (!currentVertexPositions) return -1;
  const rect = renderer.domElement.getBoundingClientRect();
  ndc.x = ((clientX - rect.left) / rect.width) * 2 - 1;
  ndc.y = -((clientY - rect.top) / rect.height) * 2 + 1;
  const PICK_RADIUS_NDC = 0.05;
  let best = -1;
  let bestDist = PICK_RADIUS_NDC;
  const tmp = new THREE.Vector3();
  const n = currentVertexPositions.length / 3;
  for (let i = 0; i < n; i++) {
    tmp.set(
      currentVertexPositions[i * 3]!,
      currentVertexPositions[i * 3 + 1]!,
      currentVertexPositions[i * 3 + 2]!,
    );
    tmp.project(camera);
    const dx = tmp.x - ndc.x;
    const dy = tmp.y - ndc.y;
    const d = Math.sqrt(dx * dx + dy * dy);
    if (d < bestDist) {
      bestDist = d;
      best = i;
    }
  }
  return best;
}

/// Pick the nearest edge by raycasting against an invisible LineSegments.
function pickEdgeAt(clientX: number, clientY: number): number {
  if (!currentEdgeEndpoints || !currentVertexPositions) return -1;
  const rect = renderer.domElement.getBoundingClientRect();
  ndc.x = ((clientX - rect.left) / rect.width) * 2 - 1;
  ndc.y = -((clientY - rect.top) / rect.height) * 2 + 1;
  // Project edges, find shortest perpendicular distance from cursor to
  // each edge segment in NDC space. Pick the closest within
  // PICK_RADIUS_NDC.
  const PICK_RADIUS_NDC = 0.03;
  let best = -1;
  let bestDist = PICK_RADIUS_NDC;
  const a = new THREE.Vector3();
  const b = new THREE.Vector3();
  const n = currentEdgeEndpoints.length / 2;
  for (let i = 0; i < n; i++) {
    const va = currentEdgeEndpoints[i * 2]!;
    const vb = currentEdgeEndpoints[i * 2 + 1]!;
    if (va === 0xffffffff || vb === 0xffffffff) continue;
    a.set(
      currentVertexPositions[va * 3]!,
      currentVertexPositions[va * 3 + 1]!,
      currentVertexPositions[va * 3 + 2]!,
    );
    b.set(
      currentVertexPositions[vb * 3]!,
      currentVertexPositions[vb * 3 + 1]!,
      currentVertexPositions[vb * 3 + 2]!,
    );
    a.project(camera);
    b.project(camera);
    // Distance from ndc to segment ab in 2D.
    const abx = b.x - a.x;
    const aby = b.y - a.y;
    const ablenSq = abx * abx + aby * aby;
    let t = 0;
    if (ablenSq > 0) {
      t = ((ndc.x - a.x) * abx + (ndc.y - a.y) * aby) / ablenSq;
      t = Math.max(0, Math.min(1, t));
    }
    const px = a.x + t * abx;
    const py = a.y + t * aby;
    const d = Math.sqrt((ndc.x - px) ** 2 + (ndc.y - py) ** 2);
    if (d < bestDist) {
      bestDist = d;
      best = i;
    }
  }
  return best;
}

/// Neighbors stored in CSR form: indices[offsets[i]..offsets[i+1]].
function neighborsOf(
  csr: { offsets: Uint32Array; indices: Uint32Array },
  i: number,
): number[] {
  if (i < 0 || i + 1 >= csr.offsets.length) return [];
  const lo = csr.offsets[i]!;
  const hi = csr.offsets[i + 1]!;
  return Array.from(csr.indices.slice(lo, hi));
}

/// Edges incident to a vertex, derived from edge_endpoints (no
/// dedicated CSR — small enough to scan).
function verticesIncidentEdges(vid: number): number[] {
  if (!currentEdgeEndpoints) return [];
  const out: number[] = [];
  const n = currentEdgeEndpoints.length / 2;
  for (let i = 0; i < n; i++) {
    if (
      currentEdgeEndpoints[i * 2] === vid ||
      currentEdgeEndpoints[i * 2 + 1] === vid
    ) {
      out.push(i);
    }
  }
  return out;
}

/// Update or create the small dot shown at the picked vertex.
function refreshVertexHighlight() {
  if (vertexHighlightDot) {
    scene.remove(vertexHighlightDot);
    vertexHighlightDot.geometry.dispose();
    vertexHighlightDot = null;
  }
  if (picking.highlightedVertex < 0 || !currentVertexPositions) return;
  const vid = picking.highlightedVertex;
  const x = currentVertexPositions[vid * 3]!;
  const y = currentVertexPositions[vid * 3 + 1]!;
  const z = currentVertexPositions[vid * 3 + 2]!;
  const sz = currentMesh?.geometry.boundingBox
    ? currentMesh.geometry.boundingBox.getSize(new THREE.Vector3()).length()
    : 50;
  const sphereGeo = new THREE.SphereGeometry(sz * 0.01, 12, 8);
  const sphereMat = new THREE.MeshBasicMaterial({ color: 0xffb84d });
  vertexHighlightDot = new THREE.Mesh(sphereGeo, sphereMat);
  vertexHighlightDot.position.set(x, y, z);
  scene.add(vertexHighlightDot);
}

/// Update or create the highlighted-edge LineSegments.
/// When an edge loop is active (highlightedEdgeLoop), all loop edges are drawn.
function refreshEdgeHighlight() {
  if (edgeHighlightLine) {
    scene.remove(edgeHighlightLine);
    edgeHighlightLine.geometry.dispose();
    edgeHighlightLine = null;
  }
  if (!currentEdgeEndpoints || !currentVertexPositions) return;

  // Collect the set of edges to draw: loop takes precedence, else single edge.
  const edgesToDraw: number[] =
    highlightedEdgeLoop.length > 0
      ? highlightedEdgeLoop
      : highlightedEdge >= 0
        ? [highlightedEdge]
        : [];
  if (edgesToDraw.length === 0) return;

  const posArr: number[] = [];
  for (const eid of edgesToDraw) {
    const va = currentEdgeEndpoints[eid * 2]!;
    const vb = currentEdgeEndpoints[eid * 2 + 1]!;
    if (va === 0xffffffff || vb === 0xffffffff) continue;
    posArr.push(
      currentVertexPositions[va * 3]!, currentVertexPositions[va * 3 + 1]!, currentVertexPositions[va * 3 + 2]!,
      currentVertexPositions[vb * 3]!, currentVertexPositions[vb * 3 + 1]!, currentVertexPositions[vb * 3 + 2]!,
    );
  }
  if (posArr.length === 0) return;

  const positions = new Float32Array(posArr);
  const geo = new THREE.BufferGeometry();
  geo.setAttribute("position", new THREE.BufferAttribute(positions, 3));
  const mat = new THREE.LineBasicMaterial({
    color: 0xffb84d,
    linewidth: 4,
    depthTest: false,
  });
  edgeHighlightLine = new THREE.LineSegments(geo, mat);
  scene.add(edgeHighlightLine);
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
  highlightedVertex = -1;
  highlightedEdge = -1;
  highlightedEdgeLoop = [];
  highlightedFaceLoop = [];
  refreshVertexHighlight();
  refreshEdgeHighlight();

  if (triangles.length === 0) {
    sectionView?.refresh(null);
    return;
  }

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

  // Update section-view slider range when a new mesh is loaded.
  if (geom.boundingBox && sectionView) {
    const bb = geom.boundingBox;
    sectionView.refresh({
      min: [bb.min.x, bb.min.y, bb.min.z],
      max: [bb.max.x, bb.max.y, bb.max.z],
    });
  }
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
  if (highlightedFaceLoop.includes(faceId)) return HIGHLIGHT_COLOR;
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

const pickModeButtons = pickModeGroupEl.querySelectorAll(
  "button",
) as NodeListOf<HTMLButtonElement>;
pickModeButtons.forEach((btn) => {
  btn.addEventListener("click", () => {
    pickMode = btn.dataset.pick as PickMode;
    pickModeButtons.forEach((b) => b.classList.toggle("active", b === btn));
    // Clear selection when switching modes.
    highlightedFace = -1;
    highlightedVertex = -1;
    highlightedEdge = -1;
    highlightedEdgeLoop = [];
    highlightedFaceLoop = [];
    refreshFaceColors();
    refreshVertexHighlight();
    refreshEdgeHighlight();
  });
});

// Wire the filter dropdown.
pickFilterEl.addEventListener("change", () => {
  filterKind = pickFilterEl.value as FilterKind;
  // Clear any selection that may no longer be valid under the new filter.
  if (filterKind !== "All" && highlightedFace >= 0) {
    if (!applyFilter(highlightedFace, faceOwnerTags, parsedFeaturesForFilter, filterKind)) {
      highlightedFace = -1;
    }
  }
  refreshFaceColors();
});

document.addEventListener("keydown", (e) => {
  if (e.target instanceof HTMLInputElement || e.target instanceof HTMLSelectElement) return;
  if (e.key === "f" || e.key === "F") {
    pickMode = "face";
    pickModeButtons.forEach((b) => b.classList.toggle("active", b.dataset.pick === "face"));
  } else if (e.key === "e" || e.key === "E") {
    pickMode = "edge";
    pickModeButtons.forEach((b) => b.classList.toggle("active", b.dataset.pick === "edge"));
  } else if (e.key === "v" || e.key === "V") {
    pickMode = "vertex";
    pickModeButtons.forEach((b) => b.classList.toggle("active", b.dataset.pick === "vertex"));
  }
});

function resize() {
  const r = stage.getBoundingClientRect();
  renderer.setSize(r.width, r.height, false);
  camera.aspect = r.width / Math.max(1, r.height);
  camera.updateProjectionMatrix();
  if (orthoCamera) resizeOrthoCamera(orthoCamera, camera);
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
  // --- Cmd+click: smart-pick — cycle face → edge → vertex → face ---
  if (e.metaKey || e.ctrlKey) {
    const cycle: PickMode[] = ["face", "edge", "vertex"];
    const next = cycle[(cycle.indexOf(pickMode) + 1) % cycle.length]!;
    pickMode = next;
    pickModeButtons.forEach((b) => b.classList.toggle("active", b.dataset.pick === next));
    // Clear selection state when mode cycles.
    highlightedFace = -1;
    highlightedVertex = -1;
    highlightedEdge = -1;
    highlightedEdgeLoop = [];
    highlightedFaceLoop = [];
    refreshFaceColors();
    refreshVertexHighlight();
    refreshEdgeHighlight();
    ok(`smart-pick: mode → ${next}`);
    return;
  }

  // --- Alt+click: loop/ring selection ---
  if (e.altKey) {
    if (pickMode === "edge" && currentEdgeEndpoints && currentVertexToEdges && currentEdgeToFaces) {
      const eid = pickEdgeAt(e.clientX, e.clientY);
      if (eid >= 0) {
        const loop = findEdgeLoop(eid, currentEdgeEndpoints, currentVertexToEdges, currentEdgeToFaces);
        highlightedEdgeLoop = loop;
        highlightedEdge = eid;
        highlightedFace = -1;
        highlightedVertex = -1;
        highlightedFaceLoop = [];
        refreshFaceColors();
        refreshVertexHighlight();
        refreshEdgeHighlight();
        ok(`edge loop: ${loop.length} edges [${loop.join(",")}]`);
      }
    } else if (pickMode === "face" && currentFaceToFaces) {
      const fid = pickFaceAt(e.clientX, e.clientY);
      if (fid >= 0) {
        const ring = findFaceLoop(fid, currentFaceToFaces);
        highlightedFaceLoop = ring;
        highlightedFace = fid;
        highlightedEdge = -1;
        highlightedVertex = -1;
        highlightedEdgeLoop = [];
        refreshFaceColors();
        refreshVertexHighlight();
        refreshEdgeHighlight();
        ok(`face ring: ${ring.length} faces [${ring.join(",")}]`);
      }
    }
    return;
  }

  // --- Normal click: single-element selection ---
  highlightedEdgeLoop = [];
  highlightedFaceLoop = [];

  if (pickMode === "face") {
    const rawFid = pickFaceAt(e.clientX, e.clientY);
    // Apply feature-kind filter: if the face doesn't pass, treat as a miss.
    const fid =
      rawFid >= 0 &&
      applyFilter(rawFid, faceOwnerTags, parsedFeaturesForFilter, filterKind)
        ? rawFid
        : -1;
    highlightedFace = fid;
    highlightedVertex = -1;
    highlightedEdge = -1;
    refreshFaceColors();
    refreshVertexHighlight();
    refreshEdgeHighlight();
    if (fid >= 0) {
      const ownerTag = faceOwnerTags.get(fid);
      const ownerInfo = ownerTag ? ` owner=${ownerTag}` : "";
      ok(`selected face #${fid}${ownerInfo} (of ${currentFaceCount})`);
    }
  } else if (pickMode === "vertex") {
    const vid = pickVertexAt(e.clientX, e.clientY);
    picking = handleVertexClick(picking, vid);
    refreshFaceColors();
    refreshVertexHighlight();
    refreshEdgeHighlight();
    if (vid >= 0 && currentVertexToFaces) {
      const faces = neighborsOf(currentVertexToFaces, vid);
      ok(`selected vertex #${vid} — touches ${faces.length} faces, ` +
         `${verticesIncidentEdges(vid).length} edges`);
    }
  } else if (pickMode === "edge") {
    const eid = pickEdgeAt(e.clientX, e.clientY);
    picking = handleEdgeClick(picking, eid);
    refreshFaceColors();
    refreshVertexHighlight();
    refreshEdgeHighlight();
    if (eid >= 0 && currentEdgeToFaces) {
      const faces = neighborsOf(currentEdgeToFaces, eid);
      ok(`selected edge #${eid} — bordered by ${faces.length} faces`);
    }
  }
});

// Debounce hover updates to ~1 frame (16 ms) so we don't raytrace on every
// raw mousemove event. We store the latest pointer coords and fire a single
// pick after the timer expires.
let hoverDebounceTimer: ReturnType<typeof setTimeout> | null = null;
let pendingHoverX = 0;
let pendingHoverY = 0;

function flushHover() {
  hoverDebounceTimer = null;
  if (!currentMesh || pickMode !== "face") return;
  const fid = pickFaceAt(pendingHoverX, pendingHoverY);
  const next = handleHover(picking, fid);
  if (next !== null) {
    picking = next;
    refreshFaceColors();
  }
}

renderer.domElement.addEventListener("pointermove", (e) => {
  if (!currentMesh) return;
  const rawFid = pickFaceAt(e.clientX, e.clientY);
  // Apply feature-kind filter to hover as well.
  const fid =
    rawFid >= 0 &&
    applyFilter(rawFid, faceOwnerTags, parsedFeaturesForFilter, filterKind)
      ? rawFid
      : -1;
  if (fid !== hoveredFace) {
    hoveredFace = fid;
    refreshFaceColors();
  }
});

function tick() {
  controls.update();
  renderer.render(scene, isOrtho && orthoCamera ? orthoCamera : camera);
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
let pmHandle: PropertyManagerHandle | null = null;

const SEGMENTS = 24;

/**
 * Build a face→face adjacency CSR from edgeToFaces data.
 * Two faces are adjacent when they share at least one edge.
 */
function buildFaceAdjacency(
  faceCount: number,
  edgeToFacesOffsets: Uint32Array,
  edgeToFacesIndices: Uint32Array,
): { offsets: Uint32Array; indices: Uint32Array } {
  const adjSets: Set<number>[] = Array.from({ length: faceCount }, () => new Set());
  const nEdges = edgeToFacesOffsets.length - 1;
  for (let eid = 0; eid < nEdges; eid++) {
    const lo = edgeToFacesOffsets[eid]!;
    const hi = edgeToFacesOffsets[eid + 1]!;
    const faces: number[] = [];
    for (let k = lo; k < hi; k++) faces.push(edgeToFacesIndices[k]!);
    for (let a = 0; a < faces.length; a++) {
      for (let b = 0; b < faces.length; b++) {
        if (a !== b) adjSets[faces[a]!]!.add(faces[b]!);
      }
    }
  }
  const offsets = new Uint32Array(faceCount + 1);
  let total = 0;
  for (let i = 0; i < faceCount; i++) {
    offsets[i] = total;
    total += adjSets[i]!.size;
  }
  offsets[faceCount] = total;
  const indices = new Uint32Array(total);
  let k = 0;
  for (let i = 0; i < faceCount; i++) {
    for (const f of adjSets[i]!) indices[k++] = f;
  }
  return { offsets, indices };
}

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
      face_owner_tags: string[];
      face_count: number;
      volume: number;
      shell_count: number;
      vertex_count: number;
      edge_count: number;
      face_count_topo: number;
      vertex_positions: number[];
      edge_endpoints: number[];
      vertex_to_edges_offsets: number[];
      vertex_to_edges_indices: number[];
      vertex_to_faces_offsets: number[];
      vertex_to_faces_indices: number[];
      edge_to_faces_offsets: number[];
      edge_to_faces_indices: number[];
      // Optional: map of face-id → owner feature id string.
      face_owner_tags?: Record<string, string>;
    };
    const tris = new Float32Array(result.triangles);
    const faceIds = new Uint32Array(result.face_ids);
    const ownerTags = result.face_owner_tags ?? [];
    const dt = performance.now() - t0;
    currentVertexPositions = new Float32Array(result.vertex_positions ?? []);
    currentEdgeEndpoints = new Uint32Array(result.edge_endpoints ?? []);
    if (result.vertex_to_edges_offsets) {
      currentVertexToFaces = {
        offsets: new Uint32Array(result.vertex_to_faces_offsets),
        indices: new Uint32Array(result.vertex_to_faces_indices),
      };
      currentVertexToEdges = {
        offsets: new Uint32Array(result.vertex_to_edges_offsets),
        indices: new Uint32Array(result.vertex_to_edges_indices),
      };
      currentEdgeToFaces = {
        offsets: new Uint32Array(result.edge_to_faces_offsets),
        indices: new Uint32Array(result.edge_to_faces_indices),
      };
      // Derive face→face adjacency from edgeToFaces: two faces are adjacent
      // if they share an edge.
      currentFaceToFaces = buildFaceAdjacency(
        result.face_count_topo,
        new Uint32Array(result.edge_to_faces_offsets),
        new Uint32Array(result.edge_to_faces_indices),
      );
    } else {
      currentVertexToFaces = null;
      currentVertexToEdges = null;
      currentEdgeToFaces = null;
      currentFaceToFaces = null;
    }
    // Extract face_owner_tags (face-id string key → owner feature id).
    faceOwnerTags = new Map();
    if (result.face_owner_tags) {
      for (const [k, v] of Object.entries(result.face_owner_tags)) {
        faceOwnerTags.set(Number(k), v);
      }
    }
    // Sync the feature kind filter dropdown with the current model.
    parsedFeaturesForFilter = model ? summarizeFeatures(model.json) : [];
    renderFilterDropdown();
    setMesh(tris, faceIds, result.face_count, fit);
    ok(
      `target='${model.targetId}'  V/E/F/S=${result.vertex_count}/${result.edge_count}/${result.face_count_topo}/${result.shell_count}` +
        `  vol=${result.volume.toFixed(3)}  tris=${tris.length / 9}  eval=${dt.toFixed(1)}ms`,
    );
    // Update mass properties panel (skip if mesh is empty).
    if (tris.length > 0) {
      try {
        const mp = mass_properties_of(
          model.json,
          model.targetId,
          JSON.stringify(model.parameters),
          SEGMENTS,
        ) as MassPropertiesData;
        massPropertiesPanel.update(mp);
      } catch {
        massPropertiesPanel.update(null);
      }
    } else {
      massPropertiesPanel.update(null);
    }
  } catch (e) {
    err(String(e));
    massPropertiesPanel.update(null);
  }
}

function renderParams() {
  if (!model) {
    paramsEl.innerHTML = "";
    pmHandle = null;
    return;
  }
  if (pmHandle) {
    pmHandle.refresh();
    return;
  }
  pmHandle = mountPropertyManager(paramsEl, {
    getParams: () => model!.parameters,
    getDefaults: () => model!.defaults,
    setParam: (name, value) => {
      model!.parameters[name] = value;
    },
    onChange: () => {
      rebuild();
    },
  });
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

/**
 * Rebuild the #pick-filter dropdown with the feature kinds present in the
 * current model. Preserves the current selection when possible.
 */
function renderFilterDropdown() {
  const prev = pickFilterEl.value;
  pickFilterEl.innerHTML = "";
  const allOpt = document.createElement("option");
  allOpt.value = "All";
  allOpt.textContent = "All";
  pickFilterEl.appendChild(allOpt);

  const kinds = collectFeatureKinds(parsedFeaturesForFilter);
  for (const k of kinds) {
    const opt = document.createElement("option");
    opt.value = k;
    opt.textContent = k;
    pickFilterEl.appendChild(opt);
  }

  // Restore previous selection when the kind still exists, else reset.
  pickFilterEl.value = kinds.includes(prev) || prev === "All" ? prev : "All";
  filterKind = pickFilterEl.value as FilterKind;
}

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

// Drag-to-reorder state.
let dragSrcId: string | null = null;

function applyFeatureSearch() {
  const query = featureSearchEl.value;
  const features = summarizeFeatures(model?.json ?? "[]");
  const visible = new Set(filterFeatures(features, query).map((f) => f.id));
  for (const li of featureListEl.children as HTMLCollectionOf<HTMLLIElement>) {
    li.classList.toggle("hidden", !visible.has(li.dataset.id ?? ""));
  }
}

featureSearchEl.addEventListener("input", applyFeatureSearch);
featureSearchClearEl.addEventListener("click", () => {
  featureSearchEl.value = "";
  applyFeatureSearch();
});

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
    li.draggable = true;
    li.innerHTML =
      `<span class="drag-handle" title="Drag to reorder">⠿</span>` +
      `<span class="kind">${f.kind}</span><span class="id">${f.id}</span>` +
      `<span class="del" title="Delete this feature">✕</span>`;
    (li.querySelector(".id") as HTMLElement).title = `Click to set as target — current: '${f.id}'`;

    // Click to set target.
    li.querySelector(".id")!.addEventListener("click", () => {
      if (!model) return;
      model.targetId = f.id;
      targetSelect.value = f.id;
      refreshFeatureTreeSelection();
      rebuild();
    });

    // Delete button.
    li.querySelector(".del")!.addEventListener("click", (e) => {
      e.stopPropagation();
      deleteFeature(f.id);
    });

    // --- HTML5 drag-to-reorder ---
    li.addEventListener("dragstart", (e) => {
      dragSrcId = f.id;
      li.classList.add("dragging");
      e.dataTransfer!.effectAllowed = "move";
      e.dataTransfer!.setData("text/plain", f.id);
    });

    li.addEventListener("dragend", () => {
      dragSrcId = null;
      li.classList.remove("dragging");
      // Clear all drop indicators.
      for (const row of featureListEl.children as HTMLCollectionOf<HTMLLIElement>) {
        row.classList.remove("drag-over-before", "drag-over-after");
      }
    });

    li.addEventListener("dragover", (e) => {
      if (!dragSrcId || dragSrcId === f.id) return;
      e.preventDefault();
      e.dataTransfer!.dropEffect = "move";
      // Determine whether to place before or after based on cursor position.
      const rect = li.getBoundingClientRect();
      const midY = rect.top + rect.height / 2;
      const position = e.clientY < midY ? "before" : "after";
      li.classList.toggle("drag-over-before", position === "before");
      li.classList.toggle("drag-over-after", position === "after");
    });

    li.addEventListener("dragleave", () => {
      li.classList.remove("drag-over-before", "drag-over-after");
    });

    li.addEventListener("drop", (e) => {
      e.preventDefault();
      li.classList.remove("drag-over-before", "drag-over-after");
      if (!dragSrcId || !model || dragSrcId === f.id) return;
      const rect = li.getBoundingClientRect();
      const midY = rect.top + rect.height / 2;
      const position: "before" | "after" = e.clientY < midY ? "before" : "after";
      try {
        model.json = reorderFeatures(model.json, dragSrcId, f.id, position);
        const newIds = summarizeFeatures(model.json).map((feat) => feat.id);
        renderTargets(newIds);
        renderFeatureTree();
        rebuild();
      } catch (ex) {
        err(String(ex));
      }
    });

    featureListEl.appendChild(li);
  }
  refreshFeatureTreeSelection();
  // Re-apply any active search filter after re-render.
  applyFeatureSearch();
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
  markDirty();
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
  input.addEventListener("input", () => {
    const n = Number(input.value);
    valSpan.textContent = n.toFixed(3);
    setFeatureFieldAtPath(path, n);
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
    pmHandle = null; // reset so mountPropertyManager re-mounts for new model
    model = {
      json,
      targetId,
      parameters: { ...params },
      defaults: { ...params },
    };
    selectedFeatureId = null;
    renderTargets(ids);
    renderParams();
    renderFeatureTree();
    renderSelectedFeaturePanel();
    actionsEl.hidden = false;
    actions2El.hidden = false;
    viewsEl.hidden = false;
    rebuild(true);
    refreshSaveStatus();
  } catch (e) {
    err(String(e));
  }
}

// --- file save / open via File System Access API (with anchor-download
// + <input> fallback for browsers without it) ---
const fileHandle = createFileHandle();
const ACCEPT = {
  description: "kerf-cad model",
  mimes: ["application/json"],
  extensions: [".json"],
};
let isDirty = false;
let unsavedToast: HTMLElement | null = null;

function currentModelJson(): string {
  if (!model) return "";
  const parsed = JSON.parse(model.json);
  parsed.parameters = { ...model.parameters };
  return JSON.stringify(parsed, null, 2);
}

function refreshSaveStatus() {
  const name = fileHandle.currentName();
  saveStatusEl.textContent = name ?? "(unsaved)";
  saveStatusEl.classList.toggle("dirty", isDirty);
}

function markDirty() {
  if (!isDirty) {
    isDirty = true;
    refreshSaveStatus();
    scheduleUnsavedToast();
  }
}

function markClean() {
  isDirty = false;
  refreshSaveStatus();
  if (unsavedToast) {
    unsavedToast.remove();
    unsavedToast = null;
  }
}

function scheduleUnsavedToast() {
  if (unsavedToast) return;
  setTimeout(() => {
    if (!isDirty || unsavedToast) return;
    unsavedToast = document.createElement("div");
    unsavedToast.className = "save-toast";
    unsavedToast.textContent = "Unsaved changes — Save (⌘S)";
    unsavedToast.addEventListener("click", () => saveBtn.click());
    document.body.appendChild(unsavedToast);
  }, 30_000);
}

async function doSave() {
  if (!model) return;
  const content = currentModelJson();
  const fallbackName = fileHandle.currentName() ?? "model.kerf-cad.json";
  const result = fileHandle.hasHandle()
    ? await fileHandle.saveExisting(content, fallbackName, ACCEPT)
    : await fileHandle.saveAs(content, fallbackName, ACCEPT);
  if (result) {
    markClean();
    ok(`Saved to ${result.name}`);
  }
}

async function doSaveAs() {
  if (!model) return;
  const result = await fileHandle.saveAs(
    currentModelJson(),
    fileHandle.currentName() ?? "model.kerf-cad.json",
    ACCEPT,
  );
  if (result) {
    markClean();
    ok(`Saved to ${result.name}`);
  }
}

async function doOpen() {
  const opened = await fileHandle.open(ACCEPT);
  if (!opened) return;
  loadJson(opened.text);
  markClean();
  ok(`Opened ${opened.name}`);
}

saveBtn.addEventListener("click", () => void doSave());
saveAsBtn.addEventListener("click", () => void doSaveAs());
openBtn.addEventListener("click", () => void doOpen());

resetBtn.addEventListener("click", () => {
  if (!model) return;
  model.parameters = { ...model.defaults };
  renderParams();
  markDirty();
  rebuild();
});

drawingBtn.addEventListener("click", () => {
  if (!model || !currentMesh) return;
  exportThreeViewPng(currentMesh, model.targetId);
});

// --- GD&T panel ---
const GDT_EXAMPLE: GdtAnnotations = {
  datums: [
    { letter: "A", anchor: [0, 0, 10] },
    { letter: "B", anchor: [10, 0, 10] },
  ],
  frames: [
    {
      characteristic: "perpendicularity",
      tolerance: 0.05,
      diametric: false,
      datum_refs: [{ letter: "A" }],
      anchor: [0, 0, 5],
    },
    {
      characteristic: "position",
      tolerance: 0.1,
      diametric: true,
      datum_refs: [{ letter: "A" }, { letter: "B" }],
      anchor: [10, 0, 5],
    },
  ],
  finishes: [
    { ra: 1.6, process: "machined", anchor: [5, 0, 10] },
  ],
};

gdtBtn.addEventListener("click", () => {
  const isHidden = gdtPanel.hidden;
  gdtPanel.hidden = !isHidden;
  gdtBtn.textContent = isHidden ? "Hide GD&T" : "GD&T";
});

gdtHelpLink.addEventListener("click", (e) => {
  e.preventDefault();
  gdtJson.value = JSON.stringify(GDT_EXAMPLE, null, 2);
  gdtError.style.display = "none";
});

gdtClearBtn.addEventListener("click", () => {
  gdtJson.value = "";
  gdtError.style.display = "none";
});

gdtExportBtn.addEventListener("click", () => {
  if (!model || !currentMesh) {
    gdtError.textContent = "No model loaded.";
    gdtError.style.display = "block";
    return;
  }
  const raw = gdtJson.value.trim();
  let annotations: GdtAnnotations | undefined;
  if (raw) {
    try {
      annotations = JSON.parse(raw) as GdtAnnotations;
    } catch (e) {
      gdtError.textContent = `JSON parse error: ${e}`;
      gdtError.style.display = "block";
      return;
    }
  }
  gdtError.style.display = "none";
  exportThreeViewPng(currentMesh, model.targetId, annotations);
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
  // Cmd/Ctrl+S → save (works even from inside an input — that's expected
  // for a save shortcut).
  if ((e.metaKey || e.ctrlKey) && (e.key === "s" || e.key === "S")) {
    e.preventDefault();
    if (model) saveBtn.click();
    return;
  }
  // Don't intercept other keys while typing in a slider/input.
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
      saveBtn.click();
      break;
    case "Escape":
      picking = clearAll(picking);
      refreshFaceColors();
      refreshVertexHighlight();
      refreshEdgeHighlight();
      break;
  }
});

// --- file drop / picker ---

// Route a dropped/picked file through either the JSON loader or the STEP
// importer based on its extension. STEP files are converted to a kerf-cad
// Model JSON via WASM, then handed to the same `loadJson` flow as a
// hand-authored JSON model — this keeps the rest of the pipeline (params
// panel, target picker, picking) unchanged.
async function handleFile(f: File) {
  const text = await f.text();
  const lower = f.name.toLowerCase();
  if (lower.endsWith(".step") || lower.endsWith(".stp")) {
    try {
      const json = import_step_to_model(text);
      loadJson(json);
    } catch (e) {
      err(`STEP import failed: ${e}`);
    }
    return;
  }
  // Default: treat as model JSON.
  loadJson(text);
}

fileInput.addEventListener("change", () => {
  const f = fileInput.files?.[0];
  if (!f) return;
  void handleFile(f);
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
  void handleFile(f);
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

// --- view cube ---
mountViewCube(stage, camera, controls);

// --- camera toggle (Persp / Ortho) ---
const cameraToggleBtn = document.getElementById("camera-toggle") as HTMLButtonElement;

function buildOrthoCamera(from: THREE.PerspectiveCamera): THREE.OrthographicCamera {
  const r = stage.getBoundingClientRect();
  const aspect = r.width / Math.max(1, r.height);
  const dist = from.position.length() || 150;
  const halfH = dist * Math.tan((from.fov * Math.PI) / 360);
  const halfW = halfH * aspect;
  const cam = new THREE.OrthographicCamera(-halfW, halfW, halfH, -halfH, from.near, from.far);
  cam.position.copy(from.position);
  cam.quaternion.copy(from.quaternion);
  cam.updateProjectionMatrix();
  return cam;
}

function resizeOrthoCamera(cam: THREE.OrthographicCamera, perspCam: THREE.PerspectiveCamera) {
  const r = stage.getBoundingClientRect();
  const aspect = r.width / Math.max(1, r.height);
  const dist = perspCam.position.distanceTo(controls.target) || 150;
  const halfH = dist * Math.tan((perspCam.fov * Math.PI) / 360);
  const halfW = halfH * aspect;
  cam.left = -halfW;
  cam.right = halfW;
  cam.top = halfH;
  cam.bottom = -halfH;
  cam.updateProjectionMatrix();
}

cameraToggleBtn.addEventListener("click", () => {
  isOrtho = !isOrtho;
  if (isOrtho) {
    orthoCamera = buildOrthoCamera(camera);
    controls.object = orthoCamera;
    cameraToggleBtn.textContent = "Ortho";
  } else {
    // Sync persp camera back to wherever the ortho camera ended up.
    if (orthoCamera) {
      camera.position.copy(orthoCamera.position);
      camera.quaternion.copy(orthoCamera.quaternion);
      camera.updateProjectionMatrix();
    }
    controls.object = camera;
    orthoCamera = null;
    cameraToggleBtn.textContent = "Persp";
  }
  controls.update();
});

// --- 2D sketcher panel ---
const sketcherPanel = document.getElementById("sketcher-panel")!;
const sketcherHost = document.getElementById("sketcher-host")!;
const sketcherToggle = document.getElementById("sketcher-toggle")!;
const sketcherClose = document.getElementById("sketcher-close")!;

const sketcher = mountSketcher(sketcherHost, {
  // Validate by attempting to build a SketchExtrude through the WASM. The
  // rust loop tracer rejects open / branching / disjoint sketches with a
  // clear error message that we surface to the user.
  onValidate: (sk: Sketch): string | null => {
    try {
      const json = buildExtrudeModelJson(sk, [0, 0, 1], "_validate");
      // Just trying to evaluate exercises Sketch::to_profile_2d.
      evaluate_with_face_ids(json, "_validate", "{}", SEGMENTS);
      return null;
    } catch (e) {
      return String(e);
    }
  },
  // Splice the extruded sketch into the main 3D viewer by loading the
  // wrapped Model JSON the same way drag-drop does.
  onExtrude: (sk: Sketch, dir) => {
    const json = buildExtrudeModelJson(sk, dir, "out");
    loadJson(json);
  },
});

sketcherToggle.addEventListener("click", () => {
  sketcherPanel.classList.toggle("visible");
  if (sketcherPanel.classList.contains("visible")) {
    sketcher.redraw();
  }
});
sketcherClose.addEventListener("click", () => sketcherPanel.classList.remove("visible"));

// Keyboard shortcuts inside the sketcher (only when its panel is open).
window.addEventListener("keydown", (e) => {
  if (!sketcherPanel.classList.contains("visible")) return;
  sketcher.handleKey(e);
});

// Sketcher example loader.
document.querySelectorAll<HTMLAnchorElement>("[data-sk-example]").forEach((a) => {
  a.addEventListener("click", async () => {
    const name = a.dataset.skExample!;
    try {
      const r = await fetch(`/examples/${name}.json`);
      const json = await r.text();
      const sk = JSON.parse(json) as Sketch;
      sketcher.loadSketch(sk);
    } catch (e) {
      err(`could not load sketch example: ${e}`);
    }
  });
});

// --- feature-insertion toolbar ---
const toolbarHost = document.getElementById("toolbar")!;
const toolbar = mountToolbar(toolbarHost, {
  getModel: () => model ? { json: model.json, targetId: model.targetId } : null,
  setModel: (json: string, newTargetId?: string) => {
    try {
      const ids = target_ids_of(json) as string[];
      const params = (parameters_of(json) ?? {}) as Record<string, number>;
      const targetId = newTargetId && ids.includes(newTargetId)
        ? newTargetId
        : ids.includes("out") ? "out" : ids[ids.length - 1]!;
      model = {
        json,
        targetId,
        parameters: model ? { ...model.parameters, ...params } : { ...params },
        defaults: { ...params },
      };
      renderTargets(ids);
      renderFeatureTree();
      actionsEl.hidden = false;
      actions2El.hidden = false;
      viewsEl.hidden = false;
      rebuild(false);
    } catch (e) {
      err(String(e));
    }
  },
  openSketcher: () => {
    sketcherToggle.click();
  },
  getSelection: () => {
    if (highlightedEdge >= 0) return { edgeId: highlightedEdge };
    if (highlightedFace >= 0) return { faceId: highlightedFace };
    return null;
  },
  triggerFillet: () => {
    // Edge picking → fillet is handled interactively; hint user.
    ok("Click an axis-aligned edge on the model to add a fillet");
  },
  triggerChamfer: () => {
    ok("Click an edge on the model to add a chamfer");
  },
});

// Refresh toolbar disabled state whenever the model becomes available or
// goes away. `actionsEl.hidden` flips with model presence in every code
// path that mutates `model`, so observing it covers loadJson, file drops,
// example clicks, and the toolbar's own setModel callback.
new MutationObserver(() => toolbar.refresh()).observe(actionsEl, {
  attributes: true,
  attributeFilter: ["hidden"],
});
