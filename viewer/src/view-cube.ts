/**
 * view-cube.ts
 *
 * Interactive 3-D orientation cube rendered as a small overlay in the
 * top-right corner of the main canvas.  Each of the six labelled faces
 * is clickable; clicking snaps the main camera to look at the model
 * from that canonical direction.  The cube always rotates in sync with
 * the main camera so it acts as a real-time orientation indicator.
 *
 * Public surface
 * ──────────────
 * mountViewCube(container, mainCamera, orbitControls)
 *   Attaches the cube widget to `container`.  Call once during init;
 *   teardown happens when the container is removed from the DOM.
 *
 * viewCubeFaceFromName(name)
 *   Pure helper: maps a canonical face name → unit direction vector.
 *   Used by the click handler and exposed for unit testing.
 */

import * as THREE from "three";
import type { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";

// ---------------------------------------------------------------------------
// Pure helper – testable without WebGL
// ---------------------------------------------------------------------------

/** Maps a canonical face name to the unit direction vector FROM which the
 *  camera looks at the model origin.  E.g. "front" → (0,0,1) means the
 *  camera sits on the +Z axis looking toward the origin. */
export function viewCubeFaceFromName(
  name: string,
): THREE.Vector3 | null {
  switch (name.toLowerCase()) {
    case "front":  return new THREE.Vector3(0,  0,  1);
    case "back":   return new THREE.Vector3(0,  0, -1);
    case "top":    return new THREE.Vector3(0,  1,  0);
    case "bottom": return new THREE.Vector3(0, -1,  0);
    case "right":  return new THREE.Vector3(1,  0,  0);
    case "left":   return new THREE.Vector3(-1,  0,  0);
    default:       return null;
  }
}

// ---------------------------------------------------------------------------
// View-cube widget
// ---------------------------------------------------------------------------

const CUBE_SIZE = 80; // px – side length of the overlay canvas
const CUBE_DISTANCE = 2.5; // how far the cube camera sits from origin

const FACE_NAMES = ["front", "back", "top", "bottom", "right", "left"] as const;

/** Per-face colour palette – subtle dark theme matching the app. */
const FACE_COLORS: Record<string, number> = {
  front:  0x2a4a7f,
  back:   0x2a4a7f,
  top:    0x7f3a2a,
  bottom: 0x7f3a2a,
  right:  0x2a7f4a,
  left:   0x2a7f4a,
};

const FACE_HOVER_COLOR = 0x58a6ff;

/** Build the six mesh planes that make up the labelled cube. */
function buildCubeFaces(): THREE.Group {
  const group = new THREE.Group();

  const half = 0.5;
  /** [position offset, Euler rotation (x, y, z)] for each face. */
  const transforms: Record<string, [THREE.Vector3, THREE.Euler]> = {
    front:  [new THREE.Vector3(0, 0, half),  new THREE.Euler(0, 0, 0)],
    back:   [new THREE.Vector3(0, 0, -half), new THREE.Euler(0, Math.PI, 0)],
    top:    [new THREE.Vector3(0, half, 0),  new THREE.Euler(-Math.PI / 2, 0, 0)],
    bottom: [new THREE.Vector3(0, -half, 0), new THREE.Euler(Math.PI / 2, 0, 0)],
    right:  [new THREE.Vector3(half, 0, 0),  new THREE.Euler(0, Math.PI / 2, 0)],
    left:   [new THREE.Vector3(-half, 0, 0), new THREE.Euler(0, -Math.PI / 2, 0)],
  };

  for (const name of FACE_NAMES) {
    const [pos, rot] = transforms[name]!;

    // Plane geometry for the face.
    const geo = new THREE.PlaneGeometry(1, 1);
    const mat = new THREE.MeshBasicMaterial({
      color: FACE_COLORS[name],
      side: THREE.FrontSide,
    });
    const mesh = new THREE.Mesh(geo, mat);
    mesh.position.copy(pos);
    mesh.rotation.copy(rot);
    mesh.userData["face"] = name;

    group.add(mesh);

    // Thin white border around each face (wireframe box edge would show
    // all 12 edges; instead we add per-face edge lines).
    const edgeGeo = new THREE.EdgesGeometry(geo);
    const edgeMat = new THREE.LineBasicMaterial({ color: 0x4a6080 });
    const edges = new THREE.LineSegments(edgeGeo, edgeMat);
    edges.position.copy(pos);
    edges.rotation.copy(rot);
    // Nudge slightly forward to avoid z-fighting with the face.
    edges.position.addScaledVector(
      new THREE.Vector3(
        Math.sign(pos.x),
        Math.sign(pos.y),
        Math.sign(pos.z),
      ).normalize(),
      0.001,
    );
    group.add(edges);

    // Canvas-based text label.
    const labelTexture = makeLabelTexture(name);
    const labelGeo = new THREE.PlaneGeometry(0.7, 0.28);
    const labelMat = new THREE.MeshBasicMaterial({
      map: labelTexture,
      transparent: true,
      side: THREE.FrontSide,
      depthTest: false,
    });
    const label = new THREE.Mesh(labelGeo, labelMat);
    label.position.copy(pos);
    label.rotation.copy(rot);
    label.position.addScaledVector(
      new THREE.Vector3(
        Math.sign(pos.x),
        Math.sign(pos.y),
        Math.sign(pos.z),
      ).normalize(),
      0.002,
    );
    group.add(label);
  }

  return group;
}

function makeLabelTexture(name: string): THREE.CanvasTexture {
  const canvas = document.createElement("canvas");
  canvas.width = 128;
  canvas.height = 48;
  const ctx = canvas.getContext("2d")!;
  ctx.clearRect(0, 0, 128, 48);
  ctx.fillStyle = "#e6edf3";
  ctx.font = "bold 22px -apple-system, BlinkMacSystemFont, 'Segoe UI', system-ui, sans-serif";
  ctx.textAlign = "center";
  ctx.textBaseline = "middle";
  ctx.fillText(name.charAt(0).toUpperCase() + name.slice(1), 64, 24);
  return new THREE.CanvasTexture(canvas);
}

// ---------------------------------------------------------------------------
// Public mount function
// ---------------------------------------------------------------------------

export function mountViewCube(
  container: HTMLElement,
  mainCamera: THREE.Camera,
  orbitControls: OrbitControls,
): { dispose: () => void } {
  // ── Overlay canvas ────────────────────────────────────────────────────────
  const canvas = document.createElement("canvas");
  canvas.width  = CUBE_SIZE * window.devicePixelRatio;
  canvas.height = CUBE_SIZE * window.devicePixelRatio;
  canvas.style.cssText = `
    position: absolute;
    top: 12px;
    right: 12px;
    width: ${CUBE_SIZE}px;
    height: ${CUBE_SIZE}px;
    cursor: pointer;
    border-radius: 4px;
    z-index: 10;
  `;
  container.style.position ||= "relative";
  container.appendChild(canvas);

  // ── Cube renderer ────────────────────────────────────────────────────────
  const cubeRenderer = new THREE.WebGLRenderer({
    canvas,
    antialias: true,
    alpha: true,
  });
  cubeRenderer.setPixelRatio(window.devicePixelRatio);
  cubeRenderer.setSize(CUBE_SIZE, CUBE_SIZE, false);
  cubeRenderer.setClearColor(0x000000, 0);

  const cubeScene = new THREE.Scene();
  const cubeCamera = new THREE.PerspectiveCamera(45, 1, 0.1, 100);
  cubeCamera.position.set(0, 0, CUBE_DISTANCE);

  cubeScene.add(new THREE.AmbientLight(0xffffff, 1.2));

  const cubeFaces = buildCubeFaces();
  cubeScene.add(cubeFaces);

  // ── Raycaster for click detection ────────────────────────────────────────
  const raycaster = new THREE.Raycaster();
  const mouse = new THREE.Vector2();
  let hoveredFaceMesh: THREE.Mesh | null = null;

  function getFaceMeshes(): THREE.Mesh[] {
    return cubeFaces.children.filter(
      (c): c is THREE.Mesh => c instanceof THREE.Mesh && c.userData["face"],
    );
  }

  function setHover(mesh: THREE.Mesh | null) {
    if (hoveredFaceMesh === mesh) return;
    if (hoveredFaceMesh) {
      (hoveredFaceMesh.material as THREE.MeshBasicMaterial).color.setHex(
        FACE_COLORS[hoveredFaceMesh.userData["face"] as string]!,
      );
    }
    hoveredFaceMesh = mesh;
    if (hoveredFaceMesh) {
      (hoveredFaceMesh.material as THREE.MeshBasicMaterial).color.setHex(FACE_HOVER_COLOR);
    }
  }

  function hitTestFace(event: MouseEvent): THREE.Mesh | null {
    const rect = canvas.getBoundingClientRect();
    mouse.x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
    mouse.y = -((event.clientY - rect.top) / rect.height) * 2 + 1;
    raycaster.setFromCamera(mouse, cubeCamera);
    const hits = raycaster.intersectObjects(getFaceMeshes(), false);
    return hits.length > 0 ? (hits[0]!.object as THREE.Mesh) : null;
  }

  canvas.addEventListener("mousemove", (e) => {
    setHover(hitTestFace(e));
  });

  canvas.addEventListener("mouseleave", () => setHover(null));

  canvas.addEventListener("click", (e) => {
    const mesh = hitTestFace(e);
    if (!mesh) return;
    const faceName = mesh.userData["face"] as string;
    snapToFace(faceName);
  });

  function snapToFace(faceName: string) {
    const dir = viewCubeFaceFromName(faceName);
    if (!dir) return;

    const target = (orbitControls.target as THREE.Vector3).clone();

    // Estimate distance: keep current camera distance from target.
    const dist = mainCamera.position.distanceTo(target) || 150;

    // For "top"/"bottom" avoid gimbal lock in OrbitControls — add a tiny
    // +Z tilt so the up-vector never aligns with the look direction.
    const nudgedDir = dir.clone();
    if (Math.abs(dir.y) > 0.99) {
      nudgedDir.z += 0.001;
      nudgedDir.normalize();
    }

    mainCamera.position.copy(target.clone().add(nudgedDir.multiplyScalar(dist)));
    orbitControls.update();
  }

  // ── Sync cube rotation with the main camera each frame ───────────────────
  // We copy the main camera's world quaternion into the cube scene's root
  // group (inverted), which keeps the cube faces aligned with world axes as
  // the main camera orbits.
  function syncCube() {
    // Compute the main camera's rotation relative to world space.
    const q = new THREE.Quaternion();
    mainCamera.getWorldQuaternion(q);
    // Apply the inverse to the cube group so it counter-rotates.
    cubeFaces.setRotationFromQuaternion(q.invert());
    cubeRenderer.render(cubeScene, cubeCamera);
  }

  // ── Attach to the main animation loop ────────────────────────────────────
  // We hook into the renderer's requestAnimationFrame via a separate loop
  // so we don't depend on internals of main.ts's tick().
  let rafId = 0;
  function loop() {
    rafId = requestAnimationFrame(loop);
    syncCube();
  }
  loop();

  // ── Cleanup ───────────────────────────────────────────────────────────────
  function dispose() {
    cancelAnimationFrame(rafId);
    cubeRenderer.dispose();
    canvas.remove();
  }

  return { dispose };
}
