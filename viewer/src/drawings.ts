// Multi-view orthographic snapshot export — front/top/right + iso, composed
// into a single PNG with labels and overall bounding-box dimensions.

import * as THREE from "three";

const VIEW_W = 480;
const VIEW_H = 360;
const PAD = 40;
const LABEL_H = 28;
const SHEET_W = VIEW_W * 2 + PAD * 3;
const SHEET_H = VIEW_H * 2 + PAD * 3 + LABEL_H * 2;

type ViewSpec = {
  name: string;
  // unit vector pointing FROM the camera TO the target
  forward: THREE.Vector3;
  // up direction in world space
  up: THREE.Vector3;
};

const VIEWS: ViewSpec[] = [
  { name: "FRONT (-Y)",  forward: new THREE.Vector3( 0, -1,  0), up: new THREE.Vector3(0, 0, 1) },
  { name: "TOP (-Z)",    forward: new THREE.Vector3( 0,  0, -1), up: new THREE.Vector3(0, 1, 0) },
  { name: "RIGHT (-X)",  forward: new THREE.Vector3(-1,  0,  0), up: new THREE.Vector3(0, 0, 1) },
  { name: "ISO",         forward: new THREE.Vector3(-1, -0.7, -1.2).normalize(), up: new THREE.Vector3(0, 0, 1) },
];

export function exportThreeViewPng(mesh: THREE.Mesh, fileName: string) {
  const sceneClone = new THREE.Scene();
  sceneClone.background = new THREE.Color(0xffffff);
  sceneClone.add(new THREE.AmbientLight(0xffffff, 0.7));
  const lightA = new THREE.DirectionalLight(0xffffff, 0.7);
  lightA.position.set(50, 80, 60);
  sceneClone.add(lightA);
  const lightB = new THREE.DirectionalLight(0xffffff, 0.3);
  lightB.position.set(-50, -80, -60);
  sceneClone.add(lightB);

  // Replace material with a clean-line drawing material; keep geometry by ref.
  const drawingMat = new THREE.MeshStandardMaterial({
    color: 0xc8d3df,
    metalness: 0.0,
    roughness: 0.7,
    flatShading: true,
    side: THREE.DoubleSide,
  });
  const proxy = new THREE.Mesh(mesh.geometry, drawingMat);
  sceneClone.add(proxy);

  // Crisp visible-edge overlay so the drawing reads like a CAD sheet.
  const edges = new THREE.EdgesGeometry(mesh.geometry, 12);
  const lines = new THREE.LineSegments(
    edges,
    new THREE.LineBasicMaterial({ color: 0x222831 }),
  );
  sceneClone.add(lines);

  const box = new THREE.Box3().setFromObject(proxy);
  const center = box.getCenter(new THREE.Vector3());
  const size = box.getSize(new THREE.Vector3());
  const span = Math.max(size.x, size.y, size.z, 1e-6);

  const renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setSize(VIEW_W, VIEW_H, false);
  renderer.setClearColor(0xffffff, 1);

  const sheet = document.createElement("canvas");
  sheet.width = SHEET_W;
  sheet.height = SHEET_H;
  const ctx = sheet.getContext("2d")!;
  ctx.fillStyle = "#ffffff";
  ctx.fillRect(0, 0, SHEET_W, SHEET_H);
  ctx.font = "bold 14px -apple-system, BlinkMacSystemFont, system-ui, sans-serif";
  ctx.fillStyle = "#161b22";

  const cells = [
    { col: 0, row: 0, view: VIEWS[0]! },
    { col: 1, row: 0, view: VIEWS[1]! },
    { col: 0, row: 1, view: VIEWS[2]! },
    { col: 1, row: 1, view: VIEWS[3]! },
  ];

  for (const cell of cells) {
    const camera = new THREE.OrthographicCamera(
      -span * 0.7, span * 0.7,
       span * 0.55, -span * 0.55,
       -span * 4, span * 4,
    );
    camera.up.copy(cell.view.up);
    const eye = center.clone().sub(cell.view.forward.clone().multiplyScalar(span * 2));
    camera.position.copy(eye);
    camera.lookAt(center);
    renderer.render(sceneClone, camera);

    const x = PAD + cell.col * (VIEW_W + PAD);
    const y = PAD + LABEL_H + cell.row * (VIEW_H + PAD + LABEL_H);
    ctx.drawImage(renderer.domElement, x, y);
    ctx.strokeStyle = "#30363d";
    ctx.lineWidth = 1;
    ctx.strokeRect(x + 0.5, y + 0.5, VIEW_W - 1, VIEW_H - 1);
    ctx.fillText(cell.view.name, x, y - 6);
  }

  // Title block + dimensions strip in the bottom margin.
  const titleY = SHEET_H - PAD / 2;
  ctx.font = "12px ui-monospace, 'SF Mono', Menlo, monospace";
  ctx.fillStyle = "#161b22";
  ctx.fillText(
    `${fileName}   |   bbox X×Y×Z = ${size.x.toFixed(2)} × ${size.y.toFixed(2)} × ${size.z.toFixed(2)}`,
    PAD,
    titleY,
  );

  renderer.dispose();
  edges.dispose();
  lines.geometry.dispose();
  (lines.material as THREE.Material).dispose();
  drawingMat.dispose();

  sheet.toBlob((blob) => {
    if (!blob) return;
    const url = URL.createObjectURL(blob);
    const a = document.createElement("a");
    a.href = url;
    a.download = `${fileName}.drawing.png`;
    document.body.appendChild(a);
    a.click();
    a.remove();
    URL.revokeObjectURL(url);
  }, "image/png");
}
