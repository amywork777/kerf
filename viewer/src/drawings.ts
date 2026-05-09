// Multi-view orthographic snapshot export — front/top/right + iso, composed
// into a single PNG with labels, overall bounding-box dimensions, and a
// SolidWorks-style hole table when the model contains hole features.

import * as THREE from "three";
import { renderGdtOverlay } from "./gdt.js";
import type { GdtAnnotations } from "./gdt.js";

const VIEW_W = 480;
const VIEW_H = 360;
const PAD = 50;
const LABEL_H = 28;
const SHEET_W = VIEW_W * 2 + PAD * 3;
const SHEET_H = VIEW_H * 2 + PAD * 3 + LABEL_H * 2 + 30;

type ViewSpec = {
  name: string;
  // unit vector pointing FROM the camera TO the target
  forward: THREE.Vector3;
  // up direction in world space
  up: THREE.Vector3;
  // which two world axes appear as horizontal / vertical screen axes for
  // this projection — used to draw correct dimension labels per view
  hAxis: "x" | "y" | "z" | null;
  vAxis: "x" | "y" | "z" | null;
};

const VIEWS: ViewSpec[] = [
  { name: "FRONT (-Y)",  forward: new THREE.Vector3( 0, -1,  0), up: new THREE.Vector3(0, 0, 1), hAxis: "x", vAxis: "z" },
  { name: "TOP (-Z)",    forward: new THREE.Vector3( 0,  0, -1), up: new THREE.Vector3(0, 1, 0), hAxis: "x", vAxis: "y" },
  { name: "RIGHT (-X)",  forward: new THREE.Vector3(-1,  0,  0), up: new THREE.Vector3(0, 0, 1), hAxis: "y", vAxis: "z" },
  { name: "ISO",         forward: new THREE.Vector3(-1, -0.7, -1.2).normalize(), up: new THREE.Vector3(0, 0, 1), hAxis: null, vAxis: null },
];

export function exportThreeViewPng(
  mesh: THREE.Mesh,
  fileName: string,
  gdt?: GdtAnnotations,
) {
  const sceneClone = new THREE.Scene();
  sceneClone.background = new THREE.Color(0xffffff);
  sceneClone.add(new THREE.AmbientLight(0xffffff, 0.7));
  const lightA = new THREE.DirectionalLight(0xffffff, 0.7);
  lightA.position.set(50, 80, 60);
  sceneClone.add(lightA);
  const lightB = new THREE.DirectionalLight(0xffffff, 0.3);
  lightB.position.set(-50, -80, -60);
  sceneClone.add(lightB);

  const drawingMat = new THREE.MeshStandardMaterial({
    color: 0xc8d3df,
    metalness: 0.0,
    roughness: 0.7,
    flatShading: true,
    side: THREE.DoubleSide,
  });
  const proxy = new THREE.Mesh(mesh.geometry, drawingMat);
  sceneClone.add(proxy);

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
  ctx.fillStyle = "#161b22";

  const cells = [
    { col: 0, row: 0, view: VIEWS[0]! },
    { col: 1, row: 0, view: VIEWS[1]! },
    { col: 0, row: 1, view: VIEWS[2]! },
    { col: 1, row: 1, view: VIEWS[3]! },
  ];

  // Camera frustum half-widths in world units. Used both for rendering and
  // for converting world-units → pixels when drawing dimensions.
  const fovHalfW = span * 0.7;
  const fovHalfH = span * 0.55;

  for (const cell of cells) {
    const camera = new THREE.OrthographicCamera(
      -fovHalfW, fovHalfW, fovHalfH, -fovHalfH, -span * 4, span * 4,
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
    ctx.font = "bold 14px -apple-system, BlinkMacSystemFont, system-ui, sans-serif";
    ctx.fillStyle = "#161b22";
    ctx.fillText(cell.view.name, x, y - 6);

    // Dimensions overlay: draw arrows for the model's bounding extent in
    // each of the two screen axes for this projection.
    if (cell.view.hAxis && cell.view.vAxis) {
      drawDimensions(ctx, x, y, VIEW_W, VIEW_H, fovHalfW, fovHalfH, size, cell.view);
    }

    // GD&T overlay: render datum refs, control frames, and surface finishes
    // into an SVG layer, then stamp it onto the canvas via a data-URL image.
    if (gdt) {
      const pxPerWorldH = VIEW_W / (2 * fovHalfW);
      const pxPerWorldV = VIEW_H / (2 * fovHalfH);
      const cx = x + VIEW_W / 2;
      const cy = y + VIEW_H / 2;

      // Build projection that maps model coords → view pixel coords.
      // We need to know which two world axes map to screen H and V for this
      // view — same logic dimensions.ts uses.
      const forward = cell.view.forward;
      const up = cell.view.up;
      // right = up × forward (screen horizontal axis in world space)
      const right = new THREE.Vector3().crossVectors(up, forward).normalize();

      const viewMatrix = {
        project(p: [number, number, number]): [number, number] {
          const world = new THREE.Vector3(p[0], p[1], p[2]).sub(center);
          const sx = world.dot(right) * pxPerWorldH;
          const sy = -world.dot(up) * pxPerWorldV; // flip Y for canvas
          return [cx + sx, cy + sy];
        },
      };

      const svgEl = document.createElementNS("http://www.w3.org/2000/svg", "svg") as unknown as SVGElement;
      renderGdtOverlay(svgEl, gdt, viewMatrix);

      // Serialize SVG to data-URL and draw onto canvas.
      const svgStr = new XMLSerializer().serializeToString(svgEl as unknown as Node);
      const blob = new Blob([svgStr], { type: "image/svg+xml" });
      const url = URL.createObjectURL(blob);
      // We draw the GD&T layer synchronously using an <img> trick is not
      // possible in all browsers within a single tick. Instead we stamp it
      // asynchronously after the PNG is otherwise complete (best-effort).
      const img = new Image();
      img.onload = () => {
        ctx.drawImage(img, x, y, VIEW_W, VIEW_H);
        URL.revokeObjectURL(url);
      };
      img.src = url;
    }
  }

  // Title block.
  const titleY = SHEET_H - 18;
  ctx.font = "12px ui-monospace, 'SF Mono', Menlo, monospace";
  ctx.fillStyle = "#161b22";
  ctx.fillText(
    `${fileName}   |   bbox X×Y×Z = ${size.x.toFixed(2)} × ${size.y.toFixed(2)} × ${size.z.toFixed(2)}`,
    PAD,
    titleY,
  );
  ctx.fillStyle = "#8b949e";
  ctx.fillText(`generated by kerf-cad viewer`, SHEET_W - 220, titleY);

  // Hole table — rendered after dimensions, appended below the title block.
  // Graceful no-op when the model has no holes or no modelJson was provided.
  if (modelJson) {
    const holes = extractHoles(modelJson);
    if (holes.length > 0) {
      // Extend the canvas height to fit the table (22 px per row + 22 header + 24 label gap).
      const tableRows = holes.length + 1; // header + data
      const tableH = tableRows * 22 + 30;
      const oldHeight = sheet.height;
      sheet.height = oldHeight + tableH;
      // Re-fill background for the new area.
      ctx.fillStyle = "#ffffff";
      ctx.fillRect(0, oldHeight, sheet.width, tableH);
      renderHoleTable(ctx, holes, oldHeight + 24, PAD);
    }
  }

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

function drawDimensions(
  ctx: CanvasRenderingContext2D,
  x0: number,
  y0: number,
  w: number,
  h: number,
  fovHalfW: number,
  fovHalfH: number,
  size: THREE.Vector3,
  view: ViewSpec,
) {
  const sizeOnH = view.hAxis ? size[view.hAxis] : 0;
  const sizeOnV = view.vAxis ? size[view.vAxis] : 0;

  // Project the model's horizontal/vertical extent onto pixels.
  const pxPerWorldH = w / (2 * fovHalfW);
  const pxPerWorldV = h / (2 * fovHalfH);
  const widthPx = sizeOnH * pxPerWorldH;
  const heightPx = sizeOnV * pxPerWorldV;

  const cx = x0 + w / 2;
  const cy = y0 + h / 2;

  ctx.strokeStyle = "#3f6cba";
  ctx.fillStyle = "#3f6cba";
  ctx.lineWidth = 1;
  ctx.font = "11px -apple-system, BlinkMacSystemFont, system-ui, sans-serif";

  // Horizontal dimension below the view.
  const dimY = y0 + h + 14;
  arrow(ctx, cx - widthPx / 2, dimY, cx + widthPx / 2, dimY);
  ctx.fillText(`${view.hAxis!.toUpperCase()} ${sizeOnH.toFixed(2)}`,
    cx - 22, dimY + 14);

  // Vertical dimension on the left of the view.
  const dimX = x0 - 14;
  arrow(ctx, dimX, cy - heightPx / 2, dimX, cy + heightPx / 2);
  ctx.save();
  ctx.translate(dimX - 14, cy + 22);
  ctx.rotate(-Math.PI / 2);
  ctx.fillText(`${view.vAxis!.toUpperCase()} ${sizeOnV.toFixed(2)}`, 0, 0);
  ctx.restore();
}

function arrow(ctx: CanvasRenderingContext2D, x1: number, y1: number, x2: number, y2: number) {
  const head = 5;
  const dx = x2 - x1;
  const dy = y2 - y1;
  const len = Math.hypot(dx, dy);
  if (len < 1) return;
  const ux = dx / len;
  const uy = dy / len;
  // Shaft
  ctx.beginPath();
  ctx.moveTo(x1, y1);
  ctx.lineTo(x2, y2);
  ctx.stroke();
  // Arrowhead at end
  ctx.beginPath();
  ctx.moveTo(x2, y2);
  ctx.lineTo(x2 - head * ux + head * uy, y2 - head * uy - head * ux);
  ctx.lineTo(x2 - head * ux - head * uy, y2 - head * uy + head * ux);
  ctx.closePath();
  ctx.fill();
  // Arrowhead at start (reversed)
  ctx.beginPath();
  ctx.moveTo(x1, y1);
  ctx.lineTo(x1 + head * ux + head * uy, y1 + head * uy - head * ux);
  ctx.lineTo(x1 + head * ux - head * uy, y1 + head * uy + head * ux);
  ctx.closePath();
  ctx.fill();
}
