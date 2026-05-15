// Interactive 2D sketcher panel.
// Import edit ops (copy/paste, mirror, pattern). Glyph rendering is
// delegated to sketcher-glyphs.ts.
import { copySelection, pasteFragment, mirrorSelection, patternRect } from "./sketcher-edit-ops.js";
//
// Self-contained module: renders a `<canvas>` for sketch authoring, drives a
// tool state machine (Point / Line / Circle / Arc / Pan / Select / Delete),
// and round-trips with the rust-side Sketch JSON DSL (see
// crates/kerf-cad/src/sketch.rs). The output is a `Sketch` object literally
// shaped like the rust serde tag layout so it splices into a Model JSON for
// `Feature::SketchExtrude` without further translation.
//
// Constraints are stored alongside primitives as `SketchConstraint` JSON. They
// are NOT solved here — the rust kernel doesn't have a solver yet either.
// The sketcher draws constraint markers (e.g. an "H" near a Horizontal line)
// so the user knows the constraint is recorded and will survive a JSON
// round-trip.
//
// Dependencies: none (pure DOM + 2D canvas). Validation calls back into the
// host page via `onValidate` (which wraps the WASM evaluator). Extruding a
// sketch likewise calls `onExtrude` with a constructed Model JSON; the host
// loads it into the 3D viewer.
//
// Undo/redo: history is managed by SketchHistory (sketcher-history.ts).
// Cmd+Z / Ctrl+Z = undo; Shift+Cmd+Z / Ctrl+Y = redo.
//
// Dimension entry: when the user adds a Distance constraint via the context
// menu an inline numeric input (sketcher-dim-entry.ts) appears near the
// anchor so the user can type an exact value.

import { SketchHistory } from "./sketcher-history.js";
import { promptDistance } from "./sketcher-dim-entry.js";

import { renderConstraintGlyphs } from "./sketcher-glyphs.js";
import { findSnap, type SnapResult } from "./sketcher-snap.js";
import { computeDof, formatDof } from "./sketcher-dof.js";

export type SketchPlane =
  | "Xy"
  | "Xz"
  | "Yz"
  | { NamedRefPlane: string };

// Discriminated union matching the rust serde tag layout: each variant has
// a `kind` field, plus its own data. We keep `id` always present and the
// rest as the variant-specific fields.
export type SketchPrim =
  | { kind: "Point"; id: string; x: number; y: number }
  | { kind: "Line"; id: string; from: string; to: string }
  | { kind: "Circle"; id: string; center: string; radius: number; n_segments: number }
  | {
      kind: "Arc";
      id: string;
      center: string;
      radius: number;
      start_angle: number;
      end_angle: number;
      n_segments: number;
    }
  // 2D editing operations: rewritten before loop tracing in the rust
  // sketcher. The TS canvas does not yet PROPOSE these (no dedicated
  // tool), but importing JSON containing them round-trips correctly.
  | { kind: "TrimLine"; id: string; line: string; at_point: string }
  | { kind: "ExtendLine"; id: string; line: string; to_point: string }
  | { kind: "FilletCorner"; id: string; corner_point: string; radius: number };

export type SketchConstraint =
  | { kind: "Coincident"; a: string; b: string }
  | { kind: "Distance"; a: string; b: string; value: number }
  | { kind: "Horizontal"; line: string }
  | { kind: "Vertical"; line: string }
  | { kind: "Parallel"; line_a: string; line_b: string }
  | { kind: "Perpendicular"; line_a: string; line_b: string }
  | { kind: "FixedPoint"; point: string };

export type Sketch = {
  plane: SketchPlane;
  primitives: SketchPrim[];
  constraints: SketchConstraint[];
};

export type Tool =
  | "point"
  | "line"
  | "circle"
  | "arc"
  | "pan"
  | "select"
  | "delete"
  | "trim"
  | "extend"
  | "fillet"
  | "mirror"
  | "pattern-rect";

type View = {
  // World origin in canvas pixels (after pan/zoom).
  ox: number;
  oy: number;
  // Pixels per world unit.
  scale: number;
};

// Snapshot of the canvas-space world coordinate of a click. Stored without
// snap so we can show the mouse-cursor crosshair in true coords; snapping
// is applied lazily when committing a primitive.
type Pt = { x: number; y: number };

export type SketcherOptions = {
  /** Validate a sketch against the rust loop tracer. Returns null if OK,
   *  else an error message. The host wires this to the WASM evaluator. */
  onValidate?: (sketch: Sketch) => string | null;
  /** Build a Model JSON wrapping the sketch in a SketchExtrude feature and
   *  load it into the 3D viewer. Caller is responsible for splicing the
   *  result into the main scene. */
  onExtrude?: (sketch: Sketch, direction: [number, number, number]) => void;
  /** Initial grid spacing in world units. */
  gridSpacing?: number;
};

/**
 * Mount the sketcher into the given container.
 * Returns a controller for programmatic access (load/save/reset).
 */
export function mountSketcher(host: HTMLElement, opts: SketcherOptions = {}) {
  // ---- DOM ----
  host.innerHTML = `
    <div class="sk-toolbar">
      <button data-tool="point" title="Point (P)">·</button>
      <button data-tool="line" title="Line (L)">/</button>
      <button data-tool="circle" title="Circle (C)">○</button>
      <button data-tool="arc" title="Arc (A)">◜</button>
      <button data-tool="pan" title="Pan (space)">✣</button>
      <button data-tool="select" title="Select (S)">▢</button>
      <button data-tool="delete" title="Delete (D)">✕</button>
    </div>
    <div class="sk-toolbar sk-edit-toolbar">
      <button data-tool="trim" title="Trim: click a Line, then click the trim point on it">Trim</button>
      <button data-tool="extend" title="Extend: click a Line, then click the target point">Extend</button>
      <button data-tool="fillet" title="Fillet: click a corner Point, then enter a radius">Fillet</button>
      <button data-tool="mirror" title="Mirror: pick axis (two clicks), then primitives are reflected">Mirror</button>
      <button data-tool="pattern-rect" title="Pattern Rect: select primitives, then enter dx/dy/nx/ny">Pattern Rect</button>
    </div>
    <canvas class="sk-canvas" width="400" height="400"></canvas>
    <div class="sk-row">
      <label>grid <input class="sk-grid" type="number" min="0.1" step="0.1" value="${opts.gridSpacing ?? 1}" style="width: 60px"></label>
      <label><input type="checkbox" class="sk-snap" checked> snap</label>
      <span class="sk-zoom" style="color: var(--muted); font-size: 11px;"></span>
    </div>
    <div class="sk-row">
      <button class="sk-validate">Validate</button>
      <button class="sk-extrude">Extrude →</button>
    </div>
    <div class="sk-row">
      <button class="sk-load">Load JSON</button>
      <button class="sk-save">Save JSON</button>
      <button class="sk-clear" title="Clear sketch">Clear</button>
    </div>
    <div class="sk-status">click on the canvas to place primitives</div>
    <div class="sk-prims" style="font-family: ui-monospace,Menlo,monospace; font-size: 10px; color: var(--muted); margin-top: 4px; max-height: 80px; overflow-y: auto;"></div>
  `;

  const canvas = host.querySelector(".sk-canvas") as HTMLCanvasElement;
  const ctx = canvas.getContext("2d")!;
  const status = host.querySelector(".sk-status") as HTMLDivElement;
  const primsList = host.querySelector(".sk-prims") as HTMLDivElement;
  const gridInput = host.querySelector(".sk-grid") as HTMLInputElement;
  const snapInput = host.querySelector(".sk-snap") as HTMLInputElement;
  const zoomLbl = host.querySelector(".sk-zoom") as HTMLSpanElement;
  const toolbarBtns = Array.from(
    host.querySelectorAll<HTMLButtonElement>(".sk-toolbar button[data-tool]"),
  );

  // ---- state ----
  let plane: SketchPlane = "Xy";
  const primitives = new Map<string, SketchPrim>();
  let constraints: SketchConstraint[] = [];
  let currentTool: Tool = "point";
  // Two-step tool state (line: first click picks/places start, second picks/places end).
  let pendingStartId: string | null = null;
  let circleCenterId: string | null = null;
  let arcCenterId: string | null = null;
  let arcStartAngle: number | null = null;
  // Trim / Extend tools: first click selects a Line; second click picks
  // the trim-at / extend-to point. We snap the second click to either an
  // existing Point (preferred) or place a new Point at the click location
  // (which lands on the line since the user is asked to click on it).
  let trimSelectedLineId: string | null = null;
  let extendSelectedLineId: string | null = null;

  // Copy/paste clipboard: stores a SketchFragment of copied primitives and
  // the last paste offset so successive Cmd+V pastes keep shifting by 10px.
  let clipboardPrims: SketchPrim[] = [];
  let lastPasteOffset: { x: number; y: number } = { x: 10, y: 10 };

  // Mirror tool: two-click axis pick (first click = axis start, second = axis end).
  let mirrorAxisStart: { x: number; y: number } | null = null;
  // Mirror/PatternRect: the selected primitive ids to operate on.
  let editOpIds: string[] = [];

  // Selection / drag state.
  let selectedId: string | null = null;
  let selectedIds: Set<string> = new Set();
  let dragPointId: string | null = null;
  let dragPanStart: { x: number; y: number; ox: number; oy: number } | null = null;

  // Hover (for visual feedback + endpoint snapping).
  let hoverPointId: string | null = null;
  // For Trim / Extend / Fillet, we hover-highlight the prim under the
  // cursor. Tracked separately so primitive picking shows feedback even
  // when no Point is nearby (e.g. hovering over a line's interior).
  let hoverPrimId: string | null = null;
  let mouseWorld: Pt | null = null;
  // Active snap result updated on every mousemove while drawing.
  let activeSnap: SnapResult | null = null;

  // View transform.
  const view: View = { ox: 200, oy: 200, scale: 30 };

  // Auto-id counter — reset on clear/load so ids are predictable.
  let nextSeq = 1;

  // ---- undo/redo ----
  const history = new SketchHistory(50);

  /** Snapshot the current sketch state onto the undo stack. Call this
   *  immediately BEFORE each mutation so undo can restore the prior state. */
  function pushHistory() {
    history.push(toSketch());
  }

  function applySketch(sk: Sketch) {
    primitives.clear();
    for (const prim of sk.primitives) primitives.set(prim.id, prim);
    constraints = [...(sk.constraints ?? [])];
    plane = sk.plane;
    // Recalibrate nextSeq to avoid id collisions.
    let maxSeq = 0;
    for (const id of primitives.keys()) {
      const m = id.match(/(\d+)$/);
      if (m) maxSeq = Math.max(maxSeq, Number(m[1]));
    }
    nextSeq = maxSeq + 1;
    redraw();
    refreshPrimsList();
  }

  // ---- coordinate transforms ----
  const worldToCanvas = (x: number, y: number): [number, number] => [
    view.ox + x * view.scale,
    view.oy - y * view.scale, // y flipped so +y is up
  ];
  const canvasToWorld = (cx: number, cy: number): Pt => ({
    x: (cx - view.ox) / view.scale,
    y: (view.oy - cy) / view.scale,
  });

  // ---- snap helpers ----
  function snapWorld(p: Pt): Pt {
    if (!snapInput.checked) return p;
    const g = Number(gridInput.value) || 1;
    return { x: Math.round(p.x / g) * g, y: Math.round(p.y / g) * g };
  }

  // Return id of the nearest existing Point within `pixelRadius` canvas px,
  // or null. Used to allow line/arc tools to attach to existing points.
  function pickNearestPointId(p: Pt, pixelRadius = 10): string | null {
    let best: string | null = null;
    let bestD = Infinity;
    const r = pixelRadius / view.scale;
    for (const prim of primitives.values()) {
      if (prim.kind !== "Point") continue;
      const dx = prim.x - p.x;
      const dy = prim.y - p.y;
      const d = Math.hypot(dx, dy);
      if (d < r && d < bestD) {
        bestD = d;
        best = prim.id;
      }
    }
    return best;
  }

  // Pick the nearest Line primitive only. Used by Trim / Extend tools.
  function pickNearestLineId(p: Pt, pixelRadius = 10): string | null {
    const r = pixelRadius / view.scale;
    let best: string | null = null;
    let bestD = Infinity;
    for (const prim of primitives.values()) {
      if (prim.kind !== "Line") continue;
      const a = primitives.get(prim.from);
      const b = primitives.get(prim.to);
      if (!a || a.kind !== "Point" || !b || b.kind !== "Point") continue;
      const d = pointSegmentDistance(p, a, b);
      if (d < r && d < bestD) {
        bestD = d;
        best = prim.id;
      }
    }
    return best;
  }

  // Project a world point onto a Line primitive (clamped to segment).
  // Returns null if the line is degenerate or the line/endpoints don't
  // exist.
  function projectPointOntoLine(p: Pt, lineId: string): Pt | null {
    const prim = primitives.get(lineId);
    if (!prim || prim.kind !== "Line") return null;
    const a = primitives.get(prim.from);
    const b = primitives.get(prim.to);
    if (!a || a.kind !== "Point" || !b || b.kind !== "Point") return null;
    const ax = b.x - a.x;
    const ay = b.y - a.y;
    const len2 = ax * ax + ay * ay;
    if (len2 < 1e-12) return null;
    let t = ((p.x - a.x) * ax + (p.y - a.y) * ay) / len2;
    t = Math.max(0, Math.min(1, t));
    return { x: a.x + t * ax, y: a.y + t * ay };
  }

  function pickNearestPrimId(p: Pt, pixelRadius = 10): string | null {
    // Prefer a Point hit, then check Lines/Circles/Arcs by approximate distance.
    const hitPt = pickNearestPointId(p, pixelRadius);
    if (hitPt) return hitPt;
    const r = pixelRadius / view.scale;
    let best: string | null = null;
    let bestD = Infinity;
    for (const prim of primitives.values()) {
      if (prim.kind === "Line") {
        const a = primitives.get(prim.from);
        const b = primitives.get(prim.to);
        if (!a || a.kind !== "Point" || !b || b.kind !== "Point") continue;
        const d = pointSegmentDistance(p, a, b);
        if (d < r && d < bestD) {
          bestD = d;
          best = prim.id;
        }
      } else if (prim.kind === "Circle") {
        const c = primitives.get(prim.center);
        if (!c || c.kind !== "Point") continue;
        const d = Math.abs(Math.hypot(p.x - c.x, p.y - c.y) - prim.radius);
        if (d < r && d < bestD) {
          bestD = d;
          best = prim.id;
        }
      } else if (prim.kind === "Arc") {
        const c = primitives.get(prim.center);
        if (!c || c.kind !== "Point") continue;
        const d = Math.abs(Math.hypot(p.x - c.x, p.y - c.y) - prim.radius);
        // Check angular range too.
        const ang = Math.atan2(p.y - c.y, p.x - c.x);
        const inRange = isAngleBetween(ang, prim.start_angle, prim.end_angle);
        if (inRange && d < r && d < bestD) {
          bestD = d;
          best = prim.id;
        }
      }
    }
    return best;
  }

  // ---- id generation ----
  function freshId(prefix: string): string {
    while (true) {
      const candidate = `${prefix}${nextSeq++}`;
      if (!primitives.has(candidate)) return candidate;
    }
  }

  // ---- mutations ----
  function addPrim(prim: SketchPrim) {
    pushHistory(); // snapshot before mutation
    primitives.set(prim.id, prim);
    redraw();
    refreshPrimsList();
  }

  // Add a Point at a given world position, OR return the id of an existing
  // Point at that snap location. Used so two clicks in the line tool that
  // happen to land on the same snap cell don't make duplicate Points.
  function addOrReusePoint(p: Pt): string {
    const snapped = snapWorld(p);
    // First: snap to existing point if close.
    const existing = pickNearestPointId(snapped, 8);
    if (existing) return existing;
    const id = freshId("p");
    addPrim({ kind: "Point", id, x: snapped.x, y: snapped.y });
    return id;
  }

  function deletePrim(id: string) {
    const prim = primitives.get(id);
    if (!prim) return;
    pushHistory(); // snapshot before mutation
    primitives.delete(id);
    // Cascade: delete any line/arc/circle that references this point.
    for (const [other_id, other] of Array.from(primitives.entries())) {
      if (other.kind === "Line" && (other.from === id || other.to === id)) {
        primitives.delete(other_id);
      } else if (
        (other.kind === "Circle" || other.kind === "Arc") &&
        other.center === id
      ) {
        primitives.delete(other_id);
      }
    }
    // Drop any constraint that references the deleted id.
    constraints = constraints.filter((c) => !constraintRefs(c).includes(id));
    if (selectedId === id) selectedId = null;
    redraw();
    refreshPrimsList();
  }

  function clearSketch() {
    pushHistory(); // snapshot before clearing
    primitives.clear();
    constraints = [];
    nextSeq = 1;
    pendingStartId = null;
    circleCenterId = null;
    arcCenterId = null;
    arcStartAngle = null;
    selectedId = null;
    redraw();
    refreshPrimsList();
    setStatus("cleared");
  }

  // ---- tool actions ----
  function setTool(t: Tool) {
    currentTool = t;
    pendingStartId = null;
    circleCenterId = null;
    arcCenterId = null;
    arcStartAngle = null;
    trimSelectedLineId = null;
    extendSelectedLineId = null;
    mirrorAxisStart = null;
    editOpIds = [];
    for (const btn of toolbarBtns) {
      btn.classList.toggle("active", btn.dataset.tool === t);
    }
    setStatus(toolHint(t));
    redraw();
  }

  function toolHint(t: Tool): string {
    switch (t) {
      case "point": return "Point: click to place";
      case "line": return "Line: click start, then end";
      case "circle": return "Circle: click center, then a point on the circumference";
      case "arc": return "Arc: click center, then start, then end (CCW)";
      case "pan": return "Pan: drag to move; wheel to zoom";
      case "select": return "Select: click a primitive; drag a Point to move";
      case "delete": return "Delete: click a primitive to remove";
      case "trim": return "Trim: click a Line, then click a Point on it (creates trim);";
      case "extend": return "Extend: click a Line, then click the target Point";
      case "fillet": return "Fillet: click a corner Point (shared by two Lines), enter a radius";
      case "mirror": return "Mirror: click axis start, then axis end; selected primitives are reflected";
      case "pattern-rect": return "Pattern Rect: select primitives (right-click to add), then enter dx/dy/nx/ny";
    }
  }

  // Apply a click in the canvas given the mouse position (canvas pixels).
  function handleClick(cx: number, cy: number, button: number) {
    const world = canvasToWorld(cx, cy);
    if (button === 2) {
      // Right-click → context menu (constraints / delete).
      const id = pickNearestPrimId(world, 12);
      if (id) showContextMenu(id, cx, cy);
      return;
    }
    switch (currentTool) {
      case "point": {
        addOrReusePoint(world);
        break;
      }
      case "line": {
        const id = addOrReusePoint(world);
        if (pendingStartId == null) {
          pendingStartId = id;
          setStatus("Line: click end point");
        } else if (pendingStartId === id) {
          setStatus("Line: pick a different point for end");
        } else {
          addPrim({ kind: "Line", id: freshId("l"), from: pendingStartId, to: id });
          pendingStartId = null;
          setStatus(toolHint("line"));
        }
        break;
      }
      case "circle": {
        if (circleCenterId == null) {
          circleCenterId = addOrReusePoint(world);
          setStatus("Circle: click a point on the circumference");
        } else {
          const cp = primitives.get(circleCenterId);
          if (cp && cp.kind === "Point") {
            const snapped = snapWorld(world);
            const r = Math.hypot(snapped.x - cp.x, snapped.y - cp.y);
            if (r > 1e-9) {
              addPrim({
                kind: "Circle",
                id: freshId("k"),
                center: circleCenterId,
                radius: r,
                n_segments: 32,
              });
            }
          }
          circleCenterId = null;
          setStatus(toolHint("circle"));
        }
        break;
      }
      case "arc": {
        if (arcCenterId == null) {
          arcCenterId = addOrReusePoint(world);
          setStatus("Arc: click start point");
        } else if (arcStartAngle == null) {
          // Resolve the start point as an existing Point so loop tracing works.
          const startId = addOrReusePoint(world);
          const cp = primitives.get(arcCenterId);
          const sp = primitives.get(startId);
          if (cp && cp.kind === "Point" && sp && sp.kind === "Point") {
            arcStartAngle = Math.atan2(sp.y - cp.y, sp.x - cp.x);
          }
          setStatus("Arc: click end point (CCW from start)");
        } else {
          const endId = addOrReusePoint(world);
          const cp = primitives.get(arcCenterId);
          const ep = primitives.get(endId);
          if (cp && cp.kind === "Point" && ep && ep.kind === "Point") {
            const r = Math.hypot(ep.x - cp.x, ep.y - cp.y);
            let a1 = Math.atan2(ep.y - cp.y, ep.x - cp.x);
            // Force CCW: if a1 < a0, add 2π.
            if (a1 <= arcStartAngle!) a1 += 2 * Math.PI;
            addPrim({
              kind: "Arc",
              id: freshId("a"),
              center: arcCenterId,
              radius: r,
              start_angle: arcStartAngle!,
              end_angle: a1,
              n_segments: 16,
            });
          }
          arcCenterId = null;
          arcStartAngle = null;
          setStatus(toolHint("arc"));
        }
        break;
      }
      case "select": {
        const id = pickNearestPrimId(world, 12);
        selectedId = id;
        redraw();
        refreshPrimsList();
        if (id) setStatus(`selected: ${id}`);
        break;
      }
      case "delete": {
        const id = pickNearestPrimId(world, 12);
        if (id) {
          deletePrim(id);
          setStatus(`deleted ${id}`);
        }
        break;
      }
      case "trim": {
        // 1st click: pick a Line. 2nd click: emit a TrimLine via the
        // testable applyTrimClick helper.
        if (trimSelectedLineId == null) {
          const id = pickNearestLineId(world, 12);
          if (id) {
            trimSelectedLineId = id;
            setStatus(`Trim: click a Point on ${id} to trim at`);
          } else {
            setStatus("Trim: click a Line first");
          }
        } else {
          const lineId = trimSelectedLineId;
          const existing = pickNearestPointId(world, 10);
          // Pre-project for snap (purely cosmetic — applyTrimClick does
          // the canonical projection/clamp itself when no existing point
          // is given).
          const proj = projectPointOntoLine(world, lineId) ?? world;
          const clickPt = snapInput.checked && !existing ? snapWorld(proj) : proj;
          const result = applyTrimClick(toSketch(), lineId, clickPt, {
            existingPointId: existing,
            nextPointId: () => freshId("p"),
            nextTrimId: () => freshId("tr"),
          });
          if (result) {
            for (const p of result.primitives) addPrim(p);
            setStatus(`trimmed ${lineId}`);
          } else {
            setStatus("Trim failed: line not found", true);
          }
          trimSelectedLineId = null;
        }
        break;
      }
      case "extend": {
        if (extendSelectedLineId == null) {
          const id = pickNearestLineId(world, 12);
          if (id) {
            extendSelectedLineId = id;
            setStatus(`Extend: click target Point for ${id}`);
          } else {
            setStatus("Extend: click a Line first");
          }
        } else {
          const lineId = extendSelectedLineId;
          const existing = pickNearestPointId(world, 10);
          const clickPt = snapInput.checked && !existing ? snapWorld(world) : world;
          const result = applyExtendClick(toSketch(), lineId, clickPt, {
            existingPointId: existing,
            nextPointId: () => freshId("p"),
            nextExtendId: () => freshId("ex"),
          });
          if (result) {
            for (const p of result.primitives) addPrim(p);
            setStatus(`extended ${lineId}`);
          } else {
            setStatus("Extend failed: line not found", true);
          }
          extendSelectedLineId = null;
        }
        break;
      }
      case "fillet": {
        const cornerId = pickNearestPointId(world, 12);
        if (!cornerId) {
          setStatus("Fillet: click a corner Point (must be shared by two Lines)");
          break;
        }
        const rStr = window.prompt(`Fillet radius for corner ${cornerId}?`, "0.5");
        if (!rStr) {
          setStatus("Fillet: cancelled");
          break;
        }
        const r = Number.parseFloat(rStr);
        const result = applyFilletClick(toSketch(), cornerId, r, {
          nextFilletId: () => freshId("f"),
        });
        if (result) {
          for (const p of result.primitives) addPrim(p);
          setStatus(`filleted @${cornerId} r=${r}`);
        } else {
          setStatus("Fillet: invalid radius (must be > 0)", true);
        }
        break;
      }
      case "mirror": {
        // Two-click axis pick. After the axis is defined, the user is prompted
        // for which primitives to mirror (using the current selection or all).
        if (mirrorAxisStart == null) {
          mirrorAxisStart = snapWorld(world);
          setStatus("Mirror: click axis end point");
        } else {
          const axisEnd = snapWorld(world);
          const ax = mirrorAxisStart;
          mirrorAxisStart = null;
          // Use editOpIds if non-empty, otherwise use selectedId or all primitives.
          const ids = editOpIds.length > 0
            ? editOpIds
            : selectedId
              ? [selectedId]
              : Array.from(primitives.keys());
          if (ids.length === 0) {
            setStatus("Mirror: nothing to mirror", true);
            break;
          }
          const result = mirrorSelection(toSketch(), ids, { from: ax, to: axisEnd });
          // Merge the new prims (those not already in primitives).
          const existingIds = new Set(primitives.keys());
          for (const p of result.primitives) {
            if (!existingIds.has(p.id)) addPrim(p);
          }
          editOpIds = [];
          setStatus(`mirrored ${ids.length} primitives`);
        }
        break;
      }
      case "pattern-rect": {
        // Prompt for parameters, then replicate.
        const ids = editOpIds.length > 0
          ? editOpIds
          : selectedId
            ? [selectedId]
            : Array.from(primitives.keys());
        if (ids.length === 0) {
          setStatus("Pattern Rect: nothing selected", true);
          break;
        }
        const params = window.prompt("Pattern Rect: dx dy nx ny (space-separated)", "5 5 2 2");
        if (!params) {
          setStatus("Pattern Rect: cancelled");
          break;
        }
        const [dxS, dyS, nxS, nyS] = params.trim().split(/\s+/);
        const pdx = Number(dxS);
        const pdy = Number(dyS);
        const pnx = Math.max(1, Math.round(Number(nxS)));
        const pny = Math.max(1, Math.round(Number(nyS)));
        if ([pdx, pdy, pnx, pny].some(Number.isNaN)) {
          setStatus("Pattern Rect: invalid parameters", true);
          break;
        }
        const result = patternRect(toSketch(), ids, pdx, pdy, pnx, pny);
        const existingIds = new Set(primitives.keys());
        for (const p of result.primitives) {
          if (!existingIds.has(p.id)) addPrim(p);
        }
        editOpIds = [];
        setStatus(`pattern ${pnx}×${pny} from ${ids.length} primitives`);
        break;
      }
      case "pan":
        // Handled in mousedown (drag).
        break;
    }
  }

  // ---- canvas event wiring ----
  canvas.addEventListener("contextmenu", (e) => {
    e.preventDefault();
    const r = canvas.getBoundingClientRect();
    handleClick(e.clientX - r.left, e.clientY - r.top, 2);
  });

  canvas.addEventListener("mousedown", (e) => {
    const r = canvas.getBoundingClientRect();
    const cx = e.clientX - r.left;
    const cy = e.clientY - r.top;
    if (e.button === 1 || (e.button === 0 && currentTool === "pan")) {
      // Pan drag.
      dragPanStart = { x: cx, y: cy, ox: view.ox, oy: view.oy };
      e.preventDefault();
      return;
    }
    if (e.button === 0 && currentTool === "select") {
      // Try to start dragging a Point.
      const world = canvasToWorld(cx, cy);
      const id = pickNearestPointId(world, 10);
      if (id) {
        pushHistory(); // snapshot before drag-move mutation
        dragPointId = id;
        return;
      }
    }
  });

  canvas.addEventListener("mousemove", (e) => {
    const r = canvas.getBoundingClientRect();
    const cx = e.clientX - r.left;
    const cy = e.clientY - r.top;
    mouseWorld = canvasToWorld(cx, cy);
    if (dragPanStart) {
      view.ox = dragPanStart.ox + (cx - dragPanStart.x);
      view.oy = dragPanStart.oy + (cy - dragPanStart.y);
      redraw();
      return;
    }
    if (dragPointId) {
      const prim = primitives.get(dragPointId);
      if (prim && prim.kind === "Point") {
        const snapped = snapWorld(mouseWorld);
        prim.x = snapped.x;
        prim.y = snapped.y;
        redraw();
        refreshPrimsList();
      }
      return;
    }
    // Update hover for visual feedback.
    const hovered = pickNearestPointId(mouseWorld, 10);
    if (hovered !== hoverPointId) {
      hoverPointId = hovered;
    }
    // For trim / extend, hover on Lines too. For fillet, hover on Points.
    if (currentTool === "trim" || currentTool === "extend") {
      hoverPrimId = pickNearestLineId(mouseWorld, 12);
    } else if (currentTool === "fillet") {
      hoverPrimId = hovered;
    } else {
      hoverPrimId = null;
    }

    // Compute snap while a drawing tool is active.
    const drawingTools = ["point", "line", "circle", "arc"];
    if (drawingTools.includes(currentTool) && snapInput.checked) {
      activeSnap = findSnap(mouseWorld, toSketch(), {
        scale: view.scale,
        gridSpacing: Number(gridInput.value) || 1,
        gridVisible: (view.scale * (Number(gridInput.value) || 1)) >= 6,
      });
    } else {
      activeSnap = null;
    }

    redraw();
  });

  canvas.addEventListener("mouseup", (e) => {
    const r = canvas.getBoundingClientRect();
    const cx = e.clientX - r.left;
    const cy = e.clientY - r.top;
    if (dragPanStart) {
      // Treat as pan-end; suppress the click-to-place if we actually moved.
      const moved = Math.hypot(cx - dragPanStart.x, cy - dragPanStart.y) > 3;
      dragPanStart = null;
      if (moved) return;
    }
    if (dragPointId) {
      // End drag, suppress click-place.
      dragPointId = null;
      return;
    }
    if (e.button !== 0) return; // right-click handled by contextmenu
    handleClick(cx, cy, 0);
  });

  canvas.addEventListener("wheel", (e) => {
    e.preventDefault();
    const r = canvas.getBoundingClientRect();
    const cx = e.clientX - r.left;
    const cy = e.clientY - r.top;
    // Zoom toward cursor: keep the world point under cursor stationary.
    const before = canvasToWorld(cx, cy);
    const factor = e.deltaY < 0 ? 1.1 : 1 / 1.1;
    view.scale = Math.max(2, Math.min(2000, view.scale * factor));
    const after = canvasToWorld(cx, cy);
    view.ox += (after.x - before.x) * view.scale;
    view.oy -= (after.y - before.y) * view.scale;
    redraw();
  }, { passive: false });

  for (const btn of toolbarBtns) {
    btn.addEventListener("click", () => setTool(btn.dataset.tool as Tool));
  }
  setTool("point");

  // ---- rendering ----
  function redraw() {
    const w = canvas.width;
    const h = canvas.height;
    ctx.clearRect(0, 0, w, h);
    ctx.fillStyle = "#0d1116";
    ctx.fillRect(0, 0, w, h);

    drawGrid(w, h);
    drawAxes(w, h);
    drawPrimitives();
    drawConstraints();
    drawCursorPreview();

    zoomLbl.textContent = `zoom ${view.scale.toFixed(0)}px/u  origin (${view.ox.toFixed(0)},${view.oy.toFixed(0)})`;
  }

  function drawGrid(w: number, h: number) {
    const g = Number(gridInput.value) || 1;
    if (view.scale * g < 6) return; // too dense; bail
    ctx.strokeStyle = "#1a1f26";
    ctx.lineWidth = 1;
    // Grid lines that fall inside the canvas.
    const tlw = canvasToWorld(0, 0);
    const brw = canvasToWorld(w, h);
    const x0 = Math.floor(tlw.x / g) * g;
    const x1 = Math.ceil(brw.x / g) * g;
    const y0 = Math.floor(brw.y / g) * g;
    const y1 = Math.ceil(tlw.y / g) * g;
    ctx.beginPath();
    for (let x = x0; x <= x1; x += g) {
      const [cx] = worldToCanvas(x, 0);
      ctx.moveTo(cx, 0);
      ctx.lineTo(cx, h);
    }
    for (let y = y0; y <= y1; y += g) {
      const [, cy] = worldToCanvas(0, y);
      ctx.moveTo(0, cy);
      ctx.lineTo(w, cy);
    }
    ctx.stroke();
  }

  function drawAxes(w: number, h: number) {
    const [ox, oy] = worldToCanvas(0, 0);
    ctx.strokeStyle = "#3d4651";
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(0, oy);
    ctx.lineTo(w, oy);
    ctx.moveTo(ox, 0);
    ctx.lineTo(ox, h);
    ctx.stroke();
  }

  function drawPrimitives() {
    // Lines first (drawn under points).
    for (const prim of primitives.values()) {
      if (prim.kind !== "Line") continue;
      const a = primitives.get(prim.from);
      const b = primitives.get(prim.to);
      if (!a || a.kind !== "Point" || !b || b.kind !== "Point") continue;
      const [ax, ay] = worldToCanvas(a.x, a.y);
      const [bx, by] = worldToCanvas(b.x, b.y);
      const isSelected = prim.id === selectedId;
      const isTrimSel = prim.id === trimSelectedLineId;
      const isExtendSel = prim.id === extendSelectedLineId;
      const isHover = prim.id === hoverPrimId &&
        (currentTool === "trim" || currentTool === "extend");
      if (isTrimSel || isExtendSel) {
        ctx.strokeStyle = "#f97583"; // editor-selected line: red-orange
        ctx.lineWidth = 3;
      } else if (isHover) {
        ctx.strokeStyle = "#a8c7ff";
        ctx.lineWidth = 2;
      } else if (isSelected) {
        ctx.strokeStyle = "#ffb84d";
        ctx.lineWidth = 2;
      } else {
        ctx.strokeStyle = "#6ea8ff";
        ctx.lineWidth = 1.5;
      }
      ctx.beginPath();
      ctx.moveTo(ax, ay);
      ctx.lineTo(bx, by);
      ctx.stroke();
    }
    // Circles + Arcs.
    for (const prim of primitives.values()) {
      if (prim.kind === "Circle") {
        const c = primitives.get(prim.center);
        if (!c || c.kind !== "Point") continue;
        const [cx, cy] = worldToCanvas(c.x, c.y);
        ctx.strokeStyle = prim.id === selectedId ? "#ffb84d" : "#6ea8ff";
        ctx.lineWidth = prim.id === selectedId ? 2 : 1.5;
        ctx.beginPath();
        ctx.arc(cx, cy, prim.radius * view.scale, 0, 2 * Math.PI);
        ctx.stroke();
      } else if (prim.kind === "Arc") {
        const c = primitives.get(prim.center);
        if (!c || c.kind !== "Point") continue;
        const [cx, cy] = worldToCanvas(c.x, c.y);
        ctx.strokeStyle = prim.id === selectedId ? "#ffb84d" : "#6ea8ff";
        ctx.lineWidth = prim.id === selectedId ? 2 : 1.5;
        ctx.beginPath();
        // Canvas arc: angles are clockwise from +x. Our world has +y up, so
        // we feed -start, -end with anticlockwise flag.
        ctx.arc(cx, cy, prim.radius * view.scale, -prim.end_angle, -prim.start_angle, false);
        ctx.stroke();
      }
    }
    // Points last (top).
    for (const prim of primitives.values()) {
      if (prim.kind !== "Point") continue;
      const [cx, cy] = worldToCanvas(prim.x, prim.y);
      const isSelected = prim.id === selectedId;
      const isHovered = prim.id === hoverPointId;
      ctx.fillStyle = isSelected ? "#ffb84d" : isHovered ? "#a8c7ff" : "#e6edf3";
      ctx.beginPath();
      ctx.arc(cx, cy, isSelected ? 4.5 : 3.5, 0, 2 * Math.PI);
      ctx.fill();
      // Tiny label.
      ctx.fillStyle = "#8b949e";
      ctx.font = "10px ui-monospace,Menlo,monospace";
      ctx.fillText(prim.id, cx + 6, cy - 4);
    }
  }

  function drawConstraints() {
    renderConstraintGlyphs(canvas, toSketch(), view);
  }

  function drawCursorPreview() {
    if (!mouseWorld) return;
    // If an active snap is present use its point; otherwise fall back to grid snap.
    const snapped = activeSnap
      ? activeSnap.point
      : snapInput.checked
        ? snapWorld(mouseWorld)
        : mouseWorld;
    const [cx, cy] = worldToCanvas(snapped.x, snapped.y);

    // Draw snap indicator (small square) when a snap is active.
    if (activeSnap) {
      const sq = 6;
      ctx.strokeStyle = "#ffd700";
      ctx.lineWidth = 1.5;
      ctx.strokeRect(cx - sq, cy - sq, sq * 2, sq * 2);
    }

    // Crosshair.
    ctx.strokeStyle = "#58a6ff";
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(cx - 5, cy);
    ctx.lineTo(cx + 5, cy);
    ctx.moveTo(cx, cy - 5);
    ctx.lineTo(cx, cy + 5);
    ctx.stroke();
    // Coords readout + snap kind.
    ctx.fillStyle = "#8b949e";
    ctx.font = "10px ui-monospace,Menlo,monospace";
    const snapLabel = activeSnap ? `  [${activeSnap.kind}]` : "";
    ctx.fillText(`(${snapped.x.toFixed(2)}, ${snapped.y.toFixed(2)})${snapLabel}`, cx + 8, cy + 14);

    // Tool-specific previews.
    if (currentTool === "line" && pendingStartId) {
      const sp = primitives.get(pendingStartId);
      if (sp && sp.kind === "Point") {
        const [sx, sy] = worldToCanvas(sp.x, sp.y);
        ctx.strokeStyle = "rgba(110, 168, 255, 0.5)";
        ctx.setLineDash([4, 4]);
        ctx.beginPath();
        ctx.moveTo(sx, sy);
        ctx.lineTo(cx, cy);
        ctx.stroke();
        ctx.setLineDash([]);
      }
    } else if (currentTool === "circle" && circleCenterId) {
      const cp = primitives.get(circleCenterId);
      if (cp && cp.kind === "Point") {
        const [ccx, ccy] = worldToCanvas(cp.x, cp.y);
        const r = Math.hypot(snapped.x - cp.x, snapped.y - cp.y);
        ctx.strokeStyle = "rgba(110, 168, 255, 0.5)";
        ctx.setLineDash([4, 4]);
        ctx.beginPath();
        ctx.arc(ccx, ccy, r * view.scale, 0, 2 * Math.PI);
        ctx.stroke();
        ctx.setLineDash([]);
      }
    } else if (currentTool === "trim" && trimSelectedLineId) {
      // Show the projection of the cursor onto the selected line as a
      // small preview marker — that's where the trim point will land.
      const proj = projectPointOntoLine(mouseWorld, trimSelectedLineId);
      if (proj) {
        const [px, py] = worldToCanvas(proj.x, proj.y);
        ctx.strokeStyle = "#f97583";
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.arc(px, py, 6, 0, 2 * Math.PI);
        ctx.stroke();
      }
    } else if (currentTool === "extend" && extendSelectedLineId) {
      // Dashed segment from the closer endpoint to the cursor.
      const prim = primitives.get(extendSelectedLineId);
      if (prim && prim.kind === "Line") {
        const a = primitives.get(prim.from);
        const b = primitives.get(prim.to);
        if (a?.kind === "Point" && b?.kind === "Point") {
          const da = Math.hypot(a.x - snapped.x, a.y - snapped.y);
          const db = Math.hypot(b.x - snapped.x, b.y - snapped.y);
          const closer = da < db ? a : b;
          const [sx, sy] = worldToCanvas(closer.x, closer.y);
          ctx.strokeStyle = "rgba(249, 117, 131, 0.6)";
          ctx.setLineDash([4, 4]);
          ctx.lineWidth = 2;
          ctx.beginPath();
          ctx.moveTo(sx, sy);
          ctx.lineTo(cx, cy);
          ctx.stroke();
          ctx.setLineDash([]);
        }
      }
    } else if (currentTool === "fillet" && hoverPointId) {
      // Show a small radius preview circle on the hovered corner Point so
      // the user sees what kind of fillet they'll get. This is just a
      // visual hint — the actual radius is asked for in a prompt.
      const cp = primitives.get(hoverPointId);
      if (cp && cp.kind === "Point") {
        const [ccx, ccy] = worldToCanvas(cp.x, cp.y);
        const previewR = 0.5 * view.scale; // fixed visual hint
        ctx.strokeStyle = "rgba(127, 231, 135, 0.55)";
        ctx.setLineDash([3, 3]);
        ctx.lineWidth = 1.5;
        ctx.beginPath();
        ctx.arc(ccx, ccy, previewR, 0, 2 * Math.PI);
        ctx.stroke();
        ctx.setLineDash([]);
      }
    }
  }

  // ---- context menu (right-click constraints) ----
  let menuEl: HTMLDivElement | null = null;
  function hideContextMenu() {
    if (menuEl) {
      menuEl.remove();
      menuEl = null;
    }
  }
  function showContextMenu(primId: string, cx: number, cy: number) {
    hideContextMenu();
    const prim = primitives.get(primId);
    if (!prim) return;
    const m = document.createElement("div");
    m.className = "sk-context-menu";
    Object.assign(m.style, {
      position: "absolute",
      background: "var(--panel)",
      border: "1px solid var(--line)",
      borderRadius: "4px",
      padding: "4px 0",
      minWidth: "140px",
      zIndex: "1000",
      fontSize: "11px",
    } as CSSStyleDeclaration);
    const r = canvas.getBoundingClientRect();
    m.style.left = `${r.left + cx}px`;
    m.style.top = `${r.top + cy}px`;
    const items: { label: string; act: () => void }[] = [
      { label: `delete ${primId}`, act: () => deletePrim(primId) },
    ];
    if (prim.kind === "Line") {
      items.push(
        { label: "constrain horizontal", act: () => addConstraint({ kind: "Horizontal", line: primId }) },
        { label: "constrain vertical", act: () => addConstraint({ kind: "Vertical", line: primId }) },
      );
    }
    if (prim.kind === "Point") {
      items.push(
        { label: "constrain fixed", act: () => addConstraint({ kind: "FixedPoint", point: primId }) },
      );
      // Distance to another Point: iterate existing points for quick picks.
      for (const other of primitives.values()) {
        if (other.kind !== "Point" || other.id === primId) continue;
        const label = `distance to ${other.id}`;
        const otherId = other.id;
        items.push({
          label,
          act: () => {
            // Canvas-space midpoint between the two points as the anchor.
            const pa = prim;
            const pb = other;
            const [ax, ay] = worldToCanvas((pa.x + pb.x) / 2, (pa.y + pb.y) / 2);
            const canvasRect = canvas.getBoundingClientRect();
            addDistanceConstraint(primId, otherId, {
              x: canvasRect.left + ax,
              y: canvasRect.top + ay,
            });
          },
        });
      }
    }
    for (const it of items) {
      const b = document.createElement("div");
      b.textContent = it.label;
      Object.assign(b.style, {
        padding: "4px 12px",
        cursor: "pointer",
        color: "var(--fg)",
      } as CSSStyleDeclaration);
      b.addEventListener("mouseenter", () => (b.style.background = "var(--bg)"));
      b.addEventListener("mouseleave", () => (b.style.background = ""));
      b.addEventListener("click", () => {
        it.act();
        hideContextMenu();
      });
      m.appendChild(b);
    }
    document.body.appendChild(m);
    menuEl = m;
    setTimeout(() => {
      const onClick = (ev: MouseEvent) => {
        if (menuEl && !menuEl.contains(ev.target as Node)) {
          hideContextMenu();
          window.removeEventListener("mousedown", onClick);
        }
      };
      window.addEventListener("mousedown", onClick);
    }, 0);
  }

  function addConstraint(c: SketchConstraint) {
    pushHistory(); // snapshot before mutation
    constraints.push(c);
    redraw();
    refreshPrimsList();
    updateDofStatus();
  }

  /**
   * Add a Distance constraint between two Points, prompting the user for a
   * value via the inline dimension-entry widget. The anchor is the canvas-space
   * midpoint between the two points. If the user cancels, the constraint is
   * created with `value = current measured distance` (locks current geometry).
   */
  async function addDistanceConstraint(
    idA: string,
    idB: string,
    anchorCanvas: { x: number; y: number },
  ) {
    const pA = primitives.get(idA);
    const pB = primitives.get(idB);
    const measured =
      pA?.kind === "Point" && pB?.kind === "Point"
        ? Math.hypot(pB.x - pA.x, pB.y - pA.y)
        : 0;

    const entered = await promptDistance(anchorCanvas, measured);
    const value = entered !== null ? entered : measured;
    addConstraint({ kind: "Distance", a: idA, b: idB, value });
  }

  // ---- buttons ----
  (host.querySelector(".sk-validate") as HTMLButtonElement).addEventListener("click", () => {
    const sk = toSketch();
    const err = opts.onValidate ? opts.onValidate(sk) : null;
    if (err) {
      setStatus(`invalid: ${err}`, true);
    } else {
      setStatus("valid loop ✓");
    }
  });
  (host.querySelector(".sk-extrude") as HTMLButtonElement).addEventListener("click", () => {
    const sk = toSketch();
    const ds = prompt("Extrude direction (x y z, world units):", "0 0 4");
    if (!ds) return;
    const parts = ds.trim().split(/\s+/).map(Number);
    if (parts.length !== 3 || parts.some(Number.isNaN)) {
      setStatus("bad direction — expected three numbers", true);
      return;
    }
    if (opts.onExtrude) opts.onExtrude(sk, parts as [number, number, number]);
  });
  (host.querySelector(".sk-save") as HTMLButtonElement).addEventListener("click", () => {
    const json = JSON.stringify(toSketch(), null, 2);
    const blob = new Blob([json], { type: "application/json" });
    const url = URL.createObjectURL(blob);
    const a = document.createElement("a");
    a.href = url;
    a.download = "sketch.json";
    document.body.appendChild(a);
    a.click();
    a.remove();
    URL.revokeObjectURL(url);
    setStatus("saved sketch.json");
  });
  (host.querySelector(".sk-load") as HTMLButtonElement).addEventListener("click", async () => {
    const inp = document.createElement("input");
    inp.type = "file";
    inp.accept = "application/json";
    inp.onchange = async () => {
      const f = inp.files?.[0];
      if (!f) return;
      const txt = await f.text();
      try {
        const sk = JSON.parse(txt) as Sketch;
        loadSketch(sk);
        setStatus(`loaded ${primitives.size} primitives`);
      } catch (e) {
        setStatus(`load failed: ${e}`, true);
      }
    };
    inp.click();
  });
  (host.querySelector(".sk-clear") as HTMLButtonElement).addEventListener("click", () => clearSketch());

  // 2D editing operations Trim / Extend / Fillet are click-driven. The
  // tool buttons use the same `data-tool` attribute as the main toolbar,
  // so the generic `for (const btn of toolbarBtns)` wiring picks them up
  // and routes to setTool(). See the trim/extend/fillet cases in
  // handleClick for the actual flow.

  gridInput.addEventListener("change", () => redraw());
  snapInput.addEventListener("change", () => redraw());

  // ---- conversions ----
  function toSketch(): Sketch {
    return {
      plane,
      primitives: Array.from(primitives.values()),
      constraints: [...constraints],
    };
  }

  function loadSketch(sk: Sketch) {
    primitives.clear();
    constraints = [];
    plane = sk.plane;
    for (const prim of sk.primitives) primitives.set(prim.id, prim);
    constraints = [...(sk.constraints ?? [])];
    // Reset id seq above the highest seen numeric suffix to avoid collisions
    // on subsequent freshId() calls.
    let maxSeq = 0;
    for (const id of primitives.keys()) {
      const m = id.match(/(\d+)$/);
      if (m) maxSeq = Math.max(maxSeq, Number(m[1]));
    }
    nextSeq = maxSeq + 1;
    redraw();
    refreshPrimsList();
    updateDofStatus();
  }

  function setStatus(msg: string, isErr = false) {
    status.textContent = msg;
    status.style.color = isErr ? "#ff7b72" : "var(--muted)";
  }

  /** Append the DOF readout to the status bar text. */
  function updateDofStatus() {
    const dof = computeDof(toSketch());
    setStatus(formatDof(dof));
  }

  function refreshPrimsList() {
    const lines: string[] = [];
    for (const p of primitives.values()) {
      lines.push(formatPrim(p));
    }
    for (const c of constraints) lines.push(`  ⚓ ${constraintTag(c)} ${constraintRefs(c).join(",")}`);
    primsList.textContent = lines.join("\n");
  }

  // ---- keyboard ----
  function handleKey(e: KeyboardEvent) {
    // Only when sketcher panel is focused-ish: skip if user is typing in an input.
    const t = e.target as HTMLElement | null;
    if (t && (t.tagName === "INPUT" || t.tagName === "SELECT" || t.tagName === "TEXTAREA")) return;

    const meta = e.metaKey || e.ctrlKey;

    // Undo: Cmd+Z / Ctrl+Z
    if (meta && !e.shiftKey && e.key === "z") {
      e.preventDefault();
      const prev = history.undo(toSketch());
      if (prev) {
        applySketch(prev);
        setStatus("undo");
      }
      return;
    }
    // Redo: Shift+Cmd+Z / Ctrl+Y
    if ((meta && e.shiftKey && e.key === "z") || (e.ctrlKey && e.key === "y")) {
      e.preventDefault();
      const next = history.redo(toSketch());
      if (next) {
        applySketch(next);
        setStatus("redo");
      }
      return;
    }

    // Cmd+C / Ctrl+C: copy the current selection to the internal clipboard.
    if (meta && (e.key === "c" || e.key === "C")) {
      const ids = selectedId ? [selectedId] : Array.from(primitives.keys());
      const frag = copySelection(toSketch(), ids);
      clipboardPrims = frag.primitives;
      lastPasteOffset = { x: 10, y: 10 };
      setStatus(`copied ${clipboardPrims.length} primitive(s)`);
      e.preventDefault();
      return;
    }
    // Cmd+V / Ctrl+V: paste the clipboard at the last paste offset.
    if (meta && (e.key === "v" || e.key === "V")) {
      if (clipboardPrims.length === 0) {
        setStatus("paste: clipboard empty", true);
        e.preventDefault();
        return;
      }
      const frag = { primitives: clipboardPrims };
      const result = pasteFragment(toSketch(), frag, lastPasteOffset);
      // Add only the new prims (those absent from current primitives).
      const existingBefore = new Set(primitives.keys());
      for (const p of result.primitives) {
        if (!existingBefore.has(p.id)) addPrim(p);
      }
      // Shift the next paste offset so repeated pastes cascade.
      lastPasteOffset = { x: lastPasteOffset.x + 10, y: lastPasteOffset.y + 10 };
      setStatus(`pasted ${clipboardPrims.length} primitive(s)`);
      e.preventDefault();
      return;
    }

    if (e.key === "p" || e.key === "P") setTool("point");
    else if (e.key === "l" || e.key === "L") setTool("line");
    else if (e.key === "c" || e.key === "C") setTool("circle");
    else if (e.key === "a" || e.key === "A") setTool("arc");
    else if (e.key === "x" || e.key === "X") setTool("delete");
    else if (e.key === "v" || e.key === "V") setTool("select");
    else if (e.key === "t" || e.key === "T") setTool("trim");
    else if (e.key === "e" || e.key === "E") setTool("extend");
    else if (e.key === "f" || e.key === "F") setTool("fillet");
    else if (e.key === "m" || e.key === "M") setTool("mirror");
    else if (e.key === "Escape") {
      pendingStartId = null;
      circleCenterId = null;
      arcCenterId = null;
      arcStartAngle = null;
      trimSelectedLineId = null;
      extendSelectedLineId = null;
      mirrorAxisStart = null;
      editOpIds = [];
      selectedId = null;
      redraw();
    } else if (e.key === "Delete" || e.key === "Backspace") {
      if (selectedId) {
        deletePrim(selectedId);
        selectedId = null;
      }
    }
  }

  // ---- public API ----
  redraw();

  return {
    loadSketch,
    toSketch,
    clear: clearSketch,
    handleKey,
    redraw,
    /** Undo/redo controller (for programmatic access). */
    history,
  };
}

// ---- helpers ----
function formatPrim(p: SketchPrim): string {
  switch (p.kind) {
    case "Point":
      return `${p.id} Point (${p.x.toFixed(2)}, ${p.y.toFixed(2)})`;
    case "Line":
      return `${p.id} Line ${p.from}→${p.to}`;
    case "Circle":
      return `${p.id} Circle c=${p.center} r=${p.radius.toFixed(2)} n=${p.n_segments}`;
    case "Arc":
      return `${p.id} Arc c=${p.center} r=${p.radius.toFixed(2)} ${(p.start_angle).toFixed(2)}→${(p.end_angle).toFixed(2)}`;
    case "TrimLine":
      return `${p.id} Trim ${p.line} @ ${p.at_point}`;
    case "ExtendLine":
      return `${p.id} Extend ${p.line} → ${p.to_point}`;
    case "FilletCorner":
      return `${p.id} Fillet @${p.corner_point} r=${p.radius.toFixed(2)}`;
  }
}

export function constraintTag(c: SketchConstraint): string {
  switch (c.kind) {
    case "Horizontal": return "H";
    case "Vertical": return "V";
    case "Coincident": return "≡";
    case "Distance": return `d=${c.value.toFixed(2)}`;
    case "Parallel": return "∥";
    case "Perpendicular": return "⊥";
    case "FixedPoint": return "📌";
  }
}

export function constraintRefs(c: SketchConstraint): string[] {
  switch (c.kind) {
    case "Horizontal":
    case "Vertical":
      return [c.line];
    case "Coincident":
    case "Distance":
      return [c.a, c.b];
    case "Parallel":
    case "Perpendicular":
      return [c.line_a, c.line_b];
    case "FixedPoint":
      return [c.point];
  }
}

function pointSegmentDistance(p: Pt, a: { x: number; y: number }, b: { x: number; y: number }): number {
  const ax = b.x - a.x;
  const ay = b.y - a.y;
  const len2 = ax * ax + ay * ay;
  if (len2 < 1e-12) return Math.hypot(p.x - a.x, p.y - a.y);
  let t = ((p.x - a.x) * ax + (p.y - a.y) * ay) / len2;
  t = Math.max(0, Math.min(1, t));
  const qx = a.x + t * ax;
  const qy = a.y + t * ay;
  return Math.hypot(p.x - qx, p.y - qy);
}

function isAngleBetween(theta: number, a0: number, a1: number): boolean {
  // Normalize a1 - a0 to be positive (CCW range).
  const norm = (x: number) => {
    let v = x - a0;
    while (v < 0) v += 2 * Math.PI;
    while (v >= 2 * Math.PI) v -= 2 * Math.PI;
    return v;
  };
  const span = norm(a1);
  const t = norm(theta);
  return t <= span;
}

/**
 * Pure functional version of the canvas Trim flow. Given a sketch and a
 * world-space click point, returns the new primitives that the click
 * should add: a Point at the projected location on `lineId` (or reuses
 * `existingPointId` if provided), plus a TrimLine primitive. Mirrors the
 * UX from the canvas — the canvas actually calls this internally for
 * easier testing without a DOM.
 *
 * `lineId` must reference a Line primitive in `sketch`. Returns null if
 * the line is missing or degenerate.
 */
export function applyTrimClick(
  sketch: Sketch,
  lineId: string,
  clickWorld: { x: number; y: number },
  options: {
    existingPointId?: string | null;
    /** Provide a deterministic id allocator for tests. Default: prefix+next. */
    nextPointId?: () => string;
    nextTrimId?: () => string;
  } = {},
): { primitives: SketchPrim[] } | null {
  const linePrim = sketch.primitives.find(
    (p) => p.kind === "Line" && p.id === lineId,
  );
  if (!linePrim || linePrim.kind !== "Line") return null;
  const a = sketch.primitives.find(
    (p) => p.kind === "Point" && p.id === linePrim.from,
  );
  const b = sketch.primitives.find(
    (p) => p.kind === "Point" && p.id === linePrim.to,
  );
  if (!a || a.kind !== "Point" || !b || b.kind !== "Point") return null;

  let pointId: string;
  const out: SketchPrim[] = [];
  if (options.existingPointId) {
    pointId = options.existingPointId;
  } else {
    // Project click onto the line, clamped to the segment.
    const ax = b.x - a.x;
    const ay = b.y - a.y;
    const len2 = ax * ax + ay * ay;
    if (len2 < 1e-12) return null;
    let t = ((clickWorld.x - a.x) * ax + (clickWorld.y - a.y) * ay) / len2;
    t = Math.max(0, Math.min(1, t));
    const px = a.x + t * ax;
    const py = a.y + t * ay;
    pointId = options.nextPointId ? options.nextPointId() : "p_trim";
    out.push({ kind: "Point", id: pointId, x: px, y: py });
  }
  const trimId = options.nextTrimId ? options.nextTrimId() : "tr_1";
  out.push({ kind: "TrimLine", id: trimId, line: lineId, at_point: pointId });
  return { primitives: out };
}

/**
 * Pure functional version of the canvas Extend flow. Returns the new
 * primitives to add: a Point at the click location (or reuse) plus an
 * ExtendLine primitive that points the closer endpoint of `lineId` at
 * `clickWorld`.
 */
export function applyExtendClick(
  sketch: Sketch,
  lineId: string,
  clickWorld: { x: number; y: number },
  options: {
    existingPointId?: string | null;
    nextPointId?: () => string;
    nextExtendId?: () => string;
  } = {},
): { primitives: SketchPrim[] } | null {
  const linePrim = sketch.primitives.find(
    (p) => p.kind === "Line" && p.id === lineId,
  );
  if (!linePrim || linePrim.kind !== "Line") return null;
  let pointId: string;
  const out: SketchPrim[] = [];
  if (options.existingPointId) {
    pointId = options.existingPointId;
  } else {
    pointId = options.nextPointId ? options.nextPointId() : "p_ext";
    out.push({ kind: "Point", id: pointId, x: clickWorld.x, y: clickWorld.y });
  }
  const extId = options.nextExtendId ? options.nextExtendId() : "ex_1";
  out.push({
    kind: "ExtendLine",
    id: extId,
    line: lineId,
    to_point: pointId,
  });
  return { primitives: out };
}

/**
 * Pure functional version of the canvas Fillet flow. Given a corner Point
 * id and a numeric radius, returns the FilletCorner primitive to append.
 * Returns null if `cornerId` doesn't reference a Point in `sketch` or
 * `radius` is non-positive / non-finite.
 */
export function applyFilletClick(
  sketch: Sketch,
  cornerId: string,
  radius: number,
  options: { nextFilletId?: () => string } = {},
): { primitives: SketchPrim[] } | null {
  const cornerPrim = sketch.primitives.find(
    (p) => p.kind === "Point" && p.id === cornerId,
  );
  if (!cornerPrim || cornerPrim.kind !== "Point") return null;
  if (!Number.isFinite(radius) || radius <= 0) return null;
  const fid = options.nextFilletId ? options.nextFilletId() : "f_1";
  return {
    primitives: [
      {
        kind: "FilletCorner",
        id: fid,
        corner_point: cornerId,
        radius,
      },
    ],
  };
}

/**
 * Build a Model JSON wrapping the given sketch in a SketchExtrude feature.
 * Output is parametric-free (literal Scalars), matching the round-trip
 * output of the rust serde derive.
 */
export function buildExtrudeModelJson(
  sketch: Sketch,
  direction: [number, number, number],
  featureId = "out",
): string {
  // Note: rust `Scalar` serde transparency: literal numbers are emitted as
  // numbers, not objects. So `[0, 0, 4]` is valid input for `direction`.
  const model = {
    parameters: {} as Record<string, number>,
    features: [
      {
        kind: "SketchExtrude",
        id: featureId,
        direction,
        sketch,
      },
    ],
  };
  return JSON.stringify(model, null, 2);
}
