/**
 * GD&T (Geometric Dimensioning & Tolerancing) overlay renderer.
 *
 * Implements ASME Y14.5 / ISO 1101 symbology:
 *   - Datum reference flags (labeled A, B, C, …)
 *   - Feature control frames (characteristic glyph | tolerance | datum refs)
 *   - Surface finish symbols (checkmark style with Ra value)
 *
 * KNOWN LIMITATION — Unicode glyph fidelity:
 *   Several ASME Y14.5 symbols have no exact Unicode counterpart and are
 *   approximated here:
 *     - cylindricity uses ⌭ (U+232D, same as concentricity) — the standard
 *       symbol is two concentric circles bracketed by lines, unrepresented.
 *     - circular runout uses ↗ (U+2197, diagonal arrow) rather than the
 *       standard single-headed curved arrow.
 *     - flatness uses ▱ (U+25B1, white parallelogram) rather than the
 *       standard two parallel lines.
 *     - straightness uses — (U+2014, em-dash) which approximates a single line.
 *   These render unambiguously in browser contexts but would not pass a
 *   formal standards review. A future iteration should use SVG <path> glyphs
 *   drawn to exact ASME proportions.
 */

// ---------------------------------------------------------------------------
// Public types
// ---------------------------------------------------------------------------

export type GdtCharacteristic =
  | "position"
  | "concentricity"
  | "symmetry"         // location
  | "circularity"
  | "cylindricity"     // form (round)
  | "flatness"
  | "straightness"     // form (flat/straight)
  | "perpendicularity"
  | "parallelism"
  | "angularity"       // orientation
  | "circular_runout"
  | "total_runout"     // runout
  | "profile_line"
  | "profile_surface"; // profile

export interface DatumReference {
  /** A single capital letter, A–Z. */
  letter: string;
  /** Anchor point in model coordinates. */
  anchor: [number, number, number];
}

export interface FeatureControlFrame {
  /** GD&T characteristic. */
  characteristic: GdtCharacteristic;
  /** Tolerance value, e.g. 0.05 mm. */
  tolerance: number;
  /** True for diametric (⌀) tolerances. */
  diametric: boolean;
  /** Material condition modifier for the tolerance cell. */
  material_modifier?: "M" | "L" | "S" | null;
  /** Up to three datum references with optional per-datum modifiers. */
  datum_refs: Array<{ letter: string; modifier?: "M" | "L" | "S" | null }>;
  /** Anchor point in model coordinates (where the leader points). */
  anchor: [number, number, number];
}

export interface SurfaceFinish {
  /** Roughness average Ra in micrometres. */
  ra: number;
  /** Optional process descriptor. */
  process?: "machined" | "ground" | "as-cast" | "any";
  /** Anchor point in model coordinates. */
  anchor: [number, number, number];
}

export interface GdtAnnotations {
  datums: DatumReference[];
  frames: FeatureControlFrame[];
  finishes: SurfaceFinish[];
}

// ---------------------------------------------------------------------------
// Characteristic → Unicode glyph
// ---------------------------------------------------------------------------

const CHARACTERISTIC_GLYPH: Record<GdtCharacteristic, string> = {
  position:        "⌖", // ⌖
  concentricity:   "⌭", // ⌭
  symmetry:        "⌯", // ⌯
  circularity:     "○", // ○
  cylindricity:    "⌭", // ⌭ (approximation — see limitation note above)
  flatness:        "▱", // ▱ (approximation)
  straightness:    "—", // — (approximation)
  perpendicularity:"⊥", // ⊥
  parallelism:     "∥", // ∥
  angularity:      "∠", // ∠
  circular_runout: "↗", // ↗ (approximation)
  total_runout:    "⌰", // ⌰
  profile_line:    "⌒", // ⌒
  profile_surface: "⌓", // ⌓
};

// ---------------------------------------------------------------------------
// SVG helpers
// ---------------------------------------------------------------------------

const SVG_NS = "http://www.w3.org/2000/svg";

function svgEl<K extends keyof SVGElementTagNameMap>(
  tag: K,
  attrs: Record<string, string | number>,
): SVGElementTagNameMap[K] {
  const el = document.createElementNS(SVG_NS, tag);
  for (const [k, v] of Object.entries(attrs)) {
    el.setAttribute(k, String(v));
  }
  return el;
}

function svgGroup(cls: string): SVGGElement {
  return svgEl("g", { class: cls });
}

/** Append `child` to `parent` and return `parent`. */
function append<T extends SVGElement>(parent: T, ...children: SVGElement[]): T {
  for (const c of children) parent.appendChild(c);
  return parent;
}

// Default drawing style constants (drawing-space units ≈ pixels at 96 dpi).
const STROKE = "#1a2233";
const FILL_BG = "#ffffff";
const FONT_SIZE = 12;          // px — cell text
const GLYPH_SIZE = 14;         // px — characteristic symbol
const CELL_PAD_X = 4;          // horizontal padding inside each cell
const CELL_PAD_Y = 3;          // vertical padding inside each cell
const CELL_H = FONT_SIZE + CELL_PAD_Y * 2 + 2; // 22 px total cell height
const LEADER_LEN = 24;         // stub leader from annotation to anchor
const DATUM_BOX = 16;          // datum flag square side
const DATUM_FLAG_H = 8;        // height of the triangular flag

// ---------------------------------------------------------------------------
// Datum reference rendering
// ---------------------------------------------------------------------------

function renderDatum(
  group: SVGGElement,
  datum: DatumReference,
  px: number,
  py: number,
): void {
  // Box sits above and to the right of the anchor, flag points down to anchor.
  const bx = px - DATUM_BOX / 2;
  const by = py - DATUM_FLAG_H - DATUM_BOX;

  // Leader line from anchor up to base of flag.
  append(
    group,
    svgEl("line", {
      x1: px, y1: py,
      x2: px, y2: py - LEADER_LEN,
      stroke: STROKE, "stroke-width": 0.8,
    }),
  );

  // Triangular flag (points down at anchor).
  const flagTop = py - LEADER_LEN;
  const flagPoints = [
    `${px},${flagTop + DATUM_FLAG_H}`,  // tip (down)
    `${px - DATUM_BOX / 2},${flagTop}`, // top-left
    `${px + DATUM_BOX / 2},${flagTop}`, // top-right
  ].join(" ");
  append(
    group,
    svgEl("polygon", {
      points: flagPoints,
      fill: STROKE, stroke: STROKE, "stroke-width": 0.5,
    }),
  );

  // Square box above the flag.
  append(
    group,
    svgEl("rect", {
      x: bx, y: by,
      width: DATUM_BOX, height: DATUM_BOX,
      fill: FILL_BG, stroke: STROKE, "stroke-width": 0.8,
      "data-gdt": "datum-box",
    }),
  );

  // Letter centered in box.
  const tx = svgEl("text", {
    x: px, y: by + DATUM_BOX / 2 + FONT_SIZE * 0.35,
    "text-anchor": "middle",
    "font-size": FONT_SIZE,
    "font-family": "ui-monospace, 'SF Mono', Menlo, monospace",
    "font-weight": "bold",
    fill: STROKE,
  });
  tx.textContent = datum.letter;
  append(group, tx);
}

// ---------------------------------------------------------------------------
// Feature control frame rendering
// ---------------------------------------------------------------------------

interface CellSpec {
  text: string;
  isGlyph: boolean;
  /** data-gdt attribute value */
  role: string;
}

function buildCells(frame: FeatureControlFrame): CellSpec[] {
  const cells: CellSpec[] = [];

  // Cell 0: characteristic glyph.
  cells.push({
    text: CHARACTERISTIC_GLYPH[frame.characteristic] ?? "?",
    isGlyph: true,
    role: "symbol",
  });

  // Cell 1: tolerance value, optionally prefixed with ⌀ and suffixed with modifier.
  let tolText = frame.diametric ? "⌀" : ""; // ⌀
  tolText += frame.tolerance.toFixed(3).replace(/\.?0+$/, "") || "0";
  if (frame.material_modifier === "M") tolText += "Ⓜ"; // Ⓜ
  else if (frame.material_modifier === "L") tolText += "Ⓛ"; // Ⓛ
  cells.push({ text: tolText, isGlyph: false, role: "tolerance" });

  // Cells 2+: datum references.
  for (const ref of frame.datum_refs) {
    let label = ref.letter;
    if (ref.modifier === "M") label += "Ⓜ";
    else if (ref.modifier === "L") label += "Ⓛ";
    cells.push({ text: label, isGlyph: false, role: "datum-ref" });
  }

  return cells;
}

function renderFrame(
  group: SVGGElement,
  frame: FeatureControlFrame,
  px: number,
  py: number,
): void {
  const cells = buildCells(frame);

  // Measure cell widths.
  // We approximate text width at ~0.6 em per character for the monospace font.
  const CHAR_W = FONT_SIZE * 0.62;
  const GLYPH_CHAR_W = GLYPH_SIZE * 0.9;
  const cellWidths = cells.map((c) => {
    const charW = c.isGlyph ? GLYPH_CHAR_W : CHAR_W;
    return Math.max(c.text.length * charW + CELL_PAD_X * 2, 18);
  });
  const totalW = cellWidths.reduce((a, b) => a + b, 0);

  // Frame sits to the right and above anchor; leader connects anchor to
  // left edge of frame.
  const frameX = px + LEADER_LEN;
  const frameY = py - CELL_H - LEADER_LEN * 0.5;

  // Leader line.
  append(
    group,
    svgEl("line", {
      x1: px, y1: py,
      x2: frameX, y2: frameY + CELL_H / 2,
      stroke: STROKE, "stroke-width": 0.8,
    }),
  );

  // Draw cells.
  let cx = frameX;
  for (let i = 0; i < cells.length; i++) {
    const cell = cells[i]!;
    const cw = cellWidths[i]!;

    append(
      group,
      svgEl("rect", {
        x: cx, y: frameY,
        width: cw, height: CELL_H,
        fill: FILL_BG, stroke: STROKE, "stroke-width": 0.5,
        "data-gdt": `frame-cell-${cell.role}`,
      }),
    );

    const fontSize = cell.isGlyph ? GLYPH_SIZE : FONT_SIZE;
    const textEl = svgEl("text", {
      x: cx + cw / 2,
      y: frameY + CELL_H / 2 + fontSize * 0.35,
      "text-anchor": "middle",
      "font-size": fontSize,
      "font-family": "ui-monospace, 'SF Mono', Menlo, monospace",
      fill: STROKE,
    });
    textEl.textContent = cell.text;
    append(group, textEl);

    cx += cw;
  }

  // Outer border (re-draw over cell borders for clean look).
  append(
    group,
    svgEl("rect", {
      x: frameX, y: frameY,
      width: totalW, height: CELL_H,
      fill: "none", stroke: STROKE, "stroke-width": 0.8,
    }),
  );
}

// ---------------------------------------------------------------------------
// Surface finish rendering
// ---------------------------------------------------------------------------

/**
 * Draws the classic ∇ / checkmark surface finish symbol.
 *
 * ASCII reference:
 *    1.6    <- Ra value above
 *     \∨/   <- check symbol; process letter inside V if non-default
 */
function renderFinish(
  group: SVGGElement,
  finish: SurfaceFinish,
  px: number,
  py: number,
): void {
  // Checkmark geometry: left leg goes up-left, right leg goes up-right,
  // apex sits at (px, py).
  const armLen = 14;
  const halfSpan = 10;

  // Left arm: from apex upward-left.
  const lx = px - halfSpan;
  const ly = py - armLen;
  // Right arm: from apex upward-right.
  const rx = px + halfSpan;
  const ry = ly;
  // Outer top extends further up-left from left arm tip.
  const ox = lx - 6;
  const oy = ly - 4;

  // The checkmark path: outer top → left arm tip → apex → right arm tip.
  const pathD = `M ${ox},${oy} L ${lx},${ly} L ${px},${py} L ${rx},${ry}`;
  append(
    group,
    svgEl("path", {
      d: pathD,
      fill: "none", stroke: STROKE, "stroke-width": 1,
      "data-gdt": "finish-check",
    }),
  );

  // Ra value above the symbol.
  const raEl = svgEl("text", {
    x: px, y: py - armLen - 5,
    "text-anchor": "middle",
    "font-size": FONT_SIZE,
    "font-family": "ui-monospace, 'SF Mono', Menlo, monospace",
    fill: STROKE,
    "data-gdt": "finish-ra",
  });
  raEl.textContent = String(finish.ra);
  append(group, raEl);

  // Process indicator inside the V, if non-default.
  if (finish.process && finish.process !== "any") {
    const procChar = { machined: "M", ground: "G", "as-cast": "C" }[finish.process] ?? "";
    if (procChar) {
      const procEl = svgEl("text", {
        x: px, y: py - 4,
        "text-anchor": "middle",
        "font-size": 9,
        "font-family": "ui-monospace, 'SF Mono', Menlo, monospace",
        fill: STROKE,
        "data-gdt": "finish-process",
      });
      procEl.textContent = procChar;
      append(group, procEl);
    }
  }
}

// ---------------------------------------------------------------------------
// Main export
// ---------------------------------------------------------------------------

/**
 * Render GD&T overlays for a single ortho view into an SVG element.
 *
 * @param targetSvg  Existing SVG element being built — a <g> layer is appended.
 * @param annotations  Datum refs, control frames, and surface finishes.
 * @param viewMatrix   Projects model 3-D coordinates to 2-D drawing coordinates.
 */
export function renderGdtOverlay(
  targetSvg: SVGElement,
  annotations: GdtAnnotations,
  viewMatrix: { project(p: [number, number, number]): [number, number] },
): void {
  const hasAny =
    annotations.datums.length > 0 ||
    annotations.frames.length > 0 ||
    annotations.finishes.length > 0;
  if (!hasAny) return;

  const layer = svgGroup("gdt-layer");

  for (const datum of annotations.datums) {
    const [px, py] = viewMatrix.project(datum.anchor);
    const g = svgGroup("gdt-datum");
    renderDatum(g, datum, px, py);
    append(layer, g);
  }

  for (const frame of annotations.frames) {
    const [px, py] = viewMatrix.project(frame.anchor);
    const g = svgGroup("gdt-frame");
    renderFrame(g, frame, px, py);
    append(layer, g);
  }

  for (const finish of annotations.finishes) {
    const [px, py] = viewMatrix.project(finish.anchor);
    const g = svgGroup("gdt-finish");
    renderFinish(g, finish, px, py);
    append(layer, g);
  }

  targetSvg.appendChild(layer);
}
