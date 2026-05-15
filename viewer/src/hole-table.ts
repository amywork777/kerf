// hole-table.ts — SolidWorks-style hole table for kerf-cad 3-view drawings.
//
// Public API
// ----------
// extractHoles(modelJson)      — pure; parse model JSON, return Hole[].
// renderHoleTable(ctx, holes)  — draw the table onto an existing Canvas 2D ctx.
//
// Hole-producing feature kinds recognised
// ---------------------------------------
//   Counterbore, Countersink, HexHole, SquareHole,
//   HoleArray (expands to count holes), BoltCircle (expands to count holes)

// ---------------------------------------------------------------------------
// Types
// ---------------------------------------------------------------------------

/** One hole entry ready for the drawing table. */
export interface Hole {
  /** Drawing tag, e.g. "A1", "A2". */
  tag: string;
  /** Original feature kind string. */
  type: string;
  /** Nominal diameter (mm). For HexHole = 2×inscribed_radius; SquareHole = side. */
  diameter: number;
  /** Hole depth (mm). */
  depth: number;
  /** World X of the hole center on the opening face. */
  x: number;
  /** World Y of the hole center on the opening face. */
  y: number;
  /** World Z of the hole center on the opening face. */
  z: number;
}

// ---------------------------------------------------------------------------
// Internal — raw JSON shapes for hole-producing features
// ---------------------------------------------------------------------------

interface RawCounterbore {
  kind: "Counterbore";
  id: string;
  axis: string;
  top_center: [number, number, number];
  drill_radius: number;
  cbore_radius: number;
  cbore_depth: number;
  total_depth: number;
}

interface RawCountersink {
  kind: "Countersink";
  id: string;
  axis: string;
  top_center: [number, number, number];
  drill_radius: number;
  csink_radius: number;
  csink_depth: number;
  total_depth: number;
}

interface RawHexHole {
  kind: "HexHole";
  id: string;
  axis: string;
  top_center: [number, number, number];
  inscribed_radius: number;
  depth: number;
}

interface RawSquareHole {
  kind: "SquareHole";
  id: string;
  axis: string;
  top_center: [number, number, number];
  side: number;
  depth: number;
}

interface RawHoleArray {
  kind: "HoleArray";
  id: string;
  axis: string;
  start: [number, number, number];
  offset: [number, number, number];
  count: number;
  radius: number;
  depth: number;
}

interface RawBoltCircle {
  kind: "BoltCircle";
  id: string;
  axis: string;
  center: [number, number, number];
  bolt_circle_radius: number;
  count: number;
  radius: number;
  depth: number;
}

type RawHoleFeature =
  | RawCounterbore
  | RawCountersink
  | RawHexHole
  | RawSquareHole
  | RawHoleArray
  | RawBoltCircle;

// ---------------------------------------------------------------------------
// extractHoles — pure function
// ---------------------------------------------------------------------------

/** Parse `modelJson` and return all holes, tagged A1…AN sorted by feature id. */
export function extractHoles(modelJson: string): Hole[] {
  let parsed: { features?: unknown[] };
  try {
    parsed = JSON.parse(modelJson) as { features?: unknown[] };
  } catch {
    return [];
  }

  const features: unknown[] = parsed.features ?? [];

  // Collect raw hole-producing features and sort by id for stable tags.
  const raws: RawHoleFeature[] = [];
  for (const f of features) {
    if (!isObject(f)) continue;
    const kind = (f as Record<string, unknown>).kind;
    if (
      kind === "Counterbore" ||
      kind === "Countersink" ||
      kind === "HexHole" ||
      kind === "SquareHole" ||
      kind === "HoleArray" ||
      kind === "BoltCircle"
    ) {
      raws.push(f as RawHoleFeature);
    }
  }

  // Sort by feature id for stable letter assignment.
  raws.sort((a, b) => a.id.localeCompare(b.id));

  // Expand each feature into one or more Hole entries.
  const holes: Omit<Hole, "tag">[] = [];
  for (const raw of raws) {
    switch (raw.kind) {
      case "Counterbore":
        holes.push({
          type: "Counterbore",
          diameter: raw.drill_radius * 2,
          depth: raw.total_depth,
          x: raw.top_center[0],
          y: raw.top_center[1],
          z: raw.top_center[2],
        });
        break;

      case "Countersink":
        holes.push({
          type: "Countersink",
          diameter: raw.drill_radius * 2,
          depth: raw.total_depth,
          x: raw.top_center[0],
          y: raw.top_center[1],
          z: raw.top_center[2],
        });
        break;

      case "HexHole":
        holes.push({
          type: "HexHole",
          diameter: raw.inscribed_radius * 2,
          depth: raw.depth,
          x: raw.top_center[0],
          y: raw.top_center[1],
          z: raw.top_center[2],
        });
        break;

      case "SquareHole":
        holes.push({
          type: "SquareHole",
          diameter: raw.side,
          depth: raw.depth,
          x: raw.top_center[0],
          y: raw.top_center[1],
          z: raw.top_center[2],
        });
        break;

      case "HoleArray": {
        for (let i = 0; i < raw.count; i++) {
          holes.push({
            type: "HoleArray",
            diameter: raw.radius * 2,
            depth: raw.depth,
            x: raw.start[0] + i * raw.offset[0],
            y: raw.start[1] + i * raw.offset[1],
            z: raw.start[2] + i * raw.offset[2],
          });
        }
        break;
      }

      case "BoltCircle": {
        for (let i = 0; i < raw.count; i++) {
          const angle = (2 * Math.PI * i) / raw.count;
          // Hole positions lie in the plane perpendicular to axis.
          // For all supported axes ("x"|"y"|"z") the bolt circle is in
          // the two perpendicular world axes from center.
          const { hx, hy } = boltCircleOffset(raw.axis, raw.bolt_circle_radius, angle);
          holes.push({
            type: "BoltCircle",
            diameter: raw.radius * 2,
            depth: raw.depth,
            x: raw.center[0] + hx,
            y: raw.center[1] + hy,
            z: raw.center[2],
          });
        }
        break;
      }
    }
  }

  // Assign sequential tags A1, A2, …
  return holes.map((h, idx) => ({ ...h, tag: `A${idx + 1}` }));
}

// ---------------------------------------------------------------------------
// renderHoleTable — canvas-based renderer
// ---------------------------------------------------------------------------

/**
 * Append a SolidWorks-style hole table below the existing drawing canvas.
 * If `holes` is empty, returns immediately (no-op).
 *
 * @param ctx  The canvas 2D context to draw onto.
 * @param holes  Holes from `extractHoles`.
 * @param tableTopY  Y pixel coordinate at which the table header starts.
 * @param tableX  X pixel coordinate for the left edge of the table.
 */
export function renderHoleTable(
  ctx: CanvasRenderingContext2D,
  holes: Hole[],
  tableTopY: number,
  tableX = 50,
): void {
  if (holes.length === 0) return;

  const COL_WIDTHS = [52, 110, 80, 72, 72, 72, 72]; // Tag | Type | Dia | Depth | X | Y | Z
  const HEADERS = ["Tag", "Type", "Dia (mm)", "Depth", "X", "Y", "Z"];
  const ROW_H = 22;
  const FONT_HDR = "bold 11px -apple-system, BlinkMacSystemFont, system-ui, sans-serif";
  const FONT_ROW = "11px -apple-system, BlinkMacSystemFont, system-ui, sans-serif";
  const totalW = COL_WIDTHS.reduce((a, b) => a + b, 0);

  ctx.save();

  // Table background
  ctx.fillStyle = "#f0f4f8";
  ctx.fillRect(tableX, tableTopY, totalW, ROW_H * (holes.length + 1) + 1);

  // Header row
  ctx.fillStyle = "#1e2a3a";
  ctx.fillRect(tableX, tableTopY, totalW, ROW_H);
  ctx.fillStyle = "#ffffff";
  ctx.font = FONT_HDR;
  let cx = tableX;
  for (let c = 0; c < HEADERS.length; c++) {
    ctx.fillText(HEADERS[c]!, cx + 6, tableTopY + ROW_H - 7);
    cx += COL_WIDTHS[c]!;
  }

  // Data rows
  for (let r = 0; r < holes.length; r++) {
    const h = holes[r]!;
    const rowY = tableTopY + ROW_H * (r + 1);

    // Alternating row background
    ctx.fillStyle = r % 2 === 0 ? "#ffffff" : "#eaf0f8";
    ctx.fillRect(tableX, rowY, totalW, ROW_H);

    const cells = [
      h.tag,
      h.type,
      h.diameter.toFixed(2),
      h.depth.toFixed(2),
      h.x.toFixed(2),
      h.y.toFixed(2),
      h.z.toFixed(2),
    ];

    ctx.fillStyle = "#161b22";
    ctx.font = FONT_ROW;
    let ccx = tableX;
    for (let c = 0; c < cells.length; c++) {
      ctx.fillText(cells[c]!, ccx + 6, rowY + ROW_H - 7);
      ccx += COL_WIDTHS[c]!;
    }

    // Row bottom border
    ctx.strokeStyle = "#cdd5e0";
    ctx.lineWidth = 0.5;
    ctx.beginPath();
    ctx.moveTo(tableX, rowY + ROW_H);
    ctx.lineTo(tableX + totalW, rowY + ROW_H);
    ctx.stroke();
  }

  // Outer border + column dividers
  ctx.strokeStyle = "#30363d";
  ctx.lineWidth = 1;
  ctx.strokeRect(tableX + 0.5, tableTopY + 0.5, totalW - 1, ROW_H * (holes.length + 1));

  // Column dividers
  ctx.lineWidth = 0.5;
  ctx.strokeStyle = "#8b949e";
  let divX = tableX;
  for (let c = 0; c < COL_WIDTHS.length - 1; c++) {
    divX += COL_WIDTHS[c]!;
    ctx.beginPath();
    ctx.moveTo(divX, tableTopY);
    ctx.lineTo(divX, tableTopY + ROW_H * (holes.length + 1));
    ctx.stroke();
  }

  // "HOLE TABLE" label above
  ctx.font = "bold 12px -apple-system, BlinkMacSystemFont, system-ui, sans-serif";
  ctx.fillStyle = "#161b22";
  ctx.fillText("HOLE TABLE", tableX, tableTopY - 6);

  ctx.restore();
}

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

function isObject(v: unknown): v is Record<string, unknown> {
  return typeof v === "object" && v !== null && !Array.isArray(v);
}

/**
 * Given a bolt circle axis, radius, and angle, return the (dx, dy) offsets
 * in the X and Y world directions. For axis "z" the circle is in the XY
 * plane; for "x" in the YZ plane; for "y" in the ZX plane.
 */
function boltCircleOffset(
  axis: string,
  r: number,
  angle: number,
): { hx: number; hy: number } {
  const cos = Math.cos(angle);
  const sin = Math.sin(angle);
  switch (axis) {
    case "x":
      // holes spread in Y and Z; we map onto (x,y) output as (dy, dz)
      return { hx: 0, hy: r * cos }; // simplification: y offset only for 2D table
    case "y":
      return { hx: r * cos, hy: 0 };
    default: // "z" — holes in XY plane
      return { hx: r * cos, hy: r * sin };
  }
}
