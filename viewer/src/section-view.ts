// section-view.ts — pure geometry: clips a triangle mesh with a plane and
// returns 2-D polyline segments forming the cross-section contour.
// No Three.js, no DOM, no WASM.  Unit-tested independently.

export interface Triangle {
  a: [number, number, number];
  b: [number, number, number];
  c: [number, number, number];
}

export interface Plane {
  /** Unit normal (need not be normalised — will be normalised internally). */
  normal: [number, number, number];
  /** Signed distance from origin along normal. */
  offset: number;
}

export interface Seg2D {
  x1: number;
  y1: number;
  x2: number;
  y2: number;
}

export interface SectionViewParams {
  plane: Plane;
  hatchAngle?: number;   // degrees, default 45
  hatchSpacing?: number; // px, default 4
}

// ---------------------------------------------------------------------------
// Pure maths helpers
// ---------------------------------------------------------------------------

function dot(a: [number, number, number], b: [number, number, number]): number {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

function len(v: [number, number, number]): number {
  return Math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

function normalize(v: [number, number, number]): [number, number, number] {
  const l = len(v);
  if (l < 1e-12) return [0, 0, 0];
  return [v[0] / l, v[1] / l, v[2] / l];
}

/** Signed distance from point p to plane. */
function planeDist(p: [number, number, number], n: [number, number, number], offset: number): number {
  return dot(p, n) - offset;
}

/** Linearly interpolate between a and b at parameter t ∈ [0,1]. */
function lerp3(
  a: [number, number, number],
  b: [number, number, number],
  t: number,
): [number, number, number] {
  return [a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]), a[2] + t * (b[2] - a[2])];
}

// ---------------------------------------------------------------------------
// Coordinate system on the cut plane — used to project 3-D intersection
// points down to 2-D for the returned Seg2D values.
// ---------------------------------------------------------------------------

function buildPlaneAxes(
  n: [number, number, number],
): { u: [number, number, number]; v: [number, number, number] } {
  // Pick a vector not parallel to n.
  const ref: [number, number, number] =
    Math.abs(n[0]) < 0.9 ? [1, 0, 0] : [0, 1, 0];
  // u = ref × n  (then normalise)
  const uRaw: [number, number, number] = [
    ref[1] * n[2] - ref[2] * n[1],
    ref[2] * n[0] - ref[0] * n[2],
    ref[0] * n[1] - ref[1] * n[0],
  ];
  const u = normalize(uRaw);
  // v = n × u
  const vRaw: [number, number, number] = [
    n[1] * u[2] - n[2] * u[1],
    n[2] * u[0] - n[0] * u[2],
    n[0] * u[1] - n[1] * u[0],
  ];
  const v = normalize(vRaw);
  return { u, v };
}

function project2D(
  p: [number, number, number],
  u: [number, number, number],
  v: [number, number, number],
): [number, number] {
  return [dot(p, u), dot(p, v)];
}

// ---------------------------------------------------------------------------
// Main export
// ---------------------------------------------------------------------------

/**
 * Clips every triangle in `tris` against `plane` and returns the set of
 * 2-D line segments where the plane intersects the triangle soup.
 *
 * The 2-D coordinate system lives in the cut plane itself:
 *   x′ = dot(p, u)   y′ = dot(p, v)
 * where u, v are orthonormal axes spanning the plane.
 *
 * Scaling: 1 world-unit = 1 "pixel" in Seg2D space.  Callers should scale
 * to fit their canvas.
 */
export function clipModelToSection(tris: Triangle[], plane: Plane): Seg2D[] {
  const n = normalize(plane.normal);
  const { u, v } = buildPlaneAxes(n);
  const result: Seg2D[] = [];

  for (const tri of tris) {
    const verts: [number, number, number][] = [tri.a, tri.b, tri.c];
    const dists = verts.map((p) => planeDist(p, n, plane.offset));

    // Collect intersection points (the plane crosses an edge if the two
    // endpoint distances have opposite signs, or one is exactly zero).
    const intersections: [number, number, number][] = [];

    for (let i = 0; i < 3; i++) {
      const j = (i + 1) % 3;
      const di = dists[i]!;
      const dj = dists[j]!;
      const pi = verts[i]!;
      const pj = verts[j]!;

      if (Math.abs(di) < 1e-10) {
        // Vertex i is exactly on the plane.
        intersections.push(pi);
      } else if (di * dj < 0) {
        // Edge crosses the plane — linear interpolation.
        const t = di / (di - dj);
        intersections.push(lerp3(pi, pj, t));
      }
    }

    // Deduplicate very close points.
    const unique: [number, number, number][] = [];
    for (const p of intersections) {
      const dup = unique.some(
        (q) => Math.abs(q[0] - p[0]) < 1e-9 && Math.abs(q[1] - p[1]) < 1e-9 && Math.abs(q[2] - p[2]) < 1e-9,
      );
      if (!dup) unique.push(p);
    }

    if (unique.length >= 2) {
      const [p2a, p2b] = [project2D(unique[0]!, u, v), project2D(unique[1]!, u, v)];
      result.push({ x1: p2a[0], y1: p2a[1], x2: p2b[0], y2: p2b[1] });
    }
  }

  return result;
}

// ---------------------------------------------------------------------------
// Canvas rendering helper — draws the cross-section with hatching inside
// an axis-aligned bounding box on the caller's canvas context.
// ---------------------------------------------------------------------------

export interface SectionRenderOpts {
  segs: Seg2D[];
  hatchAngle?: number;   // degrees, default 45
  hatchSpacing?: number; // px, default 4
  /** Destination rectangle on the target canvas [px]. */
  destX: number;
  destY: number;
  destW: number;
  destH: number;
  /** Label shown at the top of the pane. */
  label: string;
}

export function renderSectionPanel(
  ctx: CanvasRenderingContext2D,
  opts: SectionRenderOpts,
): void {
  const { segs, destX, destY, destW, destH, label } = opts;
  const hatchAngle = opts.hatchAngle ?? 45;
  const hatchSpacing = opts.hatchSpacing ?? 4;

  // Background + border.
  ctx.fillStyle = "#ffffff";
  ctx.fillRect(destX, destY, destW, destH);
  ctx.strokeStyle = "#30363d";
  ctx.lineWidth = 1;
  ctx.strokeRect(destX + 0.5, destY + 0.5, destW - 1, destH - 1);

  // Label.
  ctx.font = "bold 14px -apple-system, BlinkMacSystemFont, system-ui, sans-serif";
  ctx.fillStyle = "#161b22";
  ctx.fillText(`SECTION ${label}`, destX, destY - 6);

  if (segs.length === 0) return;

  // Compute 2-D bounding box of all segments so we can auto-fit.
  let minX = Infinity, minY = Infinity, maxX = -Infinity, maxY = -Infinity;
  for (const s of segs) {
    minX = Math.min(minX, s.x1, s.x2);
    minY = Math.min(minY, s.y1, s.y2);
    maxX = Math.max(maxX, s.x1, s.x2);
    maxY = Math.max(maxY, s.y1, s.y2);
  }
  const rangeX = maxX - minX || 1;
  const rangeY = maxY - minY || 1;
  const INNER_PAD = 16;
  const scaleX = (destW - INNER_PAD * 2) / rangeX;
  const scaleY = (destH - INNER_PAD * 2) / rangeY;
  const scale = Math.min(scaleX, scaleY);

  function toCanvas(x: number, y: number): [number, number] {
    return [
      destX + INNER_PAD + (x - minX) * scale,
      destY + INNER_PAD + (y - minY) * scale,
    ];
  }

  // --- Draw hatching first (inside cross-section bounds) ---
  // We clip to a rough convex bounding box then draw diagonal lines.
  const cvW = rangeX * scale;
  const cvH = rangeY * scale;
  const originX = destX + INNER_PAD;
  const originY = destY + INNER_PAD;

  ctx.save();
  ctx.beginPath();
  ctx.rect(originX, originY, cvW, cvH);
  ctx.clip();

  const radians = (hatchAngle * Math.PI) / 180;
  const cosA = Math.cos(radians);
  const sinA = Math.sin(radians);
  const diagonal = Math.hypot(cvW, cvH);
  ctx.strokeStyle = "#aabbcc";
  ctx.lineWidth = 0.75;
  for (let d = -diagonal; d <= diagonal * 2; d += hatchSpacing) {
    const x1 = originX + cosA * (-diagonal) - sinA * d;
    const y1 = originY + sinA * (-diagonal) + cosA * d;
    const x2 = originX + cosA * (diagonal) - sinA * d;
    const y2 = originY + sinA * (diagonal) + cosA * d;
    ctx.beginPath();
    ctx.moveTo(x1, y1);
    ctx.lineTo(x2, y2);
    ctx.stroke();
  }
  ctx.restore();

  // --- Draw cross-section outlines on top of hatching ---
  ctx.strokeStyle = "#1a3a6b";
  ctx.lineWidth = 2;
  for (const s of segs) {
    const [ax, ay] = toCanvas(s.x1, s.y1);
    const [bx, by] = toCanvas(s.x2, s.y2);
    ctx.beginPath();
    ctx.moveTo(ax, ay);
    ctx.lineTo(bx, by);
    ctx.stroke();
  }
}
