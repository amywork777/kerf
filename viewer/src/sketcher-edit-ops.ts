/**
 * sketcher-edit-ops.ts — Pure functional edit operations over Sketch.
 *
 * All functions are side-effect-free: they take a Sketch (and parameters) and
 * return a new Sketch (or SketchFragment). The source sketch is never mutated.
 *
 * Operations:
 *   - copySelection  — extract a subset of primitives as a fragment
 *   - pasteFragment  — merge a fragment into a sketch at an offset
 *   - mirrorSelection — reflect selected primitives across an axis line
 *   - patternRect    — replicate selected primitives as an N×M grid
 */

import { type Sketch, type SketchPrim } from "./sketcher.js";

// ---------------------------------------------------------------------------
// Types
// ---------------------------------------------------------------------------

export type SketchFragment = {
  primitives: SketchPrim[];
};

type Pt2 = { x: number; y: number };

// ---------------------------------------------------------------------------
// ID helpers
// ---------------------------------------------------------------------------

/**
 * Build a Set of all existing ids in a sketch, so we can generate fresh ones
 * that avoid collisions.
 */
function existingIds(sketch: Sketch): Set<string> {
  return new Set(sketch.primitives.map(p => p.id));
}

/**
 * Given a set of taken ids and a desired prefix, produce a fresh id not in
 * `taken`, then add it to `taken` so the same id won't be returned twice in
 * the same batch.
 */
function freshId(taken: Set<string>, prefix: string): string {
  let seq = 1;
  while (true) {
    const candidate = `${prefix}${seq++}`;
    if (!taken.has(candidate)) {
      taken.add(candidate);
      return candidate;
    }
  }
}

// ---------------------------------------------------------------------------
// 1. copySelection
// ---------------------------------------------------------------------------

/**
 * Returns a SketchFragment containing only the primitives whose ids appear in
 * `ids`. The primitives are deep-copied so the caller can't accidentally share
 * references with the source sketch.
 */
export function copySelection(sketch: Sketch, ids: string[]): SketchFragment {
  const idSet = new Set(ids);
  const primitives = sketch.primitives
    .filter(p => idSet.has(p.id))
    .map(p => ({ ...p })); // shallow copy of each prim (all fields are scalars)
  return { primitives };
}

// ---------------------------------------------------------------------------
// 2. pasteFragment
// ---------------------------------------------------------------------------

/**
 * Merges `fragment` into `sketch`, shifting every Point in the fragment by
 * `offset`.  All primitives in the fragment receive fresh ids to avoid
 * collisions with existing ids. Non-Point primitives (Lines, Circles, etc.)
 * that reference Point ids within the fragment have their reference fields
 * remapped to the new ids.
 *
 * Returns a new Sketch; `sketch` is not mutated.
 */
export function pasteFragment(sketch: Sketch, fragment: SketchFragment, offset: Pt2): Sketch {
  const taken = existingIds(sketch);

  // Build an id-remap table: old id → new id for every primitive in the fragment.
  const remap = new Map<string, string>();
  for (const prim of fragment.primitives) {
    const prefix = primIdPrefix(prim);
    remap.set(prim.id, freshId(taken, prefix));
  }

  // Re-emit each primitive with remapped ids and shifted Point coordinates.
  const newPrims: SketchPrim[] = fragment.primitives.map(prim =>
    remapPrim(prim, remap, offset)
  );

  return {
    plane: sketch.plane,
    primitives: [...sketch.primitives.map(p => ({ ...p })), ...newPrims],
    constraints: [...sketch.constraints],
  };
}

// ---------------------------------------------------------------------------
// 3. mirrorSelection
// ---------------------------------------------------------------------------

/**
 * Reflects the selected primitives across the given axis line (defined by two
 * world-space points). The mirrored copies are added to the sketch alongside
 * the originals.
 *
 * Returns a new Sketch; `sketch` is not mutated.
 */
export function mirrorSelection(
  sketch: Sketch,
  ids: string[],
  axis: { from: Pt2; to: Pt2 },
): Sketch {
  const fragment = copySelection(sketch, ids);
  const taken = existingIds(sketch);

  // Build id-remap for mirrored copies.
  const remap = new Map<string, string>();
  for (const prim of fragment.primitives) {
    const prefix = primIdPrefix(prim);
    remap.set(prim.id, freshId(taken, prefix + "m"));
  }

  // Reflect each primitive: Points get mirrored coordinates; other prims get
  // their reference fields remapped but no coordinate change of their own
  // (their geometry is determined by their referenced Points).
  const mirroredPrims: SketchPrim[] = fragment.primitives.map(prim => {
    if (prim.kind === "Point") {
      const { x, y } = reflectPoint(prim.x, prim.y, axis.from, axis.to);
      return { kind: "Point", id: remap.get(prim.id)!, x, y };
    }
    // Remap reference ids; no offset (pass { x:0, y:0 }).
    return remapPrim(prim, remap, { x: 0, y: 0 });
  });

  return {
    plane: sketch.plane,
    primitives: [...sketch.primitives.map(p => ({ ...p })), ...mirroredPrims],
    constraints: [...sketch.constraints],
  };
}

// ---------------------------------------------------------------------------
// 4. patternRect
// ---------------------------------------------------------------------------

/**
 * Replicates the selected primitives in an nx × ny rectangular grid. The
 * first copy (i=0, j=0) is the original — it is always included. Copies at
 * (i, j) are offset by (i * dx, j * dy).
 *
 * Returns a new Sketch; `sketch` is not mutated.
 */
export function patternRect(
  sketch: Sketch,
  ids: string[],
  dx: number,
  dy: number,
  nx: number,
  ny: number,
): Sketch {
  const fragment = copySelection(sketch, ids);
  const taken = existingIds(sketch);

  const allNewPrims: SketchPrim[] = [];

  for (let i = 0; i < nx; i++) {
    for (let j = 0; j < ny; j++) {
      if (i === 0 && j === 0) {
        // The originals stay at their positions.
        continue;
      }
      const offset: Pt2 = { x: i * dx, y: j * dy };
      // Build a fresh remap for this copy.
      const remap = new Map<string, string>();
      for (const prim of fragment.primitives) {
        const prefix = primIdPrefix(prim);
        remap.set(prim.id, freshId(taken, prefix));
      }
      for (const prim of fragment.primitives) {
        allNewPrims.push(remapPrim(prim, remap, offset));
      }
    }
  }

  return {
    plane: sketch.plane,
    primitives: [...sketch.primitives.map(p => ({ ...p })), ...allNewPrims],
    constraints: [...sketch.constraints],
  };
}

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

/**
 * Re-emit a primitive with a remapped id (and remapped reference ids for
 * Lines/Circles/Arcs), with Points shifted by `offset`.
 */
function remapPrim(prim: SketchPrim, remap: Map<string, string>, offset: Pt2): SketchPrim {
  const newId = remap.get(prim.id) ?? prim.id;
  switch (prim.kind) {
    case "Point":
      return { kind: "Point", id: newId, x: prim.x + offset.x, y: prim.y + offset.y };
    case "Line":
      return {
        kind: "Line",
        id: newId,
        from: remap.get(prim.from) ?? prim.from,
        to: remap.get(prim.to) ?? prim.to,
      };
    case "Circle":
      return {
        kind: "Circle",
        id: newId,
        center: remap.get(prim.center) ?? prim.center,
        radius: prim.radius,
        n_segments: prim.n_segments,
      };
    case "Arc":
      return {
        kind: "Arc",
        id: newId,
        center: remap.get(prim.center) ?? prim.center,
        radius: prim.radius,
        start_angle: prim.start_angle,
        end_angle: prim.end_angle,
        n_segments: prim.n_segments,
      };
    case "TrimLine":
      return {
        kind: "TrimLine",
        id: newId,
        line: remap.get(prim.line) ?? prim.line,
        at_point: remap.get(prim.at_point) ?? prim.at_point,
      };
    case "ExtendLine":
      return {
        kind: "ExtendLine",
        id: newId,
        line: remap.get(prim.line) ?? prim.line,
        to_point: remap.get(prim.to_point) ?? prim.to_point,
      };
    case "FilletCorner":
      return {
        kind: "FilletCorner",
        id: newId,
        corner_point: remap.get(prim.corner_point) ?? prim.corner_point,
        radius: prim.radius,
      };
  }
}

/** Guess a short id prefix from the prim kind. */
function primIdPrefix(prim: SketchPrim): string {
  switch (prim.kind) {
    case "Point":       return "p";
    case "Line":        return "l";
    case "Circle":      return "k";
    case "Arc":         return "a";
    case "TrimLine":    return "tr";
    case "ExtendLine":  return "ex";
    case "FilletCorner": return "f";
  }
}

/**
 * Reflect point (px, py) across the line defined by two world-space points.
 * Standard formula: project onto the line, then double the displacement.
 */
function reflectPoint(px: number, py: number, from: Pt2, to: Pt2): Pt2 {
  const ax = to.x - from.x;
  const ay = to.y - from.y;
  const len2 = ax * ax + ay * ay;
  if (len2 < 1e-12) return { x: px, y: py }; // degenerate axis
  // t = dot(P - A, axis) / |axis|²
  const t = ((px - from.x) * ax + (py - from.y) * ay) / len2;
  // Foot of perpendicular (projection).
  const fx = from.x + t * ax;
  const fy = from.y + t * ay;
  // Reflection = 2 * foot - P.
  return { x: 2 * fx - px, y: 2 * fy - py };
}
