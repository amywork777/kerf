/**
 * DOF (degrees-of-freedom) calculator for the 2D sketcher.
 *
 * Formula:
 *   DOF = sum(primitive DOF) - sum(constraint DOF removed)
 *
 * Primitive DOF:
 *   Point  → 2
 *   Line   → 0  (endpoints are Points; the Line prim itself adds no DOF)
 *   Circle → 1  (center is a Point; radius adds 1 DOF)
 *   Arc    → 1  (center is a Point; radius + angles — but angles are
 *               determined by endpoint Points in most solvers; for v1 we
 *               treat radius as 1 extra DOF just like Circle)
 *   Other  → 0
 *
 * Constraint DOF removed:
 *   Coincident(a, b)  → 2  (binds x and y of one point to another)
 *   Distance          → 1
 *   Horizontal        → 1
 *   Vertical          → 1
 *   Parallel          → 1
 *   Perpendicular     → 1
 *   FixedPoint        → 2  (fixes x and y)
 *
 * No DOM dependencies — safe to import from Node tests.
 */

import type { Sketch, SketchPrim, SketchConstraint } from "./sketcher.js";

export type DofResult = {
  constraints: number;
  dof: number;
  status: "fully-constrained" | "underconstrained" | "overconstrained";
};

function primDof(prim: SketchPrim): number {
  switch (prim.kind) {
    case "Point":
      return 2;
    case "Line":
      // A Line prim references two Points. The Points carry their own DOF;
      // the Line prim itself is pure topology with no extra DOF.
      return 0;
    case "Circle":
    case "Arc":
      // Center is a Point (carries 2 DOF). Radius adds 1 more.
      return 1;
    default:
      return 0;
  }
}

function constraintDofRemoved(c: SketchConstraint): number {
  switch (c.kind) {
    case "Coincident":
      return 2;
    case "FixedPoint":
      return 2;
    case "Distance":
    case "Horizontal":
    case "Vertical":
    case "Parallel":
    case "Perpendicular":
      return 1;
  }
}

/**
 * Compute the DOF of the given sketch.
 *
 * Returns the total primitive DOF, the constraint count, the net DOF
 * remaining, and a status string.
 */
export function computeDof(sketch: Sketch): DofResult {
  let totalDof = 0;
  for (const prim of sketch.primitives) {
    totalDof += primDof(prim);
  }

  let totalRemoved = 0;
  for (const c of sketch.constraints) {
    totalRemoved += constraintDofRemoved(c);
  }

  const dof = totalDof - totalRemoved;
  const constraints = sketch.constraints.length;

  let status: DofResult["status"];
  if (dof === 0) {
    status = "fully-constrained";
  } else if (dof > 0) {
    status = "underconstrained";
  } else {
    status = "overconstrained";
  }

  return { constraints, dof, status };
}

/**
 * Format the DOF result for display in a status bar.
 */
export function formatDof(result: DofResult): string {
  if (result.status === "fully-constrained") {
    return `Solver: ${result.constraints} constraints, fully constrained`;
  }
  return `Solver: ${result.constraints} constraints, ${result.dof} DOF remaining`;
}
