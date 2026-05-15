/**
 * pick-loops.ts — Pure-logic helpers for edge-loop and face-loop selection.
 *
 * No Three.js / DOM imports — these are plain data functions so they can be
 * tested in Node without a browser environment.
 */

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

/** Returns all entries in a CSR row without requiring a Uint32Array. */
function csrRow(offsets: ArrayLike<number>, indices: ArrayLike<number>, i: number): number[] {
  if (i < 0 || i + 1 >= offsets.length) return [];
  const lo = offsets[i]!;
  const hi = offsets[i + 1]!;
  const out: number[] = [];
  for (let k = lo; k < hi; k++) out.push(indices[k]!);
  return out;
}

// ---------------------------------------------------------------------------
// Edge-loop selection
// ---------------------------------------------------------------------------

/**
 * Walk an edge loop starting from `seedEdgeId`.
 *
 * An "edge loop" is defined as the set of edges that all border the same
 * face as the seed edge (the face boundary ring). For edges that border
 * multiple faces, we pick the face that produces the longest closed loop,
 * falling back to the first face in edgeToFaces order.
 *
 * Walk: starting from one endpoint of the seed, find the next edge at that
 * vertex that also borders the chosen loop-face, advance to the far endpoint,
 * repeat until the loop closes or no continuation exists.
 *
 * @param seedEdgeId       Index of the edge to start from.
 * @param edgeEndpoints    Flat array [v0, v1, v0, v1, …] of vertex indices per edge.
 * @param vertexToEdges    CSR structure: {offsets, indices} mapping vertex → edge ids.
 * @param edgeToFaces      CSR structure: {offsets, indices} mapping edge → face ids.
 * @returns                Ordered list of edge ids in the loop (seed is first).
 */
export function findEdgeLoop(
  seedEdgeId: number,
  edgeEndpoints: ArrayLike<number>,
  vertexToEdges: { offsets: ArrayLike<number>; indices: ArrayLike<number> },
  edgeToFaces: { offsets: ArrayLike<number>; indices: ArrayLike<number> },
): number[] {
  const nEdges = edgeEndpoints.length / 2;
  if (seedEdgeId < 0 || seedEdgeId >= nEdges) return [seedEdgeId];

  const seedVa = edgeEndpoints[seedEdgeId * 2]!;
  const seedVb = edgeEndpoints[seedEdgeId * 2 + 1]!;
  if (seedVa === 0xffffffff || seedVb === 0xffffffff) return [seedEdgeId];

  const seedFaces = csrRow(edgeToFaces.offsets, edgeToFaces.indices, seedEdgeId);
  if (seedFaces.length === 0) return [seedEdgeId];

  /**
   * Walk the face boundary ring that contains the seed edge, staying on
   * `loopFace` the whole time.  At each vertex, look for the next unvisited
   * edge that also borders `loopFace`.  Returns { loop, closed }.
   */
  function walkFaceBoundary(loopFace: number): { loop: number[]; closed: boolean } {
    const loop: number[] = [seedEdgeId];
    const visited = new Set<number>([seedEdgeId]);
    let curVtx = seedVb;

    for (let step = 0; step < nEdges; step++) {
      const vtxEdges = csrRow(vertexToEdges.offsets, vertexToEdges.indices, curVtx);
      let nextEid = -1;

      for (const eid of vtxEdges) {
        if (visited.has(eid)) continue;
        const eFaces = csrRow(edgeToFaces.offsets, edgeToFaces.indices, eid);
        if (eFaces.includes(loopFace)) { nextEid = eid; break; }
      }

      if (nextEid < 0) {
        // Dead end from B side — try continuing from A side.
        break;
      }

      loop.push(nextEid);
      visited.add(nextEid);

      const va = edgeEndpoints[nextEid * 2]!;
      const vb = edgeEndpoints[nextEid * 2 + 1]!;
      curVtx = va === curVtx ? vb : va;

      // Check if we've looped back to the start (seedVa).
      if (curVtx === seedVa) return { loop, closed: true };
    }

    // Continue from the A side of the seed.
    curVtx = seedVa;
    for (let step = 0; step < nEdges; step++) {
      const vtxEdges = csrRow(vertexToEdges.offsets, vertexToEdges.indices, curVtx);
      let nextEid = -1;

      for (const eid of vtxEdges) {
        if (visited.has(eid)) continue;
        const eFaces = csrRow(edgeToFaces.offsets, edgeToFaces.indices, eid);
        if (eFaces.includes(loopFace)) { nextEid = eid; break; }
      }

      if (nextEid < 0) break;

      loop.push(nextEid);
      visited.add(nextEid);

      const va = edgeEndpoints[nextEid * 2]!;
      const vb = edgeEndpoints[nextEid * 2 + 1]!;
      curVtx = va === curVtx ? vb : va;
    }

    return { loop, closed: false };
  }

  // Try each bordering face; pick the face that yields the longest closed loop
  // (falling back to longest open chain if no closed loop found).
  let bestLoop: number[] = [seedEdgeId];
  let bestClosed = false;

  for (const face of seedFaces) {
    const { loop, closed } = walkFaceBoundary(face);
    const better =
      (closed && !bestClosed) ||
      (closed === bestClosed && loop.length > bestLoop.length);
    if (better) {
      bestLoop = loop;
      bestClosed = closed;
    }
  }

  return bestLoop;
}

// ---------------------------------------------------------------------------
// Face-loop selection
// ---------------------------------------------------------------------------

/**
 * Walk a face ring starting from `seedFaceId`.
 *
 * Returns the seed face plus all faces directly adjacent to it in the
 * face-adjacency graph.  For simple prismatic bodies (cylinders, boxes)
 * this gives the lateral face band.
 *
 * @param seedFaceId     Index of the face to start from.
 * @param faceAdjacency  CSR structure: {offsets, indices} mapping face → adjacent face ids.
 * @returns              List of face ids in the ring (seed is first).
 */
export function findFaceLoop(
  seedFaceId: number,
  faceAdjacency: { offsets: ArrayLike<number>; indices: ArrayLike<number> },
): number[] {
  const nFaces = faceAdjacency.offsets.length - 1;
  if (seedFaceId < 0 || seedFaceId >= nFaces) return [seedFaceId];

  const ring: number[] = [seedFaceId];
  const visited = new Set<number>([seedFaceId]);

  const adjacent = csrRow(faceAdjacency.offsets, faceAdjacency.indices, seedFaceId);
  for (const adj of adjacent) {
    if (!visited.has(adj)) {
      ring.push(adj);
      visited.add(adj);
    }
  }

  return ring;
}
