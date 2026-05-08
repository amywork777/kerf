// Pure helpers for the picking → fillet flow. Kept DOM-free so vitest can
// import them in a node environment without bringing in the WASM bundle.

/** One entry in the rebuild result's `edges` array. Mirrors `EdgeOut` in
 *  crates/kerf-cad-wasm/src/lib.rs — keep field names in sync. */
export type EdgeOut = {
  id: number;
  owner_tag: string;
  p_start: [number, number, number];
  p_end: [number, number, number];
  length: number;
  curve_kind: string;
  axis_hint: string;
  edge_min_hint: [number, number, number];
  edge_length_hint: number;
  quadrant_hint: string;
};

/** Build a Fillet feature object given the input target id, the edge
 *  metadata, and a radius. The new feature `id` mixes the edge id with a
 *  random suffix so multiple fillets on the same edge don't collide. */
export function buildFilletFeature(
  inputTarget: string,
  e: Pick<
    EdgeOut,
    "id" | "axis_hint" | "edge_min_hint" | "edge_length_hint" | "quadrant_hint"
  >,
  radius: number,
  randSuffix?: string,
): Record<string, unknown> {
  const suffix = randSuffix ?? Math.random().toString(36).slice(2, 7);
  return {
    kind: "Fillet",
    id: `fillet_e${e.id}_${suffix}`,
    input: inputTarget,
    axis: e.axis_hint,
    edge_min: e.edge_min_hint,
    edge_length: e.edge_length_hint,
    radius,
    quadrant: e.quadrant_hint,
    segments: 8,
  };
}

/** True when an edge can be turned into a Fillet feature: it must be a
 *  straight line, axis-aligned (axis_hint set), and the body must occupy a
 *  resolvable quadrant relative to the edge. */
export function isEdgeFilletable(e: EdgeOut): boolean {
  return e.curve_kind === "line" && !!e.axis_hint && !!e.quadrant_hint;
}

/** Append a Fillet feature to a model JSON string and return the updated
 *  JSON plus the new feature's id. The new feature's `input` is the
 *  current target so the chain stays connected. Throws on invalid input
 *  JSON; returns null on a non-filletable edge. */
export function applyFilletToModelJson(
  json: string,
  currentTargetId: string,
  edge: EdgeOut,
  radius: number,
  randSuffix?: string,
): { json: string; newId: string; ids: string[] } | null {
  if (!isEdgeFilletable(edge)) return null;
  const parsed = JSON.parse(json);
  const features = Array.isArray(parsed.features)
    ? (parsed.features as Array<Record<string, unknown>>)
    : [];
  const fillet = buildFilletFeature(currentTargetId, edge, radius, randSuffix);
  features.push(fillet);
  parsed.features = features;
  const newId = String(fillet.id);
  const ids = features
    .map((f) => f?.id)
    .filter((x): x is string => typeof x === "string");
  return {
    json: JSON.stringify(parsed, null, 2),
    newId,
    ids,
  };
}
