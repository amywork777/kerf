/**
 * Pure functions for feature-tree operations: reorder and filter.
 * No DOM, no WASM — safe to import from Node test runners.
 */

export type FeatureSummary = { id: string; kind: string };

/**
 * Reorder features in a model JSON string.
 *
 * Removes `srcId` from its current position and inserts it immediately
 * before or after `dstId`. Returns new JSON (same formatting as a
 * `JSON.stringify(..., null, 2)` round-trip).
 *
 * Throws if either id is not found in the features array.
 */
export function reorderFeatures(
  json: string,
  srcId: string,
  dstId: string,
  position: "before" | "after",
): string {
  const parsed = JSON.parse(json) as { features?: Array<{ id?: string }> };
  const features = (parsed.features ?? []) as Array<Record<string, unknown>>;

  const srcIdx = features.findIndex((f) => f.id === srcId);
  if (srcIdx === -1) {
    throw new Error(`reorderFeatures: source id '${srcId}' not found`);
  }
  const dstIdx = features.findIndex((f) => f.id === dstId);
  if (dstIdx === -1) {
    throw new Error(`reorderFeatures: destination id '${dstId}' not found`);
  }

  // Same source and destination → no-op.
  if (srcId === dstId) {
    return JSON.stringify(parsed, null, 2);
  }

  // Remove source from its current position.
  const [srcFeature] = features.splice(srcIdx, 1) as [Record<string, unknown>];

  // After removal, find the new index of the destination.
  const newDstIdx = features.findIndex((f) => f.id === dstId);

  // Insert before or after dstId.
  const insertAt = position === "before" ? newDstIdx : newDstIdx + 1;
  features.splice(insertAt, 0, srcFeature);

  parsed.features = features;
  return JSON.stringify(parsed, null, 2);
}

/**
 * Filter a list of FeatureSummary objects by a search query.
 *
 * Matches are case-insensitive substring matches against both `id` and `kind`.
 * Empty/whitespace-only query returns the full list unchanged.
 */
export function filterFeatures(
  features: FeatureSummary[],
  query: string,
): FeatureSummary[] {
  const q = query.trim().toLowerCase();
  if (q === "") return features;
  return features.filter(
    (f) =>
      f.id.toLowerCase().includes(q) || f.kind.toLowerCase().includes(q),
  );
}
