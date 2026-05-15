/**
 * pick-filter.ts — pure logic for face-owner-tag filtering.
 *
 * `applyFilter` returns true when a face should be pickable / highlightable
 * given the current filter kind and the face's owner metadata.
 *
 * It is intentionally free of DOM and Three.js imports so it can be tested
 * in a plain Node / Vitest environment.
 */

export type FilterKind = "All" | string;

/**
 * A minimal slice of the parsed model JSON that we need to look up a
 * feature's kind by its id.
 */
export type ParsedFeature = { id: string; kind: string };

/**
 * Decide whether a face should pass the current filter.
 *
 * @param faceId        The numeric face id (0-based, as returned by WASM).
 * @param faceOwnerTags A map from face-id (as number) → owner-feature-id
 *                      string, as extracted from the WASM response.
 * @param parsedFeatures The features array from the loaded model JSON.
 * @param filterKind    The currently selected filter ("All" or a feature kind
 *                      string such as "Box", "Cylinder", "Fillet", …).
 * @returns             true → the face passes the filter and may be picked.
 */
export function applyFilter(
  faceId: number,
  faceOwnerTags: Map<number, string>,
  parsedFeatures: ParsedFeature[],
  filterKind: FilterKind,
): boolean {
  // "All" — no filtering, everything is pickable.
  if (filterKind === "All") return true;

  // Look up the owner feature id for this face.
  const ownerFeatureId = faceOwnerTags.get(faceId);

  // No owner tag recorded → pass through gracefully (don't block picking).
  if (ownerFeatureId === undefined) return true;

  // Find the feature in the parsed feature list.
  const feature = parsedFeatures.find((f) => f.id === ownerFeatureId);

  // Feature not found in the list → pass through gracefully.
  if (!feature) return true;

  return feature.kind === filterKind;
}

/**
 * Extract a sorted, deduplicated list of feature kinds present in the model.
 * Used to populate the filter dropdown.
 */
export function collectFeatureKinds(parsedFeatures: ParsedFeature[]): string[] {
  const kinds = new Set<string>();
  for (const f of parsedFeatures) {
    if (f.kind && f.kind !== "?") kinds.add(f.kind);
  }
  return Array.from(kinds).sort();
}
