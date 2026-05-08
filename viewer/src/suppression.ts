/// Pure JSON helpers for the suppression + rollback feature-tree UI.
/// Extracted from main.ts so the helpers can be unit-tested without
/// pulling in the WASM module or any DOM bindings.

/// Read the suppressed-id set from the model JSON.
export function suppressedFromJson(json: string): Set<string> {
  try {
    const parsed = JSON.parse(json);
    const arr = parsed?.suppressed;
    if (Array.isArray(arr)) {
      return new Set(arr.filter((x: unknown): x is string => typeof x === "string"));
    }
  } catch {
    /* fall through */
  }
  return new Set();
}

/// Read the rollback marker (id of last active feature) from the model JSON.
/// Returns `null` when no marker is set.
export function rollbackFromJson(json: string): string | null {
  try {
    const parsed = JSON.parse(json);
    const r = parsed?.rollback_to;
    return typeof r === "string" ? r : null;
  } catch {
    return null;
  }
}

/// Update the model JSON with a new `suppressed` array (or omit when empty).
/// IDs are sorted to match the Rust side's serialization.
export function setSuppressedInJson(json: string, ids: Iterable<string>): string {
  const parsed = JSON.parse(json);
  const arr = Array.from(ids).sort();
  if (arr.length === 0) {
    delete parsed.suppressed;
  } else {
    parsed.suppressed = arr;
  }
  return JSON.stringify(parsed, null, 2);
}

/// Update the model JSON's `rollback_to` (or remove it when null/undefined).
export function setRollbackInJson(json: string, id: string | null): string {
  const parsed = JSON.parse(json);
  if (id === null) {
    delete parsed.rollback_to;
  } else {
    parsed.rollback_to = id;
  }
  return JSON.stringify(parsed, null, 2);
}

/// Toggle a single id's membership in the suppressed set, returning the
/// updated JSON. Convenience for checkbox handlers.
export function toggleSuppressedInJson(json: string, id: string, suppress: boolean): string {
  const cur = suppressedFromJson(json);
  if (suppress) {
    cur.add(id);
  } else {
    cur.delete(id);
  }
  return setSuppressedInJson(json, cur);
}

/// Compute the rollback id from a bar position. `barIndex` is the bar's
/// position 0..features.length: `0` would put the bar above feature[0]
/// (no feature active — represented as the first feature's id, since we
/// always keep at least one feature visible to avoid an empty model);
/// `features.length` puts the bar below the last feature (no rollback,
/// returns `null`).
export function rollbackIdForBarIndex(
  featureIds: readonly string[],
  barIndex: number,
): string | null {
  if (barIndex >= featureIds.length) return null;
  if (barIndex <= 0) return featureIds.length > 0 ? featureIds[0]! : null;
  return featureIds[barIndex - 1]!;
}
