/**
 * Unit conversion and formatting helpers for the mass-properties panel.
 *
 * Supported unit systems:
 *   "metric"   — mm / mm² / mm³ / g / kg
 *   "imperial" — in / in² / in³ / lb (oz for small parts)
 */

export type UnitSystem = "metric" | "imperial";
export type ValueKind = "length" | "area" | "volume" | "mass";

// Conversion factors: metric → imperial
const MM_TO_IN   = 0.0393701;         // 1 mm  = 0.0393701 in
const MM2_TO_IN2 = 0.00155;           // 1 mm² = 0.00155   in²
const MM3_TO_IN3 = 6.10237e-5;        // 1 mm³ = 6.10237×10⁻⁵ in³
const G_TO_LB    = 0.0022046;         // 1 g   = 0.0022046 lb
const G_TO_OZ    = 0.035274;          // 1 g   = 0.035274  oz

/** Threshold below which mass is shown in oz rather than lb (imperial only). */
const OZ_THRESHOLD_G = 453.592 / 4;   // ~113 g → 0.25 lb

/**
 * Convert a value from its native metric unit to the target system.
 *
 * Input is always in metric base units:
 *   length → mm, area → mm², volume → mm³, mass → g
 *
 * When `to` is "metric" the value is returned unchanged.
 */
export function convert(
  value: number,
  kind: ValueKind,
  to: UnitSystem,
): number {
  if (to === "metric") return value;

  switch (kind) {
    case "length":  return value * MM_TO_IN;
    case "area":    return value * MM2_TO_IN2;
    case "volume":  return value * MM3_TO_IN3;
    case "mass":    return value * G_TO_LB;
    default: {
      // TypeScript exhaustiveness guard
      const _never: never = kind;
      return value;
    }
  }
}

/**
 * Round `n` to `sigFigs` significant figures.
 * Returns 0 when n === 0 to avoid log10(0) edge-case.
 */
function toSigFigs(n: number, sigFigs: number): number {
  if (n === 0) return 0;
  const d = Math.ceil(Math.log10(Math.abs(n)));
  const power = sigFigs - d;
  const magnitude = Math.pow(10, power);
  return Math.round(n * magnitude) / magnitude;
}

/**
 * Format a metric value with its unit label, converting to the requested
 * unit system and rounding to 3 significant figures.
 *
 * Input is always in metric base units (mm / mm² / mm³ / g).
 * For imperial mass, oz is used when the value is below ~0.25 lb.
 */
export function formatWithUnit(
  value: number,
  kind: ValueKind,
  system: UnitSystem,
): string {
  if (system === "metric") {
    const rounded = toSigFigs(value, 3);
    switch (kind) {
      case "length":  return `${rounded} mm`;
      case "area":    return `${rounded} mm²`;
      case "volume":  return `${rounded} mm³`;
      case "mass":    return `${rounded} g`;
    }
  }

  // Imperial
  switch (kind) {
    case "length": {
      const v = toSigFigs(value * MM_TO_IN, 3);
      return `${v} in`;
    }
    case "area": {
      const v = toSigFigs(value * MM2_TO_IN2, 3);
      return `${v} in²`;
    }
    case "volume": {
      const v = toSigFigs(value * MM3_TO_IN3, 3);
      return `${v} in³`;
    }
    case "mass": {
      if (Math.abs(value) < OZ_THRESHOLD_G) {
        const v = toSigFigs(value * G_TO_OZ, 3);
        return `${v} oz`;
      } else {
        const v = toSigFigs(value * G_TO_LB, 3);
        return `${v} lb`;
      }
    }
  }
}

/** Unit label strings shown in the toggle dropdown. */
export const UNIT_SYSTEM_LABELS: Record<UnitSystem, string> = {
  metric:   "mm / g",
  imperial: "in / lb",
};
