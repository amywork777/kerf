/**
 * Unit tests for mass-units conversion and formatting helpers.
 */
import { describe, it, expect } from "vitest";
import { convert, formatWithUnit } from "./mass-units.js";

// ── convert() ────────────────────────────────────────────────────────────────

describe("convert – metric identity", () => {
  it("leaves length unchanged", () => {
    expect(convert(25.4, "length", "metric")).toBeCloseTo(25.4);
  });
  it("leaves area unchanged", () => {
    expect(convert(645.16, "area", "metric")).toBeCloseTo(645.16);
  });
  it("leaves volume unchanged", () => {
    expect(convert(16387.064, "volume", "metric")).toBeCloseTo(16387.064);
  });
  it("leaves mass unchanged", () => {
    expect(convert(453.592, "mass", "metric")).toBeCloseTo(453.592);
  });
});

describe("convert – mm → in", () => {
  // 1 in = 25.4 mm  →  25.4 mm should convert to 1.000 in
  it("25.4 mm → 1 in", () => {
    expect(convert(25.4, "length", "imperial")).toBeCloseTo(1.0, 5);
  });
  it("0 mm → 0 in", () => {
    expect(convert(0, "length", "imperial")).toBe(0);
  });
});

describe("convert – mm² → in²", () => {
  // 1 in² = 645.16 mm²  →  645.16 mm² should convert to 1.000 in²
  it("645.16 mm² → 1 in²", () => {
    expect(convert(645.16, "area", "imperial")).toBeCloseTo(1.0, 3);
  });
});

describe("convert – mm³ → in³", () => {
  // 1 in³ = 16387.064 mm³  →  16387.064 mm³ should convert to 1.000 in³
  it("16387.064 mm³ → 1 in³", () => {
    expect(convert(16387.064, "volume", "imperial")).toBeCloseTo(1.0, 4);
  });
  it("1 mm³ → 6.10237e-5 in³", () => {
    expect(convert(1, "volume", "imperial")).toBeCloseTo(6.10237e-5, 9);
  });
});

describe("convert – g → lb", () => {
  // 1 lb = 453.592 g  →  453.592 g should convert to 1.000 lb
  it("453.592 g → 1 lb", () => {
    expect(convert(453.592, "mass", "imperial")).toBeCloseTo(1.0, 4);
  });
  it("1000 g → ~2.2046 lb", () => {
    expect(convert(1000, "mass", "imperial")).toBeCloseTo(2.2046, 3);
  });
});

// ── formatWithUnit() ─────────────────────────────────────────────────────────

describe("formatWithUnit – metric", () => {
  it("volume 1234 mm³ → 3 sig figs with unit", () => {
    // 1234 → 1230 (3 sig figs)
    expect(formatWithUnit(1234, "volume", "metric")).toBe("1230 mm³");
  });
  it("volume 0.001234 mm³ → 3 sig figs", () => {
    expect(formatWithUnit(0.001234, "volume", "metric")).toBe("0.00123 mm³");
  });
  it("area 43.21 mm² → 3 sig figs", () => {
    expect(formatWithUnit(43.21, "area", "metric")).toBe("43.2 mm²");
  });
  it("length 12.345 mm → 3 sig figs", () => {
    expect(formatWithUnit(12.345, "length", "metric")).toBe("12.3 mm");
  });
  it("mass 500 g → 500 g", () => {
    expect(formatWithUnit(500, "mass", "metric")).toBe("500 g");
  });
  it("zero volume", () => {
    expect(formatWithUnit(0, "volume", "metric")).toBe("0 mm³");
  });
});

describe("formatWithUnit – imperial", () => {
  it("volume 1234 mm³ → in³ with unit (3 sig figs)", () => {
    // 1234 * 6.10237e-5 = 0.07530... → 3 sig figs = 0.0753
    const result = formatWithUnit(1234, "volume", "imperial");
    expect(result).toMatch(/in³$/);
    // Check numeric value rounded to 3 sig figs
    expect(result).toBe("0.0753 in³");
  });
  it("1 in³ round-trip: 16387.064 mm³ → 1.0 in³", () => {
    const result = formatWithUnit(16387.064, "volume", "imperial");
    expect(result).toBe("1 in³");
  });
  it("length 25.4 mm → 1 in", () => {
    expect(formatWithUnit(25.4, "length", "imperial")).toBe("1 in");
  });
  it("area 645.16 mm² → 1 in²", () => {
    expect(formatWithUnit(645.16, "area", "imperial")).toBe("1 in²");
  });
  it("large mass (>0.25 lb) shown in lb", () => {
    // 453.592 g = 1 lb
    const result = formatWithUnit(453.592, "mass", "imperial");
    expect(result).toBe("1 lb");
  });
  it("small mass (<0.25 lb) shown in oz", () => {
    // 28.3495 g = 1 oz
    const result = formatWithUnit(28.3495, "mass", "imperial");
    expect(result).toMatch(/oz$/);
    expect(result).toBe("1 oz");
  });
  it("zero length", () => {
    expect(formatWithUnit(0, "length", "imperial")).toBe("0 in");
  });
});
