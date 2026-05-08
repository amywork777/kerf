import { describe, it, expect } from "vitest";
import {
  categoryFor,
  groupByCategory,
  searchKinds,
  defaultInstance,
} from "../catalog.js";

describe("feature catalog", () => {
  it("categorises booleans and patterns under Patterns & Booleans", () => {
    expect(categoryFor("Union")).toBe("Patterns & Booleans");
    expect(categoryFor("Intersection")).toBe("Patterns & Booleans");
    expect(categoryFor("Difference")).toBe("Patterns & Booleans");
    expect(categoryFor("LinearPattern")).toBe("Patterns & Booleans");
    expect(categoryFor("PolarPattern")).toBe("Patterns & Booleans");
  });

  it("categorises transforms under Transforms", () => {
    expect(categoryFor("Translate")).toBe("Transforms");
    expect(categoryFor("Rotate")).toBe("Transforms");
    expect(categoryFor("Mirror")).toBe("Transforms");
    expect(categoryFor("ScaleXYZ")).toBe("Transforms");
  });

  it("categorises Ref-prefixed kinds + markers under Reference", () => {
    expect(categoryFor("RefPoint")).toBe("Reference");
    expect(categoryFor("RefAxis")).toBe("Reference");
    expect(categoryFor("RefPlane")).toBe("Reference");
    expect(categoryFor("Marker3D")).toBe("Reference");
    expect(categoryFor("AnchorPoint")).toBe("Reference");
  });

  it("categorises beam/channel/bracket shapes under Structural", () => {
    expect(categoryFor("LBracket")).toBe("Structural");
    expect(categoryFor("IBeam")).toBe("Structural");
    expect(categoryFor("CChannel")).toBe("Structural");
    expect(categoryFor("BasePlate")).toBe("Structural");
  });

  it("categorises bolts/screws/washers under Fasteners", () => {
    expect(categoryFor("Bolt")).toBe("Fasteners");
    expect(categoryFor("CapScrew")).toBe("Fasteners");
    expect(categoryFor("Washer")).toBe("Fasteners");
    expect(categoryFor("Rivet")).toBe("Fasteners");
  });

  it("categorises holes/fillets/chamfers under Manufacturing", () => {
    expect(categoryFor("Fillet")).toBe("Manufacturing");
    expect(categoryFor("Chamfer")).toBe("Manufacturing");
    expect(categoryFor("Counterbore")).toBe("Manufacturing");
    expect(categoryFor("HoleArray")).toBe("Manufacturing");
    expect(categoryFor("BoltCircle")).toBe("Manufacturing");
  });

  it("categorises Loft/Revolve/Coil under Sweep & Loft", () => {
    expect(categoryFor("Loft")).toBe("Sweep & Loft");
    expect(categoryFor("Revolve")).toBe("Sweep & Loft");
    expect(categoryFor("Coil")).toBe("Sweep & Loft");
    expect(categoryFor("Spring")).toBe("Sweep & Loft");
  });

  it("falls back to Primitives for unrecognised kinds (the long tail)", () => {
    expect(categoryFor("EggShape")).toBe("Primitives");
    expect(categoryFor("Bottle")).toBe("Primitives");
    expect(categoryFor("Heart")).toBe("Primitives");
    expect(categoryFor("Box")).toBe("Primitives");
  });

  it("groupByCategory partitions a list into categories with sorted entries", () => {
    const grouped = groupByCategory(["Cylinder", "Box", "Union", "Bolt", "Translate"]);
    expect(grouped.get("Primitives")).toEqual(["Box", "Cylinder"]);
    expect(grouped.get("Patterns & Booleans")).toEqual(["Union"]);
    expect(grouped.get("Fasteners")).toEqual(["Bolt"]);
    expect(grouped.get("Transforms")).toEqual(["Translate"]);
  });

  it("searchKinds is case-insensitive substring", () => {
    const all = ["Cylinder", "CylinderAt", "Cone", "Box", "BoxAt"];
    expect(searchKinds(all, "")).toEqual(all);
    expect(searchKinds(all, "cyl")).toEqual(["Cylinder", "CylinderAt"]);
    expect(searchKinds(all, "AT")).toEqual(["CylinderAt", "BoxAt"]);
    expect(searchKinds(all, "zzz")).toEqual([]);
  });

  it("defaultInstance produces a JSON-serialisable skeleton with kind + id", () => {
    const inst = defaultInstance("Cylinder", "c1");
    expect(inst.kind).toBe("Cylinder");
    expect(inst.id).toBe("c1");
    expect(inst.radius).toBe(10);
    expect(inst.height).toBe(20);
    // Round-trip through JSON.
    const round = JSON.parse(JSON.stringify(inst));
    expect(round.kind).toBe("Cylinder");
  });

  it("defaultInstance fills inputs/inputs placeholders for transforms and booleans", () => {
    expect(defaultInstance("Union", "u").inputs).toEqual(["_a", "_b"]);
    expect(defaultInstance("Difference", "d").inputs).toEqual(["_a", "_b"]);
    expect(defaultInstance("Translate", "t").input).toBe("_");
    expect(defaultInstance("Translate", "t").offset).toEqual([0, 0, 10]);
    expect(defaultInstance("Mirror", "m").input).toBe("_");
    expect(defaultInstance("LinearPattern", "lp").count).toBe(3);
  });

  it("defaultInstance handles the long-tail variants without throwing", () => {
    // Don't assert specific defaults — just confirm we get a valid
    // skeleton record that can be stringified for any kind.
    for (const k of ["EggShape", "Heart", "Pawn", "Volute", "Tetrahedron", "ZigzagBar"]) {
      const inst = defaultInstance(k, "x");
      expect(inst.kind).toBe(k);
      expect(inst.id).toBe("x");
      expect(() => JSON.stringify(inst)).not.toThrow();
    }
  });
});
