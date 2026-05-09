import { describe, it, expect } from "vitest";
import { applyFilter, collectFeatureKinds, type ParsedFeature } from "./pick-filter";

// ---------------------------------------------------------------------------
// Fixtures
// ---------------------------------------------------------------------------

const features: ParsedFeature[] = [
  { id: "box1", kind: "Box" },
  { id: "cyl1", kind: "Cylinder" },
  { id: "fillet1", kind: "Fillet" },
];

// faceId → owner feature id
const ownerTags = new Map<number, string>([
  [0, "box1"],
  [1, "box1"],
  [2, "cyl1"],
  [3, "fillet1"],
  // face 4 intentionally has no entry (unknown owner)
]);

// ---------------------------------------------------------------------------
// applyFilter — "All" mode
// ---------------------------------------------------------------------------

describe('applyFilter with filterKind "All"', () => {
  it("always returns true for a face with a known owner", () => {
    expect(applyFilter(0, ownerTags, features, "All")).toBe(true);
  });

  it("always returns true for a face with no owner tag", () => {
    expect(applyFilter(4, ownerTags, features, "All")).toBe(true);
  });

  it("always returns true regardless of face id", () => {
    for (let fid = 0; fid < 5; fid++) {
      expect(applyFilter(fid, ownerTags, features, "All")).toBe(true);
    }
  });
});

// ---------------------------------------------------------------------------
// applyFilter — "Box" mode
// ---------------------------------------------------------------------------

describe('applyFilter with filterKind "Box"', () => {
  it("returns true for faces owned by a Box feature", () => {
    expect(applyFilter(0, ownerTags, features, "Box")).toBe(true);
    expect(applyFilter(1, ownerTags, features, "Box")).toBe(true);
  });

  it("returns false for a face owned by a Cylinder feature", () => {
    expect(applyFilter(2, ownerTags, features, "Box")).toBe(false);
  });

  it("returns false for a face owned by a Fillet feature", () => {
    expect(applyFilter(3, ownerTags, features, "Box")).toBe(false);
  });

  it("returns true (graceful) for a face with no owner tag", () => {
    expect(applyFilter(4, ownerTags, features, "Box")).toBe(true);
  });
});

// ---------------------------------------------------------------------------
// applyFilter — faces with no owner tag
// ---------------------------------------------------------------------------

describe("applyFilter graceful handling of missing owner tag", () => {
  it("returns true when the face has no owner regardless of filter", () => {
    expect(applyFilter(99, ownerTags, features, "Cylinder")).toBe(true);
    expect(applyFilter(99, ownerTags, features, "Fillet")).toBe(true);
  });

  it("returns true when the owner id has no matching feature in the list", () => {
    const orphanTags = new Map<number, string>([[0, "orphan_id"]]);
    expect(applyFilter(0, orphanTags, features, "Box")).toBe(true);
  });
});

// ---------------------------------------------------------------------------
// applyFilter — feature kind from JSON
// ---------------------------------------------------------------------------

describe("applyFilter respects feature kind from model JSON", () => {
  const modelFeatures: ParsedFeature[] = [
    { id: "base", kind: "Box" },
    { id: "hole", kind: "Cylinder" },
    { id: "round", kind: "Fillet" },
    { id: "ext", kind: "Extrude" },
  ];
  const tags = new Map<number, string>([
    [0, "base"],
    [1, "hole"],
    [2, "round"],
    [3, "ext"],
  ]);

  it("matches Cylinder kind correctly", () => {
    expect(applyFilter(1, tags, modelFeatures, "Cylinder")).toBe(true);
    expect(applyFilter(0, tags, modelFeatures, "Cylinder")).toBe(false);
  });

  it("matches Fillet kind correctly", () => {
    expect(applyFilter(2, tags, modelFeatures, "Fillet")).toBe(true);
    expect(applyFilter(3, tags, modelFeatures, "Fillet")).toBe(false);
  });

  it("matches Extrude kind correctly", () => {
    expect(applyFilter(3, tags, modelFeatures, "Extrude")).toBe(true);
    expect(applyFilter(0, tags, modelFeatures, "Extrude")).toBe(false);
  });
});

// ---------------------------------------------------------------------------
// collectFeatureKinds
// ---------------------------------------------------------------------------

describe("collectFeatureKinds", () => {
  it("returns sorted, deduplicated kinds", () => {
    const fs: ParsedFeature[] = [
      { id: "a", kind: "Box" },
      { id: "b", kind: "Fillet" },
      { id: "c", kind: "Box" },
      { id: "d", kind: "Cylinder" },
    ];
    expect(collectFeatureKinds(fs)).toEqual(["Box", "Cylinder", "Fillet"]);
  });

  it("excludes '?' placeholder kinds", () => {
    const fs: ParsedFeature[] = [
      { id: "a", kind: "?" },
      { id: "b", kind: "Box" },
    ];
    expect(collectFeatureKinds(fs)).toEqual(["Box"]);
  });

  it("returns empty array for empty feature list", () => {
    expect(collectFeatureKinds([])).toEqual([]);
  });
});
