/**
 * Feature catalog: takes the flat `feature_kinds()` list from the WASM
 * crate and (a) puts each kind in a human-friendly category and
 * (b) produces a default-parameter JSON skeleton for that kind that the
 * viewer can splice into a model.
 *
 * Categories are pattern-matched on the variant name. We err on the
 * side of "primitives" as the catch-all — most catalog entries are
 * shape primitives. Tests in `__tests__/catalog.test.ts` lock in the
 * key category buckets.
 */

export type Category =
  | "Primitives"
  | "Manufacturing"
  | "Structural"
  | "Fasteners"
  | "Joinery"
  | "Reference"
  | "Sweep & Loft"
  | "Patterns & Booleans"
  | "Transforms"
  | "Other";

export interface CatalogEntry {
  kind: string;
  category: Category;
}

/**
 * Categorise a Feature variant name. Order of checks matters — we go
 * most-specific to least-specific so e.g. "BoltCircle" lands in
 * Manufacturing (it's a hole pattern), not Fasteners.
 */
export function categoryFor(kind: string): Category {
  // Booleans and patterns first — small, well-defined set.
  if (kind === "Union" || kind === "Intersection" || kind === "Difference") {
    return "Patterns & Booleans";
  }
  if (kind === "LinearPattern" || kind === "PolarPattern") {
    return "Patterns & Booleans";
  }
  // Transforms.
  if (
    kind === "Translate" ||
    kind === "Scale" ||
    kind === "ScaleXYZ" ||
    kind === "Rotate" ||
    kind === "Mirror"
  ) {
    return "Transforms";
  }
  // Reference geometry — explicit names mostly start with Ref or end
  // in Point/Axis/Plane/Marker/Anchor.
  if (
    kind.startsWith("Ref") ||
    kind === "VectorArrow" ||
    kind === "Arrow" ||
    kind === "Marker3D" ||
    kind === "AnchorPoint" ||
    kind === "BoundingBoxRef" ||
    kind === "CentroidPoint" ||
    kind === "DistanceRod" ||
    kind === "AngleArc"
  ) {
    return "Reference";
  }
  // Sweep / loft / extrude family.
  if (
    kind === "Loft" ||
    kind === "TaperedExtrude" ||
    kind === "ExtrudePolygon" ||
    kind === "Revolve" ||
    kind === "PipeRun" ||
    kind === "SweepPath" ||
    kind === "Coil" ||
    kind === "Spring"
  ) {
    return "Sweep & Loft";
  }
  // Manufacturing operations (holes, fillets, chamfers, cuts).
  if (
    kind === "Fillet" ||
    kind === "Fillets" ||
    kind === "Chamfer" ||
    kind === "CornerCut" ||
    kind === "Counterbore" ||
    kind === "Countersink" ||
    kind === "HoleArray" ||
    kind === "BoltCircle" ||
    kind === "HexHole" ||
    kind === "SquareHole" ||
    kind === "Slot" ||
    kind === "FilletedSlot" ||
    kind === "Slot3D" ||
    kind === "Mortise" ||
    kind === "Tenon" ||
    kind === "FingerJoint" ||
    kind === "DovetailRail" ||
    kind === "DovetailSlot" ||
    kind === "VeeGroove" ||
    kind === "Keyway" ||
    kind === "TSlot" ||
    kind === "DrawerSlot" ||
    kind === "Knurled".concat("Grip")
  ) {
    return "Manufacturing";
  }
  // Joinery is a sub-bucket of manufacturing in some catalogs but the
  // user spec calls it out separately.
  if (
    kind === "ChainLink" ||
    kind === "BeltLoop" ||
    kind === "Hinge" ||
    kind === "Cleat"
  ) {
    return "Joinery";
  }
  // Structural shapes — beams, channels, plates, brackets.
  if (
    kind === "LBracket" ||
    kind === "UChannel" ||
    kind === "TBeam" ||
    kind === "IBeam" ||
    kind === "CChannel" ||
    kind === "ZBeam" ||
    kind === "AngleIron" ||
    kind === "TrussMember" ||
    kind === "Lattice" ||
    kind === "GussetPlate" ||
    kind === "ShelfBracket" ||
    kind === "CornerBracket" ||
    kind === "AsymmetricBracket" ||
    kind === "MountingFlange" ||
    kind === "BasePlate" ||
    kind === "PerforatedPlate" ||
    kind === "ChamferedPlate" ||
    kind === "RibbedPlate" ||
    kind === "TriangularPlate" ||
    kind === "OvalPlate" ||
    kind === "BeamWithHoles" ||
    kind === "Column" ||
    kind === "Stair" ||
    kind === "ParapetWall" ||
    kind === "Plinth" ||
    kind === "Pediment" ||
    kind === "Vault" ||
    kind === "ArchedDoorway" ||
    kind === "WindowFrame" ||
    kind === "Pipe" ||
    kind === "PipeRun" ||
    kind === "Stand" ||
    kind === "Stake" ||
    kind === "Brick" ||
    kind === "HollowBrick" ||
    kind === "CorrugatedPanel"
  ) {
    return "Structural";
  }
  // Fasteners — bolts, screws, nuts, washers, rivets.
  if (
    kind === "Bolt" ||
    kind === "CapBolt" ||
    kind === "FlangedBolt" ||
    kind === "EyeBolt" ||
    kind === "ShoulderBolt" ||
    kind === "CapScrew" ||
    kind === "SocketHeadCapScrew" ||
    kind === "FlatHeadScrew" ||
    kind === "WingedScrew" ||
    kind === "Nut" ||
    kind === "Washer" ||
    kind === "FlatWasher" ||
    kind === "Rivet" ||
    kind === "ThreadInsert" ||
    kind === "RoundBoss" ||
    kind === "RectBoss" ||
    kind === "ScrewBoss"
  ) {
    return "Fasteners";
  }
  // Default: it's a primitive shape (the long tail of the catalog).
  return "Primitives";
}

/**
 * Group a list of feature kinds by category, sorting kinds within each
 * group alphabetically. Useful for rendering the catalog as a sidebar
 * tree with named sections.
 */
export function groupByCategory(kinds: string[]): Map<Category, string[]> {
  const out = new Map<Category, string[]>();
  for (const k of kinds) {
    const c = categoryFor(k);
    let bucket = out.get(c);
    if (!bucket) {
      bucket = [];
      out.set(c, bucket);
    }
    bucket.push(k);
  }
  for (const v of out.values()) v.sort();
  return out;
}

/**
 * Filter kinds by a search query (case-insensitive substring match).
 * Empty query returns all entries unchanged.
 */
export function searchKinds(kinds: string[], query: string): string[] {
  const q = query.trim().toLowerCase();
  if (q === "") return kinds.slice();
  return kinds.filter((k) => k.toLowerCase().includes(q));
}

/**
 * A default-parameter JSON skeleton for a given feature kind. Returns
 * a record that can be JSON-stringified and inserted into the
 * `features` array of a kerf-cad model. The skeleton uses safe,
 * boring defaults (units: mm-ish; radii: 5; lengths: 20) that almost
 * always produce a valid solid for primitive kinds; for transforms
 * and booleans, the user must edit `input` / `inputs` after insertion
 * (we use `"_"` as a placeholder so the user notices).
 *
 * `id` is the suggested id for the new feature — the caller should
 * make it unique within the model before splicing it in.
 */
export function defaultInstance(kind: string, id: string): Record<string, unknown> {
  const base: Record<string, unknown> = { kind, id };
  // Transforms and booleans need explicit input(s) — emit a
  // placeholder so the user has somewhere to type.
  if (kind === "Translate" || kind === "Scale" || kind === "Rotate" || kind === "Mirror") {
    base.input = "_";
  }
  if (kind === "ScaleXYZ") {
    base.input = "_";
    base.factor = [1, 1, 1];
  }
  if (kind === "Translate") {
    base.offset = [0, 0, 10];
  }
  if (kind === "Scale") {
    base.factor = 1;
  }
  if (kind === "Rotate") {
    base.axis = "z";
    base.angle_deg = 90;
  }
  if (kind === "Mirror") {
    base.axis = "z";
  }
  if (kind === "Union" || kind === "Intersection" || kind === "Difference") {
    base.inputs = ["_a", "_b"];
  }
  if (kind === "LinearPattern") {
    base.input = "_";
    base.offset = [10, 0, 0];
    base.count = 3;
  }
  if (kind === "PolarPattern") {
    base.input = "_";
    base.axis = "z";
    base.count = 4;
  }
  // Common primitives — give plausible defaults so the user gets a
  // real shape on click. The kernel is forgiving about extra keys, so
  // we err on the side of "more keys, more chances of evaluating".
  if (kind === "Box" || kind === "HollowBox") {
    base.extents = [20, 20, 20];
  }
  if (kind === "BoxAt") {
    base.extents = [20, 20, 20];
    base.origin = [0, 0, 0];
  }
  if (kind === "Cylinder" || kind === "HollowCylinder") {
    base.radius = 10;
    base.height = 20;
    base.segments = 24;
  }
  if (kind === "CylinderAt" || kind === "CylinderShellAt") {
    base.radius = 10;
    base.height = 20;
    base.segments = 24;
    base.origin = [0, 0, 0];
    base.axis = "z";
  }
  if (kind === "Sphere" || kind === "SphereFaceted" || kind === "HollowSphere") {
    base.radius = 10;
  }
  if (kind === "Cone" || kind === "HollowCone") {
    base.radius = 10;
    base.height = 20;
  }
  if (kind === "Frustum" || kind === "ReducerCone") {
    base.top_radius = 5;
    base.bottom_radius = 10;
    base.height = 20;
  }
  if (kind === "Torus" || kind === "Donut") {
    base.major_radius = 15;
    base.minor_radius = 4;
    if (kind === "Donut") {
      base.major_segs = 16;
      base.minor_segs = 8;
    }
  }
  if (kind === "Tube" || kind === "TubeAt" || kind === "SquareTube") {
    base.outer_radius = 10;
    base.inner_radius = 6;
    base.height = 20;
    base.segments = 24;
  }
  if (kind === "Wedge") {
    base.extents = [20, 20, 20];
  }
  if (kind === "RegularPrism" || kind === "Pyramid" || kind === "TruncatedPyramid") {
    base.sides = 6;
    base.radius = 10;
    base.height = 20;
  }
  if (kind === "Capsule" || kind === "CapsuleAt") {
    base.radius = 5;
    base.length = 20;
  }
  if (kind === "Slot" || kind === "Slot3D" || kind === "FilletedSlot") {
    base.length = 20;
    base.width = 8;
    base.height = 5;
  }
  if (kind === "Fillet" || kind === "Fillets") {
    base.input = "_";
    base.radius = 2;
  }
  if (kind === "Chamfer") {
    base.input = "_";
    base.distance = 2;
  }
  return base;
}
