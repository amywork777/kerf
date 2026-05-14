# kerf-cad Roadmap — Beyond v1 (100%)

The SW-tier scorecard reads 100%. This document is the honest list of what's still ahead — the items deliberately out-of-scope for v1, plus operational follow-ups from the 60-PR sprint that produced the v1.

## CAD-research items (weeks-to-months each)

### 1. Cylinder × cylinder curved intersections

**Current state:** boolean engine produces planar approximations at cylinder ∩ cylinder seams. The `AnalyticEdge` data model (PR #91) and detection infrastructure (PR #93) can describe the exact intersection, but no producer emits one for this case.

**Why hard:** the seam is a degree-4 algebraic curve (Viviani's curve in the equal-radius case). A general-purpose solver is research-grade work. Naive approaches (point-sampling along one cylinder's surface, projecting to the other's) hit numerical-instability cliffs.

**Concrete next step:** implement the equal-radius special case (Viviani's curve) as a closed-form `AnalyticEdge::Viviani` variant first. That covers the common "perpendicular cylindrical bores" CAD case. General degree-4 is a longer project.

### 2. NURBS–NURBS booleans

**Current state:** no trimmed-NURBS surface support; everything is faceted polyhedra plus the analytic-edge side-table.

**Why hard:** real CAD kernels (Parasolid, ACIS, OpenCascade) handle NURBS via decades of engineering — surface-surface intersection, trimming, fairing. Reimplementing is research-grade.

**Concrete next step:** there is no realistic in-house path. Options: (a) integrate OpenCascade as a `kerf-occ` adapter crate, (b) accept that kerf stays faceted and target the maker/3D-printing market rather than mechanical-engineering.

### 3. B-spline edge emission

**Current state:** `AnalyticEdge::BSpline` variant exists; STEP/GLTF exporters can write it. No boolean operation produces a B-spline edge today.

**Why hard:** B-spline emission only matters when an upstream feature represents itself as a B-spline curve — which today means accepting B-spline input from STEP imports (PR #98 parses CIRCLE/ELLIPSE but not B_SPLINE_CURVE). The infrastructure is in place; we'd need to (a) parse B_SPLINE_CURVE entities in the importer, (b) propagate them through the boolean engine intact.

**Concrete next step:** extend the STEP importer (PR #98 follow-up) to parse `B_SPLINE_CURVE_WITH_KNOTS`. Then the round-trip lights up for B-splines that came from imported files.

## Operational follow-ups from the v1 sprint

### Merge sprint — 46 open PRs

Of the 60 PRs opened during the v1 sprint, **13 merged cleanly** (the score-relevant ones, plus deployment infrastructure). **46 remain open** — none have intrinsic problems; they all hit cascading merge-train conflicts as the trunk advanced.

Each open PR needs:
1. Local rebase onto current `integration/smart-merge`
2. Conflict resolution (mostly `viewer/main.ts`, `crates/kerf-cad/src/feature.rs` overlapping enum-variant adds, package.json/lockfile churn)
3. Re-verify tests
4. Re-push, merge

Realistic budget: 15 min per PR × 46 = ~11 hours. Suitable for a dedicated merge-sprint day, or `/ultrareview` if the toolchain allows it. Some PRs may be safely closed if their work was duplicated by another that landed first.

### Test-infrastructure cleanup

- `every_catalog_example_evaluates` is flaky under disk pressure (timeouts). Raise the per-feature timeout from 5s, or split into fast-tier + slow-tier suites
- `sketch_state.test.ts` runs via a bespoke tsc+node pipeline; the rest run via vitest. Unify under one runner
- WASM size now ~2.8 MB (was 2.69 MB at session start); the STEP-import + analytic-edge work contributed most of the growth. Investigate lazy-loading per-feature crates

### Demo + distribution

- The STOP CRITERIA (untrained user builds a parametric mug, generates a 3-view drawing, saves and reopens) was implemented but not demonstrated end-to-end in this session. Record a 90-second screencast as the canonical demo
- Activate GitHub Pages (Settings → Pages → Source: `gh-pages` branch) so the deploy workflow goes live
- Build the first Tauri binary and ship a `.dmg` / `.exe` / `.AppImage` set in a v0.1.0 GitHub Release

## Stretch — beyond CAD-research

- **WebGPU renderer** — the viewer uses WebGL via Three.js today. WebGPU would unlock larger models (10M+ triangles) at interactive frame rates
- **Mobile support** — iPad with Pencil for sketching, especially as 3D-printing slicers move to tablets
- **API gateway** — a hosted kerf-cad service that accepts model JSON, returns STL/STEP/GLTF; integrates with downstream tools (Vizcom's image-to-3D pipeline, etc.)
- **Collaboration** — multi-user editing of the same Model with conflict-free merge. The functional architecture (Model JSON as the authoritative state) is amenable; the hard work is the diff/merge layer

## What 100% on the scorecard does NOT mean

The scorecard measures *breadth* against SolidWorks's surface area. It does not measure:

- **Performance** — kerf-cad is not optimized for 10k-feature assemblies
- **Robustness** — the KNOWN_EXEMPT list in `catalog_examples.rs` documents real degenerate-input stitch failures
- **Polish** — the viewer's UI is functional but not delightful; expert SW users will notice
- **Ecosystem** — no plugin API, no marketplace, no design-system kit, no professional support contracts

These are product-readiness gaps that score-tracking doesn't capture. The 100% mark is "we have the SW-tier feature breadth"; it is not "we are ready to replace SolidWorks for paying customers." Two different bars.
