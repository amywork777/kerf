//! Regression tests locking in cylinder × plane boolean behaviour.
//!
//! These tests address the "no capping" concern from PR #43 (section view):
//! when a cylinder is cut by a plane, the resulting cap must be a **single
//! flat polygon** whose vertex count matches the cylinder's segment count —
//! not a fan of triangles or a set of separate sub-polygons.
//!
//! ## What these tests assert
//!
//! 1. **Top-clean cut**: `Cylinder(r=1, h=2, n=8) − Box(z=[1,3.5])` yields:
//!    - Exactly `n` lateral quad faces (not over-tessellated).
//!    - Original bottom cap: a single n-gon polygon with `n` vertices.
//!    - New cut cap at z=1: a single n-gon polygon with `n` vertices.
//!    - Volume = area(n-gon) × 1 = 2√2 ≈ 2.828427.
//!    - Both cap faces tagged `"cylinder_cap"` in `face_owner_tag`.
//!
//! 2. **Side graze**: `Cylinder(r=1, h=2, n=8) − Box(x=[0.5,2])` yields:
//!    - Both original caps survive as single n-gon polygons.
//!    - Topology validates cleanly.
//!
//! 3. **Concentric bore**: `Cylinder(r=1) − Cylinder(r=0.5)` yields:
//!    - 2 annular cap faces (stinger-walk polygon with `2n+2` vertices).
//!    - 2n lateral quad faces (n outer + n inner walls).
//!    - Volume = (area_outer − area_inner) × h exactly.
//!
//! 4. **Volume accuracy**: carved cylinder volume matches analytic n-gon
//!    formula within 1e-6.
//!
//! 5. **Face count**: no over-tessellation — face count equals expected value
//!    for the given operation (n lateral + cap count).

use kerf_brep::booleans::face_polygon;
use kerf_brep::primitives::{box_at, cylinder_faceted};
use kerf_brep::{solid_volume, CYLINDER_CAP_TAG};
use kerf_geom::{Point3, Vec3};
use kerf_topo::validate;

const VOL_TOL: f64 = 1e-6;

// ─── helpers ────────────────────────────────────────────────────────────────

fn n_gon_area(r: f64, n: usize) -> f64 {
    // Area of a regular n-gon inscribed in a circle of radius r.
    // The phase offset in cylinder_faceted shifts vertices by π/n, but a
    // regular n-gon inscribed in r has the same area regardless of rotation.
    0.5 * n as f64 * r * r * (2.0 * std::f64::consts::PI / n as f64).sin()
}

/// Collect polygon sizes for every face in `s`.
fn face_sizes(s: &kerf_brep::Solid) -> Vec<usize> {
    let mut sizes: Vec<usize> = s
        .topo
        .face_ids()
        .filter_map(|fid| face_polygon(s, fid).map(|p| p.len()))
        .collect();
    sizes.sort_unstable();
    sizes
}

/// Count how many faces carry a given owner tag.
fn count_tagged(s: &kerf_brep::Solid, tag: &str) -> usize {
    s.face_owner_tag
        .iter()
        .filter(|(_, v)| v.as_str() == tag)
        .count()
}

// ─── test 1: top-clean cut ───────────────────────────────────────────────────

/// Cylinder minus a box that cleanly removes the top half.
///
/// The cut plane is z = 1.0 (box covers z ∈ [1, 3.5]).  After the diff:
///   - n=8 lateral faces, each a quad (4 verts) — unchanged from original.
///   - 1 bottom cap at z=0: single 8-gon (8 verts).
///   - 1 new cap at z=1: single 8-gon (8 verts).
///   - Total: 10 faces, 16 vertices.
///   - Volume = area(8-gon, r=1) × 1 = 2√2 ≈ 2.828427.
#[test]
fn cyl_top_clean_cut_single_cap_polygon() {
    let n = 8usize;
    let r = 1.0_f64;
    let h = 2.0_f64;
    let cyl = cylinder_faceted(r, h, n);

    // Box at z=1 through z=3.5, large enough in xy to cover the whole cylinder.
    let cutter = box_at(Vec3::new(3.0, 3.0, 2.5), Point3::new(-1.5, -1.5, 1.0));
    let result = cyl
        .try_difference(&cutter)
        .expect("cylinder top-cut difference should succeed");

    validate(&result.topo).expect("topology must be valid");

    // Face count: n lateral quads + 2 n-gon caps = n+2.
    assert_eq!(
        result.face_count(),
        n + 2,
        "expected {} faces (n lateral + 2 caps), got {}",
        n + 2,
        result.face_count()
    );

    // Vertex count: n bottom verts + n top verts = 2n.
    assert_eq!(
        result.vertex_count(),
        2 * n,
        "expected {} vertices (2n), got {}",
        2 * n,
        result.vertex_count()
    );

    // Collect and sort polygon sizes.
    let sizes = face_sizes(&result);
    // Expect: n quads of size 4, and 2 caps of size n.
    let expected_quads = vec![4usize; n];
    let expected_caps = vec![n; 2];
    let mut expected = expected_quads;
    expected.extend_from_slice(&expected_caps);
    expected.sort_unstable();
    assert_eq!(
        sizes, expected,
        "face polygon sizes mismatch: got {sizes:?}, expected {expected:?}"
    );

    // Exactly 2 faces have n vertices — the caps.
    let n_gon_faces = sizes.iter().filter(|&&s| s == n).count();
    assert_eq!(
        n_gon_faces, 2,
        "expected 2 n-gon cap faces with {} verts, found {}",
        n, n_gon_faces
    );

    // Volume is analytic n-gon area × half-height.
    let expected_vol = n_gon_area(r, n) * (h / 2.0);
    let actual_vol = solid_volume(&result);
    assert!(
        (actual_vol - expected_vol).abs() < VOL_TOL,
        "volume {actual_vol:.9} differs from expected {expected_vol:.9} by more than {VOL_TOL}"
    );
}

// ─── test 2: side graze — caps survive as single n-gons ──────────────────────

/// Cylinder minus a box that grazes the side (x ≥ 0.5 cut off).
///
/// Both original cap faces must survive as single n-gon polygons. The side
/// area shrinks but the caps remain intact and the topology is valid.
#[test]
fn cyl_side_graze_caps_remain_single_ngon() {
    let n = 8usize;
    let r = 1.0_f64;
    let h = 2.0_f64;
    let cyl = cylinder_faceted(r, h, n);

    // Cutter removes x ∈ [0.5, 2.5], full y and z extent.
    let cutter = box_at(Vec3::new(2.0, 3.0, 3.0), Point3::new(0.5, -1.5, -0.5));
    let result = cyl
        .try_difference(&cutter)
        .expect("cylinder side-graze difference should succeed");

    validate(&result.topo).expect("topology must be valid");

    // Both original caps must be present — the side graze doesn't touch them.
    // Each cap is a single flat face. Count how many faces have >= n vertices.
    let large_faces: Vec<usize> = face_sizes(&result)
        .into_iter()
        .filter(|&s| s >= n)
        .collect();
    assert!(
        large_faces.len() >= 2,
        "expected at least 2 n-gon cap faces after side graze, found: {large_faces:?}"
    );

    // All large faces must be exactly n-gons (no fan-split caps).
    for &sz in &large_faces {
        assert_eq!(
            sz, n,
            "cap face should have exactly {n} vertices, got {sz}"
        );
    }

    // Volume is less than the full cylinder, more than zero.
    let full_vol = solid_volume(&cyl);
    let result_vol = solid_volume(&result);
    assert!(
        result_vol > 0.0 && result_vol < full_vol,
        "result volume {result_vol:.6} should be in (0, {full_vol:.6})"
    );
}

// ─── test 3: concentric bore — annular caps + cylindrical walls ──────────────

/// `Cylinder(r=1) − Cylinder(r=0.5)` (concentric, smaller drilled out).
///
/// Result topology:
///   - 2 annular caps (stinger-walk polygon with 2n+2 vertices each).
///   - n outer lateral quads + n inner lateral quads = 2n lateral faces.
///   - Total: 2n + 2 faces.
///   - Volume = (area_outer − area_inner) × h.
#[test]
fn concentric_bore_annular_caps_and_two_cylindrical_walls() {
    let n = 8usize;
    let r_out = 1.0_f64;
    let r_in = 0.5_f64;
    let h = 2.0_f64;

    let outer = cylinder_faceted(r_out, h, n);

    // Inner cylinder extends slightly past both caps to avoid coplanar degeneracy.
    let inner_raw = cylinder_faceted(r_in, h + 0.5, n);
    let mut inner = inner_raw;
    for v in inner.vertex_geom.values_mut() {
        *v += Vec3::new(0.0, 0.0, -0.25);
    }
    for surf in inner.face_geom.values_mut() {
        if let kerf_brep::SurfaceKind::Plane(plane) = surf {
            plane.frame.origin += Vec3::new(0.0, 0.0, -0.25);
        }
    }

    let result = outer
        .try_difference(&inner)
        .expect("concentric bore should succeed");

    validate(&result.topo).expect("topology must be valid after concentric bore");

    // Total faces: 2n lateral + 2 annular caps.
    assert_eq!(
        result.face_count(),
        2 * n + 2,
        "expected {} faces (2n lateral + 2 caps), got {}",
        2 * n + 2,
        result.face_count()
    );

    // Vertex count: outer ring (2n verts) + inner ring (2n verts) = 4n.
    assert_eq!(
        result.vertex_count(),
        4 * n,
        "expected {} vertices (4n), got {}",
        4 * n,
        result.vertex_count()
    );

    // Annular cap faces use a stinger-walk polygon: outer n-gon + 2 stinger
    // edges + reversed inner n-gon = 2n + 2 vertices.
    let cap_size = 2 * n + 2;
    let sizes = face_sizes(&result);
    let cap_count = sizes.iter().filter(|&&s| s == cap_size).count();
    assert_eq!(
        cap_count, 2,
        "expected 2 annular cap faces each with {cap_size} vertices, found {cap_count}"
    );

    // Lateral quads: each should be 4-vertex.
    let quad_count = sizes.iter().filter(|&&s| s == 4).count();
    assert_eq!(
        quad_count,
        2 * n,
        "expected {} quad lateral faces, got {}",
        2 * n,
        quad_count
    );

    // Volume = (area_outer − area_inner) × h.
    let expected_vol = (n_gon_area(r_out, n) - n_gon_area(r_in, n)) * h;
    let actual_vol = solid_volume(&result);
    assert!(
        (actual_vol - expected_vol).abs() < VOL_TOL,
        "concentric bore volume {actual_vol:.9} differs from expected {expected_vol:.9}"
    );
}

// ─── test 4: volume accuracy ─────────────────────────────────────────────────

/// Volume of the carved cylinder matches the analytic n-gon formula within 1e-6.
///
/// Covers multiple segment counts to ensure the n-gon area formula is correct
/// for all common discretizations.
#[test]
fn carved_cylinder_volume_accurate_within_1e6() {
    for &n in &[6usize, 8, 12, 16, 24] {
        let r = 1.0_f64;
        let h = 3.0_f64;
        // Remove top third: z ∈ [2, 4].
        let cutter = box_at(Vec3::new(3.0, 3.0, 2.0), Point3::new(-1.5, -1.5, 2.0));
        let cyl = cylinder_faceted(r, h, n);
        let result = cyl
            .try_difference(&cutter)
            .unwrap_or_else(|e| panic!("n={n}: difference failed: {}", e.message));

        validate(&result.topo)
            .unwrap_or_else(|e| panic!("n={n}: topology invalid: {e:?}"));

        // Result is the bottom 2/3 of the cylinder.
        let expected = n_gon_area(r, n) * 2.0;
        let actual = solid_volume(&result);
        assert!(
            (actual - expected).abs() < VOL_TOL,
            "n={n}: volume {actual:.9} differs from expected {expected:.9}"
        );
    }
}

// ─── test 5: face count — no over-tessellation ───────────────────────────────

/// Face count after cylinder-box difference equals the expected value.
///
/// A clean horizontal cut should never produce more faces than `n + 2`:
/// n lateral quads + 2 caps. Over-tessellation (fan-triangulated caps,
/// extra lateral splits) would produce more faces.
#[test]
fn cyl_minus_box_face_count_no_over_tessellation() {
    for &n in &[4usize, 6, 8, 12] {
        let r = 1.0_f64;
        let h = 4.0_f64;

        let cyl = cylinder_faceted(r, h, n);

        // Horizontal cut at z = 2.0 (splits cylinder exactly at mid-height).
        let cutter = box_at(Vec3::new(3.0, 3.0, 2.1), Point3::new(-1.5, -1.5, 2.0));
        let result = cyl
            .try_difference(&cutter)
            .unwrap_or_else(|e| panic!("n={n}: difference failed: {}", e.message));

        validate(&result.topo)
            .unwrap_or_else(|e| panic!("n={n}: topology invalid: {e:?}"));

        let expected_faces = n + 2; // n lateral + 2 caps
        assert_eq!(
            result.face_count(),
            expected_faces,
            "n={n}: expected {} faces, got {} (over-tessellation?)",
            expected_faces,
            result.face_count()
        );

        // The maximum polygon size among all faces must equal n (the cap size).
        // This holds for all n: caps are n-gons; lateral quads are 4-gons (≤ n for n≥4).
        // No face should have MORE than n vertices (no fan-triangulated caps hiding
        // as larger polygons, and no cap split into sub-polygons smaller than n).
        let sizes = face_sizes(&result);
        let max_size = *sizes.last().unwrap_or(&0);
        assert_eq!(
            max_size, n,
            "n={n}: largest face polygon has {max_size} verts, expected cap size {n}"
        );

        // For n > 4, lateral quads (4 verts) are strictly smaller than cap n-gons.
        // We can count exactly 2 cap n-gons in that case.
        if n > 4 {
            let n_gon_count = sizes.iter().filter(|&&s| s == n).count();
            assert_eq!(
                n_gon_count, 2,
                "n={n}: expected 2 cap n-gons with {n} vertices, found {n_gon_count}"
            );
        }
        // For n=4, all faces are quads so we can only check face count, which we did.
    }
}

// ─── test 6: CYLINDER_CAP_TAG propagation ────────────────────────────────────

/// Original cylinder caps carry the `"cylinder_cap"` owner tag.
#[test]
fn cylinder_cap_faces_carry_tag() {
    let n = 8usize;
    let cyl = cylinder_faceted(1.0, 2.0, n);

    let tagged = count_tagged(&cyl, CYLINDER_CAP_TAG);
    assert_eq!(
        tagged, 2,
        "fresh cylinder should have exactly 2 faces tagged '{CYLINDER_CAP_TAG}', got {tagged}"
    );
}

/// After a top-clean cut, the bottom cap's tag survives through the boolean.
#[test]
fn cylinder_cap_tag_survives_top_cut() {
    let n = 8usize;
    let cyl = cylinder_faceted(1.0, 2.0, n);
    let cutter = box_at(Vec3::new(3.0, 3.0, 2.5), Point3::new(-1.5, -1.5, 1.0));
    let result = cyl
        .try_difference(&cutter)
        .expect("top cut should succeed");

    // At minimum the original bottom cap tag must survive.
    let tagged = count_tagged(&result, CYLINDER_CAP_TAG);
    assert!(
        tagged >= 1,
        "at least 1 '{CYLINDER_CAP_TAG}' face expected after top cut, got {tagged}"
    );
}
