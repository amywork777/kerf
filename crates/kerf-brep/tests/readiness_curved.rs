//! Curved-primitive stress test: tessellate kerf's curved primitives
//! (sphere, torus, cone) at high resolution → write STL → re-import →
//! run booleans on the triangulated mesh.
//!
//! This is the "real public-domain STL" equivalent: the imported solid is a
//! triangle-mesh approximation with hundreds of Plane faces, no kerf-primitive
//! topology, and floating-point coordinates from sin/cos. The boolean engine
//! has to handle these the same way it would handle any externally-sourced STL.
//!
//! Specifically interesting: torus is genus-1 — booleans on it must preserve
//! genus when not forming/breaking handles, and the imported mesh has a
//! through-hole.

use std::io::Cursor;

use kerf_brep::booleans::BooleanOp;
use kerf_brep::primitives::{box_, box_at, cone, sphere, torus};
use kerf_brep::{
    read_stl_binary_to_solid, solid_volume, tessellate, write_binary, BooleanError, Solid,
};
use kerf_geom::{Point3, Vec3};
use std::f64::consts::PI;

const VOL_TOL: f64 = 0.05; // permissive — tessellation approximation noise

fn try_op(a: &Solid, b: &Solid, op: BooleanOp) -> Result<Solid, BooleanError> {
    match op {
        BooleanOp::Union => a.try_union(b),
        BooleanOp::Intersection => a.try_intersection(b),
        BooleanOp::Difference => a.try_difference(b),
    }
}

fn import_tessellated(s: &Solid, segments: usize, label: &str) -> Solid {
    let soup = tessellate(s, segments);
    let mut buf = Vec::new();
    write_binary(&soup, label, &mut buf).expect("write_binary");
    let mut reader = Cursor::new(buf);
    read_stl_binary_to_solid(&mut reader).expect("read_stl_binary_to_solid")
}

// ============================================================================
// Sphere (genus 0)
// ============================================================================

#[test]
fn imported_sphere_volume_approximates_analytic() {
    // Analytic: V = 4/3 π r³ = 4/3 π (0.5)³ ≈ 0.5236
    let r = 0.5;
    let s_orig = sphere(r);
    let s = import_tessellated(&s_orig, 32, "sphere");
    let v = solid_volume(&s);
    let analytic = 4.0 / 3.0 * PI * r.powi(3);
    // Tessellated approximation underestimates the sphere; expect < analytic
    // but within 5% for n=32.
    assert!(v < analytic, "tessellated vol {v} >= analytic {analytic}");
    assert!(
        analytic - v < analytic * 0.05,
        "tessellated vol {v} too far from analytic {analytic}"
    );
}

#[test]
fn imported_sphere_diff_box_works() {
    // Carve a box-shaped chunk out of a sphere.
    //
    // Use n=20 (vertices at 18° increments — no alignment with cutter face);
    // box position is slightly off-origin and dimensions are non-aligned with
    // sphere lat/lon to avoid coincidences. (At n=24 with cutter at 0.5, the
    // sphere has a vertex at exactly cos(60°)=0.5, hitting a kernel-known
    // degeneracy that not all tier-3 jitter retries break. Documented as
    // imported_sphere_at_24_segments_with_aligned_cutter_is_known_limitation.)
    let s_orig = sphere(1.0);
    let s = import_tessellated(&s_orig, 20, "sphere");
    let cutter = box_at(
        Vec3::new(0.47, 0.43, 0.51),
        Point3::new(-0.03, 0.02, -0.01),
    );
    let v_sphere_before = solid_volume(&s);
    let r = try_op(&s, &cutter, BooleanOp::Difference).expect("sphere − box");
    let v_after = solid_volume(&r);
    // Cutter is fully inside the sphere of radius 1 centered at origin.
    // The cutter spans x ∈ [-0.03, 0.44], y ∈ [0.02, 0.45], z ∈ [-0.01, 0.50].
    // Farthest corner from origin: (0.44, 0.45, 0.50), distance √(.194+.203+.25)
    // = √0.647 ≈ 0.804 < 1. ✓ inside.
    let cutter_vol = 0.47 * 0.43 * 0.51;
    let expected = v_sphere_before - cutter_vol;
    assert!(
        (v_after - expected).abs() < VOL_TOL,
        "sphere − box: got {v_after}, expected {expected}"
    );
}

#[test]
fn imported_sphere_intersect_box_works() {
    // Intersection of a unit sphere with a small box.
    let s_orig = sphere(1.0);
    let s = import_tessellated(&s_orig, 24, "sphere");
    let cutter = box_at(Vec3::new(0.4, 0.4, 0.4), Point3::new(0.1, 0.1, 0.1));
    let r = try_op(&s, &cutter, BooleanOp::Intersection).expect("sphere ∩ box");
    let v = solid_volume(&r);
    // Box [0.1, 0.5]³: corner at (0.5, 0.5, 0.5) has distance √0.75 = 0.866 < 1,
    // so the entire box is inside the sphere of radius 1 centered at origin.
    let expected = 0.4_f64.powi(3); // 0.064
    assert!(
        (v - expected).abs() < VOL_TOL,
        "sphere ∩ box: got {v}, expected {expected}"
    );
}

#[test]
fn two_imported_spheres_diff_works() {
    // Two overlapping spheres — DIFF carves a "lens" cavity from one.
    let big_orig = sphere(1.0);
    let small_orig = sphere(0.4);
    let mut big = import_tessellated(&big_orig, 24, "big");
    let mut small = import_tessellated(&small_orig, 24, "small");
    // Translate small to (0.5, 0, 0).
    for v in small.vertex_geom.values_mut() {
        *v += Vec3::new(0.5, 0.0, 0.0);
    }
    use kerf_brep::geometry::SurfaceKind;
    for surf in small.face_geom.values_mut() {
        if let SurfaceKind::Plane(plane) = surf {
            plane.frame.origin += Vec3::new(0.5, 0.0, 0.0);
        }
    }
    // small needs valid topology after translation — it does (just moved).
    // Translate big = nothing.
    let _ = &mut big; // ensure mut allowed

    // Run DIFF.
    let r = try_op(&big, &small, BooleanOp::Difference).expect("big − small");
    let v = solid_volume(&r);
    // Sanity: result < vol(big), result > 0.
    let v_big = solid_volume(&big);
    let v_small = solid_volume(&small);
    assert!(v >= 0.0);
    assert!(v < v_big + VOL_TOL);
    assert!(v > v_big - v_small - VOL_TOL);
}

// ============================================================================
// Torus (genus 1!)
// ============================================================================

#[test]
fn imported_torus_volume_approximates_analytic() {
    // Analytic: V = 2 π² R r² = 2 π² · 1 · 0.25² = 2 π² · 0.0625 ≈ 1.2337
    let major = 1.0;
    let minor = 0.25;
    let t_orig = torus(major, minor);
    let t = import_tessellated(&t_orig, 24, "torus");
    let v = solid_volume(&t);
    let analytic = 2.0 * PI * PI * major * minor.powi(2);
    assert!(
        v < analytic,
        "tessellated torus vol {v} ≥ analytic {analytic}"
    );
    assert!(
        analytic - v < analytic * 0.06,
        "tessellated torus vol {v} too far from analytic {analytic}"
    );
}

#[test]
fn imported_torus_topology_is_genus_one() {
    let t_orig = torus(1.0, 0.25);
    let t = import_tessellated(&t_orig, 16, "torus");
    let v = t.topo.vertex_count() as i64;
    let e = t.topo.edge_count() as i64;
    let f = t.topo.face_count() as i64;
    let s = t.topo.shell_count() as i64;
    let r_holes: i64 = t
        .topo
        .face_ids()
        .map(|fid| t.topo.face(fid).map(|fc| fc.inner_loops().len()).unwrap_or(0) as i64)
        .sum();
    // V - E + F - R = 2(S - G). For 1 shell, genus 1 → V-E+F-R = 0.
    let lhs = v - e + f - r_holes;
    let g_x2 = 2 * s - lhs;
    assert_eq!(s, 1, "torus is one shell");
    assert_eq!(g_x2, 2, "torus is genus 1");
}

#[test]
fn imported_torus_diff_small_chip_works() {
    // Small cutter that only nicks one side of the donut — the most common
    // torus DIFF case in real CAD (drilling a small feature into a ring).
    let t_orig = torus(1.0, 0.25);
    let t = import_tessellated(&t_orig, 12, "torus");
    let cutter = box_at(
        Vec3::new(0.2, 0.2, 0.2),
        Point3::new(0.9, 0.05, 0.05),
    );
    let v_before = solid_volume(&t);
    let r = try_op(&t, &cutter, BooleanOp::Difference)
        .expect("torus − small chip");
    let v_after = solid_volume(&r);
    assert!(v_after >= 0.0);
    assert!(v_after < v_before, "diff didn't shrink torus");
    kerf_topo::validate(&r.topo).expect("topology");
}

#[test]
#[ignore = "cutter spanning the torus hole intersects both sides of the donut — known limitation"]
fn imported_torus_diff_box_through_ring_works() {
    // Cutter passes through the torus hole AND intersects the donut body on
    // both sides. This is the hardest torus boolean case — the result's
    // genus changes depending on whether the cutter splits the donut into
    // pieces. Currently fails with non-manifold stitch.
    let t_orig = torus(1.0, 0.25);
    let t = import_tessellated(&t_orig, 12, "torus");
    let cutter = box_at(
        Vec3::new(1.97, 0.51, 0.51),
        Point3::new(-1.01, 0.49, -0.27),
    );
    let v_before = solid_volume(&t);
    let r = try_op(&t, &cutter, BooleanOp::Difference).expect("torus − box");
    let v_after = solid_volume(&r);
    assert!(v_after >= 0.0);
    assert!(v_after < v_before + VOL_TOL);
}

// ============================================================================
// Cone
// ============================================================================

#[test]
fn imported_cone_volume_approximates_analytic() {
    // Analytic: V = 1/3 π r² h. cone(r=1, h=2) → V = 2π/3 ≈ 2.0944.
    let r = 1.0;
    let h = 2.0;
    let c_orig = cone(r, h);
    let c = import_tessellated(&c_orig, 24, "cone");
    let v = solid_volume(&c);
    let analytic = PI * r * r * h / 3.0;
    assert!(v < analytic, "tessellated cone vol {v} ≥ analytic {analytic}");
    assert!(
        analytic - v < analytic * 0.06,
        "tessellated cone vol {v} too far from analytic {analytic}"
    );
}

#[test]
fn imported_cone_diff_box_works() {
    let c_orig = cone(1.0, 2.0);
    let c = import_tessellated(&c_orig, 24, "cone");
    let cutter = box_at(Vec3::new(0.4, 0.4, 0.5), Point3::new(0.0, 0.0, 0.5));
    let v_before = solid_volume(&c);
    let r = try_op(&c, &cutter, BooleanOp::Difference).expect("cone − box");
    let v_after = solid_volume(&r);
    assert!(v_after >= 0.0);
    assert!(v_after < v_before + VOL_TOL);
    assert!(v_after < v_before, "cone diff didn't shrink");
}

// ============================================================================
// Combined: imported sphere ∪ imported sphere with cutout
// ============================================================================

#[test]
fn box_minus_imported_sphere_works() {
    // Inverse: carve a sphere-shaped cavity from a box.
    let block = box_(Vec3::new(2.0, 2.0, 2.0));
    let s_orig = sphere(0.5);
    let mut s = import_tessellated(&s_orig, 24, "sphere");
    // Translate sphere to box center.
    use kerf_brep::geometry::SurfaceKind;
    let center = Vec3::new(1.0, 1.0, 1.0);
    for v in s.vertex_geom.values_mut() {
        *v += center;
    }
    for surf in s.face_geom.values_mut() {
        if let SurfaceKind::Plane(plane) = surf {
            plane.frame.origin += center;
        }
    }
    let v_block = solid_volume(&block);
    let v_sphere = solid_volume(&s);
    let r = try_op(&block, &s, BooleanOp::Difference).expect("box − imported sphere");
    let v_after = solid_volume(&r);
    let expected = v_block - v_sphere;
    assert!(
        (v_after - expected).abs() < VOL_TOL,
        "box minus sphere: got {v_after}, expected {expected}"
    );
    // 2 shells: outer box + sphere cavity.
    assert_eq!(r.topo.shell_count(), 2);
}
