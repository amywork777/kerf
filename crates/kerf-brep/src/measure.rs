//! Geometric measurement: signed volume, per-shell volume, area.
//!
//! These are *correctness checks* — the Solid's faces must walk CCW from
//! their outward normals (M40 winding normalization), and the kernel must
//! produce closed manifolds. Volume of an open mesh is meaningless.

use kerf_geom::Point3;
use kerf_topo::{FaceId, ShellId};

use crate::booleans::FaceSoup;
use crate::geometry::SurfaceKind;
use crate::Solid;

/// Signed volume of a closed manifold via the divergence theorem on
/// fan-triangulated faces. Each triangle contributes (1/6) * (p0 · p1 × p2)
/// where p0/p1/p2 are world-space vertex positions. Positive when faces walk
/// CCW from their OUTWARD normals (the kerf convention, enforced by
/// face_polygon's M40 winding normalization).
pub fn solid_volume(s: &Solid) -> f64 {
    let mut total = 0.0;
    for face_id in s.topo.face_ids() {
        total += face_signed_volume(s, face_id);
    }
    total
}

/// Volume of a single connected shell. A valid Solid may contain multiple
/// disjoint shells (e.g., union of disjoint inputs); each must enclose a
/// positive-signed volume on its own.
pub fn shell_volume(s: &Solid, shell: ShellId) -> f64 {
    let Some(sh) = s.topo.shell(shell) else {
        return 0.0;
    };
    let mut total = 0.0;
    for &face_id in sh.faces() {
        total += face_signed_volume(s, face_id);
    }
    total
}

fn face_signed_volume(s: &Solid, face_id: FaceId) -> f64 {
    // Walk the face's outer loop DIRECTLY (no winding normalization). The
    // loop order encodes the result-face's outward orientation: stitch
    // preserves input polygon order, and input polygons are normalized to
    // CCW-from-result-outward (M40 face_polygon winding fix for stitch input,
    // plus the kept-face flip in pipeline.rs for DIFF B-faces). For
    // solid_volume on a stitched result, reading the loop directly gives
    // CCW-from-result-outward — what the divergence-theorem formula needs.
    //
    // (Going through face_polygon would re-normalize against the original
    // surface frame.z, which for DIFF-flipped faces points the wrong way and
    // produces a volume with the wrong sign.)
    //
    // Bug fix (e2e scenario_06 vase): revolved analytic faces (Cone, Cylinder,
    // Sphere, Torus) have degenerate loop polygons — a full lateral face's
    // outer loop is `seam_a → circle_a → seam_b → circle_b`, where the two
    // seams traverse the same straight segment in opposite directions, and
    // the two circles are self-loops on a single profile vertex. The polygon
    // walk collapses to a degenerate quad like `[v_top, v_bot, v_bot, v_top]`
    // whose divergence-theorem integrand is zero. For analytic surfaces we
    // fall back to tessellated triangles (which carry the actual swept
    // geometry of the surface, not just its loop endpoints).
    let Some(face) = s.topo.face(face_id) else {
        return 0.0;
    };
    let surface = s.face_geom.get(face_id);
    let is_analytic = matches!(
        surface,
        Some(SurfaceKind::Cone(_))
            | Some(SurfaceKind::Cylinder(_))
            | Some(SurfaceKind::Sphere(_))
            | Some(SurfaceKind::Torus(_))
    );

    if is_analytic {
        // Tessellate just this one face and integrate its triangles.
        let mut soup = FaceSoup::default();
        // 64 segments per analytic face is enough for 1e-3 relative volume
        // accuracy on a unit-radius cylinder/sphere — tighter than the
        // tolerance the e2e scenarios assert.
        crate::tessellate::tessellate_one_face_into(&mut soup, s, face_id, 64);
        return mesh_signed_volume(&soup);
    }

    let Some(lp) = s.topo.loop_(face.outer_loop()) else {
        return 0.0;
    };
    let Some(start) = lp.half_edge() else {
        return 0.0;
    };
    let mut polygon: Vec<Point3> = Vec::new();
    let mut cur = start;
    loop {
        let Some(he) = s.topo.half_edge(cur) else {
            return 0.0;
        };
        let v = he.origin();
        let Some(p) = s.vertex_geom.get(v) else {
            return 0.0;
        };
        polygon.push(*p);
        cur = he.next();
        if cur == start {
            break;
        }
    }
    if polygon.len() < 3 {
        // Degenerate loop — typically a planar circular cap whose loop is
        // a single self-loop half-edge on one vertex (cylinder/cone caps).
        // The cap's actual area lives on the circular edge geometry, not
        // in the loop's vertex sequence. Fall back to per-face tessellation
        // (the planar Plane branch in tessellate handles this).
        let mut soup = FaceSoup::default();
        crate::tessellate::tessellate_one_face_into(&mut soup, s, face_id, 64);
        return mesh_signed_volume(&soup);
    }
    let mut v = 0.0;
    let p0 = polygon[0].coords;
    for i in 1..polygon.len() - 1 {
        let pi = polygon[i].coords;
        let pj = polygon[i + 1].coords;
        let cross = pi.cross(&pj);
        v += p0.dot(&cross);
    }
    v / 6.0
}

/// Divergence-theorem volume of a tessellated face: 1/6 * sum of signed
/// tetrahedra from origin (1/6 * p0 . p1 x p2 per outward-oriented triangle).
fn mesh_signed_volume(soup: &FaceSoup) -> f64 {
    let mut v = 0.0;
    for tri in &soup.triangles {
        let p0 = tri[0].coords;
        let p1 = tri[1].coords;
        let p2 = tri[2].coords;
        v += p0.dot(&p1.cross(&p2));
    }
    v / 6.0
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::primitives::{box_, box_at, cylinder, revolve_polyline, sphere};
    use kerf_geom::{Point3, Vec3};

    #[test]
    fn unit_box_has_volume_one() {
        let s = box_(Vec3::new(1.0, 1.0, 1.0));
        assert!((solid_volume(&s) - 1.0).abs() < 1e-9, "got {}", solid_volume(&s));
    }

    #[test]
    fn box_2x3x4_has_volume_24() {
        let s = box_(Vec3::new(2.0, 3.0, 4.0));
        assert!((solid_volume(&s) - 24.0).abs() < 1e-9);
    }

    #[test]
    fn translated_box_has_same_volume() {
        let a = box_at(Vec3::new(1.5, 1.5, 1.5), Point3::new(10.0, -5.0, 7.0));
        assert!((solid_volume(&a) - 1.5_f64.powi(3)).abs() < 1e-9);
    }

    #[test]
    fn empty_solid_has_zero_volume() {
        let s = Solid::new();
        assert_eq!(solid_volume(&s), 0.0);
    }

    #[test]
    fn shell_volume_of_box_matches_solid_volume() {
        let s = box_(Vec3::new(2.0, 2.0, 2.0));
        let total: f64 = s
            .topo
            .shell_ids()
            .map(|sh| shell_volume(&s, sh))
            .sum();
        assert!((total - 8.0).abs() < 1e-9);
        assert!((solid_volume(&s) - 8.0).abs() < 1e-9);
    }

    #[test]
    fn revolved_vase_has_nonzero_volume() {
        // Bug fix regression: revolved solids previously reported volume 0
        // because the loop walk on a single 360°-spanning lune face
        // collapses to a degenerate polygon (back-and-forth seam + self-
        // looping circle). With the analytic-surface tessellation path,
        // the vase reports its actual swept volume.
        //
        // Profile: hourglass-style vase (matches scenario_06 in e2e tests).
        // Pappus's theorem on each profile segment gives the analytic
        // volume — for our purposes we just verify it's substantially
        // > 0 (the loop-walk path returned exactly 0).
        let vase = revolve_polyline(&[
            Point3::new(0.0, 0.0, 2.0),
            Point3::new(0.5, 0.0, 1.5),
            Point3::new(0.7, 0.0, 0.5),
            Point3::new(0.0, 0.0, 0.0),
        ]);
        let v = solid_volume(&vase);
        assert!(
            v > 1.0,
            "vase volume should be positive and substantial, got {v}"
        );
        // Also bracket against a containing cylinder of radius 0.7 and
        // height 2 — the vase pinches in the middle so its volume must
        // be strictly less than the cylinder.
        let cyl_vol = std::f64::consts::PI * 0.7 * 0.7 * 2.0;
        assert!(
            v < cyl_vol,
            "vase volume {v} should be less than enclosing cylinder {cyl_vol}"
        );
    }

    #[test]
    fn unit_cylinder_volume_matches_pi_r_squared_h() {
        // Sanity check on the analytic-surface path: a cylinder of radius
        // 1, height 2 has volume 2π. Both caps are planar (loop walk) and
        // the lateral is a single Cylinder face (tessellation path). The
        // sum should match π·r²·h.
        let s = cylinder(1.0, 2.0);
        let v = solid_volume(&s);
        let expected = std::f64::consts::PI * 1.0 * 1.0 * 2.0;
        // Tessellation at 64 segments: relative error ~ (1 - sin(π/64)/(π/64)) ~ 1e-3
        let rel = (v - expected).abs() / expected;
        assert!(rel < 5e-3, "cylinder vol {v} vs analytic {expected} (rel {rel})");
    }

    #[test]
    fn unit_sphere_volume_matches_4_pi_r3_over_3() {
        // The sphere primitive has a single Sphere face wrapping the
        // whole topology. Tessellation path computes 4/3 π r^3.
        let s = sphere(1.0);
        let v = solid_volume(&s);
        let expected = 4.0 / 3.0 * std::f64::consts::PI;
        let rel = (v - expected).abs() / expected;
        assert!(rel < 1e-2, "sphere vol {v} vs analytic {expected} (rel {rel})");
    }
}
