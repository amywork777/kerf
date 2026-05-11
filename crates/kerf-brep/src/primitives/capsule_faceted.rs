//! `capsule_faceted(r, h, n)` — polyhedral capsule (cylinder + hemispherical caps).
//!
//! A capsule is a cylinder of radius `r` and body height `h` capped at both
//! ends by hemispheres of the same radius. Total height = `h + 2r`. All faces
//! are planar triangles (no analytic curved surfaces), so it works directly
//! with the planar boolean pipeline.
//!
//! The mesh is assembled from three sections whose equatorial seams share
//! vertices (deduplicated by the `from_triangles` importer):
//!
//!   1. **Top hemisphere** — north pole at `z = h/2 + r`, equator at `z = h/2`.
//!   2. **Cylinder band** — from `z = -h/2` to `z = h/2`, radius `r`.
//!   3. **Bottom hemisphere** — equator at `z = -h/2`, south pole at `z = -(h/2 + r)`.
//!
//! Triangle winding is CCW from the outward normal throughout (same convention
//! as `sphere_faceted` and `tessellate`).

use std::f64::consts::PI;

use kerf_geom::Point3;

use crate::mesh_import::from_triangles;
use crate::Solid;

/// Build a polyhedral capsule of radius `r`, cylindrical-body height `h`, and
/// `n` longitude segments.
///
/// Total height from tip to tip is `h + 2 * r`. Each hemispherical cap uses
/// `(n / 4).max(2)` latitude stacks, giving a smooth pole-to-equator transition
/// with the same density as the cylinder band.
///
/// # Panics
/// Debug builds: if `r <= 0`, `h <= 0`, or `n < 4`.
pub fn capsule_faceted(r: f64, h: f64, n: usize) -> Solid {
    debug_assert!(r > 0.0, "radius must be positive");
    debug_assert!(h > 0.0, "body height must be positive");
    debug_assert!(n >= 4, "n must be at least 4");

    let n_lon = n;
    let n_hemi = (n / 4).max(2); // latitude stacks per hemisphere

    let dlon = 2.0 * PI / n_lon as f64;
    let dlat = (PI / 2.0) / n_hemi as f64; // half-sphere spans 0..π/2 colatitude

    // Sphere surface point at colatitude v (0=north, π=south), longitude u.
    let spt = |u: f64, v: f64| -> Point3 {
        Point3::new(r * v.sin() * u.cos(), r * v.sin() * u.sin(), r * v.cos())
    };

    let mut tris: Vec<[Point3; 3]> = Vec::new();

    // ── Top hemisphere: colatitude 0 (north pole, z = h/2+r) to π/2 (equator, z = h/2).
    let dz_top = h / 2.0;
    for j in 0..n_hemi {
        let v0 = j as f64 * dlat;
        let v1 = (j + 1) as f64 * dlat;
        let north = j == 0;

        for i in 0..n_lon {
            let u0 = i as f64 * dlon;
            let u1 = ((i + 1) % n_lon) as f64 * dlon;
            let mut p00 = spt(u0, v0);
            let mut p10 = spt(u1, v0);
            let mut p11 = spt(u1, v1);
            let mut p01 = spt(u0, v1);
            p00.z += dz_top;
            p10.z += dz_top;
            p11.z += dz_top;
            p01.z += dz_top;

            if north {
                tris.push([p00, p01, p11]);
            } else {
                tris.push([p00, p01, p11]);
                tris.push([p00, p11, p10]);
            }
        }
    }

    // ── Cylinder band: from z = -h/2 to z = +h/2.
    // Winding: outward normal points radially away from the z-axis.
    // (p0_bot, p1_bot, p1_top) and (p0_bot, p1_top, p0_top) — matches tessellate.
    for i in 0..n_lon {
        let u0 = i as f64 * dlon;
        let u1 = ((i + 1) % n_lon) as f64 * dlon;
        let p0_bot = Point3::new(r * u0.cos(), r * u0.sin(), -h / 2.0);
        let p1_bot = Point3::new(r * u1.cos(), r * u1.sin(), -h / 2.0);
        let p0_top = Point3::new(r * u0.cos(), r * u0.sin(), h / 2.0);
        let p1_top = Point3::new(r * u1.cos(), r * u1.sin(), h / 2.0);
        tris.push([p0_bot, p1_bot, p1_top]);
        tris.push([p0_bot, p1_top, p0_top]);
    }

    // ── Bottom hemisphere: colatitude π/2 (equator, z = -h/2) to π (south pole, z = -(h/2+r)).
    let dz_bot = -h / 2.0;
    for j in 0..n_hemi {
        let v0 = PI / 2.0 + j as f64 * dlat;
        let v1 = PI / 2.0 + (j + 1) as f64 * dlat;
        let south = j == n_hemi - 1;

        for i in 0..n_lon {
            let u0 = i as f64 * dlon;
            let u1 = ((i + 1) % n_lon) as f64 * dlon;
            let mut p00 = spt(u0, v0);
            let mut p10 = spt(u1, v0);
            let mut p11 = spt(u1, v1);
            let mut p01 = spt(u0, v1);
            p00.z += dz_bot;
            p10.z += dz_bot;
            p11.z += dz_bot;
            p01.z += dz_bot;

            if south {
                tris.push([p00, p01, p10]);
            } else {
                tris.push([p00, p01, p11]);
                tris.push([p00, p11, p10]);
            }
        }
    }

    from_triangles(&tris).expect("capsule_faceted: produced non-manifold mesh")
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    fn analytic_capsule_vol(r: f64, h: f64) -> f64 {
        // cylinder + full sphere
        PI * r * r * h + 4.0 / 3.0 * PI * r * r * r
    }

    #[test]
    fn capsule_faceted_topology_validates() {
        let s = capsule_faceted(1.0, 2.0, 8);
        kerf_topo::validate(&s.topo).unwrap();
    }

    #[test]
    fn capsule_faceted_euler_holds() {
        let s = capsule_faceted(1.0, 2.0, 8);
        let v = s.vertex_count() as i64;
        let e = s.edge_count() as i64;
        let f = s.face_count() as i64;
        assert_eq!(v - e + f, 2, "V={v} E={e} F={f} — Euler violated");
    }

    #[test]
    fn capsule_faceted_volume_below_analytic() {
        let r = 1.0;
        let h = 2.0;
        let analytic = analytic_capsule_vol(r, h);
        let v = crate::measure::solid_volume(&capsule_faceted(r, h, 16));
        assert!(v < analytic, "faceted vol {v} >= analytic {analytic}");
        assert!(
            analytic - v < analytic * 0.05,
            "n=16 too far from analytic: vol={v} analytic={analytic}"
        );
    }

    #[test]
    fn capsule_faceted_volume_scales_correctly() {
        // Scaling radius and height by 2 → volume by 8
        let v1 = crate::measure::solid_volume(&capsule_faceted(1.0, 1.0, 16));
        let v2 = crate::measure::solid_volume(&capsule_faceted(2.0, 2.0, 16));
        let ratio = v2 / v1;
        assert!(
            (ratio - 8.0).abs() < 1e-6,
            "volume ratio {ratio} ≠ 8.0 for 2× scale"
        );
    }

    #[test]
    fn capsule_faceted_json_roundtrip() {
        use crate::serde_io::{read_json, write_json};
        let s = capsule_faceted(1.0, 2.0, 12);
        let mut buf = Vec::new();
        write_json(&s, &mut buf).unwrap();
        let s2 = read_json(&mut buf.as_slice()).unwrap();
        assert_eq!(s.vertex_count(), s2.vertex_count());
        assert_eq!(s.edge_count(), s2.edge_count());
        assert_eq!(s.face_count(), s2.face_count());
        kerf_topo::validate(&s2.topo).unwrap();
        let v1 = crate::measure::solid_volume(&s);
        let v2 = crate::measure::solid_volume(&s2);
        assert!((v1 - v2).abs() < 1e-9, "JSON roundtrip volume drift: {v1} → {v2}");
    }

    #[test]
    fn capsule_faceted_boolean_with_box() {
        use crate::primitives::box_at;
        use kerf_geom::{Point3, Vec3};
        // Slice a small box from the cylindrical body of the capsule.
        let s = capsule_faceted(1.0, 2.0, 16);
        // Cutter fully inside the cylinder body (|x|,|y| < 0.5, z in [-0.3, 0.3]).
        let cutter = box_at(
            Vec3::new(0.4, 0.4, 0.6),
            Point3::new(-0.2, -0.2, -0.3),
        );
        let v_before = crate::measure::solid_volume(&s);
        let cutter_vol = 0.4 * 0.4 * 0.6;
        let result = s.try_difference(&cutter).expect("capsule_faceted − box");
        let v_after = crate::measure::solid_volume(&result);
        let expected = v_before - cutter_vol;
        assert!(
            (v_after - expected).abs() < 0.05,
            "capsule − box: got {v_after}, expected {expected}"
        );
        kerf_topo::validate(&result.topo).unwrap();
    }
}
