//! `capsule_faceted(r, h, n_hemi, n_slices)` — pure-mesh capsule.
//!
//! A capsule is a cylinder of radius `r` and height `h` with hemispherical
//! end-caps of the same radius. Total height = `h + 2*r`.
//!
//! Ring layout (n_rings = 2*n_hemi):
//!   Ring 1..n_hemi      — top hemisphere (theta in (0, pi/2], ring n_hemi is equator)
//!   Ring n_hemi+1       — bottom cylinder edge (z = -h/2, rho = r)
//!   Ring n_hemi+2..n_rings — bottom hemisphere (theta in (pi/2, pi), stops before pole)
//!
//! Vertex count: 2 + n_rings * n_slices.
//! Triangle count: n_rings * 2 * n_slices.
//!
//! Analytic volume: π r² h  +  (4/3) π r³.

use std::f64::consts::{PI, TAU};

use kerf_geom::Point3;

use crate::mesh_import::from_triangles;
use crate::Solid;

/// Build a capsule mesh (cylinder + hemispherical caps) using planar triangles.
///
/// * `r`        — cylinder and cap radius (must be positive).
/// * `h`        — cylinder height, i.e. gap between hemisphere centers (positive).
/// * `n_hemi`   — latitude rings per hemisphere, excluding the polar vertex
///                (must be ≥ 1; use 8–16 for smooth results).
/// * `n_slices` — longitude segments around the axis (must be ≥ 3).
///
/// # Panics (debug)
/// Panics if `r <= 0`, `h <= 0`, `n_hemi < 1`, or `n_slices < 3`.
pub fn capsule_faceted(r: f64, h: f64, n_hemi: usize, n_slices: usize) -> Solid {
    debug_assert!(r > 0.0, "radius must be positive");
    debug_assert!(h > 0.0, "height must be positive");
    debug_assert!(n_hemi >= 1, "n_hemi must be at least 1");
    debug_assert!(n_slices >= 3, "n_slices must be at least 3");

    // n_rings = 2 * n_hemi rings (NOT counting poles):
    //   rings 1..n_hemi         = top hemisphere (last ring = top cylinder equator)
    //   ring  n_hemi + 1        = bottom cylinder edge (z = -h/2, rho = r)
    //   rings n_hemi+2..2*n_hemi = bottom hemisphere (stops before south pole)
    //
    // The south pole is a separate vertex; the last bottom-hemi ring has
    // theta = pi/2 + (n_hemi-1)*pi/2/n_hemi = pi - pi/(2*n_hemi) < pi, so
    // rho > 0 and no degenerate triangles are produced.
    let n_rings = 2 * n_hemi;
    let total_verts = 2 + n_rings * n_slices;
    let mut verts: Vec<Point3> = Vec::with_capacity(total_verts);

    // North pole
    verts.push(Point3::new(0.0, 0.0, h / 2.0 + r));

    // Top hemisphere rings (1..=n_hemi)
    // theta = s * pi/2 / n_hemi;  at s=n_hemi: theta=pi/2 (equator, z=h/2, rho=r)
    for s in 1..=n_hemi {
        let theta = s as f64 * PI / 2.0 / n_hemi as f64;
        push_ring(&mut verts, h / 2.0 + r * theta.cos(), r * theta.sin(), n_slices);
    }

    // Bottom cylinder edge (ring n_hemi + 1)
    push_ring(&mut verts, -h / 2.0, r, n_slices);

    // Bottom hemisphere rings (n_hemi+2..=2*n_hemi = n_rings)
    // s=1..n_hemi-1: theta = pi/2 + s * pi/2 / n_hemi  (stops strictly before pi)
    for s in 1..n_hemi {
        let theta = PI / 2.0 + s as f64 * PI / 2.0 / n_hemi as f64;
        push_ring(&mut verts, -h / 2.0 + r * theta.cos(), r * theta.sin(), n_slices);
    }

    // South pole
    verts.push(Point3::new(0.0, 0.0, -(h / 2.0 + r)));

    debug_assert_eq!(verts.len(), total_verts);

    // Ring accessor: ring index ri in 1..=n_rings, slice k (wraps mod n_slices).
    let ring = |ri: usize, k: usize| -> Point3 {
        verts[1 + (ri - 1) * n_slices + k % n_slices]
    };
    let v_north = verts[0];
    let v_south = *verts.last().unwrap();

    let mut tris: Vec<[Point3; 3]> = Vec::with_capacity(n_rings * 2 * n_slices);

    // North cap
    for k in 0..n_slices {
        tris.push([v_north, ring(1, k), ring(1, k + 1)]);
    }

    // Middle bands (same CCW convention as sphere_faceted)
    for ri in 1..n_rings {
        for k in 0..n_slices {
            tris.push([ring(ri, k), ring(ri + 1, k), ring(ri + 1, k + 1)]);
            tris.push([ring(ri, k), ring(ri + 1, k + 1), ring(ri, k + 1)]);
        }
    }

    // South cap
    for k in 0..n_slices {
        tris.push([v_south, ring(n_rings, k + 1), ring(n_rings, k)]);
    }

    from_triangles(&tris).expect("capsule_faceted: mesh import failed")
}

/// Append n_slices vertices at the given (z, rho) position to the vertex list.
fn push_ring(verts: &mut Vec<Point3>, z: f64, rho: f64, n_slices: usize) {
    for k in 0..n_slices {
        let phi = k as f64 * TAU / n_slices as f64;
        verts.push(Point3::new(rho * phi.cos(), rho * phi.sin(), z));
    }
}

#[cfg(test)]
mod tests {
    use std::f64::consts::PI;

    use super::*;
    use kerf_topo::validate;

    use crate::measure::solid_volume;
    use crate::serde_io::{read_json, write_json};

    #[test]
    fn capsule_faceted_topology_n4_s8() {
        let n_hemi = 4;
        let n_slices = 8;
        let s = capsule_faceted(1.0, 2.0, n_hemi, n_slices);
        let n_rings = 2 * n_hemi;
        assert_eq!(s.vertex_count(), 2 + n_rings * n_slices);
        assert_eq!(s.face_count(), n_rings * 2 * n_slices);
        validate(&s.topo).expect("topology invalid");
    }

    #[test]
    fn capsule_faceted_topology_n1_s6() {
        // Minimal case: 1 hemisphere ring per cap (cylinder + cone fans).
        let n_hemi = 1;
        let n_slices = 6;
        let s = capsule_faceted(1.0, 2.0, n_hemi, n_slices);
        let n_rings = 2 * n_hemi; // 2
        assert_eq!(s.vertex_count(), 2 + n_rings * n_slices);
        assert_eq!(s.face_count(), n_rings * 2 * n_slices);
        validate(&s.topo).expect("topology invalid");
    }

    #[test]
    fn capsule_faceted_volume_positive() {
        let s = capsule_faceted(1.0, 2.0, 3, 6);
        assert!(solid_volume(&s) > 0.0, "volume must be positive (winding check)");
    }

    /// Analytic volume: π r² h + (4/3) π r³.  Tolerance 2% at n_hemi=12.
    #[test]
    fn capsule_faceted_volume_approx() {
        let r = 1.0;
        let h = 3.0;
        let s = capsule_faceted(r, h, 12, 24);
        let analytic = PI * r * r * h + (4.0 / 3.0) * PI * r * r * r;
        let computed = solid_volume(&s);
        let rel_err = ((computed - analytic) / analytic).abs();
        assert!(
            rel_err < 0.02,
            "volume error {rel_err:.4} >= 2% (computed={computed:.6}, analytic={analytic:.6})"
        );
    }

    #[test]
    fn capsule_faceted_json_roundtrip() {
        let s = capsule_faceted(1.0, 2.0, 4, 8);
        let v0 = s.vertex_count();
        let e0 = s.edge_count();
        let f0 = s.face_count();

        let mut buf = Vec::new();
        write_json(&s, &mut buf).expect("write_json failed");
        let s2 = read_json(&mut std::io::Cursor::new(&buf)).expect("read_json failed");

        assert_eq!(s2.vertex_count(), v0);
        assert_eq!(s2.edge_count(), e0);
        assert_eq!(s2.face_count(), f0);
        validate(&s2.topo).expect("roundtrip solid invalid");
    }
}
