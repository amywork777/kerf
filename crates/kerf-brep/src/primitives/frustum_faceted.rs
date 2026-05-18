//! `frustum_faceted(bot_r, top_r, h, n)` — n-gon frustum with flat planar faces.
//!
//! Unlike [`frustum`], the top and bottom caps are regular n-gons (not
//! analytic circles), and the lateral face is split into n planar trapezoids.
//! All faces are flat so `solid_volume` via the divergence theorem gives the
//! exact n-gon prismatoid volume.
//!
//! When `top_r == bot_r` the result is identical in topology to
//! [`cylinder_faceted`]; when `top_r = 0` it degenerates toward a pyramid
//! (not supported — use `n < 3` assertion).
//!
//! ## Topology (same as `cylinder_faceted` / `extrude_polygon`)
//!
//! V = 2n, E = 3n, F = n + 2
//!
//! ## Volume (exact for the n-gon prismatoid)
//!
//! ```text
//! A(r) = n/2 · r² · sin(2π/n)
//! V     = h/3 · (A_bot + A_top + √(A_bot · A_top))
//! ```
//!
//! For n → ∞ this converges to the cone frustum formula
//! `π·h/3·(r_bot² + r_bot·r_top + r_top²)`.

use std::f64::consts::{PI, TAU};

use kerf_geom::{Frame, Line, Plane, Point3, Vec3};
use kerf_topo::{validate, FaceId, MevResult};

use crate::booleans::face_polygon;
use crate::geometry::{CurveSegment, SurfaceKind};
use crate::Solid;

/// Build an n-gon frustum: bottom n-gon at z = 0 with circumradius `bot_r`,
/// top n-gon at z = `h` with circumradius `top_r`.
///
/// Both caps are regular n-gons aligned with the z-axis. Vertices are
/// phase-shifted by π/n so that for n = 4 the result is an axis-aligned
/// square frustum.
///
/// # Panics (debug)
/// Panics if `bot_r <= 0`, `top_r <= 0`, `h <= 0`, or `n < 3`.
pub fn frustum_faceted(bot_r: f64, top_r: f64, h: f64, n: usize) -> Solid {
    debug_assert!(bot_r > 0.0, "bot_r must be positive");
    debug_assert!(top_r > 0.0, "top_r must be positive");
    debug_assert!(h > 0.0, "height must be positive");
    debug_assert!(n >= 3, "n must be at least 3");

    let phase = PI / n as f64;

    let bottom: Vec<Point3> = (0..n)
        .map(|i| {
            let theta = phase + TAU * i as f64 / n as f64;
            Point3::new(bot_r * theta.cos(), bot_r * theta.sin(), 0.0)
        })
        .collect();

    let top: Vec<Point3> = (0..n)
        .map(|i| {
            let theta = phase + TAU * i as f64 / n as f64;
            Point3::new(top_r * theta.cos(), top_r * theta.sin(), h)
        })
        .collect();

    build_two_ring_prism(&bottom, &top)
}

/// Build a prism solid from two n-gon rings. The rings must have the same
/// number of vertices. Topology: V = 2n, E = 3n, F = n + 2.
///
/// This is a generalisation of `extrude_polygon` where the top ring can
/// have different vertex positions (not just a translated copy of the bottom).
fn build_two_ring_prism(bottom: &[Point3], top: &[Point3]) -> Solid {
    let n = bottom.len();
    debug_assert_eq!(top.len(), n);

    let mut s = Solid::new();

    // ---- Stage 1: mvfs to seed b_0. ----
    let r = s.topo.mvfs();
    let outer_loop = r.loop_;
    s.vertex_geom.insert(r.vertex, bottom[0]);

    // ---- Stage 2: Hamiltonian-path mev chain. ----
    // Path: b_0 → b_1 → … → b_{n-1} → t_{n-1} → t_0 → t_1 → … → t_{n-2}
    // (Same ordering as extrude_polygon.)
    let mev_b0: MevResult = s.topo.mev_at_lone_vertex(outer_loop, r.vertex);
    s.vertex_geom.insert(mev_b0.vertex, bottom[1]);
    let mut anchor = mev_b0.half_edges.0;

    let mut mev_b: Vec<MevResult> = Vec::with_capacity(n - 2);
    for &bi in bottom.iter().skip(2) {
        let m = s.topo.mev(outer_loop, anchor);
        s.vertex_geom.insert(m.vertex, bi);
        anchor = m.half_edges.0;
        mev_b.push(m);
    }

    let mev_up: MevResult = s.topo.mev(outer_loop, anchor);
    s.vertex_geom.insert(mev_up.vertex, top[n - 1]);
    anchor = mev_up.half_edges.0;

    let mut mev_t_fwd: Vec<MevResult> = Vec::with_capacity(n - 1);
    for &ti in top.iter().take(n - 1) {
        let m = s.topo.mev(outer_loop, anchor);
        s.vertex_geom.insert(m.vertex, ti);
        anchor = m.half_edges.0;
        mev_t_fwd.push(m);
    }

    // ---- Stage 3: n + 1 mef closures. ----
    let h_b0 = mev_b0.half_edges.0;
    let h_bn1 = mev_b[n - 3].half_edges.1;
    let _bottom_mef = s.topo.mef(h_b0, h_bn1);

    let h_tn1 = mev_t_fwd[0].half_edges.0;
    let h_tn2 = mev_t_fwd[n - 2].half_edges.1;
    let _top_mef = s.topo.mef(h_tn1, h_tn2);

    for j in 0..n - 1 {
        let h_tj = mev_t_fwd[j].half_edges.1;
        let h_bj = if j == 0 {
            mev_b0.half_edges.0
        } else {
            mev_b[j - 1].half_edges.0
        };
        s.topo.mef(h_tj, h_bj);
    }

    validate(&s.topo).expect("frustum_faceted topology violates Euler invariant");

    // ---- Stage 4: Attach edge geometry (line segments). ----
    let edge_ids: Vec<_> = s.topo.edge_ids().collect();
    for eid in edge_ids {
        let edge = s.topo.edge(eid).unwrap();
        let [he_a, _] = edge.half_edges();
        let v0 = s.topo.half_edge(he_a).unwrap().origin();
        let twin = s.topo.half_edge(he_a).unwrap().twin();
        let v1 = s.topo.half_edge(twin).unwrap().origin();
        let p0 = *s.vertex_geom.get(v0).unwrap();
        let p1 = *s.vertex_geom.get(v1).unwrap();
        let line = Line::through(p0, p1).unwrap();
        let length = (p1 - p0).norm();
        let seg = CurveSegment::line(line, 0.0, length);
        s.edge_geom.insert(eid, seg);
    }

    // ---- Stage 5: Attach face geometry (planes from vertex positions). ----
    let face_ids: Vec<_> = s.topo.face_ids().collect();
    for fid in face_ids {
        let frame = face_frame_from_polygon(&s, fid);
        s.face_geom.insert(fid, SurfaceKind::Plane(Plane::new(frame)));
    }

    s
}

/// Compute an outward-pointing face frame from the face polygon.
/// Uses the first non-degenerate cross product for the normal, then flips
/// to ensure it points away from the solid's centroid.
fn face_frame_from_polygon(s: &Solid, face: FaceId) -> Frame {
    let poly = face_polygon(s, face).expect("face must have polygon");
    let np = poly.len();
    debug_assert!(np >= 3);

    let p0 = poly[0];
    let p1 = poly[1];
    let p2 = poly[2];
    let mut normal = (p1 - p0).cross(&(p2 - p0));
    let nn = normal.norm();
    debug_assert!(nn > 1e-12, "degenerate face");
    normal /= nn;

    // Flip so normal points away from solid centroid.
    let centroid_face = poly.iter().fold(Vec3::zeros(), |acc, p| acc + p.coords) / np as f64;
    let mut solid_sum = Vec3::zeros();
    let mut count = 0.0_f64;
    for (_, p) in &s.vertex_geom {
        solid_sum += p.coords;
        count += 1.0;
    }
    let solid_centroid = solid_sum / count;
    if normal.dot(&(centroid_face - solid_centroid)) < 0.0 {
        normal = -normal;
    }

    let seed = if normal.dot(&Vec3::x()).abs() < 0.9 {
        Vec3::x()
    } else {
        Vec3::y()
    };
    let x = (seed - normal * seed.dot(&normal)).normalize();
    let y = normal.cross(&x);
    Frame { origin: p0, x, y, z: normal }
}

/// Exact volume of a regular n-gon frustum with bottom circumradius `bot_r`,
/// top circumradius `top_r`, and height `h`.
///
/// Formula: V = h/3 · (A_bot + A_top + √(A_bot · A_top))
/// where A(r) = n/2 · r² · sin(2π/n).
pub fn frustum_faceted_volume(bot_r: f64, top_r: f64, h: f64, n: usize) -> f64 {
    let a_bot = (n as f64 / 2.0) * bot_r * bot_r * (TAU / n as f64).sin();
    let a_top = (n as f64 / 2.0) * top_r * top_r * (TAU / n as f64).sin();
    h / 3.0 * (a_bot + a_top + (a_bot * a_top).sqrt())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::measure::solid_volume;
    use crate::serde_io::{read_json, write_json};
    use kerf_topo::validate;

    // ── topology ──────────────────────────────────────────────────────────────

    #[test]
    fn square_frustum_topology() {
        // n=4: V=8, E=12, F=6
        let s = frustum_faceted(2.0, 1.0, 3.0, 4);
        assert_eq!(s.vertex_count(), 8);
        assert_eq!(s.edge_count(), 12);
        assert_eq!(s.face_count(), 6);
        validate(&s.topo).unwrap();
    }

    #[test]
    fn hex_frustum_topology() {
        // n=6: V=12, E=18, F=8
        let s = frustum_faceted(3.0, 1.5, 2.0, 6);
        assert_eq!(s.vertex_count(), 12);
        assert_eq!(s.edge_count(), 18);
        assert_eq!(s.face_count(), 8);
        validate(&s.topo).unwrap();
    }

    #[test]
    fn tri_frustum_topology() {
        // n=3 (triangular frustum): V=6, E=9, F=5
        let s = frustum_faceted(2.0, 0.5, 4.0, 3);
        assert_eq!(s.vertex_count(), 6);
        assert_eq!(s.edge_count(), 9);
        assert_eq!(s.face_count(), 5);
        validate(&s.topo).unwrap();
    }

    #[test]
    fn all_faces_are_planar() {
        let s = frustum_faceted(2.0, 1.0, 3.0, 8);
        for (_, surf) in &s.face_geom {
            assert!(matches!(surf, SurfaceKind::Plane(_)), "expected Plane, got {surf:?}");
        }
    }

    #[test]
    fn all_edges_are_linear() {
        let s = frustum_faceted(2.0, 1.0, 3.0, 8);
        for (_, seg) in &s.edge_geom {
            assert!(matches!(seg.curve, crate::CurveKind::Line(_)));
        }
    }

    // ── volume ────────────────────────────────────────────────────────────────

    #[test]
    fn square_prism_volume_matches_box() {
        // When top_r == bot_r, frustum_faceted is a prism.
        // For a square prism with n=4, phase=π/4, each vertex is at distance r
        // from the axis. Side length = r * √2, so A = 2r².
        // V = 2r² * h.
        // With r=1, h=1: V = 2.0.
        let r = 1.0_f64;
        let h = 1.0;
        let s = frustum_faceted(r, r, h, 4);
        let vol = solid_volume(&s);
        let expected = frustum_faceted_volume(r, r, h, 4);
        // n=4 polygon area = 4/2 * r² * sin(π/2) = 2r²; V = 2r²*h.
        assert!((vol - expected).abs() < 1e-9, "got {vol}, expected {expected}");
        assert!((vol - 2.0).abs() < 1e-9, "n=4,r=1,h=1 should give V=2, got {vol}");
    }

    #[test]
    fn frustum_volume_matches_analytic_formula() {
        let bot_r = 3.0;
        let top_r = 1.5;
        let h = 4.0;
        let n = 8;
        let s = frustum_faceted(bot_r, top_r, h, n);
        let computed = solid_volume(&s);
        let expected = frustum_faceted_volume(bot_r, top_r, h, n);
        let rel = (computed - expected).abs() / expected;
        assert!(
            rel < 1e-9,
            "volume mismatch: computed={computed:.9}, expected={expected:.9}, rel={rel:.2e}"
        );
    }

    #[test]
    fn volume_converges_to_cone_frustum_as_n_grows() {
        // Analytic cone frustum: π·h/3·(r1² + r1·r2 + r2²)
        let (r1, r2, h) = (3.0, 1.5, 4.0);
        let analytic = std::f64::consts::PI / 3.0 * h * (r1 * r1 + r1 * r2 + r2 * r2);
        let n = 512;
        let vol = solid_volume(&frustum_faceted(r1, r2, h, n));
        let rel = (vol - analytic).abs() / analytic;
        // n/2 * sin(2π/n) ≈ π * (1 − π²/(3n²)); at n=512 the deficit is ~2.5e-5.
        assert!(rel < 3e-5, "n=512 should be within 0.003% of analytic; rel={rel:.2e}");
    }

    #[test]
    fn volume_monotone_increasing_in_n() {
        // More sides → volume increases (polygon area approaches circle area).
        let (r1, r2, h) = (2.0, 1.0, 3.0);
        let v6  = solid_volume(&frustum_faceted(r1, r2, h, 6));
        let v12 = solid_volume(&frustum_faceted(r1, r2, h, 12));
        let v24 = solid_volume(&frustum_faceted(r1, r2, h, 24));
        assert!(v6 < v12 && v12 < v24, "v6={v6}, v12={v12}, v24={v24}");
    }

    // ── geometry / bounds ─────────────────────────────────────────────────────

    #[test]
    fn vertex_z_extents_match_h() {
        let h = 5.0;
        let s = frustum_faceted(2.0, 1.0, h, 6);
        let z_min = s.vertex_geom.values().map(|p| p.z).fold(f64::INFINITY, f64::min);
        let z_max = s.vertex_geom.values().map(|p| p.z).fold(f64::NEG_INFINITY, f64::max);
        assert!((z_min - 0.0).abs() < 1e-12, "z_min={z_min}");
        assert!((z_max - h).abs() < 1e-12, "z_max={z_max}");
    }

    #[test]
    fn bottom_ring_radius_is_bot_r() {
        let bot_r = 3.0;
        let s = frustum_faceted(bot_r, 1.0, 2.0, 6);
        for (_, p) in &s.vertex_geom {
            if p.z.abs() < 1e-12 {
                let r = (p.x * p.x + p.y * p.y).sqrt();
                assert!((r - bot_r).abs() < 1e-10, "bottom radius={r}, expected {bot_r}");
            }
        }
    }

    #[test]
    fn top_ring_radius_is_top_r() {
        let top_r = 1.5;
        let h = 4.0;
        let s = frustum_faceted(3.0, top_r, h, 6);
        for (_, p) in &s.vertex_geom {
            if (p.z - h).abs() < 1e-12 {
                let r = (p.x * p.x + p.y * p.y).sqrt();
                assert!((r - top_r).abs() < 1e-10, "top radius={r}, expected {top_r}");
            }
        }
    }

    // ── json round-trip ───────────────────────────────────────────────────────

    #[test]
    fn json_roundtrip_preserves_counts() {
        let s = frustum_faceted(2.0, 1.0, 3.0, 6);
        let mut buf = Vec::new();
        write_json(&s, &mut buf).unwrap();
        let s2 = read_json(&mut buf.as_slice()).unwrap();
        assert_eq!(s.vertex_count(), s2.vertex_count());
        assert_eq!(s.edge_count(), s2.edge_count());
        assert_eq!(s.face_count(), s2.face_count());
        validate(&s2.topo).unwrap();
    }

    #[test]
    fn json_roundtrip_volume_preserved() {
        let s = frustum_faceted(3.0, 1.5, 4.0, 8);
        let vol = solid_volume(&s);
        let mut buf = Vec::new();
        write_json(&s, &mut buf).unwrap();
        let s2 = read_json(&mut buf.as_slice()).unwrap();
        let vol2 = solid_volume(&s2);
        assert!(
            (vol - vol2).abs() < 1e-9,
            "volume before={vol}, after={vol2}"
        );
    }
}
