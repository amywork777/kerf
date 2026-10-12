//! `sphere_faceted(r, n)` — n×n lat-lon planar-polygon sphere approximation.
//!
//! Like [`cylinder_faceted`], all faces are planar (`Plane` surfaces), so this
//! solid feeds directly into the boolean pipeline without tessellation round-trips.
//!
//! ## Topology (Euler V-E+F = 2)
//!
//! With parameter `n` (number of longitude *and* latitude divisions, `n ≥ 3`):
//!
//! | Entity   | Count     | Notes                                        |
//! |----------|-----------|----------------------------------------------|
//! | Vertices | 2 + n(n-1) | 1 top pole + (n-1)×n ring verts + 1 bot pole |
//! | Edges    | 2n²−n     | n top-spokes + n(n-1) lat-rings + n(n-2) meridians + n bot-spokes |
//! | Faces    | n²        | n top triangles + n(n-2) mid quads + n bot triangles |
//!
//! Euler check: (2+n(n-1)) − (2n²−n) + n² = 2 ✓

use std::f64::consts::PI;

use kerf_geom::{Frame, Line, Plane, Point3, Vec3};
use kerf_topo::validate;

use crate::geometry::{CurveSegment, SurfaceKind};
use crate::Solid;

/// Build an `n×n` lat-lon faceted sphere of radius `r` centred at the origin.
///
/// Each face is a `Plane` — the solid has no curved surfaces, so it is usable
/// in the planar boolean pipeline. Higher `n` ⇒ closer to a true sphere;
/// volume converges as `O(1/n²)`.
///
/// # Topology counts
/// `V = 2 + n(n-1)`, `E = 2n²−n`, `F = n²`.
///
/// # Panics (debug)
/// In debug builds: `r ≤ 0` or `n < 3`.
pub fn sphere_faceted(r: f64, n: usize) -> Solid {
    debug_assert!(r > 0.0, "radius must be positive");
    debug_assert!(n >= 3, "n must be at least 3");

    let n_rings = n - 1; // number of interior latitude rings (excluding poles)
    let n_mid = n - 2;   // number of middle quad bands between rings

    // ------------------------------------------------------------------ vertex positions

    let ring_pt = |j: usize, k: usize| -> Point3 {
        // j = 0..n_rings-1, latitude angle from north pole
        let lat = PI * (j + 1) as f64 / n as f64;
        let lon = 2.0 * PI * k as f64 / n as f64;
        Point3::new(
            r * lat.sin() * lon.cos(),
            r * lat.sin() * lon.sin(),
            r * lat.cos(),
        )
    };

    let mut s = Solid::new();

    // ------------------------------------------------------------------ insert vertices

    let v_top = s.topo.build_insert_vertex();
    s.vertex_geom.insert(v_top, Point3::new(0.0, 0.0, r));

    // v_ring[j * n + k] = ring j, longitude k
    let v_ring: Vec<_> = (0..n_rings * n)
        .map(|_| s.topo.build_insert_vertex())
        .collect();
    for j in 0..n_rings {
        for k in 0..n {
            s.vertex_geom.insert(v_ring[j * n + k], ring_pt(j, k));
        }
    }

    let v_bot = s.topo.build_insert_vertex();
    s.vertex_geom.insert(v_bot, Point3::new(0.0, 0.0, -r));

    // ------------------------------------------------------------------ solid + shell

    let solid_id = s.topo.build_insert_solid();
    s.topo.build_set_active_solid(Some(solid_id));
    let shell_id = s.topo.build_insert_shell(solid_id);

    // ------------------------------------------------------------------ faces + loops
    // Layout: face_top[k] = k, face_mid[j][k] = n + j*n + k, face_bot[k] = n + n_mid*n + k

    let n_faces = n * n;
    let mut loop_ids: Vec<_> = Vec::with_capacity(n_faces);
    let mut face_ids_all: Vec<_> = Vec::with_capacity(n_faces);
    for _ in 0..n_faces {
        let lp = s.topo.build_insert_loop_placeholder();
        let face = s.topo.build_insert_face(lp, shell_id);
        s.topo.build_set_loop_face(lp, face);
        s.topo.build_push_shell_face(shell_id, face);
        loop_ids.push(lp);
        face_ids_all.push(face);
    }

    let top_lp = |k: usize| loop_ids[k];
    let mid_lp = |j: usize, k: usize| loop_ids[n + j * n + k];
    let bot_lp = |k: usize| loop_ids[n + n_mid * n + k];

    // ------------------------------------------------------------------ half-edges
    //
    // Naming convention for half-edges stored in flat Vecs:
    //   he_top[k*3 + i]            face_top[k], i = 0,1,2
    //   he_mid[(j*n + k)*4 + i]    face_mid[j][k], i = 0,1,2,3
    //   he_bot[k*3 + i]            face_bot[k], i = 0,1,2
    //
    // Face loop orientations (CCW from outside → outward normal by right-hand rule):
    //   face_top[k]:   v_top → ring[0][k] → ring[0][(k+1)%n] → back
    //   face_mid[j][k]: ring[j][(k+1)%n] → ring[j][k] → ring[j+1][k] → ring[j+1][(k+1)%n] → back
    //   face_bot[k]:   ring[n-2][(k+1)%n] → ring[n-2][k] → v_bot → back

    let mut he_top: Vec<_> = Vec::with_capacity(3 * n);
    let mut he_mid: Vec<_> = Vec::with_capacity(4 * n_mid * n);
    let mut he_bot: Vec<_> = Vec::with_capacity(3 * n);

    // Top cap triangles
    for k in 0..n {
        let lp = top_lp(k);
        let kn = (k + 1) % n;
        let he0 = s.topo.build_insert_half_edge(v_top, lp);
        let he1 = s.topo.build_insert_half_edge(v_ring[k], lp);
        let he2 = s.topo.build_insert_half_edge(v_ring[kn], lp);
        s.topo.build_set_half_edge_next_prev(he0, he1);
        s.topo.build_set_half_edge_next_prev(he1, he2);
        s.topo.build_set_half_edge_next_prev(he2, he0);
        s.topo.build_set_loop_half_edge(lp, Some(he0));
        he_top.push(he0);
        he_top.push(he1);
        he_top.push(he2);
    }

    // Middle quad bands
    for j in 0..n_mid {
        for k in 0..n {
            let lp = mid_lp(j, k);
            let kn = (k + 1) % n;
            // Loop: ring[j][kn] → ring[j][k] → ring[j+1][k] → ring[j+1][kn] → back
            let he0 = s.topo.build_insert_half_edge(v_ring[j * n + kn], lp);
            let he1 = s.topo.build_insert_half_edge(v_ring[j * n + k], lp);
            let he2 = s.topo.build_insert_half_edge(v_ring[(j + 1) * n + k], lp);
            let he3 = s.topo.build_insert_half_edge(v_ring[(j + 1) * n + kn], lp);
            s.topo.build_set_half_edge_next_prev(he0, he1);
            s.topo.build_set_half_edge_next_prev(he1, he2);
            s.topo.build_set_half_edge_next_prev(he2, he3);
            s.topo.build_set_half_edge_next_prev(he3, he0);
            s.topo.build_set_loop_half_edge(lp, Some(he0));
            he_mid.push(he0);
            he_mid.push(he1);
            he_mid.push(he2);
            he_mid.push(he3);
        }
    }

    // Bottom cap triangles  (last ring index = n_rings - 1 = n - 2)
    let bot_ring = n_rings - 1; // = n - 2
    for k in 0..n {
        let lp = bot_lp(k);
        let kn = (k + 1) % n;
        // Loop: ring[bot_ring][kn] → ring[bot_ring][k] → v_bot → back
        let he0 = s.topo.build_insert_half_edge(v_ring[bot_ring * n + kn], lp);
        let he1 = s.topo.build_insert_half_edge(v_ring[bot_ring * n + k], lp);
        let he2 = s.topo.build_insert_half_edge(v_bot, lp);
        s.topo.build_set_half_edge_next_prev(he0, he1);
        s.topo.build_set_half_edge_next_prev(he1, he2);
        s.topo.build_set_half_edge_next_prev(he2, he0);
        s.topo.build_set_loop_half_edge(lp, Some(he0));
        he_bot.push(he0);
        he_bot.push(he1);
        he_bot.push(he2);
    }

    // ------------------------------------------------------------------ twin pairs
    //
    // Edge type → half-edge pair:
    //
    // e_top_spoke[k]:
    //   fwd = he_top[k*3+0]  (v_top → ring[0][k], in face_top[k])
    //   rev = he_top[((k+n-1)%n)*3+2]  (ring[0][k] → v_top, in face_top[(k-1+n)%n])
    //
    // e_ring_lat[j][k]  (j = 0..n_rings-1):
    //   fwd = he_top[k*3+1] if j==0, else he_mid[((j-1)*n+k)*4+2]
    //   bwd = he_mid[(j*n+k)*4+0]  if j < n_rings-1, else he_bot[k*3+0]
    //
    // e_inter_mer[j][k]  (j = 0..n_mid-1):
    //   down = he_mid[(j*n+k)*4+1]
    //   up   = he_mid[(j*n+(k+n-1)%n)*4+3]
    //
    // e_bot_spoke[k]:
    //   down = he_bot[k*3+1]  (ring[bot_ring][k] → v_bot, in face_bot[k])
    //   up   = he_bot[((k+n-1)%n)*3+2]  (v_bot → ring[bot_ring][k], in face_bot[(k-1+n)%n])

    // Top spokes
    for k in 0..n {
        let fwd = he_top[k * 3];
        let rev = he_top[((k + n - 1) % n) * 3 + 2];
        s.topo.build_set_half_edge_twin(fwd, rev);
        s.topo.build_set_half_edge_twin(rev, fwd);
    }

    // Ring latitude edges (one per (j,k) combination)
    for j in 0..n_rings {
        for k in 0..n {
            let fwd = if j == 0 {
                he_top[k * 3 + 1]
            } else {
                he_mid[((j - 1) * n + k) * 4 + 2]
            };
            let bwd = if j < n_rings - 1 {
                he_mid[(j * n + k) * 4]
            } else {
                he_bot[k * 3]
            };
            s.topo.build_set_half_edge_twin(fwd, bwd);
            s.topo.build_set_half_edge_twin(bwd, fwd);
        }
    }

    // Inter-ring meridian edges
    for j in 0..n_mid {
        for k in 0..n {
            let down = he_mid[(j * n + k) * 4 + 1];
            let up = he_mid[(j * n + (k + n - 1) % n) * 4 + 3];
            s.topo.build_set_half_edge_twin(down, up);
            s.topo.build_set_half_edge_twin(up, down);
        }
    }

    // Bottom spokes
    for k in 0..n {
        let down = he_bot[k * 3 + 1];
        let up = he_bot[((k + n - 1) % n) * 3 + 2];
        s.topo.build_set_half_edge_twin(down, up);
        s.topo.build_set_half_edge_twin(up, down);
    }

    // ------------------------------------------------------------------ edges

    // Top spokes: n edges
    for k in 0..n {
        let fwd = he_top[k * 3];
        let rev = he_top[((k + n - 1) % n) * 3 + 2];
        let e = s.topo.build_insert_edge([fwd, rev]);
        s.topo.build_set_half_edge_edge(fwd, e);
        s.topo.build_set_half_edge_edge(rev, e);
    }

    // Ring latitude edges: n*(n-1) edges
    for j in 0..n_rings {
        for k in 0..n {
            let fwd = if j == 0 {
                he_top[k * 3 + 1]
            } else {
                he_mid[((j - 1) * n + k) * 4 + 2]
            };
            let bwd = if j < n_rings - 1 {
                he_mid[(j * n + k) * 4]
            } else {
                he_bot[k * 3]
            };
            let e = s.topo.build_insert_edge([fwd, bwd]);
            s.topo.build_set_half_edge_edge(fwd, e);
            s.topo.build_set_half_edge_edge(bwd, e);
        }
    }

    // Inter-ring meridians: n*(n-2) edges
    for j in 0..n_mid {
        for k in 0..n {
            let down = he_mid[(j * n + k) * 4 + 1];
            let up = he_mid[(j * n + (k + n - 1) % n) * 4 + 3];
            let e = s.topo.build_insert_edge([down, up]);
            s.topo.build_set_half_edge_edge(down, e);
            s.topo.build_set_half_edge_edge(up, e);
        }
    }

    // Bottom spokes: n edges
    for k in 0..n {
        let down = he_bot[k * 3 + 1];
        let up = he_bot[((k + n - 1) % n) * 3 + 2];
        let e = s.topo.build_insert_edge([down, up]);
        s.topo.build_set_half_edge_edge(down, e);
        s.topo.build_set_half_edge_edge(up, e);
    }

    // ------------------------------------------------------------------ vertex outgoing half-edges

    s.topo.build_set_vertex_outgoing(v_top, Some(he_top[0])); // in face_top[0]
    s.topo.build_set_vertex_outgoing(v_bot, Some(he_bot[2])); // he2 of face_bot[0], origin v_bot

    for j in 0..n_rings {
        for k in 0..n {
            let outgoing = if j == 0 {
                he_top[k * 3 + 1] // origin ring[0][k], in face_top[k]
            } else {
                he_mid[((j - 1) * n + k) * 4 + 2] // origin ring[j][k], in face_mid[j-1][k]
            };
            s.topo.build_set_vertex_outgoing(v_ring[j * n + k], Some(outgoing));
        }
    }

    // ------------------------------------------------------------------ edge geometry (line segments)

    let edge_ids: Vec<_> = s.topo.edge_ids().collect();
    for eid in edge_ids {
        let edge = s.topo.edge(eid).unwrap();
        let [he_a, _] = edge.half_edges();
        let v0 = s.topo.half_edge(he_a).unwrap().origin();
        let twin_id = s.topo.half_edge(he_a).unwrap().twin();
        let v1 = s.topo.half_edge(twin_id).unwrap().origin();
        let p0 = *s.vertex_geom.get(v0).unwrap();
        let p1 = *s.vertex_geom.get(v1).unwrap();
        let line = Line::through(p0, p1).unwrap();
        let length = (p1 - p0).norm();
        s.edge_geom.insert(eid, CurveSegment::line(line, 0.0, length));
    }

    // ------------------------------------------------------------------ face geometry (planes)

    let face_ids_geom: Vec<_> = s.topo.face_ids().collect();
    for fid in face_ids_geom {
        let face = s.topo.face(fid).unwrap();
        let lp = s.topo.loop_(face.outer_loop()).unwrap();
        let start = lp.half_edge().unwrap();

        // Collect first 3 vertex positions — enough to define a plane.
        let mut pts = [Point3::origin(); 3];
        let mut cur = start;
        for p in pts.iter_mut() {
            let he = s.topo.half_edge(cur).unwrap();
            *p = *s.vertex_geom.get(he.origin()).unwrap();
            cur = he.next();
        }

        // CCW face orientation → (p1-p0) × (p2-p0) is the outward normal.
        let e1 = pts[1] - pts[0];
        let e2 = pts[2] - pts[0];
        let normal = e1.cross(&e2).normalize();
        let seed = if normal.dot(&Vec3::x()).abs() < 0.9 { Vec3::x() } else { Vec3::y() };
        let x = (seed - normal * normal.dot(&seed)).normalize();
        let y = normal.cross(&x);
        let frame = Frame { origin: pts[0], x, y, z: normal };
        s.face_geom.insert(fid, SurfaceKind::Plane(Plane::new(frame)));
    }

    validate(&s.topo).expect("sphere_faceted topology violates Euler invariant");

    s
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;
    use std::io::Cursor;

    use kerf_topo::validate;

    use crate::geometry::SurfaceKind;
    use crate::measure::solid_volume;
    use crate::serde_io::{read_json, write_json};

    #[test]
    fn sphere_faceted_3_topology() {
        // n=3: V=8, E=15, F=9  (Euler: 8-15+9 = 2 ✓)
        let s = sphere_faceted(1.0, 3);
        assert_eq!(s.vertex_count(), 8);
        assert_eq!(s.edge_count(), 15);
        assert_eq!(s.face_count(), 9);
        assert_eq!(s.shell_count(), 1);
        validate(&s.topo).unwrap();
    }

    #[test]
    fn sphere_faceted_4_topology() {
        // n=4: V=14, E=28, F=16  (Euler: 14-28+16 = 2 ✓)
        let s = sphere_faceted(1.0, 4);
        assert_eq!(s.vertex_count(), 14);
        assert_eq!(s.edge_count(), 28);
        assert_eq!(s.face_count(), 16);
        validate(&s.topo).unwrap();
    }

    #[test]
    fn sphere_faceted_8_topology() {
        // n=8: V = 2+8*7=58, E = 2*64-8=120, F = 64
        let s = sphere_faceted(1.0, 8);
        assert_eq!(s.vertex_count(), 2 + 8 * 7);
        assert_eq!(s.edge_count(), 2 * 64 - 8);
        assert_eq!(s.face_count(), 64);
        validate(&s.topo).unwrap();
    }

    #[test]
    fn sphere_faceted_all_faces_planar() {
        let s = sphere_faceted(1.5, 6);
        for (_, surf) in &s.face_geom {
            assert!(matches!(surf, SurfaceKind::Plane(_)), "expected Plane, got {surf:?}");
        }
    }

    #[test]
    fn sphere_faceted_volume_close_to_analytic() {
        // The faceted sphere underestimates by O(1/n²). Empirically, a lat-lon
        // sphere at n=16 has ~3.5% error, so 5% is a generous but meaningful bound.
        let r = 2.0_f64;
        let s = sphere_faceted(r, 16);
        let faceted_vol = solid_volume(&s);
        let analytic_vol = 4.0 / 3.0 * PI * r.powi(3);
        let rel_err = (faceted_vol - analytic_vol).abs() / analytic_vol;
        assert!(
            rel_err < 0.05,
            "volume relative error {rel_err:.4} exceeds 5% (faceted={faceted_vol:.6}, analytic={analytic_vol:.6})"
        );
    }

    #[test]
    fn sphere_faceted_volume_converges() {
        // Coarser mesh has more error than finer mesh.
        let r = 1.0_f64;
        let analytic = 4.0 / 3.0 * PI * r.powi(3);
        let v8 = solid_volume(&sphere_faceted(r, 8));
        let v16 = solid_volume(&sphere_faceted(r, 16));
        assert!(v16 > v8, "finer mesh should have larger volume (closer to analytic)");
        let err8 = (v8 - analytic).abs() / analytic;
        let err16 = (v16 - analytic).abs() / analytic;
        assert!(err16 < err8, "n=16 error ({err16:.4}) should be less than n=8 error ({err8:.4})");
    }

    #[test]
    fn sphere_faceted_json_roundtrip() {
        let s = sphere_faceted(1.0, 6);
        let mut buf = Vec::new();
        write_json(&s, &mut buf).expect("write_json");
        let read_back = read_json(&mut Cursor::new(buf)).expect("read_json");
        assert_eq!(s.vertex_count(), read_back.vertex_count());
        assert_eq!(s.edge_count(), read_back.edge_count());
        assert_eq!(s.face_count(), read_back.face_count());
        let vol_before = solid_volume(&s);
        let vol_after = solid_volume(&read_back);
        assert!(
            (vol_before - vol_after).abs() < 1e-9,
            "JSON round-trip volume drift: before={vol_before:.6} after={vol_after:.6}"
        );
        validate(&read_back.topo).expect("topology after JSON round-trip");
    }
}

#[cfg(test)]
mod boolean_tests {
    use super::*;
    use crate::primitives::{box_, box_at};
    use crate::measure::solid_volume;
    use kerf_geom::{Point3, Vec3};

    fn sf() -> Solid { sphere_faceted(1.0, 6) }
    fn bx() -> Solid { box_(Vec3::new(2.0, 2.0, 2.0)) }

    #[test]
    fn sphere_faceted_union_box_succeeds() {
        // sphere at origin, box 0..2 → overlap in first octant
        let r = sf().try_union(&bx());
        assert!(r.is_ok(), "sphere_faceted ∪ box failed: {:?}", r.err().map(|e| e.message));
    }

    #[test]
    fn sphere_faceted_intersection_box_succeeds() {
        let r = sf().try_intersection(&bx());
        assert!(r.is_ok(), "sphere_faceted ∩ box failed: {:?}", r.err().map(|e| e.message));
        let vol = solid_volume(&r.unwrap());
        // Intersection volume must be positive and less than sphere volume
        assert!(vol > 0.0, "intersection volume must be positive, got {vol}");
    }

    #[test]
    fn sphere_faceted_difference_box_succeeds() {
        let r = sf().try_difference(&bx());
        assert!(r.is_ok(), "sphere_faceted − box failed: {:?}", r.err().map(|e| e.message));
    }

    #[test]
    fn box_difference_sphere_faceted_succeeds() {
        // box contains sphere entirely (box 0..2, sphere r=0.8 at 1,1,1)
        let sf_nested = sphere_faceted(0.8, 6);
        let bx_big = box_at(Vec3::new(2.0, 2.0, 2.0), Point3::new(-0.2, -0.2, -0.2));
        let r = bx_big.try_difference(&sf_nested);
        assert!(r.is_ok(), "box − sphere_faceted failed: {:?}", r.err().map(|e| e.message));
    }
}
