//! `extrude_polygon(profile, direction)` — build a prism with an N-gon base.

use kerf_geom::{Frame, Line, Plane, Point3, Vec3};
use kerf_topo::{validate, FaceId, MevResult};

use crate::booleans::face_polygon;
use crate::geometry::{CurveSegment, SurfaceKind};
use crate::Solid;

/// Extrude a CCW convex polygon along `direction`, returning the prism solid.
///
/// `profile` must have at least 3 vertices in CCW order (when viewed from
/// `+direction`). `direction` must not be coplanar with the profile.
///
/// Panics (debug builds) on degenerate input.
pub fn extrude_polygon(profile: &[Point3], direction: Vec3) -> Solid {
    debug_assert!(profile.len() >= 3, "profile must have at least 3 vertices");
    debug_assert!(direction.norm() > 1e-12, "direction must be non-zero");
    let top: Vec<Point3> = profile.iter().map(|p| *p + direction).collect();
    extrude_lofted(profile, &top)
}

/// Connect two parallel polygons with the same vertex count via flat side
/// faces, producing a closed prism-or-frustum-or-loft solid. Each side
/// face is a quad from `bottom[i] - bottom[(i+1) % n] - top[(i+1) % n] -
/// top[i]`. Quads are accepted as planar by the topology validator if
/// they actually are planar (the caller's responsibility — for true loft
/// between non-similar polygons, side faces may be non-planar and the
/// engine will treat them as such with degraded boolean robustness).
///
/// `bottom` and `top` must have the same length ≥ 3 and the same CCW
/// orientation when viewed from outside (typically: bottom's normal is
/// opposite the direction toward top).
pub fn extrude_lofted(bottom: &[Point3], top: &[Point3]) -> Solid {
    debug_assert!(bottom.len() >= 3, "bottom profile must have at least 3 vertices");
    debug_assert_eq!(
        bottom.len(),
        top.len(),
        "top profile must have the same vertex count as bottom"
    );

    let n = bottom.len();
    let mut s = Solid::new();
    let bottom: Vec<Point3> = bottom.to_vec();
    let top: Vec<Point3> = top.to_vec();

    // ---- Stage 1: mvfs to seed b_0. ----
    let r = s.topo.mvfs();
    let outer_loop = r.loop_;
    s.vertex_geom.insert(r.vertex, bottom[0]);

    // ---- Stage 2: Hamiltonian-path mev chain. ----
    //
    // Path: b_0 -> b_1 -> ... -> b_{N-1} -> t_{N-1} -> t_0 -> t_1 -> ... -> t_{N-2}
    //
    // This ordering mirrors box_(): after the vertical edge b_{N-1}->t_{N-1},
    // the top chain visits t_0 first (forward order), so that the resulting
    // active loop after the bottom+top mef caps has a clean sequential
    // structure for the side mef closures.
    //
    // At each mev(loop, anchor):
    //   v_old = anchor.twin.origin  (current tip)
    //   result.he_a (.half_edges.0) origin = v_old
    //   result.he_b (.half_edges.1) origin = v_new
    //   next anchor = result.half_edges.0

    // mev_b0: mev_at_lone_vertex growing b_1.
    //   .0 origin = b_0,  .1 origin = b_1
    let mev_b0: MevResult = s.topo.mev_at_lone_vertex(outer_loop, r.vertex);
    s.vertex_geom.insert(mev_b0.vertex, bottom[1]);
    let mut anchor = mev_b0.half_edges.0;

    // mev_b[k] for k in 0..n-2: grows b_{k+2} from b_{k+1}.
    //   .0 origin = b_{k+1},  .1 origin = b_{k+2}
    let mut mev_b: Vec<MevResult> = Vec::with_capacity(n - 2);
    for &bi in bottom.iter().skip(2) {
        let m = s.topo.mev(outer_loop, anchor);
        s.vertex_geom.insert(m.vertex, bi);
        anchor = m.half_edges.0;
        mev_b.push(m);
    }
    // mev_b.len() == n-2 (may be 0 for n=3... wait n>=3 so n-2>=1, but for n=3 it's 1)
    // For n=3: mev_b has 1 element: mev_b[0] grew b_2.

    // mev_up: grows t_{N-1} from b_{N-1}.
    //   .0 origin = b_{N-1},  .1 origin = t_{N-1}
    let mev_up: MevResult = s.topo.mev(outer_loop, anchor);
    s.vertex_geom.insert(mev_up.vertex, top[n - 1]);
    anchor = mev_up.half_edges.0;

    // mev_t_fwd[j] for j in 0..n-1: grows t_j (NOT t_{N-1-j} — forward order).
    //   j=0: grows t_0 from t_{N-1}.  .0 origin = t_{N-1}, .1 origin = t_0
    //   j=1: grows t_1 from t_0.      .0 origin = t_0,     .1 origin = t_1
    //   ...
    //   j=N-2: grows t_{N-2}.         .0 origin = t_{N-3}, .1 origin = t_{N-2}
    //
    // Length n-1 (visits t_0 through t_{N-2}).
    let mut mev_t_fwd: Vec<MevResult> = Vec::with_capacity(n - 1);
    for &ti in top.iter().take(n - 1) {
        let m = s.topo.mev(outer_loop, anchor);
        s.vertex_geom.insert(m.vertex, ti); // grows t_j in order
        anchor = m.half_edges.0;
        mev_t_fwd.push(m);
    }

    // ---- Stage 3: N+1 mef closures. ----
    //
    // Full loop before any mef (2N half-edges, walking the path and back):
    //
    //   mev_b0.0(b_0), mev_b[0].0(b_1), ..., mev_b[n-3].0(b_{N-2}),
    //   mev_up.0(b_{N-1}),
    //   mev_t_fwd[0].0(t_{N-1}), mev_t_fwd[1].0(t_0), ..., mev_t_fwd[n-2].0(t_{N-3}),
    //   mev_t_fwd[n-2].1(t_{N-2}), ..., mev_t_fwd[1].1(t_1), mev_t_fwd[0].1(t_0),
    //   mev_up.1(t_{N-1}),
    //   mev_b[n-3].1(b_{N-1}), ..., mev_b[0].1(b_2), mev_b0.1(b_1)
    //
    // (Wait — I need to re-check. The anchor for mev_t_fwd[0] is mev_up.0, and
    //  anchor.twin = mev_up.1 (origin t_{N-1}), so v_old = t_{N-1}.
    //  mev_t_fwd[0] grows t_0: .0 origin = t_{N-1}, .1 origin = t_0.
    //  That's inserted AFTER mev_up.0 in the loop.)
    //
    // The full loop for N=3 is:
    //   mev_b0.0(b_0), mev_b[0].0(b_1), mev_up.0(b_2),
    //   mev_t_fwd[0].0(t_2), mev_t_fwd[1].0(t_0), [nothing more for n-1=2 iters],
    //   mev_t_fwd[1].1(t_1), mev_t_fwd[0].1(t_0),
    //   mev_up.1(t_2), mev_b[0].1(b_2), mev_b0.1(b_1)
    //
    // a) Bottom closure: mef(mev_b0.0 = h(b_0), mev_b[n-3].1 = h(b_{N-1}))
    //    Creates the bottom polygon face and leaves the active shell in the new loop.
    //    (Mirrors box_: mef(m1.0, m3.1) for N=4.)

    let h_b0 = mev_b0.half_edges.0;          // origin b_0 (forward)
    let h_bn1 = mev_b[n - 3].half_edges.1;   // origin b_{N-1} (back)
    let _bottom_mef = s.topo.mef(h_b0, h_bn1);
    // outer_loop = bottom polygon.  bottom_mef.loop_ = active shell.

    // b) Top closure: mef(mev_t_fwd[0].0 = h(t_{N-1}), mev_t_fwd[n-2].1 = h(t_{N-2}))
    //    Splits off the top polygon from the active shell.
    //    (Mirrors box_: mef(m5.0, m7.1) for N=4.)
    let h_tn1 = mev_t_fwd[0].half_edges.0;       // origin t_{N-1}
    let h_tn2 = mev_t_fwd[n - 2].half_edges.1;   // origin t_{N-2}
    let _top_mef = s.topo.mef(h_tn1, h_tn2);
    // bottom_mef.loop_ = side shell.  top_mef.loop_ = top polygon.

    // c) Side closures: n-1 mefs, each closing one side face.
    //    After bottom+top mefs, the active shell (bottom_mef.loop_) contains:
    //
    //    [top_mef.0(t_{N-1}), mev_t_fwd[n-2].1(t_{N-2}), ..., mev_t_fwd[0].1(t_0),
    //     mev_up.1(t_{N-1}), bottom_mef.1(b_{N-1}),
    //     mev_b0.0(b_0), mev_b[0].0(b_1), ..., mev_b[n-3].0(b_{N-2}), mev_up.0(b_{N-1})]
    //
    //    Side mef j (j in 0..n-2) connects t_j to b_j:
    //      h(t_j) = mev_t_fwd[j].half_edges.1    (origin t_j, in back chain)
    //      h(b_j):
    //        j=0: mev_b0.half_edges.0              (origin b_0, forward)
    //        j>=1: mev_b[j-1].half_edges.0         (origin b_j, forward)
    //    (Mirrors box_: mef(m5.1,m1.0), mef(m6.1,m2.0), mef(m3.0,m7.1) for N=4.)
    //
    //    The (N-1)th side face is the residual.

    for j in 0..n - 1 {
        let h_tj = mev_t_fwd[j].half_edges.1; // origin t_j
        let h_bj = if j == 0 {
            mev_b0.half_edges.0 // origin b_0
        } else {
            mev_b[j - 1].half_edges.0 // origin b_j
        };
        s.topo.mef(h_tj, h_bj);
    }

    validate(&s.topo).expect("extrude topology violates Euler invariant");

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

    // ---- Stage 5: Attach face geometry (planes). ----
    // Synthesize an upward direction from bottom-to-top centroid difference.
    let bot_centroid =
        bottom.iter().fold(Vec3::zeros(), |a, p| a + p.coords) / n as f64;
    let top_centroid = top.iter().fold(Vec3::zeros(), |a, p| a + p.coords) / n as f64;
    let dir_unit = (top_centroid - bot_centroid).normalize();
    let face_ids: Vec<_> = s.topo.face_ids().collect();
    for fid in face_ids {
        let frame = compute_face_frame(&s, fid, &dir_unit);
        s.face_geom.insert(fid, SurfaceKind::Plane(Plane::new(frame)));
    }

    s
}

fn compute_face_frame(s: &Solid, face: FaceId, _dir_unit: &Vec3) -> Frame {
    let poly = face_polygon(s, face).expect("face must have polygon");
    let n_poly = poly.len();
    debug_assert!(n_poly >= 3, "face must be at least a triangle");

    let p0 = poly[0];
    let p1 = poly[1];
    let p2 = poly[2];
    let mut normal = (p1 - p0).cross(&(p2 - p0));
    let nn = normal.norm();
    debug_assert!(nn > 1e-12, "degenerate face");
    normal /= nn;

    // Flip normal outward using face centroid vs prism centroid heuristic.
    let centroid_face =
        poly.iter().fold(Vec3::zeros(), |acc, p| acc + p.coords) / n_poly as f64;
    let mut prism_sum = Vec3::zeros();
    let mut count = 0_f64;
    for (_, p) in &s.vertex_geom {
        prism_sum += p.coords;
        count += 1.0;
    }
    let prism_centroid = prism_sum / count;
    let outward_check = centroid_face - prism_centroid;
    if normal.dot(&outward_check) < 0.0 {
        normal = -normal;
    }

    // Build right-handed frame: z = outward normal, x = arbitrary perpendicular.
    let seed = if normal.dot(&Vec3::x()).abs() < 0.9 {
        Vec3::x()
    } else {
        Vec3::y()
    };
    let x = (seed - normal * seed.dot(&normal)).normalize();
    let y = normal.cross(&x);
    Frame { origin: p0, x, y, z: normal }
}

#[cfg(test)]
mod tests {
    use super::*;
    use kerf_topo::validate;

    #[test]
    fn triangular_prism_has_correct_counts() {
        let profile = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.5, 1.0, 0.0),
        ];
        let s = extrude_polygon(&profile, Vec3::new(0.0, 0.0, 2.0));
        assert_eq!(s.vertex_count(), 6);  // 2N
        assert_eq!(s.edge_count(), 9);    // 3N
        assert_eq!(s.face_count(), 5);    // N+2
        validate(&s.topo).unwrap();
    }

    #[test]
    fn square_prism_matches_box_counts() {
        let profile = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ];
        let s = extrude_polygon(&profile, Vec3::new(0.0, 0.0, 1.0));
        assert_eq!(s.vertex_count(), 8);
        assert_eq!(s.edge_count(), 12);
        assert_eq!(s.face_count(), 6);
        validate(&s.topo).unwrap();
    }

    #[test]
    fn pentagon_prism_has_correct_counts() {
        use std::f64::consts::TAU;
        let n = 5;
        let profile: Vec<Point3> = (0..n)
            .map(|i| {
                let theta = i as f64 * TAU / n as f64;
                Point3::new(theta.cos(), theta.sin(), 0.0)
            })
            .collect();
        let s = extrude_polygon(&profile, Vec3::new(0.0, 0.0, 0.5));
        assert_eq!(s.vertex_count(), 10);
        assert_eq!(s.edge_count(), 15);
        assert_eq!(s.face_count(), 7);
        validate(&s.topo).unwrap();
    }

    #[test]
    fn extrude_vertex_positions_match_profile_and_top() {
        let profile = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(2.0, 0.0, 0.0),
            Point3::new(1.0, 1.5, 0.0),
        ];
        let direction = Vec3::new(0.0, 0.0, 3.0);
        let s = extrude_polygon(&profile, direction);
        let positions: Vec<Point3> = s.vertex_geom.iter().map(|(_, p)| *p).collect();
        for &p in &profile {
            assert!(
                positions.iter().any(|q| (p - q).norm() < 1e-12),
                "profile point {p:?} missing"
            );
            let t = p + direction;
            assert!(
                positions.iter().any(|q| (t - q).norm() < 1e-12),
                "top point {t:?} missing"
            );
        }
    }

    #[test]
    fn every_face_is_planar_every_edge_is_linear() {
        let profile = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ];
        let s = extrude_polygon(&profile, Vec3::new(0.0, 0.0, 1.0));
        for (_, surf) in &s.face_geom {
            assert!(matches!(surf, SurfaceKind::Plane(_)));
        }
        for (_, seg) in &s.edge_geom {
            assert!(matches!(seg.curve, crate::CurveKind::Line(_)));
        }
    }
}
