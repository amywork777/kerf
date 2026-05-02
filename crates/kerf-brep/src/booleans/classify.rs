//! Face classification: for each face in one solid, determine whether it lies
//! Inside, Outside, or OnBoundary of the other solid.

use kerf_geom::{Line, Point3, Tolerance, Vec3};
use kerf_topo::FaceId;

use crate::Solid;
use crate::booleans::face_polygon;
use crate::geometry::SurfaceKind;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FaceClassification {
    Inside,
    Outside,
    OnBoundary,
}

/// Compute the centroid of a face (signed-area-weighted, fjord-safe).
pub fn face_centroid(solid: &Solid, face: FaceId) -> Option<Point3> {
    let poly = face_polygon(solid, face)?;
    if poly.is_empty() {
        return None;
    }
    if poly.len() < 3 {
        let n = poly.len() as f64;
        let sum: Vec3 = poly.iter().map(|p| p.coords).sum();
        return Some(Point3::from(sum / n));
    }

    // M40: use the standard polygon-centroid formula (sum over directed edges)
    // on the UN-DEDUPED polygon walk. This formula handles fjord polygons
    // correctly: a stinger that goes out and returns along the same vertices
    // contributes zero net signed area (and zero centroid contribution), so
    // the result reflects the polygon's actual enclosed shape — including
    // annulus topology where an inner-ring fjord subtracts the hole's area
    // from the outer perimeter.
    //
    // Earlier (M39i) used dedup + fan-triangulation, which for annulus + stinger
    // fjord made the polygon non-simple after dedup (jumps across the deleted
    // duplicates) and produced a centroid that landed in the hole — yielding
    // mis-classification when the centroid is supposed to land on the annulus
    // body.
    if let Some(plane) = solid.face_geom.get(face)
        && let SurfaceKind::Plane(plane) = plane
    {
        let origin = plane.frame.origin;
        let fx = plane.frame.x;
        let fy = plane.frame.y;
        let to_2d = |p: Point3| -> (f64, f64) {
            let d = p - origin;
            (d.dot(&fx), d.dot(&fy))
        };
        let pts: Vec<(f64, f64)> = poly.iter().map(|p| to_2d(*p)).collect();
        let n = pts.len();
        let mut signed_area_x2 = 0.0f64;
        let mut cx = 0.0f64;
        let mut cy = 0.0f64;
        for i in 0..n {
            let (xi, yi) = pts[i];
            let (xj, yj) = pts[(i + 1) % n];
            let cross = xi * yj - xj * yi;
            signed_area_x2 += cross;
            cx += (xi + xj) * cross;
            cy += (yi + yj) * cross;
        }
        let signed_area = 0.5 * signed_area_x2;
        if signed_area.abs() > 1e-12 {
            let area6 = 6.0 * signed_area;
            let area_cx = cx / area6;
            let area_cy = cy / area6;
            // Verify centroid lands inside the polygon. For fjord polygons the
            // standard formula may still place the centroid on a stinger axis
            // (zero-width region) when the polygon's "body" is heavily
            // asymmetric — fall back in that case to the centroid of the
            // largest fan triangle, which is guaranteed to be inside.
            if point_in_polygon_2d(area_cx, area_cy, &pts) {
                return Some(origin + area_cx * fx + area_cy * fy);
            }
            // Fallback: try every fan triangle's centroid; pick the largest
            // whose centroid is verified inside the polygon body. For
            // L-shaped polygons (e.g., a rectangle with a triangular notch),
            // a fan triangle that straddles the notch may have its centroid
            // inside the notch (= outside the polygon body) — so we filter
            // by point-in-polygon.
            //
            // Try all polygon vertices as fan-origin until one yields a
            // valid in-body centroid. With ≥1 fan triangle inside the body
            // (which always exists for a non-degenerate polygon), at least
            // one base vertex's fan must produce an in-body centroid.
            for base in 0..n {
                let p0 = pts[base];
                let mut best: Option<(f64, f64, f64)> = None;
                for k in 1..n - 1 {
                    let i = (base + k) % n;
                    let j = (base + k + 1) % n;
                    let pi = pts[i];
                    let pj = pts[j];
                    let area = 0.5
                        * ((pi.0 - p0.0) * (pj.1 - p0.1)
                            - (pi.1 - p0.1) * (pj.0 - p0.0));
                    let centroid_x = (p0.0 + pi.0 + pj.0) / 3.0;
                    let centroid_y = (p0.1 + pi.1 + pj.1) / 3.0;
                    if !point_in_polygon_2d(centroid_x, centroid_y, &pts) {
                        continue;
                    }
                    if best.map_or(true, |(_, _, a)| area.abs() > a.abs()) {
                        best = Some((centroid_x, centroid_y, area));
                    }
                }
                if let Some((bx, by, _)) = best {
                    return Some(origin + bx * fx + by * fy);
                }
            }
        }
    }

    // Fallback: vertex average.
    let n = poly.len() as f64;
    let sum: Vec3 = poly.iter().map(|p| p.coords).sum();
    Some(Point3::from(sum / n))
}

/// Test whether a point lies on the boundary of `solid` (i.e., on any face plane
/// AND inside that face's polygon, within tolerance).
pub fn point_on_solid_boundary(solid: &Solid, p: Point3, tol: &Tolerance) -> Option<FaceId> {
    for face_id in solid.topo.face_ids() {
        let SurfaceKind::Plane(plane) = solid.face_geom.get(face_id)? else {
            continue;
        };
        // Distance from p to face plane.
        let d = (p - plane.frame.origin).dot(&plane.frame.z);
        if d.abs() > tol.point_eq {
            continue;
        }
        // p is on plane; check it's inside polygon.
        let poly = face_polygon(solid, face_id)?;
        if point_in_convex_polygon_2d(
            p,
            &poly,
            &plane.frame.x,
            &plane.frame.y,
            &plane.frame.origin,
            tol,
        ) {
            return Some(face_id);
        }
    }
    None
}

/// Crossing-number point-in-polygon test for a 2D polygon (works for both
/// convex and concave polygons, including self-intersecting/spike geometry).
fn point_in_polygon_2d(px: f64, py: f64, poly: &[(f64, f64)]) -> bool {
    let n = poly.len();
    if n < 3 {
        return false;
    }
    let mut inside = false;
    let mut j = n - 1;
    for i in 0..n {
        let (xi, yi) = poly[i];
        let (xj, yj) = poly[j];
        if (yi > py) != (yj > py) {
            let x_intercept = (xj - xi) * (py - yi) / (yj - yi) + xi;
            if px < x_intercept {
                inside = !inside;
            }
        }
        j = i;
    }
    inside
}

fn point_in_convex_polygon_2d(
    p: Point3,
    poly: &[Point3],
    fx: &Vec3,
    fy: &Vec3,
    origin: &Point3,
    tol: &Tolerance,
) -> bool {
    let to_2d = |q: Point3| -> (f64, f64) {
        let d = q - *origin;
        (d.dot(fx), d.dot(fy))
    };
    let (px, py) = to_2d(p);
    let n = poly.len();
    let mut sign: Option<f64> = None;
    for i in 0..n {
        let (ax, ay) = to_2d(poly[i]);
        let (bx, by) = to_2d(poly[(i + 1) % n]);
        // Cross product (b - a) x (p - a).
        let cross = (bx - ax) * (py - ay) - (by - ay) * (px - ax);
        if cross.abs() < tol.point_eq {
            continue;
        }
        match sign {
            None => sign = Some(cross.signum()),
            Some(s) => {
                if cross.signum() != s {
                    return false;
                }
            }
        }
    }
    true
}

/// Count how many times a ray (origin + t*dir, t > 0) crosses the boundary of `solid`.
/// Returns the count or `None` if a degeneracy was hit (ray exactly through a vertex
/// or edge); caller should perturb and retry.
pub fn ray_solid_crossings(
    solid: &Solid,
    origin: Point3,
    dir: Vec3,
    tol: &Tolerance,
) -> Option<usize> {
    let line = Line::from_origin_dir(origin, dir).unwrap();
    let mut count = 0;
    for face_id in solid.topo.face_ids() {
        let SurfaceKind::Plane(plane) = solid.face_geom.get(face_id)? else {
            continue;
        };
        let dn = dir.dot(&plane.frame.z);
        if dn.abs() < tol.angle_eq {
            continue;
        } // ray parallel to face plane

        // t at which line crosses face plane.
        let to_origin = plane.frame.origin - origin;
        let t = to_origin.dot(&plane.frame.z) / dn;
        if t < tol.point_eq {
            continue;
        } // crossing behind origin (or at it)

        // Point of crossing.
        let cross = origin + t * dir;
        let poly = face_polygon(solid, face_id)?;
        // 2D polygon containment using crossing-number test (handles non-convex
        // polygons — after Phase B/C splits the splitter face polygons are
        // generally non-convex due to fjord patterns).
        let to_2d = |q: Point3| -> (f64, f64) {
            let d = q - plane.frame.origin;
            (d.dot(&plane.frame.x), d.dot(&plane.frame.y))
        };
        let (cx, cy) = to_2d(cross);
        let poly_2d: Vec<(f64, f64)> = poly.iter().map(|p| to_2d(*p)).collect();
        if point_in_polygon_2d(cx, cy, &poly_2d) {
            // Reject if crossing exactly through polygon vertex or edge → return None for retry.
            if on_polygon_boundary(
                cross,
                &poly,
                &plane.frame.x,
                &plane.frame.y,
                &plane.frame.origin,
                tol,
            ) {
                return None;
            }
            count += 1;
        }
    }
    let _ = line; // silence
    Some(count)
}

fn on_polygon_boundary(
    p: Point3,
    poly: &[Point3],
    fx: &Vec3,
    fy: &Vec3,
    origin: &Point3,
    tol: &Tolerance,
) -> bool {
    let to_2d = |q: Point3| -> (f64, f64) {
        let d = q - *origin;
        (d.dot(fx), d.dot(fy))
    };
    let (px, py) = to_2d(p);
    let n = poly.len();
    for i in 0..n {
        let (ax, ay) = to_2d(poly[i]);
        let (bx, by) = to_2d(poly[(i + 1) % n]);
        // Distance from (px,py) to segment (a,b).
        let ex = bx - ax;
        let ey = by - ay;
        let len2 = ex * ex + ey * ey;
        if len2 < tol.point_eq * tol.point_eq {
            continue;
        }
        let s = ((px - ax) * ex + (py - ay) * ey) / len2;
        if !(0.0..=1.0).contains(&s) {
            continue;
        }
        let foot_x = ax + s * ex;
        let foot_y = ay + s * ey;
        let d2 = (px - foot_x).powi(2) + (py - foot_y).powi(2);
        if d2 < tol.point_eq * tol.point_eq {
            return true;
        }
    }
    false
}

/// Classify `face_a` (in `a`) against `b`: pick a centroid, ray-cast through `b`,
/// count crossings. If centroid is on b's boundary (within tol), report OnBoundary.
pub fn classify_face(a: &Solid, face_a: FaceId, b: &Solid, tol: &Tolerance) -> FaceClassification {
    let centroid = match face_centroid(a, face_a) {
        Some(c) => c,
        None => return FaceClassification::Outside,
    };

    // Boundary check first.
    if point_on_solid_boundary(b, centroid, tol).is_some() {
        return FaceClassification::OnBoundary;
    }

    // Try a ray in +x; if degenerate, try +y, then +z, then a slight perturbation.
    let dirs = [
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
        Vec3::new(0.7, 0.5, 0.3).normalize(),
        Vec3::new(0.31, -0.42, 0.85).normalize(),
    ];
    for &dir in &dirs {
        if let Some(count) = ray_solid_crossings(b, centroid, dir, tol) {
            return if count % 2 == 1 {
                FaceClassification::Inside
            } else {
                FaceClassification::Outside
            };
        }
    }
    // All rays were degenerate — extreme rare case. Default to Outside.
    FaceClassification::Outside
}

#[cfg(test)]
mod tests {
    use super::*;
    use kerf_geom::{Point3, Vec3};

    use crate::primitives::{box_, box_at};

    #[test]
    fn nested_small_box_is_fully_inside_big() {
        // Big [0,10]^3, small [4,6]^3. Every face of small has centroid inside big.
        let big = box_(Vec3::new(10.0, 10.0, 10.0));
        let small = box_at(Vec3::new(2.0, 2.0, 2.0), Point3::new(4.0, 4.0, 4.0));
        let tol = Tolerance::default();
        for face_id in small.topo.face_ids() {
            let cls = classify_face(&small, face_id, &big, &tol);
            assert_eq!(
                cls,
                FaceClassification::Inside,
                "face {face_id:?} expected Inside, got {cls:?}"
            );
        }
    }

    #[test]
    fn disjoint_boxes_classify_as_outside() {
        let a = box_(Vec3::new(1.0, 1.0, 1.0));
        let b = box_at(Vec3::new(1.0, 1.0, 1.0), Point3::new(5.0, 0.0, 0.0));
        let tol = Tolerance::default();
        for face_id in a.topo.face_ids() {
            let cls = classify_face(&a, face_id, &b, &tol);
            assert_eq!(cls, FaceClassification::Outside);
        }
    }

    /// M39 regression: corner-overlap union has L-shape post-split pieces
    /// whose vertex-average centroid lands on the OTHER solid's edge,
    /// mis-classifying them as OnBoundary. The area-weighted formula must
    /// produce a centroid strictly inside the L's body.
    #[test]
    fn area_weighted_centroid_lands_inside_l_shape() {
        // Build a solid whose only face is an L-shape on the x=1 plane,
        // simulating the post-split B.x=1 face from corner-overlap union.
        // L vertices (CCW from -x view): (1,2,1),(1,3,1),(1,3,3),(1,1,3),
        // (1,1,2),(1,2,2). Vertex-average centroid is (1,2,2) — exactly on
        // the inner-corner concavity. Area-weighted centroid lands strictly
        // inside the L (away from the corner).
        use crate::primitives::extrude_polygon;
        let poly = vec![
            Point3::new(2.0, 1.0, 1.0),
            Point3::new(3.0, 1.0, 1.0),
            Point3::new(3.0, 3.0, 1.0),
            Point3::new(1.0, 3.0, 1.0),
            Point3::new(1.0, 2.0, 1.0),
            Point3::new(2.0, 2.0, 1.0),
        ];
        let solid = extrude_polygon(&poly, Vec3::new(0.0, 0.0, 1.0));
        // Bottom face has the L shape. Find it via face_id iteration —
        // its centroid should NOT coincide with the polygon-average concavity.
        let mut found_l_face = false;
        for fid in solid.topo.face_ids() {
            let centroid = face_centroid(&solid, fid).unwrap();
            // Skip non-L faces (top, sides) by checking z == 1.0 (the L face).
            if (centroid.z - 1.0).abs() > 1e-6 {
                continue;
            }
            found_l_face = true;
            // Polygon-average concavity is at (2, 2, 1). Area-weighted
            // centroid must be measurably away from it (well inside the
            // L's body).
            let avg = Point3::new(2.0, 2.0, 1.0);
            let dist = (centroid - avg).norm();
            assert!(
                dist > 0.05,
                "L-shape centroid too close to polygon-average concavity: \
                 centroid={centroid:?}, avg={avg:?}, dist={dist}"
            );
        }
        assert!(found_l_face, "no L-shape face found in extrusion");
    }

    #[test]
    fn box_face_inside_other_box_is_inside() {
        // Big [0,10]^3 with small [4,6]^3 inside. Big's faces are far from small.
        // But a face of big lying entirely outside small → classify as Outside.
        let big = box_(Vec3::new(10.0, 10.0, 10.0));
        let small = box_at(Vec3::new(2.0, 2.0, 2.0), Point3::new(4.0, 4.0, 4.0));
        let tol = Tolerance::default();
        for face_id in big.topo.face_ids() {
            let cls = classify_face(&big, face_id, &small, &tol);
            assert_eq!(
                cls,
                FaceClassification::Outside,
                "big's faces should be outside small"
            );
        }
    }
}
