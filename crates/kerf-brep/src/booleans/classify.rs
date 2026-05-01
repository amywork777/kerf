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

/// Compute the centroid of a face (mean of outer-loop vertices).
pub fn face_centroid(solid: &Solid, face: FaceId) -> Option<Point3> {
    let poly = face_polygon(solid, face)?;
    if poly.is_empty() {
        return None;
    }
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
        // 2D polygon containment using the same helper.
        if point_in_convex_polygon_2d(
            cross,
            &poly,
            &plane.frame.x,
            &plane.frame.y,
            &plane.frame.origin,
            tol,
        ) {
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
