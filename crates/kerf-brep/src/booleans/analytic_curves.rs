//! Brep-level analytic curve intersections.
//!
//! This module returns *typed* analytic curve segments for the curved-surface
//! booleans roadmap. It wraps the closed-form intersection routines in
//! `kerf_geom::intersect` and lifts them into `EllipseSegment` and friends —
//! the brep-layer types that the topology stitcher will eventually consume.
//!
//! Today it's used by the analytic-curves prototyping path. Wiring it into
//! `face_intersections` is intentionally deferred: the existing stitch pipeline
//! assumes line-segment chords, and converting it to support arc chords is a
//! multi-week project (vertex coincidence, half-edge twin matching, and
//! interior classification all need the curve parameter rather than a 3D
//! direction).
//!
//! Scope of this module:
//!   * `cylinder_plane_intersection` — closed-form of `Cylinder ∩ Plane`,
//!     returning `CylinderPlaneIntersection`.
//!   * Tests covering all four geometric regimes (perpendicular → circle,
//!     parallel-tangent → line, parallel-secant → two lines, oblique →
//!     ellipse), plus degeneracy (parallel-disjoint → empty).
//!
//! Math reference (oblique case):
//!
//! ```text
//! A right circular cylinder with axis a and radius r, intersected with
//! a plane with unit normal n, produces an ellipse with:
//!     semi-minor = r        (perpendicular to the plane-axis dihedral)
//!     semi-major = r / |cos theta|   where theta = angle(a, n)
//! Equivalently semi-major = r / sin(alpha) where alpha = angle(a, plane).
//!
//! The center of the ellipse is the point where the cylinder axis pierces
//! the plane.
//! ```

use kerf_geom::intersect::{
    IntersectionComponent, SurfaceSurfaceIntersection, intersect_plane_cylinder,
};
use kerf_geom::{Cylinder, Line, Plane, Tolerance};

use crate::geometry::EllipseSegment;

/// Closed-form result of `Cylinder ∩ Plane`.
///
/// Variants match the four geometric regimes:
///   * `Empty` — plane is parallel to the cylinder axis but doesn't touch it.
///   * `Tangent(line)` — plane is parallel to the axis and tangent to the
///     cylinder (single ruling line).
///   * `TwoLines(l1, l2)` — plane is parallel to the axis and cuts the
///     cylinder in two ruling lines.
///   * `Circle(EllipseSegment)` — plane is perpendicular to the axis: the
///     intersection is a circle, returned as a degenerate ellipse (semi-major
///     == semi-minor) so downstream code has one curve type.
///   * `Ellipse(EllipseSegment)` — general oblique plane: the intersection is
///     a closed ellipse loop.
#[derive(Clone, Debug)]
pub enum CylinderPlaneIntersection {
    Empty,
    Tangent(Line),
    TwoLines(Line, Line),
    Circle(EllipseSegment),
    Ellipse(EllipseSegment),
}

/// Closed-form Cylinder × Plane intersection, lifted into brep-layer types.
///
/// For a finite cylinder you'd subsequently clip the returned curve against
/// the cylinder's height range; this function returns the unclipped infinite
/// intersection so callers can do their own segment clipping consistently
/// with how they treat planar chords.
pub fn cylinder_plane_intersection(
    cyl: &Cylinder,
    plane: &Plane,
    tol: &Tolerance,
) -> CylinderPlaneIntersection {
    use kerf_geom::Ellipse;

    match intersect_plane_cylinder(plane, cyl, tol) {
        SurfaceSurfaceIntersection::Empty => CylinderPlaneIntersection::Empty,
        SurfaceSurfaceIntersection::Coincident => {
            // A plane and a cylinder cannot be geometrically coincident; treat
            // defensively as empty rather than panicking.
            CylinderPlaneIntersection::Empty
        }
        SurfaceSurfaceIntersection::Components(comps) => match comps.len() {
            0 => CylinderPlaneIntersection::Empty,
            1 => match comps.into_iter().next().unwrap() {
                IntersectionComponent::Circle(c) => {
                    // Lift to an ellipse with semi_major == semi_minor == r.
                    // This keeps downstream consumers on a single curve kind
                    // when they want to walk the closed loop.
                    let e = Ellipse::new(c.frame, c.radius, c.radius);
                    CylinderPlaneIntersection::Circle(EllipseSegment::full(e))
                }
                IntersectionComponent::Ellipse(e) => {
                    CylinderPlaneIntersection::Ellipse(EllipseSegment::full(e))
                }
                IntersectionComponent::Line(l) => CylinderPlaneIntersection::Tangent(l),
                IntersectionComponent::Point(_) => CylinderPlaneIntersection::Empty,
            },
            2 => {
                let mut it = comps.into_iter();
                let a = it.next().unwrap();
                let b = it.next().unwrap();
                match (a, b) {
                    (IntersectionComponent::Line(l1), IntersectionComponent::Line(l2)) => {
                        CylinderPlaneIntersection::TwoLines(l1, l2)
                    }
                    _ => CylinderPlaneIntersection::Empty,
                }
            }
            _ => CylinderPlaneIntersection::Empty,
        },
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use kerf_geom::{Frame, Point3, Vec3};
    use std::f64::consts::SQRT_2;

    fn unit_cyl_z() -> Cylinder {
        Cylinder::new(Frame::world(Point3::origin()), 1.0)
    }

    /// Helper: a plane through `origin` with unit normal `n`. Builds the
    /// frame so `frame.z == n` exactly.
    fn plane_with_normal(origin: Point3, n: Vec3) -> Plane {
        let n = n.normalize();
        // Pick any vector not collinear with n for x-hint.
        let x_hint = if n.dot(&Vec3::x()).abs() < 0.9 {
            Vec3::x()
        } else {
            Vec3::y()
        };
        let x = (x_hint - n * x_hint.dot(&n)).normalize();
        let y = n.cross(&x).normalize();
        Plane::new(Frame { origin, x, y, z: n })
    }

    #[test]
    fn cylinder_plane_perp_returns_circle() {
        // Plane normal = +Z (parallel to cylinder axis). Intersection is a
        // unit circle at z=2, lifted to a degenerate-ellipse full segment.
        let cyl = unit_cyl_z();
        let plane = plane_with_normal(Point3::new(0.0, 0.0, 2.0), Vec3::z());
        let tol = Tolerance::default();
        match cylinder_plane_intersection(&cyl, &plane, &tol) {
            CylinderPlaneIntersection::Circle(seg) => {
                assert!(seg.is_full());
                assert_relative_eq!(seg.ellipse.semi_major, 1.0, epsilon = 1e-12);
                assert_relative_eq!(seg.ellipse.semi_minor, 1.0, epsilon = 1e-12);
                assert_relative_eq!(
                    seg.ellipse.frame.origin,
                    Point3::new(0.0, 0.0, 2.0),
                    epsilon = 1e-12
                );
            }
            other => panic!("expected Circle, got {other:?}"),
        }
    }

    #[test]
    fn cylinder_plane_parallel_returns_lines() {
        // Plane normal = +Y, plane through origin: the XZ plane.
        // A unit cylinder centered on Z axis is cut by the XZ plane in two
        // ruling lines at x = ±1.
        let cyl = unit_cyl_z();
        let plane = plane_with_normal(Point3::origin(), Vec3::y());
        let tol = Tolerance::default();
        match cylinder_plane_intersection(&cyl, &plane, &tol) {
            CylinderPlaneIntersection::TwoLines(l1, l2) => {
                let mut xs = [l1.origin.x, l2.origin.x];
                xs.sort_by(|a, b| a.partial_cmp(b).unwrap());
                assert_relative_eq!(xs[0], -1.0, epsilon = 1e-9);
                assert_relative_eq!(xs[1], 1.0, epsilon = 1e-9);
                // Both lines are parallel to the Z axis.
                for l in [l1, l2] {
                    assert_relative_eq!(l.direction.cross(&Vec3::z()).norm(), 0.0, epsilon = 1e-12);
                }
            }
            other => panic!("expected TwoLines, got {other:?}"),
        }
    }

    #[test]
    fn cylinder_plane_parallel_disjoint_is_empty() {
        // Plane parallel to axis but offset 5 units past the cylinder surface.
        let cyl = unit_cyl_z();
        let plane = plane_with_normal(Point3::new(0.0, 5.0, 0.0), Vec3::y());
        let tol = Tolerance::default();
        assert!(matches!(
            cylinder_plane_intersection(&cyl, &plane, &tol),
            CylinderPlaneIntersection::Empty
        ));
    }

    #[test]
    fn cylinder_plane_45deg_returns_ellipse_with_known_axes() {
        // Plane through origin with normal n = (1,0,1)/√2. The angle θ
        // between cylinder axis (Z) and the plane normal is 45°, so:
        //     semi-minor = r           = 1
        //     semi-major = r / cos(45°) = √2
        let cyl = unit_cyl_z();
        let n = Vec3::new(1.0, 0.0, 1.0).normalize();
        let plane = plane_with_normal(Point3::origin(), n);
        let tol = Tolerance::default();
        match cylinder_plane_intersection(&cyl, &plane, &tol) {
            CylinderPlaneIntersection::Ellipse(seg) => {
                assert!(seg.is_full());
                assert_relative_eq!(seg.ellipse.semi_minor, 1.0, epsilon = 1e-9);
                assert_relative_eq!(seg.ellipse.semi_major, SQRT_2, epsilon = 1e-9);
                // Center is where axis pierces plane: axis is z-line through
                // origin, plane passes through origin → center at origin.
                assert_relative_eq!(seg.ellipse.frame.origin, Point3::origin(), epsilon = 1e-9);
            }
            other => panic!("expected Ellipse, got {other:?}"),
        }
    }

    #[test]
    fn cylinder_plane_30deg_returns_ellipse_with_predicted_axes() {
        // Pick θ = 30°: normal n = (sin 30°, 0, cos 30°) = (1/2, 0, √3/2).
        // semi-minor = r = 1
        // semi-major = r / cos(30°) = 1 / (√3/2) = 2/√3.
        let cyl = unit_cyl_z();
        let theta = std::f64::consts::FRAC_PI_6; // 30°
        let n = Vec3::new(theta.sin(), 0.0, theta.cos()).normalize();
        let plane = plane_with_normal(Point3::origin(), n);
        let tol = Tolerance::default();
        match cylinder_plane_intersection(&cyl, &plane, &tol) {
            CylinderPlaneIntersection::Ellipse(seg) => {
                assert_relative_eq!(seg.ellipse.semi_minor, 1.0, epsilon = 1e-9);
                assert_relative_eq!(seg.ellipse.semi_major, 1.0 / theta.cos(), epsilon = 1e-9);
                // Sanity: every sampled point on the ellipse is exactly on the
                // cylinder surface (radial distance from Z axis == r) AND on
                // the plane (signed distance to plane == 0).
                for k in 0..16 {
                    let t = (k as f64) * std::f64::consts::TAU / 16.0;
                    let p = seg.point_at(t);
                    let radial = (p.x * p.x + p.y * p.y).sqrt();
                    assert_relative_eq!(radial, 1.0, epsilon = 1e-9);
                    let signed = (p - plane.frame.origin).dot(&n);
                    assert_relative_eq!(signed, 0.0, epsilon = 1e-9);
                }
            }
            other => panic!("expected Ellipse, got {other:?}"),
        }
    }

    #[test]
    fn cylinder_plane_tangent_returns_single_line() {
        // Plane parallel to Z axis (normal = +Y), offset by exactly the
        // cylinder radius (1.0). Intersection is the single ruling x=0, y=1.
        let cyl = unit_cyl_z();
        let plane = plane_with_normal(Point3::new(0.0, 1.0, 0.0), Vec3::y());
        let tol = Tolerance::default();
        match cylinder_plane_intersection(&cyl, &plane, &tol) {
            CylinderPlaneIntersection::Tangent(l) => {
                // Foot of the ruling: (0, 1, *) on the cylinder.
                assert_relative_eq!(l.origin.x, 0.0, epsilon = 1e-9);
                assert_relative_eq!(l.origin.y, 1.0, epsilon = 1e-9);
                // Direction is the cylinder axis.
                assert_relative_eq!(l.direction.cross(&Vec3::z()).norm(), 0.0, epsilon = 1e-12);
            }
            other => panic!("expected Tangent line, got {other:?}"),
        }
    }
}
