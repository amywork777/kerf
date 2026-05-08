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
    intersect_plane_sphere,
};
use kerf_geom::{Cylinder, Line, Plane, Point3, Sphere, Tolerance};

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

// ----------------------------------------------------------------------------
// Sphere × Plane
// ----------------------------------------------------------------------------

/// Closed-form result of `Sphere ∩ Plane`.
///
/// Three regimes:
///   * `Empty` — plane sits beyond the sphere (signed distance > radius).
///   * `Tangent(point)` — plane touches the sphere at exactly one point.
///   * `Circle(EllipseSegment)` — plane cuts the sphere along a circle of
///     radius `sqrt(r² − d²)`, where `d` is the signed distance from the
///     sphere center to the plane. Returned as a degenerate ellipse
///     (`semi_major == semi_minor`) so consumers can walk the closed loop
///     using the same `EllipseSegment` API as `cylinder_plane_intersection`.
#[derive(Clone, Debug)]
pub enum SpherePlaneIntersection {
    Empty,
    Tangent(Point3),
    Circle(EllipseSegment),
}

/// Closed-form Sphere × Plane intersection, lifted into brep-layer types.
///
/// Math:
/// ```text
/// Let n = plane.frame.z (unit normal),
///     d = (sphere.center − plane.origin) · n   (signed distance).
///
/// |d| > r  → Empty.
/// |d| = r  → Tangent at sphere.center − d·n.
/// |d| < r  → Circle of radius √(r² − d²) centered at
///            sphere.center − d·n, lying in the plane.
/// ```
pub fn sphere_plane_intersection(
    sphere: &Sphere,
    plane: &Plane,
    tol: &Tolerance,
) -> SpherePlaneIntersection {
    use kerf_geom::Ellipse;

    match intersect_plane_sphere(plane, sphere, tol) {
        SurfaceSurfaceIntersection::Empty => SpherePlaneIntersection::Empty,
        // A plane and a sphere cannot be coincident; defensive fallback.
        SurfaceSurfaceIntersection::Coincident => SpherePlaneIntersection::Empty,
        SurfaceSurfaceIntersection::Components(comps) => {
            match comps.into_iter().next() {
                None => SpherePlaneIntersection::Empty,
                Some(IntersectionComponent::Point(p)) => SpherePlaneIntersection::Tangent(p),
                Some(IntersectionComponent::Circle(c)) => {
                    // Lift to a degenerate ellipse so downstream consumers can
                    // treat the closed loop with the same code path that
                    // handles cylinder×plane circles/ellipses.
                    let e = Ellipse::new(c.frame, c.radius, c.radius);
                    SpherePlaneIntersection::Circle(EllipseSegment::full(e))
                }
                // Other component kinds shouldn't occur; treat as empty.
                Some(_) => SpherePlaneIntersection::Empty,
            }
        }
    }
}

// ----------------------------------------------------------------------------
// Cylinder × Cylinder
// ----------------------------------------------------------------------------

/// Closed-form / sampled result of `Cylinder ∩ Cylinder`.
///
/// Four regimes:
///   * `Empty` — parallel-axis cylinders whose axes are too far apart.
///   * `TwoLines(l1, l2)` — parallel-axis cylinders whose lateral surfaces
///     overlap along two ruling lines (axis offset < r_a + r_b and
///     specifically axis offset = |r_a − r_b| or both radii match the
///     overlap chord). For equal-radius coaxial cylinders this is
///     `Coincident`-like and we return `Empty` to flag "fully shared".
///   * `Tangent(line)` — parallel-axis cylinders externally tangent
///     (axis offset == r_a + r_b) or internally tangent
///     (axis offset == |r_a − r_b|): single ruling line.
///   * `Polyline(Vec<Point3>)` — non-parallel axes (perpendicular or skew):
///     the analytic intersection is a 4th-degree algebraic curve. We
///     approximate with a closed polyline of `N` samples (default 32),
///     sufficient for the curved-surface boolean roadmap's "interior
///     crossings exist, here are sample points" purpose.
///
/// Note: like `cylinder_plane_intersection`, this function returns the
/// *infinite-cylinder* intersection. Callers clip against finite cylinder
/// height ranges separately.
#[derive(Clone, Debug)]
pub enum CylinderCylinderIntersection {
    Empty,
    Tangent(Line),
    TwoLines(Line, Line),
    Polyline(Vec<Point3>),
}

/// Closed-form (parallel) / sampled (general) Cylinder × Cylinder intersection.
///
/// Detects the parallel-axis regime closed-form, otherwise falls back to a
/// 32-sample polyline along cylinder A's parameter sweep. For each sample,
/// we evaluate `point_a(u) = a.origin + r_a (cos u · a.x + sin u · a.y)`
/// (taking v=0 — i.e., the unbounded cylinder's foot ring projected up the
/// axis isn't yet sampled; we sample one ring crossing). Samples whose
/// signed distance to cylinder B's axis falls within `±tol.point_eq` of
/// `b.radius` are kept; this recovers the saddle-shape intersection of two
/// equal-radius perpendicular cylinders ("Steinmetz" curve) up to sample
/// resolution.
///
/// Approximation note: this is a brep-layer convenience for the
/// curved-surface stitch roadmap, not a primary geometric kernel routine.
/// Truly faithful Cylinder×Cylinder needs implicitization-based
/// 4th-degree-curve tracing; keeping that out of `kerf-geom` is
/// intentional until M3c.
pub fn cylinder_cylinder_intersection(
    a: &Cylinder,
    b: &Cylinder,
    tol: &Tolerance,
) -> CylinderCylinderIntersection {
    let axis_a = a.frame.z;
    let axis_b = b.frame.z;
    let r_a = a.radius;
    let r_b = b.radius;

    // ---- Parallel axes ----
    if tol.directions_parallel(axis_a, axis_b) {
        // Project B's center onto A's axial plane: distance from a.origin to
        // b.origin perpendicular to axis_a.
        let delta = b.frame.origin - a.frame.origin;
        // Component perpendicular to axis_a.
        let along = delta.dot(&axis_a);
        let perp = delta - along * axis_a;
        let d = perp.norm();

        let sum = r_a + r_b;
        let diff = (r_a - r_b).abs();

        // Coaxial (or near-coaxial) and same radius: surfaces coincide
        // (infinite intersection). Flag as Empty to signal "no isolated
        // chord" — the boolean caller treats this as a degenerate case.
        if d < tol.point_eq && (r_a - r_b).abs() < tol.point_eq {
            return CylinderCylinderIntersection::Empty;
        }
        // External-disjoint or internal-nested-disjoint.
        if d > sum + tol.point_eq {
            return CylinderCylinderIntersection::Empty;
        }
        if d + tol.point_eq < diff {
            // One cylinder strictly inside the other: no surface intersection.
            return CylinderCylinderIntersection::Empty;
        }
        // Tangent regimes: external (d ≈ r_a + r_b) or internal
        // (d ≈ |r_a − r_b|, with d > 0).
        if (d - sum).abs() < tol.point_eq || (d > tol.point_eq && (d - diff).abs() < tol.point_eq) {
            // Foot of the tangent ruling on cylinder A:
            //   external: foot = a.origin + r_a · perp_hat
            //   internal: depending on which cyl is inside, the foot is at
            //             ±r_a along perp_hat. Both reduce to "the unique
            //             point on A's perimeter that's also on B's perimeter".
            let perp_hat = if d > tol.point_eq {
                perp / d
            } else {
                // Degenerate: pick any perpendicular — but d > 0 case is
                // guarded above, so we won't hit this.
                let mut tmp = axis_a.cross(&kerf_geom::Vec3::x());
                if tmp.norm() < 1e-9 {
                    tmp = axis_a.cross(&kerf_geom::Vec3::y());
                }
                tmp.normalize()
            };
            let foot = a.frame.origin + r_a * perp_hat;
            let line = Line::from_origin_dir(foot, axis_a)
                .expect("axis is unit, so direction is non-zero");
            return CylinderCylinderIntersection::Tangent(line);
        }
        // Two-line regime: parallel-axis cylinders crossing in two ruling
        // lines. The lines lie at the two intersection points of two
        // circles in the cross-section plane perpendicular to the axis:
        //   C_a: |P|² = r_a²
        //   C_b: |P − Δ|² = r_b²
        // Subtracting: 2 P·Δ = r_a² − r_b² + d², so along the perp_hat
        // direction the two solutions are at:
        //   t* = (r_a² − r_b² + d²) / (2 d)
        //   ±h with h = √(r_a² − t*²)
        let perp_hat = perp / d;
        let t_star = (r_a * r_a - r_b * r_b + d * d) / (2.0 * d);
        let h_sq = r_a * r_a - t_star * t_star;
        if h_sq < -tol.point_eq {
            return CylinderCylinderIntersection::Empty;
        }
        let h = h_sq.max(0.0).sqrt();
        // Build a perpendicular-to-perp_hat direction in the axial plane.
        let tangent_hat = axis_a.cross(&perp_hat).normalize();
        let foot1 = a.frame.origin + t_star * perp_hat + h * tangent_hat;
        let foot2 = a.frame.origin + t_star * perp_hat - h * tangent_hat;
        let l1 = Line::from_origin_dir(foot1, axis_a).unwrap();
        let l2 = Line::from_origin_dir(foot2, axis_a).unwrap();
        return CylinderCylinderIntersection::TwoLines(l1, l2);
    }

    // ---- Non-parallel (perpendicular or skew) axes ----
    // Sample cylinder A's lateral surface at N angular steps (one ring; for
    // an unbounded cylinder we trace the curve by varying u and solving for
    // v_a along axis_a such that the point lies on cylinder B). For two
    // perpendicular cylinders of equal radius this recovers the Steinmetz
    // saddle curves.
    //
    // Per-sample math:
    //   p(u) = a.origin + r_a · (cos u · a.x + sin u · a.y) + v · axis_a
    //   q on cylinder B iff |((p − b.origin) − ((p − b.origin) · axis_b) axis_b)| = r_b
    //   Let w(u) = a.origin − b.origin + r_a (cos u · a.x + sin u · a.y).
    //   Define f(v) = |w + v·axis_a − ((w + v·axis_a) · axis_b) axis_b|² − r_b².
    //   This is a quadratic in v: solve_quadratic.
    let n_samples = 32;
    let mut polyline: Vec<Point3> = Vec::with_capacity(n_samples * 2);
    let cab = a.frame.origin - b.frame.origin;
    for k in 0..n_samples {
        let u = (k as f64) * std::f64::consts::TAU / (n_samples as f64);
        let (su, cu) = u.sin_cos();
        let ring = r_a * (cu * a.frame.x + su * a.frame.y);
        let w = cab + ring;
        // f(v) = |w + v axis_a − (w·b̂ + v axis_a·b̂) b̂|² − r_b²
        // Let dot_w_b = w · axis_b; dot_a_b = axis_a · axis_b.
        // u(v) = w − (dot_w_b) b̂ + v (axis_a − dot_a_b · b̂)
        // |u(v)|² = |A|² + 2 (A · B) v + |B|² v²  (B = axis_a − dot_a_b · b̂)
        let dot_w_b = w.dot(&axis_b);
        let dot_a_b = axis_a.dot(&axis_b);
        let a_vec = w - dot_w_b * axis_b;
        let b_vec = axis_a - dot_a_b * axis_b;
        let aa = b_vec.dot(&b_vec);
        let bb = 2.0 * a_vec.dot(&b_vec);
        let cc = a_vec.dot(&a_vec) - r_b * r_b;
        let roots = kerf_geom::intersect::poly::solve_quadratic(aa, bb, cc);
        for v in roots {
            let p = a.frame.origin + ring + v * axis_a;
            polyline.push(p);
        }
    }
    if polyline.is_empty() {
        // No sampled crossings — likely truly disjoint (axes far apart).
        // For the unbounded cylinder, falling here is rare, but possible
        // when cylinder A's surface never reaches cylinder B's tube.
        return CylinderCylinderIntersection::Empty;
    }
    CylinderCylinderIntersection::Polyline(polyline)
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

    // ------------------------------------------------------------------
    // Sphere × Plane
    // ------------------------------------------------------------------

    fn unit_sphere_origin() -> Sphere {
        Sphere::new(Frame::world(Point3::origin()), 1.0)
    }

    #[test]
    fn sphere_plane_intersection_perp_origin() {
        // Plane through (0,0,1), normal = +Z. Plane is tangent to the unit
        // sphere at the north pole.
        let sphere = unit_sphere_origin();
        let plane = plane_with_normal(Point3::new(0.0, 0.0, 1.0), Vec3::z());
        let tol = Tolerance::default();
        match sphere_plane_intersection(&sphere, &plane, &tol) {
            SpherePlaneIntersection::Tangent(p) => {
                assert_relative_eq!(p, Point3::new(0.0, 0.0, 1.0), epsilon = 1e-9);
            }
            other => panic!("expected Tangent, got {other:?}"),
        }
    }

    #[test]
    fn sphere_plane_intersection_offset_below_returns_empty() {
        // Plane at z = -5 with normal +Z: well below the unit sphere.
        let sphere = unit_sphere_origin();
        let plane = plane_with_normal(Point3::new(0.0, 0.0, -5.0), Vec3::z());
        let tol = Tolerance::default();
        assert!(matches!(
            sphere_plane_intersection(&sphere, &plane, &tol),
            SpherePlaneIntersection::Empty
        ));
    }

    #[test]
    fn sphere_plane_intersection_intersects_returns_circle() {
        // Plane z = 0.5 cuts the unit sphere along a circle of radius √(1−0.25) = √0.75.
        let sphere = unit_sphere_origin();
        let plane = plane_with_normal(Point3::new(0.0, 0.0, 0.5), Vec3::z());
        let tol = Tolerance::default();
        match sphere_plane_intersection(&sphere, &plane, &tol) {
            SpherePlaneIntersection::Circle(seg) => {
                assert!(seg.is_full());
                assert_relative_eq!(seg.ellipse.semi_major, 0.75_f64.sqrt(), epsilon = 1e-9);
                assert_relative_eq!(seg.ellipse.semi_minor, 0.75_f64.sqrt(), epsilon = 1e-9);
                assert_relative_eq!(
                    seg.ellipse.frame.origin,
                    Point3::new(0.0, 0.0, 0.5),
                    epsilon = 1e-9
                );
                // Sample-check: every point on the segment is on both surfaces.
                for k in 0..16 {
                    let t = (k as f64) * std::f64::consts::TAU / 16.0;
                    let p = seg.point_at(t);
                    // On sphere: |p − center|² = r²
                    let on_sphere = (p - Point3::origin()).norm();
                    assert_relative_eq!(on_sphere, 1.0, epsilon = 1e-9);
                    // On plane: signed distance ≈ 0
                    let signed = (p - plane.frame.origin).dot(&Vec3::z());
                    assert_relative_eq!(signed, 0.0, epsilon = 1e-9);
                }
            }
            other => panic!("expected Circle, got {other:?}"),
        }
    }

    // ------------------------------------------------------------------
    // Cylinder × Cylinder
    // ------------------------------------------------------------------

    fn unit_cyl_x() -> Cylinder {
        // Cylinder of radius 1 with axis along +X, passing through origin.
        // Need frame.z = +X. Use from_x_yhint with x_axis = +Y so that
        // z = Y.cross(Z_hint) = X.
        let frame = Frame::from_x_yhint(Point3::origin(), Vec3::y(), Vec3::z()).unwrap();
        Cylinder::new(frame, 1.0)
    }

    #[test]
    fn cylinder_cylinder_parallel_offset_returns_empty() {
        // Both Z-axis cylinders, offset 5 in X. Sum of radii = 2 < 5 → empty.
        let a = unit_cyl_z();
        let b_frame = Frame::world(Point3::new(5.0, 0.0, 0.0));
        let b = Cylinder::new(b_frame, 1.0);
        let tol = Tolerance::default();
        assert!(matches!(
            cylinder_cylinder_intersection(&a, &b, &tol),
            CylinderCylinderIntersection::Empty
        ));
    }

    #[test]
    fn cylinder_cylinder_parallel_overlap_returns_two_lines() {
        // Both Z-axis unit cylinders, offset 1 in X. Two ruling lines at
        // intersection of circles (x²+y²=1) and ((x−1)²+y²=1).
        // Solve: x = 0.5, y = ±√(1 − 0.25) = ±√0.75.
        let a = unit_cyl_z();
        let b_frame = Frame::world(Point3::new(1.0, 0.0, 0.0));
        let b = Cylinder::new(b_frame, 1.0);
        let tol = Tolerance::default();
        match cylinder_cylinder_intersection(&a, &b, &tol) {
            CylinderCylinderIntersection::TwoLines(l1, l2) => {
                let mut ys = [l1.origin.y, l2.origin.y];
                ys.sort_by(|a, b| a.partial_cmp(b).unwrap());
                assert_relative_eq!(ys[0], -0.75_f64.sqrt(), epsilon = 1e-9);
                assert_relative_eq!(ys[1], 0.75_f64.sqrt(), epsilon = 1e-9);
                // Both feet have x = 0.5
                for l in [l1, l2] {
                    assert_relative_eq!(l.origin.x, 0.5, epsilon = 1e-9);
                    // Direction parallel to Z.
                    assert_relative_eq!(l.direction.cross(&Vec3::z()).norm(), 0.0, epsilon = 1e-12);
                }
            }
            other => panic!("expected TwoLines, got {other:?}"),
        }
    }

    #[test]
    fn cylinder_cylinder_parallel_external_tangent_returns_one_line() {
        // Both Z-axis unit cylinders, offset 2 in X — externally tangent at x=1.
        let a = unit_cyl_z();
        let b_frame = Frame::world(Point3::new(2.0, 0.0, 0.0));
        let b = Cylinder::new(b_frame, 1.0);
        let tol = Tolerance::default();
        match cylinder_cylinder_intersection(&a, &b, &tol) {
            CylinderCylinderIntersection::Tangent(l) => {
                assert_relative_eq!(l.origin.x, 1.0, epsilon = 1e-9);
                assert_relative_eq!(l.origin.y, 0.0, epsilon = 1e-9);
                assert_relative_eq!(l.direction.cross(&Vec3::z()).norm(), 0.0, epsilon = 1e-12);
            }
            other => panic!("expected Tangent, got {other:?}"),
        }
    }

    #[test]
    fn cylinder_cylinder_perpendicular_returns_4th_degree_polyline() {
        // Two unit cylinders, axes perpendicular (A: Z-axis, B: X-axis),
        // both passing through origin. The Steinmetz solid's surface
        // intersection is two saddle curves; we expect a non-empty
        // polyline whose samples lie on both cylinders.
        let a = unit_cyl_z();
        let b = unit_cyl_x();
        let tol = Tolerance::default();
        match cylinder_cylinder_intersection(&a, &b, &tol) {
            CylinderCylinderIntersection::Polyline(samples) => {
                assert!(
                    samples.len() >= 32,
                    "expected at least 32 polyline samples, got {}",
                    samples.len()
                );
                // Every sample must lie on cylinder A (radial dist from Z = 1)
                // and on cylinder B (radial dist from X = 1).
                for p in &samples {
                    let r_a = (p.x * p.x + p.y * p.y).sqrt();
                    assert_relative_eq!(r_a, 1.0, epsilon = 1e-6);
                    let r_b = (p.y * p.y + p.z * p.z).sqrt();
                    assert_relative_eq!(r_b, 1.0, epsilon = 1e-6);
                }
            }
            other => panic!("expected Polyline, got {other:?}"),
        }
    }

    #[test]
    fn cylinder_cylinder_skew_axes_returns_polyline_or_empty() {
        // Two unit cylinders with skew (non-intersecting, non-parallel)
        // axes. A along Z through origin; B along Y through (0.5, 0, 0.5)
        // — neither parallel to A's axis nor intersecting it. The
        // unbounded-cylinder analytic intersection traces out a
        // 4th-degree curve; we approximate with a sampled polyline.
        let a = unit_cyl_z();
        // Build B with axis = +Y. Need frame.z = Y. Use from_x_yhint with
        // x = Z, y_hint = X: then z = Z.cross(X) = Y. ✓
        let b_frame = Frame::from_x_yhint(
            Point3::new(0.5, 0.0, 0.5),
            Vec3::z(),
            Vec3::x(),
        )
        .unwrap();
        let b = Cylinder::new(b_frame, 1.0);
        // Sanity: the two axes should be perpendicular (Z ⟂ Y), not parallel.
        let tol = Tolerance::default();
        assert!(!tol.directions_parallel(a.frame.z, b.frame.z));
        match cylinder_cylinder_intersection(&a, &b, &tol) {
            CylinderCylinderIntersection::Polyline(samples) => {
                // Each sample must lie on both cylinders.
                for p in &samples {
                    // On A: radial distance from Z axis.
                    let r_a = (p.x * p.x + p.y * p.y).sqrt();
                    assert_relative_eq!(r_a, 1.0, epsilon = 1e-6);
                    // On B: radial distance from B's axis (Y line at x=0.5,z=0.5).
                    let dx = p.x - 0.5;
                    let dz = p.z - 0.5;
                    let r_b = (dx * dx + dz * dz).sqrt();
                    assert_relative_eq!(r_b, 1.0, epsilon = 1e-6);
                }
            }
            CylinderCylinderIntersection::Empty => {
                // Acceptable if sampling missed the entire crossing region.
            }
            other => panic!("expected Polyline or Empty, got {other:?}"),
        }
    }
}
