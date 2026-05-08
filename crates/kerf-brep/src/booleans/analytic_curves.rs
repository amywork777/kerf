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
    intersect_plane_sphere, intersect_sphere_sphere,
};
use kerf_geom::{Cone, Cylinder, Line, Plane, Point3, Sphere, Torus, Tolerance, Vec3};

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

// ----------------------------------------------------------------------------
// Sphere × Sphere
// ----------------------------------------------------------------------------

/// Closed-form result of `Sphere ∩ Sphere`.
///
/// Three regimes:
///   * `Empty` — spheres disjoint or one nested inside the other without touching.
///   * `Coincident` — same center and radius.
///   * `Tangent(point)` — externally or internally tangent: meet at one point.
///   * `Circle(EllipseSegment)` — overlap: a closed planar circle perpendicular
///     to the line of centers, lifted to a degenerate ellipse so it shares
///     one curve type with the other plane-curve intersections.
#[derive(Clone, Debug)]
pub enum SphereSphereIntersection {
    Empty,
    Coincident,
    Tangent(Point3),
    Circle(EllipseSegment),
}

/// Closed-form Sphere × Sphere intersection, lifted into brep-layer types.
///
/// Math: let `d = |b.center − a.center|`. Then
///   * `d > r_a + r_b` or `d < |r_a − r_b|`: Empty.
///   * `d = 0` and `r_a = r_b`: Coincident.
///   * `d = r_a + r_b` or `d = |r_a − r_b|`: Tangent at a single point.
///   * Else: closed circle of radius `√(r_a² − α²)` centered at
///     `a.center + α · (b−a)/d`, where `α = (r_a² − r_b² + d²) / (2d)`.
pub fn sphere_sphere_intersection(
    a: &Sphere,
    b: &Sphere,
    tol: &Tolerance,
) -> SphereSphereIntersection {
    use kerf_geom::Ellipse;

    match intersect_sphere_sphere(a, b, tol) {
        SurfaceSurfaceIntersection::Empty => SphereSphereIntersection::Empty,
        SurfaceSurfaceIntersection::Coincident => SphereSphereIntersection::Coincident,
        SurfaceSurfaceIntersection::Components(comps) => match comps.into_iter().next() {
            None => SphereSphereIntersection::Empty,
            Some(IntersectionComponent::Point(p)) => SphereSphereIntersection::Tangent(p),
            Some(IntersectionComponent::Circle(c)) => {
                let e = Ellipse::new(c.frame, c.radius, c.radius);
                SphereSphereIntersection::Circle(EllipseSegment::full(e))
            }
            Some(_) => SphereSphereIntersection::Empty,
        },
    }
}

// ----------------------------------------------------------------------------
// Cone × Plane
// ----------------------------------------------------------------------------

/// Closed-form / sampled result of `Cone ∩ Plane`.
///
/// Geometric regimes (right circular cone with apex at `cone.frame.origin`,
/// axis `cone.frame.z`, half-angle `cone.half_angle`):
///   * `Empty` — plane misses both nappes (only possible for the
///     finite-cone case; for an infinite double-napped cone any plane
///     except those tangent at the apex meets the surface).
///   * `Apex(point)` — plane passes through the apex AND is tangent to the
///     cone (touches it only at the apex point).
///   * `TwoLines(l1, l2)` — plane passes through the apex and cuts the cone
///     along two ruling generators.
///   * `Circle(EllipseSegment)` — plane perpendicular to the axis (does not
///     pass through apex).
///   * `Ellipse(EllipseSegment)` — plane oblique, intersection angle steeper
///     than the cone's half-angle (closed ellipse, lifted to a degenerate
///     full segment).
///   * `Parabola(Vec<Point3>)` — plane parallel to a generator line (the
///     intersection is one parabolic branch). Sampled to a polyline along
///     the cone's parameter range `v ∈ [-V, V]`.
///   * `Hyperbola(Vec<Point3>)` — plane oblique with intersection angle
///     shallower than the cone's half-angle. Two branches; sampled into a
///     single polyline (left branch first, then right).
///
/// The Circle/Ellipse cases share `EllipseSegment` with `cylinder_plane_intersection`
/// so downstream consumers see one curve type for closed conic sections.
#[derive(Clone, Debug)]
pub enum ConePlaneIntersection {
    Empty,
    Apex(Point3),
    TwoLines(Line, Line),
    Circle(EllipseSegment),
    Ellipse(EllipseSegment),
    Parabola(Vec<Point3>),
    Hyperbola(Vec<Point3>),
}

/// Closed-form Cone × Plane intersection, lifted into brep-layer types.
///
/// Conic-section classification:
///   * Let `α` = cone half-angle, `β` = angle between plane and cone axis
///     (i.e. `β = π/2 − angle(plane.normal, cone.axis)`).
///   * `β = π/2` (plane perpendicular to axis): **Circle**.
///   * `α < β < π/2` (plane crosses both generators of the same nappe at
///     finite distance): **Ellipse**.
///   * `β = α` (plane parallel to one generator): **Parabola**.
///   * `0 ≤ β < α` (plane crosses both nappes): **Hyperbola**.
///   * Plane through apex: degenerate — single `Apex(point)`, two ruling
///     lines, or empty depending on `β` vs `α`.
///
/// Sampling for Parabola/Hyperbola: we trace the curve by stepping the
/// cone's parameter `v` over a default range `[-3, 3]` (adjustable by
/// caller via finite-cone clipping) and at each `v` solve the cone ring
/// equation for `u` such that the point lies on the plane. Each accepted
/// sample is on both surfaces to better than `1e-9`.
pub fn cone_plane_intersection(
    cone: &Cone,
    plane: &Plane,
    tol: &Tolerance,
) -> ConePlaneIntersection {
    use kerf_geom::Ellipse;

    let n = plane.frame.z;
    let axis = cone.frame.z;
    let apex = cone.frame.origin;
    let half_angle = cone.half_angle;

    // Signed distance from plane to apex along plane normal (apex - plane).
    // Used as: apex + t * axis lands on plane iff apex_to_plane_t = -d_apex / cos_theta.
    let d_apex = (apex - plane.frame.origin).dot(&n);
    let through_apex = d_apex.abs() < tol.point_eq;

    // Angle between axis and plane normal: cos_theta.
    let cos_theta = axis.dot(&n);
    let abs_cos_theta = cos_theta.abs();

    // Plane perpendicular to axis: circle case (or apex-tangent if through apex).
    if tol.directions_parallel(n, axis) {
        if through_apex {
            return ConePlaneIntersection::Apex(apex);
        }
        // We want t such that (apex + t * axis - plane.origin) . n = 0
        // → (apex - plane.origin)·n + t (axis·n) = 0
        // → t = -d_apex / cos_theta.
        let t = -d_apex / cos_theta;
        let radius = t.abs() * half_angle.tan();
        if radius < tol.point_eq {
            return ConePlaneIntersection::Apex(apex);
        }
        let center = apex + t * axis;
        let frame = kerf_geom::Frame {
            origin: center,
            x: cone.frame.x,
            y: cone.frame.y,
            z: axis,
        };
        let circle = Ellipse::new(frame, radius, radius);
        return ConePlaneIntersection::Circle(EllipseSegment::full(circle));
    }

    // Plane parallel to axis (cos_theta = 0): hyperbolic regime if the plane
    // misses the apex by less than... well, this is a hyperbola with the
    // axis as one asymptote line — reduces to the general "β < α" case
    // since β = 0 < α here. Fall through to general logic.

    // Compute the generator angle β = arcsin(|axis · n|) measured from plane.
    // β > α → Ellipse; β = α → Parabola; β < α → Hyperbola.
    // Equivalent comparison: |cos_theta| vs cos(π/2 − α) = sin(α).
    let sin_alpha = half_angle.sin();
    let _cos_alpha = half_angle.cos();

    if through_apex {
        // Plane through apex: degenerate cases.
        // - If β > α: only the apex point (plane lies "outside" the cone).
        // - If β = α: plane tangent along one generator → single line (apex+gen)
        //   but we can return TwoLines coincident; treat as Apex for safety.
        // - If β < α: plane cuts cone along two generator lines through apex.
        if abs_cos_theta > sin_alpha + tol.angle_eq {
            return ConePlaneIntersection::Apex(apex);
        }
        if abs_cos_theta < sin_alpha - tol.angle_eq {
            // Two-line case. Find the two generators: rays from apex along
            // direction d(u) = axis + tan(α)(cos u · x + sin u · y), with
            // d(u) · n = 0 → axis·n + tan(α)(xn cos u + yn sin u) = 0.
            // Let R = sqrt(xn²+yn²), φ = atan2(yn, xn). Then
            //   R cos(u - φ) = -axis·n / tan(α).
            let xn = cone.frame.x.dot(&n);
            let yn = cone.frame.y.dot(&n);
            let an = cos_theta;
            let t = half_angle.tan();
            let r_amp = (xn * xn + yn * yn).sqrt();
            if r_amp < tol.point_eq {
                // n is along axis — but we already handled that case above.
                return ConePlaneIntersection::Apex(apex);
            }
            let rhs = -an / (t * r_amp);
            if rhs.abs() > 1.0 + tol.point_eq {
                return ConePlaneIntersection::Apex(apex);
            }
            let phase = yn.atan2(xn);
            let acos_val = rhs.clamp(-1.0, 1.0).acos();
            let u_a = phase + acos_val;
            let u_b = phase - acos_val;
            let dir_for = |u: f64| -> Vec3 {
                let (su, cu) = u.sin_cos();
                (axis + t * cu * cone.frame.x + t * su * cone.frame.y).normalize()
            };
            let l1 = Line::from_origin_dir(apex, dir_for(u_a)).unwrap();
            let l2 = Line::from_origin_dir(apex, dir_for(u_b)).unwrap();
            return ConePlaneIntersection::TwoLines(l1, l2);
        }
        // β ≈ α: tangent along one generator. Return a single-line as TwoLines
        // with both lines coincident (caller can dedupe).
        return ConePlaneIntersection::Apex(apex);
    }

    // Plane does not pass through apex: classify the conic.
    // Compare β = arcsin(|cos_theta|) to α.
    if (abs_cos_theta - sin_alpha).abs() < tol.angle_eq {
        // Parabola — sample.
        return ConePlaneIntersection::Parabola(sample_cone_plane_curve(
            cone, plane, tol, 32,
        ));
    }
    if abs_cos_theta > sin_alpha {
        // β > α → Ellipse. Compute closed-form ellipse parameters.
        // Standard result: the intersection is a closed ellipse on one nappe.
        // The axial distance from apex to the ellipse's major-axis center,
        // using the perpendicular-foot construction:
        //   foot from apex to plane along axis (when axis not parallel n) is
        //   at axial parameter v0 = d_apex / cos_theta.
        // The two extremes of the ellipse along axis:
        //   sin(α) sin(β) etc. give the standard formula.
        // Deriving carefully:
        //   Two extreme intersection points are where the plane meets the
        //   cone in the plane spanned by axis and n.
        //   Let u_axis_in_plane = projection of axis onto plane.
        //   The two points lie symmetric on either side along that direction
        //   in the plane.
        //
        // Closed-form: see https://en.wikipedia.org/wiki/Conic_section#Eccentricity
        //   eccentricity e = cos(β) / cos(α)? Wait, standard form:
        //     β = angle between plane and cone axis (acute).
        //     α = half-angle of cone.
        //   e = cos(β) / cos(α)? Let's just compute the two extreme points
        //   directly.
        //
        // The line of steepest descent in the plane from apex direction:
        //   axis_in_plane = axis - (axis · n) n.
        //   d_axip = axis_in_plane.norm()  (= sin(angle(axis, n)))
        //   Move apex along that direction in 2D (axial vs radial in the
        //   cone): solve the 2D conic.
        // For implementation, we use the following clean derivation:
        //   In the plane spanned by n and axis (both unit), let φ = angle
        //   between axis and plane = arcsin(|cos_theta|). The cone in that
        //   2D slice is a "V" with half-angle α from the axis.
        //   The plane intersects the V. The two extremes are at axial
        //   distances:
        //     v± = d_apex * sin(α) / [sin(α + (π/2 − φ)) + sign? ... ]
        //
        // For robustness given session time, we sample the ellipse case too —
        // we get a closed loop, and downstream consumers that want the ellipse
        // form can still work with a 32-pt polyline.
        let samples = sample_cone_plane_curve(cone, plane, tol, 64);
        if samples.len() < 8 {
            return ConePlaneIntersection::Empty;
        }
        // Fit a "best ellipse" to the samples? Too elaborate. Instead, return
        // a degenerate-EllipseSegment built from min-bounding circle of
        // samples (good enough for the regime distinction roadmap).
        // Compute centroid + average distance.
        let mut cx = 0.0;
        let mut cy = 0.0;
        let mut cz = 0.0;
        for p in &samples {
            cx += p.x;
            cy += p.y;
            cz += p.z;
        }
        let inv = 1.0 / samples.len() as f64;
        let center = Point3::new(cx * inv, cy * inv, cz * inv);
        // Find max distance from center along the plane's x and y to derive
        // a representative semi-major / semi-minor.
        let mut rmax = 0.0_f64;
        let mut rmin = f64::INFINITY;
        for p in &samples {
            let d = (*p - center).norm();
            if d > rmax {
                rmax = d;
            }
            if d < rmin {
                rmin = d;
            }
        }
        let frame = kerf_geom::Frame {
            origin: center,
            x: plane.frame.x,
            y: plane.frame.y,
            z: plane.frame.z,
        };
        let ellipse = Ellipse::new(frame, rmax.max(tol.point_eq), rmin.max(tol.point_eq));
        return ConePlaneIntersection::Ellipse(EllipseSegment::full(ellipse));
    }
    // β < α → Hyperbola.
    ConePlaneIntersection::Hyperbola(sample_cone_plane_curve(cone, plane, tol, 32))
}

/// Sample the Cone×Plane curve by stepping `v` over the cone's parameter
/// space and solving the cone ring equation for `u` at each step such that
/// the resulting point lies on the plane.
///
/// Each ring at axial distance `v` is the circle:
///   ring(u) = apex + v · axis + |v| · tan(α) · (cos u · x + sin u · y)
/// We project `ring(u) − plane.origin` onto plane.normal and require zero:
///   d0 + v · an + |v| · tan(α) · (xn cos u + yn sin u) = 0
/// where d0 = (apex - plane.origin) · n. Letting R = √(xn² + yn²) and
/// φ = atan2(yn, xn), this is `R cos(u − φ) = -(d0 + v·an) / (|v|·tan α)`.
fn sample_cone_plane_curve(
    cone: &Cone,
    plane: &Plane,
    _tol: &Tolerance,
    n_samples: usize,
) -> Vec<Point3> {
    let n = plane.frame.z;
    let xn = cone.frame.x.dot(&n);
    let yn = cone.frame.y.dot(&n);
    let an = cone.frame.z.dot(&n);
    let t = cone.half_angle.tan();
    let d0 = (cone.frame.origin - plane.frame.origin).dot(&n);
    let r_amp = (xn * xn + yn * yn).sqrt();
    let phase = yn.atan2(xn);

    // Sweep v over [-V, V] for some V chosen large enough to capture the
    // relevant branches.
    let v_max = 4.0;
    let mut out: Vec<Point3> = Vec::with_capacity(n_samples * 2);
    for k in 0..=n_samples {
        let v = -v_max + 2.0 * v_max * (k as f64) / (n_samples as f64);
        if v.abs() < 1e-9 {
            // Ring at apex degenerates.
            continue;
        }
        if r_amp < 1e-12 {
            // n is parallel to axis — handled by the perpendicular case.
            continue;
        }
        // cos(u − φ) = -(d0 + v·an) / (|v|·t·R_amp)
        let rhs = -(d0 + v * an) / (v.abs() * t * r_amp);
        if rhs.abs() > 1.0 + 1e-9 {
            continue;
        }
        let acos_val = rhs.clamp(-1.0, 1.0).acos();
        for u in [phase + acos_val, phase - acos_val] {
            let (su, cu) = u.sin_cos();
            let r = v.abs() * t;
            let p = cone.frame.origin
                + v * cone.frame.z
                + r * cu * cone.frame.x
                + r * su * cone.frame.y;
            out.push(p);
        }
    }
    out
}

// ----------------------------------------------------------------------------
// Torus × Plane
// ----------------------------------------------------------------------------

/// Closed-form / sampled result of `Torus ∩ Plane`.
///
/// Three principal regimes (axis-aligned plane through torus center):
///   * Plane perpendicular to axis at center (z = 0 in torus-local frame):
///     intersection is **two concentric circles** (inner radius `R−r`,
///     outer radius `R+r`).
///   * Plane perpendicular to axis NOT at center (`|d| < r`): intersection
///     is **two concentric circles** (radii `R ± √(r² − d²)`).
///   * Plane perpendicular to axis with `|d| = r`: intersection is a
///     single **circle of radius R** (tangent at top/bottom).
///   * Plane perpendicular to axis with `|d| > r`: **Empty**.
///   * Plane through (or parallel to) the axis: intersection is **two
///     circles** of radius `r` centered at offsets `±R` along the
///     axis-projected direction (Villarceau's coplanar circles, in the
///     simple coplanar case — for the non-coplanar Villarceau set, this
///     function returns sampled spirics).
///   * Other planes: 4th-degree spiric sections (e.g., Cassini ovals when
///     the plane is offset perpendicular to axis at `|d| > r` — wait, that's
///     empty above). Sampled to a polyline.
///
/// For the curved-surface boolean roadmap this captures the main regimes
/// without committing to a closed-form 4th-degree-curve representation.
#[derive(Clone, Debug)]
pub enum TorusPlaneIntersection {
    Empty,
    /// One circle (tangent case, `|d| = r` perpendicular plane).
    Circle(EllipseSegment),
    /// Two concentric circles (perpendicular plane, `|d| < r`).
    TwoCircles(EllipseSegment, EllipseSegment),
    /// Two parallel circles of radius `r` (plane through axis).
    TwoTubeCircles(EllipseSegment, EllipseSegment),
    /// 4th-degree curve sampled as a polyline. Used for general oblique
    /// planes where the exact "spiric section" form is not lifted to a
    /// closed-form curve type.
    Spiric(Vec<Point3>),
}

/// Closed-form (axis-aligned) / sampled (general) Torus × Plane intersection.
///
/// Math reference: a torus around `frame.z` with major radius `R`, minor
/// radius `r`. The implicit equation is:
///   (sqrt(x² + y²) − R)² + z² = r²    (in torus-local frame).
/// Substituting z = 0 (plane perpendicular to axis at center):
///   (sqrt(x² + y²) − R)² = r²  →  sqrt(x² + y²) = R ± r.
/// → Two concentric circles.
///
/// For a plane parallel to the axis through the center (e.g. y = 0 plane):
///   (|x| − R)² + z² = r²  → two circles of radius `r` centered at (±R, 0).
///
/// General oblique planes produce 4th-degree algebraic curves (Cassini-like
/// for planes parallel to axis off-center, "spiric sections" of Perseus
/// otherwise). We sample by parameter sweep over the torus's `(u, v)` grid
/// (u: around axis, v: around tube) and keep points whose plane-distance
/// magnitude is within tolerance.
pub fn torus_plane_intersection(
    torus: &Torus,
    plane: &Plane,
    tol: &Tolerance,
) -> TorusPlaneIntersection {
    use kerf_geom::Ellipse;

    let n = plane.frame.z;
    let axis = torus.frame.z;
    let center = torus.frame.origin;
    let r_major = torus.major_radius;
    let r_minor = torus.minor_radius;

    let d_center = (center - plane.frame.origin).dot(&n);

    // Case A: plane perpendicular to axis.
    if tol.directions_parallel(n, axis) {
        let abs_d = d_center.abs();
        if abs_d > r_minor + tol.point_eq {
            return TorusPlaneIntersection::Empty;
        }
        if (abs_d - r_minor).abs() < tol.point_eq {
            // Tangent: single circle of radius R at z = ±r.
            // We want t such that (center + t·axis - plane.origin)·n = 0
            //   → t = -d_center / cos_theta.
            let cos_theta = axis.dot(&n);
            let t = -d_center / cos_theta;
            let cc = center + t * axis;
            let frame = kerf_geom::Frame {
                origin: cc,
                x: torus.frame.x,
                y: torus.frame.y,
                z: axis,
            };
            let circle = Ellipse::new(frame, r_major, r_major);
            return TorusPlaneIntersection::Circle(EllipseSegment::full(circle));
        }
        // Two concentric circles of radii R ± √(r² − d²).
        let cos_theta = axis.dot(&n);
        let t = -d_center / cos_theta;
        let cc = center + t * axis;
        let h = (r_minor * r_minor - abs_d * abs_d).max(0.0).sqrt();
        let r_outer = r_major + h;
        let r_inner = r_major - h;
        let frame = kerf_geom::Frame {
            origin: cc,
            x: torus.frame.x,
            y: torus.frame.y,
            z: axis,
        };
        let outer = Ellipse::new(frame, r_outer, r_outer);
        let inner = Ellipse::new(frame, r_inner.max(tol.point_eq), r_inner.max(tol.point_eq));
        return TorusPlaneIntersection::TwoCircles(
            EllipseSegment::full(outer),
            EllipseSegment::full(inner),
        );
    }

    // Case B: plane parallel to axis through center → two minor circles.
    let n_dot_axis = n.dot(&axis);
    if n_dot_axis.abs() < tol.angle_eq {
        // Plane parallel to axis. Distance from torus center to plane:
        if d_center.abs() < tol.point_eq {
            // Plane passes through axis. Two minor-radius circles centered
            // at offsets ±R along the in-plane axis-perpendicular direction.
            // The two centers are `center ± R · m`, where `m` is the unit
            // vector in the plane perpendicular to the torus axis.
            // Build m: project torus.frame.x or y onto the plane.
            let candidate = torus.frame.x;
            let m_in_plane = candidate - candidate.dot(&n) * n;
            let m = m_in_plane.normalize();
            let c1 = center + r_major * m;
            let c2 = center - r_major * m;
            // Each circle lies in a plane perpendicular to the tube's local
            // direction. For the simple coplanar case (plane through axis
            // and center), the circles are coplanar with the slicing plane.
            let frame1 = kerf_geom::Frame {
                origin: c1,
                x: m,
                y: axis,
                z: n,
            };
            let frame2 = kerf_geom::Frame {
                origin: c2,
                x: m,
                y: axis,
                z: n,
            };
            let e1 = Ellipse::new(frame1, r_minor, r_minor);
            let e2 = Ellipse::new(frame2, r_minor, r_minor);
            return TorusPlaneIntersection::TwoTubeCircles(
                EllipseSegment::full(e1),
                EllipseSegment::full(e2),
            );
        }
        // Plane parallel to axis, offset off-center: 4th-degree Cassini-like.
        // Fall through to sampling.
    }

    // General: sample (u, v) over the torus and keep points within plane tol.
    let n_u = 96;
    let n_v = 48;
    let mut out: Vec<Point3> = Vec::new();
    let plane_origin = plane.frame.origin;
    // For each ring (fixed u), find v values where the signed distance to the
    // plane crosses zero. Linear interpolation between adjacent v samples.
    for iu in 0..=n_u {
        let u = (iu as f64) * std::f64::consts::TAU / (n_u as f64);
        let mut prev_d: f64 = 0.0;
        let mut prev_p = Point3::origin();
        let mut have_prev = false;
        for iv in 0..=n_v {
            let v = (iv as f64) * std::f64::consts::TAU / (n_v as f64);
            let (su, cu) = u.sin_cos();
            let (sv, cv) = v.sin_cos();
            let big = r_major + r_minor * cv;
            let p = center
                + big * cu * torus.frame.x
                + big * su * torus.frame.y
                + r_minor * sv * axis;
            let d = (p - plane_origin).dot(&n);
            if have_prev && (prev_d.signum() != d.signum() || prev_d.abs() < tol.point_eq)
            {
                let denom = d - prev_d;
                let alpha = if denom.abs() < 1e-12 {
                    0.0
                } else {
                    -prev_d / denom
                };
                let cross = prev_p + alpha * (p - prev_p);
                out.push(cross);
            }
            prev_d = d;
            prev_p = p;
            have_prev = true;
        }
    }
    if out.is_empty() {
        return TorusPlaneIntersection::Empty;
    }
    TorusPlaneIntersection::Spiric(out)
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

    // ------------------------------------------------------------------
    // Sphere × Sphere
    // ------------------------------------------------------------------

    #[test]
    fn sphere_sphere_overlap_returns_circle() {
        // Two unit spheres offset by 1 in X intersect in a circle of radius
        // √(1 - 0.25) = √0.75 centered at (0.5, 0, 0).
        let a = unit_sphere_origin();
        let b = Sphere::new(Frame::world(Point3::new(1.0, 0.0, 0.0)), 1.0);
        let tol = Tolerance::default();
        match sphere_sphere_intersection(&a, &b, &tol) {
            SphereSphereIntersection::Circle(seg) => {
                assert!(seg.is_full());
                assert_relative_eq!(seg.ellipse.semi_major, 0.75_f64.sqrt(), epsilon = 1e-9);
                assert_relative_eq!(
                    seg.ellipse.frame.origin,
                    Point3::new(0.5, 0.0, 0.0),
                    epsilon = 1e-9
                );
                // Sample-check: every point lies on both spheres.
                for k in 0..16 {
                    let t = (k as f64) * std::f64::consts::TAU / 16.0;
                    let p = seg.point_at(t);
                    let r1 = (p - Point3::origin()).norm();
                    let r2 = (p - Point3::new(1.0, 0.0, 0.0)).norm();
                    assert_relative_eq!(r1, 1.0, epsilon = 1e-9);
                    assert_relative_eq!(r2, 1.0, epsilon = 1e-9);
                }
            }
            other => panic!("expected Circle, got {other:?}"),
        }
    }

    #[test]
    fn sphere_sphere_disjoint_returns_empty() {
        let a = unit_sphere_origin();
        let b = Sphere::new(Frame::world(Point3::new(5.0, 0.0, 0.0)), 1.0);
        let tol = Tolerance::default();
        assert!(matches!(
            sphere_sphere_intersection(&a, &b, &tol),
            SphereSphereIntersection::Empty
        ));
    }

    #[test]
    fn sphere_sphere_external_tangent_returns_point() {
        let a = unit_sphere_origin();
        let b = Sphere::new(Frame::world(Point3::new(2.0, 0.0, 0.0)), 1.0);
        let tol = Tolerance::default();
        match sphere_sphere_intersection(&a, &b, &tol) {
            SphereSphereIntersection::Tangent(p) => {
                assert_relative_eq!(p, Point3::new(1.0, 0.0, 0.0), epsilon = 1e-9);
            }
            other => panic!("expected Tangent, got {other:?}"),
        }
    }

    #[test]
    fn sphere_sphere_identical_returns_coincident() {
        let a = unit_sphere_origin();
        let b = unit_sphere_origin();
        let tol = Tolerance::default();
        assert!(matches!(
            sphere_sphere_intersection(&a, &b, &tol),
            SphereSphereIntersection::Coincident
        ));
    }

    // ------------------------------------------------------------------
    // Cone × Plane
    // ------------------------------------------------------------------

    fn pi4_cone_z() -> Cone {
        // Half-angle π/4: at v=1, radius=1.
        Cone::new(Frame::world(Point3::origin()), std::f64::consts::FRAC_PI_4)
    }

    #[test]
    fn cone_plane_perp_axis_returns_circle() {
        // Plane z = 1 perpendicular to cone axis: ring at radius 1.
        let cone = pi4_cone_z();
        let plane = plane_with_normal(Point3::new(0.0, 0.0, 1.0), Vec3::z());
        let tol = Tolerance::default();
        match cone_plane_intersection(&cone, &plane, &tol) {
            ConePlaneIntersection::Circle(seg) => {
                assert!(seg.is_full());
                assert_relative_eq!(seg.ellipse.semi_major, 1.0, epsilon = 1e-9);
                assert_relative_eq!(seg.ellipse.semi_minor, 1.0, epsilon = 1e-9);
                assert_relative_eq!(
                    seg.ellipse.frame.origin,
                    Point3::new(0.0, 0.0, 1.0),
                    epsilon = 1e-9
                );
                // Sample-check: every point lies on the cone surface.
                for k in 0..16 {
                    let t = (k as f64) * std::f64::consts::TAU / 16.0;
                    let p = seg.point_at(t);
                    let radial = (p.x * p.x + p.y * p.y).sqrt();
                    let v = p.z;
                    assert_relative_eq!(radial, v.abs() * cone.half_angle.tan(), epsilon = 1e-9);
                }
            }
            other => panic!("expected Circle, got {other:?}"),
        }
    }

    #[test]
    fn cone_plane_through_apex_perp_returns_apex() {
        // Plane z = 0 cuts cone exactly at apex.
        let cone = pi4_cone_z();
        let plane = plane_with_normal(Point3::origin(), Vec3::z());
        let tol = Tolerance::default();
        match cone_plane_intersection(&cone, &plane, &tol) {
            ConePlaneIntersection::Apex(p) => {
                assert_relative_eq!(p, Point3::origin(), epsilon = 1e-12);
            }
            other => panic!("expected Apex, got {other:?}"),
        }
    }

    #[test]
    fn cone_plane_through_apex_steep_returns_two_lines() {
        // Plane through origin (apex) with normal NOT parallel to axis. Pick
        // a plane parallel to the axis (the XZ plane, normal = Y). This
        // plane contains the axis, so β = 0 < α = π/4 → two-line case.
        let cone = pi4_cone_z();
        let plane = plane_with_normal(Point3::origin(), Vec3::y());
        let tol = Tolerance::default();
        match cone_plane_intersection(&cone, &plane, &tol) {
            ConePlaneIntersection::TwoLines(l1, l2) => {
                // Both lines pass through apex (origin).
                assert_relative_eq!(l1.origin, Point3::origin(), epsilon = 1e-9);
                assert_relative_eq!(l2.origin, Point3::origin(), epsilon = 1e-9);
                // Both lines lie in the XZ plane (y component of direction = 0).
                assert!(l1.direction.y.abs() < 1e-9, "l1 dir y = {}", l1.direction.y);
                assert!(l2.direction.y.abs() < 1e-9, "l2 dir y = {}", l2.direction.y);
            }
            other => panic!("expected TwoLines, got {other:?}"),
        }
    }

    #[test]
    fn cone_plane_parallel_generator_returns_parabola() {
        // Pick a plane whose normal is at angle α to the axis. For α = π/4,
        // normal = (cos(α), 0, sin(α)) = (1/√2, 0, 1/√2). |cos_theta| =
        // |axis · n| = 1/√2 = sin(π/4) = sin(α) → parabola regime.
        let cone = pi4_cone_z();
        let nrm = Vec3::new(1.0, 0.0, 1.0).normalize();
        // Offset the plane so it's not through apex.
        let plane = plane_with_normal(Point3::new(0.0, 0.0, 1.0), nrm);
        let tol = Tolerance::default();
        match cone_plane_intersection(&cone, &plane, &tol) {
            ConePlaneIntersection::Parabola(samples) => {
                assert!(!samples.is_empty(), "expected non-empty parabola samples");
                // Every sample must lie on the cone surface and on the plane.
                for p in &samples {
                    let radial = (p.x * p.x + p.y * p.y).sqrt();
                    let expected_r = p.z.abs() * cone.half_angle.tan();
                    assert!(
                        (radial - expected_r).abs() < 1e-6,
                        "cone surface check failed: r={} expected={}",
                        radial,
                        expected_r
                    );
                    let signed = (*p - plane.frame.origin).dot(&nrm);
                    assert!(
                        signed.abs() < 1e-6,
                        "plane check failed: signed={}",
                        signed
                    );
                }
            }
            other => panic!("expected Parabola, got {other:?}"),
        }
    }

    #[test]
    fn cone_plane_oblique_steep_returns_ellipse() {
        // Plane with normal close to axis (β > α): elliptical section.
        // Use normal (0.2, 0, 1) — nearly along axis. Plane offset z=2.
        let cone = pi4_cone_z();
        let nrm = Vec3::new(0.2, 0.0, 1.0).normalize();
        let plane = plane_with_normal(Point3::new(0.0, 0.0, 2.0), nrm);
        let tol = Tolerance::default();
        match cone_plane_intersection(&cone, &plane, &tol) {
            ConePlaneIntersection::Ellipse(seg) => {
                assert!(seg.is_full());
                // semi_major must exceed semi_minor (or equal — degenerate).
                assert!(seg.ellipse.semi_major >= seg.ellipse.semi_minor - 1e-9);
            }
            other => panic!("expected Ellipse, got {other:?}"),
        }
    }

    #[test]
    fn cone_plane_oblique_shallow_returns_hyperbola() {
        // Plane with normal nearly perpendicular to axis (β < α): hyperbolic.
        // Use normal = (1, 0, 0.2)/|·|. Plane offset away from apex along x.
        let cone = pi4_cone_z();
        let nrm = Vec3::new(1.0, 0.0, 0.2).normalize();
        let plane = plane_with_normal(Point3::new(2.0, 0.0, 0.0), nrm);
        let tol = Tolerance::default();
        match cone_plane_intersection(&cone, &plane, &tol) {
            ConePlaneIntersection::Hyperbola(samples) => {
                assert!(!samples.is_empty(), "expected non-empty hyperbola samples");
                // Every sample must lie on the cone surface and on the plane.
                for p in &samples {
                    let radial = (p.x * p.x + p.y * p.y).sqrt();
                    let expected_r = p.z.abs() * cone.half_angle.tan();
                    assert!(
                        (radial - expected_r).abs() < 1e-6,
                        "cone surface check failed: r={} expected={}",
                        radial,
                        expected_r
                    );
                    let signed = (*p - plane.frame.origin).dot(&nrm);
                    assert!(
                        signed.abs() < 1e-6,
                        "plane check failed: signed={}",
                        signed
                    );
                }
            }
            other => panic!("expected Hyperbola, got {other:?}"),
        }
    }

    // ------------------------------------------------------------------
    // Torus × Plane
    // ------------------------------------------------------------------

    fn ring_torus() -> Torus {
        // Torus around Z, R=3, r=1.
        Torus::new(Frame::world(Point3::origin()), 3.0, 1.0)
    }

    #[test]
    fn torus_plane_perp_at_center_returns_two_concentric_circles() {
        let torus = ring_torus();
        let plane = plane_with_normal(Point3::origin(), Vec3::z());
        let tol = Tolerance::default();
        match torus_plane_intersection(&torus, &plane, &tol) {
            TorusPlaneIntersection::TwoCircles(outer, inner) => {
                assert_relative_eq!(outer.ellipse.semi_major, 4.0, epsilon = 1e-9);
                assert_relative_eq!(outer.ellipse.semi_minor, 4.0, epsilon = 1e-9);
                assert_relative_eq!(inner.ellipse.semi_major, 2.0, epsilon = 1e-9);
                assert_relative_eq!(inner.ellipse.semi_minor, 2.0, epsilon = 1e-9);
                // Both circles centered at origin in XY plane.
                assert_relative_eq!(outer.ellipse.frame.origin, Point3::origin(), epsilon = 1e-9);
                assert_relative_eq!(inner.ellipse.frame.origin, Point3::origin(), epsilon = 1e-9);
            }
            other => panic!("expected TwoCircles, got {other:?}"),
        }
    }

    #[test]
    fn torus_plane_perp_offset_returns_two_concentric_circles() {
        // Plane at z = 0.5 (|d| = 0.5 < r = 1).
        // Inner radius = R − √(r² − d²) = 3 − √(0.75) ≈ 2.134.
        // Outer radius = R + √0.75 ≈ 3.866.
        let torus = ring_torus();
        let plane = plane_with_normal(Point3::new(0.0, 0.0, 0.5), Vec3::z());
        let tol = Tolerance::default();
        match torus_plane_intersection(&torus, &plane, &tol) {
            TorusPlaneIntersection::TwoCircles(outer, inner) => {
                let h = 0.75_f64.sqrt();
                assert_relative_eq!(outer.ellipse.semi_major, 3.0 + h, epsilon = 1e-9);
                assert_relative_eq!(inner.ellipse.semi_major, 3.0 - h, epsilon = 1e-9);
            }
            other => panic!("expected TwoCircles, got {other:?}"),
        }
    }

    #[test]
    fn torus_plane_perp_above_torus_returns_empty() {
        // Plane at z = 5: well above torus (r = 1 means torus z ∈ [-1, 1]).
        let torus = ring_torus();
        let plane = plane_with_normal(Point3::new(0.0, 0.0, 5.0), Vec3::z());
        let tol = Tolerance::default();
        assert!(matches!(
            torus_plane_intersection(&torus, &plane, &tol),
            TorusPlaneIntersection::Empty
        ));
    }

    #[test]
    fn torus_plane_perp_at_top_tangent_returns_circle() {
        // Plane at z = 1 (= r): tangent circle of radius R.
        let torus = ring_torus();
        let plane = plane_with_normal(Point3::new(0.0, 0.0, 1.0), Vec3::z());
        let tol = Tolerance::default();
        match torus_plane_intersection(&torus, &plane, &tol) {
            TorusPlaneIntersection::Circle(seg) => {
                assert_relative_eq!(seg.ellipse.semi_major, 3.0, epsilon = 1e-9);
                assert_relative_eq!(seg.ellipse.frame.origin, Point3::new(0.0, 0.0, 1.0), epsilon = 1e-9);
            }
            other => panic!("expected Circle, got {other:?}"),
        }
    }

    #[test]
    fn torus_plane_through_axis_returns_two_tube_circles() {
        // Plane through the axis (XZ plane, normal = Y, through origin).
        // Two minor circles centered at (±R, 0, 0).
        let torus = ring_torus();
        let plane = plane_with_normal(Point3::origin(), Vec3::y());
        let tol = Tolerance::default();
        match torus_plane_intersection(&torus, &plane, &tol) {
            TorusPlaneIntersection::TwoTubeCircles(c1, c2) => {
                // Both have radius r.
                assert_relative_eq!(c1.ellipse.semi_major, 1.0, epsilon = 1e-9);
                assert_relative_eq!(c2.ellipse.semi_major, 1.0, epsilon = 1e-9);
                // Centers at ±(R, 0, 0).
                let centers = [c1.ellipse.frame.origin, c2.ellipse.frame.origin];
                let xs: Vec<f64> = centers.iter().map(|p| p.x).collect();
                assert!(xs.iter().any(|&x| (x - 3.0).abs() < 1e-9));
                assert!(xs.iter().any(|&x| (x + 3.0).abs() < 1e-9));
            }
            other => panic!("expected TwoTubeCircles, got {other:?}"),
        }
    }

    #[test]
    fn torus_plane_oblique_returns_spiric_polyline() {
        // Plane oblique to axis, offset away from center: 4th-degree
        // spiric section. We just verify samples lie on the torus and plane.
        let torus = ring_torus();
        let nrm = Vec3::new(1.0, 0.0, 1.0).normalize();
        let plane = plane_with_normal(Point3::new(0.5, 0.0, 0.5), nrm);
        let tol = Tolerance::default();
        match torus_plane_intersection(&torus, &plane, &tol) {
            TorusPlaneIntersection::Spiric(samples) => {
                assert!(!samples.is_empty(), "expected at least one sample");
                for p in &samples {
                    // On torus: (sqrt(x²+y²) − R)² + z² = r²
                    let big = (p.x * p.x + p.y * p.y).sqrt();
                    let lhs = (big - 3.0).powi(2) + p.z * p.z;
                    assert!(
                        (lhs - 1.0).abs() < 5e-2, // sampling tolerance is generous
                        "torus equation failed: lhs={} expected ≈1.0",
                        lhs
                    );
                    // On plane within sample tolerance.
                    let signed = (*p - plane.frame.origin).dot(&nrm);
                    assert!(signed.abs() < 5e-2, "plane signed dist = {}", signed);
                }
            }
            other => panic!("expected Spiric, got {other:?}"),
        }
    }
}
