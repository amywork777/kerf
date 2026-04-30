//! Sphere–Sphere intersection. Closed-form: empty / tangent point / radical circle.

use super::{IntersectionComponent, SurfaceSurfaceIntersection};
use crate::curves::Circle;
use crate::surfaces::Sphere;
use crate::tolerance::Tolerance;
use crate::types::{Frame, Vec3};

pub fn intersect_sphere_sphere(
    a: &Sphere,
    b: &Sphere,
    tol: &Tolerance,
) -> SurfaceSurfaceIntersection {
    let offset = b.frame.origin - a.frame.origin;
    let d = offset.norm();
    let r1 = a.radius;
    let r2 = b.radius;

    if d < tol.point_eq && (r1 - r2).abs() < tol.point_eq {
        return SurfaceSurfaceIntersection::Coincident;
    }
    if d > r1 + r2 + tol.point_eq || d < (r1 - r2).abs() - tol.point_eq {
        return SurfaceSurfaceIntersection::Empty;
    }

    let axis = offset / d;
    let alpha = (r1 * r1 - r2 * r2 + d * d) / (2.0 * d);
    let h2 = r1 * r1 - alpha * alpha;

    if h2 < tol.point_eq * tol.point_eq {
        let p = a.frame.origin + alpha * axis;
        return SurfaceSurfaceIntersection::Components(vec![IntersectionComponent::Point(p)]);
    }

    let r_x = h2.sqrt();
    let center = a.frame.origin + alpha * axis;
    let seed = if axis.dot(&Vec3::x()).abs() < 0.9 {
        Vec3::x()
    } else {
        Vec3::y()
    };
    let x = (seed - axis * seed.dot(&axis)).normalize();
    let y = axis.cross(&x);
    let frame = Frame {
        origin: center,
        x,
        y,
        z: axis,
    };
    let circle = Circle::new(frame, r_x);
    SurfaceSurfaceIntersection::Components(vec![IntersectionComponent::Circle(circle)])
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::{Frame, Point3, Vec3};
    use approx::assert_relative_eq;

    #[test]
    fn unit_spheres_offset_one_intersect_in_a_circle() {
        let a = Sphere::at_origin(1.0);
        let b = Sphere::new(Frame::world(Point3::new(1.0, 0.0, 0.0)), 1.0);
        match intersect_sphere_sphere(&a, &b, &Tolerance::default()) {
            SurfaceSurfaceIntersection::Components(comps) => {
                assert_eq!(comps.len(), 1);
                if let IntersectionComponent::Circle(c) = &comps[0] {
                    assert_relative_eq!(c.radius, 0.75_f64.sqrt(), epsilon = 1e-12);
                    assert_relative_eq!(
                        c.frame.origin,
                        Point3::new(0.5, 0.0, 0.0),
                        epsilon = 1e-12
                    );
                    assert_relative_eq!(c.frame.z.cross(&Vec3::x()).norm(), 0.0, epsilon = 1e-12);
                } else {
                    panic!("expected Circle");
                }
            }
            other => panic!("{other:?}"),
        }
    }

    #[test]
    fn tangent_external_yields_point() {
        let a = Sphere::at_origin(1.0);
        let b = Sphere::new(Frame::world(Point3::new(2.0, 0.0, 0.0)), 1.0);
        match intersect_sphere_sphere(&a, &b, &Tolerance::default()) {
            SurfaceSurfaceIntersection::Components(comps) => {
                assert_eq!(comps.len(), 1);
                if let IntersectionComponent::Point(p) = &comps[0] {
                    assert_relative_eq!(*p, Point3::new(1.0, 0.0, 0.0), epsilon = 1e-12);
                } else {
                    panic!("expected Point");
                }
            }
            other => panic!("{other:?}"),
        }
    }

    #[test]
    fn disjoint_spheres_are_empty() {
        let a = Sphere::at_origin(1.0);
        let b = Sphere::new(Frame::world(Point3::new(5.0, 0.0, 0.0)), 1.0);
        assert!(matches!(
            intersect_sphere_sphere(&a, &b, &Tolerance::default()),
            SurfaceSurfaceIntersection::Empty
        ));
    }

    #[test]
    fn identical_spheres_are_coincident() {
        let a = Sphere::at_origin(1.0);
        let b = Sphere::at_origin(1.0);
        assert!(matches!(
            intersect_sphere_sphere(&a, &b, &Tolerance::default()),
            SurfaceSurfaceIntersection::Coincident
        ));
    }
}
