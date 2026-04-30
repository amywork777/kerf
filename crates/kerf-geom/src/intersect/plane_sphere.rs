//! Plane–Sphere intersection. Closed-form: empty / tangent point / cross-section circle.

use super::{IntersectionComponent, SurfaceSurfaceIntersection};
use crate::curves::Circle;
use crate::surfaces::{Plane, Sphere};
use crate::tolerance::Tolerance;
use crate::types::Frame;

pub fn intersect_plane_sphere(
    plane: &Plane,
    sphere: &Sphere,
    tol: &Tolerance,
) -> SurfaceSurfaceIntersection {
    let n = plane.frame.z;
    let d = (sphere.frame.origin - plane.frame.origin).dot(&n);
    let r = sphere.radius;
    if d.abs() > r + tol.point_eq {
        return SurfaceSurfaceIntersection::Empty;
    }
    if (d.abs() - r).abs() < tol.point_eq {
        let p = sphere.frame.origin - d * n;
        return SurfaceSurfaceIntersection::Components(vec![IntersectionComponent::Point(p)]);
    }
    let center = sphere.frame.origin - d * n;
    let cross_r = (r * r - d * d).sqrt();
    let frame = Frame {
        origin: center,
        x: plane.frame.x,
        y: plane.frame.y,
        z: plane.frame.z,
    };
    let circle = Circle::new(frame, cross_r);
    SurfaceSurfaceIntersection::Components(vec![IntersectionComponent::Circle(circle)])
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::{Frame, Point3};
    use approx::assert_relative_eq;

    #[test]
    fn xy_plane_through_unit_sphere_yields_unit_circle() {
        let plane = Plane::new(Frame::world(Point3::origin()));
        let sphere = Sphere::at_origin(1.0);
        match intersect_plane_sphere(&plane, &sphere, &Tolerance::default()) {
            SurfaceSurfaceIntersection::Components(comps) => {
                assert_eq!(comps.len(), 1);
                if let IntersectionComponent::Circle(c) = &comps[0] {
                    assert_relative_eq!(c.radius, 1.0, epsilon = 1e-12);
                    assert_relative_eq!(c.frame.origin, Point3::origin(), epsilon = 1e-12);
                } else {
                    panic!("expected Circle");
                }
            }
            other => panic!("{other:?}"),
        }
    }

    #[test]
    fn plane_tangent_to_sphere_yields_point() {
        let plane = Plane::new(Frame::world(Point3::new(0.0, 0.0, 1.0)));
        let sphere = Sphere::at_origin(1.0);
        match intersect_plane_sphere(&plane, &sphere, &Tolerance::default()) {
            SurfaceSurfaceIntersection::Components(comps) => {
                assert_eq!(comps.len(), 1);
                if let IntersectionComponent::Point(p) = &comps[0] {
                    assert_relative_eq!(*p, Point3::new(0.0, 0.0, 1.0), epsilon = 1e-12);
                } else {
                    panic!("expected Point");
                }
            }
            other => panic!("{other:?}"),
        }
    }

    #[test]
    fn plane_above_sphere_is_empty() {
        let plane = Plane::new(Frame::world(Point3::new(0.0, 0.0, 5.0)));
        let sphere = Sphere::at_origin(1.0);
        assert!(matches!(
            intersect_plane_sphere(&plane, &sphere, &Tolerance::default()),
            SurfaceSurfaceIntersection::Empty
        ));
    }

    #[test]
    fn slicing_plane_yields_smaller_circle() {
        let plane = Plane::new(Frame::world(Point3::new(0.0, 0.0, 0.5)));
        let sphere = Sphere::at_origin(1.0);
        if let SurfaceSurfaceIntersection::Components(comps) =
            intersect_plane_sphere(&plane, &sphere, &Tolerance::default())
        {
            if let IntersectionComponent::Circle(c) = &comps[0] {
                assert_relative_eq!(c.radius, 0.75_f64.sqrt(), epsilon = 1e-9);
                assert_relative_eq!(c.frame.origin, Point3::new(0.0, 0.0, 0.5), epsilon = 1e-12);
            } else {
                panic!();
            }
        } else {
            panic!();
        }
    }
}
