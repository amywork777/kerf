//! Plane–Cylinder intersection. Closed-form: line(s) or ellipse or circle.

use super::{IntersectionComponent, SurfaceSurfaceIntersection};
use crate::curves::{Circle, Ellipse, Line};
use crate::surfaces::{Cylinder, Plane};
use crate::tolerance::Tolerance;
use crate::types::Frame;

pub fn intersect_plane_cylinder(
    plane: &Plane,
    cyl: &Cylinder,
    tol: &Tolerance,
) -> SurfaceSurfaceIntersection {
    let n = plane.frame.z;
    let axis = cyl.frame.z;
    let cos_alpha = axis.dot(&n);

    // Case 1: Plane perpendicular to axis (normal parallel to axis): circle.
    if tol.directions_parallel(n, axis) {
        let to_origin = plane.frame.origin - cyl.frame.origin;
        let t = to_origin.dot(&n) / axis.dot(&n);
        let center = cyl.frame.origin + t * axis;
        let frame = Frame {
            origin: center,
            x: cyl.frame.x,
            y: cyl.frame.y,
            z: cyl.frame.z,
        };
        let circle = Circle::new(frame, cyl.radius);
        return SurfaceSurfaceIntersection::Components(vec![IntersectionComponent::Circle(circle)]);
    }

    // Case 2: Plane parallel to axis (normal perpendicular to axis): line(s).
    if cos_alpha.abs() < tol.angle_eq {
        let delta = (cyl.frame.origin - plane.frame.origin).dot(&n);
        if delta.abs() > cyl.radius + tol.point_eq {
            return SurfaceSurfaceIntersection::Empty;
        }
        if (delta.abs() - cyl.radius).abs() < tol.point_eq {
            let foot = cyl.frame.origin - delta * n;
            let line = Line::from_origin_dir(foot, axis).unwrap();
            return SurfaceSurfaceIntersection::Components(vec![IntersectionComponent::Line(line)]);
        }
        let foot = cyl.frame.origin - delta * n;
        let t_dir = axis.cross(&n).normalize();
        let half_chord = (cyl.radius * cyl.radius - delta * delta).sqrt();
        let p1 = foot + half_chord * t_dir;
        let p2 = foot - half_chord * t_dir;
        let l1 = Line::from_origin_dir(p1, axis).unwrap();
        let l2 = Line::from_origin_dir(p2, axis).unwrap();
        return SurfaceSurfaceIntersection::Components(vec![
            IntersectionComponent::Line(l1),
            IntersectionComponent::Line(l2),
        ]);
    }

    // Case 3: Oblique — ellipse.
    let to_origin = plane.frame.origin - cyl.frame.origin;
    let t = to_origin.dot(&n) / cos_alpha;
    let center = cyl.frame.origin + t * axis;
    let minor_dir = axis.cross(&n).normalize();
    let major_dir = (axis - cos_alpha * n).normalize();
    let semi_major = cyl.radius / cos_alpha.abs();
    let semi_minor = cyl.radius;
    let frame = Frame {
        origin: center,
        x: major_dir,
        y: minor_dir,
        z: n,
    };
    let ellipse = Ellipse::new(frame, semi_major, semi_minor);
    SurfaceSurfaceIntersection::Components(vec![IntersectionComponent::Ellipse(ellipse)])
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::{Frame, Point3, Vec3};
    use approx::assert_relative_eq;

    fn unit_cyl_z() -> Cylinder {
        Cylinder::new(Frame::world(Point3::origin()), 1.0)
    }

    #[test]
    fn xy_plane_through_axis_cylinder_yields_circle() {
        let plane = Plane::new(Frame::world(Point3::origin()));
        match intersect_plane_cylinder(&plane, &unit_cyl_z(), &Tolerance::default()) {
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
    fn plane_parallel_to_axis_through_axis_yields_two_lines() {
        // Build a plane whose normal is Y (i.e. the XZ plane at y=0).
        // frame.z = normal = Y: use from_x_yhint(origin, Z, X) → z = Z×X = Y.
        let yz_frame = Frame::from_x_yhint(Point3::origin(), Vec3::z(), Vec3::x()).unwrap();
        let plane = Plane::new(yz_frame);
        match intersect_plane_cylinder(&plane, &unit_cyl_z(), &Tolerance::default()) {
            SurfaceSurfaceIntersection::Components(comps) => {
                assert_eq!(comps.len(), 2);
                let mut xs = Vec::new();
                for c in &comps {
                    if let IntersectionComponent::Line(l) = c {
                        xs.push(l.origin.x);
                        assert_relative_eq!(
                            l.direction.cross(&Vec3::z()).norm(),
                            0.0,
                            epsilon = 1e-12
                        );
                    } else {
                        panic!("expected Line");
                    }
                }
                xs.sort_by(|a, b| a.partial_cmp(b).unwrap());
                assert_relative_eq!(xs[0], -1.0, epsilon = 1e-9);
                assert_relative_eq!(xs[1], 1.0, epsilon = 1e-9);
            }
            other => panic!("{other:?}"),
        }
    }

    #[test]
    fn plane_parallel_to_axis_outside_cylinder_is_empty() {
        // Same orientation (normal = Y), but translated to y=5, which is outside radius=1.
        let frame = Frame::from_x_yhint(Point3::new(0.0, 5.0, 0.0), Vec3::z(), Vec3::x()).unwrap();
        let plane = Plane::new(frame);
        assert!(matches!(
            intersect_plane_cylinder(&plane, &unit_cyl_z(), &Tolerance::default()),
            SurfaceSurfaceIntersection::Empty
        ));
    }

    #[test]
    fn oblique_plane_yields_ellipse() {
        // Plane through origin with normal (1,0,1)/sqrt(2): 45° to z. Build manually so frame.z == n.
        let n = Vec3::new(1.0, 0.0, 1.0).normalize();
        let x = Vec3::y();
        let y = n.cross(&x).normalize();
        let frame = Frame {
            origin: Point3::origin(),
            x,
            y,
            z: n,
        };
        let plane = Plane::new(frame);
        match intersect_plane_cylinder(&plane, &unit_cyl_z(), &Tolerance::default()) {
            SurfaceSurfaceIntersection::Components(comps) => {
                assert_eq!(comps.len(), 1);
                if let IntersectionComponent::Ellipse(e) = &comps[0] {
                    assert_relative_eq!(e.semi_minor, 1.0, epsilon = 1e-9);
                    assert_relative_eq!(e.semi_major, std::f64::consts::SQRT_2, epsilon = 1e-9);
                } else {
                    panic!("expected Ellipse, got {:?}", comps[0]);
                }
            }
            other => panic!("{other:?}"),
        }
    }
}
