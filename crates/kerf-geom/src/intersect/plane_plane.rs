//! Plane–Plane intersection. Closed-form: a line (or coincident planes).

use super::{IntersectionComponent, SurfaceSurfaceIntersection};
use crate::curves::Line;
use crate::surfaces::Plane;
use crate::tolerance::Tolerance;

pub fn intersect_plane_plane(a: &Plane, b: &Plane, tol: &Tolerance) -> SurfaceSurfaceIntersection {
    let na = a.frame.z;
    let nb = b.frame.z;
    let dir = na.cross(&nb);
    if dir.norm() < tol.angle_eq {
        if (b.frame.origin - a.frame.origin).dot(&na).abs() < tol.point_eq {
            return SurfaceSurfaceIntersection::Coincident;
        }
        return SurfaceSurfaceIntersection::Empty;
    }
    let dir = dir.normalize();
    let perp = dir.cross(&na);
    let denom = perp.dot(&nb);
    if denom.abs() < tol.angle_eq {
        return SurfaceSurfaceIntersection::Empty;
    }
    let s = (b.frame.origin - a.frame.origin).dot(&nb) / denom;
    let p0 = a.frame.origin + s * perp;
    let line = match Line::from_origin_dir(p0, dir) {
        Some(l) => l,
        None => return SurfaceSurfaceIntersection::Empty,
    };
    SurfaceSurfaceIntersection::Components(vec![IntersectionComponent::Line(line)])
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::{Frame, Point3, Vec3};
    use approx::assert_relative_eq;

    fn xy_plane() -> Plane {
        Plane::new(Frame::world(Point3::origin()))
    }

    #[test]
    fn xy_and_xz_intersect_in_x_axis() {
        let xy = xy_plane();
        let xz_frame = Frame::from_x_yhint(Point3::origin(), Vec3::x(), Vec3::z()).unwrap();
        let xz = Plane::new(xz_frame);
        match intersect_plane_plane(&xy, &xz, &Tolerance::default()) {
            SurfaceSurfaceIntersection::Components(comps) => {
                assert_eq!(comps.len(), 1);
                if let IntersectionComponent::Line(line) = &comps[0] {
                    assert_relative_eq!(
                        line.direction.cross(&Vec3::x()).norm(),
                        0.0,
                        epsilon = 1e-12
                    );
                    assert_relative_eq!(line.origin.y, 0.0, epsilon = 1e-9);
                    assert_relative_eq!(line.origin.z, 0.0, epsilon = 1e-9);
                } else {
                    panic!("expected Line component");
                }
            }
            other => panic!("{other:?}"),
        }
    }

    #[test]
    fn parallel_disjoint_planes_are_empty() {
        let xy0 = xy_plane();
        let xy1 = Plane::new(Frame::world(Point3::new(0.0, 0.0, 1.0)));
        assert!(matches!(
            intersect_plane_plane(&xy0, &xy1, &Tolerance::default()),
            SurfaceSurfaceIntersection::Empty
        ));
    }

    #[test]
    fn identical_planes_are_coincident() {
        let p = xy_plane();
        let q = xy_plane();
        assert!(matches!(
            intersect_plane_plane(&p, &q, &Tolerance::default()),
            SurfaceSurfaceIntersection::Coincident
        ));
    }
}
