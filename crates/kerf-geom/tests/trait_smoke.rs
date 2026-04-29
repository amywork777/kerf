//! Cross-cutting smoke tests: every Curve and Surface respects basic invariants.

use approx::assert_relative_eq;
use kerf_geom::{
    Circle, Cone, Curve, Cylinder, Ellipse, Frame, Line, Plane, Point3, Sphere, Surface, Torus,
    Vec3,
};
use std::f64::consts::FRAC_PI_4;

fn unit_frame() -> Frame {
    Frame::world(Point3::origin())
}

fn curves() -> Vec<Box<dyn Curve>> {
    vec![
        Box::new(Line::from_origin_dir(Point3::origin(), Vec3::x()).unwrap()),
        Box::new(Circle::new(unit_frame(), 1.0)),
        Box::new(Ellipse::new(unit_frame(), 3.0, 2.0)),
    ]
}

fn surfaces() -> Vec<Box<dyn Surface>> {
    vec![
        Box::new(Plane::new(unit_frame())),
        Box::new(Cylinder::new(unit_frame(), 1.5)),
        Box::new(Sphere::at_origin(2.0)),
        Box::new(Cone::new(unit_frame(), FRAC_PI_4)),
        Box::new(Torus::new(unit_frame(), 3.0, 1.0)),
    ]
}

#[test]
fn every_curve_has_unit_tangent_and_round_trips() {
    for c in curves() {
        for &t in &[0.0_f64, 0.5, 1.7, 3.0] {
            let p = c.point_at(t);
            let tan = c.tangent_at(t);
            assert_relative_eq!(tan.norm(), 1.0, epsilon = 1e-9);
            let (_t2, p2) = c.project(p);
            assert_relative_eq!(p2, p, epsilon = 1e-9);
        }
    }
}

#[test]
fn every_surface_has_unit_normal_and_round_trips() {
    for s in surfaces() {
        for &(u, v) in &[(0.5_f64, 0.5_f64), (1.2, 0.8), (3.0, 1.5)] {
            let p = s.point_at(u, v);
            let n = s.normal_at(u, v);
            assert_relative_eq!(n.norm(), 1.0, epsilon = 1e-9);
            let (_uv, p2) = s.project(p);
            assert_relative_eq!(p2, p, epsilon = 1e-9);
        }
    }
}
