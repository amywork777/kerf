//! Rigid-body transforms (translate, rotate) applied to a kerf Solid.
//!
//! kerf has no built-in transform helper — primitives are constructed in their
//! local frame and composed via boolean ops. To position children in a model
//! tree we walk every Point3 + Frame attached to the solid and rewrite them.

use kerf_brep::{
    geometry::{CurveKind, SurfaceKind},
    Solid,
};
use kerf_geom::{Frame, Point3, Vec3};
use nalgebra::{Rotation3, Unit};

pub fn translate_solid(s: &Solid, offset: Vec3) -> Solid {
    let mut out = s.clone();
    for (_, p) in out.vertex_geom.iter_mut() {
        *p += offset;
    }
    for (_, surf) in out.face_geom.iter_mut() {
        translate_surface(surf, offset);
    }
    for (_, seg) in out.edge_geom.iter_mut() {
        translate_curve(&mut seg.curve, offset);
    }
    out
}

/// Rotate every point + frame in `s` by `angle_rad` around `axis` through `center`.
/// `axis` must be non-zero; it is normalized internally.
pub fn rotate_solid(s: &Solid, axis: Vec3, angle_rad: f64, center: Point3) -> Solid {
    let unit_axis = Unit::new_normalize(axis);
    let rot = Rotation3::from_axis_angle(&unit_axis, angle_rad);
    let mut out = s.clone();
    for (_, p) in out.vertex_geom.iter_mut() {
        *p = rotate_point(*p, &rot, center);
    }
    for (_, surf) in out.face_geom.iter_mut() {
        rotate_surface(surf, &rot, center);
    }
    for (_, seg) in out.edge_geom.iter_mut() {
        rotate_curve(&mut seg.curve, &rot, center);
    }
    out
}

fn translate_surface(surf: &mut SurfaceKind, offset: Vec3) {
    match surf {
        SurfaceKind::Plane(p) => p.frame.origin += offset,
        SurfaceKind::Cylinder(c) => c.frame.origin += offset,
        SurfaceKind::Sphere(s) => s.frame.origin += offset,
        SurfaceKind::Cone(c) => c.frame.origin += offset,
        SurfaceKind::Torus(t) => t.frame.origin += offset,
    }
}

fn translate_curve(curve: &mut CurveKind, offset: Vec3) {
    match curve {
        CurveKind::Line(l) => l.origin += offset,
        CurveKind::Circle(c) => c.frame.origin += offset,
        CurveKind::Ellipse(e) => e.frame.origin += offset,
    }
}

fn rotate_point(p: Point3, rot: &Rotation3<f64>, center: Point3) -> Point3 {
    let v = p - center;
    center + rot * v
}

fn rotate_frame(frame: &mut Frame, rot: &Rotation3<f64>, center: Point3) {
    frame.origin = rotate_point(frame.origin, rot, center);
    frame.x = rot * frame.x;
    frame.y = rot * frame.y;
    frame.z = rot * frame.z;
}

fn rotate_surface(surf: &mut SurfaceKind, rot: &Rotation3<f64>, center: Point3) {
    match surf {
        SurfaceKind::Plane(p) => rotate_frame(&mut p.frame, rot, center),
        SurfaceKind::Cylinder(c) => rotate_frame(&mut c.frame, rot, center),
        SurfaceKind::Sphere(s) => rotate_frame(&mut s.frame, rot, center),
        SurfaceKind::Cone(c) => rotate_frame(&mut c.frame, rot, center),
        SurfaceKind::Torus(t) => rotate_frame(&mut t.frame, rot, center),
    }
}

fn rotate_curve(curve: &mut CurveKind, rot: &Rotation3<f64>, center: Point3) {
    match curve {
        CurveKind::Line(l) => {
            l.origin = rotate_point(l.origin, rot, center);
            l.direction = rot * l.direction;
        }
        CurveKind::Circle(c) => rotate_frame(&mut c.frame, rot, center),
        CurveKind::Ellipse(e) => rotate_frame(&mut e.frame, rot, center),
    }
}
