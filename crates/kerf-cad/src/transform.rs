//! Rigid-body transforms (translate, rotate) applied to a kerf Solid.
//!
//! kerf has no built-in transform helper — primitives are constructed in their
//! local frame and composed via boolean ops. To position children in a model
//! tree we walk every Point3 + Frame attached to the solid and rewrite them.

use kerf_brep::{
    geometry::{CurveKind, SurfaceKind},
    Solid,
};
use kerf_geom::{Frame, Plane, Point3, Vec3};
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

/// Reflect `s` across the plane through `plane_origin` with unit normal
/// `plane_normal`. Volume is preserved; chirality flips. Surface frames
/// have their z (outward normal) negated; loop walks become CW-from-outside,
/// but [`face_polygon`] normalises winding via signed area against frame.z,
/// so downstream consumers see CCW polygons w.r.t. the new (flipped) z.
pub fn mirror_solid(s: &Solid, plane_origin: Point3, plane_normal: Vec3) -> Solid {
    let n = plane_normal.normalize();
    let reflect_p = |p: Point3| -> Point3 {
        let d = (p - plane_origin).dot(&n);
        p - 2.0 * d * n
    };
    let reflect_v = |v: Vec3| -> Vec3 { v - 2.0 * v.dot(&n) * n };
    let mut out = s.clone();
    for (_, p) in out.vertex_geom.iter_mut() {
        *p = reflect_p(*p);
    }
    for (_, surf) in out.face_geom.iter_mut() {
        mirror_surface(surf, plane_origin, n, &reflect_p, &reflect_v);
    }
    for (_, seg) in out.edge_geom.iter_mut() {
        mirror_curve(&mut seg.curve, &reflect_p, &reflect_v);
    }
    out
}

fn mirror_frame(
    frame: &mut Frame,
    reflect_p: &impl Fn(Point3) -> Point3,
    reflect_v: &impl Fn(Vec3) -> Vec3,
) {
    frame.origin = reflect_p(frame.origin);
    // Reflect each basis vector. The result is a left-handed frame; restore
    // right-handedness by recomputing y = z × x. (Negating z while keeping
    // reflected x and y would be wrong because reflection generally tilts
    // the basis non-uniformly.)
    let zr = reflect_v(frame.z);
    let xr = reflect_v(frame.x);
    let z = -zr; // negate so the polygon (now CW-from-outside in old frame) lines
                 //  up CCW-from-outside under the new outward direction.
    let x = (xr - z * xr.dot(&z)).normalize();
    let y = z.cross(&x);
    frame.x = x;
    frame.y = y;
    frame.z = z;
}

fn mirror_surface(
    surf: &mut SurfaceKind,
    _plane_origin: Point3,
    _plane_normal: Vec3,
    reflect_p: &impl Fn(Point3) -> Point3,
    reflect_v: &impl Fn(Vec3) -> Vec3,
) {
    match surf {
        SurfaceKind::Plane(p) => {
            let mut frame = p.frame;
            mirror_frame(&mut frame, reflect_p, reflect_v);
            *p = Plane::new(frame);
        }
        SurfaceKind::Cylinder(c) => mirror_frame(&mut c.frame, reflect_p, reflect_v),
        SurfaceKind::Sphere(s) => mirror_frame(&mut s.frame, reflect_p, reflect_v),
        SurfaceKind::Cone(c) => mirror_frame(&mut c.frame, reflect_p, reflect_v),
        SurfaceKind::Torus(t) => mirror_frame(&mut t.frame, reflect_p, reflect_v),
    }
}

fn mirror_curve(
    curve: &mut CurveKind,
    reflect_p: &impl Fn(Point3) -> Point3,
    reflect_v: &impl Fn(Vec3) -> Vec3,
) {
    match curve {
        CurveKind::Line(l) => {
            l.origin = reflect_p(l.origin);
            l.direction = reflect_v(l.direction);
        }
        CurveKind::Circle(c) => mirror_frame(&mut c.frame, reflect_p, reflect_v),
        CurveKind::Ellipse(e) => mirror_frame(&mut e.frame, reflect_p, reflect_v),
    }
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
