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

/// Uniform scale of `s` around the origin by `factor`. `factor` must be
/// positive (negative would invert chirality and break loop walks).
pub fn scale_solid(s: &Solid, factor: f64) -> Solid {
    debug_assert!(factor > 0.0, "scale factor must be positive");
    let mut out = s.clone();
    for (_, p) in out.vertex_geom.iter_mut() {
        // Point3::new(p.x * factor, p.y * factor, p.z * factor)
        p.x *= factor;
        p.y *= factor;
        p.z *= factor;
    }
    for (_, surf) in out.face_geom.iter_mut() {
        scale_surface(surf, factor);
    }
    for (_, seg) in out.edge_geom.iter_mut() {
        scale_curve(&mut seg.curve, factor);
    }
    out
}

fn scale_surface(surf: &mut SurfaceKind, factor: f64) {
    use kerf_brep::geometry::SurfaceKind as SK;
    match surf {
        SK::Plane(p) => {
            p.frame.origin.x *= factor;
            p.frame.origin.y *= factor;
            p.frame.origin.z *= factor;
        }
        SK::Cylinder(c) => {
            c.frame.origin.x *= factor;
            c.frame.origin.y *= factor;
            c.frame.origin.z *= factor;
            c.radius *= factor;
        }
        SK::Sphere(s) => {
            s.frame.origin.x *= factor;
            s.frame.origin.y *= factor;
            s.frame.origin.z *= factor;
            s.radius *= factor;
        }
        SK::Cone(c) => {
            c.frame.origin.x *= factor;
            c.frame.origin.y *= factor;
            c.frame.origin.z *= factor;
            // half_angle is dimensionless under uniform scale.
        }
        SK::Torus(t) => {
            t.frame.origin.x *= factor;
            t.frame.origin.y *= factor;
            t.frame.origin.z *= factor;
            t.major_radius *= factor;
            t.minor_radius *= factor;
        }
    }
}

fn scale_curve(curve: &mut CurveKind, factor: f64) {
    match curve {
        CurveKind::Line(l) => {
            l.origin.x *= factor;
            l.origin.y *= factor;
            l.origin.z *= factor;
            // direction stays unit; line length is implicit.
        }
        CurveKind::Circle(c) => {
            c.frame.origin.x *= factor;
            c.frame.origin.y *= factor;
            c.frame.origin.z *= factor;
            c.radius *= factor;
        }
        CurveKind::Ellipse(e) => {
            e.frame.origin.x *= factor;
            e.frame.origin.y *= factor;
            e.frame.origin.z *= factor;
            e.semi_major *= factor;
            e.semi_minor *= factor;
        }
    }
}

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
/// `plane_normal`. Volume is preserved; chirality flips. After reflection
/// every face's outer loop is also reversed so loop walk direction stays
/// CCW-from-outward (where outward is the new flipped frame.z) — this
/// keeps the result composable with downstream booleans.
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
    reverse_all_loops(&mut out);
    out
}

/// Reverse the half-edge walk direction of every outer loop in the solid.
/// Walks each loop, collects its half-edges in order, then swaps each
/// half-edge's next/prev so the walk runs the opposite way.
fn reverse_all_loops(s: &mut Solid) {
    let face_ids: Vec<_> = s.topo.face_ids().collect();
    for face_id in face_ids {
        let outer = match s.topo.face(face_id) {
            Some(f) => f.outer_loop(),
            None => continue,
        };
        let lp = match s.topo.loop_(outer) {
            Some(l) => l,
            None => continue,
        };
        let start = match lp.half_edge() {
            Some(h) => h,
            None => continue,
        };
        let mut he_ids = Vec::new();
        let mut cur = start;
        loop {
            he_ids.push(cur);
            let he = match s.topo.half_edge(cur) {
                Some(h) => h,
                None => break,
            };
            cur = he.next();
            if cur == start {
                break;
            }
        }
        let n = he_ids.len();
        if n < 2 {
            continue;
        }
        // Reversed walk: he_ids[i].next was he_ids[(i+1)%n]; now make it
        // he_ids[(i+n-1)%n] (i.e., the previous in old order).
        for i in 0..n {
            let cur = he_ids[i];
            let new_next = he_ids[(i + n - 1) % n];
            s.topo.build_set_half_edge_next_prev(cur, new_next);
        }
    }
}

fn mirror_frame(
    frame: &mut Frame,
    reflect_p: &impl Fn(Point3) -> Point3,
    reflect_v: &impl Fn(Vec3) -> Vec3,
) {
    frame.origin = reflect_p(frame.origin);
    // Reflect each basis vector. Keep z = reflected z (the new outward
    // direction of the mirrored face — material side is on the opposite side
    // of the reflected plane, so the reflected normal stays "outward").
    // The frame would be left-handed after pure reflection; restore right-
    // handedness by recomputing y = z × x from the reflected x.
    let z = reflect_v(frame.z).normalize();
    let xr = reflect_v(frame.x);
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
