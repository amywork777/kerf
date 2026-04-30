//! Sphere–Sphere intersection (stub).

#![allow(dead_code, unused_variables, unused_imports)]

use crate::surfaces::Sphere;
use crate::tolerance::Tolerance;
use super::SurfaceSurfaceIntersection;

pub fn intersect_sphere_sphere(_a: &Sphere, _b: &Sphere, _tol: &Tolerance) -> SurfaceSurfaceIntersection {
    SurfaceSurfaceIntersection::Empty
}
