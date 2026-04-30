//! Plane–Sphere intersection (stub).

#![allow(dead_code, unused_variables, unused_imports)]

use crate::surfaces::{Plane, Sphere};
use crate::tolerance::Tolerance;
use super::SurfaceSurfaceIntersection;

pub fn intersect_plane_sphere(_p: &Plane, _s: &Sphere, _tol: &Tolerance) -> SurfaceSurfaceIntersection {
    SurfaceSurfaceIntersection::Empty
}
