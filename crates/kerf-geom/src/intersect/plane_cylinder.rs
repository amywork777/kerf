//! Plane–Cylinder intersection (stub).

#![allow(dead_code, unused_variables, unused_imports)]

use crate::surfaces::{Cylinder, Plane};
use crate::tolerance::Tolerance;
use super::SurfaceSurfaceIntersection;

pub fn intersect_plane_cylinder(_p: &Plane, _c: &Cylinder, _tol: &Tolerance) -> SurfaceSurfaceIntersection {
    SurfaceSurfaceIntersection::Empty
}
