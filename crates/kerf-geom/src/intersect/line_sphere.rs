//! Line-Sphere intersection (stub).

#![allow(dead_code, unused_variables, unused_imports)]

use crate::curves::Line;
use crate::surfaces::Sphere;
use crate::tolerance::Tolerance;

use super::CurveSurfaceIntersection;

pub fn intersect_line_sphere(_l: &Line, _s: &Sphere, _tol: &Tolerance) -> CurveSurfaceIntersection {
    CurveSurfaceIntersection::Empty
}
