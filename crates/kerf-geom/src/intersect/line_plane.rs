//! Line-Plane intersection (stub).

#![allow(dead_code, unused_variables, unused_imports)]

use crate::curves::Line;
use crate::surfaces::Plane;
use crate::tolerance::Tolerance;

use super::CurveSurfaceIntersection;

pub fn intersect_line_plane(_l: &Line, _p: &Plane, _tol: &Tolerance) -> CurveSurfaceIntersection {
    CurveSurfaceIntersection::Empty
}
