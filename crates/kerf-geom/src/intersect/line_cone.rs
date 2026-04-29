//! Line-Cone intersection (stub).

#![allow(dead_code, unused_variables, unused_imports)]

use crate::curves::Line;
use crate::surfaces::Cone;
use crate::tolerance::Tolerance;

use super::CurveSurfaceIntersection;

pub fn intersect_line_cone(_l: &Line, _c: &Cone, _tol: &Tolerance) -> CurveSurfaceIntersection {
    CurveSurfaceIntersection::Empty
}
