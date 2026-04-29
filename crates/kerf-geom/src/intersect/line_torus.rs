//! Line-Torus intersection (stub).

#![allow(dead_code, unused_variables, unused_imports)]

use crate::curves::Line;
use crate::surfaces::Torus;
use crate::tolerance::Tolerance;

use super::CurveSurfaceIntersection;

pub fn intersect_line_torus(_l: &Line, _t: &Torus, _tol: &Tolerance) -> CurveSurfaceIntersection {
    CurveSurfaceIntersection::Empty
}
