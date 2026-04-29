//! Line-Line intersection (stub).

#![allow(dead_code, unused_variables, unused_imports)]

use crate::curves::Line;
use crate::tolerance::Tolerance;

use super::CurveCurveIntersection;

pub fn intersect_line_line(_a: &Line, _b: &Line, _tol: &Tolerance) -> CurveCurveIntersection {
    CurveCurveIntersection::Empty
}
