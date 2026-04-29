//! Line-Cylinder intersection (stub).

#![allow(dead_code, unused_variables, unused_imports)]

use crate::curves::Line;
use crate::surfaces::Cylinder;
use crate::tolerance::Tolerance;

use super::CurveSurfaceIntersection;

pub fn intersect_line_cylinder(_l: &Line, _c: &Cylinder, _tol: &Tolerance) -> CurveSurfaceIntersection {
    CurveSurfaceIntersection::Empty
}
