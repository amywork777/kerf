//! Closed-form intersection routines for Kerf geometry.
//!
//! M2a covers line-* pairs: Line–Line, Line–Plane,
//! Line–Cylinder, Line–Sphere, Line–Cone, Line–Torus.
//! M2b adds the closed-form circle pairs:
//! Line–Circle, Circle–Circle (coplanar), Circle–Plane, Circle–Sphere.

use crate::types::Point3;

pub mod line_circle;
pub mod line_cone;
pub mod line_cylinder;
pub mod line_line;
pub mod line_plane;
pub mod line_sphere;
pub mod line_torus;
pub mod poly;

pub use line_circle::intersect_line_circle;
pub use line_cone::intersect_line_cone;
pub use line_cylinder::intersect_line_cylinder;
pub use line_line::intersect_line_line;
pub use line_plane::intersect_line_plane;
pub use line_sphere::intersect_line_sphere;
pub use line_torus::intersect_line_torus;

/// Result of intersecting two curves.
#[derive(Clone, Debug, PartialEq)]
pub enum CurveCurveIntersection {
    Empty,
    Points(Vec<(f64, f64, Point3)>),
    Coincident,
}

/// Result of intersecting a curve with a surface.
#[derive(Clone, Debug, PartialEq)]
pub enum CurveSurfaceIntersection {
    Empty,
    Points(Vec<(f64, (f64, f64), Point3)>),
    OnSurface,
}
