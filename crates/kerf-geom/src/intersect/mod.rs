//! Closed-form intersection routines for Kerf geometry.
//!
//! M2a covers line-* pairs. M2b adds the closed-form circle pairs:
//! Line–Circle, Circle–Circle (coplanar), Circle–Plane, Circle–Sphere.
//! Harder pairs (Circle vs Cylinder/Cone/Torus, all Ellipse pairs) are
//! deferred to the numerical fallback in M3.

use crate::types::Point3;

pub mod circle_circle;
pub mod circle_plane;
pub mod circle_sphere;
pub mod line_circle;
pub mod line_cone;
pub mod line_cylinder;
pub mod line_line;
pub mod line_plane;
pub mod line_sphere;
pub mod line_torus;
pub mod poly;

pub use circle_circle::intersect_circle_circle;
pub use circle_plane::intersect_circle_plane;
pub use circle_sphere::intersect_circle_sphere;
pub use line_circle::intersect_line_circle;
pub use line_cone::intersect_line_cone;
pub use line_cylinder::intersect_line_cylinder;
pub use line_line::intersect_line_line;
pub use line_plane::intersect_line_plane;
pub use line_sphere::intersect_line_sphere;
pub use line_torus::intersect_line_torus;

#[derive(Clone, Debug, PartialEq)]
pub enum CurveCurveIntersection {
    Empty,
    Points(Vec<(f64, f64, Point3)>),
    Coincident,
}

#[derive(Clone, Debug, PartialEq)]
pub enum CurveSurfaceIntersection {
    Empty,
    Points(Vec<(f64, (f64, f64), Point3)>),
    OnSurface,
}
