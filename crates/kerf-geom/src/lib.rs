//! Exact analytic geometry for the Kerf kernel.

pub mod curve;
pub mod curves;
pub mod intersect;
pub mod surface;
pub mod surfaces;
pub mod tolerance;
pub mod types;

pub use curve::Curve;
pub use curves::{Circle, Ellipse, Line};
pub use intersect::{CurveCurveIntersection, CurveSurfaceIntersection};
pub use surface::{Domain2, Surface};
pub use surfaces::{Cone, Cylinder, Plane, Sphere, Torus};
pub use tolerance::Tolerance;
pub use types::{Axis, Frame, Point3, Vec3};
