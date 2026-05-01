//! Boolean operations on B-rep solids.

pub mod clip;
pub mod face_polygon;
pub mod intersect;

pub use clip::{ClipResult, clip_line_to_convex_polygon};
pub use face_polygon::face_polygon;
pub use intersect::{FaceIntersection, face_intersections};
