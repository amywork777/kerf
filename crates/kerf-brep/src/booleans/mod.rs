//! Boolean operations on B-rep solids.

pub mod clip;
pub mod face_polygon;
pub mod intersect;

pub use clip::{clip_line_to_convex_polygon, ClipResult};
pub use face_polygon::face_polygon;
pub use intersect::{face_intersections, FaceIntersection};
