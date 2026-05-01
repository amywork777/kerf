//! Boolean operations on B-rep solids.

pub mod clip;
pub mod edge_lookup;
pub mod face_polygon;
pub mod intersect;
pub mod splice;
pub mod split;

pub use clip::{ClipResult, clip_line_to_convex_polygon};
pub use edge_lookup::{PointLocation, locate_point_on_face};
pub use face_polygon::face_polygon;
pub use intersect::{FaceIntersection, face_intersections};
pub use splice::{AddedEdge, SolidIndex, add_intersection_edges, find_he_in_loop, vertices_connected};
pub use split::{EndpointVertices, SplitOutcome, split_solids_at_intersections};
