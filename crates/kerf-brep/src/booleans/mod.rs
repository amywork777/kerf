//! Boolean operations on B-rep solids.

pub mod classify;
pub mod clip;
pub mod edge_lookup;
pub mod face_polygon;
pub mod intersect;
pub mod pipeline;
pub mod select;
pub mod splice;
pub mod split;
pub mod stitch;
pub mod triangulate;

pub use classify::{FaceClassification, classify_face, face_centroid};
pub use clip::{ClipResult, clip_line_to_convex_polygon};
pub use edge_lookup::{PointLocation, locate_point_on_face};
pub use face_polygon::face_polygon;
pub use intersect::{FaceIntersection, face_intersections};
pub use pipeline::{FaceSoup, boolean};
pub use select::{BooleanOp, SelectedFaces, flip_b_face, keep_a_face, keep_b_face};
pub use splice::{
    AddedEdge, SolidIndex, add_intersection_edges, find_he_in_loop, vertices_connected,
};
pub use split::{EndpointVertices, SplitOutcome, split_solids_at_intersections};
pub use stitch::{KeptFace, stitch};
pub use triangulate::{fan_triangulate, fan_triangulate_reversed};
