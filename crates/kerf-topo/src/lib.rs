//! Half-edge topology for the Kerf B-rep kernel.

pub mod entity;
pub mod euler;
pub mod id;
pub mod solid;
pub mod validate;

pub use entity::{Edge, Face, HalfEdge, Loop, Shell, Vertex};
pub use euler::{MevResult, MvfsResult};
pub use id::{EdgeId, FaceId, HalfEdgeId, LoopId, ShellId, SolidId, VertexId};
pub use solid::{LoopWalker, Solid};
