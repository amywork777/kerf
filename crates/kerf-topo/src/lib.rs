//! Half-edge topology for the Kerf B-rep kernel.

// Entity fields unused at T2 will be consumed by mev/kev/mef/kef (T3-T4).
#![allow(dead_code)]

pub mod entity;
pub mod euler;
pub mod id;
pub mod solid;
pub mod validate;

pub use entity::{Edge, Face, HalfEdge, Loop, Shell, Vertex};
pub use euler::MvfsResult;
pub use id::{EdgeId, FaceId, HalfEdgeId, LoopId, ShellId, SolidId, VertexId};
pub use solid::{LoopWalker, Solid};
