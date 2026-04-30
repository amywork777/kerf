//! Half-edge topology for the Kerf B-rep kernel.

// Entity fields are intentionally forward-declared for Euler operators (T2-T5).
#![allow(dead_code)]

pub mod entity;
pub mod id;
pub mod solid;

pub use entity::{Edge, Face, HalfEdge, Loop, Shell, Vertex};
pub use id::{EdgeId, FaceId, HalfEdgeId, LoopId, ShellId, SolidId, VertexId};
pub use solid::{LoopWalker, Solid};
