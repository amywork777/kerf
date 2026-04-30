//! Persistent IDs for half-edge topology entities.
//!
//! Each ID is a slotmap key so the arena can recycle freed slots safely.

use slotmap::new_key_type;

new_key_type! {
    pub struct VertexId;
    pub struct HalfEdgeId;
    pub struct EdgeId;
    pub struct LoopId;
    pub struct FaceId;
    pub struct ShellId;
    pub struct SolidId;
}
