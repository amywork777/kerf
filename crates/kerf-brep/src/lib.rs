//! The Kerf B-rep kernel: topology + geometry + constructors.

pub mod booleans;
pub mod geometry;
pub mod primitives;
pub mod solid;
pub mod stl;

pub use geometry::{CurveKind, CurveSegment, Sense, SurfaceKind};
pub use primitives::box_;
pub use solid::Solid;
pub use stl::{write_ascii, write_binary};
