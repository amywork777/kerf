//! The Kerf B-rep kernel: topology + geometry + constructors.

pub mod geometry;
pub mod primitives;
pub mod solid;

pub use geometry::{CurveKind, CurveSegment, Sense, SurfaceKind};
pub use primitives::box_;
pub use solid::Solid;
