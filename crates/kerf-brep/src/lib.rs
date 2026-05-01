//! The Kerf B-rep kernel: topology + geometry + constructors.

pub mod booleans;
pub mod geometry;
pub mod primitives;
pub mod solid;
pub mod stl;
pub mod tessellate;

pub use geometry::{CurveKind, CurveSegment, Sense, SurfaceKind};
pub use primitives::{box_, box_at, cone, cylinder, extrude_polygon};
pub use solid::Solid;
pub use stl::{write_ascii, write_binary};
pub use tessellate::tessellate;
