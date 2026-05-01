//! The Kerf B-rep kernel: topology + geometry + constructors.

pub mod booleans;
pub mod geometry;
pub mod obj;
pub mod primitives;
pub mod solid;
pub mod step;
pub mod stl;
pub mod tessellate;

pub use geometry::{CurveKind, CurveSegment, Sense, SurfaceKind};
pub use obj::write_obj;
pub use primitives::{box_, box_at, cone, cylinder, extrude_polygon, frustum, sphere, torus};
pub use solid::Solid;
pub use step::write_step;
pub use stl::{write_ascii, write_binary};
pub use tessellate::tessellate;
