//! The Kerf B-rep kernel: topology + geometry + constructors.

pub mod booleans;
pub mod geometry;
pub mod measure;
pub mod mesh_import;
pub mod obj;
pub mod primitives;
pub mod serde_io;
pub mod solid;
pub mod step;
pub mod stl;
pub mod tessellate;

pub use geometry::{CurveKind, CurveSegment, EllipseSegment, Sense, SurfaceKind};
pub use measure::{shell_volume, solid_volume};
pub use mesh_import::{
    MeshImportError, from_triangles, read_ascii, read_stl_auto, read_stl_binary_to_solid,
    read_stl_to_solid,
};
pub use obj::{read_obj, read_obj_to_solid, write_obj};
pub use primitives::{
    box_, box_at, cone, cylinder, extrude_polygon, frustum, revolve_polyline, sphere, torus,
};
pub use serde_io::{read_json, write_json};
pub use solid::{BooleanError, Solid, try_boolean_solid};
pub use step::write_step;
pub use stl::{write_ascii, write_binary};
pub use tessellate::{tessellate, tessellate_with_face_index};
