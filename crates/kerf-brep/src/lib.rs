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

pub use geometry::{CurveKind, CurveSegment, Sense, SurfaceKind};
pub use measure::{
    axis_aligned_line_edge, edge_info, face_boundary_edges, quadrant_hint_for_axis_edge,
    shell_volume, solid_volume, EdgeInfo,
};
pub use mesh_import::{from_triangles, read_ascii, read_stl_auto, read_stl_binary_to_solid, read_stl_to_solid, MeshImportError};
pub use obj::{read_obj, read_obj_to_solid, write_obj};
pub use primitives::{box_, box_at, cone, cylinder, extrude_polygon, frustum, revolve_polyline, sphere, torus};
pub use serde_io::{read_json, write_json};
pub use solid::{try_boolean_solid, BooleanError, Solid};
pub use step::write_step;
pub use stl::{write_ascii, write_binary};
pub use tessellate::{tessellate, tessellate_with_face_index};
