//! The Kerf B-rep kernel: topology + geometry + constructors.

pub mod analytic_edge;
pub mod booleans;
pub mod dimension;
pub mod export_3mf;
pub mod export_gltf;
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

pub use analytic_edge::AnalyticEdge;
pub use dimension::{
    angle_at_vertex, angle_between_vectors, collect_snap_candidates, distance, project_to_plane,
    projected_silhouette, render_dimensioned_view, silhouette_loops, snap_pick, to_2d_view,
    view_direction, Dimension, DimensionKind, SilhouetteLoops, SnapCandidate, SnapKind, ViewKind,
    ViewportSpec,
};
pub use geometry::{CurveKind, CurveSegment, Sense, SurfaceKind};
pub use measure::{shell_volume, solid_volume};
pub use mesh_import::{from_triangles, read_ascii, read_stl_auto, read_stl_binary_to_solid, read_stl_to_solid, MeshImportError};
pub use export_3mf::{write_3mf, Export3mfError};
pub use export_gltf::{write_gltf, ExportGltfError};
pub use obj::{read_obj, read_obj_to_solid, write_obj};
pub use primitives::{
    box_, box_at, cone, cylinder, extrude_polygon, extrude_polygon_with_holes, frustum,
    revolve_polyline, sphere, torus, PolygonWithHolesError,
};
pub use serde_io::{read_json, write_json};
pub use solid::{try_boolean_solid, BooleanError, Solid};
pub use step::write_step;
pub use stl::{write_ascii, write_binary};
pub use tessellate::{tessellate, tessellate_with_face_index};
