//! kerf-cad: a parametric CAD layer over the kerf B-rep kernel.
//!
//! Models are declarative DAGs of `Feature` nodes keyed by id. `Model::evaluate`
//! walks the DAG, calls into kerf primitives + booleans, and returns a `Solid`.
//! JSON serde lets the recipe be data, not code.

pub mod assembly;
pub mod bom;
pub mod cache;
pub mod catalog;
pub mod equations;
pub mod eval;
pub mod feature;
pub mod geometric;
pub mod mass;
pub mod model;
pub mod scalar;
pub mod sketch;
pub mod solver;
pub mod step_import;
pub mod transform;

pub use assembly::{
    apply_pose_to_solid, Assembly, AssemblyError, AssemblyRef, AxisRef, Instance, Interference,
    Mate, MateError, Pose, ResolvedPose, SurfaceRef,
};
pub use bom::{assembly_bom, BomComponent, BomEntry, BomInput, BomInstance};
pub use cache::{EvalCache, Fingerprint};
pub use mass::{mass_properties, MassProperties};
pub use eval::EvalError;
pub use feature::{Feature, FilletEdge, Profile2D};
pub use geometric::{
    ConstraintError, EdgeRef, FaceRef, GeoAxisRef, GeometricConstraint, PointRef,
    solve_constraints,
};
pub use model::{Model, ModelError};
pub use scalar::{lits, Scalar};
pub use sketch::{Sketch, SketchConstraint, SketchError, SketchPlane, SketchPrim};
pub use solver::{DiagnosticReport, SolverConfig, SolverError};
pub use step_import::{import_step, import_step_to_model, StepImportError};
