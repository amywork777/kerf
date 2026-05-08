//! kerf-cad: a parametric CAD layer over the kerf B-rep kernel.
//!
//! Models are declarative DAGs of `Feature` nodes keyed by id. `Model::evaluate`
//! walks the DAG, calls into kerf primitives + booleans, and returns a `Solid`.
//! JSON serde lets the recipe be data, not code.

pub mod assembly;
pub mod cache;
pub mod eval;
pub mod feature;
pub mod model;
pub mod scalar;
pub mod sketch;
pub mod solver;
pub mod transform;

pub use assembly::{
    apply_pose_to_solid, Assembly, AssemblyError, AssemblyRef, AxisRef, Instance, Mate, MateError,
    Pose, ResolvedPose, SurfaceRef,
};
pub use cache::{EvalCache, Fingerprint};
pub use eval::EvalError;
pub use feature::{Feature, FilletEdge, Profile2D};
pub use model::{Model, ModelError};
pub use scalar::{lits, Scalar};
pub use sketch::{Sketch, SketchConstraint, SketchError, SketchPlane, SketchPrim};
pub use solver::{DiagnosticReport, SolverConfig, SolverError};
