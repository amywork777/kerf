//! kerf-cad: a parametric CAD layer over the kerf B-rep kernel.
//!
//! Models are declarative DAGs of `Feature` nodes keyed by id. `Model::evaluate`
//! walks the DAG, calls into kerf primitives + booleans, and returns a `Solid`.
//! JSON serde lets the recipe be data, not code.

pub mod eval;
pub mod feature;
pub mod model;
pub mod scalar;
pub mod transform;

pub use eval::EvalError;
pub use feature::{Feature, Profile2D};
pub use model::{Model, ModelError};
pub use scalar::{lits, Scalar};
