//! Re-export of the mass properties computation from `kerf-brep`.
//!
//! The actual implementation lives in `kerf_brep::mass`. This module
//! surfaces it through `kerf-cad`'s public API so callers only need to
//! depend on `kerf-cad`.

pub use kerf_brep::mass::{mass_properties, MassProperties};
