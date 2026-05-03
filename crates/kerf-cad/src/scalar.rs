//! Scalar values that can be either a literal or a `$parameter` reference.
//!
//! In JSON, this serializes as either a number (`1.5`) or a string starting
//! with `$` (`"$plate_x"`). A bare numeric string like `"1.5"` is rejected.

use std::collections::HashMap;

use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(untagged)]
pub enum Scalar {
    /// A concrete numeric value.
    Lit(f64),
    /// A `$name` reference resolved against `Model::parameters`.
    Param(String),
}

impl Scalar {
    pub fn lit(x: f64) -> Self {
        Scalar::Lit(x)
    }

    pub fn param(name: impl Into<String>) -> Self {
        let mut s = name.into();
        if !s.starts_with('$') {
            s.insert(0, '$');
        }
        Scalar::Param(s)
    }

    /// Resolve this scalar to an f64. Literals pass through; param references
    /// look up the bare name (without leading `$`) in `params`.
    pub fn resolve(&self, params: &HashMap<String, f64>) -> Result<f64, String> {
        match self {
            Scalar::Lit(x) => Ok(*x),
            Scalar::Param(raw) => {
                let name = raw.strip_prefix('$').unwrap_or(raw);
                params
                    .get(name)
                    .copied()
                    .ok_or_else(|| format!("unknown parameter '{raw}'"))
            }
        }
    }
}

impl From<f64> for Scalar {
    fn from(x: f64) -> Self {
        Scalar::Lit(x)
    }
}

/// Convert a fixed-size array of f64s into the same-size array of `Scalar::Lit`.
/// Lets call sites write `lits([10.0, 5.0, 2.0])` instead of three `.into()` casts.
pub fn lits<const N: usize>(arr: [f64; N]) -> [Scalar; N] {
    arr.map(Scalar::Lit)
}

pub(crate) fn resolve_arr<const N: usize>(
    arr: &[Scalar; N],
    params: &HashMap<String, f64>,
) -> Result<[f64; N], String> {
    let mut out = [0.0; N];
    for (i, s) in arr.iter().enumerate() {
        out[i] = s.resolve(params)?;
    }
    Ok(out)
}
