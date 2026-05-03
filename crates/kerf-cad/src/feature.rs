//! Feature: one node in the model DAG. Carries its id, inputs, and parameters.

use serde::{Deserialize, Serialize};

use crate::scalar::Scalar;

/// A 2D polygonal profile, used by `ExtrudePolygon`. Points are in the XY
/// plane and consumed in order. The polygon should be simple (non-self-
/// intersecting) and CCW for the extrusion to come out outward-facing.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct Profile2D {
    pub points: Vec<[Scalar; 2]>,
}

/// One operation in the model DAG.
///
/// Every variant has an `id` (its key in the model) and either parameters
/// (for primitives) or `input`/`inputs` referencing other feature ids
/// (for transforms and booleans).
///
/// Booleans take 2+ inputs and fold left: `Union { inputs: [a,b,c] }`
/// evaluates as `(a ∪ b) ∪ c`.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(tag = "kind")]
pub enum Feature {
    Box {
        id: String,
        extents: [Scalar; 3],
    },
    BoxAt {
        id: String,
        extents: [Scalar; 3],
        origin: [Scalar; 3],
    },
    /// Faceted (polyhedral) cylinder. `segments` is the number of sides;
    /// it is structural, not a measurement, so it stays a literal `usize`.
    Cylinder {
        id: String,
        radius: Scalar,
        height: Scalar,
        segments: usize,
    },
    Sphere {
        id: String,
        radius: Scalar,
    },
    Torus {
        id: String,
        major_radius: Scalar,
        minor_radius: Scalar,
    },
    Cone {
        id: String,
        radius: Scalar,
        height: Scalar,
    },
    Frustum {
        id: String,
        top_radius: Scalar,
        bottom_radius: Scalar,
        height: Scalar,
    },
    ExtrudePolygon {
        id: String,
        profile: Profile2D,
        direction: [Scalar; 3],
    },

    Translate {
        id: String,
        input: String,
        offset: [Scalar; 3],
    },
    Rotate {
        id: String,
        input: String,
        axis: [Scalar; 3],
        angle_deg: Scalar,
        center: [Scalar; 3],
    },

    Union {
        id: String,
        inputs: Vec<String>,
    },
    Intersection {
        id: String,
        inputs: Vec<String>,
    },
    Difference {
        id: String,
        inputs: Vec<String>,
    },
}

impl Feature {
    pub fn id(&self) -> &str {
        match self {
            Feature::Box { id, .. }
            | Feature::BoxAt { id, .. }
            | Feature::Cylinder { id, .. }
            | Feature::Sphere { id, .. }
            | Feature::Torus { id, .. }
            | Feature::Cone { id, .. }
            | Feature::Frustum { id, .. }
            | Feature::ExtrudePolygon { id, .. }
            | Feature::Translate { id, .. }
            | Feature::Rotate { id, .. }
            | Feature::Union { id, .. }
            | Feature::Intersection { id, .. }
            | Feature::Difference { id, .. } => id,
        }
    }

    /// Ids this feature depends on.
    pub fn inputs(&self) -> Vec<&str> {
        match self {
            Feature::Box { .. }
            | Feature::BoxAt { .. }
            | Feature::Cylinder { .. }
            | Feature::Sphere { .. }
            | Feature::Torus { .. }
            | Feature::Cone { .. }
            | Feature::Frustum { .. }
            | Feature::ExtrudePolygon { .. } => Vec::new(),
            Feature::Translate { input, .. } | Feature::Rotate { input, .. } => {
                vec![input.as_str()]
            }
            Feature::Union { inputs, .. }
            | Feature::Intersection { inputs, .. }
            | Feature::Difference { inputs, .. } => inputs.iter().map(String::as_str).collect(),
        }
    }
}
