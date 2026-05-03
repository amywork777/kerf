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
    /// Revolve an open polyline in the xz-plane around the z-axis. The first
    /// and last points must lie on the z-axis (x = 0); all interior points
    /// must have x > 0. Produces a closed axisymmetric solid.
    Revolve {
        id: String,
        profile: Profile2D,
    },

    /// Subtract an axis-aligned box (a "corner cutter") from `input`. The
    /// cutter has its `min` corner at `corner` and the given `extents`. Useful
    /// as quick-and-dirty chamfer-by-cut or "flatten this corner of a
    /// bounding box". Composes as `Difference(input, BoxAt(extents, corner))`.
    CornerCut {
        id: String,
        input: String,
        corner: [Scalar; 3],
        extents: [Scalar; 3],
    },

    /// Hollow circular tube: outer cylinder minus a centered inner cylinder.
    /// Both share the same axis (origin, +z direction) and height.
    Tube {
        id: String,
        outer_radius: Scalar,
        inner_radius: Scalar,
        height: Scalar,
        segments: usize,
    },
    /// Box with a uniform inset wall thickness on all six faces. Equivalent
    /// to BoxAt(extents, [0,0,0]) minus BoxAt(extents - 2*wall, [wall,wall,wall]).
    HollowBox {
        id: String,
        extents: [Scalar; 3],
        wall_thickness: Scalar,
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
    /// Reflect `input` across the plane defined by `plane_origin` and
    /// `plane_normal`. Volume is preserved. The result is a single mirrored
    /// body (NOT unioned with the original — for a symmetric design,
    /// follow this with a Union of input + the mirrored result).
    Mirror {
        id: String,
        input: String,
        plane_origin: [Scalar; 3],
        plane_normal: [Scalar; 3],
    },

    /// Replicate `input` `count` times along `offset`, unioning all copies.
    /// `count = 1` returns the input unchanged. `count = 0` is an error.
    LinearPattern {
        id: String,
        input: String,
        count: usize,
        offset: [Scalar; 3],
    },
    /// Replicate `input` `count` times around `axis` through `center`,
    /// distributing `total_angle_deg` evenly. `total_angle_deg = 360`
    /// gives a closed circle. Copies are unioned.
    PolarPattern {
        id: String,
        input: String,
        count: usize,
        axis: [Scalar; 3],
        center: [Scalar; 3],
        total_angle_deg: Scalar,
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
            | Feature::Revolve { id, .. }
            | Feature::Tube { id, .. }
            | Feature::HollowBox { id, .. }
            | Feature::CornerCut { id, .. }
            | Feature::Translate { id, .. }
            | Feature::Rotate { id, .. }
            | Feature::Mirror { id, .. }
            | Feature::LinearPattern { id, .. }
            | Feature::PolarPattern { id, .. }
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
            | Feature::ExtrudePolygon { .. }
            | Feature::Revolve { .. }
            | Feature::Tube { .. }
            | Feature::HollowBox { .. } => Vec::new(),
            Feature::Translate { input, .. }
            | Feature::Rotate { input, .. }
            | Feature::Mirror { input, .. }
            | Feature::LinearPattern { input, .. }
            | Feature::PolarPattern { input, .. }
            | Feature::CornerCut { input, .. } => {
                vec![input.as_str()]
            }
            Feature::Union { inputs, .. }
            | Feature::Intersection { inputs, .. }
            | Feature::Difference { inputs, .. } => inputs.iter().map(String::as_str).collect(),
        }
    }
}
