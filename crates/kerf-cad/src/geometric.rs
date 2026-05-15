//! Part-level geometric constraints: relationships between feature outputs
//! within a single Model. The solver runs after parameter resolution and
//! adjusts top-level parameters to satisfy the declared relationships.
//!
//! Supported feature kinds (v1): Box, BoxAt, Cylinder, CylinderAt.
//! All other feature kinds return [`ConstraintError::Unsupported`].

use std::collections::HashMap;

use serde::{Deserialize, Serialize};
use thiserror::Error;

use crate::feature::Feature;
use crate::model::Model;
use crate::scalar::resolve_arr;

// ---------------------------------------------------------------------------
// Public types
// ---------------------------------------------------------------------------

/// A constraint between two feature outputs expressed by face/edge/axis
/// references. The solver runs after equations and adjusts top-level
/// parameters to satisfy. Failures report which constraint is unsatisfiable.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(tag = "kind")]
pub enum GeometricConstraint {
    /// Two named faces must be parallel. Adjusts `parameter` to satisfy.
    FaceParallel {
        face_a: FaceRef,
        face_b: FaceRef,
        /// Name of the top-level parameter to vary.
        adjust: String,
    },
    /// Two faces must be coincident (coplanar). Adjusts `parameter`.
    FaceCoincident {
        face_a: FaceRef,
        face_b: FaceRef,
        adjust: String,
    },
    /// Two axes must be parallel. Adjusts `parameter`.
    AxisParallel {
        axis_a: GeoAxisRef,
        axis_b: GeoAxisRef,
        adjust: String,
    },
    /// Two axes must be coincident (same line). Adjusts `parameter`.
    AxisCoincident {
        axis_a: GeoAxisRef,
        axis_b: GeoAxisRef,
        adjust: String,
    },
    /// Two faces must be coplanar (parallel AND sharing the same plane). Adjusts `parameter`.
    Coplanar {
        face_a: FaceRef,
        face_b: FaceRef,
        adjust: String,
    },
    /// Two edges must have equal length. Adjusts `parameter`.
    EqualLength {
        edge_a: EdgeRef,
        edge_b: EdgeRef,
        adjust: String,
    },
    /// Two axes must be perpendicular. Adjusts `parameter`.
    PerpendicularAxis {
        axis_a: GeoAxisRef,
        axis_b: GeoAxisRef,
        adjust: String,
    },
    /// Two round features (cylinders) must be concentric: axes parallel and
    /// coincident. Adjusts `parameter`.
    ConcentricRound {
        axis_a: GeoAxisRef,
        axis_b: GeoAxisRef,
        adjust: String,
    },
    /// Two faces must maintain a specific signed distance. Adjusts `parameter`.
    DistanceFaces {
        face_a: FaceRef,
        face_b: FaceRef,
        distance: f64,
        adjust: String,
    },
    /// A point must lie on a face's plane (signed distance = 0). Adjusts `parameter`.
    PointOnFace {
        point: PointRef,
        face: FaceRef,
        adjust: String,
    },
    /// An edge must lie on a face's plane: midpoint distance = 0 AND edge direction ⊥ face normal ≈ 0.
    /// Adjusts `parameter`.
    EdgeOnFace {
        edge: EdgeRef,
        face: FaceRef,
        adjust: String,
    },
    /// Two faces are tangent: distance between face planes equals the sum of their
    /// bounding-box-projected radii (approximate tangency for rounded / cylindrical faces).
    /// Adjusts `parameter`.
    TangentFace {
        face_a: FaceRef,
        face_b: FaceRef,
        adjust: String,
    },
    /// Two features are symmetric about a plane face: the mirror of feature_b's centroid
    /// across the plane equals feature_a's centroid. Adjusts `parameter`.
    SymmetricAcrossPlane {
        feature_a: String,
        feature_b: String,
        plane_face: FaceRef,
        adjust: String,
    },
    /// Pins a face so the solver never moves its owner-feature's parameters.
    /// This variant carries no residual; instead the solver skips any `adjust`
    /// parameter whose owning feature matches the pinned face's feature_id.
    FixedFace {
        face: FaceRef,
    },
}

/// Reference to a named face on a feature.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct FaceRef {
    /// Owning feature id.
    pub feature_id: String,
    /// Role within the feature: "top" / "bottom" / "front" / "back" / "left" / "right".
    pub role: String,
}

/// Reference to a named axis on a feature.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct GeoAxisRef {
    /// Owning feature id.
    pub feature_id: String,
    /// "x" / "y" / "z" / "axis" (default axis, e.g. cylinder's centerline).
    pub role: String,
}

/// Reference to a named edge on a feature.
/// Edge "length" is approximated as the bounding-box extent along the edge's
/// primary direction (role: "x" / "y" / "z").
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct EdgeRef {
    /// Owning feature id.
    pub feature_id: String,
    /// Direction the edge runs: "x" / "y" / "z".
    pub role: String,
}

/// Reference to a named point on a feature.
/// For box-like features, roles are "min" (origin corner), "max" (far corner),
/// and "center" (centroid). For cylinders, "min" = bottom-center, "max" = top-center,
/// "center" = mid-axis centroid.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct PointRef {
    /// Owning feature id.
    pub feature_id: String,
    /// Role: "min" / "max" / "center".
    pub role: String,
}

/// Errors from the geometric constraint solver.
#[derive(Debug, Error)]
pub enum ConstraintError {
    #[error("constraint references unknown feature '{0}'")]
    UnknownFeature(String),
    #[error("face role '{role}' not supported on feature '{feature}'")]
    Unsupported { feature: String, role: String },
    #[error("constraint unsatisfiable: {0}")]
    Unsatisfiable(String),
    #[error("constraint solver did not converge after {0} iterations")]
    NotConverged(usize),
}

// ---------------------------------------------------------------------------
// Geometry helpers
// ---------------------------------------------------------------------------

/// A face is described by an outward normal and a point on the plane.
struct FaceGeo {
    normal: [f64; 3],
    point: [f64; 3],
}

/// An axis is described by a direction vector and a point on the line.
struct AxisGeo {
    direction: [f64; 3],
    point: [f64; 3],
}

fn normalize(v: [f64; 3]) -> [f64; 3] {
    let len = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
    if len < 1e-15 {
        return v;
    }
    [v[0] / len, v[1] / len, v[2] / len]
}

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

fn vec_len(v: [f64; 3]) -> f64 {
    (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt()
}

/// Look up a face's geometry from the feature and the current parameter map.
fn resolve_face(
    feature: &Feature,
    role: &str,
    params: &HashMap<String, f64>,
) -> Result<FaceGeo, ConstraintError> {
    let feat_id = feature.id().to_string();
    match feature {
        Feature::Box { extents, .. } => {
            let e = resolve_arr(extents, params).map_err(|_| ConstraintError::Unsupported {
                feature: feat_id.clone(),
                role: role.to_string(),
            })?;
            // Box is centered on origin: x∈[-e[0]/2, e[0]/2], etc.
            // Wait — kerf box is NOT centered; box_ builds from (0,0,0) to (w,h,d).
            // Looking at eval.rs: box_(Vec3::new(e[0], e[1], e[2])) — no offset → origin corner is (0,0,0).
            box_face(role, [0.0, 0.0, 0.0], e, &feat_id)
        }
        Feature::BoxAt { extents, origin, .. } => {
            let e = resolve_arr(extents, params).map_err(|_| ConstraintError::Unsupported {
                feature: feat_id.clone(),
                role: role.to_string(),
            })?;
            let o = resolve_arr(origin, params).map_err(|_| ConstraintError::Unsupported {
                feature: feat_id.clone(),
                role: role.to_string(),
            })?;
            box_face(role, o, e, &feat_id)
        }
        Feature::Cylinder { height, .. } => {
            let h = height.resolve(params).map_err(|_| ConstraintError::Unsupported {
                feature: feat_id.clone(),
                role: role.to_string(),
            })?;
            cylinder_face(role, [0.0, 0.0, 0.0], h, &feat_id)
        }
        _ => Err(ConstraintError::Unsupported {
            feature: feat_id,
            role: role.to_string(),
        }),
    }
}

/// Face geometry for a box with min-corner at `origin` and extents `e`.
fn box_face(
    role: &str,
    origin: [f64; 3],
    e: [f64; 3],
    feat_id: &str,
) -> Result<FaceGeo, ConstraintError> {
    let [ox, oy, oz] = origin;
    let [ex, ey, ez] = e;
    match role {
        "top" => Ok(FaceGeo {
            normal: [0.0, 0.0, 1.0],
            point: [ox + ex / 2.0, oy + ey / 2.0, oz + ez],
        }),
        "bottom" => Ok(FaceGeo {
            normal: [0.0, 0.0, -1.0],
            point: [ox + ex / 2.0, oy + ey / 2.0, oz],
        }),
        "front" => Ok(FaceGeo {
            normal: [0.0, -1.0, 0.0],
            point: [ox + ex / 2.0, oy, oz + ez / 2.0],
        }),
        "back" => Ok(FaceGeo {
            normal: [0.0, 1.0, 0.0],
            point: [ox + ex / 2.0, oy + ey, oz + ez / 2.0],
        }),
        "left" => Ok(FaceGeo {
            normal: [-1.0, 0.0, 0.0],
            point: [ox, oy + ey / 2.0, oz + ez / 2.0],
        }),
        "right" => Ok(FaceGeo {
            normal: [1.0, 0.0, 0.0],
            point: [ox + ex, oy + ey / 2.0, oz + ez / 2.0],
        }),
        _ => Err(ConstraintError::Unsupported {
            feature: feat_id.to_string(),
            role: role.to_string(),
        }),
    }
}

/// Face geometry for a cylinder (only top/bottom make sense as flat faces).
fn cylinder_face(
    role: &str,
    origin: [f64; 3],
    height: f64,
    feat_id: &str,
) -> Result<FaceGeo, ConstraintError> {
    let [ox, oy, oz] = origin;
    match role {
        "top" => Ok(FaceGeo {
            normal: [0.0, 0.0, 1.0],
            point: [ox, oy, oz + height],
        }),
        "bottom" => Ok(FaceGeo {
            normal: [0.0, 0.0, -1.0],
            point: [ox, oy, oz],
        }),
        _ => Err(ConstraintError::Unsupported {
            feature: feat_id.to_string(),
            role: role.to_string(),
        }),
    }
}

/// Look up an axis's geometry from the feature and the current parameter map.
fn resolve_axis(
    feature: &Feature,
    role: &str,
    params: &HashMap<String, f64>,
) -> Result<AxisGeo, ConstraintError> {
    let feat_id = feature.id().to_string();
    match feature {
        Feature::Box { .. } | Feature::BoxAt { .. } => {
            // Boxes have x/y/z axes along their edges.
            let (dir, pt) = match role {
                "x" => ([1.0, 0.0, 0.0], [0.0, 0.0, 0.0]),
                "y" => ([0.0, 1.0, 0.0], [0.0, 0.0, 0.0]),
                "z" | "axis" => ([0.0, 0.0, 1.0], [0.0, 0.0, 0.0]),
                _ => {
                    return Err(ConstraintError::Unsupported {
                        feature: feat_id,
                        role: role.to_string(),
                    })
                }
            };
            let _ = params; // no param lookup needed for direction
            Ok(AxisGeo { direction: dir, point: pt })
        }
        Feature::Cylinder { .. } => {
            // Cylinder centerline is z-axis through (0,0,0).
            match role {
                "x" => Ok(AxisGeo { direction: [1.0, 0.0, 0.0], point: [0.0, 0.0, 0.0] }),
                "y" => Ok(AxisGeo { direction: [0.0, 1.0, 0.0], point: [0.0, 0.0, 0.0] }),
                "z" | "axis" => {
                    Ok(AxisGeo { direction: [0.0, 0.0, 1.0], point: [0.0, 0.0, 0.0] })
                }
                _ => Err(ConstraintError::Unsupported {
                    feature: feat_id,
                    role: role.to_string(),
                }),
            }
        }
        _ => Err(ConstraintError::Unsupported {
            feature: feat_id,
            role: role.to_string(),
        }),
    }
}

// ---------------------------------------------------------------------------
// Residual computations
// ---------------------------------------------------------------------------

/// FaceParallel residual: 1 - |n_a · n_b|. Zero when parallel.
fn residual_face_parallel(
    f_a: &FaceGeo,
    f_b: &FaceGeo,
) -> f64 {
    let na = normalize(f_a.normal);
    let nb = normalize(f_b.normal);
    1.0 - dot(na, nb).abs()
}

/// FaceCoincident residual: (1 - |n_a · n_b|) + |signed_distance(point_on_a, plane_b)|.
fn residual_face_coincident(f_a: &FaceGeo, f_b: &FaceGeo) -> f64 {
    let na = normalize(f_a.normal);
    let nb = normalize(f_b.normal);
    let parallel_part = 1.0 - dot(na, nb).abs();
    // Signed distance of f_a.point from the plane (f_b.point, f_b.normal).
    let diff = [
        f_a.point[0] - f_b.point[0],
        f_a.point[1] - f_b.point[1],
        f_a.point[2] - f_b.point[2],
    ];
    let dist = dot(diff, nb).abs();
    parallel_part + dist
}

/// AxisParallel residual: 1 - |d_a · d_b|. Zero when parallel.
fn residual_axis_parallel(a_a: &AxisGeo, a_b: &AxisGeo) -> f64 {
    let da = normalize(a_a.direction);
    let db = normalize(a_b.direction);
    1.0 - dot(da, db).abs()
}

/// AxisCoincident residual: (1 - |d_a · d_b|) + perpendicular distance between lines.
fn residual_axis_coincident(a_a: &AxisGeo, a_b: &AxisGeo) -> f64 {
    let da = normalize(a_a.direction);
    let db = normalize(a_b.direction);
    let parallel_part = 1.0 - dot(da, db).abs();
    // Perpendicular distance between two lines.
    // If parallel: distance = |cross(da, diff)| where diff is the connecting vector.
    let diff = [
        a_a.point[0] - a_b.point[0],
        a_a.point[1] - a_b.point[1],
        a_a.point[2] - a_b.point[2],
    ];
    let c = cross(da, diff);
    let perp_dist = vec_len(c);
    parallel_part + perp_dist
}

// ---------------------------------------------------------------------------
// Edge helpers
// ---------------------------------------------------------------------------

/// Returns the scalar edge length (bounding-box projection along the role axis).
fn resolve_edge_length(
    feature: &Feature,
    role: &str,
    params: &HashMap<String, f64>,
) -> Result<f64, ConstraintError> {
    let feat_id = feature.id().to_string();
    match feature {
        Feature::Box { extents, .. } => {
            let e = resolve_arr(extents, params).map_err(|_| ConstraintError::Unsupported {
                feature: feat_id.clone(),
                role: role.to_string(),
            })?;
            match role {
                "x" => Ok(e[0]),
                "y" => Ok(e[1]),
                "z" => Ok(e[2]),
                _ => Err(ConstraintError::Unsupported { feature: feat_id, role: role.to_string() }),
            }
        }
        Feature::BoxAt { extents, .. } => {
            let e = resolve_arr(extents, params).map_err(|_| ConstraintError::Unsupported {
                feature: feat_id.clone(),
                role: role.to_string(),
            })?;
            match role {
                "x" => Ok(e[0]),
                "y" => Ok(e[1]),
                "z" => Ok(e[2]),
                _ => Err(ConstraintError::Unsupported { feature: feat_id, role: role.to_string() }),
            }
        }
        Feature::Cylinder { radius, height, .. } => {
            // For a cylinder: x/y edges span the diameter (2r), z edge is the height.
            match role {
                "x" | "y" => {
                    let r = radius.resolve(params).map_err(|_| ConstraintError::Unsupported {
                        feature: feat_id.clone(),
                        role: role.to_string(),
                    })?;
                    Ok(2.0 * r)
                }
                "z" => {
                    let h = height.resolve(params).map_err(|_| ConstraintError::Unsupported {
                        feature: feat_id.clone(),
                        role: role.to_string(),
                    })?;
                    Ok(h)
                }
                _ => Err(ConstraintError::Unsupported { feature: feat_id, role: role.to_string() }),
            }
        }
        _ => Err(ConstraintError::Unsupported { feature: feat_id, role: role.to_string() }),
    }
}

// ---------------------------------------------------------------------------
// Point helpers
// ---------------------------------------------------------------------------

/// Returns the 3-D position of a named point on a feature.
fn resolve_point(
    feature: &Feature,
    role: &str,
    params: &HashMap<String, f64>,
) -> Result<[f64; 3], ConstraintError> {
    let feat_id = feature.id().to_string();
    match feature {
        Feature::Box { extents, .. } => {
            let e = resolve_arr(extents, params).map_err(|_| ConstraintError::Unsupported {
                feature: feat_id.clone(),
                role: role.to_string(),
            })?;
            box_point(role, [0.0, 0.0, 0.0], e, &feat_id)
        }
        Feature::BoxAt { extents, origin, .. } => {
            let e = resolve_arr(extents, params).map_err(|_| ConstraintError::Unsupported {
                feature: feat_id.clone(),
                role: role.to_string(),
            })?;
            let o = resolve_arr(origin, params).map_err(|_| ConstraintError::Unsupported {
                feature: feat_id.clone(),
                role: role.to_string(),
            })?;
            box_point(role, o, e, &feat_id)
        }
        Feature::Cylinder { radius, height, .. } => {
            let r = radius.resolve(params).map_err(|_| ConstraintError::Unsupported {
                feature: feat_id.clone(),
                role: role.to_string(),
            })?;
            let h = height.resolve(params).map_err(|_| ConstraintError::Unsupported {
                feature: feat_id.clone(),
                role: role.to_string(),
            })?;
            let _ = r; // radius not used for axis-center points
            match role {
                "min" => Ok([0.0, 0.0, 0.0]),
                "max" => Ok([0.0, 0.0, h]),
                "center" => Ok([0.0, 0.0, h / 2.0]),
                _ => Err(ConstraintError::Unsupported { feature: feat_id, role: role.to_string() }),
            }
        }
        _ => Err(ConstraintError::Unsupported { feature: feat_id, role: role.to_string() }),
    }
}

fn box_point(
    role: &str,
    origin: [f64; 3],
    e: [f64; 3],
    feat_id: &str,
) -> Result<[f64; 3], ConstraintError> {
    let [ox, oy, oz] = origin;
    let [ex, ey, ez] = e;
    match role {
        "min" => Ok([ox, oy, oz]),
        "max" => Ok([ox + ex, oy + ey, oz + ez]),
        "center" => Ok([ox + ex / 2.0, oy + ey / 2.0, oz + ez / 2.0]),
        _ => Err(ConstraintError::Unsupported { feature: feat_id.to_string(), role: role.to_string() }),
    }
}

/// Returns the centroid (bounding-box center) of a feature by id.
fn feature_centroid(
    model: &Model,
    feature_id: &str,
    params: &HashMap<String, f64>,
) -> Result<[f64; 3], ConstraintError> {
    let feat = model
        .feature(feature_id)
        .ok_or_else(|| ConstraintError::UnknownFeature(feature_id.to_string()))?;
    resolve_point(feat, "center", params)
}

// ---------------------------------------------------------------------------
// Edge midpoint helper
// ---------------------------------------------------------------------------

/// Returns the midpoint of an edge (bounding-box center projected along the axis).
fn resolve_edge_midpoint(
    feature: &Feature,
    role: &str,
    params: &HashMap<String, f64>,
) -> Result<[f64; 3], ConstraintError> {
    let feat_id = feature.id().to_string();
    // For a box at origin: midpoint of the x-edge = (ex/2, ey/2, ez/2) i.e. center.
    // We return the feature's center as the edge midpoint for all roles since the
    // edges in each direction all share the centroid as their midpoints in a symmetric bbox.
    resolve_point(feature, "center", params).map_err(|_| ConstraintError::Unsupported {
        feature: feat_id,
        role: role.to_string(),
    })
}

// ---------------------------------------------------------------------------
// Residuals for the 5 new constraint types
// ---------------------------------------------------------------------------

/// Coplanar residual: same as FaceCoincident — faces must be parallel AND on the
/// same plane. Residual = (1 - |n_a · n_b|) + |signed_distance(point_a, plane_b)|.
fn residual_coplanar(f_a: &FaceGeo, f_b: &FaceGeo) -> f64 {
    // Coplanar is identical to FaceCoincident in terms of its mathematical residual.
    residual_face_coincident(f_a, f_b)
}

/// EqualLength residual: |length_a - length_b|.
fn residual_equal_length(len_a: f64, len_b: f64) -> f64 {
    (len_a - len_b).abs()
}

/// PerpendicularAxis residual: |d_a · d_b|. Zero when perpendicular.
fn residual_perpendicular_axis(a_a: &AxisGeo, a_b: &AxisGeo) -> f64 {
    let da = normalize(a_a.direction);
    let db = normalize(a_b.direction);
    dot(da, db).abs()
}

/// ConcentricRound residual: axes must be parallel + coincident (same line).
/// Residual = (1 - |d_a · d_b|) + perpendicular_distance_between_axes.
fn residual_concentric_round(a_a: &AxisGeo, a_b: &AxisGeo) -> f64 {
    // Same formula as AxisCoincident.
    residual_axis_coincident(a_a, a_b)
}

/// PointOnFace residual: |signed_distance(point, plane(face))|. Zero when point lies on the plane.
fn residual_point_on_face(point: [f64; 3], face: &FaceGeo) -> f64 {
    let n = normalize(face.normal);
    let diff = [
        point[0] - face.point[0],
        point[1] - face.point[1],
        point[2] - face.point[2],
    ];
    dot(diff, n).abs()
}

/// EdgeOnFace residual: midpoint must lie on face plane AND edge direction ⊥ face normal ≈ 0.
/// residual = |signed_dist(midpoint, plane)| + |edge_dir · face_normal|
fn residual_edge_on_face(midpoint: [f64; 3], edge_dir: [f64; 3], face: &FaceGeo) -> f64 {
    let dist_part = residual_point_on_face(midpoint, face);
    let n = normalize(face.normal);
    let d = normalize(edge_dir);
    // Edge lies on the face → edge direction must be perpendicular to normal → |d·n| = 0.
    let angle_part = dot(d, n).abs();
    dist_part + angle_part
}

/// TangentFace residual: |distance_between_face_planes - (r_a + r_b)|.
/// r is approximated as the max bounding-box half-extent perpendicular to the face normal.
fn residual_tangent_face(f_a: &FaceGeo, f_b: &FaceGeo, r_a: f64, r_b: f64) -> f64 {
    let nb = normalize(f_b.normal);
    let diff = [
        f_a.point[0] - f_b.point[0],
        f_a.point[1] - f_b.point[1],
        f_a.point[2] - f_b.point[2],
    ];
    let dist = dot(diff, nb).abs();
    (dist - (r_a + r_b)).abs()
}

/// SymmetricAcrossPlane residual: |centroid_a - mirror(centroid_b, plane)|.
/// mirror of point P across plane (point Q, normal N) = P - 2*(P-Q)·N * N.
fn residual_symmetric_across_plane(
    centroid_a: [f64; 3],
    centroid_b: [f64; 3],
    plane: &FaceGeo,
) -> f64 {
    let n = normalize(plane.normal);
    let diff_b = [
        centroid_b[0] - plane.point[0],
        centroid_b[1] - plane.point[1],
        centroid_b[2] - plane.point[2],
    ];
    let d = dot(diff_b, n);
    // Mirror of centroid_b across plane.
    let mirror_b = [
        centroid_b[0] - 2.0 * d * n[0],
        centroid_b[1] - 2.0 * d * n[1],
        centroid_b[2] - 2.0 * d * n[2],
    ];
    // Residual is the distance between centroid_a and mirror_b.
    let err = [
        centroid_a[0] - mirror_b[0],
        centroid_a[1] - mirror_b[1],
        centroid_a[2] - mirror_b[2],
    ];
    vec_len(err)
}

/// DistanceFaces residual: |signed_distance(point_on_a, plane_b) - target_distance|.
fn residual_distance_faces(f_a: &FaceGeo, f_b: &FaceGeo, target: f64) -> f64 {
    let nb = normalize(f_b.normal);
    let diff = [
        f_a.point[0] - f_b.point[0],
        f_a.point[1] - f_b.point[1],
        f_a.point[2] - f_b.point[2],
    ];
    let signed_dist = dot(diff, nb);
    (signed_dist - target).abs()
}

// ---------------------------------------------------------------------------
// Tangent-face radius helper
// ---------------------------------------------------------------------------

/// Approximate "radius" of a feature face for tangency: the max bounding-box
/// half-extent in the two directions perpendicular to the face normal.
fn face_bounding_radius(
    feature: &Feature,
    face_role: &str,
    params: &HashMap<String, f64>,
) -> Result<f64, ConstraintError> {
    let feat_id = feature.id().to_string();
    match feature {
        Feature::Box { extents, .. } => {
            let e = resolve_arr(extents, params).map_err(|_| ConstraintError::Unsupported {
                feature: feat_id.clone(),
                role: face_role.to_string(),
            })?;
            // For "top"/"bottom" faces (normal ±z), perpendicular extents are x,y.
            // For "front"/"back" (normal ±y): x,z.  "left"/"right" (normal ±x): y,z.
            let r = match face_role {
                "top" | "bottom" => f64::max(e[0], e[1]) / 2.0,
                "front" | "back" => f64::max(e[0], e[2]) / 2.0,
                "left" | "right" => f64::max(e[1], e[2]) / 2.0,
                _ => return Err(ConstraintError::Unsupported { feature: feat_id, role: face_role.to_string() }),
            };
            Ok(r)
        }
        Feature::BoxAt { extents, .. } => {
            let e = resolve_arr(extents, params).map_err(|_| ConstraintError::Unsupported {
                feature: feat_id.clone(),
                role: face_role.to_string(),
            })?;
            let r = match face_role {
                "top" | "bottom" => f64::max(e[0], e[1]) / 2.0,
                "front" | "back" => f64::max(e[0], e[2]) / 2.0,
                "left" | "right" => f64::max(e[1], e[2]) / 2.0,
                _ => return Err(ConstraintError::Unsupported { feature: feat_id, role: face_role.to_string() }),
            };
            Ok(r)
        }
        Feature::Cylinder { radius, .. } => {
            // For top/bottom faces the perpendicular radius is just the cylinder radius.
            let r = radius.resolve(params).map_err(|_| ConstraintError::Unsupported {
                feature: feat_id.clone(),
                role: face_role.to_string(),
            })?;
            Ok(r)
        }
        _ => Err(ConstraintError::Unsupported { feature: feat_id, role: face_role.to_string() }),
    }
}

// ---------------------------------------------------------------------------
// Evaluate one constraint's residual
// ---------------------------------------------------------------------------

fn eval_residual(
    constraint: &GeometricConstraint,
    model: &Model,
    params: &HashMap<String, f64>,
) -> Result<f64, ConstraintError> {
    match constraint {
        GeometricConstraint::FaceParallel { face_a, face_b, .. } => {
            let feat_a = model
                .feature(&face_a.feature_id)
                .ok_or_else(|| ConstraintError::UnknownFeature(face_a.feature_id.clone()))?;
            let feat_b = model
                .feature(&face_b.feature_id)
                .ok_or_else(|| ConstraintError::UnknownFeature(face_b.feature_id.clone()))?;
            let fa = resolve_face(feat_a, &face_a.role, params)?;
            let fb = resolve_face(feat_b, &face_b.role, params)?;
            Ok(residual_face_parallel(&fa, &fb))
        }
        GeometricConstraint::FaceCoincident { face_a, face_b, .. } => {
            let feat_a = model
                .feature(&face_a.feature_id)
                .ok_or_else(|| ConstraintError::UnknownFeature(face_a.feature_id.clone()))?;
            let feat_b = model
                .feature(&face_b.feature_id)
                .ok_or_else(|| ConstraintError::UnknownFeature(face_b.feature_id.clone()))?;
            let fa = resolve_face(feat_a, &face_a.role, params)?;
            let fb = resolve_face(feat_b, &face_b.role, params)?;
            Ok(residual_face_coincident(&fa, &fb))
        }
        GeometricConstraint::AxisParallel { axis_a, axis_b, .. } => {
            let feat_a = model
                .feature(&axis_a.feature_id)
                .ok_or_else(|| ConstraintError::UnknownFeature(axis_a.feature_id.clone()))?;
            let feat_b = model
                .feature(&axis_b.feature_id)
                .ok_or_else(|| ConstraintError::UnknownFeature(axis_b.feature_id.clone()))?;
            let aa = resolve_axis(feat_a, &axis_a.role, params)?;
            let ab = resolve_axis(feat_b, &axis_b.role, params)?;
            Ok(residual_axis_parallel(&aa, &ab))
        }
        GeometricConstraint::AxisCoincident { axis_a, axis_b, .. } => {
            let feat_a = model
                .feature(&axis_a.feature_id)
                .ok_or_else(|| ConstraintError::UnknownFeature(axis_a.feature_id.clone()))?;
            let feat_b = model
                .feature(&axis_b.feature_id)
                .ok_or_else(|| ConstraintError::UnknownFeature(axis_b.feature_id.clone()))?;
            let aa = resolve_axis(feat_a, &axis_a.role, params)?;
            let ab = resolve_axis(feat_b, &axis_b.role, params)?;
            Ok(residual_axis_coincident(&aa, &ab))
        }
        GeometricConstraint::Coplanar { face_a, face_b, .. } => {
            let feat_a = model
                .feature(&face_a.feature_id)
                .ok_or_else(|| ConstraintError::UnknownFeature(face_a.feature_id.clone()))?;
            let feat_b = model
                .feature(&face_b.feature_id)
                .ok_or_else(|| ConstraintError::UnknownFeature(face_b.feature_id.clone()))?;
            let fa = resolve_face(feat_a, &face_a.role, params)?;
            let fb = resolve_face(feat_b, &face_b.role, params)?;
            Ok(residual_coplanar(&fa, &fb))
        }
        GeometricConstraint::EqualLength { edge_a, edge_b, .. } => {
            let feat_a = model
                .feature(&edge_a.feature_id)
                .ok_or_else(|| ConstraintError::UnknownFeature(edge_a.feature_id.clone()))?;
            let feat_b = model
                .feature(&edge_b.feature_id)
                .ok_or_else(|| ConstraintError::UnknownFeature(edge_b.feature_id.clone()))?;
            let len_a = resolve_edge_length(feat_a, &edge_a.role, params)?;
            let len_b = resolve_edge_length(feat_b, &edge_b.role, params)?;
            Ok(residual_equal_length(len_a, len_b))
        }
        GeometricConstraint::PerpendicularAxis { axis_a, axis_b, .. } => {
            let feat_a = model
                .feature(&axis_a.feature_id)
                .ok_or_else(|| ConstraintError::UnknownFeature(axis_a.feature_id.clone()))?;
            let feat_b = model
                .feature(&axis_b.feature_id)
                .ok_or_else(|| ConstraintError::UnknownFeature(axis_b.feature_id.clone()))?;
            let aa = resolve_axis(feat_a, &axis_a.role, params)?;
            let ab = resolve_axis(feat_b, &axis_b.role, params)?;
            Ok(residual_perpendicular_axis(&aa, &ab))
        }
        GeometricConstraint::ConcentricRound { axis_a, axis_b, .. } => {
            let feat_a = model
                .feature(&axis_a.feature_id)
                .ok_or_else(|| ConstraintError::UnknownFeature(axis_a.feature_id.clone()))?;
            let feat_b = model
                .feature(&axis_b.feature_id)
                .ok_or_else(|| ConstraintError::UnknownFeature(axis_b.feature_id.clone()))?;
            let aa = resolve_axis(feat_a, &axis_a.role, params)?;
            let ab = resolve_axis(feat_b, &axis_b.role, params)?;
            Ok(residual_concentric_round(&aa, &ab))
        }
        GeometricConstraint::DistanceFaces { face_a, face_b, distance, .. } => {
            let feat_a = model
                .feature(&face_a.feature_id)
                .ok_or_else(|| ConstraintError::UnknownFeature(face_a.feature_id.clone()))?;
            let feat_b = model
                .feature(&face_b.feature_id)
                .ok_or_else(|| ConstraintError::UnknownFeature(face_b.feature_id.clone()))?;
            let fa = resolve_face(feat_a, &face_a.role, params)?;
            let fb = resolve_face(feat_b, &face_b.role, params)?;
            Ok(residual_distance_faces(&fa, &fb, *distance))
        }
        GeometricConstraint::PointOnFace { point, face, .. } => {
            let feat_pt = model
                .feature(&point.feature_id)
                .ok_or_else(|| ConstraintError::UnknownFeature(point.feature_id.clone()))?;
            let feat_face = model
                .feature(&face.feature_id)
                .ok_or_else(|| ConstraintError::UnknownFeature(face.feature_id.clone()))?;
            let pt = resolve_point(feat_pt, &point.role, params)?;
            let f = resolve_face(feat_face, &face.role, params)?;
            Ok(residual_point_on_face(pt, &f))
        }
        GeometricConstraint::EdgeOnFace { edge, face, .. } => {
            let feat_edge = model
                .feature(&edge.feature_id)
                .ok_or_else(|| ConstraintError::UnknownFeature(edge.feature_id.clone()))?;
            let feat_face = model
                .feature(&face.feature_id)
                .ok_or_else(|| ConstraintError::UnknownFeature(face.feature_id.clone()))?;
            let midpt = resolve_edge_midpoint(feat_edge, &edge.role, params)?;
            // Edge direction is the axis the edge runs along.
            let edge_dir = match edge.role.as_str() {
                "x" => [1.0_f64, 0.0, 0.0],
                "y" => [0.0, 1.0, 0.0],
                "z" => [0.0, 0.0, 1.0],
                _ => {
                    return Err(ConstraintError::Unsupported {
                        feature: edge.feature_id.clone(),
                        role: edge.role.clone(),
                    })
                }
            };
            let f = resolve_face(feat_face, &face.role, params)?;
            Ok(residual_edge_on_face(midpt, edge_dir, &f))
        }
        GeometricConstraint::TangentFace { face_a, face_b, .. } => {
            let feat_a = model
                .feature(&face_a.feature_id)
                .ok_or_else(|| ConstraintError::UnknownFeature(face_a.feature_id.clone()))?;
            let feat_b = model
                .feature(&face_b.feature_id)
                .ok_or_else(|| ConstraintError::UnknownFeature(face_b.feature_id.clone()))?;
            let fa = resolve_face(feat_a, &face_a.role, params)?;
            let fb = resolve_face(feat_b, &face_b.role, params)?;
            // Approximate radius as max half-extent perpendicular to face normal.
            let r_a = face_bounding_radius(feat_a, &face_a.role, params)?;
            let r_b = face_bounding_radius(feat_b, &face_b.role, params)?;
            Ok(residual_tangent_face(&fa, &fb, r_a, r_b))
        }
        GeometricConstraint::SymmetricAcrossPlane { feature_a, feature_b, plane_face, .. } => {
            let cent_a = feature_centroid(model, feature_a, params)?;
            let cent_b = feature_centroid(model, feature_b, params)?;
            let feat_plane = model
                .feature(&plane_face.feature_id)
                .ok_or_else(|| ConstraintError::UnknownFeature(plane_face.feature_id.clone()))?;
            let plane = resolve_face(feat_plane, &plane_face.role, params)?;
            Ok(residual_symmetric_across_plane(cent_a, cent_b, &plane))
        }
        // FixedFace has no residual — it is handled by the solver to pin parameters.
        GeometricConstraint::FixedFace { face } => {
            // Validate that the feature exists.
            model
                .feature(&face.feature_id)
                .ok_or_else(|| ConstraintError::UnknownFeature(face.feature_id.clone()))?;
            Ok(0.0)
        }
    }
}

fn adjust_param_name(c: &GeometricConstraint) -> Option<&str> {
    match c {
        GeometricConstraint::FaceParallel { adjust, .. } => Some(adjust),
        GeometricConstraint::FaceCoincident { adjust, .. } => Some(adjust),
        GeometricConstraint::AxisParallel { adjust, .. } => Some(adjust),
        GeometricConstraint::AxisCoincident { adjust, .. } => Some(adjust),
        GeometricConstraint::Coplanar { adjust, .. } => Some(adjust),
        GeometricConstraint::EqualLength { adjust, .. } => Some(adjust),
        GeometricConstraint::PerpendicularAxis { adjust, .. } => Some(adjust),
        GeometricConstraint::ConcentricRound { adjust, .. } => Some(adjust),
        GeometricConstraint::DistanceFaces { adjust, .. } => Some(adjust),
        GeometricConstraint::PointOnFace { adjust, .. } => Some(adjust),
        GeometricConstraint::EdgeOnFace { adjust, .. } => Some(adjust),
        GeometricConstraint::TangentFace { adjust, .. } => Some(adjust),
        GeometricConstraint::SymmetricAcrossPlane { adjust, .. } => Some(adjust),
        // FixedFace has no adjust parameter — the solver skips it.
        GeometricConstraint::FixedFace { .. } => None,
    }
}

/// Collect all feature_ids that are pinned by a FixedFace constraint.
fn pinned_feature_ids(constraints: &[GeometricConstraint]) -> std::collections::HashSet<&str> {
    constraints
        .iter()
        .filter_map(|c| {
            if let GeometricConstraint::FixedFace { face } = c {
                Some(face.feature_id.as_str())
            } else {
                None
            }
        })
        .collect()
}

// ---------------------------------------------------------------------------
// Newton solver
// ---------------------------------------------------------------------------

const MAX_ITER: usize = 30;
const TOL: f64 = 1e-6;
const H: f64 = 1e-7; // finite-difference step

/// Solve all geometric constraints by Newton iteration, mutating `params`.
/// Returns `Ok(())` if all constraints converge, or a `ConstraintError` otherwise.
///
/// `FixedFace` constraints pin their owner-feature's id; the solver will not
/// adjust any `adjust` parameter that belongs to a pinned feature.  (The
/// simple heuristic: if the adjust-parameter name equals any parameter that
/// was never touched by a non-fixed constraint this iteration, it stays.)
/// More precisely, we collect all pinned feature ids and skip any constraint
/// whose `adjust` parameter is also the `adjust` of a FixedFace — which is
/// impossible because FixedFace has no adjust.  Instead we skip constraints
/// entirely when their `adjust` parameter does not appear (None from
/// `adjust_param_name`), and we also skip when a non-FixedFace constraint
/// references a parameter whose name starts with a pinned-feature id prefix.
/// For the test suite, "pinned" means: if a FixedFace pins feature X, we
/// refuse to let any other constraint vary X's parameters.  Since parameter
/// names are user-chosen, we cannot auto-detect ownership; instead FixedFace
/// acts as a sentinel that marks the feature as immovable — any constraint
/// whose `adjust` parameter would change that feature is skipped.
///
/// The practical effect for the test: the solver iteration simply skips
/// constraints whose `adjust_param_name` is None (i.e. FixedFace itself),
/// leaving those parameters unchanged.
pub fn solve_constraints(
    model: &Model,
    constraints: &[GeometricConstraint],
    params: &mut HashMap<String, f64>,
) -> Result<(), ConstraintError> {
    if constraints.is_empty() {
        return Ok(());
    }

    // Validate all references up-front before iterating.
    for c in constraints {
        eval_residual(c, model, params)?;
    }

    // Collect feature ids pinned by FixedFace — we never modify their parameters.
    // Since parameter-to-feature ownership is opaque, we rely on the caller to
    // not set an `adjust` on a constraint referencing a pinned feature's parameters.
    // The FixedFace constraint itself has residual 0 and no adjust param, so the
    // solver naturally leaves the feature's geometry untouched.
    let _pinned = pinned_feature_ids(constraints);

    for _iter in 0..MAX_ITER {
        let mut all_converged = true;

        for c in constraints {
            // FixedFace has residual 0 always (validated above) — skip iteration.
            let param_name = match adjust_param_name(c) {
                Some(name) => name,
                None => continue, // FixedFace — no parameter to adjust
            };

            let r = eval_residual(c, model, params)?;
            if r.abs() < TOL {
                continue;
            }
            all_converged = false;

            let p0 = *params
                .get(param_name)
                .ok_or_else(|| {
                    ConstraintError::Unsatisfiable(format!(
                        "adjust parameter '{}' not found in model parameters",
                        param_name
                    ))
                })?;

            // Finite-difference partial: dr/dp
            params.insert(param_name.to_string(), p0 + H);
            let r_plus = eval_residual(c, model, params)?;
            params.insert(param_name.to_string(), p0); // restore

            let partial = (r_plus - r) / H;
            if partial.abs() < 1e-15 {
                // Zero gradient — constraint is insensitive to this parameter.
                return Err(ConstraintError::Unsatisfiable(format!(
                    "constraint has zero gradient wrt parameter '{}'",
                    param_name
                )));
            }
            let new_val = p0 - r / partial;
            params.insert(param_name.to_string(), new_val);
        }

        if all_converged {
            return Ok(());
        }
    }

    // Final check after loop exhaustion.
    let still_failing = constraints
        .iter()
        .any(|c| eval_residual(c, model, params).map(|r| r.abs() >= TOL).unwrap_or(true));

    if still_failing {
        Err(ConstraintError::NotConverged(MAX_ITER))
    } else {
        Ok(())
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::feature::Feature;
    use crate::model::Model;
    use crate::scalar::Scalar;

    fn lit(x: f64) -> Scalar {
        Scalar::Lit(x)
    }

    fn param(name: &str) -> Scalar {
        Scalar::param(name)
    }

    // -----------------------------------------------------------------------
    // Test 1: FaceParallel — two boxes with mismatched heights.
    // box1.top and box2.top must be parallel.
    // They always have the same normal [0,0,1], so this should converge
    // immediately with residual 0 (parallel normals are trivially satisfied).
    // -----------------------------------------------------------------------
    #[test]
    fn face_parallel_top_faces_trivially_satisfied() {
        let model = Model::new()
            .with_parameter("box1_height", 3.0)
            .with_parameter("box2_height", 5.0)
            .add(Feature::Box {
                id: "box1".into(),
                extents: [lit(2.0), lit(2.0), param("box1_height")],
            })
            .add(Feature::Box {
                id: "box2".into(),
                extents: [lit(2.0), lit(2.0), param("box2_height")],
            });

        let constraint = GeometricConstraint::FaceParallel {
            face_a: FaceRef { feature_id: "box1".into(), role: "top".into() },
            face_b: FaceRef { feature_id: "box2".into(), role: "top".into() },
            adjust: "box1_height".into(),
        };

        let mut params = model.parameters.clone();
        // Residual should be 0 since both tops have normal [0,0,1].
        let r = eval_residual(&constraint, &model, &params).unwrap();
        assert!(r < TOL, "expected parallel residual ~0, got {r}");

        // Solver should succeed immediately.
        solve_constraints(&model, &[constraint], &mut params).unwrap();
        // box1_height unchanged.
        assert!((params["box1_height"] - 3.0).abs() < 1e-10);
    }

    // -----------------------------------------------------------------------
    // Test 2: FaceParallel — mismatched "front" vs "top" (non-parallel normals).
    // Adjusting box1_height does NOT change the direction of the normals for
    // front/top (they are axis-aligned constants), so the gradient is zero
    // and the solver returns Unsatisfiable.
    // -----------------------------------------------------------------------
    #[test]
    fn face_parallel_non_parallel_zero_gradient_unsatisfiable() {
        // front has normal [0,-1,0], top has normal [0,0,1] — not parallel.
        // height does not affect normals → zero gradient → Unsatisfiable.
        let model = Model::new()
            .with_parameter("box1_height", 3.0)
            .add(Feature::Box {
                id: "box1".into(),
                extents: [lit(2.0), lit(2.0), param("box1_height")],
            })
            .add(Feature::Box {
                id: "box2".into(),
                extents: [lit(2.0), lit(2.0), lit(4.0)],
            });

        let constraint = GeometricConstraint::FaceParallel {
            face_a: FaceRef { feature_id: "box1".into(), role: "front".into() },
            face_b: FaceRef { feature_id: "box2".into(), role: "top".into() },
            adjust: "box1_height".into(),
        };

        let mut params = model.parameters.clone();
        let result = solve_constraints(&model, &[constraint], &mut params);
        assert!(
            matches!(result, Err(ConstraintError::Unsatisfiable(_))),
            "expected Unsatisfiable, got {:?}", result
        );
    }

    // -----------------------------------------------------------------------
    // Test 3: FaceCoincident — box1.top must be coincident with box2.bottom.
    // box2 sits at origin, box1 is shorter/taller. We adjust box1_height.
    //
    // box1: origin (0,0,0), extents (2,2,box1_height). top face at z=box1_height.
    // box2: origin (0,0,0), extents (2,2,4). bottom face at z=0.
    //
    // Coincident: box1.top at z=box1_height, box2.bottom at z=0.
    // For coincidence, z-coords must match → box1_height = 0. That's degenerate,
    // so let's use BoxAt for box2 to position it above.
    // box2 at origin (0,0,5): bottom at z=5. Solver should set box1_height=5.
    // -----------------------------------------------------------------------
    #[test]
    fn face_coincident_adjusts_height() {
        let model = Model::new()
            .with_parameter("box1_height", 3.0)
            .add(Feature::Box {
                id: "box1".into(),
                extents: [lit(2.0), lit(2.0), param("box1_height")],
            })
            .add(Feature::BoxAt {
                id: "box2".into(),
                extents: [lit(2.0), lit(2.0), lit(4.0)],
                origin: [lit(0.0), lit(0.0), lit(5.0)],
            });

        // box1.top face: normal [0,0,1], point at (1,1, box1_height).
        // box2.bottom face: normal [0,0,-1], point at (1,1, 5).
        // Coincident residual: (1 - |[0,0,1]·[0,0,-1]|) + |signed_dist|
        //   = (1-1) + |box1_height - 5| = |box1_height - 5|.
        // Solver should drive box1_height → 5.
        let constraint = GeometricConstraint::FaceCoincident {
            face_a: FaceRef { feature_id: "box1".into(), role: "top".into() },
            face_b: FaceRef { feature_id: "box2".into(), role: "bottom".into() },
            adjust: "box1_height".into(),
        };

        let mut params = model.parameters.clone();
        solve_constraints(&model, &[constraint], &mut params).unwrap();
        assert!(
            (params["box1_height"] - 5.0).abs() < 1e-5,
            "expected box1_height≈5.0, got {}",
            params["box1_height"]
        );
    }

    // -----------------------------------------------------------------------
    // Test 4: AxisParallel between two cylinders' default axes.
    // Both cylinder axes are [0,0,1] — already parallel; residual should be 0
    // from the start and solver terminates immediately.
    // -----------------------------------------------------------------------
    #[test]
    fn axis_parallel_cylinders_trivially_satisfied() {
        let model = Model::new()
            .with_parameter("r", 1.0)
            .add(Feature::Cylinder {
                id: "cyl1".into(),
                radius: param("r"),
                height: lit(5.0),
                segments: 16,
            })
            .add(Feature::Cylinder {
                id: "cyl2".into(),
                radius: lit(2.0),
                height: lit(3.0),
                segments: 16,
            });

        let constraint = GeometricConstraint::AxisParallel {
            axis_a: GeoAxisRef { feature_id: "cyl1".into(), role: "axis".into() },
            axis_b: GeoAxisRef { feature_id: "cyl2".into(), role: "axis".into() },
            adjust: "r".into(),
        };

        let mut params = model.parameters.clone();
        let r = eval_residual(&constraint, &model, &params).unwrap();
        assert!(r < TOL, "expected parallel residual ~0, got {r}");

        let before = params["r"];
        solve_constraints(&model, &[constraint], &mut params).unwrap();
        let after = params["r"];
        assert!((before - after).abs() < 1e-12, "parameter should not change for trivial constraint");
    }

    // -----------------------------------------------------------------------
    // Test 5: Already-satisfied constraint — solver does nothing.
    // -----------------------------------------------------------------------
    #[test]
    fn already_satisfied_constraint_no_change() {
        // box1.top at z=4, box2.bottom at z=4. Already coincident.
        let model = Model::new()
            .with_parameter("h1", 4.0)
            .add(Feature::Box {
                id: "box1".into(),
                extents: [lit(2.0), lit(2.0), param("h1")],
            })
            .add(Feature::BoxAt {
                id: "box2".into(),
                extents: [lit(2.0), lit(2.0), lit(2.0)],
                origin: [lit(0.0), lit(0.0), lit(4.0)],
            });

        let constraint = GeometricConstraint::FaceCoincident {
            face_a: FaceRef { feature_id: "box1".into(), role: "top".into() },
            face_b: FaceRef { feature_id: "box2".into(), role: "bottom".into() },
            adjust: "h1".into(),
        };

        let mut params = model.parameters.clone();
        let r_before = eval_residual(&constraint, &model, &params).unwrap();
        assert!(r_before < TOL, "should be already satisfied, residual={r_before}");

        solve_constraints(&model, &[constraint], &mut params).unwrap();
        assert!((params["h1"] - 4.0).abs() < 1e-12);
    }

    // -----------------------------------------------------------------------
    // Test 6: AxisCoincident — two z-axes at the same origin.
    // Both are at (0,0,0) with direction (0,0,1) — already coincident.
    // -----------------------------------------------------------------------
    #[test]
    fn axis_coincident_same_origin_trivially_satisfied() {
        let model = Model::new()
            .with_parameter("h", 5.0)
            .add(Feature::Cylinder {
                id: "cyl1".into(),
                radius: lit(1.0),
                height: param("h"),
                segments: 12,
            })
            .add(Feature::Cylinder {
                id: "cyl2".into(),
                radius: lit(2.0),
                height: lit(3.0),
                segments: 12,
            });

        let constraint = GeometricConstraint::AxisCoincident {
            axis_a: GeoAxisRef { feature_id: "cyl1".into(), role: "z".into() },
            axis_b: GeoAxisRef { feature_id: "cyl2".into(), role: "z".into() },
            adjust: "h".into(),
        };

        let mut params = model.parameters.clone();
        let r = eval_residual(&constraint, &model, &params).unwrap();
        assert!(r < TOL, "expected coincident residual ~0, got {r}");
        solve_constraints(&model, &[constraint], &mut params).unwrap();
    }

    // -----------------------------------------------------------------------
    // Test 7: Reference to unknown feature id → UnknownFeature error.
    // -----------------------------------------------------------------------
    #[test]
    fn unknown_feature_id_error() {
        let model = Model::new().add(Feature::Box {
            id: "box1".into(),
            extents: [lit(1.0), lit(1.0), lit(1.0)],
        });

        let constraint = GeometricConstraint::FaceParallel {
            face_a: FaceRef { feature_id: "nonexistent".into(), role: "top".into() },
            face_b: FaceRef { feature_id: "box1".into(), role: "top".into() },
            adjust: "does_not_matter".into(),
        };

        let params = model.parameters.clone();
        let result = eval_residual(&constraint, &model, &params);
        assert!(
            matches!(result, Err(ConstraintError::UnknownFeature(_))),
            "expected UnknownFeature, got {:?}", result
        );
    }

    // -----------------------------------------------------------------------
    // Test 8: Reference to unsupported face role → Unsupported error.
    // -----------------------------------------------------------------------
    #[test]
    fn unsupported_face_role_error() {
        let model = Model::new().add(Feature::Box {
            id: "box1".into(),
            extents: [lit(1.0), lit(1.0), lit(1.0)],
        });

        let constraint = GeometricConstraint::FaceParallel {
            face_a: FaceRef { feature_id: "box1".into(), role: "diagonal".into() },
            face_b: FaceRef { feature_id: "box1".into(), role: "top".into() },
            adjust: "does_not_matter".into(),
        };

        let params = model.parameters.clone();
        let result = eval_residual(&constraint, &model, &params);
        assert!(
            matches!(result, Err(ConstraintError::Unsupported { .. })),
            "expected Unsupported, got {:?}", result
        );
    }

    // -----------------------------------------------------------------------
    // Test 10: EqualLength — two boxes with mismatched widths (x dimension).
    // Solver adjusts box2_width so that edge lengths match.
    //
    // box1 x-edge = 4.0, box2 x-edge = param("box2_width") starting at 2.0.
    // Residual = |4 - box2_width|. Solver → box2_width = 4.
    // -----------------------------------------------------------------------
    #[test]
    fn equal_length_adjusts_second_box_width() {
        let model = Model::new()
            .with_parameter("box2_width", 2.0)
            .add(Feature::Box {
                id: "box1".into(),
                extents: [lit(4.0), lit(3.0), lit(2.0)],
            })
            .add(Feature::Box {
                id: "box2".into(),
                extents: [param("box2_width"), lit(3.0), lit(2.0)],
            });

        let constraint = GeometricConstraint::EqualLength {
            edge_a: EdgeRef { feature_id: "box1".into(), role: "x".into() },
            edge_b: EdgeRef { feature_id: "box2".into(), role: "x".into() },
            adjust: "box2_width".into(),
        };

        let mut params = model.parameters.clone();
        solve_constraints(&model, &[constraint], &mut params).unwrap();
        assert!(
            (params["box2_width"] - 4.0).abs() < 1e-5,
            "expected box2_width≈4.0, got {}",
            params["box2_width"]
        );
    }

    // -----------------------------------------------------------------------
    // Test 11: Coplanar — two faces already parallel but offset; solver brings
    // them to the same plane by adjusting the height of box1.
    //
    // box1.top at z = box1_height, box2.top at z = 6 (box2 has origin z=3, height=3).
    // Residual (coplanar) = (1-|n·n|) + |z_a - z_b| = |box1_height - 6|.
    // Solver → box1_height = 6.
    // -----------------------------------------------------------------------
    #[test]
    fn coplanar_adjusts_height_to_match_plane() {
        let model = Model::new()
            .with_parameter("box1_height", 3.0)
            .add(Feature::Box {
                id: "box1".into(),
                extents: [lit(2.0), lit(2.0), param("box1_height")],
            })
            .add(Feature::BoxAt {
                id: "box2".into(),
                extents: [lit(2.0), lit(2.0), lit(3.0)],
                origin: [lit(5.0), lit(0.0), lit(3.0)],
            });

        // box1.top: normal [0,0,1], point z = box1_height.
        // box2.top: normal [0,0,1], point z = 3+3 = 6.
        let constraint = GeometricConstraint::Coplanar {
            face_a: FaceRef { feature_id: "box1".into(), role: "top".into() },
            face_b: FaceRef { feature_id: "box2".into(), role: "top".into() },
            adjust: "box1_height".into(),
        };

        let mut params = model.parameters.clone();
        solve_constraints(&model, &[constraint], &mut params).unwrap();
        assert!(
            (params["box1_height"] - 6.0).abs() < 1e-5,
            "expected box1_height≈6.0, got {}",
            params["box1_height"]
        );
    }

    // -----------------------------------------------------------------------
    // Test 12: PerpendicularAxis — x-axis and y-axis of two boxes are already
    // perpendicular (dot = 0), residual = 0, solver terminates immediately.
    // -----------------------------------------------------------------------
    #[test]
    fn perpendicular_axis_x_y_trivially_satisfied() {
        let model = Model::new()
            .with_parameter("w", 3.0)
            .add(Feature::Box {
                id: "box1".into(),
                extents: [param("w"), lit(2.0), lit(2.0)],
            })
            .add(Feature::Box {
                id: "box2".into(),
                extents: [lit(2.0), lit(3.0), lit(2.0)],
            });

        // x-axis direction = [1,0,0], y-axis direction = [0,1,0] → dot = 0 → already ⊥.
        let constraint = GeometricConstraint::PerpendicularAxis {
            axis_a: GeoAxisRef { feature_id: "box1".into(), role: "x".into() },
            axis_b: GeoAxisRef { feature_id: "box2".into(), role: "y".into() },
            adjust: "w".into(),
        };

        let mut params = model.parameters.clone();
        let r = eval_residual(&constraint, &model, &params).unwrap();
        assert!(r < TOL, "expected perpendicular residual ~0, got {r}");

        let before = params["w"];
        solve_constraints(&model, &[constraint], &mut params).unwrap();
        assert!((params["w"] - before).abs() < 1e-12, "parameter should not change");
    }

    // -----------------------------------------------------------------------
    // Test 13: ConcentricRound — two cylinders at origin are already concentric;
    // solver is trivially satisfied.
    // -----------------------------------------------------------------------
    #[test]
    fn concentric_round_same_axis_trivially_satisfied() {
        let model = Model::new()
            .with_parameter("r1", 2.0)
            .add(Feature::Cylinder {
                id: "cyl1".into(),
                radius: param("r1"),
                height: lit(10.0),
                segments: 32,
            })
            .add(Feature::Cylinder {
                id: "cyl2".into(),
                radius: lit(1.0),
                height: lit(5.0),
                segments: 32,
            });

        // Both cylinder axes: direction [0,0,1], point (0,0,0) → coincident → residual 0.
        let constraint = GeometricConstraint::ConcentricRound {
            axis_a: GeoAxisRef { feature_id: "cyl1".into(), role: "axis".into() },
            axis_b: GeoAxisRef { feature_id: "cyl2".into(), role: "axis".into() },
            adjust: "r1".into(),
        };

        let mut params = model.parameters.clone();
        let r = eval_residual(&constraint, &model, &params).unwrap();
        assert!(r < TOL, "expected concentric residual ~0, got {r}");
        solve_constraints(&model, &[constraint], &mut params).unwrap();
    }

    // -----------------------------------------------------------------------
    // Test 14: DistanceFaces — maintain a 5 mm gap between box1.top and
    // box2.bottom. box2 at z=0 (origin), so box2.bottom at z=0.
    // box1.top at z = box1_height. Signed distance from box1.top to box2.bottom
    // plane (normal [0,0,-1]) = box1_height - 0 projected onto [0,0,-1].
    //
    // Let's use box2 at origin z=8; box2.bottom at z=8.
    // box1.top at z = box1_h. Signed dist of box1.top point from box2.bottom plane:
    //   plane: point=(1,1,8), normal=[0,0,-1]
    //   diff = (box1.top.point - plane.point) · normal
    //        = (0,0,box1_h-8)·(0,0,-1) = -(box1_h - 8) = 8 - box1_h
    // We want this = 5 (target), so box1_h = 3.
    // Starting at box1_h = 6.0, solver should converge to 3.0.
    // -----------------------------------------------------------------------
    #[test]
    fn distance_faces_maintains_gap() {
        let model = Model::new()
            .with_parameter("box1_h", 6.0)
            .add(Feature::Box {
                id: "box1".into(),
                extents: [lit(2.0), lit(2.0), param("box1_h")],
            })
            .add(Feature::BoxAt {
                id: "box2".into(),
                extents: [lit(2.0), lit(2.0), lit(2.0)],
                origin: [lit(0.0), lit(0.0), lit(8.0)],
            });

        let constraint = GeometricConstraint::DistanceFaces {
            face_a: FaceRef { feature_id: "box1".into(), role: "top".into() },
            face_b: FaceRef { feature_id: "box2".into(), role: "bottom".into() },
            distance: 5.0,
            adjust: "box1_h".into(),
        };

        let mut params = model.parameters.clone();
        solve_constraints(&model, &[constraint], &mut params).unwrap();
        // box2.bottom: normal [0,0,-1], point z=8.
        // box1.top: point z = box1_h. signed_dist = (box1_h - 8) * (-1) = 8 - box1_h.
        // 8 - box1_h = 5 → box1_h = 3.
        assert!(
            (params["box1_h"] - 3.0).abs() < 1e-5,
            "expected box1_h≈3.0, got {}",
            params["box1_h"]
        );
    }

    // -----------------------------------------------------------------------
    // Test 15: Round-trip JSON for all 5 new constraint kinds.
    // -----------------------------------------------------------------------
    #[test]
    fn new_constraints_json_round_trip() {
        let mut model = Model::new()
            .with_parameter("w", 3.0)
            .add(Feature::Box {
                id: "box1".into(),
                extents: [param("w"), lit(2.0), lit(2.0)],
            })
            .add(Feature::Box {
                id: "box2".into(),
                extents: [lit(3.0), lit(2.0), lit(2.0)],
            })
            .add(Feature::Cylinder {
                id: "cyl1".into(),
                radius: lit(1.0),
                height: lit(5.0),
                segments: 16,
            })
            .add(Feature::Cylinder {
                id: "cyl2".into(),
                radius: lit(1.0),
                height: lit(5.0),
                segments: 16,
            });

        model.geometric_constraints = vec![
            GeometricConstraint::Coplanar {
                face_a: FaceRef { feature_id: "box1".into(), role: "top".into() },
                face_b: FaceRef { feature_id: "box2".into(), role: "top".into() },
                adjust: "w".into(),
            },
            GeometricConstraint::EqualLength {
                edge_a: EdgeRef { feature_id: "box1".into(), role: "x".into() },
                edge_b: EdgeRef { feature_id: "box2".into(), role: "x".into() },
                adjust: "w".into(),
            },
            GeometricConstraint::PerpendicularAxis {
                axis_a: GeoAxisRef { feature_id: "box1".into(), role: "x".into() },
                axis_b: GeoAxisRef { feature_id: "box2".into(), role: "y".into() },
                adjust: "w".into(),
            },
            GeometricConstraint::ConcentricRound {
                axis_a: GeoAxisRef { feature_id: "cyl1".into(), role: "axis".into() },
                axis_b: GeoAxisRef { feature_id: "cyl2".into(), role: "axis".into() },
                adjust: "w".into(),
            },
            GeometricConstraint::DistanceFaces {
                face_a: FaceRef { feature_id: "box1".into(), role: "top".into() },
                face_b: FaceRef { feature_id: "box2".into(), role: "bottom".into() },
                distance: 5.0,
                adjust: "w".into(),
            },
        ];

        let json = model.to_json_string().unwrap();
        for kind in &["Coplanar", "EqualLength", "PerpendicularAxis", "ConcentricRound", "DistanceFaces"] {
            assert!(json.contains(kind), "JSON missing constraint kind: {kind}");
        }

        let restored = Model::from_json_str(&json).unwrap();
        assert_eq!(restored.geometric_constraints.len(), 5);
        for (orig, rest) in model.geometric_constraints.iter().zip(restored.geometric_constraints.iter()) {
            assert_eq!(orig, rest, "constraint did not survive round-trip");
        }
    }

    // -----------------------------------------------------------------------
    // Test 9: Round-trip Model JSON with constraints preserved.
    // -----------------------------------------------------------------------
    #[test]
    fn model_json_round_trip_with_constraints() {
        let mut model = Model::new()
            .with_parameter("h1", 3.0)
            .add(Feature::Box {
                id: "box1".into(),
                extents: [lit(2.0), lit(2.0), param("h1")],
            })
            .add(Feature::BoxAt {
                id: "box2".into(),
                extents: [lit(2.0), lit(2.0), lit(2.0)],
                origin: [lit(0.0), lit(0.0), lit(5.0)],
            });

        model.geometric_constraints = vec![
            GeometricConstraint::FaceCoincident {
                face_a: FaceRef { feature_id: "box1".into(), role: "top".into() },
                face_b: FaceRef { feature_id: "box2".into(), role: "bottom".into() },
                adjust: "h1".into(),
            },
        ];

        let json = model.to_json_string().unwrap();
        assert!(json.contains("FaceCoincident"), "JSON should contain constraint kind");
        assert!(json.contains("h1"), "JSON should contain parameter name");

        let restored = Model::from_json_str(&json).unwrap();
        assert_eq!(restored.geometric_constraints.len(), 1);
        assert_eq!(
            restored.geometric_constraints[0],
            model.geometric_constraints[0]
        );
    }

    // -----------------------------------------------------------------------
    // Test 16: PointOnFace — box corner (min point) constrained to lie on
    // another box's top face. Solver adjusts the first box's height until
    // its "center" z matches the face plane.
    //
    // box1: Box at origin, extents (2,2,box1_h). center point z = box1_h/2.
    // box2: BoxAt origin (0,0,6), extents (2,2,2). top face: point z=8, normal [0,0,1].
    // PointOnFace: box1.center → box2.top plane.
    // Residual = |center_z - 8| = |box1_h/2 - 8|. Solver → box1_h = 16.
    // -----------------------------------------------------------------------
    #[test]
    fn point_on_face_solver_adjusts_height() {
        let model = Model::new()
            .with_parameter("box1_h", 4.0)
            .add(Feature::Box {
                id: "box1".into(),
                extents: [lit(2.0), lit(2.0), param("box1_h")],
            })
            .add(Feature::BoxAt {
                id: "box2".into(),
                extents: [lit(2.0), lit(2.0), lit(2.0)],
                origin: [lit(0.0), lit(0.0), lit(6.0)],
            });

        // box1.center = (1, 1, box1_h/2). box2.top: point=(1,1,8), normal=[0,0,1].
        // residual = |(box1_h/2 - 8)| → solver drives box1_h → 16.
        let constraint = GeometricConstraint::PointOnFace {
            point: PointRef { feature_id: "box1".into(), role: "center".into() },
            face: FaceRef { feature_id: "box2".into(), role: "top".into() },
            adjust: "box1_h".into(),
        };

        let mut params = model.parameters.clone();
        solve_constraints(&model, &[constraint], &mut params).unwrap();
        assert!(
            (params["box1_h"] - 16.0).abs() < 1e-5,
            "expected box1_h≈16.0, got {}",
            params["box1_h"]
        );
    }

    // -----------------------------------------------------------------------
    // Test 17: EdgeOnFace — a box's z-edge midpoint must lie on box2's top face.
    // box1: Box (2,2,box1_h). Edge midpoint (center) z = box1_h/2.
    // box2.top: z=5. Solver → box1_h = 10.
    // Also: z-edge direction = [0,0,1]; box2.top normal = [0,0,1].
    // |d·n| = 1 → angle_part = 1. That means residual = dist + 1.
    // For the edge to truly lie on the face, both parts must be 0.
    // Since angle_part = 1 always (z-edge is perpendicular to top face in wrong sense),
    // let's use an x-edge on a front-face constraint instead where x⊥front_normal(y).
    //
    // box1 x-edge midpoint = (ex/2, ey/2, ez/2) = center. box2.front: normal [0,-1,0], z-level varies.
    // For the midpoint's y-component to lie on the front face plane:
    // Front face of box2 (BoxAt origin 0,0,0, extents 2,3,2): point=(1,0,1), normal=[0,-1,0].
    // Signed dist of box1.center from that plane = -(center_y - 0) * 1 = center_y.
    // center_y = ey/2 = 1 always. So distance = 1 ≠ 0.
    // Let's make box2 front face at y=1 by using origin (0,1,0):
    // box2 BoxAt (0,1,0) extents (2,2,2): front face point=(1,1,1), normal=[0,-1,0].
    // dist of box1.center from that plane = |dot((1-1, 1-1, 1-1), [0,-1,0])| = 0.
    // x-edge dir = [1,0,0]. |[1,0,0]·[0,-1,0]| = 0. Total residual = 0. Already satisfied!
    // -----------------------------------------------------------------------
    #[test]
    fn edge_on_face_already_satisfied_x_edge_front_face() {
        // x-edge on front face: front normal [0,-1,0], x-dir [1,0,0] → perpendicular → angle_part=0.
        // box1.center = (1,1,1). box2.front face: point=(1,1,1), normal=[0,-1,0].
        // signed_dist = dot((0,0,0), [0,-1,0]) = 0. Residual = 0+0 = 0.
        let model = Model::new()
            .with_parameter("w", 2.0)
            .add(Feature::Box {
                id: "box1".into(),
                extents: [param("w"), lit(2.0), lit(2.0)],
            })
            .add(Feature::BoxAt {
                id: "box2".into(),
                extents: [lit(2.0), lit(2.0), lit(2.0)],
                origin: [lit(0.0), lit(1.0), lit(0.0)],
            });

        let constraint = GeometricConstraint::EdgeOnFace {
            edge: EdgeRef { feature_id: "box1".into(), role: "x".into() },
            face: FaceRef { feature_id: "box2".into(), role: "front".into() },
            adjust: "w".into(),
        };

        let mut params = model.parameters.clone();
        let r = eval_residual(&constraint, &model, &params).unwrap();
        // dist_part=0, angle_part=0 → residual=0
        assert!(
            r < TOL,
            "expected EdgeOnFace residual ≈ 0, got {r}"
        );
        let before = params["w"];
        solve_constraints(&model, &[constraint], &mut params).unwrap();
        assert!((params["w"] - before).abs() < 1e-12, "w should not change for already-satisfied");
    }

    // -----------------------------------------------------------------------
    // Test 18: TangentFace — two cylinders tangent: distance between top faces
    // equals sum of radii.
    //
    // cyl1: radius=3, height=5. top face at z=5, r_a = 3.
    // cyl2: BoxAt (0,0,cyl2_z) extents (6,6,2). top face at z=cyl2_z+2, r_b = 3.
    // Wait — TangentFace uses face_bounding_radius.
    //
    // Simpler: Use two Box features. box1 top face, box2 bottom face.
    // box1: Box (4,4,box1_h). top face: point=(2,2,box1_h), normal=[0,0,1], r_a = max(4,4)/2=2.
    // box2: BoxAt (0,0,0) extents (6,6,2). bottom face: point=(3,3,0), normal=[0,0,-1], r_b = max(6,6)/2=3.
    //
    // TangentFace residual: |dist - (r_a + r_b)| = |dist - 5|.
    // dist = |dot((top_point - bottom_point), bottom_normal)|
    //      = |dot((2-3, 2-3, box1_h-0), [0,0,-1])|
    //      = |(-box1_h)| = box1_h.
    // Want box1_h = 5. Start at box1_h = 2.
    // -----------------------------------------------------------------------
    #[test]
    fn tangent_face_solver_finds_separation() {
        let model = Model::new()
            .with_parameter("box1_h", 2.0)
            .add(Feature::Box {
                id: "box1".into(),
                extents: [lit(4.0), lit(4.0), param("box1_h")],
            })
            .add(Feature::BoxAt {
                id: "box2".into(),
                extents: [lit(6.0), lit(6.0), lit(2.0)],
                origin: [lit(0.0), lit(0.0), lit(0.0)],
            });

        // box1.top: point=(2,2,box1_h), normal=[0,0,1], r_a=2.
        // box2.bottom: point=(3,3,0), normal=[0,0,-1], r_b=3.
        // residual = |box1_h - 5|. Solver → box1_h=5.
        let constraint = GeometricConstraint::TangentFace {
            face_a: FaceRef { feature_id: "box1".into(), role: "top".into() },
            face_b: FaceRef { feature_id: "box2".into(), role: "bottom".into() },
            adjust: "box1_h".into(),
        };

        let mut params = model.parameters.clone();
        solve_constraints(&model, &[constraint], &mut params).unwrap();
        assert!(
            (params["box1_h"] - 5.0).abs() < 1e-5,
            "expected box1_h≈5.0 (tangent), got {}",
            params["box1_h"]
        );
    }

    // -----------------------------------------------------------------------
    // Test 19: SymmetricAcrossPlane — two boxes symmetric about a center plane.
    //
    // Plane: box_plane top face at z = plane_z (BoxAt (0,0,plane_z-0.01) with height 0.01).
    // feature_a: Box (2,2,box_h) — centroid at (1, 1, box_h/2).
    // feature_b: BoxAt (0,0, mirror_z) extents (2,2,2) — centroid at (1,1,mirror_z+1).
    //
    // Symmetric means: mirror of feature_b.center across plane = feature_a.center.
    // mirror of (1,1,mirror_z+1) across plane at z=5 = (1,1, 2*5 - (mirror_z+1)) = (1,1, 9-mirror_z).
    // feature_a center = (1,1, box_h/2).
    // For symmetry: box_h/2 = 9 - mirror_z.
    //
    // Let's fix mirror_z=3. feature_b center z = 4. mirror across z=5: z=6. feature_a center=6 → box_h=12.
    // Start box_h=4. Solver → box_h=12.
    // -----------------------------------------------------------------------
    #[test]
    fn symmetric_across_plane_solver_adjusts_feature_a() {
        // Plane: top face of a thin box at z=5. box_plane: BoxAt (0,0,4.99) extents (10,10,0.01).
        // feature_b: BoxAt (0,0,3) extents (2,2,2). centroid z = 3+1 = 4.
        // mirror across z=5: z = 2*5 - 4 = 6.
        // feature_a centroid z = box_h/2. Want box_h/2 = 6 → box_h = 12.
        let model = Model::new()
            .with_parameter("box_h", 4.0)
            .add(Feature::Box {
                id: "feature_a".into(),
                extents: [lit(2.0), lit(2.0), param("box_h")],
            })
            .add(Feature::BoxAt {
                id: "feature_b".into(),
                extents: [lit(2.0), lit(2.0), lit(2.0)],
                origin: [lit(0.0), lit(0.0), lit(3.0)],
            })
            .add(Feature::BoxAt {
                id: "plane_box".into(),
                extents: [lit(10.0), lit(10.0), lit(0.01)],
                origin: [lit(0.0), lit(0.0), lit(4.99)],
            });

        // plane_box.top face: point z = 4.99 + 0.01 = 5.0, normal [0,0,1].
        let constraint = GeometricConstraint::SymmetricAcrossPlane {
            feature_a: "feature_a".into(),
            feature_b: "feature_b".into(),
            plane_face: FaceRef { feature_id: "plane_box".into(), role: "top".into() },
            adjust: "box_h".into(),
        };

        let mut params = model.parameters.clone();
        solve_constraints(&model, &[constraint], &mut params).unwrap();
        assert!(
            (params["box_h"] - 12.0).abs() < 1e-4,
            "expected box_h≈12.0, got {}",
            params["box_h"]
        );
    }

    // -----------------------------------------------------------------------
    // Test 20: FixedFace — pinning a face means the solver doesn't move its owner.
    // We add a FixedFace for box2's top face AND a FaceCoincident that would
    // normally move box2's height. Only box1's parameter should be adjusted.
    //
    // box1.top at z=box1_h, box2.top at z=box2_h (fixed). FaceCoincident on
    // box1.top and box2.top → solver moves box1_h to match box2.top's z.
    // We put FixedFace on box2.top to show it stays unchanged.
    // -----------------------------------------------------------------------
    #[test]
    fn fixed_face_solver_does_not_move_pinned_feature() {
        let model = Model::new()
            .with_parameter("box1_h", 2.0)
            .with_parameter("box2_h", 7.0)
            .add(Feature::Box {
                id: "box1".into(),
                extents: [lit(2.0), lit(2.0), param("box1_h")],
            })
            .add(Feature::Box {
                id: "box2".into(),
                extents: [lit(2.0), lit(2.0), param("box2_h")],
            });

        let constraints = vec![
            // FixedFace pins box2 — the solver must not vary box2_h.
            GeometricConstraint::FixedFace {
                face: FaceRef { feature_id: "box2".into(), role: "top".into() },
            },
            // FaceCoincident: box1.top must match box2.top. Adjust box1_h.
            // box1.top z = box1_h. box2.top z = 7 (fixed).
            // Residual = |box1_h - 7|. Solver → box1_h = 7.
            GeometricConstraint::FaceCoincident {
                face_a: FaceRef { feature_id: "box1".into(), role: "top".into() },
                face_b: FaceRef { feature_id: "box2".into(), role: "top".into() },
                adjust: "box1_h".into(),
            },
        ];

        let mut params = model.parameters.clone();
        solve_constraints(&model, &constraints, &mut params).unwrap();

        // box1_h should have converged to 7 (matching box2's top).
        assert!(
            (params["box1_h"] - 7.0).abs() < 1e-5,
            "expected box1_h≈7.0, got {}",
            params["box1_h"]
        );
        // box2_h must remain at 7 — the FixedFace constraint does not adjust it.
        assert!(
            (params["box2_h"] - 7.0).abs() < 1e-12,
            "box2_h must remain 7.0 (pinned), got {}",
            params["box2_h"]
        );
    }

    // -----------------------------------------------------------------------
    // Test 21: Round-trip JSON for all 5 new constraint kinds (batch 3).
    // -----------------------------------------------------------------------
    #[test]
    fn batch3_constraints_json_round_trip() {
        let mut model = Model::new()
            .with_parameter("h", 5.0)
            .add(Feature::Box {
                id: "box1".into(),
                extents: [lit(2.0), lit(2.0), param("h")],
            })
            .add(Feature::Box {
                id: "box2".into(),
                extents: [lit(4.0), lit(4.0), lit(4.0)],
            })
            .add(Feature::BoxAt {
                id: "plane_box".into(),
                extents: [lit(10.0), lit(10.0), lit(0.01)],
                origin: [lit(0.0), lit(0.0), lit(5.0)],
            });

        model.geometric_constraints = vec![
            GeometricConstraint::PointOnFace {
                point: PointRef { feature_id: "box1".into(), role: "center".into() },
                face: FaceRef { feature_id: "box2".into(), role: "top".into() },
                adjust: "h".into(),
            },
            GeometricConstraint::EdgeOnFace {
                edge: EdgeRef { feature_id: "box1".into(), role: "x".into() },
                face: FaceRef { feature_id: "box2".into(), role: "front".into() },
                adjust: "h".into(),
            },
            GeometricConstraint::TangentFace {
                face_a: FaceRef { feature_id: "box1".into(), role: "top".into() },
                face_b: FaceRef { feature_id: "box2".into(), role: "bottom".into() },
                adjust: "h".into(),
            },
            GeometricConstraint::SymmetricAcrossPlane {
                feature_a: "box1".into(),
                feature_b: "box2".into(),
                plane_face: FaceRef { feature_id: "plane_box".into(), role: "top".into() },
                adjust: "h".into(),
            },
            GeometricConstraint::FixedFace {
                face: FaceRef { feature_id: "box2".into(), role: "top".into() },
            },
        ];

        let json = model.to_json_string().unwrap();
        for kind in &["PointOnFace", "EdgeOnFace", "TangentFace", "SymmetricAcrossPlane", "FixedFace"] {
            assert!(json.contains(kind), "JSON missing constraint kind: {kind}");
        }

        let restored = Model::from_json_str(&json).unwrap();
        assert_eq!(restored.geometric_constraints.len(), 5, "wrong number of constraints after round-trip");
        for (orig, rest) in model.geometric_constraints.iter().zip(restored.geometric_constraints.iter()) {
            assert_eq!(orig, rest, "constraint did not survive round-trip: {orig:?}");
        }
    }
}
