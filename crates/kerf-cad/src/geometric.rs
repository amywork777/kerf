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
    }
}

fn adjust_param_name(c: &GeometricConstraint) -> &str {
    match c {
        GeometricConstraint::FaceParallel { adjust, .. } => adjust,
        GeometricConstraint::FaceCoincident { adjust, .. } => adjust,
        GeometricConstraint::AxisParallel { adjust, .. } => adjust,
        GeometricConstraint::AxisCoincident { adjust, .. } => adjust,
    }
}

// ---------------------------------------------------------------------------
// Newton solver
// ---------------------------------------------------------------------------

const MAX_ITER: usize = 30;
const TOL: f64 = 1e-6;
const H: f64 = 1e-7; // finite-difference step

/// Solve all geometric constraints by Newton iteration, mutating `params`.
/// Returns `Ok(())` if all constraints converge, or a `ConstraintError` otherwise.
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

    for _iter in 0..MAX_ITER {
        let mut all_converged = true;

        for c in constraints {
            let r = eval_residual(c, model, params)?;
            if r.abs() < TOL {
                continue;
            }
            all_converged = false;

            let param_name = adjust_param_name(c);
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
}
