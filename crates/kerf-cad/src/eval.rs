//! Evaluator: walks the model DAG depth-first and produces a Solid.

use std::collections::HashMap;

use kerf_brep::{
    primitives::{box_, box_at, cone, cylinder_faceted, extrude_polygon, frustum, sphere, torus},
    Solid,
};
use kerf_geom::{Point3, Vec3};
use thiserror::Error;

use crate::feature::{Feature, Profile2D};
use crate::model::Model;
use crate::scalar::{resolve_arr, Scalar};
use crate::transform::{rotate_solid, translate_solid};

#[derive(Debug, Error)]
pub enum EvalError {
    #[error("unknown feature id: {0}")]
    UnknownId(String),
    #[error("cycle detected involving id: {0}")]
    Cycle(String),
    #[error("boolean op '{op}' on '{id}' failed: {message}")]
    Boolean { id: String, op: &'static str, message: String },
    #[error("invalid feature '{id}': {reason}")]
    Invalid { id: String, reason: String },
    #[error("in feature '{id}': {message}")]
    Parameter { id: String, message: String },
}

impl Model {
    /// Evaluate `target_id` and all its transitive dependencies. Returns the
    /// computed Solid for `target_id`. Intermediate results are cached so
    /// shared sub-DAGs are evaluated once.
    pub fn evaluate(&self, target_id: &str) -> Result<Solid, EvalError> {
        if self.feature(target_id).is_none() {
            return Err(EvalError::UnknownId(target_id.to_string()));
        }
        let mut cache: HashMap<String, Solid> = HashMap::new();
        let mut stack: Vec<String> = Vec::new();
        self.eval_into(target_id, &mut cache, &mut stack)?;
        Ok(cache.remove(target_id).expect("just computed"))
    }

    fn eval_into(
        &self,
        id: &str,
        cache: &mut HashMap<String, Solid>,
        stack: &mut Vec<String>,
    ) -> Result<(), EvalError> {
        if cache.contains_key(id) {
            return Ok(());
        }
        if stack.iter().any(|s| s == id) {
            return Err(EvalError::Cycle(id.to_string()));
        }
        let feature = self
            .feature(id)
            .ok_or_else(|| EvalError::UnknownId(id.to_string()))?;

        stack.push(id.to_string());
        for dep in feature.inputs() {
            self.eval_into(dep, cache, stack)?;
        }
        stack.pop();

        let result = build(feature, &self.parameters, cache)?;
        cache.insert(id.to_string(), result);
        Ok(())
    }
}

fn build(
    feature: &Feature,
    params: &HashMap<String, f64>,
    cache: &HashMap<String, Solid>,
) -> Result<Solid, EvalError> {
    let id = feature.id();
    match feature {
        Feature::Box { extents, .. } => {
            let e = resolve3(id, extents, params)?;
            Ok(box_(Vec3::new(e[0], e[1], e[2])))
        }
        Feature::BoxAt {
            extents, origin, ..
        } => {
            let e = resolve3(id, extents, params)?;
            let o = resolve3(id, origin, params)?;
            Ok(box_at(Vec3::new(e[0], e[1], e[2]), Point3::new(o[0], o[1], o[2])))
        }
        Feature::Cylinder {
            radius,
            height,
            segments,
            ..
        } => Ok(cylinder_faceted(
            resolve_one(id, radius, params)?,
            resolve_one(id, height, params)?,
            *segments,
        )),
        Feature::Sphere { radius, .. } => Ok(sphere(resolve_one(id, radius, params)?)),
        Feature::Torus {
            major_radius,
            minor_radius,
            ..
        } => Ok(torus(
            resolve_one(id, major_radius, params)?,
            resolve_one(id, minor_radius, params)?,
        )),
        Feature::Cone { radius, height, .. } => Ok(cone(
            resolve_one(id, radius, params)?,
            resolve_one(id, height, params)?,
        )),
        Feature::Frustum {
            top_radius,
            bottom_radius,
            height,
            ..
        } => Ok(frustum(
            resolve_one(id, top_radius, params)?,
            resolve_one(id, bottom_radius, params)?,
            resolve_one(id, height, params)?,
        )),
        Feature::ExtrudePolygon {
            profile,
            direction,
            ..
        } => build_extrude(id, profile, direction, params),

        Feature::Translate { input, offset, .. } => {
            let base = cache_get(cache, input)?;
            let o = resolve3(id, offset, params)?;
            Ok(translate_solid(base, Vec3::new(o[0], o[1], o[2])))
        }
        Feature::Rotate {
            input,
            axis,
            angle_deg,
            center,
            ..
        } => {
            let base = cache_get(cache, input)?;
            let a = resolve3(id, axis, params)?;
            let c = resolve3(id, center, params)?;
            let theta = resolve_one(id, angle_deg, params)?;
            let axis_v = Vec3::new(a[0], a[1], a[2]);
            if axis_v.norm() < 1e-12 {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: "rotate axis must be non-zero".into(),
                });
            }
            Ok(rotate_solid(
                base,
                axis_v,
                theta.to_radians(),
                Point3::new(c[0], c[1], c[2]),
            ))
        }

        Feature::Union { inputs, .. } => fold_boolean(id, inputs, cache, BoolKind::Union),
        Feature::Intersection { inputs, .. } => {
            fold_boolean(id, inputs, cache, BoolKind::Intersection)
        }
        Feature::Difference { inputs, .. } => fold_boolean(id, inputs, cache, BoolKind::Difference),
    }
}

fn resolve_one(id: &str, s: &Scalar, params: &HashMap<String, f64>) -> Result<f64, EvalError> {
    s.resolve(params).map_err(|message| EvalError::Parameter {
        id: id.into(),
        message,
    })
}

fn resolve3(
    id: &str,
    arr: &[Scalar; 3],
    params: &HashMap<String, f64>,
) -> Result<[f64; 3], EvalError> {
    resolve_arr(arr, params).map_err(|message| EvalError::Parameter {
        id: id.into(),
        message,
    })
}

fn build_extrude(
    id: &str,
    profile: &Profile2D,
    direction: &[Scalar; 3],
    params: &HashMap<String, f64>,
) -> Result<Solid, EvalError> {
    if profile.points.len() < 3 {
        return Err(EvalError::Invalid {
            id: id.into(),
            reason: format!(
                "extrude profile needs at least 3 points (got {})",
                profile.points.len()
            ),
        });
    }
    let dir3 = resolve3(id, direction, params)?;
    let dir = Vec3::new(dir3[0], dir3[1], dir3[2]);
    if dir.norm() < 1e-12 {
        return Err(EvalError::Invalid {
            id: id.into(),
            reason: "extrude direction must be non-zero".into(),
        });
    }
    let mut pts: Vec<Point3> = Vec::with_capacity(profile.points.len());
    for p in &profile.points {
        let xy = resolve_arr(p, params).map_err(|message| EvalError::Parameter {
            id: id.into(),
            message,
        })?;
        pts.push(Point3::new(xy[0], xy[1], 0.0));
    }
    Ok(extrude_polygon(&pts, dir))
}

#[derive(Clone, Copy)]
enum BoolKind {
    Union,
    Intersection,
    Difference,
}

impl BoolKind {
    fn name(self) -> &'static str {
        match self {
            BoolKind::Union => "union",
            BoolKind::Intersection => "intersection",
            BoolKind::Difference => "difference",
        }
    }
    fn apply(self, a: &Solid, b: &Solid) -> Result<Solid, kerf_brep::BooleanError> {
        match self {
            BoolKind::Union => a.try_union(b),
            BoolKind::Intersection => a.try_intersection(b),
            BoolKind::Difference => a.try_difference(b),
        }
    }
}

fn fold_boolean(
    id: &str,
    inputs: &[String],
    cache: &HashMap<String, Solid>,
    op: BoolKind,
) -> Result<Solid, EvalError> {
    if inputs.len() < 2 {
        return Err(EvalError::Invalid {
            id: id.into(),
            reason: format!(
                "boolean '{}' needs at least 2 inputs (got {})",
                op.name(),
                inputs.len()
            ),
        });
    }
    let mut acc = cache_get(cache, &inputs[0])?.clone();
    for next_id in &inputs[1..] {
        let next = cache_get(cache, next_id)?;
        acc = op.apply(&acc, next).map_err(|e| EvalError::Boolean {
            id: id.into(),
            op: op.name(),
            message: e.message,
        })?;
    }
    Ok(acc)
}

fn cache_get<'a>(cache: &'a HashMap<String, Solid>, id: &str) -> Result<&'a Solid, EvalError> {
    cache
        .get(id)
        .ok_or_else(|| EvalError::UnknownId(id.into()))
}
