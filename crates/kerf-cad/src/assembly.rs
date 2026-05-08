//! Assembly: a collection of `Instance`s of `Model`s, positioned by `Pose`s
//! and (optionally) constrained by `Mate`s.
//!
//! Architecture:
//! - Each `Instance` has an id, a `Model` (or path to one), and an
//!   initial `default_pose` that gives a starting position in world space.
//! - `Mate`s are simple-case constraints (Coincident / Concentric / Distance)
//!   applied **in declaration order**. The solver treats each mate as
//!   "fix instance_a, move instance_b to satisfy this mate" — except the
//!   FIRST time an instance appears as `instance_a`, it stays at its
//!   default pose; the FIRST time it appears as `instance_b`, it gets
//!   moved.
//! - Once an instance has been moved by an earlier mate, a later mate that
//!   tries to move it again is checked for satisfiability against the
//!   existing pose; if it conflicts, the mate is reported as
//!   over-constrained.
//! - This is intentionally NOT a general 6-DOF symbolic solver. It's
//!   enough to express the common cases (lid coincident with base,
//!   shaft concentric with bore) without bringing in a constraint
//!   library.
//!
//! Pose math:
//! - `Pose::translation` is straight x/y/z offset in world space.
//! - `Pose::rotation_axis` + `Pose::rotation_angle` is axis-angle. Applied
//!   FIRST (around the local origin), THEN translation. So a posed point is
//!   `R * p_local + t`.
//!
//! Once mates are solved, `Assembly::evaluate` walks each instance, evaluates
//! its inner `Model` to a `Solid`, and applies the resolved pose using the
//! existing `transform::translate_solid` and `transform::rotate_solid`
//! helpers.

use std::collections::HashMap;

use kerf_brep::Solid;
use kerf_geom::{Point3, Vec3};
use serde::{Deserialize, Serialize};
use thiserror::Error;

use crate::eval::EvalError;
use crate::model::Model;
use crate::scalar::Scalar;
use crate::transform::{rotate_solid, translate_solid};

// ---------------------------------------------------------------------------
// Types
// ---------------------------------------------------------------------------

/// Rigid-body transform in axis-angle + translation form. Default is
/// identity (zero translation, zero rotation around +z).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Pose {
    pub translation: [Scalar; 3],
    /// Axis of rotation. Need not be unit; the solver normalizes.
    /// Zero-length axis means "no rotation" and is allowed.
    pub rotation_axis: [Scalar; 3],
    /// Rotation angle in radians.
    pub rotation_angle: Scalar,
}

impl Pose {
    pub fn identity() -> Self {
        Self {
            translation: [Scalar::lit(0.0), Scalar::lit(0.0), Scalar::lit(0.0)],
            rotation_axis: [Scalar::lit(0.0), Scalar::lit(0.0), Scalar::lit(1.0)],
            rotation_angle: Scalar::lit(0.0),
        }
    }

    pub fn at(x: f64, y: f64, z: f64) -> Self {
        Self {
            translation: [Scalar::lit(x), Scalar::lit(y), Scalar::lit(z)],
            rotation_axis: [Scalar::lit(0.0), Scalar::lit(0.0), Scalar::lit(1.0)],
            rotation_angle: Scalar::lit(0.0),
        }
    }

    /// Resolve every `Scalar` against `params` and return a `ResolvedPose`.
    pub fn resolve(&self, params: &HashMap<String, f64>) -> Result<ResolvedPose, EvalError> {
        let resolve_arr = |arr: &[Scalar; 3]| -> Result<[f64; 3], EvalError> {
            let mut out = [0.0; 3];
            for (i, s) in arr.iter().enumerate() {
                out[i] = s.resolve(params).map_err(|message| EvalError::Parameter {
                    id: "pose".into(),
                    message,
                })?;
            }
            Ok(out)
        };
        let t = resolve_arr(&self.translation)?;
        let axis = resolve_arr(&self.rotation_axis)?;
        let angle = self
            .rotation_angle
            .resolve(params)
            .map_err(|message| EvalError::Parameter {
                id: "pose".into(),
                message,
            })?;
        Ok(ResolvedPose {
            translation: Vec3::new(t[0], t[1], t[2]),
            rotation_axis: Vec3::new(axis[0], axis[1], axis[2]),
            rotation_angle: angle,
        })
    }
}

impl Default for Pose {
    fn default() -> Self {
        Self::identity()
    }
}

/// A `Pose` with all `Scalar`s evaluated to f64s. The "live" representation
/// the solver operates on.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ResolvedPose {
    pub translation: Vec3,
    pub rotation_axis: Vec3,
    pub rotation_angle: f64,
}

impl ResolvedPose {
    pub fn identity() -> Self {
        Self {
            translation: Vec3::zeros(),
            rotation_axis: Vec3::z(),
            rotation_angle: 0.0,
        }
    }

    /// Apply this pose to a point in the instance's local frame: rotate
    /// around the origin (Rodrigues' formula), then translate.
    pub fn apply_point(&self, p: Vec3) -> Vec3 {
        rodrigues(p, self.rotation_axis, self.rotation_angle) + self.translation
    }

    /// Apply this pose to a direction vector: rotation only.
    pub fn apply_vector(&self, v: Vec3) -> Vec3 {
        rodrigues(v, self.rotation_axis, self.rotation_angle)
    }
}

/// Reference to the geometry that an `Instance` shows. Either an inline
/// `Model` or a path to a JSON model on disk (resolved by the host).
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AssemblyRef {
    Inline(Box<Model>),
    Path(String),
}

/// One occurrence of a `Model` in an assembly.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Instance {
    pub id: String,
    pub model: AssemblyRef,
    /// `target_id` inside the inline `Model` to evaluate. If `None`, the
    /// last feature in declaration order is used.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub target: Option<String>,
    pub default_pose: Pose,
}

/// A line in 3-space, expressed as origin + direction. Direction need not
/// be unit — the solver normalizes it.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct AxisRef {
    pub origin: [Scalar; 3],
    pub direction: [Scalar; 3],
}

/// One mate (positioning constraint) between two instances. Applied in
/// declaration order. Points and axes are interpreted in each instance's
/// LOCAL frame; the solver resolves them through the current poses.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind")]
pub enum Mate {
    /// Make `point_a` (in instance_a's local frame, transformed by A's pose)
    /// equal `point_b` (in instance_b's local frame, transformed by B's
    /// pose). Solved by translating B; its rotation is left alone.
    Coincident {
        instance_a: String,
        point_a: [Scalar; 3],
        instance_b: String,
        point_b: [Scalar; 3],
    },
    /// Align `axis_b` to `axis_a`. Solved by ROTATING B so its axis
    /// direction matches A's (around the cross product of the two
    /// directions), then TRANSLATING B so the rotated axis_b origin
    /// lies on axis_a's line.
    Concentric {
        instance_a: String,
        axis_a: AxisRef,
        instance_b: String,
        axis_b: AxisRef,
    },
    /// Place `point_b` at `value` distance from `point_a`, along the
    /// direction from current point_b to current point_a (so the part
    /// moves toward A by the existing approach direction). Translates B
    /// only.
    Distance {
        instance_a: String,
        point_a: [Scalar; 3],
        instance_b: String,
        point_b: [Scalar; 3],
        value: Scalar,
    },
}

/// Top-level container.
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct Assembly {
    pub instances: Vec<Instance>,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub mates: Vec<Mate>,
    /// Named numeric parameters available to every `Pose` and `Mate`
    /// scalar in the assembly. Models inside instances have their own
    /// parameters, evaluated independently.
    #[serde(default, skip_serializing_if = "HashMap::is_empty")]
    pub parameters: HashMap<String, f64>,
}

#[derive(Debug, Error)]
pub enum MateError {
    #[error("unknown instance id '{0}' referenced in mate {1}")]
    UnknownInstance(String, usize),
    #[error("mate {0} is over-constrained: {message}", message = .1)]
    OverConstrained(usize, String),
    #[error("mate {0} parameter error: {1}")]
    Parameter(usize, String),
    #[error("mate {0} concentric: axis direction is zero-length")]
    ZeroAxisDirection(usize),
}

#[derive(Debug, Error)]
pub enum AssemblyError {
    #[error("duplicate instance id: {0}")]
    DuplicateInstance(String),
    #[error("instance '{0}' references non-inline model — only Inline is currently supported")]
    UnresolvedRef(String),
    #[error("mate solve failed: {0}")]
    Mate(#[from] MateError),
    #[error("instance '{0}' evaluation: {1}")]
    Eval(String, EvalError),
    #[error("instance '{0}' has no features")]
    EmptyModel(String),
    #[error("instance '{0}' parameter error: {1}")]
    Parameter(String, String),
    #[error("json error: {0}")]
    Json(#[from] serde_json::Error),
}

// ---------------------------------------------------------------------------
// Solver
// ---------------------------------------------------------------------------

/// Tolerance for "is this constraint already satisfied?" checks during
/// over-constrained detection.
const MATE_TOL: f64 = 1e-9;

impl Assembly {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn with_instance(mut self, inst: Instance) -> Self {
        self.instances.push(inst);
        self
    }

    pub fn with_mate(mut self, mate: Mate) -> Self {
        self.mates.push(mate);
        self
    }

    pub fn with_parameter(mut self, name: impl Into<String>, value: f64) -> Self {
        self.parameters.insert(name.into(), value);
        self
    }

    fn check_unique_instances(&self) -> Result<(), AssemblyError> {
        let mut seen = std::collections::HashSet::new();
        for inst in &self.instances {
            if !seen.insert(inst.id.clone()) {
                return Err(AssemblyError::DuplicateInstance(inst.id.clone()));
            }
        }
        Ok(())
    }

    /// Solve mates and return the resolved pose for every instance, keyed by
    /// instance id. Mates are applied in declaration order; if a mate
    /// conflicts with an already-frozen instance, returns
    /// `MateError::OverConstrained` carrying the conflicting mate's index.
    pub fn solve_poses(&self) -> Result<HashMap<String, ResolvedPose>, AssemblyError> {
        self.check_unique_instances()?;

        // Initialize every instance's pose to its default_pose, resolved
        // against the assembly-level parameters.
        let mut poses: HashMap<String, ResolvedPose> = HashMap::new();
        for inst in &self.instances {
            let pose = inst.default_pose.resolve(&self.parameters).map_err(|e| {
                AssemblyError::Parameter(inst.id.clone(), format!("default_pose: {e}"))
            })?;
            poses.insert(inst.id.clone(), pose);
        }

        // Track which instance has been "moved" by some mate already. Used
        // for over-constrained detection: if an instance has already been
        // positioned and a new mate tries to position it but the constraint
        // doesn't already hold, that's over-constrained.
        let mut frozen: std::collections::HashSet<String> = std::collections::HashSet::new();

        for (mi, mate) in self.mates.iter().enumerate() {
            self.apply_mate(mi, mate, &mut poses, &mut frozen)?;
        }

        Ok(poses)
    }

    fn apply_mate(
        &self,
        mi: usize,
        mate: &Mate,
        poses: &mut HashMap<String, ResolvedPose>,
        frozen: &mut std::collections::HashSet<String>,
    ) -> Result<(), MateError> {
        match mate {
            Mate::Coincident {
                instance_a,
                point_a,
                instance_b,
                point_b,
            } => {
                let pa = self.resolve_pt_in_world(mi, instance_a, point_a, poses)?;
                let pb_world = self.resolve_pt_in_world(mi, instance_b, point_b, poses)?;

                let delta = pa - pb_world;

                if frozen.contains(instance_b) {
                    if delta.norm() > MATE_TOL {
                        return Err(MateError::OverConstrained(
                            mi,
                            format!(
                                "Coincident: instance '{instance_b}' is already positioned by an \
                                 earlier mate; current point_b is {} from point_a, want 0",
                                delta.norm()
                            ),
                        ));
                    }
                    return Ok(());
                }

                // Translate B by `delta` so its point lands on A's point.
                let pose_b = poses.get_mut(instance_b).expect("checked");
                pose_b.translation += delta;
                frozen.insert(instance_b.clone());
                Ok(())
            }
            Mate::Concentric {
                instance_a,
                axis_a,
                instance_b,
                axis_b,
            } => {
                let (oa_w, da_w) = self.resolve_axis_in_world(mi, instance_a, axis_a, poses)?;
                let (ob_w, db_w) = self.resolve_axis_in_world(mi, instance_b, axis_b, poses)?;

                if frozen.contains(instance_b) {
                    // Verify already-aligned to within tolerance.
                    let parallel = db_w.cross(&da_w).norm() < MATE_TOL;
                    let on_line = {
                        let v = ob_w - oa_w;
                        let proj = da_w.dot(&v);
                        let perp = v - da_w * proj;
                        perp.norm() < MATE_TOL
                    };
                    if !(parallel && on_line) {
                        return Err(MateError::OverConstrained(
                            mi,
                            format!(
                                "Concentric: '{instance_b}' is already positioned and its axis \
                                 doesn't match (parallel={parallel}, on_line={on_line})"
                            ),
                        ));
                    }
                    return Ok(());
                }

                let pose_b = poses.get_mut(instance_b).expect("checked");

                // Step 1: rotate B so its axis direction matches A's.
                // Rotation axis = db_w × da_w; angle = atan2(|cross|, dot).
                let cross = db_w.cross(&da_w);
                let dot = db_w.dot(&da_w);
                let cross_norm = cross.norm();
                if cross_norm > MATE_TOL {
                    let rot_axis = cross / cross_norm;
                    let angle = cross_norm.atan2(dot);
                    // Compose this rotation onto pose_b's existing rotation.
                    // pose_b currently rotates by R_b then translates by t_b.
                    // We want a NEW rotation R_extra applied AFTER R_b (in
                    // world space). Equivalently, the combined rotation
                    // takes a local point p to R_extra*(R_b*p) + R_extra*t_b
                    // ... but for an axis-angle representation we'd need
                    // quaternion composition. We avoid that by representing
                    // the additional rotation as a re-pose: instead of
                    // composing, treat the world-space point/direction as
                    // already-rotated by pose_b and apply R_extra around
                    // origin in world-space. The clean way is to update
                    // pose_b so that:
                    //   new_apply(p) = R_extra * old_apply(p)
                    //               = R_extra * (R_b * p + t_b)
                    //               = (R_extra*R_b) * p + R_extra*t_b
                    // We can express this by recomposing the axis-angle:
                    //   - new rotation = R_extra * R_b  (composed)
                    //   - new translation = R_extra * t_b
                    let r_b_then_extra = compose_axis_angle(
                        pose_b.rotation_axis,
                        pose_b.rotation_angle,
                        rot_axis,
                        angle,
                    );
                    pose_b.rotation_axis = r_b_then_extra.0;
                    pose_b.rotation_angle = r_b_then_extra.1;
                    pose_b.translation = rodrigues(pose_b.translation, rot_axis, angle);
                } else if dot < 0.0 {
                    // Anti-parallel: rotate 180 about any axis perpendicular to db_w.
                    let perp = pick_perpendicular(db_w);
                    let r_axis = perp;
                    let angle = std::f64::consts::PI;
                    let r_b_then_extra = compose_axis_angle(
                        pose_b.rotation_axis,
                        pose_b.rotation_angle,
                        r_axis,
                        angle,
                    );
                    pose_b.rotation_axis = r_b_then_extra.0;
                    pose_b.rotation_angle = r_b_then_extra.1;
                    pose_b.translation = rodrigues(pose_b.translation, r_axis, angle);
                }
                // (If parallel and same direction, no rotation needed.)

                // Step 2: translate B so its (now-rotated) axis origin lies
                // on A's line.
                // Re-evaluate axis_b's origin in world space using the
                // updated pose_b (we have a mutable borrow already).
                let mut ob_local = [0.0; 3];
                for (i, s) in axis_b.origin.iter().enumerate() {
                    ob_local[i] = s
                        .resolve(&self.parameters)
                        .map_err(|e| MateError::Parameter(mi, format!("axis.origin[{i}]: {e}")))?;
                }
                let new_ob_w = pose_b.apply_point(Vec3::new(ob_local[0], ob_local[1], ob_local[2]));
                let v = new_ob_w - oa_w;
                let proj = da_w.dot(&v);
                let perp = v - da_w * proj;
                pose_b.translation -= perp;

                frozen.insert(instance_b.clone());
                Ok(())
            }
            Mate::Distance {
                instance_a,
                point_a,
                instance_b,
                point_b,
                value,
            } => {
                let val = value.resolve(&self.parameters).map_err(|e| {
                    MateError::Parameter(mi, format!("distance value: {e}"))
                })?;
                let pa = self.resolve_pt_in_world(mi, instance_a, point_a, poses)?;
                let pb_world = self.resolve_pt_in_world(mi, instance_b, point_b, poses)?;

                let current_vec = pb_world - pa;
                let current_dist = current_vec.norm();

                if frozen.contains(instance_b) {
                    if (current_dist - val).abs() > MATE_TOL {
                        return Err(MateError::OverConstrained(
                            mi,
                            format!(
                                "Distance: '{instance_b}' is already positioned at distance \
                                 {current_dist}, want {val}"
                            ),
                        ));
                    }
                    return Ok(());
                }

                // Direction from A to B; if degenerate (B at A), pick +x.
                let dir = if current_dist > MATE_TOL {
                    current_vec / current_dist
                } else {
                    Vec3::x()
                };
                // Want pb at pa + dir*val. Move B by (target - current).
                let target = pa + dir * val;
                let delta = target - pb_world;
                let pose_b = poses.get_mut(instance_b).expect("checked");
                pose_b.translation += delta;
                frozen.insert(instance_b.clone());
                Ok(())
            }
        }
    }

    fn resolve_pt_in_world(
        &self,
        mi: usize,
        instance_id: &str,
        local_pt: &[Scalar; 3],
        poses: &HashMap<String, ResolvedPose>,
    ) -> Result<Vec3, MateError> {
        let pose = poses
            .get(instance_id)
            .ok_or_else(|| MateError::UnknownInstance(instance_id.to_string(), mi))?;
        let mut p = [0.0; 3];
        for (i, s) in local_pt.iter().enumerate() {
            p[i] = s
                .resolve(&self.parameters)
                .map_err(|e| MateError::Parameter(mi, format!("point[{i}]: {e}")))?;
        }
        Ok(pose.apply_point(Vec3::new(p[0], p[1], p[2])))
    }

    fn resolve_axis_in_world(
        &self,
        mi: usize,
        instance_id: &str,
        axis: &AxisRef,
        poses: &HashMap<String, ResolvedPose>,
    ) -> Result<(Vec3, Vec3), MateError> {
        let pose = poses
            .get(instance_id)
            .ok_or_else(|| MateError::UnknownInstance(instance_id.to_string(), mi))?;
        let mut o = [0.0; 3];
        let mut d = [0.0; 3];
        for (i, s) in axis.origin.iter().enumerate() {
            o[i] = s
                .resolve(&self.parameters)
                .map_err(|e| MateError::Parameter(mi, format!("axis.origin[{i}]: {e}")))?;
        }
        for (i, s) in axis.direction.iter().enumerate() {
            d[i] = s
                .resolve(&self.parameters)
                .map_err(|e| MateError::Parameter(mi, format!("axis.direction[{i}]: {e}")))?;
        }
        let origin_world = pose.apply_point(Vec3::new(o[0], o[1], o[2]));
        let dir_local = Vec3::new(d[0], d[1], d[2]);
        if dir_local.norm() < MATE_TOL {
            return Err(MateError::ZeroAxisDirection(mi));
        }
        let dir_world = pose.apply_vector(dir_local).normalize();
        Ok((origin_world, dir_world))
    }

    /// Evaluate every instance — produce its `Solid` and apply the resolved
    /// pose. Returns instances in declaration order.
    pub fn evaluate(&self) -> Result<Vec<(String, Solid)>, AssemblyError> {
        let poses = self.solve_poses()?;
        let mut out = Vec::with_capacity(self.instances.len());
        for inst in &self.instances {
            let model = match &inst.model {
                AssemblyRef::Inline(m) => m.as_ref(),
                AssemblyRef::Path(_) => return Err(AssemblyError::UnresolvedRef(inst.id.clone())),
            };
            // Pick the target feature: explicit `target`, else the last
            // feature in declaration order.
            let target_id = match &inst.target {
                Some(t) => t.clone(),
                None => model
                    .ids()
                    .last()
                    .ok_or_else(|| AssemblyError::EmptyModel(inst.id.clone()))?
                    .to_string(),
            };
            let solid = model
                .evaluate(&target_id)
                .map_err(|e| AssemblyError::Eval(inst.id.clone(), e))?;
            let pose = poses.get(&inst.id).expect("solver populated all instances");
            let posed = apply_pose_to_solid(&solid, *pose);
            out.push((inst.id.clone(), posed));
        }
        Ok(out)
    }

    pub fn to_json_string(&self) -> serde_json::Result<String> {
        serde_json::to_string_pretty(self)
    }

    pub fn from_json_str(s: &str) -> Result<Self, AssemblyError> {
        let mut a: Assembly = serde_json::from_str(s).map_err(AssemblyError::Json)?;
        // Inline models need their index rebuilt (reindex is called by
        // `Model::from_json_str`, but we got them via raw serde).
        for inst in &mut a.instances {
            if let AssemblyRef::Inline(m) = &mut inst.model {
                m.reindex().map_err(|e| {
                    AssemblyError::Eval(
                        inst.id.clone(),
                        EvalError::Invalid {
                            id: "model".into(),
                            reason: format!("reindex failed: {e:?}"),
                        },
                    )
                })?;
            }
        }
        Ok(a)
    }
}

// ---------------------------------------------------------------------------
// Pose math helpers
// ---------------------------------------------------------------------------

/// Apply a `ResolvedPose` to a `Solid`: rotate around origin, then translate.
pub fn apply_pose_to_solid(s: &Solid, pose: ResolvedPose) -> Solid {
    let mut out = s.clone();
    let axis_norm = pose.rotation_axis.norm();
    if pose.rotation_angle.abs() > 0.0 && axis_norm > 1e-15 {
        out = rotate_solid(
            &out,
            pose.rotation_axis,
            pose.rotation_angle,
            Point3::origin(),
        );
    }
    if pose.translation.norm() > 0.0 {
        out = translate_solid(&out, pose.translation);
    }
    out
}

/// Rodrigues' formula: rotate vector `v` by angle `theta` around axis `axis`.
/// `axis` need not be unit; zero-length axis means "no rotation".
pub fn rodrigues(v: Vec3, axis: Vec3, theta: f64) -> Vec3 {
    let n2 = axis.norm_squared();
    if n2 < 1e-30 || theta == 0.0 {
        return v;
    }
    let k = axis / n2.sqrt();
    let c = theta.cos();
    let s = theta.sin();
    v * c + k.cross(&v) * s + k * (k.dot(&v) * (1.0 - c))
}

/// Compose two axis-angle rotations: returns axis-angle equivalent of
/// `R_2 * R_1` (i.e., first apply `r1`, then `r2`). Uses quaternion
/// composition under the hood.
fn compose_axis_angle(
    axis_1: Vec3,
    angle_1: f64,
    axis_2: Vec3,
    angle_2: f64,
) -> (Vec3, f64) {
    // Convert each to quaternion (w, x, y, z).
    let q1 = axis_angle_to_quat(axis_1, angle_1);
    let q2 = axis_angle_to_quat(axis_2, angle_2);
    // q_total = q2 * q1.
    let (w1, x1, y1, z1) = q1;
    let (w2, x2, y2, z2) = q2;
    let w = w2 * w1 - x2 * x1 - y2 * y1 - z2 * z1;
    let x = w2 * x1 + x2 * w1 + y2 * z1 - z2 * y1;
    let y = w2 * y1 - x2 * z1 + y2 * w1 + z2 * x1;
    let z = w2 * z1 + x2 * y1 - y2 * x1 + z2 * w1;
    quat_to_axis_angle((w, x, y, z))
}

fn axis_angle_to_quat(axis: Vec3, angle: f64) -> (f64, f64, f64, f64) {
    let n = axis.norm();
    if n < 1e-15 || angle == 0.0 {
        return (1.0, 0.0, 0.0, 0.0);
    }
    let unit = axis / n;
    let half = angle * 0.5;
    let s = half.sin();
    (half.cos(), unit.x * s, unit.y * s, unit.z * s)
}

fn quat_to_axis_angle(q: (f64, f64, f64, f64)) -> (Vec3, f64) {
    let (w, x, y, z) = q;
    // Normalize defensively.
    let n = (w * w + x * x + y * y + z * z).sqrt();
    let (w, x, y, z) = if n > 1e-15 {
        (w / n, x / n, y / n, z / n)
    } else {
        (1.0, 0.0, 0.0, 0.0)
    };
    let w_clamped = w.clamp(-1.0, 1.0);
    let angle = 2.0 * w_clamped.acos();
    let s = (1.0 - w_clamped * w_clamped).sqrt();
    if s < 1e-12 {
        // Near zero rotation; axis is arbitrary, use +z.
        (Vec3::z(), 0.0)
    } else {
        (Vec3::new(x / s, y / s, z / s), angle)
    }
}

fn pick_perpendicular(v: Vec3) -> Vec3 {
    // Pick whichever world axis is least parallel to v, then orthogonalize.
    let candidates = [Vec3::x(), Vec3::y(), Vec3::z()];
    let mut best = candidates[0];
    let mut best_dot = v.dot(&best).abs();
    for c in &candidates[1..] {
        let d = v.dot(c).abs();
        if d < best_dot {
            best = *c;
            best_dot = d;
        }
    }
    let proj = v.dot(&best);
    let n2 = v.norm_squared();
    let perp = best - v * (proj / n2.max(1e-30));
    perp.normalize()
}

#[cfg(test)]
mod unit_tests {
    use super::*;

    #[test]
    fn rodrigues_identity() {
        let v = Vec3::new(1.0, 2.0, 3.0);
        let r = rodrigues(v, Vec3::z(), 0.0);
        assert!((r - v).norm() < 1e-15);
    }

    #[test]
    fn rodrigues_quarter_turn_z() {
        let v = Vec3::new(1.0, 0.0, 0.0);
        let r = rodrigues(v, Vec3::z(), std::f64::consts::FRAC_PI_2);
        assert!((r - Vec3::new(0.0, 1.0, 0.0)).norm() < 1e-12);
    }

    #[test]
    fn compose_quarter_turns() {
        // 90 about z, then 90 about z = 180 about z.
        let (axis, angle) = compose_axis_angle(
            Vec3::z(),
            std::f64::consts::FRAC_PI_2,
            Vec3::z(),
            std::f64::consts::FRAC_PI_2,
        );
        // Should be ~+z and ~PI.
        assert!((angle - std::f64::consts::PI).abs() < 1e-12);
        assert!((axis - Vec3::z()).norm() < 1e-9);
    }

    #[test]
    fn pick_perpendicular_is_orthogonal() {
        let v = Vec3::new(1.0, 1.0, 0.0).normalize();
        let p = pick_perpendicular(v);
        assert!(v.dot(&p).abs() < 1e-12);
        assert!((p.norm() - 1.0).abs() < 1e-12);
    }
}
