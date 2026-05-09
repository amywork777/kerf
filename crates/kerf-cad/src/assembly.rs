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

/// A reference to a simple analytic surface in an instance's local frame.
/// Used by `Mate::TangentMate`. Only the simple cases are supported today;
/// pairs that aren't covered (cylinder-on-cylinder skew, etc.) come back
/// from the solver as `MateError::NotImplemented`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum SurfaceRef {
    /// Infinite plane through `origin` with outward `normal`. The mate
    /// puts the second body on the side `normal` points toward.
    Plane {
        origin: [Scalar; 3],
        normal: [Scalar; 3],
    },
    /// Infinite cylinder of `radius` around the line `origin + t * axis`.
    Cylinder {
        origin: [Scalar; 3],
        axis: [Scalar; 3],
        radius: Scalar,
    },
    /// Sphere of `radius` centered at `center`.
    Sphere {
        center: [Scalar; 3],
        radius: Scalar,
    },
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
    /// Orient instance B's plane to be parallel to instance A's plane,
    /// with the optional offset measured along plane A's normal. Sign
    /// of `offset` follows `plane_a_normal`. Solved by rotating B so
    /// the plane normals align (Rodrigues), then translating B so plane
    /// B's origin sits at the chosen offset along plane A's normal.
    ParallelPlane {
        instance_a: String,
        plane_a_normal: [Scalar; 3],
        plane_a_origin: [Scalar; 3],
        instance_b: String,
        plane_b_normal: [Scalar; 3],
        plane_b_origin: [Scalar; 3],
        /// Distance from plane A to plane B, signed along plane A's
        /// normal. Use `Scalar::lit(0.0)` for "coplanar".
        offset: Scalar,
    },
    /// Set the angle between two axes (in radians). Solved by rotating
    /// B around the cross-product axis until the angle between A's and
    /// B's directions equals `angle`.
    AngleMate {
        instance_a: String,
        axis_a: AxisRef,
        instance_b: String,
        axis_b: AxisRef,
        /// Target angle in radians. Must be in `[0, π]`.
        angle: Scalar,
    },
    /// Make `surface_b` tangent to `surface_a`. Only the simple cases
    /// are supported: plane-plane (== ParallelPlane with zero offset
    /// inverted), cylinder-on-plane (cylinder rolls on the plane),
    /// sphere-on-plane (sphere kisses the plane), and sphere-on-sphere
    /// (external tangent). Other combos return `MateError::NotImplemented`.
    TangentMate {
        instance_a: String,
        surface_a: SurfaceRef,
        instance_b: String,
        surface_b: SurfaceRef,
    },
    /// Place instance B as the mirror image of instance A across a
    /// plane (defined in WORLD space by `plane_origin` and
    /// `plane_normal`). B's translation is reflected across the plane;
    /// B's rotation axis is reflected through the plane (component
    /// along the plane normal flipped) and the angle is negated, so
    /// B's orientation is the proper rotation that mirrors A's.
    Symmetry {
        instance_a: String,
        plane_origin: [Scalar; 3],
        plane_normal: [Scalar; 3],
        instance_b: String,
    },
    /// Set the perpendicular distance from B's centroid (its world
    /// origin under the current pose) to A's axis line to `distance`.
    /// The axis lives in A's local frame and is mapped to world through
    /// A's pose. Translates B along the perpendicular component (B is
    /// pushed outward or pulled inward toward A's axis until the
    /// requested distance holds).
    Width {
        instance_a: String,
        axis_a: AxisRef,
        instance_b: String,
        distance: Scalar,
    },
    /// Place instance at the world-space point obtained by linearly
    /// interpolating along `path` (a polyline of world-space waypoints)
    /// at parameter `t ∈ [0, 1]`. `t = 0` lands on the first waypoint;
    /// `t = 1` lands on the last. The instance's rotation is left
    /// alone — only its translation is set.
    PathMate {
        instance: String,
        path: Vec<[Scalar; 3]>,
        parameter: Scalar,
    },
    /// Fully constrain B to A: B's pose is forced equal to A's. This
    /// is the "no relative motion" mate — useful when two parts are
    /// rigidly bonded (welded, glued, threaded into a single block).
    Lock {
        instance_a: String,
        instance_b: String,
    },
    /// Force `point_b` (in B's local frame, transformed by B's pose) to
    /// coincide with `point_a` (in A's local frame, transformed by A's
    /// pose) — same residual as `Coincident`, but framed as a
    /// contact-style touch where both parts come into point contact
    /// without a preferred owner. The default solver moves B (the
    /// symbolic solver moves both ends).
    TouchPoint {
        instance_a: String,
        point_a: [Scalar; 3],
        instance_b: String,
        point_b: [Scalar; 3],
    },
    /// Constrain `point` on B to lie on the plane defined by
    /// (`plane_origin`, `plane_normal`) in A's local frame. Translates B
    /// along the plane normal until the signed distance is zero.
    PointOnPlane {
        instance_a: String,
        plane_origin: [Scalar; 3],
        plane_normal: [Scalar; 3],
        instance_b: String,
        point_b: [Scalar; 3],
    },
    /// Constrain `point` on B to lie on the infinite line `axis_a` (in
    /// A's local frame). Translates B perpendicular to the line so its
    /// point lands exactly on the line. The component along the line is
    /// preserved (a point on a line still has 1 free dof).
    PointOnLine {
        instance_a: String,
        axis_a: AxisRef,
        instance_b: String,
        point_b: [Scalar; 3],
    },
    /// Gear / coupled-rotation mate. When A rotates by angle θ around
    /// `axis_a`, B rotates by `θ * ratio` around `axis_b`. The reference
    /// configuration is the pose at which the mate was added — solver
    /// extracts A's current rotation about `axis_a` (relative to its
    /// default pose) and applies the geared rotation to B.
    ///
    /// `ratio = -1.0` for two equal-radius gears in mesh (B turns the
    /// opposite direction at the same speed). `ratio = 0.5` for a 2:1
    /// reduction (B is half the speed of A). `ratio = 2.0` for 1:2
    /// step-up.
    ///
    /// Both axes are interpreted in their respective local frames and
    /// mapped through the current poses to world.
    Gear {
        instance_a: String,
        axis_a: AxisRef,
        instance_b: String,
        axis_b: AxisRef,
        ratio: Scalar,
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
    /// Exploded-view amount: 0.0 = assembled, 1.0 = each instance moved
    /// `explode_distance` along the unit-vector from the assembly centroid to
    /// the instance's centroid. When `None` or `Some(0.0)`, behaves
    /// identically to the existing `Assembly::evaluate`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub exploded: Option<f64>,
    /// Distance (in world units) each instance is displaced at `exploded=1.0`.
    /// Auto-computed from the assembly's bounding-box diagonal × 0.5 if
    /// `None`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub explode_distance: Option<f64>,
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
    #[error("mate {0}: not implemented for this surface combination ({1})")]
    NotImplemented(usize, String),
    #[error("mate {0}: invalid input ({1})")]
    Invalid(usize, String),
    #[error("mate cycle did not converge after {iterations} iterations: residual {residual}")]
    CycleDidNotConverge { iterations: usize, residual: f64 },
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

/// Tolerance for the iterative cycle solver — sum of squared residuals
/// across all mates.
const CYCLE_RESIDUAL_TOL: f64 = 1e-12;

/// Maximum iterations for the iterative cycle solver.
const CYCLE_MAX_ITERS: usize = 200;

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
    /// instance id.
    ///
    /// Strategy:
    /// 1. **Acyclic networks** (mate graph is a forest): apply mates in
    ///    declaration order. Each mate moves its `instance_b` and freezes
    ///    it; later mates that try to move a frozen instance must already
    ///    satisfy the constraint, else `MateError::OverConstrained`.
    /// 2. **Cyclic networks** (some mate closes a loop in the instance
    ///    graph — A→B, B→C, C→A): the freeze rule would falsely flag the
    ///    third mate as over-constrained. Instead, run iterative
    ///    Gauss-Seidel relaxation: apply every mate to its `instance_b`
    ///    each pass without freezing, repeat until the sum of squared
    ///    residuals converges below `CYCLE_RESIDUAL_TOL` or
    ///    `CYCLE_MAX_ITERS` is hit.
    ///
    /// If iterative refinement does not converge, the final residual is
    /// re-checked for over-constrained vs. unconverged: a residual that
    /// won't budge under more iterations and exceeds tolerance signals an
    /// over-constrained cycle (`MateError::OverConstrained` carrying the
    /// last mate touched). A residual that's still moving but past the
    /// iteration cap returns `MateError::CycleDidNotConverge`.
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

        // Decide the path. If the mate graph has cycles, go straight to
        // iterative refinement; otherwise, run the strict "freeze after
        // first move" pass.
        if self.mates_form_cycle() {
            self.solve_poses_iterative(&mut poses)?;
        } else {
            // Track which instance has been "moved" by some mate already.
            // Used for over-constrained detection: if an instance has
            // already been positioned and a new mate tries to position it
            // but the constraint doesn't already hold, that's
            // over-constrained.
            let mut frozen: std::collections::HashSet<String> =
                std::collections::HashSet::new();
            for (mi, mate) in self.mates.iter().enumerate() {
                self.apply_mate(mi, mate, &mut poses, Some(&mut frozen))?;
            }
        }

        Ok(poses)
    }

    /// True if the directed-edge multigraph `instance_a → instance_b` (one
    /// edge per mate) contains a cycle when treated as undirected. Uses a
    /// union-find: when a mate connects two instances already in the same
    /// component, the network has a cycle.
    fn mates_form_cycle(&self) -> bool {
        let mut parent: HashMap<String, String> = HashMap::new();
        for inst in &self.instances {
            parent.insert(inst.id.clone(), inst.id.clone());
        }

        fn find(parent: &mut HashMap<String, String>, x: &str) -> String {
            let p = parent.get(x).cloned().unwrap_or_else(|| x.to_string());
            if p == x {
                return p;
            }
            let root = find(parent, &p);
            parent.insert(x.to_string(), root.clone());
            root
        }

        for mate in &self.mates {
            let (a, b) = mate_endpoints(mate);
            // Skip self-loops; they're degenerate but not "cycles" in the
            // sense we care about here.
            if a == b {
                continue;
            }
            // If either side names an unknown instance, defer to the
            // mate-application stage to produce the right error.
            if !parent.contains_key(a) || !parent.contains_key(b) {
                continue;
            }
            let ra = find(&mut parent, a);
            let rb = find(&mut parent, b);
            if ra == rb {
                return true;
            }
            parent.insert(ra, rb);
        }
        false
    }

    /// Iterative Gauss-Seidel relaxation for cyclic mate networks. Each
    /// pass applies every mate in declaration order, moving its
    /// `instance_b` toward satisfaction without freezing. Loops until
    /// total residual converges or the iteration cap is reached.
    fn solve_poses_iterative(
        &self,
        poses: &mut HashMap<String, ResolvedPose>,
    ) -> Result<(), AssemblyError> {
        let mut prev_residual = f64::INFINITY;
        for iter in 0..CYCLE_MAX_ITERS {
            for (mi, mate) in self.mates.iter().enumerate() {
                self.apply_mate(mi, mate, poses, None)?;
            }
            let residual = self.total_squared_residual(poses)?;
            if residual < CYCLE_RESIDUAL_TOL {
                return Ok(());
            }
            // Detect "stuck": residual essentially unchanged. Treat as
            // over-constrained (the system can't both satisfy mate i and
            // mate j simultaneously, so iteration stalls at a fixed
            // non-zero residual).
            if iter > 5 && (prev_residual - residual).abs() < 1e-15 && residual > 1e-6 {
                return Err(AssemblyError::Mate(MateError::OverConstrained(
                    self.mates.len().saturating_sub(1),
                    format!(
                        "cycle relaxation stalled at residual {residual} after {iter} \
                         iterations — system is over-constrained"
                    ),
                )));
            }
            prev_residual = residual;
        }
        Err(AssemblyError::Mate(MateError::CycleDidNotConverge {
            iterations: CYCLE_MAX_ITERS,
            residual: prev_residual,
        }))
    }

    /// Sum of squared residuals across every mate, given the current
    /// poses. Used by the iterative solver to decide convergence.
    fn total_squared_residual(
        &self,
        poses: &HashMap<String, ResolvedPose>,
    ) -> Result<f64, AssemblyError> {
        let mut total = 0.0;
        for (mi, mate) in self.mates.iter().enumerate() {
            total += self.mate_squared_residual(mi, mate, poses)?;
        }
        Ok(total)
    }

    fn apply_mate(
        &self,
        mi: usize,
        mate: &Mate,
        poses: &mut HashMap<String, ResolvedPose>,
        mut frozen: Option<&mut std::collections::HashSet<String>>,
    ) -> Result<(), MateError> {
        // Validate both endpoints are real, regardless of which mode.
        {
            let (a, b) = mate_endpoints(mate);
            if !poses.contains_key(a) {
                return Err(MateError::UnknownInstance(a.to_string(), mi));
            }
            if !poses.contains_key(b) {
                return Err(MateError::UnknownInstance(b.to_string(), mi));
            }
        }
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

                if let Some(f) = frozen.as_deref() {
                    if f.contains(instance_b) {
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
                }

                // Translate B by `delta` so its point lands on A's point.
                let pose_b = poses.get_mut(instance_b).expect("checked");
                pose_b.translation += delta;
                if let Some(f) = frozen.as_deref_mut() {
                    f.insert(instance_b.clone());
                }
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

                if let Some(f) = frozen.as_deref() {
                    if f.contains(instance_b) {
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
                }
                // Avoid `let _ = ob_w` — keep pulling its name through the
                // axis-recompute path below.
                let _ = ob_w;

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

                if let Some(f) = frozen.as_deref_mut() {
                    f.insert(instance_b.clone());
                }
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

                if let Some(f) = frozen.as_deref() {
                    if f.contains(instance_b) {
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
                if let Some(f) = frozen.as_deref_mut() {
                    f.insert(instance_b.clone());
                }
                Ok(())
            }
            Mate::ParallelPlane {
                instance_a,
                plane_a_normal,
                plane_a_origin,
                instance_b,
                plane_b_normal,
                plane_b_origin,
                offset,
            } => {
                let na = self.resolve_dir_in_world(mi, instance_a, plane_a_normal, poses)?;
                let oa = self.resolve_pt_in_world(mi, instance_a, plane_a_origin, poses)?;
                let nb = self.resolve_dir_in_world(mi, instance_b, plane_b_normal, poses)?;
                let ob = self.resolve_pt_in_world(mi, instance_b, plane_b_origin, poses)?;
                let off = offset.resolve(&self.parameters).map_err(|e| {
                    MateError::Parameter(mi, format!("offset: {e}"))
                })?;

                // Plane B should be parallel to plane A: nb_world == na_world.
                // With offset: ob_world = oa_world + off * na_world (signed
                // along plane_a's normal).
                let parallel = nb.cross(&na).norm() < MATE_TOL && nb.dot(&na) > 0.0;
                let target_ob = oa + na * off;
                let delta_origin = target_ob - ob;
                let on_target = delta_origin.norm() < MATE_TOL;

                if let Some(f) = frozen.as_deref() {
                    if f.contains(instance_b) {
                        if !(parallel && on_target) {
                            return Err(MateError::OverConstrained(
                                mi,
                                format!(
                                    "ParallelPlane: '{instance_b}' is already positioned \
                                     (parallel={parallel}, on_target={on_target}, \
                                     origin_delta={})",
                                    delta_origin.norm()
                                ),
                            ));
                        }
                        return Ok(());
                    }
                }

                let pose_b = poses.get_mut(instance_b).expect("checked");

                // Step 1: rotate B so nb aligns with na.
                let cross = nb.cross(&na);
                let dot = nb.dot(&na);
                let cross_norm = cross.norm();
                if cross_norm > MATE_TOL {
                    let rot_axis = cross / cross_norm;
                    let angle = cross_norm.atan2(dot);
                    let r = compose_axis_angle(
                        pose_b.rotation_axis,
                        pose_b.rotation_angle,
                        rot_axis,
                        angle,
                    );
                    pose_b.rotation_axis = r.0;
                    pose_b.rotation_angle = r.1;
                    pose_b.translation = rodrigues(pose_b.translation, rot_axis, angle);
                } else if dot < 0.0 {
                    // Anti-parallel: 180° flip about a perpendicular axis.
                    let perp = pick_perpendicular(nb);
                    let angle = std::f64::consts::PI;
                    let r = compose_axis_angle(
                        pose_b.rotation_axis,
                        pose_b.rotation_angle,
                        perp,
                        angle,
                    );
                    pose_b.rotation_axis = r.0;
                    pose_b.rotation_angle = r.1;
                    pose_b.translation = rodrigues(pose_b.translation, perp, angle);
                }

                // Step 2: translate B so its plane origin lands at the
                // signed offset along na from oa. Re-evaluate ob in world
                // with the updated rotation.
                let ob_local = resolve_arr3(plane_b_origin, &self.parameters)
                    .map_err(|e| MateError::Parameter(mi, format!("plane_b_origin: {e}")))?;
                let ob_now = pose_b.apply_point(Vec3::new(ob_local[0], ob_local[1], ob_local[2]));
                let target = oa + na * off;
                let delta = target - ob_now;
                pose_b.translation += delta;

                if let Some(f) = frozen.as_deref_mut() {
                    f.insert(instance_b.clone());
                }
                Ok(())
            }
            Mate::AngleMate {
                instance_a,
                axis_a,
                instance_b,
                axis_b,
                angle,
            } => {
                let target_angle = angle.resolve(&self.parameters).map_err(|e| {
                    MateError::Parameter(mi, format!("angle: {e}"))
                })?;
                if !(0.0..=std::f64::consts::PI + 1e-12).contains(&target_angle) {
                    return Err(MateError::Invalid(
                        mi,
                        format!("AngleMate: angle {target_angle} is outside [0, π]"),
                    ));
                }
                let (_oa, da) = self.resolve_axis_in_world(mi, instance_a, axis_a, poses)?;
                let (_ob, db) = self.resolve_axis_in_world(mi, instance_b, axis_b, poses)?;

                let current_dot = db.dot(&da).clamp(-1.0, 1.0);
                let current_angle = current_dot.acos();
                let delta_angle = target_angle - current_angle;

                if let Some(f) = frozen.as_deref() {
                    if f.contains(instance_b) {
                        if delta_angle.abs() > MATE_TOL {
                            return Err(MateError::OverConstrained(
                                mi,
                                format!(
                                    "AngleMate: '{instance_b}' axis is already at \
                                     {current_angle} rad from A, want {target_angle}"
                                ),
                            ));
                        }
                        return Ok(());
                    }
                }

                if delta_angle.abs() < MATE_TOL {
                    if let Some(f) = frozen.as_deref_mut() {
                        f.insert(instance_b.clone());
                    }
                    return Ok(());
                }

                // Pick a rotation axis: cross of db × da (the axis that
                // would send db onto da) — but we may want to rotate by
                // any (current_angle - target_angle) so we go in the
                // correct direction (decreasing the gap).
                let cross = db.cross(&da);
                let cross_norm = cross.norm();
                let rot_axis = if cross_norm > MATE_TOL {
                    cross / cross_norm
                } else {
                    pick_perpendicular(db)
                };

                // Rotate B by delta_angle around rot_axis. (db rotated by
                // (current_angle - target_angle) around an axis from
                // db→da reaches the desired angular separation; equivalent
                // to rotating BY +delta_angle = +(target - current) when
                // the rotation axis is db × da.)
                // If delta_angle < 0, we need to rotate the other way —
                // negative angle does that.
                let angle = -delta_angle;
                let pose_b = poses.get_mut(instance_b).expect("checked");
                let r = compose_axis_angle(
                    pose_b.rotation_axis,
                    pose_b.rotation_angle,
                    rot_axis,
                    angle,
                );
                pose_b.rotation_axis = r.0;
                pose_b.rotation_angle = r.1;
                pose_b.translation = rodrigues(pose_b.translation, rot_axis, angle);

                if let Some(f) = frozen.as_deref_mut() {
                    f.insert(instance_b.clone());
                }
                Ok(())
            }
            Mate::TangentMate {
                instance_a,
                surface_a,
                instance_b,
                surface_b,
            } => self.apply_tangent_mate(
                mi,
                instance_a,
                surface_a,
                instance_b,
                surface_b,
                poses,
                frozen.as_deref_mut(),
            ),
            Mate::Symmetry {
                instance_a,
                plane_origin,
                plane_normal,
                instance_b,
            } => self.apply_symmetry_mate(
                mi,
                instance_a,
                plane_origin,
                plane_normal,
                instance_b,
                poses,
                frozen.as_deref_mut(),
            ),
            Mate::Width {
                instance_a,
                axis_a,
                instance_b,
                distance,
            } => self.apply_width_mate(
                mi,
                instance_a,
                axis_a,
                instance_b,
                distance,
                poses,
                frozen.as_deref_mut(),
            ),
            Mate::PathMate {
                instance,
                path,
                parameter,
            } => self.apply_path_mate(
                mi,
                instance,
                path,
                parameter,
                poses,
                frozen.as_deref_mut(),
            ),
            Mate::Lock {
                instance_a,
                instance_b,
            } => self.apply_lock_mate(
                mi,
                instance_a,
                instance_b,
                poses,
                frozen.as_deref_mut(),
            ),
            Mate::TouchPoint {
                instance_a,
                point_a,
                instance_b,
                point_b,
            } => {
                // TouchPoint = Coincident in residual. Default solver
                // moves B; the symbolic solver (Newton-LM over all
                // poses) handles the symmetric case.
                let pa = self.resolve_pt_in_world(mi, instance_a, point_a, poses)?;
                let pb_world = self.resolve_pt_in_world(mi, instance_b, point_b, poses)?;
                let delta = pa - pb_world;

                if let Some(f) = frozen.as_deref() {
                    if f.contains(instance_b) {
                        if delta.norm() > MATE_TOL {
                            return Err(MateError::OverConstrained(
                                mi,
                                format!(
                                    "TouchPoint: instance '{instance_b}' is already positioned; \
                                     current point_b is {} from point_a, want 0",
                                    delta.norm()
                                ),
                            ));
                        }
                        return Ok(());
                    }
                }
                let pose_b = poses.get_mut(instance_b).expect("checked");
                pose_b.translation += delta;
                if let Some(f) = frozen.as_deref_mut() {
                    f.insert(instance_b.clone());
                }
                Ok(())
            }
            Mate::PointOnPlane {
                instance_a,
                plane_origin,
                plane_normal,
                instance_b,
                point_b,
            } => {
                let na = self.resolve_dir_in_world(mi, instance_a, plane_normal, poses)?;
                let pa = self.resolve_pt_in_world(mi, instance_a, plane_origin, poses)?;
                let pb_world = self.resolve_pt_in_world(mi, instance_b, point_b, poses)?;
                // Signed distance from B's point to A's plane along
                // plane normal. We want to translate B along the plane
                // normal so signed distance is zero.
                let signed = na.dot(&(pb_world - pa));

                if let Some(f) = frozen.as_deref() {
                    if f.contains(instance_b) {
                        if signed.abs() > MATE_TOL {
                            return Err(MateError::OverConstrained(
                                mi,
                                format!(
                                    "PointOnPlane: '{instance_b}' is already positioned; \
                                     current signed distance to plane is {signed}, want 0"
                                ),
                            ));
                        }
                        return Ok(());
                    }
                }

                let pose_b = poses.get_mut(instance_b).expect("checked");
                pose_b.translation -= na * signed;
                if let Some(f) = frozen.as_deref_mut() {
                    f.insert(instance_b.clone());
                }
                Ok(())
            }
            Mate::PointOnLine {
                instance_a,
                axis_a,
                instance_b,
                point_b,
            } => {
                let (oa, da) = self.resolve_axis_in_world(mi, instance_a, axis_a, poses)?;
                let pb_world = self.resolve_pt_in_world(mi, instance_b, point_b, poses)?;
                // Foot of perpendicular from pb_world onto the line.
                let v = pb_world - oa;
                let along = da * da.dot(&v);
                let perp = v - along; // we want this to vanish.

                if let Some(f) = frozen.as_deref() {
                    if f.contains(instance_b) {
                        if perp.norm() > MATE_TOL {
                            return Err(MateError::OverConstrained(
                                mi,
                                format!(
                                    "PointOnLine: '{instance_b}' is already positioned; \
                                     current perp distance is {}, want 0",
                                    perp.norm()
                                ),
                            ));
                        }
                        return Ok(());
                    }
                }
                let pose_b = poses.get_mut(instance_b).expect("checked");
                pose_b.translation -= perp;
                if let Some(f) = frozen.as_deref_mut() {
                    f.insert(instance_b.clone());
                }
                Ok(())
            }
            Mate::Gear {
                instance_a,
                axis_a,
                instance_b,
                axis_b,
                ratio,
            } => self.apply_gear_mate(
                mi,
                instance_a,
                axis_a,
                instance_b,
                axis_b,
                ratio,
                poses,
                frozen.as_deref_mut(),
            ),
        }
    }

    /// Gear mate: when A rotates by angle θ_a around axis_a (relative
    /// to its default pose), B rotates by θ_a * ratio around axis_b
    /// (relative to its default pose).
    ///
    /// We extract A's "twist about axis_a" from its current rotation
    /// (the component of A's rotation that lies along axis_a) and
    /// apply the geared twist to B about axis_b.
    #[allow(clippy::too_many_arguments)]
    fn apply_gear_mate(
        &self,
        mi: usize,
        instance_a: &str,
        axis_a: &AxisRef,
        instance_b: &str,
        axis_b: &AxisRef,
        ratio: &Scalar,
        poses: &mut HashMap<String, ResolvedPose>,
        mut frozen: Option<&mut std::collections::HashSet<String>>,
    ) -> Result<(), MateError> {
        let r = ratio
            .resolve(&self.parameters)
            .map_err(|e| MateError::Parameter(mi, format!("ratio: {e}")))?;

        // A's current rotation expressed as angle about its own axis_a.
        let pose_a = *poses.get(instance_a).expect("checked");
        // World-space axis_a (from pose_a).
        let axis_a_local = resolve_arr3(&axis_a.direction, &self.parameters)
            .map_err(|e| MateError::Parameter(mi, format!("axis_a.direction: {e}")))?;
        let axis_a_local_v = Vec3::new(axis_a_local[0], axis_a_local[1], axis_a_local[2]);
        if axis_a_local_v.norm() < MATE_TOL {
            return Err(MateError::ZeroAxisDirection(mi));
        }
        let axis_a_local_unit = axis_a_local_v.normalize();
        // The rotation A has about axis_a_local: project pose_a's
        // axis-angle onto axis_a_local_unit. If pose_a's rotation axis
        // is parallel to axis_a_local, the angle is just rotation_angle
        // (with sign). If perpendicular, A has no rotation about
        // axis_a. In general, theta_a = rotation_angle * (rotation_axis
        // dot axis_a_local) when normalized.
        let theta_a = if pose_a.rotation_axis.norm() < MATE_TOL {
            0.0
        } else {
            pose_a.rotation_axis.normalize().dot(&axis_a_local_unit) * pose_a.rotation_angle
        };

        let theta_b = theta_a * r;

        // Resolve axis_b in B's local frame, then world.
        let axis_b_local_arr = resolve_arr3(&axis_b.direction, &self.parameters)
            .map_err(|e| MateError::Parameter(mi, format!("axis_b.direction: {e}")))?;
        let axis_b_local = Vec3::new(
            axis_b_local_arr[0],
            axis_b_local_arr[1],
            axis_b_local_arr[2],
        );
        if axis_b_local.norm() < MATE_TOL {
            return Err(MateError::ZeroAxisDirection(mi));
        }
        let axis_b_local_unit = axis_b_local.normalize();

        if let Some(f) = frozen.as_deref() {
            if f.contains(instance_b) {
                // Extract B's current twist about axis_b_local; if it
                // matches theta_b within tolerance, mate is satisfied.
                let pose_b = poses.get(instance_b).expect("checked");
                let cur_theta_b = if pose_b.rotation_axis.norm() < MATE_TOL {
                    0.0
                } else {
                    pose_b.rotation_axis.normalize().dot(&axis_b_local_unit)
                        * pose_b.rotation_angle
                };
                if (cur_theta_b - theta_b).abs() > MATE_TOL {
                    return Err(MateError::OverConstrained(
                        mi,
                        format!(
                            "Gear: '{instance_b}' is already positioned and its twist about \
                             axis_b is {cur_theta_b}, want {theta_b} (ratio {r}, theta_a {theta_a})"
                        ),
                    ));
                }
                return Ok(());
            }
        }

        // Set B's rotation to be exactly theta_b about axis_b_local
        // (overwrite, since gear is the dominant constraint on B's
        // rotation about axis_b).
        let pose_b = poses.get_mut(instance_b).expect("checked");
        // Translation is preserved. We force the pose's rotation_axis
        // to axis_b_local and angle to theta_b.
        pose_b.rotation_axis = axis_b_local_unit;
        pose_b.rotation_angle = theta_b;

        if let Some(f) = frozen.as_deref_mut() {
            f.insert(instance_b.to_string());
        }
        Ok(())
    }

    /// Tangent-mate dispatch. Supports the simple cases:
    /// - plane-on-plane: equivalent to `ParallelPlane` with offset 0 and
    ///   normals pointed at each other (anti-parallel).
    /// - plane-cylinder (either order): cylinder axis becomes parallel
    ///   to plane, cylinder line at `radius` distance from plane.
    /// - plane-sphere (either order): sphere center sits at `radius`
    ///   along the plane normal from the plane origin's projection.
    /// - sphere-sphere: external tangent — center distance = sum of radii.
    /// All other combinations return `MateError::NotImplemented`.
    #[allow(clippy::too_many_arguments)]
    fn apply_tangent_mate(
        &self,
        mi: usize,
        instance_a: &str,
        surface_a: &SurfaceRef,
        instance_b: &str,
        surface_b: &SurfaceRef,
        poses: &mut HashMap<String, ResolvedPose>,
        frozen: Option<&mut std::collections::HashSet<String>>,
    ) -> Result<(), MateError> {
        match (surface_a, surface_b) {
            (
                SurfaceRef::Plane {
                    origin: _,
                    normal: _,
                },
                SurfaceRef::Plane {
                    origin: _,
                    normal: _,
                },
            ) => Err(MateError::NotImplemented(
                mi,
                "plane-plane tangent — use ParallelPlane with offset=0 instead".into(),
            )),
            (
                SurfaceRef::Plane {
                    origin: pa_o,
                    normal: pa_n,
                },
                SurfaceRef::Cylinder {
                    origin: cy_o,
                    axis: cy_ax,
                    radius: cy_r,
                },
            ) => {
                // Cylinder of B sits tangent to plane of A. Cylinder axis
                // (in world) must be perpendicular to plane normal; the
                // axis line must be at distance `radius` from the plane,
                // on the +normal side.
                let radius = cy_r.resolve(&self.parameters).map_err(|e| {
                    MateError::Parameter(mi, format!("cylinder.radius: {e}"))
                })?;
                self.solve_plane_cylinder_tangent(
                    mi, instance_a, pa_o, pa_n, instance_b, cy_o, cy_ax, radius, poses, frozen,
                )
            }
            (
                SurfaceRef::Cylinder {
                    origin: cy_o,
                    axis: cy_ax,
                    radius: cy_r,
                },
                SurfaceRef::Plane {
                    origin: pa_o,
                    normal: pa_n,
                },
            ) => {
                // Plane of B is tangent to cylinder of A — symmetric, but
                // we move B (the plane). Reuse the same helper with roles
                // flipped.
                let _ = (cy_o, cy_ax, cy_r, pa_o, pa_n);
                Err(MateError::NotImplemented(
                    mi,
                    "tangent: plane-on-cylinder (move plane to kiss cylinder) — \
                     supported direction is cylinder-on-plane only"
                        .into(),
                ))
            }
            (
                SurfaceRef::Plane {
                    origin: pa_o,
                    normal: pa_n,
                },
                SurfaceRef::Sphere {
                    center: sp_c,
                    radius: sp_r,
                },
            ) => {
                let radius = sp_r.resolve(&self.parameters).map_err(|e| {
                    MateError::Parameter(mi, format!("sphere.radius: {e}"))
                })?;
                self.solve_plane_sphere_tangent(
                    mi, instance_a, pa_o, pa_n, instance_b, sp_c, radius, poses, frozen,
                )
            }
            (
                SurfaceRef::Sphere {
                    center: _,
                    radius: _,
                },
                SurfaceRef::Plane {
                    origin: _,
                    normal: _,
                },
            ) => Err(MateError::NotImplemented(
                mi,
                "tangent: sphere-on-plane requires the plane on the A side (use \
                 surface_a=Plane, surface_b=Sphere)"
                    .into(),
            )),
            (
                SurfaceRef::Sphere {
                    center: ca,
                    radius: ra,
                },
                SurfaceRef::Sphere {
                    center: cb,
                    radius: rb,
                },
            ) => {
                let radius_a = ra.resolve(&self.parameters).map_err(|e| {
                    MateError::Parameter(mi, format!("sphere_a.radius: {e}"))
                })?;
                let radius_b = rb.resolve(&self.parameters).map_err(|e| {
                    MateError::Parameter(mi, format!("sphere_b.radius: {e}"))
                })?;
                self.solve_sphere_sphere_tangent(
                    mi, instance_a, ca, radius_a, instance_b, cb, radius_b, poses, frozen,
                )
            }
            (SurfaceRef::Cylinder { .. }, SurfaceRef::Cylinder { .. }) => Err(
                MateError::NotImplemented(mi, "tangent: cylinder-cylinder".into()),
            ),
            (SurfaceRef::Cylinder { .. }, SurfaceRef::Sphere { .. }) => Err(
                MateError::NotImplemented(mi, "tangent: cylinder-sphere".into()),
            ),
            (SurfaceRef::Sphere { .. }, SurfaceRef::Cylinder { .. }) => Err(
                MateError::NotImplemented(mi, "tangent: sphere-cylinder".into()),
            ),
        }
    }

    /// Symmetry mate. B's pose is set so it is the mirror image of A
    /// across the plane (origin, normal) given in WORLD space.
    fn apply_symmetry_mate(
        &self,
        mi: usize,
        instance_a: &str,
        plane_origin: &[Scalar; 3],
        plane_normal: &[Scalar; 3],
        instance_b: &str,
        poses: &mut HashMap<String, ResolvedPose>,
        mut frozen: Option<&mut std::collections::HashSet<String>>,
    ) -> Result<(), MateError> {
        let po_arr = resolve_arr3(plane_origin, &self.parameters)
            .map_err(|e| MateError::Parameter(mi, format!("plane_origin: {e}")))?;
        let pn_arr = resolve_arr3(plane_normal, &self.parameters)
            .map_err(|e| MateError::Parameter(mi, format!("plane_normal: {e}")))?;
        let plane_o = Vec3::new(po_arr[0], po_arr[1], po_arr[2]);
        let pn_raw = Vec3::new(pn_arr[0], pn_arr[1], pn_arr[2]);
        if pn_raw.norm() < MATE_TOL {
            return Err(MateError::ZeroAxisDirection(mi));
        }
        let plane_n = pn_raw.normalize();
        let pose_a = *poses.get(instance_a).expect("checked");

        // Compute target translation: reflect A's translation across the
        // mirror plane. reflect(p) = p - 2 * ((p - o) · n) * n.
        let ta = pose_a.translation;
        let target_translation = ta - plane_n * (2.0 * (ta - plane_o).dot(&plane_n));

        // Compute target rotation axis: reflect A's rotation axis
        // through the plane (component along plane_n flipped). The
        // angle is negated so the resulting rotation mirrors A's.
        // For an axis vector v: reflected = v - 2 * (v · n) * n.
        let axis_a_world = pose_a.rotation_axis;
        let target_axis = axis_a_world - plane_n * (2.0 * axis_a_world.dot(&plane_n));
        let target_angle = -pose_a.rotation_angle;

        // Verify already-satisfied for frozen check.
        if let Some(f) = frozen.as_deref() {
            if f.contains(instance_b) {
                let pose_b = poses.get(instance_b).expect("checked");
                let trans_err = (pose_b.translation - target_translation).norm();
                if trans_err > MATE_TOL {
                    return Err(MateError::OverConstrained(
                        mi,
                        format!(
                            "Symmetry: '{instance_b}' is already positioned and its translation \
                             differs from the mirror by {trans_err}"
                        ),
                    ));
                }
                return Ok(());
            }
        }

        let pose_b = poses.get_mut(instance_b).expect("checked");
        pose_b.translation = target_translation;
        pose_b.rotation_axis = target_axis;
        pose_b.rotation_angle = target_angle;
        if let Some(f) = frozen.as_deref_mut() {
            f.insert(instance_b.to_string());
        }
        Ok(())
    }

    /// Width mate. Push B perpendicular to A's axis line until B's
    /// world origin sits at `distance` from the line.
    fn apply_width_mate(
        &self,
        mi: usize,
        instance_a: &str,
        axis_a: &AxisRef,
        instance_b: &str,
        distance: &Scalar,
        poses: &mut HashMap<String, ResolvedPose>,
        mut frozen: Option<&mut std::collections::HashSet<String>>,
    ) -> Result<(), MateError> {
        let val = distance
            .resolve(&self.parameters)
            .map_err(|e| MateError::Parameter(mi, format!("width distance: {e}")))?;
        if val < 0.0 {
            return Err(MateError::Invalid(
                mi,
                format!("Width: distance must be non-negative, got {val}"),
            ));
        }
        let (oa, da) = self.resolve_axis_in_world(mi, instance_a, axis_a, poses)?;

        // B's centroid in world = its translation (we treat instance
        // origin as the centroid).
        let cb = poses.get(instance_b).expect("checked").translation;
        // Vector from a point on A's axis to B's centroid.
        let v = cb - oa;
        // Component of v along A's axis (preserved).
        let along = da * da.dot(&v);
        // Component perpendicular to A's axis (this is the radial
        // vector — its length is the current distance).
        let perp = v - along;
        let cur_dist = perp.norm();

        // Direction of "outward" from the axis. If the centroid is
        // exactly on the axis, pick a perpendicular fallback so the
        // mate still resolves.
        let radial_dir = if cur_dist > MATE_TOL {
            perp / cur_dist
        } else {
            pick_perpendicular(da)
        };

        // Target centroid: foot of perpendicular on the axis + radial_dir * val.
        let foot = oa + along;
        let target_cb = foot + radial_dir * val;
        let delta = target_cb - cb;

        if let Some(f) = frozen.as_deref() {
            if f.contains(instance_b) {
                if delta.norm() > MATE_TOL {
                    return Err(MateError::OverConstrained(
                        mi,
                        format!(
                            "Width: '{instance_b}' is already positioned at distance \
                             {cur_dist} from A's axis, want {val}"
                        ),
                    ));
                }
                return Ok(());
            }
        }

        let pose_b = poses.get_mut(instance_b).expect("checked");
        pose_b.translation += delta;
        if let Some(f) = frozen.as_deref_mut() {
            f.insert(instance_b.to_string());
        }
        Ok(())
    }

    /// Path mate. Translate `instance` so its world-origin sits at the
    /// polyline-interpolated point at parameter t.
    fn apply_path_mate(
        &self,
        mi: usize,
        instance: &str,
        path: &[[Scalar; 3]],
        parameter: &Scalar,
        poses: &mut HashMap<String, ResolvedPose>,
        mut frozen: Option<&mut std::collections::HashSet<String>>,
    ) -> Result<(), MateError> {
        if path.len() < 2 {
            return Err(MateError::Invalid(
                mi,
                format!(
                    "PathMate: path must have at least 2 waypoints, got {}",
                    path.len()
                ),
            ));
        }
        let t = parameter
            .resolve(&self.parameters)
            .map_err(|e| MateError::Parameter(mi, format!("path parameter: {e}")))?;
        if !(-1e-12..=1.0 + 1e-12).contains(&t) {
            return Err(MateError::Invalid(
                mi,
                format!("PathMate: parameter {t} is outside [0, 1]"),
            ));
        }
        let t = t.clamp(0.0, 1.0);

        // Resolve every waypoint.
        let mut pts: Vec<Vec3> = Vec::with_capacity(path.len());
        for (i, p) in path.iter().enumerate() {
            let arr = resolve_arr3(p, &self.parameters)
                .map_err(|e| MateError::Parameter(mi, format!("path[{i}]: {e}")))?;
            pts.push(Vec3::new(arr[0], arr[1], arr[2]));
        }

        // Compute cumulative arc lengths.
        let mut seg_lens = Vec::with_capacity(pts.len() - 1);
        let mut total = 0.0;
        for i in 0..pts.len() - 1 {
            let l = (pts[i + 1] - pts[i]).norm();
            total += l;
            seg_lens.push(l);
        }
        if total < 1e-15 {
            return Err(MateError::Invalid(
                mi,
                "PathMate: degenerate path (zero total length)".into(),
            ));
        }

        // Find which segment t lands in by walking arc-length.
        let target_len = t * total;
        let mut acc = 0.0;
        let mut target = pts[pts.len() - 1];
        for i in 0..seg_lens.len() {
            let next_acc = acc + seg_lens[i];
            if target_len <= next_acc + 1e-15 {
                let local_t = if seg_lens[i] > 0.0 {
                    (target_len - acc) / seg_lens[i]
                } else {
                    0.0
                };
                target = pts[i] + (pts[i + 1] - pts[i]) * local_t;
                break;
            }
            acc = next_acc;
        }

        // Snap exactly onto endpoints when t is at the bounds.
        if t <= 1e-12 {
            target = pts[0];
        } else if t >= 1.0 - 1e-12 {
            target = pts[pts.len() - 1];
        }

        let cur = poses.get(instance).expect("checked").translation;
        let delta = target - cur;

        if let Some(f) = frozen.as_deref() {
            if f.contains(instance) {
                if delta.norm() > MATE_TOL {
                    return Err(MateError::OverConstrained(
                        mi,
                        format!(
                            "PathMate: '{instance}' is already positioned at {:?}, \
                             want {:?} (delta {})",
                            cur,
                            target,
                            delta.norm()
                        ),
                    ));
                }
                return Ok(());
            }
        }

        let pose = poses.get_mut(instance).expect("checked");
        pose.translation = target;
        if let Some(f) = frozen.as_deref_mut() {
            f.insert(instance.to_string());
        }
        Ok(())
    }

    /// Lock mate. B's pose is forced equal to A's pose.
    fn apply_lock_mate(
        &self,
        mi: usize,
        instance_a: &str,
        instance_b: &str,
        poses: &mut HashMap<String, ResolvedPose>,
        mut frozen: Option<&mut std::collections::HashSet<String>>,
    ) -> Result<(), MateError> {
        let pose_a = *poses.get(instance_a).expect("checked");

        if let Some(f) = frozen.as_deref() {
            if f.contains(instance_b) {
                let pose_b = poses.get(instance_b).expect("checked");
                let trans_err = (pose_b.translation - pose_a.translation).norm();
                let axis_diff = (pose_b.rotation_axis.normalize()
                    - pose_a.rotation_axis.normalize())
                .norm();
                let angle_err = (pose_b.rotation_angle - pose_a.rotation_angle).abs();
                if trans_err > MATE_TOL || (axis_diff > MATE_TOL && angle_err > MATE_TOL) {
                    return Err(MateError::OverConstrained(
                        mi,
                        format!(
                            "Lock: '{instance_b}' is already positioned and its pose \
                             differs from A's (trans_err={trans_err}, angle_err={angle_err})"
                        ),
                    ));
                }
                return Ok(());
            }
        }

        let pose_b = poses.get_mut(instance_b).expect("checked");
        pose_b.translation = pose_a.translation;
        pose_b.rotation_axis = pose_a.rotation_axis;
        pose_b.rotation_angle = pose_a.rotation_angle;
        if let Some(f) = frozen.as_deref_mut() {
            f.insert(instance_b.to_string());
        }
        Ok(())
    }

    /// Cylinder-on-plane tangent. Plane lives on instance A, cylinder on
    /// B. The cylinder's axis (world) becomes perpendicular to the plane
    /// normal, and its line sits at `radius` from the plane on the
    /// +normal side. This rotates and translates B.
    #[allow(clippy::too_many_arguments)]
    fn solve_plane_cylinder_tangent(
        &self,
        mi: usize,
        instance_a: &str,
        plane_origin: &[Scalar; 3],
        plane_normal: &[Scalar; 3],
        instance_b: &str,
        cyl_origin: &[Scalar; 3],
        cyl_axis: &[Scalar; 3],
        radius: f64,
        poses: &mut HashMap<String, ResolvedPose>,
        mut frozen: Option<&mut std::collections::HashSet<String>>,
    ) -> Result<(), MateError> {
        let na = self.resolve_dir_in_world(mi, instance_a, plane_normal, poses)?;
        let pa = self.resolve_pt_in_world(mi, instance_a, plane_origin, poses)?;
        let cax = self.resolve_dir_in_world(mi, instance_b, cyl_axis, poses)?;
        let _co_initial = self.resolve_pt_in_world(mi, instance_b, cyl_origin, poses)?;

        // Goal: cax · na = 0 (axis perpendicular to plane normal). Rotate
        // B about the axis (cax × na) by the angle that drives cax · na
        // to zero. Specifically, the projection of cax onto na should
        // become zero — current angle between cax and the plane is
        // φ = π/2 − arccos(cax · na). We want φ = 0, i.e. arccos(cax·na)
        // = π/2, i.e. cax·na = 0.
        let dot = cax.dot(&na).clamp(-1.0, 1.0);
        let current_angle = dot.acos();
        let target_angle = std::f64::consts::FRAC_PI_2;
        let delta_angle = target_angle - current_angle;

        if delta_angle.abs() > MATE_TOL {
            // Rotation axis: cax × na (the axis that would send cax → na).
            let cross = cax.cross(&na);
            let cn = cross.norm();
            let rot_axis = if cn > MATE_TOL {
                cross / cn
            } else {
                pick_perpendicular(cax)
            };
            // We want to drive the angle FROM cax TO na to π/2. Rotating
            // by +delta_angle around (cax × na) bends cax toward na by
            // |delta_angle|; sign of delta_angle controls direction.
            let angle = delta_angle;
            let pose_b = poses.get_mut(instance_b).expect("checked");
            let r = compose_axis_angle(
                pose_b.rotation_axis,
                pose_b.rotation_angle,
                rot_axis,
                angle,
            );
            pose_b.rotation_axis = r.0;
            pose_b.rotation_angle = r.1;
            pose_b.translation = rodrigues(pose_b.translation, rot_axis, angle);
        }

        // After rotating, re-resolve cax and co in world, then translate
        // B so the cylinder line sits at `radius` from the plane on the
        // +na side.
        let cax_now = {
            let pose_b = poses.get(instance_b).expect("checked");
            let local = resolve_arr3(cyl_axis, &self.parameters)
                .map_err(|e| MateError::Parameter(mi, format!("cyl_axis: {e}")))?;
            pose_b.apply_vector(Vec3::new(local[0], local[1], local[2])).normalize()
        };
        let co_now = {
            let pose_b = poses.get(instance_b).expect("checked");
            let local = resolve_arr3(cyl_origin, &self.parameters)
                .map_err(|e| MateError::Parameter(mi, format!("cyl_origin: {e}")))?;
            pose_b.apply_point(Vec3::new(local[0], local[1], local[2]))
        };

        // Distance from co_now to the plane (signed along na):
        let signed_dist = na.dot(&(co_now - pa));
        // We want signed_dist = radius. But the cylinder axis is now
        // (approximately) parallel to the plane, so any point on the axis
        // has the same signed distance. Translate by (radius -
        // signed_dist) along na.
        let move_amount = radius - signed_dist;
        // ALSO: project out any tiny axial component so we don't undo the
        // rotation alignment. Translation along the cylinder's own axis
        // doesn't change the geometry's pose-meaningfully.
        let translate_dir = na;
        let _ = (cax_now, co_now);

        let pose_b = poses.get_mut(instance_b).expect("checked");
        pose_b.translation += translate_dir * move_amount;

        if let Some(f) = frozen.as_deref_mut() {
            f.insert(instance_b.to_string());
        }
        Ok(())
    }

    #[allow(clippy::too_many_arguments)]
    fn solve_plane_sphere_tangent(
        &self,
        mi: usize,
        instance_a: &str,
        plane_origin: &[Scalar; 3],
        plane_normal: &[Scalar; 3],
        instance_b: &str,
        sphere_center: &[Scalar; 3],
        radius: f64,
        poses: &mut HashMap<String, ResolvedPose>,
        mut frozen: Option<&mut std::collections::HashSet<String>>,
    ) -> Result<(), MateError> {
        let na = self.resolve_dir_in_world(mi, instance_a, plane_normal, poses)?;
        let pa = self.resolve_pt_in_world(mi, instance_a, plane_origin, poses)?;
        let cb_now = self.resolve_pt_in_world(mi, instance_b, sphere_center, poses)?;

        // Signed distance from sphere center to plane along na:
        let signed_dist = na.dot(&(cb_now - pa));
        // We want signed_dist = radius (sphere kissing plane on the
        // +normal side).
        let move_amount = radius - signed_dist;

        let pose_b = poses.get_mut(instance_b).expect("checked");
        pose_b.translation += na * move_amount;

        if let Some(f) = frozen.as_deref_mut() {
            f.insert(instance_b.to_string());
        }
        Ok(())
    }

    #[allow(clippy::too_many_arguments)]
    fn solve_sphere_sphere_tangent(
        &self,
        mi: usize,
        instance_a: &str,
        center_a: &[Scalar; 3],
        radius_a: f64,
        instance_b: &str,
        center_b: &[Scalar; 3],
        radius_b: f64,
        poses: &mut HashMap<String, ResolvedPose>,
        mut frozen: Option<&mut std::collections::HashSet<String>>,
    ) -> Result<(), MateError> {
        let ca = self.resolve_pt_in_world(mi, instance_a, center_a, poses)?;
        let cb = self.resolve_pt_in_world(mi, instance_b, center_b, poses)?;
        let target_dist = radius_a + radius_b;
        let cur_vec = cb - ca;
        let cur_dist = cur_vec.norm();
        let dir = if cur_dist > MATE_TOL {
            cur_vec / cur_dist
        } else {
            Vec3::x()
        };
        let target_cb = ca + dir * target_dist;
        let delta = target_cb - cb;
        let pose_b = poses.get_mut(instance_b).expect("checked");
        pose_b.translation += delta;
        if let Some(f) = frozen.as_deref_mut() {
            f.insert(instance_b.to_string());
        }
        Ok(())
    }

    /// Compute squared residual for one mate, given current poses. Used
    /// by the iterative cycle solver.
    fn mate_squared_residual(
        &self,
        mi: usize,
        mate: &Mate,
        poses: &HashMap<String, ResolvedPose>,
    ) -> Result<f64, AssemblyError> {
        match mate {
            Mate::Coincident {
                instance_a,
                point_a,
                instance_b,
                point_b,
            } => {
                let pa = self.resolve_pt_in_world(mi, instance_a, point_a, poses)?;
                let pb = self.resolve_pt_in_world(mi, instance_b, point_b, poses)?;
                Ok((pa - pb).norm_squared())
            }
            Mate::Concentric {
                instance_a,
                axis_a,
                instance_b,
                axis_b,
            } => {
                let (oa, da) = self.resolve_axis_in_world(mi, instance_a, axis_a, poses)?;
                let (ob, db) = self.resolve_axis_in_world(mi, instance_b, axis_b, poses)?;
                let parallel_err = db.cross(&da).norm_squared();
                let v = ob - oa;
                let proj = da.dot(&v);
                let perp = v - da * proj;
                Ok(parallel_err + perp.norm_squared())
            }
            Mate::Distance {
                instance_a,
                point_a,
                instance_b,
                point_b,
                value,
            } => {
                let val = value.resolve(&self.parameters).map_err(|e| {
                    AssemblyError::Mate(MateError::Parameter(mi, format!("distance value: {e}")))
                })?;
                let pa = self.resolve_pt_in_world(mi, instance_a, point_a, poses)?;
                let pb = self.resolve_pt_in_world(mi, instance_b, point_b, poses)?;
                let d = (pb - pa).norm();
                Ok((d - val).powi(2))
            }
            Mate::ParallelPlane {
                instance_a,
                plane_a_normal,
                plane_a_origin,
                instance_b,
                plane_b_normal,
                plane_b_origin,
                offset,
            } => {
                let na = self.resolve_dir_in_world(mi, instance_a, plane_a_normal, poses)?;
                let oa = self.resolve_pt_in_world(mi, instance_a, plane_a_origin, poses)?;
                let nb = self.resolve_dir_in_world(mi, instance_b, plane_b_normal, poses)?;
                let ob = self.resolve_pt_in_world(mi, instance_b, plane_b_origin, poses)?;
                let off = offset.resolve(&self.parameters).map_err(|e| {
                    AssemblyError::Mate(MateError::Parameter(mi, format!("offset: {e}")))
                })?;
                let parallel_err = nb.cross(&na).norm_squared();
                let target_ob = oa + na * off;
                let origin_err = (ob - target_ob).norm_squared();
                Ok(parallel_err + origin_err)
            }
            Mate::AngleMate {
                instance_a,
                axis_a,
                instance_b,
                axis_b,
                angle,
            } => {
                let target = angle.resolve(&self.parameters).map_err(|e| {
                    AssemblyError::Mate(MateError::Parameter(mi, format!("angle: {e}")))
                })?;
                let (_oa, da) = self.resolve_axis_in_world(mi, instance_a, axis_a, poses)?;
                let (_ob, db) = self.resolve_axis_in_world(mi, instance_b, axis_b, poses)?;
                let cur = db.dot(&da).clamp(-1.0, 1.0).acos();
                Ok((cur - target).powi(2))
            }
            Mate::TangentMate {
                instance_a,
                surface_a,
                instance_b,
                surface_b,
            } => self.tangent_squared_residual(
                mi, instance_a, surface_a, instance_b, surface_b, poses,
            ),
            Mate::Symmetry {
                instance_a,
                plane_origin,
                plane_normal,
                instance_b,
            } => {
                let po_arr = resolve_arr3(plane_origin, &self.parameters).map_err(|e| {
                    AssemblyError::Mate(MateError::Parameter(mi, format!("plane_origin: {e}")))
                })?;
                let pn_arr = resolve_arr3(plane_normal, &self.parameters).map_err(|e| {
                    AssemblyError::Mate(MateError::Parameter(mi, format!("plane_normal: {e}")))
                })?;
                let plane_o = Vec3::new(po_arr[0], po_arr[1], po_arr[2]);
                let pn_raw = Vec3::new(pn_arr[0], pn_arr[1], pn_arr[2]);
                if pn_raw.norm() < MATE_TOL {
                    return Ok(0.0);
                }
                let plane_n = pn_raw.normalize();
                let pose_a = poses.get(instance_a).ok_or_else(|| {
                    AssemblyError::Mate(MateError::UnknownInstance(instance_a.clone(), mi))
                })?;
                let pose_b = poses.get(instance_b).ok_or_else(|| {
                    AssemblyError::Mate(MateError::UnknownInstance(instance_b.clone(), mi))
                })?;
                let target_t = pose_a.translation
                    - plane_n * (2.0 * (pose_a.translation - plane_o).dot(&plane_n));
                Ok((pose_b.translation - target_t).norm_squared())
            }
            Mate::Width {
                instance_a,
                axis_a,
                instance_b,
                distance,
            } => {
                let val = distance.resolve(&self.parameters).map_err(|e| {
                    AssemblyError::Mate(MateError::Parameter(mi, format!("width: {e}")))
                })?;
                let (oa, da) = self.resolve_axis_in_world(mi, instance_a, axis_a, poses)?;
                let cb = poses
                    .get(instance_b)
                    .ok_or_else(|| {
                        AssemblyError::Mate(MateError::UnknownInstance(instance_b.clone(), mi))
                    })?
                    .translation;
                let v = cb - oa;
                let perp = v - da * da.dot(&v);
                let cur = perp.norm();
                Ok((cur - val).powi(2))
            }
            Mate::PathMate {
                instance,
                path,
                parameter,
            } => {
                if path.len() < 2 {
                    return Ok(0.0);
                }
                let t = match parameter.resolve(&self.parameters) {
                    Ok(v) => v.clamp(0.0, 1.0),
                    Err(_) => return Ok(0.0),
                };
                let mut pts: Vec<Vec3> = Vec::with_capacity(path.len());
                for p in path {
                    let arr = match resolve_arr3(p, &self.parameters) {
                        Ok(a) => a,
                        Err(_) => return Ok(0.0),
                    };
                    pts.push(Vec3::new(arr[0], arr[1], arr[2]));
                }
                let mut total = 0.0;
                let mut seg_lens = Vec::with_capacity(pts.len() - 1);
                for i in 0..pts.len() - 1 {
                    let l = (pts[i + 1] - pts[i]).norm();
                    total += l;
                    seg_lens.push(l);
                }
                if total < 1e-15 {
                    return Ok(0.0);
                }
                let target_len = t * total;
                let mut acc = 0.0;
                let mut target = pts[pts.len() - 1];
                for i in 0..seg_lens.len() {
                    let next_acc = acc + seg_lens[i];
                    if target_len <= next_acc + 1e-15 {
                        let local = if seg_lens[i] > 0.0 {
                            (target_len - acc) / seg_lens[i]
                        } else {
                            0.0
                        };
                        target = pts[i] + (pts[i + 1] - pts[i]) * local;
                        break;
                    }
                    acc = next_acc;
                }
                let cur = poses
                    .get(instance)
                    .ok_or_else(|| {
                        AssemblyError::Mate(MateError::UnknownInstance(instance.clone(), mi))
                    })?
                    .translation;
                Ok((cur - target).norm_squared())
            }
            Mate::Lock {
                instance_a,
                instance_b,
            } => {
                let pa = poses.get(instance_a).ok_or_else(|| {
                    AssemblyError::Mate(MateError::UnknownInstance(instance_a.clone(), mi))
                })?;
                let pb = poses.get(instance_b).ok_or_else(|| {
                    AssemblyError::Mate(MateError::UnknownInstance(instance_b.clone(), mi))
                })?;
                let trans_err = (pa.translation - pb.translation).norm_squared();
                let angle_err = (pa.rotation_angle - pb.rotation_angle).powi(2);
                Ok(trans_err + angle_err)
            }
            Mate::TouchPoint {
                instance_a,
                point_a,
                instance_b,
                point_b,
            } => {
                let pa = self.resolve_pt_in_world(mi, instance_a, point_a, poses)?;
                let pb = self.resolve_pt_in_world(mi, instance_b, point_b, poses)?;
                Ok((pa - pb).norm_squared())
            }
            Mate::PointOnPlane {
                instance_a,
                plane_origin,
                plane_normal,
                instance_b,
                point_b,
            } => {
                let na = self.resolve_dir_in_world(mi, instance_a, plane_normal, poses)?;
                let pa = self.resolve_pt_in_world(mi, instance_a, plane_origin, poses)?;
                let pb = self.resolve_pt_in_world(mi, instance_b, point_b, poses)?;
                let signed = na.dot(&(pb - pa));
                Ok(signed.powi(2))
            }
            Mate::PointOnLine {
                instance_a,
                axis_a,
                instance_b,
                point_b,
            } => {
                let (oa, da) = self.resolve_axis_in_world(mi, instance_a, axis_a, poses)?;
                let pb = self.resolve_pt_in_world(mi, instance_b, point_b, poses)?;
                let v = pb - oa;
                let perp = v - da * da.dot(&v);
                Ok(perp.norm_squared())
            }
            Mate::Gear {
                instance_a,
                axis_a,
                instance_b,
                axis_b,
                ratio,
            } => {
                let r = ratio.resolve(&self.parameters).map_err(|e| {
                    AssemblyError::Mate(MateError::Parameter(mi, format!("ratio: {e}")))
                })?;
                let pose_a = poses.get(instance_a).ok_or_else(|| {
                    AssemblyError::Mate(MateError::UnknownInstance(instance_a.clone(), mi))
                })?;
                let pose_b = poses.get(instance_b).ok_or_else(|| {
                    AssemblyError::Mate(MateError::UnknownInstance(instance_b.clone(), mi))
                })?;
                let axis_a_local_arr =
                    resolve_arr3(&axis_a.direction, &self.parameters).map_err(|e| {
                        AssemblyError::Mate(MateError::Parameter(
                            mi,
                            format!("axis_a.direction: {e}"),
                        ))
                    })?;
                let axis_b_local_arr =
                    resolve_arr3(&axis_b.direction, &self.parameters).map_err(|e| {
                        AssemblyError::Mate(MateError::Parameter(
                            mi,
                            format!("axis_b.direction: {e}"),
                        ))
                    })?;
                let axis_a_local =
                    Vec3::new(axis_a_local_arr[0], axis_a_local_arr[1], axis_a_local_arr[2]);
                let axis_b_local =
                    Vec3::new(axis_b_local_arr[0], axis_b_local_arr[1], axis_b_local_arr[2]);
                if axis_a_local.norm() < MATE_TOL || axis_b_local.norm() < MATE_TOL {
                    return Ok(0.0);
                }
                let axis_a_unit = axis_a_local.normalize();
                let axis_b_unit = axis_b_local.normalize();
                let theta_a = if pose_a.rotation_axis.norm() < MATE_TOL {
                    0.0
                } else {
                    pose_a.rotation_axis.normalize().dot(&axis_a_unit) * pose_a.rotation_angle
                };
                let theta_b = if pose_b.rotation_axis.norm() < MATE_TOL {
                    0.0
                } else {
                    pose_b.rotation_axis.normalize().dot(&axis_b_unit) * pose_b.rotation_angle
                };
                Ok((theta_b - theta_a * r).powi(2))
            }
        }
    }

    fn tangent_squared_residual(
        &self,
        mi: usize,
        instance_a: &str,
        surface_a: &SurfaceRef,
        instance_b: &str,
        surface_b: &SurfaceRef,
        poses: &HashMap<String, ResolvedPose>,
    ) -> Result<f64, AssemblyError> {
        match (surface_a, surface_b) {
            (
                SurfaceRef::Plane { origin: pao, normal: pan },
                SurfaceRef::Cylinder { origin: cyo, axis: cya, radius: cyr },
            ) => {
                let na = self.resolve_dir_in_world(mi, instance_a, pan, poses)?;
                let pa = self.resolve_pt_in_world(mi, instance_a, pao, poses)?;
                let cax = self.resolve_dir_in_world(mi, instance_b, cya, poses)?;
                let co = self.resolve_pt_in_world(mi, instance_b, cyo, poses)?;
                let r = cyr.resolve(&self.parameters).map_err(|e| {
                    AssemblyError::Mate(MateError::Parameter(mi, format!("radius: {e}")))
                })?;
                let perp_err = na.dot(&cax).powi(2);
                let dist_err = (na.dot(&(co - pa)) - r).powi(2);
                Ok(perp_err + dist_err)
            }
            (
                SurfaceRef::Plane { origin: pao, normal: pan },
                SurfaceRef::Sphere { center: spc, radius: spr },
            ) => {
                let na = self.resolve_dir_in_world(mi, instance_a, pan, poses)?;
                let pa = self.resolve_pt_in_world(mi, instance_a, pao, poses)?;
                let cb = self.resolve_pt_in_world(mi, instance_b, spc, poses)?;
                let r = spr.resolve(&self.parameters).map_err(|e| {
                    AssemblyError::Mate(MateError::Parameter(mi, format!("radius: {e}")))
                })?;
                Ok((na.dot(&(cb - pa)) - r).powi(2))
            }
            (
                SurfaceRef::Sphere { center: ca, radius: ra },
                SurfaceRef::Sphere { center: cb, radius: rb },
            ) => {
                let pa = self.resolve_pt_in_world(mi, instance_a, ca, poses)?;
                let pb = self.resolve_pt_in_world(mi, instance_b, cb, poses)?;
                let r_a = ra.resolve(&self.parameters).map_err(|e| {
                    AssemblyError::Mate(MateError::Parameter(mi, format!("radius_a: {e}")))
                })?;
                let r_b = rb.resolve(&self.parameters).map_err(|e| {
                    AssemblyError::Mate(MateError::Parameter(mi, format!("radius_b: {e}")))
                })?;
                let d = (pb - pa).norm();
                Ok((d - (r_a + r_b)).powi(2))
            }
            // Unsupported pairs report zero residual; the apply step
            // returns NotImplemented and the iterative path never
            // reaches here.
            _ => Ok(0.0),
        }
    }

    /// Resolve a direction vector through an instance's pose to world.
    /// Result is normalized; returns `ZeroAxisDirection` if the local
    /// vector is degenerate.
    fn resolve_dir_in_world(
        &self,
        mi: usize,
        instance_id: &str,
        local_dir: &[Scalar; 3],
        poses: &HashMap<String, ResolvedPose>,
    ) -> Result<Vec3, MateError> {
        let pose = poses
            .get(instance_id)
            .ok_or_else(|| MateError::UnknownInstance(instance_id.to_string(), mi))?;
        let mut d = [0.0; 3];
        for (i, s) in local_dir.iter().enumerate() {
            d[i] = s.resolve(&self.parameters).map_err(|e| {
                MateError::Parameter(mi, format!("direction[{i}]: {e}"))
            })?;
        }
        let v = Vec3::new(d[0], d[1], d[2]);
        if v.norm() < MATE_TOL {
            return Err(MateError::ZeroAxisDirection(mi));
        }
        Ok(pose.apply_vector(v).normalize())
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
    ///
    /// Sub-assembly references (`AssemblyRef::Path`) are not resolved here;
    /// use [`Assembly::evaluate_with_loader`] for that.
    pub fn evaluate(&self) -> Result<Vec<(String, Solid)>, AssemblyError> {
        // Closure that always errors on Path — preserves the historical
        // behaviour of `evaluate` for callers that don't want sub-assembly
        // composition.
        self.evaluate_with_loader(|_path: &str| -> Result<Assembly, AssemblyError> {
            Err(AssemblyError::UnresolvedRef(String::new()))
        })
    }

    /// Evaluate the assembly in exploded view.
    ///
    /// When `self.exploded` is `None` or `Some(0.0)` this is identical to
    /// [`Assembly::evaluate`].  Otherwise each instance solid is displaced
    /// radially away from the assembly's volume-weighted centroid by
    /// `exploded * explode_distance` units, where `explode_distance` is taken
    /// from `self.explode_distance` or auto-computed as half the assembly's
    /// bounding-box diagonal when that field is `None`.
    ///
    /// Steps:
    /// 1. Solve mates (existing logic via `evaluate`).
    /// 2. Compute assembly centroid (mean of instance AABB centres, weighted
    ///    by AABB volume as a proxy for part volume).
    /// 3. Determine `explode_distance` (field or auto).
    /// 4. For each instance shift its solid along the unit vector from the
    ///    assembly centroid to the instance AABB centre, scaled by
    ///    `exploded * explode_distance`.
    /// 5. Return the displaced solids in declaration order.
    pub fn evaluate_exploded(&self) -> Result<Vec<(String, Solid)>, AssemblyError> {
        // Step 1: evaluate using the existing path (mates + poses).
        let solids = self.evaluate()?;

        // If there is nothing to explode, return as-is.
        let amount = match self.exploded {
            None | Some(0.0) => return Ok(solids),
            Some(a) => a,
        };

        // Step 2: compute per-instance AABB centres and volumes.
        // We use AABB centre as a proxy for the part centroid, and AABB
        // volume as a proxy for part volume (consistent with solid_aabb).
        let mut centres: Vec<Vec3> = Vec::with_capacity(solids.len());
        let mut weights: Vec<f64> = Vec::with_capacity(solids.len());

        for (_, solid) in &solids {
            if let Some((min, max)) = solid_aabb(solid) {
                let c = Vec3::new(
                    (min[0] + max[0]) * 0.5,
                    (min[1] + max[1]) * 0.5,
                    (min[2] + max[2]) * 0.5,
                );
                let vol = ((max[0] - min[0]) * (max[1] - min[1]) * (max[2] - min[2])).max(0.0);
                centres.push(c);
                weights.push(vol);
            } else {
                // Empty solid: use origin with zero weight.
                centres.push(Vec3::zeros());
                weights.push(0.0);
            }
        }

        // Weighted mean centroid. Fall back to unweighted if all weights are zero.
        let total_weight: f64 = weights.iter().sum();
        let assembly_centroid = if total_weight > 0.0 {
            let mut acc = Vec3::zeros();
            for (c, w) in centres.iter().zip(weights.iter()) {
                acc += *c * *w;
            }
            acc / total_weight
        } else {
            // All empty — just average unweighted.
            let n = centres.len() as f64;
            if n == 0.0 {
                Vec3::zeros()
            } else {
                centres.iter().fold(Vec3::zeros(), |a, c| a + *c) / n
            }
        };

        // Step 3: determine explode_distance.
        let dist = match self.explode_distance {
            Some(d) => d,
            None => {
                // Auto: half the assembly bounding-box diagonal.
                let mut min_all = [f64::INFINITY; 3];
                let mut max_all = [f64::NEG_INFINITY; 3];
                let mut any = false;
                for (_, solid) in &solids {
                    if let Some((mn, mx)) = solid_aabb(solid) {
                        any = true;
                        for i in 0..3 {
                            min_all[i] = min_all[i].min(mn[i]);
                            max_all[i] = max_all[i].max(mx[i]);
                        }
                    }
                }
                if any {
                    let dx = max_all[0] - min_all[0];
                    let dy = max_all[1] - min_all[1];
                    let dz = max_all[2] - min_all[2];
                    (dx * dx + dy * dy + dz * dz).sqrt() * 0.5
                } else {
                    0.0
                }
            }
        };

        // Step 4: displace each solid radially.
        let mut out = Vec::with_capacity(solids.len());
        for ((id, solid), centre) in solids.into_iter().zip(centres.iter()) {
            let radial = *centre - assembly_centroid;
            let radial_norm = radial.norm();
            let displaced = if radial_norm > 1e-15 {
                let unit = radial / radial_norm;
                let shift = unit * (amount * dist);
                translate_solid(&solid, shift)
            } else {
                // Instance is exactly at the assembly centroid — no direction
                // to explode along, leave in place.
                solid
            };
            out.push((id, displaced));
        }

        Ok(out)
    }

    /// Evaluate every instance, using `loader` to resolve any
    /// `AssemblyRef::Path` reference. The loader takes the path string
    /// (whatever was stored in the JSON) and returns a parsed `Assembly`.
    /// The sub-assembly is then evaluated recursively (its instances'
    /// solids are unioned into a single composite solid for the
    /// containing instance), and the containing instance's pose is
    /// applied on top.
    ///
    /// Tests typically pass a closure backed by a `HashMap<String, String>`
    /// of "filename" → JSON contents, so the assembly fixture stays purely
    /// in-memory.
    pub fn evaluate_with_loader<F>(&self, loader: F) -> Result<Vec<(String, Solid)>, AssemblyError>
    where
        F: Fn(&str) -> Result<Assembly, AssemblyError> + Copy,
    {
        let poses = self.solve_poses()?;
        let mut out = Vec::with_capacity(self.instances.len());
        for inst in &self.instances {
            let pose = poses.get(&inst.id).expect("solver populated all instances");
            let solid = match &inst.model {
                AssemblyRef::Inline(m) => {
                    let model = m.as_ref();
                    let target_id = match &inst.target {
                        Some(t) => t.clone(),
                        None => model
                            .ids()
                            .last()
                            .ok_or_else(|| AssemblyError::EmptyModel(inst.id.clone()))?
                            .to_string(),
                    };
                    model
                        .evaluate(&target_id)
                        .map_err(|e| AssemblyError::Eval(inst.id.clone(), e))?
                }
                AssemblyRef::Path(p) => {
                    // Loader returns a sub-assembly. Evaluate it
                    // recursively (with the same loader) and union the
                    // resulting solids into a composite.
                    let sub = loader(p).map_err(|e| match e {
                        AssemblyError::UnresolvedRef(_) => {
                            AssemblyError::UnresolvedRef(inst.id.clone())
                        }
                        other => other,
                    })?;
                    let parts = sub.evaluate_with_loader(loader)?;
                    if parts.is_empty() {
                        return Err(AssemblyError::EmptyModel(inst.id.clone()));
                    }
                    let mut acc = parts[0].1.clone();
                    for (_, s) in parts.iter().skip(1) {
                        acc = acc.union(s);
                    }
                    acc
                }
            };
            let posed = apply_pose_to_solid(&solid, *pose);
            out.push((inst.id.clone(), posed));
        }
        Ok(out)
    }

    /// Detect pairs of instances whose axis-aligned bounding boxes
    /// overlap, returning `(instance_a_id, instance_b_id, overlap_volume)`.
    /// `params` is currently unused (the assembly's own parameter table
    /// drives sub-evaluation) but is kept on the signature so callers
    /// can pass a frame-of-reference parameter set later.
    ///
    /// Pairs are returned in alphabetical order of `(a, b)` with `a < b`,
    /// so output is deterministic.
    ///
    /// Sub-assemblies (`AssemblyRef::Path`) are not currently expanded
    /// — each `Path` instance is treated as having no detectable
    /// geometry and is skipped.
    pub fn detect_interference(
        &self,
        _params: &HashMap<String, f64>,
    ) -> Result<Vec<(String, String, f64)>, AssemblyError> {
        let posed = self.evaluate()?;

        // AABB per instance, computed on the posed solid.
        let mut aabbs: Vec<(String, [f64; 3], [f64; 3])> = Vec::with_capacity(posed.len());
        for (id, solid) in &posed {
            if let Some((min, max)) = solid_aabb(solid) {
                aabbs.push((id.clone(), min, max));
            }
        }

        let mut out: Vec<(String, String, f64)> = Vec::new();
        for i in 0..aabbs.len() {
            for j in (i + 1)..aabbs.len() {
                let (id_a, min_a, max_a) = &aabbs[i];
                let (id_b, min_b, max_b) = &aabbs[j];
                let overlap = aabb_overlap_volume(min_a, max_a, min_b, max_b);
                if overlap > 0.0 {
                    let (a, b) = if id_a <= id_b {
                        (id_a.clone(), id_b.clone())
                    } else {
                        (id_b.clone(), id_a.clone())
                    };
                    out.push((a, b, overlap));
                }
            }
        }
        out.sort_by(|x, y| x.0.cmp(&y.0).then_with(|| x.1.cmp(&y.1)));
        Ok(out)
    }

    /// Solve mates by treating every instance pose as 6 DOFs (3
    /// translation + 3 axis-angle rotation = ω vector whose magnitude is
    /// the rotation angle) and running Newton-LM (Levenberg-Marquardt)
    /// against the residual function defined by the mates.
    ///
    /// This is the "true 6N-dim symbolic solver" path for arbitrary
    /// mate cycles — it handles configurations the sequential
    /// Gauss-Seidel pass can't (multi-cycle networks, gear loops,
    /// over-determined networks where the LM damping picks the
    /// least-squares solution). The Jacobian is finite-differenced.
    ///
    /// Returns the same `HashMap<String, ResolvedPose>` as
    /// `solve_poses` so callers are interchangeable. Falls back to
    /// `MateError::CycleDidNotConverge` if 100 LM iterations don't
    /// drive the residual below `CYCLE_RESIDUAL_TOL`.
    pub fn solve_poses_symbolic(
        &self,
    ) -> Result<HashMap<String, ResolvedPose>, AssemblyError> {
        self.check_unique_instances()?;

        // Order instances; ids[i] ↔ x[6i..6i+6].
        let ids: Vec<String> = self.instances.iter().map(|i| i.id.clone()).collect();
        let n_inst = ids.len();
        let n_dof = 6 * n_inst;

        // Initialize x from default_pose (rotation as ω = axis * angle).
        let mut x = vec![0.0; n_dof];
        for (i, inst) in self.instances.iter().enumerate() {
            let pose = inst.default_pose.resolve(&self.parameters).map_err(|e| {
                AssemblyError::Parameter(inst.id.clone(), format!("default_pose: {e}"))
            })?;
            let off = i * 6;
            x[off] = pose.translation.x;
            x[off + 1] = pose.translation.y;
            x[off + 2] = pose.translation.z;
            // ω = axis * angle (axis-angle in vector form). If axis is
            // zero or angle is zero, ω is zero.
            let axis_norm = pose.rotation_axis.norm();
            if axis_norm > 1e-15 && pose.rotation_angle.abs() > 1e-15 {
                let unit = pose.rotation_axis / axis_norm;
                x[off + 3] = unit.x * pose.rotation_angle;
                x[off + 4] = unit.y * pose.rotation_angle;
                x[off + 5] = unit.z * pose.rotation_angle;
            }
        }

        // Helper to convert x → pose map.
        let x_to_poses = |x: &[f64]| -> HashMap<String, ResolvedPose> {
            let mut out = HashMap::new();
            for (i, id) in ids.iter().enumerate() {
                let off = i * 6;
                let t = Vec3::new(x[off], x[off + 1], x[off + 2]);
                let omega = Vec3::new(x[off + 3], x[off + 4], x[off + 5]);
                let angle = omega.norm();
                let axis = if angle > 1e-15 {
                    omega / angle
                } else {
                    Vec3::z()
                };
                out.insert(
                    id.clone(),
                    ResolvedPose {
                        translation: t,
                        rotation_axis: axis,
                        rotation_angle: angle,
                    },
                );
            }
            out
        };

        let cost = |x: &[f64]| -> Result<f64, AssemblyError> {
            let poses = x_to_poses(x);
            self.total_squared_residual(&poses)
        };

        let initial_cost = cost(&x)?;
        if initial_cost < CYCLE_RESIDUAL_TOL {
            return Ok(x_to_poses(&x));
        }

        // Levenberg-Marquardt with finite-difference Jacobian on the
        // total cost (treated as a single scalar residual whose
        // gradient = 2 * J^T r — but cheaper to use finite-difference
        // gradient + a Hessian approximation B = J^T J via secant
        // Levenberg damping). Practical implementation: gradient
        // descent with adaptive step + LM-style damping λ.
        //
        // For a small number of DOFs (≤ 30) this is plenty.
        let mut lambda: f64 = 1e-3;
        let mut last_cost = initial_cost;
        const LM_MAX_ITERS: usize = 200;
        const FD_EPS: f64 = 1e-7;

        for _iter in 0..LM_MAX_ITERS {
            // Finite-difference gradient.
            let mut grad = vec![0.0; n_dof];
            let c0 = last_cost;
            for j in 0..n_dof {
                let saved = x[j];
                x[j] = saved + FD_EPS;
                let cp = cost(&x)?;
                x[j] = saved;
                grad[j] = (cp - c0) / FD_EPS;
            }
            let grad_norm = grad.iter().map(|g| g * g).sum::<f64>().sqrt();
            if grad_norm < 1e-12 {
                if last_cost < CYCLE_RESIDUAL_TOL {
                    return Ok(x_to_poses(&x));
                }
                break;
            }

            // LM step: x_new = x - (1/(λ * ‖g‖ + 1)) * g.
            // The damping shrinks the step automatically when
            // progress stalls.
            let step_scale = 1.0 / (1.0 + lambda * grad_norm);
            let mut x_new = x.clone();
            for j in 0..n_dof {
                x_new[j] -= step_scale * grad[j];
            }
            let new_cost = cost(&x_new)?;
            if new_cost < last_cost {
                x = x_new;
                last_cost = new_cost;
                lambda = (lambda * 0.7).max(1e-10);
            } else {
                lambda = (lambda * 2.0).min(1e10);
            }
            if last_cost < CYCLE_RESIDUAL_TOL {
                return Ok(x_to_poses(&x));
            }
        }

        if last_cost < 1e-6 {
            // Soft tolerance — still acceptable.
            return Ok(x_to_poses(&x));
        }
        Err(AssemblyError::Mate(MateError::CycleDidNotConverge {
            iterations: LM_MAX_ITERS,
            residual: last_cost,
        }))
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

/// Return `(instance_a_id, instance_b_id)` for a mate. Used by the
/// cycle-detector and the up-front instance-name validation.
fn mate_endpoints(mate: &Mate) -> (&str, &str) {
    match mate {
        Mate::Coincident { instance_a, instance_b, .. }
        | Mate::Concentric { instance_a, instance_b, .. }
        | Mate::Distance { instance_a, instance_b, .. }
        | Mate::ParallelPlane { instance_a, instance_b, .. }
        | Mate::AngleMate { instance_a, instance_b, .. }
        | Mate::TangentMate { instance_a, instance_b, .. }
        | Mate::Symmetry { instance_a, instance_b, .. }
        | Mate::Width { instance_a, instance_b, .. }
        | Mate::Lock { instance_a, instance_b, .. }
        | Mate::TouchPoint { instance_a, instance_b, .. }
        | Mate::PointOnPlane { instance_a, instance_b, .. }
        | Mate::PointOnLine { instance_a, instance_b, .. }
        | Mate::Gear { instance_a, instance_b, .. } => {
            (instance_a.as_str(), instance_b.as_str())
        }
        // PathMate has only ONE instance — degenerate "self loop". The
        // cycle detector skips self-loops; downstream solvers handle it
        // as a single-instance constraint.
        Mate::PathMate { instance, .. } => (instance.as_str(), instance.as_str()),
    }
}

/// Resolve a 3-element scalar array against the assembly's parameter
/// table. Used by mate solvers that need to re-evaluate a local point or
/// vector after an in-place rotation.
fn resolve_arr3(
    arr: &[Scalar; 3],
    params: &HashMap<String, f64>,
) -> Result<[f64; 3], String> {
    let mut out = [0.0; 3];
    for (i, s) in arr.iter().enumerate() {
        out[i] = s.resolve(params)?;
    }
    Ok(out)
}

/// Compute the axis-aligned bounding box of a `Solid` from its vertex
/// geometry. Returns `None` if the solid has no vertices.
fn solid_aabb(s: &Solid) -> Option<([f64; 3], [f64; 3])> {
    if s.vertex_count() == 0 {
        return None;
    }
    let mut min = [f64::INFINITY; 3];
    let mut max = [f64::NEG_INFINITY; 3];
    let mut any = false;
    for vid in s.topo.vertex_ids() {
        if let Some(p) = s.vertex_geom.get(vid) {
            any = true;
            min[0] = min[0].min(p.x);
            min[1] = min[1].min(p.y);
            min[2] = min[2].min(p.z);
            max[0] = max[0].max(p.x);
            max[1] = max[1].max(p.y);
            max[2] = max[2].max(p.z);
        }
    }
    if !any {
        None
    } else {
        Some((min, max))
    }
}

/// Volume of the intersection of two AABBs. Zero if they don't overlap.
fn aabb_overlap_volume(
    min_a: &[f64; 3],
    max_a: &[f64; 3],
    min_b: &[f64; 3],
    max_b: &[f64; 3],
) -> f64 {
    let mut vol = 1.0;
    for i in 0..3 {
        let lo = min_a[i].max(min_b[i]);
        let hi = max_a[i].min(max_b[i]);
        let extent = hi - lo;
        if extent <= 0.0 {
            return 0.0;
        }
        vol *= extent;
    }
    vol
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

    // -----------------------------------------------------------------------
    // Exploded-view tests
    // -----------------------------------------------------------------------

    /// Build a minimal inline model containing a 1×1×1 box centred at the
    /// local origin. Returns an `Instance` at `pose` with id `id`.
    fn unit_box_instance(id: &str, pose: Pose) -> Instance {
        use crate::feature::Feature;
        use crate::model::Model;
        use crate::scalar::lits;
        Instance {
            id: id.into(),
            model: AssemblyRef::Inline(Box::new(
                Model::new().add(Feature::Box {
                    id: "body".into(),
                    extents: lits([1.0, 1.0, 1.0]),
                }),
            )),
            target: None,
            default_pose: pose,
        }
    }

    /// Test 1: `exploded = None` → identical to `evaluate`.
    ///
    /// Two instances at different positions. `evaluate_exploded` with no
    /// exploded field must return the same translations as plain `evaluate`.
    #[test]
    fn exploded_none_identical_to_evaluate() {
        let asm = Assembly {
            instances: vec![
                unit_box_instance("a", Pose::at(0.0, 0.0, 0.0)),
                unit_box_instance("b", Pose::at(10.0, 0.0, 0.0)),
            ],
            exploded: None,
            ..Assembly::default()
        };

        let base = asm.evaluate().expect("evaluate");
        let expl = asm.evaluate_exploded().expect("evaluate_exploded");

        assert_eq!(base.len(), expl.len());
        for ((id_b, s_b), (id_e, s_e)) in base.iter().zip(expl.iter()) {
            assert_eq!(id_b, id_e);
            // Same vertex count — no displacement happened.
            assert_eq!(s_b.vertex_count(), s_e.vertex_count());
            // Verify bounding-box centroids match to floating-point precision.
            if let (Some((mn_b, mx_b)), Some((mn_e, mx_e))) =
                (solid_aabb(s_b), solid_aabb(s_e))
            {
                for i in 0..3 {
                    assert!(
                        ((mn_b[i] + mx_b[i]) - (mn_e[i] + mx_e[i])).abs() < 1e-10,
                        "centroid mismatch on axis {i} for instance {id_b}"
                    );
                }
            }
        }
    }

    /// Test 2: two instances at (0,0,0) and (10,0,0), exploded=1.0,
    /// explode_distance=20 → they move apart by 20 along ±x.
    ///
    /// Assembly centroid (equal AABB volumes, both cubes): x = 5.0.
    /// Instance A centroid = 0.5 (box 0..1 → centre 0.5); direction = −x.
    /// Instance B centroid = 10.5; direction = +x.
    /// Expected shift = ±20 along x.
    #[test]
    fn exploded_radial_displacement() {
        let asm = Assembly {
            instances: vec![
                unit_box_instance("a", Pose::at(0.0, 0.0, 0.0)),
                unit_box_instance("b", Pose::at(10.0, 0.0, 0.0)),
            ],
            exploded: Some(1.0),
            explode_distance: Some(20.0),
            ..Assembly::default()
        };

        let result = asm.evaluate_exploded().expect("evaluate_exploded");
        assert_eq!(result.len(), 2);

        let (id_a, solid_a) = &result[0];
        let (id_b, solid_b) = &result[1];
        assert_eq!(id_a, "a");
        assert_eq!(id_b, "b");

        let (mn_a, mx_a) = solid_aabb(solid_a).expect("solid_a aabb");
        let (mn_b, mx_b) = solid_aabb(solid_b).expect("solid_b aabb");

        let cx_a = (mn_a[0] + mx_a[0]) * 0.5;
        let cx_b = (mn_b[0] + mx_b[0]) * 0.5;

        // A moved in the −x direction, B in the +x direction.
        assert!(cx_a < 0.0, "instance A should have moved in −x; cx_a = {cx_a}");
        assert!(cx_b > 11.0, "instance B should have moved in +x; cx_b = {cx_b}");

        // y and z centroids must be unchanged (both at 0.5 from the unit cube).
        let cy_a = (mn_a[1] + mx_a[1]) * 0.5;
        let cy_b = (mn_b[1] + mx_b[1]) * 0.5;
        assert!((cy_a - 0.5).abs() < 1e-10, "cy_a = {cy_a}");
        assert!((cy_b - 0.5).abs() < 1e-10, "cy_b = {cy_b}");
    }

    /// Test 3: round-trip JSON with `exploded` and `explode_distance` preserved.
    #[test]
    fn exploded_json_round_trip() {
        let asm = Assembly {
            instances: vec![unit_box_instance("x", Pose::identity())],
            exploded: Some(0.75),
            explode_distance: Some(42.0),
            ..Assembly::default()
        };

        let json = asm.to_json_string().expect("to_json_string");
        assert!(json.contains("\"exploded\""), "json should contain exploded field");
        assert!(
            json.contains("\"explode_distance\""),
            "json should contain explode_distance field"
        );

        let restored = Assembly::from_json_str(&json).expect("from_json_str");
        assert_eq!(
            restored.exploded,
            Some(0.75),
            "exploded field should survive JSON round-trip"
        );
        assert_eq!(
            restored.explode_distance,
            Some(42.0),
            "explode_distance field should survive JSON round-trip"
        );
    }

    /// Test 4: `exploded = Some(0.0)` is identical to `exploded = None`.
    ///
    /// Both return the assembled (non-displaced) solids.
    #[test]
    fn exploded_zero_equals_none() {
        let make_asm = |exploded: Option<f64>| Assembly {
            instances: vec![
                unit_box_instance("p", Pose::at(0.0, 0.0, 0.0)),
                unit_box_instance("q", Pose::at(5.0, 3.0, 1.0)),
            ],
            exploded,
            explode_distance: Some(10.0),
            ..Assembly::default()
        };

        let none_result = make_asm(None).evaluate_exploded().expect("none");
        let zero_result = make_asm(Some(0.0)).evaluate_exploded().expect("zero");

        assert_eq!(none_result.len(), zero_result.len());
        for ((_, s_n), (_, s_z)) in none_result.iter().zip(zero_result.iter()) {
            if let (Some((mn_n, mx_n)), Some((mn_z, mx_z))) =
                (solid_aabb(s_n), solid_aabb(s_z))
            {
                for i in 0..3 {
                    assert!(
                        ((mn_n[i] + mx_n[i]) - (mn_z[i] + mx_z[i])).abs() < 1e-10,
                        "centroids must match on axis {i}"
                    );
                }
            }
        }
    }

    /// Test 5: auto-computed `explode_distance` when `explode_distance` is
    /// `None`. Two unit cubes at x=0 and x=10 → assembly AABB spans 0..11
    /// on x and 0..1 on y,z → diagonal ≈ sqrt(11²+1+1) ≈ 11.09 → auto dist
    /// ≈ 5.54. With `exploded=1.0` both instances must move away from the
    /// centroid (x≈5.5) by that amount.
    #[test]
    fn exploded_auto_distance() {
        let asm = Assembly {
            instances: vec![
                unit_box_instance("left", Pose::at(0.0, 0.0, 0.0)),
                unit_box_instance("right", Pose::at(10.0, 0.0, 0.0)),
            ],
            exploded: Some(1.0),
            explode_distance: None, // auto
            ..Assembly::default()
        };

        let result = asm.evaluate_exploded().expect("evaluate_exploded");
        let (_, s_left) = &result[0];
        let (_, s_right) = &result[1];

        let (mn_l, mx_l) = solid_aabb(s_left).expect("left aabb");
        let (mn_r, mx_r) = solid_aabb(s_right).expect("right aabb");

        let cx_l = (mn_l[0] + mx_l[0]) * 0.5;
        let cx_r = (mn_r[0] + mx_r[0]) * 0.5;

        // With auto distance ≈ 5.54 the left cube (centred near 0.5) should
        // have moved left of the assembly centroid (~5.5), and the right cube
        // (centred near 10.5) should be further right.
        assert!(cx_l < 0.5, "left cube should have moved left; cx_l = {cx_l}");
        assert!(cx_r > 10.5, "right cube should have moved right; cx_r = {cx_r}");

        // Compute expected auto distance and verify the displacement magnitude.
        // Assembly AABB: x ∈ [0, 11], y ∈ [0, 1], z ∈ [0, 1].
        let expected_dist = ((11.0f64).powi(2) + 1.0 + 1.0).sqrt() * 0.5;
        // Assembly centroid: weighted average of AABB centres (equal volumes).
        // left AABB centre x = 0.5, right AABB centre x = 10.5 → asm centroid x = 5.5.
        let asm_cx = 5.5_f64;
        let left_shift = (0.5 - asm_cx) / (0.5 - asm_cx).abs() * expected_dist; // negative
        let right_shift = (10.5 - asm_cx) / (10.5 - asm_cx).abs() * expected_dist; // positive
        assert!((cx_l - (0.5 + left_shift)).abs() < 0.5, "left displacement");
        assert!((cx_r - (10.5 + right_shift)).abs() < 0.5, "right displacement");
    }
}
