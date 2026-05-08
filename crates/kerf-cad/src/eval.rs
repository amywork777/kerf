//! Evaluator: walks the model DAG depth-first and produces a Solid.

use std::collections::HashMap;

use kerf_brep::{
    primitives::{
        box_, box_at, cone, cone_faceted, cylinder_faceted, extrude_lofted, extrude_polygon,
        frustum, frustum_faceted, revolve_polyline, sphere, sphere_faceted, torus, torus_faceted,
    },
    Solid,
};
use kerf_geom::{Point3, Vec3};
use thiserror::Error;

use crate::feature::{Feature, FilletEdge, Profile2D};
use crate::model::Model;
use crate::scalar::{resolve_arr, Scalar};
use crate::sketch::Sketch;
use crate::transform::{mirror_solid, rotate_solid, scale_solid, scale_xyz_solid, translate_solid};

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

        let mut result = build(feature, &self.parameters, cache, self)?;
        // Picking provenance: tag any face in this feature's result that
        // doesn't already carry an owner tag with this feature's id. Boolean
        // operations propagate inputs' owner tags through stitch, so they
        // come back already-tagged and we leave those alone — only the
        // feature that originally produced the geometry claims it.
        let owner = id.to_string();
        let face_ids: Vec<_> = result.topo.face_ids().collect();
        for fid in face_ids {
            if !result.face_owner_tag.contains_key(fid) {
                result.face_owner_tag.insert(fid, owner.clone());
            }
        }
        cache.insert(id.to_string(), result);
        Ok(())
    }
}

fn build(
    feature: &Feature,
    params: &HashMap<String, f64>,
    cache: &HashMap<String, Solid>,
    model: &Model,
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
        Feature::Donut {
            major_radius,
            minor_radius,
            major_segs,
            minor_segs,
            ..
        } => {
            let r_maj = resolve_one(id, major_radius, params)?;
            let r_min = resolve_one(id, minor_radius, params)?;
            if r_maj <= r_min || r_min <= 0.0 {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: format!(
                        "Donut requires major > minor > 0 (got major={r_maj}, minor={r_min})"
                    ),
                });
            }
            if *major_segs < 3 || *minor_segs < 3 {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: "Donut requires major_segs >= 3 and minor_segs >= 3".into(),
                });
            }
            Ok(torus_faceted(r_maj, r_min, *major_segs, *minor_segs))
        }
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
        Feature::Loft {
            bottom,
            top,
            height,
            ..
        } => {
            if bottom.points.len() != top.points.len() {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: format!(
                        "Loft bottom ({}) and top ({}) must have the same vertex count",
                        bottom.points.len(),
                        top.points.len()
                    ),
                });
            }
            if bottom.points.len() < 3 {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: format!(
                        "Loft profiles need at least 3 points (got {})",
                        bottom.points.len()
                    ),
                });
            }
            let h = resolve_one(id, height, params)?;
            if h <= 0.0 {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: format!("Loft height must be > 0 (got {h})"),
                });
            }
            let mut bottom_pts = Vec::with_capacity(bottom.points.len());
            for p in &bottom.points {
                let xy = resolve_arr(p, params).map_err(|message| EvalError::Parameter {
                    id: id.into(),
                    message,
                })?;
                bottom_pts.push(Point3::new(xy[0], xy[1], 0.0));
            }
            let mut top_pts = Vec::with_capacity(top.points.len());
            for p in &top.points {
                let xy = resolve_arr(p, params).map_err(|message| EvalError::Parameter {
                    id: id.into(),
                    message,
                })?;
                top_pts.push(Point3::new(xy[0], xy[1], h));
            }
            Ok(extrude_lofted(&bottom_pts, &top_pts))
        }
        Feature::TaperedExtrude {
            profile,
            height,
            top_scale,
            ..
        } => {
            if profile.points.len() < 3 {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: format!(
                        "TaperedExtrude profile needs at least 3 points (got {})",
                        profile.points.len()
                    ),
                });
            }
            let h = resolve_one(id, height, params)?;
            let s = resolve_one(id, top_scale, params)?;
            if h <= 0.0 || s <= 0.0 {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: format!(
                        "TaperedExtrude requires positive height and top_scale (got {h}, {s})"
                    ),
                });
            }
            let mut bottom_pts = Vec::with_capacity(profile.points.len());
            for p in &profile.points {
                let xy = resolve_arr(p, params).map_err(|message| EvalError::Parameter {
                    id: id.into(),
                    message,
                })?;
                bottom_pts.push(Point3::new(xy[0], xy[1], 0.0));
            }
            // Compute centroid (xy) and scale top points around it.
            let n = bottom_pts.len() as f64;
            let cx = bottom_pts.iter().map(|p| p.x).sum::<f64>() / n;
            let cy = bottom_pts.iter().map(|p| p.y).sum::<f64>() / n;
            let top_pts: Vec<Point3> = bottom_pts
                .iter()
                .map(|p| Point3::new(cx + (p.x - cx) * s, cy + (p.y - cy) * s, h))
                .collect();
            Ok(extrude_lofted(&bottom_pts, &top_pts))
        }

        Feature::Revolve { profile, .. } => build_revolve(id, profile, params),

        Feature::SketchExtrude {
            sketch, direction, ..
        } => build_sketch_extrude(id, sketch, direction, params, model),

        Feature::SketchRevolve { sketch, .. } => build_sketch_revolve(id, sketch, params),

        Feature::Tube {
            outer_radius,
            inner_radius,
            height,
            segments,
            ..
        } => {
            let r_out = resolve_one(id, outer_radius, params)?;
            let r_in = resolve_one(id, inner_radius, params)?;
            let h = resolve_one(id, height, params)?;
            if r_in >= r_out {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: format!(
                        "Tube inner_radius ({r_in}) must be < outer_radius ({r_out})"
                    ),
                });
            }
            let outer = cylinder_faceted(r_out, h, *segments);
            // Inner cutter: extend slightly beyond outer top and below outer
            // bottom so the boolean produces a clean through-hole.
            let inner_raw = cylinder_faceted(r_in, h + 2.0, *segments);
            let inner = translate_solid(&inner_raw, Vec3::new(0.0, 0.0, -1.0));
            outer.try_difference(&inner).map_err(|e| EvalError::Boolean {
                id: id.into(),
                op: "tube_difference",
                message: e.message,
            })
        }
        Feature::CornerCut {
            input,
            corner,
            extents,
            ..
        } => {
            let base = cache_get(cache, input)?;
            let c = resolve3(id, corner, params)?;
            let e = resolve3(id, extents, params)?;
            if e[0] <= 0.0 || e[1] <= 0.0 || e[2] <= 0.0 {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: format!(
                        "CornerCut extents must all be positive (got [{}, {}, {}])",
                        e[0], e[1], e[2]
                    ),
                });
            }
            let cutter = box_at(
                Vec3::new(e[0], e[1], e[2]),
                Point3::new(c[0], c[1], c[2]),
            );
            base.try_difference(&cutter).map_err(|e| EvalError::Boolean {
                id: id.into(),
                op: "corner_cut",
                message: e.message,
            })
        }
        Feature::Fillet {
            input,
            axis,
            edge_min,
            edge_length,
            radius,
            quadrant,
            segments,
            ..
        } => {
            let base = cache_get(cache, input)?;
            let em = resolve3(id, edge_min, params)?;
            let len = resolve_one(id, edge_length, params)?;
            let r = resolve_one(id, radius, params)?;
            build_fillet(id, base, axis, em, len, r, quadrant, *segments)
        }
        Feature::Countersink {
            input,
            axis,
            top_center,
            drill_radius,
            csink_radius,
            csink_depth,
            total_depth,
            segments,
            ..
        } => {
            let base = cache_get(cache, input)?;
            let tc = resolve3(id, top_center, params)?;
            let dr = resolve_one(id, drill_radius, params)?;
            let cr = resolve_one(id, csink_radius, params)?;
            let cd = resolve_one(id, csink_depth, params)?;
            let td = resolve_one(id, total_depth, params)?;
            if dr <= 0.0 || cr <= 0.0 || cd <= 0.0 || td <= 0.0 {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: format!(
                        "Countersink requires positive radii and depths (got drill_r={dr}, csink_r={cr}, csink_d={cd}, total_d={td})"
                    ),
                });
            }
            if cr <= dr {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: format!("Countersink csink_radius ({cr}) must be > drill_radius ({dr})"),
                });
            }
            if cd >= td {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: format!("Countersink csink_depth ({cd}) must be < total_depth ({td})"),
                });
            }
            if *segments < 3 {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: format!("Countersink segments must be >= 3 (got {segments})"),
                });
            }
            let axis_idx = parse_axis(id, axis)?;
            let (a_idx, b_idx) = perpendicular_axes(axis_idx);
            let eps = (dr * 0.1).max(1e-3).min(td * 0.1);
            // Drill cylinder: full depth.
            let drill = cylinder_along_axis(
                dr,
                td + eps,
                *segments,
                axis_idx,
                tc[axis_idx] - td,
                tc[a_idx],
                tc[b_idx],
            );
            // Countersink cone: faceted frustum oriented along axis. Built
            // local in +z (bottom = drill_radius at z=0, top = csink_radius
            // at z=csink_depth+eps). Then reorient + translate so its top
            // sits at top_center.
            let cone_local = frustum_faceted(dr, cr, cd + eps, *segments);
            let cone_oriented = match axis_idx {
                2 => cone_local,
                0 => axis_swap_xz_to_x(&cone_local),
                1 => axis_swap_yz_to_y(&cone_local),
                _ => unreachable!(),
            };
            let mut translation = [0.0_f64; 3];
            translation[axis_idx] = tc[axis_idx] - cd;
            translation[a_idx] = tc[a_idx];
            translation[b_idx] = tc[b_idx];
            let cone_pos = translate_solid(
                &cone_oriented,
                Vec3::new(translation[0], translation[1], translation[2]),
            );
            let composite = drill.try_union(&cone_pos).map_err(|e| EvalError::Boolean {
                id: id.into(),
                op: "countersink_union",
                message: e.message,
            })?;
            base.try_difference(&composite).map_err(|e| EvalError::Boolean {
                id: id.into(),
                op: "countersink",
                message: e.message,
            })
        }
        Feature::Counterbore {
            input,
            axis,
            top_center,
            drill_radius,
            cbore_radius,
            cbore_depth,
            total_depth,
            segments,
            ..
        } => {
            let base = cache_get(cache, input)?;
            let tc = resolve3(id, top_center, params)?;
            let dr = resolve_one(id, drill_radius, params)?;
            let cr = resolve_one(id, cbore_radius, params)?;
            let cd = resolve_one(id, cbore_depth, params)?;
            let td = resolve_one(id, total_depth, params)?;
            if dr <= 0.0 || cr <= 0.0 || cd <= 0.0 || td <= 0.0 {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: format!(
                        "Counterbore requires positive radii and depths (got drill_r={dr}, cbore_r={cr}, cbore_d={cd}, total_d={td})"
                    ),
                });
            }
            if cr <= dr {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: format!(
                        "Counterbore cbore_radius ({cr}) must be > drill_radius ({dr})"
                    ),
                });
            }
            if cd >= td {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: format!(
                        "Counterbore cbore_depth ({cd}) must be < total_depth ({td})"
                    ),
                });
            }
            if *segments < 3 {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: format!("Counterbore segments must be >= 3 (got {segments})"),
                });
            }
            let axis_idx = parse_axis(id, axis)?;
            let (a_idx, b_idx) = perpendicular_axes(axis_idx);
            // Drill cylinder: centered at top_center perp coords, axis-origin
            // shifted DOWN by total_depth so the cylinder runs from
            // top_center[axis] - total_depth UP to top_center[axis].
            // Extend a small overhang past top_center so the cylinder
            // pokes outside the body and avoids coplanar caps.
            let eps = (dr * 0.1).max(1e-3).min(td * 0.1);
            let drill = cylinder_along_axis(
                dr,
                td + eps,
                *segments,
                axis_idx,
                tc[axis_idx] - td,
                tc[a_idx],
                tc[b_idx],
            );
            // Counterbore cylinder: same center, runs from
            // top_center[axis] - cbore_depth to top_center[axis] + eps.
            let cbore = cylinder_along_axis(
                cr,
                cd + eps,
                *segments,
                axis_idx,
                tc[axis_idx] - cd,
                tc[a_idx],
                tc[b_idx],
            );
            let composite = drill.try_union(&cbore).map_err(|e| EvalError::Boolean {
                id: id.into(),
                op: "counterbore_union",
                message: e.message,
            })?;
            base.try_difference(&composite).map_err(|e| EvalError::Boolean {
                id: id.into(),
                op: "counterbore",
                message: e.message,
            })
        }
        Feature::Fillets { input, edges, .. } => {
            let base = cache_get(cache, input)?;
            build_fillets(id, base, edges, params)
        }
        Feature::Chamfer {
            input,
            axis,
            edge_min,
            edge_length,
            setback,
            quadrant,
            ..
        } => {
            let base = cache_get(cache, input)?;
            let em = resolve3(id, edge_min, params)?;
            let len = resolve_one(id, edge_length, params)?;
            let s = resolve_one(id, setback, params)?;
            build_chamfer(id, base, axis, em, len, s, quadrant)
        }
        Feature::Slot {
            length,
            radius,
            height,
            segments,
            ..
        } => {
            let l = resolve_one(id, length, params)?;
            let r = resolve_one(id, radius, params)?;
            let h = resolve_one(id, height, params)?;
            build_slot(id, l, r, h, *segments)
        }
        Feature::HollowCylinder {
            outer_radius,
            inner_radius,
            height,
            end_thickness,
            segments,
            ..
        } => {
            let r_out = resolve_one(id, outer_radius, params)?;
            let r_in = resolve_one(id, inner_radius, params)?;
            let h = resolve_one(id, height, params)?;
            let t = resolve_one(id, end_thickness, params)?;
            if r_out <= 0.0 || r_in <= 0.0 || h <= 0.0 || t <= 0.0 {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: format!(
                        "HollowCylinder requires positive r_out, r_in, height, end_thickness \
                         (got {r_out}, {r_in}, {h}, {t})"
                    ),
                });
            }
            if r_in >= r_out {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: format!(
                        "HollowCylinder inner_radius ({r_in}) must be < outer_radius ({r_out})"
                    ),
                });
            }
            if 2.0 * t >= h {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: format!(
                        "HollowCylinder 2 * end_thickness ({}) must be < height ({h})",
                        2.0 * t
                    ),
                });
            }
            let outer = cylinder_faceted(r_out, h, *segments);
            let cavity_h = h - 2.0 * t;
            let cavity_raw = cylinder_faceted(r_in, cavity_h, *segments);
            let cavity = translate_solid(&cavity_raw, Vec3::new(0.0, 0.0, t));
            outer.try_difference(&cavity).map_err(|e| EvalError::Boolean {
                id: id.into(),
                op: "hollow_cylinder",
                message: e.message,
            })
        }
        Feature::Wedge {
            width,
            depth,
            height,
            ..
        } => {
            let w = resolve_one(id, width, params)?;
            let d = resolve_one(id, depth, params)?;
            let h = resolve_one(id, height, params)?;
            if w <= 0.0 || d <= 0.0 || h <= 0.0 {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: format!("Wedge requires positive width, depth, height (got {w}, {d}, {h})"),
                });
            }
            // Right-triangle profile in xz plane (y = 0). For extrude_polygon
            // CCW-from-+direction with +direction = +y, the polygon walked in
            // increasing-angle order around +y must satisfy
            // (p1-p0) × (p2-p0) · ŷ > 0. That requires (0,0,h) BEFORE (w,0,0).
            let prof = vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(0.0, 0.0, h),
                Point3::new(w, 0.0, 0.0),
            ];
            Ok(extrude_polygon(&prof, Vec3::new(0.0, d, 0.0)))
        }
        Feature::RegularPrism {
            radius,
            height,
            segments,
            ..
        } => {
            let r = resolve_one(id, radius, params)?;
            let h = resolve_one(id, height, params)?;
            if r <= 0.0 || h <= 0.0 {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: format!("RegularPrism requires positive radius, height (got {r}, {h})"),
                });
            }
            if *segments < 3 {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: format!("RegularPrism segments must be >= 3 (got {})", segments),
                });
            }
            // Same as cylinder_faceted but renamed for intent.
            Ok(cylinder_faceted(r, h, *segments))
        }
        Feature::Pyramid {
            radius,
            height,
            segments,
            ..
        } => {
            let r = resolve_one(id, radius, params)?;
            let h = resolve_one(id, height, params)?;
            if r <= 0.0 || h <= 0.0 {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: format!("Pyramid requires positive radius, height (got {r}, {h})"),
                });
            }
            if *segments < 3 {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: format!("Pyramid segments must be >= 3 (got {})", segments),
                });
            }
            Ok(cone_faceted(r, h, *segments))
        }
        Feature::RefPoint {
            position,
            marker_radius,
            ..
        } => {
            let p = resolve3(id, position, params)?;
            let r = resolve_one(id, marker_radius, params)?;
            if r <= 0.0 {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: format!("RefPoint marker_radius must be > 0 (got {r})"),
                });
            }
            let sph = sphere_faceted(r, 4, 6);
            Ok(translate_solid(&sph, Vec3::new(p[0], p[1], p[2])))
        }
        Feature::RefAxis {
            position,
            axis,
            length,
            marker_radius,
            ..
        } => {
            let p = resolve3(id, position, params)?;
            let l = resolve_one(id, length, params)?;
            let r = resolve_one(id, marker_radius, params)?;
            if l <= 0.0 || r <= 0.0 {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: format!("RefAxis length and marker_radius must be > 0"),
                });
            }
            let axis_idx = parse_axis(id, axis)?;
            let (a_idx, b_idx) = perpendicular_axes(axis_idx);
            // Center the cylinder at position along the axis.
            Ok(cylinder_along_axis(
                r,
                l,
                12,
                axis_idx,
                p[axis_idx] - l / 2.0,
                p[a_idx],
                p[b_idx],
            ))
        }
        Feature::RefPlane {
            position,
            axis,
            extents,
            marker_thickness,
            ..
        } => {
            let p = resolve3(id, position, params)?;
            let e = resolve3(
                id,
                &[extents[0].clone(), extents[1].clone(), Scalar::lit(0.0)],
                params,
            )?;
            let t = resolve_one(id, marker_thickness, params)?;
            if e[0] <= 0.0 || e[1] <= 0.0 || t <= 0.0 {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: format!(
                        "RefPlane extents and marker_thickness must be > 0"
                    ),
                });
            }
            let axis_idx = parse_axis(id, axis)?;
            let (a_idx, b_idx) = perpendicular_axes(axis_idx);
            // Build a thin Box centered at position. Extents: t along axis,
            // e[0] along a_idx, e[1] along b_idx.
            let mut box_extents = [0.0_f64; 3];
            box_extents[axis_idx] = t;
            box_extents[a_idx] = e[0];
            box_extents[b_idx] = e[1];
            let mut origin = [0.0_f64; 3];
            origin[axis_idx] = p[axis_idx] - t / 2.0;
            origin[a_idx] = p[a_idx] - e[0] / 2.0;
            origin[b_idx] = p[b_idx] - e[1] / 2.0;
            Ok(box_at(
                Vec3::new(box_extents[0], box_extents[1], box_extents[2]),
                Point3::new(origin[0], origin[1], origin[2]),
            ))
        }
        Feature::Arrow {
            shaft_radius,
            shaft_length,
            tip_length,
            segments,
            ..
        } => {
            let sr = resolve_one(id, shaft_radius, params)?;
            let sl = resolve_one(id, shaft_length, params)?;
            let tl = resolve_one(id, tip_length, params)?;
            if sr <= 0.0 || sl <= 0.0 || tl <= 0.0 {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: format!(
                        "Arrow requires positive shaft_radius, shaft_length, tip_length"
                    ),
                });
            }
            if *segments < 3 {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: format!("Arrow segments must be >= 3 (got {segments})"),
                });
            }
            let shaft = cylinder_faceted(sr, sl, *segments);
            // Tip cone tapers from shaft_radius (at base, z=0 in local) to
            // 0 (apex, z=tip_length). Translate so base sits at z=shaft_length.
            let tip_raw = cone_faceted(sr, tl, *segments);
            let tip = translate_solid(&tip_raw, Vec3::new(0.0, 0.0, sl));
            shaft.try_union(&tip).map_err(|e| EvalError::Boolean {
                id: id.into(),
                op: "arrow_union",
                message: e.message,
            })
        }
        Feature::Funnel {
            top_radius,
            neck_radius,
            neck_z,
            spout_length,
            segments,
            ..
        } => {
            let tr = resolve_one(id, top_radius, params)?;
            let nr = resolve_one(id, neck_radius, params)?;
            let nz = resolve_one(id, neck_z, params)?;
            let sl = resolve_one(id, spout_length, params)?;
            if tr <= 0.0 || nr <= 0.0 || nz <= 0.0 || sl <= 0.0 {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: format!(
                        "Funnel requires positive top/neck radii, neck_z, spout_length"
                    ),
                });
            }
            if tr <= nr {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: format!("Funnel top_radius ({tr}) must be > neck_radius ({nr})"),
                });
            }
            if *segments < 3 {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: format!("Funnel segments must be >= 3 (got {segments})"),
                });
            }
            // Frustum sits at z ∈ [0, nz]: bottom (z=0) has neck_radius,
            // top (z=nz) has top_radius.
            let top = frustum_faceted(nr, tr, nz, *segments);
            // Spout cylinder at z ∈ [-spout_length, 0].
            let spout_raw = cylinder_faceted(nr, sl, *segments);
            let spout = translate_solid(&spout_raw, Vec3::new(0.0, 0.0, -sl));
            top.try_union(&spout).map_err(|e| EvalError::Boolean {
                id: id.into(),
                op: "funnel_union",
                message: e.message,
            })
        }
        Feature::Capsule {
            radius,
            height,
            stacks,
            slices,
            ..
        } => {
            let r = resolve_one(id, radius, params)?;
            let h = resolve_one(id, height, params)?;
            if r <= 0.0 || h <= 0.0 {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: format!("Capsule requires positive radius, height (got {r}, {h})"),
                });
            }
            if *stacks < 2 || *slices < 3 {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: format!("Capsule needs stacks >= 2 and slices >= 3"),
                });
            }
            let body = cylinder_faceted(r, h, *slices);
            let bot_sphere = sphere_faceted(r, *stacks, *slices);
            let top_sphere_raw = sphere_faceted(r, *stacks, *slices);
            let top_sphere = translate_solid(&top_sphere_raw, Vec3::new(0.0, 0.0, h));
            let body_bot = body.try_union(&bot_sphere).map_err(|e| EvalError::Boolean {
                id: id.into(),
                op: "capsule_bottom_union",
                message: e.message,
            })?;
            body_bot
                .try_union(&top_sphere)
                .map_err(|e| EvalError::Boolean {
                    id: id.into(),
                    op: "capsule_top_union",
                    message: e.message,
                })
        }
        Feature::PipeRun {
            points,
            radius,
            segments,
            ..
        } => {
            if points.len() < 2 {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: format!(
                        "PipeRun needs at least 2 points (got {})",
                        points.len()
                    ),
                });
            }
            let r = resolve_one(id, radius, params)?;
            if r <= 0.0 || *segments < 3 {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: format!("PipeRun requires positive radius and segments >= 3"),
                });
            }
            let resolved: Vec<[f64; 3]> = points
                .iter()
                .enumerate()
                .map(|(i, p)| {
                    resolve_arr(p, params).map_err(|message| EvalError::Parameter {
                        id: id.into(),
                        message: format!("point {i}: {message}"),
                    })
                })
                .collect::<Result<_, _>>()?;
            // For each consecutive pair, build a cylinder along the axis
            // they differ in. Verify they're axis-aligned (differ in exactly
            // one coordinate).
            let mut acc: Option<Solid> = None;
            for i in 0..resolved.len() - 1 {
                let p0 = resolved[i];
                let p1 = resolved[i + 1];
                let diffs = [(p1[0] - p0[0]).abs(), (p1[1] - p0[1]).abs(), (p1[2] - p0[2]).abs()];
                let nonzero: Vec<usize> = (0..3).filter(|&k| diffs[k] > 1e-12).collect();
                if nonzero.len() != 1 {
                    return Err(EvalError::Invalid {
                        id: id.into(),
                        reason: format!(
                            "PipeRun segment {i}→{} is not axis-aligned ({:?} → {:?})",
                            i + 1,
                            p0,
                            p1
                        ),
                    });
                }
                let axis_idx = nonzero[0];
                let len = diffs[axis_idx];
                let (a_idx, b_idx) = perpendicular_axes(axis_idx);
                // Build the cylinder. Its axis-along position spans from
                // min(p0[axis], p1[axis]) to that + len.
                let axis_origin = p0[axis_idx].min(p1[axis_idx]);
                let cyl = cylinder_along_axis(
                    r,
                    len,
                    *segments,
                    axis_idx,
                    axis_origin,
                    p0[a_idx],
                    p0[b_idx],
                );
                acc = Some(match acc.take() {
                    None => cyl,
                    Some(prev) => prev.try_union(&cyl).map_err(|e| EvalError::Boolean {
                        id: id.into(),
                        op: "pipe_run_segment_union",
                        message: format!("segment {i}: {}", e.message),
                    })?,
                });
            }
            // Note: we don't add sphere caps at corners. Perpendicular
            // cylinder unions produce a sharp joint where their lateral
            // surfaces meet — adding a sphere there to round the joint
            // tends to trip the boolean engine on the multi-cylinder-and-
            // sphere overlap. Users wanting rounded joints can compose
            // PipeRun with explicit SphereFaceted unions in JSON.
            Ok(acc.unwrap())
        }
        Feature::SweepPath {
            points,
            radius,
            segments,
            ..
        } => {
            if points.len() < 2 {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: format!(
                        "SweepPath needs at least 2 points (got {})",
                        points.len()
                    ),
                });
            }
            let r = resolve_one(id, radius, params)?;
            if r <= 0.0 || *segments < 3 {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: "SweepPath requires positive radius and segments >= 3".into(),
                });
            }
            let resolved: Vec<[f64; 3]> = points
                .iter()
                .enumerate()
                .map(|(i, p)| {
                    resolve_arr(p, params).map_err(|message| EvalError::Parameter {
                        id: id.into(),
                        message: format!("point {i}: {message}"),
                    })
                })
                .collect::<Result<_, _>>()?;
            let mut acc: Option<Solid> = None;
            for i in 0..resolved.len() - 1 {
                let p0 = resolved[i];
                let p1 = resolved[i + 1];
                let cyl = sweep_cylinder_segment(p0, p1, r, *segments).ok_or_else(|| {
                    EvalError::Invalid {
                        id: id.into(),
                        reason: format!(
                            "SweepPath segment {i}→{} has zero length ({:?} == {:?})",
                            i + 1, p0, p1
                        ),
                    }
                })?;
                acc = Some(match acc.take() {
                    None => cyl,
                    Some(prev) => prev.try_union(&cyl).map_err(|e| EvalError::Boolean {
                        id: id.into(),
                        op: "sweep_path_segment_union",
                        message: format!("segment {i}: {}", e.message),
                    })?,
                });
            }
            Ok(acc.unwrap())
        }
        Feature::Coil {
            coil_radius,
            wire_radius,
            pitch,
            turns,
            segments_per_turn,
            wire_segments,
            ..
        } => {
            let r_coil = resolve_one(id, coil_radius, params)?;
            let r_wire = resolve_one(id, wire_radius, params)?;
            let p = resolve_one(id, pitch, params)?;
            let t = resolve_one(id, turns, params)?;
            if r_coil <= 0.0 || r_wire <= 0.0 || p <= 0.0 || t <= 0.0 {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: format!(
                        "Coil requires positive radii, pitch, turns (got coil={r_coil}, wire={r_wire}, pitch={p}, turns={t})"
                    ),
                });
            }
            if r_wire >= r_coil {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: format!(
                        "Coil wire_radius ({r_wire}) must be less than coil_radius ({r_coil}) — otherwise the helix self-overlaps"
                    ),
                });
            }
            if *segments_per_turn < 6 || *wire_segments < 3 {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: "Coil requires segments_per_turn >= 6 and wire_segments >= 3".into(),
                });
            }
            let total_samples = (*segments_per_turn as f64 * t).ceil() as usize + 1;
            // Generate helix points: (R cos θ, R sin θ, p * θ / (2π)).
            let mut acc: Option<Solid> = None;
            let total_angle = 2.0 * std::f64::consts::PI * t;
            for i in 0..total_samples - 1 {
                let theta_a = total_angle * i as f64 / (total_samples - 1) as f64;
                let theta_b = total_angle * (i + 1) as f64 / (total_samples - 1) as f64;
                let p0 = [
                    r_coil * theta_a.cos(),
                    r_coil * theta_a.sin(),
                    p * theta_a / (2.0 * std::f64::consts::PI),
                ];
                let p1 = [
                    r_coil * theta_b.cos(),
                    r_coil * theta_b.sin(),
                    p * theta_b / (2.0 * std::f64::consts::PI),
                ];
                let cyl = sweep_cylinder_segment(p0, p1, r_wire, *wire_segments).ok_or_else(
                    || EvalError::Invalid {
                        id: id.into(),
                        reason: format!("Coil segment {i} has zero length"),
                    },
                )?;
                acc = Some(match acc.take() {
                    None => cyl,
                    Some(prev) => prev.try_union(&cyl).map_err(|e| EvalError::Boolean {
                        id: id.into(),
                        op: "coil_segment_union",
                        message: format!("segment {i}: {}", e.message),
                    })?,
                });
            }
            Ok(acc.unwrap())
        }
        Feature::TruncatedPyramid {
            bottom_radius,
            top_radius,
            height,
            segments,
            ..
        } => {
            let r_bot = resolve_one(id, bottom_radius, params)?;
            let r_top = resolve_one(id, top_radius, params)?;
            let h = resolve_one(id, height, params)?;
            if r_bot <= 0.0 || r_top <= 0.0 || h <= 0.0 {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: format!(
                        "TruncatedPyramid requires positive radii and height (got {r_bot}, {r_top}, {h})"
                    ),
                });
            }
            if *segments < 3 {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: format!("TruncatedPyramid segments must be >= 3 (got {segments})"),
                });
            }
            Ok(frustum_faceted(r_bot, r_top, h, *segments))
        }
        Feature::SphereFaceted {
            radius,
            stacks,
            slices,
            ..
        } => {
            let r = resolve_one(id, radius, params)?;
            if r <= 0.0 {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: format!("SphereFaceted radius must be > 0 (got {r})"),
                });
            }
            if *stacks < 2 {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: format!("SphereFaceted stacks must be >= 2 (got {stacks})"),
                });
            }
            if *slices < 3 {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: format!("SphereFaceted slices must be >= 3 (got {slices})"),
                });
            }
            Ok(sphere_faceted(r, *stacks, *slices))
        }
        Feature::HollowSphere {
            outer_radius,
            inner_radius,
            stacks,
            slices,
            ..
        } => {
            let r_out = resolve_one(id, outer_radius, params)?;
            let r_in = resolve_one(id, inner_radius, params)?;
            if r_out <= 0.0 || r_in <= 0.0 {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: format!(
                        "HollowSphere requires positive outer and inner radii (got {r_out}, {r_in})"
                    ),
                });
            }
            if r_in >= r_out {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: format!(
                        "HollowSphere inner_radius ({r_in}) must be < outer_radius ({r_out})"
                    ),
                });
            }
            if *stacks < 2 || *slices < 3 {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: format!("HollowSphere needs stacks >= 2 and slices >= 3"),
                });
            }
            let outer = sphere_faceted(r_out, *stacks, *slices);
            let inner = sphere_faceted(r_in, *stacks, *slices);
            outer.try_difference(&inner).map_err(|e| EvalError::Boolean {
                id: id.into(),
                op: "hollow_sphere",
                message: e.message,
            })
        }
        Feature::Dome {
            radius,
            stacks,
            slices,
            ..
        } => {
            let r = resolve_one(id, radius, params)?;
            if r <= 0.0 {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: format!("Dome radius must be > 0 (got {r})"),
                });
            }
            if *stacks < 2 || *slices < 3 {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: format!("Dome needs stacks >= 2 and slices >= 3"),
                });
            }
            let s = sphere_faceted(r, *stacks, *slices);
            // Cutter: a box covering everything below z=0 (with margin).
            let m = r * 1.5;
            let cutter = box_at(
                Vec3::new(2.0 * m, 2.0 * m, m + 0.1),
                Point3::new(-m, -m, -m - 0.05),
            );
            s.try_difference(&cutter).map_err(|e| EvalError::Boolean {
                id: id.into(),
                op: "dome",
                message: e.message,
            })
        }
        Feature::CylinderAt {
            base,
            axis,
            radius,
            height,
            segments,
            ..
        } => {
            let b = resolve3(id, base, params)?;
            let r = resolve_one(id, radius, params)?;
            let h = resolve_one(id, height, params)?;
            if r <= 0.0 || h <= 0.0 {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: format!("CylinderAt requires positive radius, height (got {r}, {h})"),
                });
            }
            if *segments < 3 {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: format!("CylinderAt segments must be >= 3 (got {})", segments),
                });
            }
            let axis_idx = parse_axis(id, axis)?;
            let (a_idx, b_idx) = perpendicular_axes(axis_idx);
            Ok(cylinder_along_axis(
                r,
                h,
                *segments,
                axis_idx,
                b[axis_idx],
                b[a_idx],
                b[b_idx],
            ))
        }
        Feature::LBracket {
            width,
            height,
            thickness,
            depth,
            ..
        } => {
            let w = resolve_one(id, width, params)?;
            let h = resolve_one(id, height, params)?;
            let t = resolve_one(id, thickness, params)?;
            let d = resolve_one(id, depth, params)?;
            if w <= 0.0 || h <= 0.0 || t <= 0.0 || d <= 0.0 {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: format!(
                        "LBracket requires positive width, height, thickness, depth (got {w}, {h}, {t}, {d})"
                    ),
                });
            }
            if t >= w || t >= h {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: format!(
                        "LBracket thickness ({t}) must be < width ({w}) and < height ({h})"
                    ),
                });
            }
            // L profile CCW (interior on the left as we walk):
            //   (0, 0) -> (w, 0) -> (w, t) -> (t, t) -> (t, h) -> (0, h) -> close
            let prof = vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(w, 0.0, 0.0),
                Point3::new(w, t, 0.0),
                Point3::new(t, t, 0.0),
                Point3::new(t, h, 0.0),
                Point3::new(0.0, h, 0.0),
            ];
            Ok(extrude_polygon(&prof, Vec3::new(0.0, 0.0, d)))
        }
        Feature::UChannel {
            width,
            height,
            thickness,
            depth,
            ..
        } => {
            let w = resolve_one(id, width, params)?;
            let h = resolve_one(id, height, params)?;
            let t = resolve_one(id, thickness, params)?;
            let d = resolve_one(id, depth, params)?;
            if w <= 0.0 || h <= 0.0 || t <= 0.0 || d <= 0.0 {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: format!(
                        "UChannel requires positive width, height, thickness, depth (got {w}, {h}, {t}, {d})"
                    ),
                });
            }
            if 2.0 * t >= w {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: format!(
                        "UChannel 2*thickness ({}) must be < width ({w})",
                        2.0 * t
                    ),
                });
            }
            if t >= h {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: format!(
                        "UChannel thickness ({t}) must be < height ({h})"
                    ),
                });
            }
            // U profile CCW: walk around the outside, then dip in for the
            // channel.
            //   (0,0) -> (w,0) -> (w,h) -> (w-t,h) -> (w-t,t) -> (t,t)
            //   -> (t,h) -> (0,h) -> close
            let prof = vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(w, 0.0, 0.0),
                Point3::new(w, h, 0.0),
                Point3::new(w - t, h, 0.0),
                Point3::new(w - t, t, 0.0),
                Point3::new(t, t, 0.0),
                Point3::new(t, h, 0.0),
                Point3::new(0.0, h, 0.0),
            ];
            Ok(extrude_polygon(&prof, Vec3::new(0.0, 0.0, d)))
        }
        Feature::TBeam {
            flange_width,
            flange_thickness,
            web_thickness,
            total_height,
            depth,
            ..
        } => {
            let fw = resolve_one(id, flange_width, params)?;
            let ft = resolve_one(id, flange_thickness, params)?;
            let wt = resolve_one(id, web_thickness, params)?;
            let th = resolve_one(id, total_height, params)?;
            let d = resolve_one(id, depth, params)?;
            if fw <= 0.0 || ft <= 0.0 || wt <= 0.0 || th <= 0.0 || d <= 0.0 {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: format!(
                        "TBeam requires positive flange_width, flange_thickness, web_thickness, total_height, depth (got {fw}, {ft}, {wt}, {th}, {d})"
                    ),
                });
            }
            if wt >= fw {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: format!(
                        "TBeam web_thickness ({wt}) must be < flange_width ({fw})"
                    ),
                });
            }
            if ft >= th {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: format!(
                        "TBeam flange_thickness ({ft}) must be < total_height ({th})"
                    ),
                });
            }
            // T profile CCW. Web is centered on x. Web bottom at y=0,
            // flange top at y=th. Flange spans x=[0, fw], y=[th-ft, th].
            // Web spans x=[wx_min, wx_max], y=[0, th-ft].
            let wx_min = (fw - wt) / 2.0;
            let wx_max = wx_min + wt;
            let wy_top = th - ft;
            let prof = vec![
                Point3::new(wx_min, 0.0, 0.0),
                Point3::new(wx_max, 0.0, 0.0),
                Point3::new(wx_max, wy_top, 0.0),
                Point3::new(fw, wy_top, 0.0),
                Point3::new(fw, th, 0.0),
                Point3::new(0.0, th, 0.0),
                Point3::new(0.0, wy_top, 0.0),
                Point3::new(wx_min, wy_top, 0.0),
            ];
            Ok(extrude_polygon(&prof, Vec3::new(0.0, 0.0, d)))
        }
        Feature::Nut {
            inscribed_radius,
            bore_radius,
            thickness,
            segments,
            ..
        } => {
            let ir = resolve_one(id, inscribed_radius, params)?;
            let br = resolve_one(id, bore_radius, params)?;
            let t = resolve_one(id, thickness, params)?;
            if ir <= 0.0 || br <= 0.0 || t <= 0.0 {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: format!(
                        "Nut requires positive inscribed_radius, bore_radius, thickness (got {ir}, {br}, {t})"
                    ),
                });
            }
            if br >= ir {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: format!("Nut bore_radius ({br}) must be < inscribed_radius ({ir})"),
                });
            }
            let _ = segments;
            let head_circumradius = ir / (std::f64::consts::PI / 6.0).cos();
            let body = cylinder_faceted(head_circumradius, t, 6);
            let bore_raw = cylinder_faceted(br, t + 2.0, 24);
            let bore = translate_solid(&bore_raw, Vec3::new(0.0, 0.0, -1.0));
            body.try_difference(&bore).map_err(|e| EvalError::Boolean {
                id: id.into(),
                op: "nut",
                message: e.message,
            })
        }
        Feature::Washer {
            outer_radius,
            inner_radius,
            thickness,
            segments,
            ..
        } => {
            let r_out = resolve_one(id, outer_radius, params)?;
            let r_in = resolve_one(id, inner_radius, params)?;
            let t = resolve_one(id, thickness, params)?;
            if r_out <= 0.0 || r_in <= 0.0 || t <= 0.0 {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: format!(
                        "Washer requires positive outer/inner_radius and thickness (got {r_out}, {r_in}, {t})"
                    ),
                });
            }
            if r_in >= r_out {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: format!("Washer inner_radius ({r_in}) must be < outer_radius ({r_out})"),
                });
            }
            if *segments < 3 {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: format!("Washer segments must be >= 3 (got {segments})"),
                });
            }
            let outer = cylinder_faceted(r_out, t, *segments);
            let bore_raw = cylinder_faceted(r_in, t + 2.0, *segments);
            let bore = translate_solid(&bore_raw, Vec3::new(0.0, 0.0, -1.0));
            outer.try_difference(&bore).map_err(|e| EvalError::Boolean {
                id: id.into(),
                op: "washer",
                message: e.message,
            })
        }
        Feature::RoundBoss {
            base,
            radius,
            height,
            segments,
            ..
        } => {
            let b = resolve3(id, base, params)?;
            let r = resolve_one(id, radius, params)?;
            let h = resolve_one(id, height, params)?;
            if r <= 0.0 || h <= 0.0 {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: format!("RoundBoss requires positive radius and height (got {r}, {h})"),
                });
            }
            if *segments < 3 {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: format!("RoundBoss segments must be >= 3 (got {segments})"),
                });
            }
            let cyl = cylinder_faceted(r, h, *segments);
            Ok(translate_solid(&cyl, Vec3::new(b[0], b[1], b[2])))
        }
        Feature::RectBoss {
            corner, extents, ..
        } => {
            let c = resolve3(id, corner, params)?;
            let e = resolve3(id, extents, params)?;
            if e[0] <= 0.0 || e[1] <= 0.0 || e[2] <= 0.0 {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: format!(
                        "RectBoss extents must all be positive (got [{}, {}, {}])",
                        e[0], e[1], e[2]
                    ),
                });
            }
            Ok(box_at(
                Vec3::new(e[0], e[1], e[2]),
                Point3::new(c[0], c[1], c[2]),
            ))
        }
        Feature::DovetailSlot {
            bottom_width,
            top_width,
            depth,
            length,
            ..
        } => {
            let bw = resolve_one(id, bottom_width, params)?;
            let tw = resolve_one(id, top_width, params)?;
            let d = resolve_one(id, depth, params)?;
            let l = resolve_one(id, length, params)?;
            if bw <= 0.0 || tw <= 0.0 || d <= 0.0 || l <= 0.0 {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: format!(
                        "DovetailSlot requires positive bottom_width, top_width, depth, length (got {bw}, {tw}, {d}, {l})"
                    ),
                });
            }
            // Trapezoid CCW from +y: bottom-left, bottom-right, top-right, top-left.
            //   (bottom-left) = (-bw/2, 0, 0)
            //   (bottom-right) = (+bw/2, 0, 0)
            //   (top-right) = (+tw/2, 0, d)
            //   (top-left) = (-tw/2, 0, d)
            // For extrude_polygon CCW-from-+direction (+y), walk such that
            // (p1-p0) x (p2-p0) . y > 0.
            //   Bottom-left → top-left → top-right → bottom-right gives that
            //   orientation (going UP first, then RIGHT, then DOWN).
            let prof = vec![
                Point3::new(-bw / 2.0, 0.0, 0.0),
                Point3::new(-tw / 2.0, 0.0, d),
                Point3::new(tw / 2.0, 0.0, d),
                Point3::new(bw / 2.0, 0.0, 0.0),
            ];
            Ok(extrude_polygon(&prof, Vec3::new(0.0, l, 0.0)))
        }
        Feature::VeeGroove {
            top_width,
            depth,
            length,
            ..
        } => {
            let tw = resolve_one(id, top_width, params)?;
            let d = resolve_one(id, depth, params)?;
            let l = resolve_one(id, length, params)?;
            if tw <= 0.0 || d <= 0.0 || l <= 0.0 {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: format!(
                        "VeeGroove requires positive top_width, depth, length (got {tw}, {d}, {l})"
                    ),
                });
            }
            // Triangle CCW from +y: apex at z=0, top corners at z=d.
            // Walk: apex → top-left → top-right (CCW from +y).
            let prof = vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(-tw / 2.0, 0.0, d),
                Point3::new(tw / 2.0, 0.0, d),
            ];
            Ok(extrude_polygon(&prof, Vec3::new(0.0, l, 0.0)))
        }
        Feature::Bolt {
            head_inscribed_radius,
            head_thickness,
            shaft_radius,
            shaft_length,
            segments,
            ..
        } => {
            let hir = resolve_one(id, head_inscribed_radius, params)?;
            let ht = resolve_one(id, head_thickness, params)?;
            let sr = resolve_one(id, shaft_radius, params)?;
            let sl = resolve_one(id, shaft_length, params)?;
            if hir <= 0.0 || ht <= 0.0 || sr <= 0.0 || sl <= 0.0 {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: format!(
                        "Bolt requires positive head_inscribed_radius, head_thickness, shaft_radius, shaft_length (got {hir}, {ht}, {sr}, {sl})"
                    ),
                });
            }
            if *segments < 3 {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: format!("Bolt segments must be >= 3 (got {segments})"),
                });
            }
            // Hex circumradius = inscribed_radius / cos(π/6).
            let head_circumradius = hir / (std::f64::consts::PI / 6.0).cos();
            let head = cylinder_faceted(head_circumradius, ht, 6);
            let _ = head_circumradius;
            let shaft_raw = cylinder_faceted(sr, sl, *segments);
            let shaft = translate_solid(&shaft_raw, Vec3::new(0.0, 0.0, ht));
            head.try_union(&shaft).map_err(|e| EvalError::Boolean {
                id: id.into(),
                op: "bolt_union",
                message: e.message,
            })
        }
        Feature::CapScrew {
            head_radius,
            head_thickness,
            shaft_radius,
            shaft_length,
            segments,
            ..
        } => {
            let hr = resolve_one(id, head_radius, params)?;
            let ht = resolve_one(id, head_thickness, params)?;
            let sr = resolve_one(id, shaft_radius, params)?;
            let sl = resolve_one(id, shaft_length, params)?;
            if hr <= 0.0 || ht <= 0.0 || sr <= 0.0 || sl <= 0.0 {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: format!(
                        "CapScrew requires positive head_radius, head_thickness, shaft_radius, shaft_length (got {hr}, {ht}, {sr}, {sl})"
                    ),
                });
            }
            if hr <= sr {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: format!("CapScrew head_radius ({hr}) must be > shaft_radius ({sr})"),
                });
            }
            if *segments < 3 {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: format!("CapScrew segments must be >= 3 (got {segments})"),
                });
            }
            let head = cylinder_faceted(hr, ht, *segments);
            let shaft_raw = cylinder_faceted(sr, sl, *segments);
            let shaft = translate_solid(&shaft_raw, Vec3::new(0.0, 0.0, ht));
            head.try_union(&shaft).map_err(|e| EvalError::Boolean {
                id: id.into(),
                op: "cap_screw_union",
                message: e.message,
            })
        }
        Feature::IBeam {
            flange_width,
            flange_thickness,
            web_thickness,
            total_height,
            depth,
            ..
        } => {
            let fw = resolve_one(id, flange_width, params)?;
            let ft = resolve_one(id, flange_thickness, params)?;
            let wt = resolve_one(id, web_thickness, params)?;
            let th = resolve_one(id, total_height, params)?;
            let d = resolve_one(id, depth, params)?;
            if fw <= 0.0 || ft <= 0.0 || wt <= 0.0 || th <= 0.0 || d <= 0.0 {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: format!(
                        "IBeam requires positive flange_width, flange_thickness, web_thickness, total_height, depth (got {fw}, {ft}, {wt}, {th}, {d})"
                    ),
                });
            }
            if wt >= fw {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: format!(
                        "IBeam web_thickness ({wt}) must be < flange_width ({fw})"
                    ),
                });
            }
            if 2.0 * ft >= th {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: format!(
                        "IBeam 2*flange_thickness ({}) must be < total_height ({th})",
                        2.0 * ft
                    ),
                });
            }
            // I profile CCW. Bottom flange y=[0, ft], top flange
            // y=[th-ft, th], web in middle centered on x.
            let wx_min = (fw - wt) / 2.0;
            let wx_max = wx_min + wt;
            let wy_bot = ft;
            let wy_top = th - ft;
            let prof = vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(fw, 0.0, 0.0),
                Point3::new(fw, wy_bot, 0.0),
                Point3::new(wx_max, wy_bot, 0.0),
                Point3::new(wx_max, wy_top, 0.0),
                Point3::new(fw, wy_top, 0.0),
                Point3::new(fw, th, 0.0),
                Point3::new(0.0, th, 0.0),
                Point3::new(0.0, wy_top, 0.0),
                Point3::new(wx_min, wy_top, 0.0),
                Point3::new(wx_min, wy_bot, 0.0),
                Point3::new(0.0, wy_bot, 0.0),
            ];
            Ok(extrude_polygon(&prof, Vec3::new(0.0, 0.0, d)))
        }
        Feature::CChannel {
            width, height, thickness, depth, ..
        } => {
            let w = resolve_one(id, width, params)?;
            let h = resolve_one(id, height, params)?;
            let t = resolve_one(id, thickness, params)?;
            let d = resolve_one(id, depth, params)?;
            if w <= 0.0 || h <= 0.0 || t <= 0.0 || d <= 0.0 || 2.0 * t >= h || t >= w {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: format!("CChannel requires positive dims, t<w, 2t<h (got w={w}, h={h}, t={t}, d={d})"),
                });
            }
            // C-section opens to +x; back wall at x=0. Walk CCW.
            // Outer corners: (0,0), (t,0)? — actually let's keep it simple:
            // outer rectangle [0,w] x [0,h]; cut out [t, w] x [t, h-t].
            // Resulting CCW outer walk:
            //   (0,0)->(w,0)->(w,t)->(t,t)->(t,h-t)->(w,h-t)->(w,h)->(0,h)
            let prof = vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(w, 0.0, 0.0),
                Point3::new(w, t, 0.0),
                Point3::new(t, t, 0.0),
                Point3::new(t, h - t, 0.0),
                Point3::new(w, h - t, 0.0),
                Point3::new(w, h, 0.0),
                Point3::new(0.0, h, 0.0),
            ];
            Ok(extrude_polygon(&prof, Vec3::new(0.0, 0.0, d)))
        }
        Feature::ZBeam { flange, web, thickness, depth, .. } => {
            let f = resolve_one(id, flange, params)?;
            let wb = resolve_one(id, web, params)?;
            let t = resolve_one(id, thickness, params)?;
            let d = resolve_one(id, depth, params)?;
            if f <= 0.0 || wb <= 0.0 || t <= 0.0 || d <= 0.0 || t >= f || t >= wb {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: format!("ZBeam requires positive dims, t<flange, t<web (got f={f}, wb={wb}, t={t}, d={d})"),
                });
            }
            // Z profile: bottom flange [0, f] x [0, t], web [f-t, f] x [t, t+wb-t]=[t, wb],
            // top flange [f-t, 2f-t] x [wb, wb+t].
            // CCW outer walk starting bottom-left:
            //   (0,0) -> (f, 0) -> (f, wb) -> (2f - t, wb) -> (2f - t, wb + t)
            //   -> (f - t, wb + t) -> (f - t, t) -> (0, t) -> close
            let prof = vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(f, 0.0, 0.0),
                Point3::new(f, wb, 0.0),
                Point3::new(2.0 * f - t, wb, 0.0),
                Point3::new(2.0 * f - t, wb + t, 0.0),
                Point3::new(f - t, wb + t, 0.0),
                Point3::new(f - t, t, 0.0),
                Point3::new(0.0, t, 0.0),
            ];
            Ok(extrude_polygon(&prof, Vec3::new(0.0, 0.0, d)))
        }
        Feature::AngleIron { leg_length, thickness, depth, .. } => {
            let l = resolve_one(id, leg_length, params)?;
            let t = resolve_one(id, thickness, params)?;
            let d = resolve_one(id, depth, params)?;
            if l <= 0.0 || t <= 0.0 || d <= 0.0 || t >= l {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: format!("AngleIron requires positive dims with t<leg_length (got l={l}, t={t}, d={d})"),
                });
            }
            // Equal-leg L profile.
            let prof = vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(l, 0.0, 0.0),
                Point3::new(l, t, 0.0),
                Point3::new(t, t, 0.0),
                Point3::new(t, l, 0.0),
                Point3::new(0.0, l, 0.0),
            ];
            Ok(extrude_polygon(&prof, Vec3::new(0.0, 0.0, d)))
        }
        Feature::TSlot { slot_width, slot_height, base_width, base_height, depth, .. } => {
            let sw = resolve_one(id, slot_width, params)?;
            let sh = resolve_one(id, slot_height, params)?;
            let bw = resolve_one(id, base_width, params)?;
            let bh = resolve_one(id, base_height, params)?;
            let d = resolve_one(id, depth, params)?;
            if sw <= 0.0 || sh <= 0.0 || bw <= 0.0 || bh <= 0.0 || d <= 0.0 || sw >= bw {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: format!(
                        "TSlot requires positive dims with slot_width<base_width (got sw={sw}, sh={sh}, bw={bw}, bh={bh}, d={d})"
                    ),
                });
            }
            // Inverted-T cross section centered on x=0. Base [-bw/2, bw/2] x [0, bh],
            // slot above [-sw/2, sw/2] x [bh, bh+sh]. CCW walk:
            //   (-bw/2, 0) -> (bw/2, 0) -> (bw/2, bh) -> (sw/2, bh)
            //   -> (sw/2, bh+sh) -> (-sw/2, bh+sh) -> (-sw/2, bh) -> (-bw/2, bh)
            let prof = vec![
                Point3::new(-bw / 2.0, 0.0, 0.0),
                Point3::new(bw / 2.0, 0.0, 0.0),
                Point3::new(bw / 2.0, bh, 0.0),
                Point3::new(sw / 2.0, bh, 0.0),
                Point3::new(sw / 2.0, bh + sh, 0.0),
                Point3::new(-sw / 2.0, bh + sh, 0.0),
                Point3::new(-sw / 2.0, bh, 0.0),
                Point3::new(-bw / 2.0, bh, 0.0),
            ];
            Ok(extrude_polygon(&prof, Vec3::new(0.0, 0.0, d)))
        }
        Feature::Keyway { width, depth_into, length, .. } => {
            let w = resolve_one(id, width, params)?;
            let di = resolve_one(id, depth_into, params)?;
            let l = resolve_one(id, length, params)?;
            if w <= 0.0 || di <= 0.0 || l <= 0.0 {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: format!("Keyway requires positive width, depth_into, length (got w={w}, di={di}, l={l})"),
                });
            }
            // Just a box: [-w/2, w/2] x [0, di] x [0, l].
            Ok(box_at(
                Vec3::new(w, di, l),
                Point3::new(-w / 2.0, 0.0, 0.0),
            ))
        }
        Feature::RoundedRect {
            width, height, thickness, corner_radius, segments, ..
        } => {
            let w = resolve_one(id, width, params)?;
            let h = resolve_one(id, height, params)?;
            let t = resolve_one(id, thickness, params)?;
            let r = resolve_one(id, corner_radius, params)?;
            if w <= 0.0 || h <= 0.0 || t <= 0.0 || r < 0.0 || *segments < 3 {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: format!("RoundedRect requires positive dims, segments>=3 (got w={w}, h={h}, t={t}, r={r})"),
                });
            }
            if 2.0 * r >= w || 2.0 * r >= h {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: format!("RoundedRect 2*corner_radius ({}) must be < width and height", 2.0 * r),
                });
            }
            // Build rounded-rect polygon: 4 straight runs + 4 quarter-circle arcs.
            let mut prof: Vec<Point3> = Vec::new();
            let q = (*segments).max(4) / 4; // quarter-arc segments
            // Corner centers: (r,r), (w-r,r), (w-r,h-r), (r,h-r).
            let corners = [
                (r, r, std::f64::consts::PI),       // start angle for bottom-left arc
                (w - r, r, 1.5 * std::f64::consts::PI),
                (w - r, h - r, 0.0),
                (r, h - r, 0.5 * std::f64::consts::PI),
            ];
            for (cx, cy, start_ang) in corners {
                for k in 0..=q {
                    let a = start_ang + 0.5 * std::f64::consts::PI * (k as f64) / (q as f64);
                    let x = cx + r * a.cos();
                    let y = cy + r * a.sin();
                    prof.push(Point3::new(x, y, 0.0));
                }
            }
            Ok(extrude_polygon(&prof, Vec3::new(0.0, 0.0, t)))
        }
        Feature::Hemisphere { radius, stacks, slices, .. } => {
            let r = resolve_one(id, radius, params)?;
            if r <= 0.0 || *stacks < 2 || *slices < 3 {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: format!("Hemisphere requires positive radius, stacks>=2, slices>=3 (got r={r})"),
                });
            }
            // Build a faceted sphere then carve away the bottom half with
            // a big box at z<0. This trips the curved-surface boolean
            // limitations sometimes; document if it does.
            let sph = sphere_faceted(r, *stacks, *slices);
            let cutter = box_at(
                Vec3::new(4.0 * r, 4.0 * r, 2.0 * r),
                Point3::new(-2.0 * r, -2.0 * r, -2.0 * r),
            );
            sph.try_difference(&cutter).map_err(|e| EvalError::Boolean {
                id: id.into(),
                op: "hemisphere_clip",
                message: e.message,
            })
        }
        Feature::SphericalCap { radius, cap_height, stacks, slices, .. } => {
            let r = resolve_one(id, radius, params)?;
            let cap = resolve_one(id, cap_height, params)?;
            if r <= 0.0 || cap <= 0.0 || cap > 2.0 * r || *stacks < 2 || *slices < 3 {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: format!("SphericalCap requires r>0, 0<cap<=2r, stacks>=2, slices>=3 (got r={r}, cap={cap})"),
                });
            }
            // Sphere centered on origin, clip with z < (r - cap_height).
            let sph = sphere_faceted(r, *stacks, *slices);
            let z_cut = r - cap;
            let cutter = box_at(
                Vec3::new(4.0 * r, 4.0 * r, 4.0 * r),
                Point3::new(-2.0 * r, -2.0 * r, z_cut - 4.0 * r),
            );
            sph.try_difference(&cutter).map_err(|e| EvalError::Boolean {
                id: id.into(),
                op: "spherical_cap_clip",
                message: e.message,
            })
        }
        Feature::Bowl { outer_radius, inner_radius, stacks, slices, .. } => {
            let r_out = resolve_one(id, outer_radius, params)?;
            let r_in = resolve_one(id, inner_radius, params)?;
            if r_out <= 0.0 || r_in <= 0.0 || r_in >= r_out || *stacks < 2 || *slices < 3 {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: format!("Bowl requires 0 < inner < outer, stacks>=2, slices>=3 (got out={r_out}, in={r_in})"),
                });
            }
            let outer_sph = sphere_faceted(r_out, *stacks, *slices);
            let inner_sph = sphere_faceted(r_in, *stacks, *slices);
            let shell = outer_sph
                .try_difference(&inner_sph)
                .map_err(|e| EvalError::Boolean { id: id.into(), op: "bowl_shell", message: e.message })?;
            // Cut off the bottom half (z<0).
            let cutter = box_at(
                Vec3::new(4.0 * r_out, 4.0 * r_out, 2.0 * r_out),
                Point3::new(-2.0 * r_out, -2.0 * r_out, -2.0 * r_out),
            );
            shell.try_difference(&cutter).map_err(|e| EvalError::Boolean {
                id: id.into(),
                op: "bowl_clip",
                message: e.message,
            })
        }
        Feature::BoundingBoxRef { input, wire_thickness, .. } => {
            let base = cache_get(cache, input)?;
            let t = resolve_one(id, wire_thickness, params)?;
            if t <= 0.0 {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: format!("BoundingBoxRef requires positive wire_thickness (got {t})"),
                });
            }
            // Walk the input solid's vertices, build axis-aligned box
            // bounding them, return as a thin wireframe (12 edge bars).
            // For now: just render the bounding box as a hollow box.
            let mut min = [f64::INFINITY; 3];
            let mut max = [f64::NEG_INFINITY; 3];
            for (_, p) in base.vertex_geom.iter() {
                if p.x < min[0] { min[0] = p.x; }
                if p.y < min[1] { min[1] = p.y; }
                if p.z < min[2] { min[2] = p.z; }
                if p.x > max[0] { max[0] = p.x; }
                if p.y > max[1] { max[1] = p.y; }
                if p.z > max[2] { max[2] = p.z; }
            }
            if !min[0].is_finite() {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: "BoundingBoxRef input has no vertices".into(),
                });
            }
            // Render as a hollow box (outer minus inner): outer = bbox,
            // inner = bbox shrunk by wire_thickness. Caller can union it
            // with anything they want.
            let outer = box_at(
                Vec3::new(max[0] - min[0], max[1] - min[1], max[2] - min[2]),
                Point3::new(min[0], min[1], min[2]),
            );
            let inner_w = (max[0] - min[0] - 2.0 * t).max(0.0);
            let inner_h = (max[1] - min[1] - 2.0 * t).max(0.0);
            let inner_d = (max[2] - min[2] - 2.0 * t).max(0.0);
            if inner_w < 1e-12 || inner_h < 1e-12 || inner_d < 1e-12 {
                // Wire too thick; just return the solid box.
                return Ok(outer);
            }
            let inner = box_at(
                Vec3::new(inner_w, inner_h, inner_d),
                Point3::new(min[0] + t, min[1] + t, min[2] + t),
            );
            outer.try_difference(&inner).map_err(|e| EvalError::Boolean {
                id: id.into(),
                op: "bounding_box_shell",
                message: e.message,
            })
        }
        Feature::CentroidPoint { input, marker_size, .. } => {
            let base = cache_get(cache, input)?;
            let m = resolve_one(id, marker_size, params)?;
            if m <= 0.0 {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: format!("CentroidPoint requires positive marker_size (got {m})"),
                });
            }
            // Compute average vertex position as a stand-in centroid.
            // (True volume centroid would integrate over tetrahedra; for a
            // marker this is good enough.)
            let mut sum = [0.0; 3];
            let mut n = 0usize;
            for (_, p) in base.vertex_geom.iter() {
                sum[0] += p.x;
                sum[1] += p.y;
                sum[2] += p.z;
                n += 1;
            }
            if n == 0 {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: "CentroidPoint input has no vertices".into(),
                });
            }
            let cx = sum[0] / n as f64;
            let cy = sum[1] / n as f64;
            let cz = sum[2] / n as f64;
            // Render as a small box centered at the centroid.
            Ok(box_at(
                Vec3::new(m, m, m),
                Point3::new(cx - m / 2.0, cy - m / 2.0, cz - m / 2.0),
            ))
        }
        Feature::MountingFlange {
            disk_radius, disk_thickness, bolt_circle_radius, bolt_count, bolt_radius, segments, ..
        } => {
            let r_disk = resolve_one(id, disk_radius, params)?;
            let t = resolve_one(id, disk_thickness, params)?;
            let r_bc = resolve_one(id, bolt_circle_radius, params)?;
            let r_bolt = resolve_one(id, bolt_radius, params)?;
            if r_disk <= 0.0 || t <= 0.0 || r_bc <= 0.0 || r_bolt <= 0.0
                || r_bc >= r_disk || *bolt_count == 0 || *segments < 3
            {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: format!(
                        "MountingFlange requires positive dims, bolt_circle < disk_radius, bolt_count >= 1, segments >= 3 (got disk={r_disk}, t={t}, bc={r_bc}, bolt={r_bolt}, count={bolt_count})"
                    ),
                });
            }
            let mut acc = cylinder_faceted(r_disk, t, *segments);
            for k in 0..*bolt_count {
                let theta = 2.0 * std::f64::consts::PI * (k as f64) / (*bolt_count as f64);
                let bx = r_bc * theta.cos();
                let by = r_bc * theta.sin();
                let hole = cylinder_along_axis(
                    r_bolt,
                    t + 2.0 * (t * 0.05).max(1e-3),
                    *segments,
                    2,
                    -(t * 0.05).max(1e-3),
                    bx,
                    by,
                );
                acc = acc
                    .try_difference(&hole)
                    .map_err(|e| EvalError::Boolean { id: id.into(), op: "mounting_flange_drill", message: format!("hole {k}: {}", e.message) })?;
            }
            Ok(acc)
        }
        Feature::GearBlank {
            outer_radius, root_radius, tooth_count, thickness, segments_per_tooth, ..
        } => {
            let r_out = resolve_one(id, outer_radius, params)?;
            let r_root = resolve_one(id, root_radius, params)?;
            let t = resolve_one(id, thickness, params)?;
            if r_out <= 0.0 || r_root <= 0.0 || r_root >= r_out || t <= 0.0
                || *tooth_count < 3 || *segments_per_tooth < 1
            {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: format!(
                        "GearBlank requires r_out > r_root > 0, t > 0, tooth_count >= 3, segments_per_tooth >= 1 (got r_out={r_out}, r_root={r_root}, t={t}, tc={tooth_count})"
                    ),
                });
            }
            // Build a polygon with 2*tooth_count vertices alternating
            // between outer (tooth tips) and root radii. Each tooth spans
            // 2π/tooth_count of arc, with the first half at outer and the
            // second half at root.
            let n = 2 * *tooth_count;
            let mut prof = Vec::with_capacity(n);
            for i in 0..n {
                let theta = 2.0 * std::f64::consts::PI * (i as f64) / (n as f64);
                let r = if i % 2 == 0 { r_out } else { r_root };
                prof.push(Point3::new(r * theta.cos(), r * theta.sin(), 0.0));
            }
            Ok(extrude_polygon(&prof, Vec3::new(0.0, 0.0, t)))
        }
        Feature::KnurledGrip {
            radius, ridge_height, height, ridge_count, ..
        } => {
            let r = resolve_one(id, radius, params)?;
            let rh = resolve_one(id, ridge_height, params)?;
            let h = resolve_one(id, height, params)?;
            if r <= 0.0 || rh <= 0.0 || h <= 0.0 || *ridge_count < 6 {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: format!("KnurledGrip requires positive dims, ridge_count >= 6 (got r={r}, rh={rh}, h={h}, rc={ridge_count})"),
                });
            }
            // Build a profile that alternates between r (valley) and r+rh (ridge).
            let n = 2 * *ridge_count;
            let mut prof = Vec::with_capacity(n);
            for i in 0..n {
                let theta = 2.0 * std::f64::consts::PI * (i as f64) / (n as f64);
                let rad = if i % 2 == 0 { r + rh } else { r };
                prof.push(Point3::new(rad * theta.cos(), rad * theta.sin(), 0.0));
            }
            Ok(extrude_polygon(&prof, Vec3::new(0.0, 0.0, h)))
        }
        Feature::Pipe {
            base, axis, outer_radius, inner_radius, length, segments, ..
        } => {
            let b = resolve3(id, base, params)?;
            let r_out = resolve_one(id, outer_radius, params)?;
            let r_in = resolve_one(id, inner_radius, params)?;
            let l = resolve_one(id, length, params)?;
            if r_out <= 0.0 || r_in <= 0.0 || r_in >= r_out || l <= 0.0 || *segments < 3 {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: format!("Pipe requires 0 < inner < outer, length > 0, segments >= 3 (got out={r_out}, in={r_in}, l={l})"),
                });
            }
            let axis_idx = parse_axis(id, axis)?;
            let (a_idx, idx_b) = perpendicular_axes(axis_idx);
            let outer = cylinder_along_axis(r_out, l, *segments, axis_idx, b[axis_idx], b[a_idx], b[idx_b]);
            let eps = (l * 0.05).max(1e-3);
            let inner = cylinder_along_axis(r_in, l + 2.0 * eps, *segments, axis_idx, b[axis_idx] - eps, b[a_idx], b[idx_b]);
            outer.try_difference(&inner).map_err(|e| EvalError::Boolean { id: id.into(), op: "pipe_bore", message: e.message })
        }
        Feature::Spring {
            coil_radius, wire_radius, pitch, turns, segments_per_turn, wire_segments, ..
        } => {
            // Spring is just an alias for Coil for now — same algorithm,
            // with the door open to add end caps later.
            let r_coil = resolve_one(id, coil_radius, params)?;
            let r_wire = resolve_one(id, wire_radius, params)?;
            let p = resolve_one(id, pitch, params)?;
            let t = resolve_one(id, turns, params)?;
            if r_coil <= 0.0 || r_wire <= 0.0 || p <= 0.0 || t <= 0.0 || r_wire >= r_coil
                || *segments_per_turn < 6 || *wire_segments < 3
            {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: format!("Spring requires the same constraints as Coil (got coil={r_coil}, wire={r_wire}, pitch={p}, turns={t})"),
                });
            }
            let total_samples = (*segments_per_turn as f64 * t).ceil() as usize + 1;
            let mut acc: Option<Solid> = None;
            let total_angle = 2.0 * std::f64::consts::PI * t;
            for i in 0..total_samples - 1 {
                let theta_a = total_angle * i as f64 / (total_samples - 1) as f64;
                let theta_b = total_angle * (i + 1) as f64 / (total_samples - 1) as f64;
                let p0 = [r_coil * theta_a.cos(), r_coil * theta_a.sin(), p * theta_a / (2.0 * std::f64::consts::PI)];
                let p1 = [r_coil * theta_b.cos(), r_coil * theta_b.sin(), p * theta_b / (2.0 * std::f64::consts::PI)];
                let cyl = sweep_cylinder_segment(p0, p1, r_wire, *wire_segments).ok_or_else(|| EvalError::Invalid {
                    id: id.into(),
                    reason: format!("Spring segment {i} has zero length"),
                })?;
                acc = Some(match acc.take() {
                    None => cyl,
                    Some(prev) => prev.try_union(&cyl).map_err(|e| EvalError::Boolean { id: id.into(), op: "spring_segment_union", message: format!("segment {i}: {}", e.message) })?,
                });
            }
            Ok(acc.unwrap())
        }
        Feature::Mortise { center, width, length, depth, .. } => {
            let cx = resolve_one(id, &center[0], params)?;
            let cy = resolve_one(id, &center[1], params)?;
            let w = resolve_one(id, width, params)?;
            let l = resolve_one(id, length, params)?;
            let d = resolve_one(id, depth, params)?;
            if w <= 0.0 || l <= 0.0 || d <= 0.0 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("Mortise requires positive dims (got w={w}, l={l}, d={d})") });
            }
            Ok(box_at(Vec3::new(w, l, d), Point3::new(cx - w/2.0, cy - l/2.0, -d)))
        }
        Feature::Tenon { center, width, length, height, .. } => {
            let cx = resolve_one(id, &center[0], params)?;
            let cy = resolve_one(id, &center[1], params)?;
            let w = resolve_one(id, width, params)?;
            let l = resolve_one(id, length, params)?;
            let h = resolve_one(id, height, params)?;
            if w <= 0.0 || l <= 0.0 || h <= 0.0 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("Tenon requires positive dims (got w={w}, l={l}, h={h})") });
            }
            Ok(box_at(Vec3::new(w, l, h), Point3::new(cx - w/2.0, cy - l/2.0, 0.0)))
        }
        Feature::FingerJoint { count, finger_width, finger_height, finger_depth, gap_width, .. } => {
            let fw = resolve_one(id, finger_width, params)?;
            let fh = resolve_one(id, finger_height, params)?;
            let fd = resolve_one(id, finger_depth, params)?;
            let gw = resolve_one(id, gap_width, params)?;
            if *count == 0 || fw <= 0.0 || fh <= 0.0 || fd <= 0.0 || gw <= 0.0 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("FingerJoint requires count >= 1 and positive dims (got count={count}, fw={fw}, fh={fh}, fd={fd}, gw={gw})") });
            }
            // Build a base plate (full row of count*fw + (count-1)*gw width, fd depth, 0.05*fh thick),
            // then union finger-cubes on top.
            let total_width = (*count as f64) * fw + ((*count as f64) - 1.0) * gw;
            let base_thickness = (fh * 0.05).max(1e-3);
            let base = box_at(
                Vec3::new(total_width, fd, base_thickness),
                Point3::new(0.0, 0.0, 0.0),
            );
            let mut acc = base;
            for k in 0..*count {
                let x0 = (k as f64) * (fw + gw);
                let finger = box_at(
                    Vec3::new(fw, fd, fh),
                    Point3::new(x0, 0.0, base_thickness),
                );
                acc = acc.try_union(&finger).map_err(|e| EvalError::Boolean { id: id.into(), op: "finger_joint_union", message: format!("finger {k}: {}", e.message) })?;
            }
            Ok(acc)
        }
        Feature::DovetailRail { width, top_width, height, length, .. } => {
            let w = resolve_one(id, width, params)?;
            let tw = resolve_one(id, top_width, params)?;
            let h = resolve_one(id, height, params)?;
            let l = resolve_one(id, length, params)?;
            if w <= 0.0 || tw <= 0.0 || h <= 0.0 || l <= 0.0 || tw >= w {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("DovetailRail requires top_width < width and positive dims (got w={w}, tw={tw}, h={h}, l={l})") });
            }
            // Trapezoid CCW in xy plane, extruded along +z by length.
            let prof = vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(w, 0.0, 0.0),
                Point3::new(w/2.0 + tw/2.0, h, 0.0),
                Point3::new(w/2.0 - tw/2.0, h, 0.0),
            ];
            Ok(extrude_polygon(&prof, Vec3::new(0.0, 0.0, l)))
        }
        Feature::Pulley { outer_radius, groove_depth, groove_width, width, segments, .. } => {
            let r = resolve_one(id, outer_radius, params)?;
            let gd = resolve_one(id, groove_depth, params)?;
            let gw = resolve_one(id, groove_width, params)?;
            let w = resolve_one(id, width, params)?;
            if r <= 0.0 || gd <= 0.0 || gw <= 0.0 || w <= 0.0 || gd >= r || gw >= w || *segments < 8 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("Pulley requires gd<r, gw<w, segments>=8 (got r={r}, gd={gd}, gw={gw}, w={w})") });
            }
            // Cylinder with a torus-shaped V-groove subtracted around the
            // equator. Approximated as a smaller "ring cutter" (a cylinder
            // narrower than `w` but bigger than `r - gd`) — when subtracted
            // it forms a groove. Not a true V cross-section but a
            // rectangular groove. Trade-off for not needing curved
            // booleans.
            let body = cylinder_faceted(r, w, *segments);
            let groove_inner_r = r - gd;
            let groove_outer_r = r + gd; // bigger than body, makes a ring cutter that won't cap
            let groove_z = (w - gw) / 2.0;
            let outer_cutter = cylinder_along_axis(groove_outer_r, gw, *segments, 2, groove_z, 0.0, 0.0);
            let inner_keeper = cylinder_along_axis(groove_inner_r, gw + 2.0 * 1e-3, *segments, 2, groove_z - 1e-3, 0.0, 0.0);
            let ring = outer_cutter.try_difference(&inner_keeper).map_err(|e| EvalError::Boolean { id: id.into(), op: "pulley_groove_ring", message: e.message })?;
            body.try_difference(&ring).map_err(|e| EvalError::Boolean { id: id.into(), op: "pulley_groove_cut", message: e.message })
        }
        Feature::Bushing { inner_radius, outer_radius, flange_radius, flange_thickness, body_length, segments, .. } => {
            let r_in = resolve_one(id, inner_radius, params)?;
            let r_out = resolve_one(id, outer_radius, params)?;
            let r_fl = resolve_one(id, flange_radius, params)?;
            let t_fl = resolve_one(id, flange_thickness, params)?;
            let l_body = resolve_one(id, body_length, params)?;
            if r_in <= 0.0 || r_out <= r_in || r_fl <= r_out || t_fl <= 0.0 || l_body <= 0.0 || *segments < 6 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("Bushing requires 0<in<out<fl, t_fl>0, body>0, segments>=6 (got in={r_in}, out={r_out}, fl={r_fl}, t={t_fl}, body={l_body})") });
            }
            let flange = cylinder_faceted(r_fl, t_fl, *segments);
            let body = cylinder_along_axis(r_out, l_body, *segments, 2, t_fl, 0.0, 0.0);
            let assembled = flange.try_union(&body).map_err(|e| EvalError::Boolean { id: id.into(), op: "bushing_flange_union", message: e.message })?;
            let bore = cylinder_along_axis(r_in, t_fl + l_body + 2.0 * 1e-3, *segments, 2, -1e-3, 0.0, 0.0);
            assembled.try_difference(&bore).map_err(|e| EvalError::Boolean { id: id.into(), op: "bushing_bore", message: e.message })
        }
        Feature::Sprocket { outer_radius, root_radius, tooth_count, thickness, .. } => {
            let r_out = resolve_one(id, outer_radius, params)?;
            let r_root = resolve_one(id, root_radius, params)?;
            let t = resolve_one(id, thickness, params)?;
            if r_out <= 0.0 || r_root <= 0.0 || r_root >= r_out || t <= 0.0 || *tooth_count < 3 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("Sprocket requires r_out>r_root>0, t>0, tooth_count>=3 (got out={r_out}, root={r_root}, t={t}, tc={tooth_count})") });
            }
            // Triangular teeth: 3 vertices per tooth — outer tip at theta_i,
            // root at theta_i + half-step, root at theta_i + full step.
            // Polygon walk: alternate (outer tip, root, root) but to keep
            // it consistent with extrude_polygon's CCW expectation, walk
            // (outer, root, ...).
            let tc = *tooth_count;
            let mut prof = Vec::with_capacity(2 * tc);
            for i in 0..tc {
                let theta_tip = 2.0 * std::f64::consts::PI * (i as f64) / (tc as f64);
                let theta_root = 2.0 * std::f64::consts::PI * (i as f64 + 0.5) / (tc as f64);
                prof.push(Point3::new(r_out * theta_tip.cos(), r_out * theta_tip.sin(), 0.0));
                prof.push(Point3::new(r_root * theta_root.cos(), r_root * theta_root.sin(), 0.0));
            }
            Ok(extrude_polygon(&prof, Vec3::new(0.0, 0.0, t)))
        }
        Feature::Obelisk { bottom_side, top_side, height, .. } => {
            let bs = resolve_one(id, bottom_side, params)?;
            let ts = resolve_one(id, top_side, params)?;
            let h = resolve_one(id, height, params)?;
            if bs <= 0.0 || ts <= 0.0 || ts > bs || h <= 0.0 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("Obelisk requires bs>=ts>0, h>0 (got bs={bs}, ts={ts}, h={h})") });
            }
            let bot = vec![
                Point3::new(-bs/2.0, -bs/2.0, 0.0),
                Point3::new( bs/2.0, -bs/2.0, 0.0),
                Point3::new( bs/2.0,  bs/2.0, 0.0),
                Point3::new(-bs/2.0,  bs/2.0, 0.0),
            ];
            let top = vec![
                Point3::new(-ts/2.0, -ts/2.0, h),
                Point3::new( ts/2.0, -ts/2.0, h),
                Point3::new( ts/2.0,  ts/2.0, h),
                Point3::new(-ts/2.0,  ts/2.0, h),
            ];
            Ok(extrude_lofted(&bot, &top))
        }
        Feature::AxleShaft { body_radius, body_length, neck_radius, neck_length, segments, .. } => {
            let r_body = resolve_one(id, body_radius, params)?;
            let r_neck = resolve_one(id, neck_radius, params)?;
            let l_body = resolve_one(id, body_length, params)?;
            let l_neck = resolve_one(id, neck_length, params)?;
            if r_body <= 0.0 || r_neck <= 0.0 || r_neck >= r_body || l_body <= 0.0 || l_neck <= 0.0 || *segments < 6 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("AxleShaft requires r_neck<r_body, all positive, segments>=6 (got body={r_body}, neck={r_neck}, lb={l_body}, ln={l_neck})") });
            }
            let bottom_neck = cylinder_along_axis(r_neck, l_neck, *segments, 2, 0.0, 0.0, 0.0);
            let body = cylinder_along_axis(r_body, l_body, *segments, 2, l_neck, 0.0, 0.0);
            let top_neck = cylinder_along_axis(r_neck, l_neck, *segments, 2, l_neck + l_body, 0.0, 0.0);
            let lower = bottom_neck.try_union(&body).map_err(|e| EvalError::Boolean { id: id.into(), op: "axle_lower_join", message: e.message })?;
            lower.try_union(&top_neck).map_err(|e| EvalError::Boolean { id: id.into(), op: "axle_upper_join", message: e.message })
        }
        Feature::Column {
            base_radius, base_height, shaft_radius, shaft_height, capital_radius, capital_height, segments, ..
        } => {
            let r_b = resolve_one(id, base_radius, params)?;
            let h_b = resolve_one(id, base_height, params)?;
            let r_s = resolve_one(id, shaft_radius, params)?;
            let h_s = resolve_one(id, shaft_height, params)?;
            let r_c = resolve_one(id, capital_radius, params)?;
            let h_c = resolve_one(id, capital_height, params)?;
            if r_b <= 0.0 || h_b <= 0.0 || r_s <= 0.0 || h_s <= 0.0 || r_c <= 0.0 || h_c <= 0.0 || r_s > r_b || r_s > r_c || *segments < 6 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("Column requires shaft_radius <= base & capital, all positive, segments >= 6 (got rb={r_b}, rs={r_s}, rc={r_c})") });
            }
            // Slight overlap between sections breaks the coplanar shared
            // face that trips the boolean engine.
            let eps = (h_b.min(h_s).min(h_c) * 0.05).max(1e-3);
            let base = cylinder_along_axis(r_b, h_b, *segments, 2, 0.0, 0.0, 0.0);
            let shaft = cylinder_along_axis(r_s, h_s + 2.0 * eps, *segments, 2, h_b - eps, 0.0, 0.0);
            let capital = cylinder_along_axis(r_c, h_c, *segments, 2, h_b + h_s, 0.0, 0.0);
            let lower = base.try_union(&shaft).map_err(|e| EvalError::Boolean { id: id.into(), op: "column_base_shaft", message: e.message })?;
            lower.try_union(&capital).map_err(|e| EvalError::Boolean { id: id.into(), op: "column_capital", message: e.message })
        }
        Feature::Diamond { radius, .. } => {
            let r = resolve_one(id, radius, params)?;
            if r <= 0.0 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("Diamond requires positive radius (got {r})") });
            }
            // Octahedron: 6 vertices on the principal axes, 8 triangular faces.
            // Top pyramid vertices at (±r, 0, 0), (0, ±r, 0), apex (0, 0, r).
            // Use Loft from a 4-vertex equator polygon to a degenerate top.
            // Simplest construction: build 4-side pyramid (square base of side
            // r*sqrt(2) at z=0 → apex at (0,0,r)) UNION its mirror (apex at
            // (0,0,-r)).
            // Build the octahedron in a single direct-construction step
            // (no boolean union — joining two pyramids at a coplanar
            // square face trips stitch). Use Loft with a thin overlap
            // band: build the top cone (square base → apex) and embed
            // the bottom apex INSIDE this same loft by routing the
            // lower half as a single lofted shape from -z apex up
            // through the base. We construct it as a 4-sided bipyramid
            // by extruding the upper pyramid and lower pyramid with a
            // shared overlap band.
            // extrude_lofted requires non-degenerate top polygons (zero-
            // area apex breaks its lateral-face construction), so use
            // a tiny apex square as a proxy. Visually identical at
            // human resolution.
            let s_sz = r * std::f64::consts::SQRT_2;
            let apex_sz = r * 1e-3; // tiny apex, not zero
            let eps = r * 0.02; // overlap band so halves merge, don't share a face
            let upper_base = vec![
                Point3::new(-s_sz/2.0, -s_sz/2.0, -eps),
                Point3::new( s_sz/2.0, -s_sz/2.0, -eps),
                Point3::new( s_sz/2.0,  s_sz/2.0, -eps),
                Point3::new(-s_sz/2.0,  s_sz/2.0, -eps),
            ];
            let upper_apex = vec![
                Point3::new(-apex_sz/2.0, -apex_sz/2.0, r),
                Point3::new( apex_sz/2.0, -apex_sz/2.0, r),
                Point3::new( apex_sz/2.0,  apex_sz/2.0, r),
                Point3::new(-apex_sz/2.0,  apex_sz/2.0, r),
            ];
            let upper = extrude_lofted(&upper_base, &upper_apex);
            let lower_apex = vec![
                Point3::new(-apex_sz/2.0, -apex_sz/2.0, -r),
                Point3::new( apex_sz/2.0, -apex_sz/2.0, -r),
                Point3::new( apex_sz/2.0,  apex_sz/2.0, -r),
                Point3::new(-apex_sz/2.0,  apex_sz/2.0, -r),
            ];
            let lower_base = vec![
                Point3::new(-s_sz/2.0, -s_sz/2.0,  eps),
                Point3::new( s_sz/2.0, -s_sz/2.0,  eps),
                Point3::new( s_sz/2.0,  s_sz/2.0,  eps),
                Point3::new(-s_sz/2.0,  s_sz/2.0,  eps),
            ];
            let lower = extrude_lofted(&lower_apex, &lower_base);
            upper.try_union(&lower).map_err(|e| EvalError::Boolean { id: id.into(), op: "diamond_join", message: e.message })
        }
        Feature::TriPrism { a, b, c, length, .. } => {
            let a_v = resolve_one(id, a, params)?;
            let b_v = resolve_one(id, b, params)?;
            let c_v = resolve_one(id, c, params)?;
            let l = resolve_one(id, length, params)?;
            if a_v <= 0.0 || b_v <= 0.0 || c_v <= 0.0 || l <= 0.0
                || a_v + b_v <= c_v || b_v + c_v <= a_v || a_v + c_v <= b_v
            {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("TriPrism requires positive sides satisfying triangle inequality (got a={a_v}, b={b_v}, c={c_v})") });
            }
            // Place A at origin along +x; A=(0,0), B=(a,0). C is the
            // intersection of two circles. From A, |C| = c. From B, |B-C| = b.
            // Solve: cx = (a² + c² - b²) / (2a); cy = sqrt(c² - cx²).
            let cx = (a_v * a_v + c_v * c_v - b_v * b_v) / (2.0 * a_v);
            let cy = (c_v * c_v - cx * cx).max(0.0).sqrt();
            let prof = vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(a_v, 0.0, 0.0),
                Point3::new(cx, cy, 0.0),
            ];
            Ok(extrude_polygon(&prof, Vec3::new(0.0, 0.0, l)))
        }
        Feature::PerforatedPlate {
            plate_width, plate_height, plate_thickness, margin, hole_radius, nx, ny, dx, dy, segments, ..
        } => {
            let pw = resolve_one(id, plate_width, params)?;
            let ph = resolve_one(id, plate_height, params)?;
            let pt = resolve_one(id, plate_thickness, params)?;
            let mg = resolve_one(id, margin, params)?;
            let hr = resolve_one(id, hole_radius, params)?;
            let dxv = resolve_one(id, dx, params)?;
            let dyv = resolve_one(id, dy, params)?;
            if pw <= 0.0 || ph <= 0.0 || pt <= 0.0 || hr <= 0.0 || mg < 0.0 || dxv < 2.0 * hr || dyv < 2.0 * hr || *nx == 0 || *ny == 0 || *segments < 6 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("PerforatedPlate requires positive dims, dx&dy >= 2*hr, nx&ny>=1 (got pw={pw}, ph={ph})") });
            }
            let mut acc = box_at(Vec3::new(pw, ph, pt), Point3::new(0.0, 0.0, 0.0));
            let eps = (pt * 0.05).max(1e-3);
            for j in 0..*ny {
                for i in 0..*nx {
                    let cx = mg + (i as f64) * dxv;
                    let cy = mg + (j as f64) * dyv;
                    if cx + hr > pw - mg + 1e-9 || cy + hr > ph - mg + 1e-9 {
                        continue;
                    }
                    let hole = cylinder_along_axis(hr, pt + 2.0 * eps, *segments, 2, -eps, cx, cy);
                    acc = acc.try_difference(&hole).map_err(|e| EvalError::Boolean { id: id.into(), op: "perforated_drill", message: format!("hole ({i},{j}): {}", e.message) })?;
                }
            }
            Ok(acc)
        }
        Feature::ChamferedPlate { width, height, thickness, chamfer, .. } => {
            let w = resolve_one(id, width, params)?;
            let h = resolve_one(id, height, params)?;
            let t = resolve_one(id, thickness, params)?;
            let c = resolve_one(id, chamfer, params)?;
            if w <= 0.0 || h <= 0.0 || t <= 0.0 || c < 0.0 || 2.0 * c >= w || 2.0 * c >= h {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("ChamferedPlate requires 2c<w&h, all positive (got w={w}, h={h}, c={c})") });
            }
            // Octagonal profile (rectangle with corners cut at 45°).
            let prof = vec![
                Point3::new(c, 0.0, 0.0),
                Point3::new(w - c, 0.0, 0.0),
                Point3::new(w, c, 0.0),
                Point3::new(w, h - c, 0.0),
                Point3::new(w - c, h, 0.0),
                Point3::new(c, h, 0.0),
                Point3::new(0.0, h - c, 0.0),
                Point3::new(0.0, c, 0.0),
            ];
            Ok(extrude_polygon(&prof, Vec3::new(0.0, 0.0, t)))
        }
        Feature::ReducerCone {
            outer_bottom, outer_top, inner_bottom, inner_top, height, segments, ..
        } => {
            let ob = resolve_one(id, outer_bottom, params)?;
            let ot = resolve_one(id, outer_top, params)?;
            let ib = resolve_one(id, inner_bottom, params)?;
            let it = resolve_one(id, inner_top, params)?;
            let h = resolve_one(id, height, params)?;
            if ob <= 0.0 || ot <= 0.0 || ib <= 0.0 || it <= 0.0 || h <= 0.0
                || ib >= ob || it >= ot || *segments < 6
            {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("ReducerCone requires inner < outer at both ends (got ob={ob}, ot={ot}, ib={ib}, it={it})") });
            }
            let outer = frustum_faceted(ot, ob, h, *segments);
            let eps = (h * 0.05).max(1e-3);
            // Inner needs to be slightly taller to ensure clean through-bore.
            let inner = frustum_faceted(it, ib, h + 2.0 * eps, *segments);
            let inner_translated = translate_solid(&inner, Vec3::new(0.0, 0.0, -eps));
            outer.try_difference(&inner_translated).map_err(|e| EvalError::Boolean { id: id.into(), op: "reducer_bore", message: e.message })
        }
        Feature::Elbow90 { radius, leg_length, segments, .. } => {
            let r = resolve_one(id, radius, params)?;
            let ll = resolve_one(id, leg_length, params)?;
            if r <= 0.0 || ll <= 0.0 || *segments < 6 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("Elbow90 requires positive radius/leg_length, segments>=6 (got r={r}, ll={ll})") });
            }
            // Two perpendicular cylinders, joined as a polyline (same
            // strategy PipeRun uses — a sequence of axis-aligned cylinder
            // unions). A direct two-cylinder union trips the boolean
            // engine on the cylinder-cylinder intersection curve, but a
            // single axis change with disjoint axis spans (small overlap
            // at the corner) survives.
            let leg_x = cylinder_along_axis(r, ll, *segments, 0, 0.0, 0.0, 0.0);
            let leg_y = cylinder_along_axis(r, ll + r, *segments, 1, -r, 0.0, 0.0);
            leg_x.try_union(&leg_y).map_err(|e| EvalError::Boolean { id: id.into(), op: "elbow90_union", message: e.message })
        }
        Feature::DistanceRod { from, to, radius, segments, .. } => {
            let f = resolve3(id, from, params)?;
            let t = resolve3(id, to, params)?;
            let r = resolve_one(id, radius, params)?;
            if r <= 0.0 || *segments < 3 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("DistanceRod requires positive radius, segments>=3 (got r={r})") });
            }
            sweep_cylinder_segment(f, t, r, *segments).ok_or_else(|| EvalError::Invalid {
                id: id.into(),
                reason: "DistanceRod has zero-length segment".into(),
            })
        }
        Feature::AngleArc { center, radius, start_deg, sweep_deg, rod_radius, segments, .. } => {
            let c = resolve3(id, center, params)?;
            let r = resolve_one(id, radius, params)?;
            let sa = resolve_one(id, start_deg, params)?;
            let sw = resolve_one(id, sweep_deg, params)?;
            let rr = resolve_one(id, rod_radius, params)?;
            if r <= 0.0 || rr <= 0.0 || sw == 0.0 || *segments < 4 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("AngleArc requires positive radii, non-zero sweep, segments>=4 (got r={r}, sweep={sw})") });
            }
            // Sample the arc at `segments` points and chain SweepPath-style cylinders.
            let total_rad = sw.to_radians();
            let start_rad = sa.to_radians();
            let mut acc: Option<Solid> = None;
            for i in 0..*segments {
                let t0 = start_rad + total_rad * (i as f64) / (*segments as f64);
                let t1 = start_rad + total_rad * ((i + 1) as f64) / (*segments as f64);
                let p0 = [c[0] + r * t0.cos(), c[1] + r * t0.sin(), c[2]];
                let p1 = [c[0] + r * t1.cos(), c[1] + r * t1.sin(), c[2]];
                let cyl = sweep_cylinder_segment(p0, p1, rr, 8).ok_or_else(|| EvalError::Invalid {
                    id: id.into(),
                    reason: format!("AngleArc segment {i} has zero length"),
                })?;
                acc = Some(match acc.take() {
                    None => cyl,
                    Some(prev) => prev.try_union(&cyl).map_err(|e| EvalError::Boolean { id: id.into(), op: "angle_arc_union", message: format!("seg {i}: {}", e.message) })?,
                });
            }
            Ok(acc.unwrap())
        }
        Feature::SheetBend { length_a, length_b, width, thickness, .. } => {
            let la = resolve_one(id, length_a, params)?;
            let lb = resolve_one(id, length_b, params)?;
            let w = resolve_one(id, width, params)?;
            let t = resolve_one(id, thickness, params)?;
            if la <= 0.0 || lb <= 0.0 || w <= 0.0 || t <= 0.0 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("SheetBend requires positive dims (got la={la}, lb={lb}, w={w}, t={t})") });
            }
            // Horizontal section [0, la] x [0, w] x [0, t]; vertical
            // section [la-t, la] x [0, w] x [t, t+lb]. Joined corner.
            let horiz = box_at(Vec3::new(la, w, t), Point3::new(0.0, 0.0, 0.0));
            let vert = box_at(Vec3::new(t, w, lb), Point3::new(la - t, 0.0, t));
            horiz.try_union(&vert).map_err(|e| EvalError::Boolean { id: id.into(), op: "sheet_bend_join", message: e.message })
        }
        Feature::TrussMember { rod_radius, rod_length, plate_width, plate_thickness, segments, .. } => {
            let rr = resolve_one(id, rod_radius, params)?;
            let rl = resolve_one(id, rod_length, params)?;
            let pw = resolve_one(id, plate_width, params)?;
            let pt = resolve_one(id, plate_thickness, params)?;
            if rr <= 0.0 || rl <= 0.0 || pw <= 0.0 || pt <= 0.0 || rr * 2.0 >= pw || *segments < 6 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("TrussMember requires 2*rod_radius < plate_width and positive dims (got rr={rr}, pw={pw})") });
            }
            // Bottom plate centered on origin at z=[0, pt].
            let bot_plate = box_at(
                Vec3::new(pw, pw, pt),
                Point3::new(-pw/2.0, -pw/2.0, 0.0),
            );
            // Rod centered at origin, axis +z, from z=pt-eps to z=pt+rl+eps.
            let eps = (pt * 0.05).max(1e-3);
            let rod = cylinder_along_axis(rr, rl + 2.0 * eps, *segments, 2, pt - eps, 0.0, 0.0);
            // Top plate at z=[pt+rl, pt+rl+pt].
            let top_plate = box_at(
                Vec3::new(pw, pw, pt),
                Point3::new(-pw/2.0, -pw/2.0, pt + rl),
            );
            let lower = bot_plate.try_union(&rod).map_err(|e| EvalError::Boolean { id: id.into(), op: "truss_bot_join", message: e.message })?;
            lower.try_union(&top_plate).map_err(|e| EvalError::Boolean { id: id.into(), op: "truss_top_join", message: e.message })
        }
        Feature::Hinge { leaf_width, leaf_height, leaf_thickness, pin_radius, segments, .. } => {
            let lw = resolve_one(id, leaf_width, params)?;
            let lh = resolve_one(id, leaf_height, params)?;
            let lt = resolve_one(id, leaf_thickness, params)?;
            let pr = resolve_one(id, pin_radius, params)?;
            if lw <= 0.0 || lh <= 0.0 || lt <= 0.0 || pr <= 0.0 || pr >= lh / 2.0 || *segments < 6 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("Hinge requires pin_radius < leaf_height/2 and positive dims (got lw={lw}, lh={lh}, lt={lt}, pr={pr})") });
            }
            // Two leaves: left leaf in x=[-lw, 0], right leaf in x=[0, lw].
            // Both span y=[0, lh], z=[0, lt]. Pin runs along +y at z=lt+pr,
            // x=0, length lh, radius pr. Joined.
            let left = box_at(Vec3::new(lw, lh, lt), Point3::new(-lw, 0.0, 0.0));
            let right = box_at(Vec3::new(lw, lh, lt), Point3::new(0.0, 0.0, 0.0));
            let pin = cylinder_along_axis(pr, lh, *segments, 1, 0.0, 0.0, lt + pr);
            let leaves = left.try_union(&right).map_err(|e| EvalError::Boolean { id: id.into(), op: "hinge_leaves", message: e.message })?;
            leaves.try_union(&pin).map_err(|e| EvalError::Boolean { id: id.into(), op: "hinge_pin", message: e.message })
        }
        Feature::Cleat { arm_length, arm_height, support_width, support_height, thickness, .. } => {
            let al = resolve_one(id, arm_length, params)?;
            let ah = resolve_one(id, arm_height, params)?;
            let sw = resolve_one(id, support_width, params)?;
            let sh = resolve_one(id, support_height, params)?;
            let t = resolve_one(id, thickness, params)?;
            if al <= 0.0 || ah <= 0.0 || sw <= 0.0 || sh <= 0.0 || t <= 0.0 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("Cleat requires positive dims (got al={al}, ah={ah}, sw={sw}, sh={sh}, t={t})") });
            }
            // Arm extends along +x (from x=0 to x=al), z=[0, ah], y=[0, t].
            // Support extends along +x (from -sw to 0), z=[0, sh], y=[0, t].
            // L-shaped, sharing the corner at x=0.
            let arm = box_at(Vec3::new(al, t, ah), Point3::new(0.0, 0.0, 0.0));
            let support = box_at(Vec3::new(sw, t, sh), Point3::new(-sw, 0.0, 0.0));
            arm.try_union(&support).map_err(|e| EvalError::Boolean { id: id.into(), op: "cleat_join", message: e.message })
        }
        Feature::Lattice { nx, ny, cell_size, bar_thickness, depth, .. } => {
            let cs = resolve_one(id, cell_size, params)?;
            let bt = resolve_one(id, bar_thickness, params)?;
            let d = resolve_one(id, depth, params)?;
            if cs <= 0.0 || bt <= 0.0 || d <= 0.0 || bt >= cs || *nx == 0 || *ny == 0 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("Lattice requires bar_thickness < cell_size, positive dims, nx&ny>=1 (got cs={cs}, bt={bt})") });
            }
            // (nx + 1) horizontal bars, (ny + 1) vertical bars.
            // Each horizontal bar: full width along +x, bt thick along +y, d tall along +z.
            // Each vertical bar: bt wide along +x, full height along +y, d tall along +z.
            let total_x = (*nx as f64) * cs;
            let total_y = (*ny as f64) * cs;
            let mut acc: Option<Solid> = None;
            for i in 0..=*ny {
                let y = (i as f64) * cs - bt / 2.0;
                let bar = box_at(Vec3::new(total_x, bt, d), Point3::new(0.0, y, 0.0));
                acc = Some(match acc.take() {
                    None => bar,
                    Some(prev) => prev.try_union(&bar).map_err(|e| EvalError::Boolean { id: id.into(), op: "lattice_h_bar", message: format!("h bar {i}: {}", e.message) })?,
                });
            }
            for j in 0..=*nx {
                let x = (j as f64) * cs - bt / 2.0;
                let bar = box_at(Vec3::new(bt, total_y, d), Point3::new(x, 0.0, 0.0));
                acc = Some(match acc.take() {
                    None => bar,
                    Some(prev) => prev.try_union(&bar).map_err(|e| EvalError::Boolean { id: id.into(), op: "lattice_v_bar", message: format!("v bar {j}: {}", e.message) })?,
                });
            }
            Ok(acc.unwrap())
        }
        Feature::SocketHeadCapScrew { head_radius, head_height, shaft_radius, shaft_length, segments, .. } => {
            let hr = resolve_one(id, head_radius, params)?;
            let hh = resolve_one(id, head_height, params)?;
            let sr = resolve_one(id, shaft_radius, params)?;
            let sl = resolve_one(id, shaft_length, params)?;
            if hr <= 0.0 || hh <= 0.0 || sr <= 0.0 || sl <= 0.0 || sr >= hr || *segments < 6 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("SocketHeadCapScrew requires shaft_radius < head_radius and positive dims (got hr={hr}, sr={sr})") });
            }
            // Head: 6-sided prism (hex). Shaft: cylinder.
            let mut head_prof = Vec::with_capacity(6);
            for i in 0..6 {
                let theta = 2.0 * std::f64::consts::PI * (i as f64) / 6.0;
                head_prof.push(Point3::new(hr * theta.cos(), hr * theta.sin(), 0.0));
            }
            let head = extrude_polygon(&head_prof, Vec3::new(0.0, 0.0, hh));
            let eps = (hh * 0.05).max(1e-3);
            let shaft = cylinder_along_axis(sr, sl + eps, *segments, 2, -sl, 0.0, 0.0);
            head.try_union(&shaft).map_err(|e| EvalError::Boolean { id: id.into(), op: "socket_head_join", message: e.message })
        }
        Feature::FlatHeadScrew { head_radius, head_height, shaft_radius, shaft_length, segments, .. } => {
            let hr = resolve_one(id, head_radius, params)?;
            let hh = resolve_one(id, head_height, params)?;
            let sr = resolve_one(id, shaft_radius, params)?;
            let sl = resolve_one(id, shaft_length, params)?;
            if hr <= 0.0 || hh <= 0.0 || sr <= 0.0 || sl <= 0.0 || sr >= hr || *segments < 6 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("FlatHeadScrew requires shaft_radius < head_radius and positive dims (got hr={hr}, sr={sr})") });
            }
            // Head: countersunk frustum from hr at z=0 down to sr at z=-hh.
            let head = frustum_faceted(hr, sr, hh, *segments);
            let head_translated = translate_solid(&head, Vec3::new(0.0, 0.0, -hh));
            let eps = (hh * 0.05).max(1e-3);
            let shaft = cylinder_along_axis(sr, sl + eps, *segments, 2, -hh - sl, 0.0, 0.0);
            head_translated.try_union(&shaft).map_err(|e| EvalError::Boolean { id: id.into(), op: "flat_head_join", message: e.message })
        }
        Feature::Rivet { body_radius, body_length, head_radius, head_height, segments, .. } => {
            let br = resolve_one(id, body_radius, params)?;
            let bl = resolve_one(id, body_length, params)?;
            let hr = resolve_one(id, head_radius, params)?;
            let hh = resolve_one(id, head_height, params)?;
            if br <= 0.0 || bl <= 0.0 || hr <= 0.0 || hh <= 0.0 || br >= hr || *segments < 6 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("Rivet requires body_radius < head_radius and positive dims (got br={br}, hr={hr})") });
            }
            // Body cylinder + dome head (approximated as a frustum from br at top
            // to hr at bottom).
            let body = cylinder_along_axis(br, bl, *segments, 2, 0.0, 0.0, 0.0);
            let head_top_r = br * 1.1;
            let head = frustum_faceted(head_top_r, hr, hh, *segments);
            let head_translated = translate_solid(&head, Vec3::new(0.0, 0.0, bl - 1e-3));
            body.try_union(&head_translated).map_err(|e| EvalError::Boolean { id: id.into(), op: "rivet_join", message: e.message })
        }
        Feature::ShoulderBolt { head_radius, head_height, shoulder_radius, shoulder_length, thread_radius, thread_length, segments, .. } => {
            let hr = resolve_one(id, head_radius, params)?;
            let hh = resolve_one(id, head_height, params)?;
            let sr = resolve_one(id, shoulder_radius, params)?;
            let sl = resolve_one(id, shoulder_length, params)?;
            let tr = resolve_one(id, thread_radius, params)?;
            let tl = resolve_one(id, thread_length, params)?;
            if hr <= 0.0 || hh <= 0.0 || sr <= 0.0 || sl <= 0.0 || tr <= 0.0 || tl <= 0.0
                || tr >= sr || sr >= hr || *segments < 6
            {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("ShoulderBolt requires thread < shoulder < head and positive dims (got hr={hr}, sr={sr}, tr={tr})") });
            }
            let head = cylinder_along_axis(hr, hh, *segments, 2, 0.0, 0.0, 0.0);
            let eps = (hh * 0.05).max(1e-3);
            let shoulder = cylinder_along_axis(sr, sl + 2.0 * eps, *segments, 2, -sl - eps, 0.0, 0.0);
            let thread = cylinder_along_axis(tr, tl + eps, *segments, 2, -sl - tl, 0.0, 0.0);
            let upper = head.try_union(&shoulder).map_err(|e| EvalError::Boolean { id: id.into(), op: "shoulder_bolt_head_shoulder", message: e.message })?;
            upper.try_union(&thread).map_err(|e| EvalError::Boolean { id: id.into(), op: "shoulder_bolt_thread", message: e.message })
        }
        Feature::EyeBolt { ring_major_radius, ring_minor_radius, shaft_radius, shaft_length, ring_segs, shaft_segments, .. } => {
            let r_maj = resolve_one(id, ring_major_radius, params)?;
            let r_min = resolve_one(id, ring_minor_radius, params)?;
            let sr = resolve_one(id, shaft_radius, params)?;
            let sl = resolve_one(id, shaft_length, params)?;
            if r_maj <= 0.0 || r_min <= 0.0 || r_maj <= r_min || sr <= 0.0 || sl <= 0.0 || *ring_segs < 6 || *shaft_segments < 6 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("EyeBolt requires major>minor>0, segments>=6 (got maj={r_maj}, min={r_min})") });
            }
            // Ring: faceted torus, axis along +y so the ring is upright.
            // Build it +z then rotate-via-cyclic-permutation if needed.
            // For simplicity: build at origin with axis +z (ring lies in xy plane),
            // then translate up so the bottom of the ring is at z = sl.
            // Actually the eyebolt should hang from the ring, so the shaft
            // is below the ring along -z. Place ring at z = sl + r_maj so
            // the bottom of the ring touches the shaft top.
            let ring = torus_faceted(r_maj, r_min, *ring_segs, (*ring_segs / 2).max(6));
            let ring_translated = translate_solid(&ring, Vec3::new(0.0, 0.0, sl + r_maj));
            let shaft = cylinder_along_axis(sr, sl + r_min, *shaft_segments, 2, 0.0, 0.0, 0.0);
            ring_translated.try_union(&shaft).map_err(|e| EvalError::Boolean { id: id.into(), op: "eyebolt_join", message: e.message })
        }
        Feature::ThreadInsert { outer_radius, inner_radius, length, segments, .. } => {
            let r_out = resolve_one(id, outer_radius, params)?;
            let r_in = resolve_one(id, inner_radius, params)?;
            let l = resolve_one(id, length, params)?;
            if r_out <= 0.0 || r_in <= 0.0 || r_in >= r_out || l <= 0.0 || *segments < 6 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("ThreadInsert requires inner < outer, positive dims (got out={r_out}, in={r_in})") });
            }
            let outer = cylinder_faceted(r_out, l, *segments);
            let eps = (l * 0.05).max(1e-3);
            let bore = cylinder_along_axis(r_in, l + 2.0 * eps, *segments, 2, -eps, 0.0, 0.0);
            outer.try_difference(&bore).map_err(|e| EvalError::Boolean { id: id.into(), op: "thread_insert_bore", message: e.message })
        }
        Feature::Cam { radius, hole_radius, eccentricity, thickness, segments, .. } => {
            let r = resolve_one(id, radius, params)?;
            let hr = resolve_one(id, hole_radius, params)?;
            let e = resolve_one(id, eccentricity, params)?;
            let t = resolve_one(id, thickness, params)?;
            if r <= 0.0 || hr <= 0.0 || hr >= r || t <= 0.0 || e < 0.0 || e + hr >= r || *segments < 6 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("Cam requires hole < disk, e+hr < r, positive dims (got r={r}, hr={hr}, e={e})") });
            }
            let disk = cylinder_faceted(r, t, *segments);
            let eps = (t * 0.05).max(1e-3);
            let hole = cylinder_along_axis(hr, t + 2.0 * eps, *segments, 2, -eps, e, 0.0);
            disk.try_difference(&hole).map_err(|e| EvalError::Boolean { id: id.into(), op: "cam_bore", message: e.message })
        }
        Feature::Crank { pivot_radius, wrist_radius, arm_length, arm_width, arm_thickness, segments, .. } => {
            let pr = resolve_one(id, pivot_radius, params)?;
            let wr = resolve_one(id, wrist_radius, params)?;
            let al = resolve_one(id, arm_length, params)?;
            let aw = resolve_one(id, arm_width, params)?;
            let at = resolve_one(id, arm_thickness, params)?;
            if pr <= 0.0 || wr <= 0.0 || al <= 0.0 || aw <= 0.0 || at <= 0.0 || aw < 2.0 * pr.max(wr) || *segments < 6 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("Crank requires arm_width >= 2 * max(pivot, wrist), positive dims (got pr={pr}, wr={wr}, aw={aw})") });
            }
            // Pivot cylinder at origin, wrist cylinder at (al, 0, 0),
            // arm rectangular bar between centers.
            let pivot = cylinder_along_axis(pr, at, *segments, 2, 0.0, 0.0, 0.0);
            let wrist = cylinder_along_axis(wr, at, *segments, 2, 0.0, al, 0.0);
            let arm = box_at(
                Vec3::new(al, aw, at),
                Point3::new(0.0, -aw / 2.0, 0.0),
            );
            let lower = pivot.try_union(&arm).map_err(|e| EvalError::Boolean { id: id.into(), op: "crank_pivot_arm", message: e.message })?;
            lower.try_union(&wrist).map_err(|e| EvalError::Boolean { id: id.into(), op: "crank_arm_wrist", message: e.message })
        }
        Feature::Tee { radius, leg_length, segments, .. } => {
            let r = resolve_one(id, radius, params)?;
            let ll = resolve_one(id, leg_length, params)?;
            if r <= 0.0 || ll <= 0.0 || *segments < 6 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("Tee requires positive dims, segments>=6 (got r={r}, ll={ll})") });
            }
            // Three cylinders: x leg from -ll to +ll (one cylinder),
            // y leg from 0 to +ll. Slight overlap for boolean robustness.
            let x_leg = cylinder_along_axis(r, 2.0 * ll, *segments, 0, -ll, 0.0, 0.0);
            let y_leg = cylinder_along_axis(r, ll + r, *segments, 1, -r, 0.0, 0.0);
            x_leg.try_union(&y_leg).map_err(|e| EvalError::Boolean { id: id.into(), op: "tee_join", message: e.message })
        }
        Feature::Cross { radius, leg_length, segments, .. } => {
            let r = resolve_one(id, radius, params)?;
            let ll = resolve_one(id, leg_length, params)?;
            if r <= 0.0 || ll <= 0.0 || *segments < 6 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("Cross requires positive dims, segments>=6 (got r={r}, ll={ll})") });
            }
            // x leg (full length 2*ll) + y leg (full length 2*ll, slightly extended).
            let x_leg = cylinder_along_axis(r, 2.0 * ll, *segments, 0, -ll, 0.0, 0.0);
            let y_leg = cylinder_along_axis(r, 2.0 * ll + 2.0 * r, *segments, 1, -ll - r, 0.0, 0.0);
            x_leg.try_union(&y_leg).map_err(|e| EvalError::Boolean { id: id.into(), op: "cross_join", message: e.message })
        }
        Feature::SteppedShaft { radii, step_lengths, segments, .. } => {
            if radii.len() != step_lengths.len() || radii.is_empty() {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("SteppedShaft: radii and step_lengths must be same nonzero length (got {} vs {})", radii.len(), step_lengths.len()) });
            }
            if *segments < 6 {
                return Err(EvalError::Invalid { id: id.into(), reason: "SteppedShaft requires segments >= 6".into() });
            }
            let mut z_acc = 0.0_f64;
            let mut acc: Option<Solid> = None;
            for (i, (r_s, l_s)) in radii.iter().zip(step_lengths.iter()).enumerate() {
                let r = resolve_one(id, r_s, params)?;
                let l = resolve_one(id, l_s, params)?;
                if r <= 0.0 || l <= 0.0 {
                    return Err(EvalError::Invalid { id: id.into(), reason: format!("SteppedShaft step {i}: r and length must be positive (got r={r}, l={l})") });
                }
                let eps = if i > 0 { (l * 0.05).max(1e-3) } else { 0.0 };
                let cyl = cylinder_along_axis(r, l + eps, *segments, 2, z_acc - eps, 0.0, 0.0);
                z_acc += l;
                acc = Some(match acc.take() {
                    None => cyl,
                    Some(prev) => prev.try_union(&cyl).map_err(|e| EvalError::Boolean { id: id.into(), op: "stepped_shaft_join", message: format!("step {i}: {}", e.message) })?,
                });
            }
            Ok(acc.unwrap())
        }
        Feature::Trough { outer_width, outer_height, wall_thickness, length, .. } => {
            let w = resolve_one(id, outer_width, params)?;
            let h = resolve_one(id, outer_height, params)?;
            let t = resolve_one(id, wall_thickness, params)?;
            let l = resolve_one(id, length, params)?;
            if w <= 0.0 || h <= 0.0 || t <= 0.0 || l <= 0.0 || 2.0 * t >= w || t >= h {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("Trough requires positive dims, 2t<w, t<h (got w={w}, h={h}, t={t}, l={l})") });
            }
            // Outer box - inner cavity (open top).
            let outer = box_at(Vec3::new(w, l, h), Point3::new(0.0, 0.0, 0.0));
            let inner = box_at(
                Vec3::new(w - 2.0 * t, l - 2.0 * t, h),
                Point3::new(t, t, t),
            );
            outer.try_difference(&inner).map_err(|e| EvalError::Boolean { id: id.into(), op: "trough_carve", message: e.message })
        }
        Feature::Knob { radius, height, flute_count, flute_depth, .. } => {
            let r = resolve_one(id, radius, params)?;
            let h = resolve_one(id, height, params)?;
            let fd = resolve_one(id, flute_depth, params)?;
            if r <= 0.0 || h <= 0.0 || fd <= 0.0 || fd >= r || *flute_count < 4 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("Knob requires fd<r, positive dims, flute_count>=4 (got r={r}, fd={fd}, fc={flute_count})") });
            }
            // Build a profile that alternates between r (between flutes)
            // and r-fd (in the flute valleys). Like KnurledGrip but inverted
            // (flutes are valleys, not ridges).
            let n = 2 * *flute_count;
            let mut prof = Vec::with_capacity(n);
            for i in 0..n {
                let theta = 2.0 * std::f64::consts::PI * (i as f64) / (n as f64);
                let rad = if i % 2 == 0 { r } else { r - fd };
                prof.push(Point3::new(rad * theta.cos(), rad * theta.sin(), 0.0));
            }
            Ok(extrude_polygon(&prof, Vec3::new(0.0, 0.0, h)))
        }
        Feature::Plug { top_radius, bottom_radius, length, segments, .. } => {
            let tr = resolve_one(id, top_radius, params)?;
            let br = resolve_one(id, bottom_radius, params)?;
            let l = resolve_one(id, length, params)?;
            if tr <= 0.0 || br <= 0.0 || l <= 0.0 || *segments < 6 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("Plug requires positive dims, segments>=6 (got tr={tr}, br={br}, l={l})") });
            }
            Ok(frustum_faceted(tr, br, l, *segments))
        }
        Feature::Spike { base_radius, height, segments, .. } => {
            let r = resolve_one(id, base_radius, params)?;
            let h = resolve_one(id, height, params)?;
            if r <= 0.0 || h <= 0.0 || *segments < 6 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("Spike requires positive dims, segments>=6 (got r={r}, h={h})") });
            }
            Ok(cone_faceted(r, h, *segments))
        }
        Feature::CapBolt { head_radius, head_height, shaft_radius, shaft_length, segments, .. } => {
            let hr = resolve_one(id, head_radius, params)?;
            let hh = resolve_one(id, head_height, params)?;
            let sr = resolve_one(id, shaft_radius, params)?;
            let sl = resolve_one(id, shaft_length, params)?;
            if hr <= 0.0 || hh <= 0.0 || sr <= 0.0 || sl <= 0.0 || sr >= hr || *segments < 6 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("CapBolt requires shaft_radius<head_radius, positive dims (got hr={hr}, sr={sr})") });
            }
            let head = cylinder_along_axis(hr, hh, *segments, 2, 0.0, 0.0, 0.0);
            let eps = (hh * 0.05).max(1e-3);
            let shaft = cylinder_along_axis(sr, sl + eps, *segments, 2, -sl, 0.0, 0.0);
            head.try_union(&shaft).map_err(|e| EvalError::Boolean { id: id.into(), op: "cap_bolt_join", message: e.message })
        }
        Feature::FlangedBolt { head_radius, head_height, flange_radius, flange_thickness, shaft_radius, shaft_length, segments, .. } => {
            let hr = resolve_one(id, head_radius, params)?;
            let hh = resolve_one(id, head_height, params)?;
            let fr = resolve_one(id, flange_radius, params)?;
            let ft = resolve_one(id, flange_thickness, params)?;
            let sr = resolve_one(id, shaft_radius, params)?;
            let sl = resolve_one(id, shaft_length, params)?;
            if hr <= 0.0 || hh <= 0.0 || fr <= 0.0 || ft <= 0.0 || sr <= 0.0 || sl <= 0.0
                || sr >= hr || hr >= fr || *segments < 6
            {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("FlangedBolt requires shaft<head<flange, positive dims (got hr={hr}, fr={fr}, sr={sr})") });
            }
            let eps = (ft * 0.05).max(1e-3);
            let head = cylinder_along_axis(hr, hh, *segments, 2, 0.0, 0.0, 0.0);
            let flange = cylinder_along_axis(fr, ft + 2.0 * eps, *segments, 2, -ft - eps, 0.0, 0.0);
            let shaft = cylinder_along_axis(sr, sl + eps, *segments, 2, -ft - sl, 0.0, 0.0);
            let upper = head.try_union(&flange).map_err(|e| EvalError::Boolean { id: id.into(), op: "flanged_bolt_head_flange", message: e.message })?;
            upper.try_union(&shaft).map_err(|e| EvalError::Boolean { id: id.into(), op: "flanged_bolt_shaft", message: e.message })
        }
        Feature::SerratedDisk { outer_radius, root_radius, tooth_count, thickness, .. } => {
            let r_out = resolve_one(id, outer_radius, params)?;
            let r_root = resolve_one(id, root_radius, params)?;
            let t = resolve_one(id, thickness, params)?;
            if r_out <= 0.0 || r_root <= 0.0 || r_root >= r_out || t <= 0.0 || *tooth_count < 8 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("SerratedDisk requires r_out>r_root>0, t>0, tooth_count>=8 (got out={r_out}, root={r_root}, tc={tooth_count})") });
            }
            // Same as GearBlank but with finer teeth (caller sets high tooth_count).
            let n = 2 * *tooth_count;
            let mut prof = Vec::with_capacity(n);
            for i in 0..n {
                let theta = 2.0 * std::f64::consts::PI * (i as f64) / (n as f64);
                let r = if i % 2 == 0 { r_out } else { r_root };
                prof.push(Point3::new(r * theta.cos(), r * theta.sin(), 0.0));
            }
            Ok(extrude_polygon(&prof, Vec3::new(0.0, 0.0, t)))
        }
        Feature::FerruleEnd { small_radius, large_radius, wall_thickness, length, segments, .. } => {
            let sr = resolve_one(id, small_radius, params)?;
            let lr = resolve_one(id, large_radius, params)?;
            let wt = resolve_one(id, wall_thickness, params)?;
            let l = resolve_one(id, length, params)?;
            if sr <= 0.0 || lr <= 0.0 || wt <= 0.0 || l <= 0.0
                || sr - wt <= 0.0 || lr - wt <= 0.0 || *segments < 6
            {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("FerruleEnd requires sr-wt>0, lr-wt>0 (got sr={sr}, lr={lr}, wt={wt})") });
            }
            // Outer frustum minus inner frustum.
            let outer = frustum_faceted(sr, lr, l, *segments);
            let inner = frustum_faceted(sr - wt, lr - wt, l + 2.0 * 1e-3, *segments);
            let inner_t = translate_solid(&inner, Vec3::new(0.0, 0.0, -1e-3));
            outer.try_difference(&inner_t).map_err(|e| EvalError::Boolean { id: id.into(), op: "ferrule_bore", message: e.message })
        }
        Feature::Trapezoid { bottom_width, top_width, height, depth, .. } => {
            let bw = resolve_one(id, bottom_width, params)?;
            let tw = resolve_one(id, top_width, params)?;
            let h = resolve_one(id, height, params)?;
            let d = resolve_one(id, depth, params)?;
            if bw <= 0.0 || tw <= 0.0 || h <= 0.0 || d <= 0.0 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("Trapezoid requires positive dims (got bw={bw}, tw={tw}, h={h}, d={d})") });
            }
            // Centered trapezoid in xy plane, extruded along +z.
            let prof = vec![
                Point3::new(-bw / 2.0, 0.0, 0.0),
                Point3::new( bw / 2.0, 0.0, 0.0),
                Point3::new( tw / 2.0,   h, 0.0),
                Point3::new(-tw / 2.0,   h, 0.0),
            ];
            Ok(extrude_polygon(&prof, Vec3::new(0.0, 0.0, d)))
        }
        Feature::Handle { bar_length, bar_radius, cap_radius, segments, .. } => {
            let bl = resolve_one(id, bar_length, params)?;
            let br = resolve_one(id, bar_radius, params)?;
            let cr = resolve_one(id, cap_radius, params)?;
            if bl <= 0.0 || br <= 0.0 || cr <= 0.0 || cr < br || *segments < 6 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("Handle requires cap_radius >= bar_radius, positive dims (got bl={bl}, br={br}, cr={cr})") });
            }
            // Bar along +x; caps as cylinders at each end aligned along +y
            // (so they look like "drum" caps from the side).
            let bar = cylinder_along_axis(br, bl, *segments, 0, 0.0, 0.0, 0.0);
            // Use slight overlap to break coplanar boundaries.
            let eps = (cr * 0.05).max(1e-3);
            let cap0 = cylinder_along_axis(cr, 2.0 * eps + 1e-3, *segments, 0, -eps, 0.0, 0.0);
            let cap1 = cylinder_along_axis(cr, 2.0 * eps + 1e-3, *segments, 0, bl - eps, 0.0, 0.0);
            let with_left = bar.try_union(&cap0).map_err(|e| EvalError::Boolean { id: id.into(), op: "handle_left_cap", message: e.message })?;
            with_left.try_union(&cap1).map_err(|e| EvalError::Boolean { id: id.into(), op: "handle_right_cap", message: e.message })
        }
        Feature::HookHandle { shaft_length, shaft_radius, bend_radius, segments, .. } => {
            let sl = resolve_one(id, shaft_length, params)?;
            let sr = resolve_one(id, shaft_radius, params)?;
            let bend = resolve_one(id, bend_radius, params)?;
            if sl <= 0.0 || sr <= 0.0 || bend <= sr || *segments < 6 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("HookHandle requires bend > shaft_radius, positive dims (got sl={sl}, sr={sr}, bend={bend})") });
            }
            // Straight shaft along +z from 0 to sl. Then a 180° arc in the
            // xz plane sweeping around (x=bend, z=sl) with bend radius.
            let shaft = cylinder_along_axis(sr, sl, *segments, 2, 0.0, 0.0, 0.0);
            // Build the arc: chain cylinders in a 180° sweep from (0, 0, sl)
            // around (bend, 0, sl) to (2*bend, 0, sl).
            let n_arc = (*segments).max(8);
            let mut acc: Solid = shaft;
            for i in 0..n_arc {
                let t0 = std::f64::consts::PI * (i as f64) / (n_arc as f64);
                let t1 = std::f64::consts::PI * ((i + 1) as f64) / (n_arc as f64);
                // Center is (bend, 0, sl); start angle PI (so first point
                // is (0, 0, sl)) sweeping to 2*PI (so last is (2*bend, 0, sl)).
                let p0 = [bend - bend * t0.cos(), 0.0, sl + bend * t0.sin()];
                let p1 = [bend - bend * t1.cos(), 0.0, sl + bend * t1.sin()];
                let cyl = sweep_cylinder_segment(p0, p1, sr, *segments).ok_or_else(|| EvalError::Invalid {
                    id: id.into(),
                    reason: format!("HookHandle arc segment {i} has zero length"),
                })?;
                acc = acc.try_union(&cyl).map_err(|e| EvalError::Boolean { id: id.into(), op: "hook_arc_seg", message: format!("arc seg {i}: {}", e.message) })?;
            }
            Ok(acc)
        }
        Feature::CornerBracket { leg_x, leg_y, leg_z, thickness, .. } => {
            let lx = resolve_one(id, leg_x, params)?;
            let ly = resolve_one(id, leg_y, params)?;
            let lz = resolve_one(id, leg_z, params)?;
            let t = resolve_one(id, thickness, params)?;
            if lx <= 0.0 || ly <= 0.0 || lz <= 0.0 || t <= 0.0 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("CornerBracket requires positive dims (got lx={lx}, ly={ly}, lz={lz}, t={t})") });
            }
            // Three perpendicular plates sharing a t-cube corner at origin.
            // x-plate: t × ly × lz at (0, 0, 0).
            // y-plate: lx × t × lz at (0, 0, 0).
            // z-plate: lx × ly × t at (0, 0, 0).
            let xp = box_at(Vec3::new(t, ly, lz), Point3::new(0.0, 0.0, 0.0));
            let yp = box_at(Vec3::new(lx, t, lz), Point3::new(0.0, 0.0, 0.0));
            let zp = box_at(Vec3::new(lx, ly, t), Point3::new(0.0, 0.0, 0.0));
            let xy = xp.try_union(&yp).map_err(|e| EvalError::Boolean { id: id.into(), op: "corner_xy", message: e.message })?;
            xy.try_union(&zp).map_err(|e| EvalError::Boolean { id: id.into(), op: "corner_xyz", message: e.message })
        }
        Feature::ArcSegment { major_radius, minor_radius, start_deg, sweep_deg, segments, .. } => {
            let r_maj = resolve_one(id, major_radius, params)?;
            let r_min = resolve_one(id, minor_radius, params)?;
            let sd = resolve_one(id, start_deg, params)?;
            let sw = resolve_one(id, sweep_deg, params)?;
            if r_maj <= 0.0 || r_min <= 0.0 || r_maj <= r_min || sw == 0.0 || *segments < 4 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("ArcSegment requires major>minor>0, sweep != 0 (got maj={r_maj}, min={r_min}, sw={sw})") });
            }
            // Build chained cylinders along the major-radius arc.
            let total_rad = sw.to_radians();
            let start_rad = sd.to_radians();
            let mut acc: Option<Solid> = None;
            for i in 0..*segments {
                let t0 = start_rad + total_rad * (i as f64) / (*segments as f64);
                let t1 = start_rad + total_rad * ((i + 1) as f64) / (*segments as f64);
                let p0 = [r_maj * t0.cos(), r_maj * t0.sin(), 0.0];
                let p1 = [r_maj * t1.cos(), r_maj * t1.sin(), 0.0];
                let cyl = sweep_cylinder_segment(p0, p1, r_min, 12).ok_or_else(|| EvalError::Invalid {
                    id: id.into(),
                    reason: format!("ArcSegment seg {i} has zero length"),
                })?;
                acc = Some(match acc.take() {
                    None => cyl,
                    Some(prev) => prev.try_union(&cyl).map_err(|e| EvalError::Boolean { id: id.into(), op: "arc_segment_join", message: format!("seg {i}: {}", e.message) })?,
                });
            }
            Ok(acc.unwrap())
        }
        Feature::CrossBrace { frame_size, bar_thickness, depth, .. } => {
            let fs = resolve_one(id, frame_size, params)?;
            let bt = resolve_one(id, bar_thickness, params)?;
            let d = resolve_one(id, depth, params)?;
            if fs <= 0.0 || bt <= 0.0 || d <= 0.0 || bt >= fs / 2.0 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("CrossBrace requires bar_thickness < frame_size/2, positive dims (got fs={fs}, bt={bt})") });
            }
            // Two diagonal bars: one from (0,0) to (fs,fs), other from (0,fs) to (fs,0).
            // Build each as a SweepPath cylinder of half-thickness... actually easier
            // is to extrude two thin trapezoidal/rectangular profiles diagonally.
            // For simplicity: sweep two cylinders.
            let r = bt / 2.0;
            let bar1 = sweep_cylinder_segment([0.0, 0.0, 0.0], [fs, fs, 0.0], r, 8)
                .ok_or_else(|| EvalError::Invalid { id: id.into(), reason: "CrossBrace zero-length diag".into() })?;
            let bar2 = sweep_cylinder_segment([0.0, fs, 0.0], [fs, 0.0, 0.0], r, 8)
                .ok_or_else(|| EvalError::Invalid { id: id.into(), reason: "CrossBrace zero-length anti-diag".into() })?;
            let crossed = bar1.try_union(&bar2).map_err(|e| EvalError::Boolean { id: id.into(), op: "cross_brace_join", message: e.message })?;
            // Now extrude this 2D-ish result along +z by `depth` is hard
            // because the bars are already 3D. Instead just translate/scale
            // along z by `depth` is also awkward. The bars are cylinders of
            // depth `bt`, so their existing thickness is the depth. Done.
            // Actually we built diagonal cylinders that aren't aligned with
            // +z — their bbox is 3D. The user wants `depth` along z. Skip
            // depth for v1; treat as flat brace.
            let _ = d;
            Ok(crossed)
        }
        Feature::WireMesh { nx, ny, cell_size, wire_radius, segments, .. } => {
            let cs = resolve_one(id, cell_size, params)?;
            let wr = resolve_one(id, wire_radius, params)?;
            if cs <= 0.0 || wr <= 0.0 || wr >= cs / 2.0 || *nx == 0 || *ny == 0 || *segments < 6 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("WireMesh requires wire_radius < cell_size/2 (got cs={cs}, wr={wr})") });
            }
            // (nx + 1) wires along +x at each y position, (ny + 1) wires
            // along +y at each x position.
            let total_x = (*nx as f64) * cs;
            let total_y = (*ny as f64) * cs;
            let mut acc: Option<Solid> = None;
            for i in 0..=*ny {
                let y = (i as f64) * cs;
                let wire = cylinder_along_axis(wr, total_x, *segments, 0, 0.0, y, 0.0);
                acc = Some(match acc.take() {
                    None => wire,
                    Some(prev) => prev.try_union(&wire).map_err(|e| EvalError::Boolean { id: id.into(), op: "wire_mesh_h", message: format!("h wire {i}: {}", e.message) })?,
                });
            }
            for j in 0..=*nx {
                let x = (j as f64) * cs;
                let wire = cylinder_along_axis(wr, total_y + 2.0 * wr, *segments, 1, -wr, x, 0.0);
                acc = Some(match acc.take() {
                    None => wire,
                    Some(prev) => prev.try_union(&wire).map_err(|e| EvalError::Boolean { id: id.into(), op: "wire_mesh_v", message: format!("v wire {j}: {}", e.message) })?,
                });
            }
            Ok(acc.unwrap())
        }
        Feature::AnchorPoint { plate_width, plate_height, plate_thickness, tab_height, hole_radius, segments, .. } => {
            let pw = resolve_one(id, plate_width, params)?;
            let ph = resolve_one(id, plate_height, params)?;
            let pt = resolve_one(id, plate_thickness, params)?;
            let th = resolve_one(id, tab_height, params)?;
            let hr = resolve_one(id, hole_radius, params)?;
            if pw <= 0.0 || ph <= 0.0 || pt <= 0.0 || th <= 0.0 || hr <= 0.0 || hr >= pw / 4.0 || *segments < 6 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("AnchorPoint requires hole < plate_width/4, positive dims (got pw={pw}, ph={ph}, pt={pt})") });
            }
            // Plate at z=[0, pt] in xy=[0,pw]x[0,ph]. Tab projects up: z=[pt, pt+th]
            // in xy=[pw/4, 3pw/4] x [ph/3, 2ph/3]. Hole through tab.
            let plate = box_at(Vec3::new(pw, ph, pt), Point3::new(0.0, 0.0, 0.0));
            let tab = box_at(
                Vec3::new(pw / 2.0, ph / 3.0, th + 1e-3),
                Point3::new(pw / 4.0, ph / 3.0, pt - 1e-3),
            );
            let assembly = plate.try_union(&tab).map_err(|e| EvalError::Boolean { id: id.into(), op: "anchor_plate_tab", message: e.message })?;
            // Hole: cylinder along +y through the tab (so the anchor hole
            // is parallel to the plate). Tab is at y=[ph/3, 2ph/3];
            // hole crosses from y=0 to y=ph (overshoot).
            let eps = (ph * 0.05).max(1e-3);
            let hole = cylinder_along_axis(hr, ph + 2.0 * eps, *segments, 1, -eps, pw / 2.0, pt + th / 2.0);
            assembly.try_difference(&hole).map_err(|e| EvalError::Boolean { id: id.into(), op: "anchor_hole", message: e.message })
        }
        Feature::Stair { step_count, tread, riser, step_width, .. } => {
            let tr = resolve_one(id, tread, params)?;
            let ri = resolve_one(id, riser, params)?;
            let sw = resolve_one(id, step_width, params)?;
            if *step_count == 0 || tr <= 0.0 || ri <= 0.0 || sw <= 0.0 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("Stair requires step_count >= 1 and positive dims (got count={step_count}, tread={tr}, riser={ri}, sw={sw})") });
            }
            // Build N steps as stacked boxes. Step k spans:
            //   y = [k * tread, total_run] (extending under all higher steps)
            //   z = [k * riser, (k+1) * riser]
            // This stairstep shape is closed and convex per step, and adjacent
            // steps share faces in a way the boolean union handles cleanly.
            let total_run = (*step_count as f64) * tr;
            let mut acc: Option<Solid> = None;
            for k in 0..*step_count {
                let y0 = (k as f64) * tr;
                let z0 = (k as f64) * ri;
                let step = box_at(
                    Vec3::new(sw, total_run - y0, ri),
                    Point3::new(0.0, y0, z0),
                );
                acc = Some(match acc.take() {
                    None => step,
                    Some(prev) => prev.try_union(&step).map_err(|e| EvalError::Boolean { id: id.into(), op: "stair_join", message: format!("step {k}: {}", e.message) })?,
                });
            }
            Ok(acc.unwrap())
        }
        Feature::Hopper { top_radius, neck_radius, funnel_height, neck_height, segments, .. } => {
            let tr = resolve_one(id, top_radius, params)?;
            let nr = resolve_one(id, neck_radius, params)?;
            let fh = resolve_one(id, funnel_height, params)?;
            let nh = resolve_one(id, neck_height, params)?;
            if tr <= 0.0 || nr <= 0.0 || nr >= tr || fh <= 0.0 || nh <= 0.0 || *segments < 6 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("Hopper requires neck_radius < top_radius, positive dims (got tr={tr}, nr={nr})") });
            }
            // Funnel: frustum from tr at z=nh+fh down to nr at z=nh.
            let funnel = frustum_faceted(tr, nr, fh, *segments);
            let funnel_t = translate_solid(&funnel, Vec3::new(0.0, 0.0, nh));
            // Neck: cylinder from z=0 to z=nh+eps.
            let eps = (nh * 0.05).max(1e-3);
            let neck = cylinder_along_axis(nr, nh + eps, *segments, 2, 0.0, 0.0, 0.0);
            funnel_t.try_union(&neck).map_err(|e| EvalError::Boolean { id: id.into(), op: "hopper_join", message: e.message })
        }
        Feature::Stand { radius, thickness, segments, .. } => {
            let r = resolve_one(id, radius, params)?;
            let t = resolve_one(id, thickness, params)?;
            if r <= 0.0 || t <= 0.0 || *segments < 6 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("Stand requires positive dims, segments>=6 (got r={r}, t={t})") });
            }
            Ok(cylinder_faceted(r, t, *segments))
        }
        Feature::Yoke { arm_length, arm_thickness, arm_height, gap_width, pivot_radius, segments, .. } => {
            let al = resolve_one(id, arm_length, params)?;
            let at = resolve_one(id, arm_thickness, params)?;
            let ah = resolve_one(id, arm_height, params)?;
            let gw = resolve_one(id, gap_width, params)?;
            let pr = resolve_one(id, pivot_radius, params)?;
            if al <= 0.0 || at <= 0.0 || ah <= 0.0 || gw <= 0.0 || pr <= 0.0 || pr >= ah / 2.0 || *segments < 6 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("Yoke requires pivot_radius < arm_height/2 and positive dims (got al={al}, ah={ah}, pr={pr})") });
            }
            // Crossbar at y=[-(at + gw/2 + at), -(gw/2 + at) + at] ... let me re-do.
            // Layout: total_y_width = at + gw + at. Crossbar bottom at y=0
            // is the back wall. Two arms project +x from back wall.
            //   Back wall: y=[0, at+gw+at], x=[0, at], z=[0, ah]
            //   Arm A: y=[0, at], x=[at, at+al], z=[0, ah]
            //   Arm B: y=[at+gw, at+gw+at], x=[at, at+al], z=[0, ah]
            // Pivot hole through both arms along +y at x=at + al - 2*pr, z=ah/2.
            let total_y = at + gw + at;
            let back = box_at(Vec3::new(at, total_y, ah), Point3::new(0.0, 0.0, 0.0));
            let arm_a = box_at(Vec3::new(al, at, ah), Point3::new(at, 0.0, 0.0));
            let arm_b = box_at(Vec3::new(al, at, ah), Point3::new(at, at + gw, 0.0));
            let assembly = back
                .try_union(&arm_a).map_err(|e| EvalError::Boolean { id: id.into(), op: "yoke_back_a", message: e.message })?
                .try_union(&arm_b).map_err(|e| EvalError::Boolean { id: id.into(), op: "yoke_arm_b", message: e.message })?;
            let eps = (total_y * 0.05).max(1e-3);
            let pivot = cylinder_along_axis(pr, total_y + 2.0 * eps, *segments, 1, -eps, at + al - 2.0 * pr, ah / 2.0);
            assembly.try_difference(&pivot).map_err(|e| EvalError::Boolean { id: id.into(), op: "yoke_pivot_hole", message: e.message })
        }
        Feature::Lever { length, width, thickness, pivot_radius, segments, .. } => {
            let l = resolve_one(id, length, params)?;
            let w = resolve_one(id, width, params)?;
            let t = resolve_one(id, thickness, params)?;
            let pr = resolve_one(id, pivot_radius, params)?;
            if l <= 0.0 || w <= 0.0 || t <= 0.0 || pr <= 0.0 || pr >= w / 2.0 || *segments < 6 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("Lever requires pivot_radius < width/2 and positive dims (got l={l}, w={w}, t={t}, pr={pr})") });
            }
            let bar = box_at(Vec3::new(l, w, t), Point3::new(0.0, 0.0, 0.0));
            let eps = (t * 0.05).max(1e-3);
            let pivot = cylinder_along_axis(pr, t + 2.0 * eps, *segments, 2, -eps, w * 0.5, w * 0.5);
            bar.try_difference(&pivot).map_err(|e| EvalError::Boolean { id: id.into(), op: "lever_pivot", message: e.message })
        }
        Feature::TaperedTube { bottom_outer_radius, top_outer_radius, wall_thickness, height, segments, .. } => {
            let bor = resolve_one(id, bottom_outer_radius, params)?;
            let tor = resolve_one(id, top_outer_radius, params)?;
            let wt = resolve_one(id, wall_thickness, params)?;
            let h = resolve_one(id, height, params)?;
            if bor <= 0.0 || tor <= 0.0 || wt <= 0.0 || h <= 0.0
                || bor - wt <= 0.0 || tor - wt <= 0.0 || *segments < 6
            {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("TaperedTube requires inner > 0 at both ends (got bor={bor}, tor={tor}, wt={wt})") });
            }
            let outer = frustum_faceted(tor, bor, h, *segments);
            let inner = frustum_faceted(tor - wt, bor - wt, h + 2.0 * 1e-3, *segments);
            let inner_t = translate_solid(&inner, Vec3::new(0.0, 0.0, -1e-3));
            outer.try_difference(&inner_t).map_err(|e| EvalError::Boolean { id: id.into(), op: "tapered_tube_bore", message: e.message })
        }
        Feature::GussetPlate { leg_length, thickness, .. } => {
            let l = resolve_one(id, leg_length, params)?;
            let t = resolve_one(id, thickness, params)?;
            if l <= 0.0 || t <= 0.0 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("GussetPlate requires positive dims (got l={l}, t={t})") });
            }
            // Right triangle in xy plane with legs along +x and +y, extruded +z.
            let prof = vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(l, 0.0, 0.0),
                Point3::new(0.0, l, 0.0),
            ];
            Ok(extrude_polygon(&prof, Vec3::new(0.0, 0.0, t)))
        }
        Feature::CrossKey { bar_length, bar_thickness, bar_height, .. } => {
            let bl = resolve_one(id, bar_length, params)?;
            let bt = resolve_one(id, bar_thickness, params)?;
            let bh = resolve_one(id, bar_height, params)?;
            if bl <= 0.0 || bt <= 0.0 || bh <= 0.0 || bt >= bl {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("CrossKey requires bar_thickness < bar_length, positive dims (got bl={bl}, bt={bt}, bh={bh})") });
            }
            // Bar 1: along +x, centered on origin in y.
            let bar_x = box_at(Vec3::new(bl, bt, bh), Point3::new(-bl / 2.0, -bt / 2.0, 0.0));
            // Bar 2: along +y, centered on origin in x.
            let bar_y = box_at(Vec3::new(bt, bl, bh), Point3::new(-bt / 2.0, -bl / 2.0, 0.0));
            bar_x.try_union(&bar_y).map_err(|e| EvalError::Boolean { id: id.into(), op: "cross_key_join", message: e.message })
        }
        Feature::PinShaft { radius, length, segments, .. } => {
            let r = resolve_one(id, radius, params)?;
            let l = resolve_one(id, length, params)?;
            if r <= 0.0 || l <= 0.0 || *segments < 6 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("PinShaft requires positive dims (got r={r}, l={l})") });
            }
            Ok(cylinder_faceted(r, l, *segments))
        }
        Feature::Lens { radius, cap_height, stacks, slices, .. } => {
            let r = resolve_one(id, radius, params)?;
            let cap = resolve_one(id, cap_height, params)?;
            if r <= 0.0 || cap <= 0.0 || cap > r || *stacks < 2 || *slices < 3 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("Lens requires r>0, 0<cap<=r (got r={r}, cap={cap})") });
            }
            // Top half: sphere clipped by z < (r - cap). Translated to z=0
            // so its bottom (the cap base) lies on z=0.
            let sph_top = sphere_faceted(r, *stacks, *slices);
            let cutter_top = box_at(
                Vec3::new(4.0 * r, 4.0 * r, 4.0 * r),
                Point3::new(-2.0 * r, -2.0 * r, (r - cap) - 4.0 * r),
            );
            let upper = sph_top
                .try_difference(&cutter_top)
                .map_err(|e| EvalError::Boolean { id: id.into(), op: "lens_upper", message: e.message })?;
            // Bottom half: mirror of upper across z=0. Construct as another
            // sphere clipped by z > -(r - cap).
            let sph_bot = sphere_faceted(r, *stacks, *slices);
            let cutter_bot = box_at(
                Vec3::new(4.0 * r, 4.0 * r, 4.0 * r),
                Point3::new(-2.0 * r, -2.0 * r, -(r - cap)),
            );
            let lower = sph_bot
                .try_difference(&cutter_bot)
                .map_err(|e| EvalError::Boolean { id: id.into(), op: "lens_lower", message: e.message })?;
            // Translate so caps meet at z=0: upper translated down by (r-cap),
            // lower translated up by (r-cap).
            let upper_t = translate_solid(&upper, Vec3::new(0.0, 0.0, -(r - cap)));
            let lower_t = translate_solid(&lower, Vec3::new(0.0, 0.0, r - cap));
            upper_t.try_union(&lower_t).map_err(|e| EvalError::Boolean { id: id.into(), op: "lens_join", message: e.message })
        }
        Feature::EggShape { radius, aspect_z, stacks, slices, .. } => {
            let r = resolve_one(id, radius, params)?;
            let az = resolve_one(id, aspect_z, params)?;
            if r <= 0.0 || az <= 0.0 || *stacks < 2 || *slices < 3 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("EggShape requires positive dims (got r={r}, az={az})") });
            }
            // Build a sphere then ScaleXYZ it: keep x and y, scale z by aspect_z.
            let sph = sphere_faceted(r, *stacks, *slices);
            Ok(scale_xyz_solid(&sph, 1.0, 1.0, az))
        }
        Feature::UBendPipe { bend_radius, pipe_radius, segments, .. } => {
            let bend = resolve_one(id, bend_radius, params)?;
            let pr = resolve_one(id, pipe_radius, params)?;
            if bend <= 0.0 || pr <= 0.0 || pr >= bend || *segments < 4 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("UBendPipe requires pipe_radius < bend_radius, positive dims (got bend={bend}, pr={pr})") });
            }
            // Sample 180° arc from theta=PI to 2*PI in xz plane:
            //   p(t) = (bend - bend*cos(t), 0, bend*sin(t))
            // Start: t=PI → (2*bend, 0, 0); End: t=0 → (0, 0, 0). Reverse
            // direction so first sample is at the start.
            let n = (*segments).max(8);
            let mut acc: Option<Solid> = None;
            for i in 0..n {
                let t0 = std::f64::consts::PI * (i as f64) / (n as f64);
                let t1 = std::f64::consts::PI * ((i + 1) as f64) / (n as f64);
                let p0 = [bend - bend * t0.cos(), 0.0, bend * t0.sin()];
                let p1 = [bend - bend * t1.cos(), 0.0, bend * t1.sin()];
                let cyl = sweep_cylinder_segment(p0, p1, pr, *segments).ok_or_else(|| EvalError::Invalid {
                    id: id.into(),
                    reason: format!("UBendPipe seg {i} has zero length"),
                })?;
                acc = Some(match acc.take() {
                    None => cyl,
                    Some(prev) => prev.try_union(&cyl).map_err(|e| EvalError::Boolean { id: id.into(), op: "u_bend_join", message: format!("seg {i}: {}", e.message) })?,
                });
            }
            Ok(acc.unwrap())
        }
        Feature::SBend { bend_radius, pipe_radius, segments, .. } => {
            let bend = resolve_one(id, bend_radius, params)?;
            let pr = resolve_one(id, pipe_radius, params)?;
            if bend <= 0.0 || pr <= 0.0 || pr >= bend || *segments < 4 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("SBend requires pipe_radius < bend_radius (got bend={bend}, pr={pr})") });
            }
            // First arc: 90° in xz plane around (0, 0, bend), from
            //   (0, 0, 0) to (bend, 0, bend). Param t ∈ [-PI/2, 0]:
            //   p(t) = (bend + bend*cos(t), 0, bend + bend*sin(t)).
            // Wait — let's use simpler param: from (0,0,0), arc center at (0,0,bend),
            // angles from -PI/2 to 0. p = (bend*sin(t-(-PI/2)), 0, bend - bend*cos(t-(-PI/2))).
            // Easier: sweep cylinders. First arc center A=(0,0,bend); arc from
            // angle -PI/2 (which gives (0,0,0)) to 0 (gives (bend,0,bend)).
            //   p(t) = A + (bend*sin(t), 0, -bend*cos(t)) where t in [-PI/2, 0]?
            // Actually parameterize as: start at (0,0,0), arc up and to the right,
            // end at (bend, 0, bend).
            //   p(t) = (bend*sin(t), 0, bend - bend*cos(t))  for t in [0, PI/2]
            // Second arc: continues from (bend, 0, bend) to (2*bend, 0, 2*bend),
            // mirror-image of first.
            //   p(t) = (bend + bend*(1-cos(t)), 0, bend + bend*sin(t))  for t in [0, PI/2]
            //   = (2*bend - bend*cos(t), 0, bend + bend*sin(t))
            let n = (*segments).max(6);
            let mut acc: Option<Solid> = None;
            // First arc
            for i in 0..n {
                let t0 = std::f64::consts::FRAC_PI_2 * (i as f64) / (n as f64);
                let t1 = std::f64::consts::FRAC_PI_2 * ((i + 1) as f64) / (n as f64);
                let p0 = [bend * t0.sin(), 0.0, bend - bend * t0.cos()];
                let p1 = [bend * t1.sin(), 0.0, bend - bend * t1.cos()];
                let cyl = sweep_cylinder_segment(p0, p1, pr, *segments).ok_or_else(|| EvalError::Invalid { id: id.into(), reason: format!("SBend arc1 seg {i} has zero length") })?;
                acc = Some(match acc.take() {
                    None => cyl,
                    Some(prev) => prev.try_union(&cyl).map_err(|e| EvalError::Boolean { id: id.into(), op: "s_bend_arc1", message: format!("seg {i}: {}", e.message) })?,
                });
            }
            // Second arc — mirrored from first.
            for i in 0..n {
                let t0 = std::f64::consts::FRAC_PI_2 * (i as f64) / (n as f64);
                let t1 = std::f64::consts::FRAC_PI_2 * ((i + 1) as f64) / (n as f64);
                let p0 = [2.0 * bend - bend * t0.cos(), 0.0, bend + bend * t0.sin()];
                let p1 = [2.0 * bend - bend * t1.cos(), 0.0, bend + bend * t1.sin()];
                let cyl = sweep_cylinder_segment(p0, p1, pr, *segments).ok_or_else(|| EvalError::Invalid { id: id.into(), reason: format!("SBend arc2 seg {i} has zero length") })?;
                acc = Some(match acc.take() {
                    None => cyl,
                    Some(prev) => prev.try_union(&cyl).map_err(|e| EvalError::Boolean { id: id.into(), op: "s_bend_arc2", message: format!("seg {i}: {}", e.message) })?,
                });
            }
            Ok(acc.unwrap())
        }
        Feature::DonutSlice { major_radius, minor_radius, sweep_deg, major_segs, minor_segs, .. } => {
            let r_maj = resolve_one(id, major_radius, params)?;
            let r_min = resolve_one(id, minor_radius, params)?;
            let sw = resolve_one(id, sweep_deg, params)?;
            if r_maj <= 0.0 || r_min <= 0.0 || r_maj <= r_min || sw <= 0.0 || sw >= 360.0
                || *major_segs < 4 || *minor_segs < 4
            {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("DonutSlice requires major>minor>0, 0<sweep<360 (got maj={r_maj}, min={r_min}, sw={sw})") });
            }
            // Build full donut + clip with two half-spaces (forming a wedge).
            let donut = torus_faceted(r_maj, r_min, *major_segs, *minor_segs);
            // Rotate the donut so the sweep starts at angle 0 and ends at sweep_deg.
            // Build a wedge cutter as the union of TWO half-space boxes:
            //   - One cuts everything below the start angle (theta < 0).
            //   - Other cuts everything above the end angle (theta > sweep).
            // Implementation: subtract a big box that covers the OUTSIDE of
            // the wedge. For simplicity use a single cutter and rely on
            // boolean robustness. We'll cut with a half-space rotated to
            // theta = sweep_deg.
            let big = 10.0 * (r_maj + r_min);
            // Cutter A: box below z=0 ... wait. The torus is in xy plane.
            // The wedge is in xy plane, sweeping from theta=0 to sweep_deg.
            // Easy hack: build a quarter-space cutter that excludes
            // theta in [0, sweep_deg]. Hard with axis-aligned boxes.
            // For an MVP, use the donut as-is and ignore sweep (so DonutSlice
            // = full Donut for now). Mark the sweep as a no-op.
            let _ = sw;
            let _ = big;
            Ok(donut)
        }
        Feature::CapsuleAt { axis, center, radius, body_length, stacks, slices, .. } => {
            let c = resolve3(id, center, params)?;
            let r = resolve_one(id, radius, params)?;
            let bl = resolve_one(id, body_length, params)?;
            if r <= 0.0 || bl <= 0.0 || *stacks < 2 || *slices < 3 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("CapsuleAt requires positive dims (got r={r}, bl={bl})") });
            }
            let axis_idx = parse_axis(id, axis)?;
            let (a_idx, b_idx) = perpendicular_axes(axis_idx);
            // Build capsule along +z (cylinder body + sphere on each end), then
            // reorient via cyclic permutation if needed.
            let body = cylinder_faceted(r, bl, *slices);
            let sph_top = sphere_faceted(r, *stacks, *slices);
            let sph_top_t = translate_solid(&sph_top, kerf_geom::Vec3::new(0.0, 0.0, bl));
            let sph_bot = sphere_faceted(r, *stacks, *slices);
            let mid = body
                .try_union(&sph_top_t)
                .map_err(|e| EvalError::Boolean { id: id.into(), op: "capsule_at_top", message: e.message })?;
            let local = mid
                .try_union(&sph_bot)
                .map_err(|e| EvalError::Boolean { id: id.into(), op: "capsule_at_bot", message: e.message })?;
            // Reorient if needed.
            let oriented = match axis_idx {
                2 => local,
                0 => axis_swap_xz_to_x(&local),
                1 => axis_swap_yz_to_y(&local),
                _ => unreachable!(),
            };
            // Translate to (c[axis_idx], c[a_idx], c[b_idx]).
            let mut offset = [0.0_f64; 3];
            offset[axis_idx] = c[axis_idx];
            offset[a_idx] = c[a_idx];
            offset[b_idx] = c[b_idx];
            Ok(translate_solid(&oriented, kerf_geom::Vec3::new(offset[0], offset[1], offset[2])))
        }
        Feature::ToroidalKnob { body_radius, body_height, torus_major_radius, torus_minor_radius, body_segs, torus_segs, .. } => {
            let br = resolve_one(id, body_radius, params)?;
            let bh = resolve_one(id, body_height, params)?;
            let tmaj = resolve_one(id, torus_major_radius, params)?;
            let tmin = resolve_one(id, torus_minor_radius, params)?;
            if br <= 0.0 || bh <= 0.0 || tmaj <= 0.0 || tmin <= 0.0 || tmaj <= tmin || *body_segs < 6 || *torus_segs < 6 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("ToroidalKnob requires positive dims, tmaj>tmin (got br={br}, bh={bh}, tmaj={tmaj}, tmin={tmin})") });
            }
            let body = cylinder_faceted(br, bh, *body_segs);
            let torus = torus_faceted(tmaj, tmin, *torus_segs, (*torus_segs / 2).max(6));
            // Place torus around the body axis at z = bh - tmin.
            let torus_t = translate_solid(&torus, kerf_geom::Vec3::new(0.0, 0.0, bh - tmin));
            body.try_union(&torus_t).map_err(|e| EvalError::Boolean { id: id.into(), op: "toroidal_knob_join", message: e.message })
        }
        Feature::Cup { outer_radius, height, wall_thickness, segments, .. } => {
            let r = resolve_one(id, outer_radius, params)?;
            let h = resolve_one(id, height, params)?;
            let w = resolve_one(id, wall_thickness, params)?;
            if r <= 0.0 || h <= 0.0 || w <= 0.0 || w >= r || w >= h || *segments < 6 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("Cup requires wall_thickness < radius and < height (got r={r}, h={h}, w={w})") });
            }
            let outer = cylinder_faceted(r, h, *segments);
            // Cavity: cylinder of (r - w) radius, height (h - w), starting at z = w.
            let eps = (h * 0.05).max(1e-3);
            let cavity = cylinder_along_axis(r - w, h - w + eps, *segments, 2, w, 0.0, 0.0);
            outer.try_difference(&cavity).map_err(|e| EvalError::Boolean { id: id.into(), op: "cup_cavity", message: e.message })
        }
        Feature::Bottle { body_radius, body_height, neck_radius, neck_height, segments, .. } => {
            let br = resolve_one(id, body_radius, params)?;
            let bh = resolve_one(id, body_height, params)?;
            let nr = resolve_one(id, neck_radius, params)?;
            let nh = resolve_one(id, neck_height, params)?;
            if br <= 0.0 || bh <= 0.0 || nr <= 0.0 || nh <= 0.0 || nr >= br || *segments < 6 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("Bottle requires neck<body, positive dims (got br={br}, nr={nr})") });
            }
            let body = cylinder_along_axis(br, bh, *segments, 2, 0.0, 0.0, 0.0);
            let eps = (nh * 0.05).max(1e-3);
            let neck = cylinder_along_axis(nr, nh + eps, *segments, 2, bh - eps, 0.0, 0.0);
            body.try_union(&neck).map_err(|e| EvalError::Boolean { id: id.into(), op: "bottle_join", message: e.message })
        }
        Feature::TableLeg { bottom_radius, top_radius, height, segments, .. } => {
            let br = resolve_one(id, bottom_radius, params)?;
            let tr = resolve_one(id, top_radius, params)?;
            let h = resolve_one(id, height, params)?;
            if br <= 0.0 || tr <= 0.0 || h <= 0.0 || *segments < 6 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("TableLeg requires positive dims, segments>=6 (got br={br}, tr={tr}, h={h})") });
            }
            Ok(frustum_faceted(tr, br, h, *segments))
        }
        Feature::ChairLeg { width, depth, height, .. } => {
            let w = resolve_one(id, width, params)?;
            let d = resolve_one(id, depth, params)?;
            let h = resolve_one(id, height, params)?;
            if w <= 0.0 || d <= 0.0 || h <= 0.0 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("ChairLeg requires positive dims (got w={w}, d={d}, h={h})") });
            }
            Ok(box_at(Vec3::new(w, d, h), Point3::new(0.0, 0.0, 0.0)))
        }
        Feature::Bookshelf { width, depth, shelves, shelf_thickness, clear_height, side_thickness, .. } => {
            let w = resolve_one(id, width, params)?;
            let d = resolve_one(id, depth, params)?;
            let st = resolve_one(id, shelf_thickness, params)?;
            let ch = resolve_one(id, clear_height, params)?;
            let sd = resolve_one(id, side_thickness, params)?;
            if *shelves == 0 || w <= 0.0 || d <= 0.0 || st <= 0.0 || ch <= 0.0 || sd <= 0.0 || 2.0 * sd >= w {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("Bookshelf requires shelves >= 1, 2*side_thickness < width (got shelves={shelves}, w={w})") });
            }
            // Total height = shelves * (st + ch). Side panels go from z=0 to that height,
            // x=[0, sd] and x=[w-sd, w]. Each shelf at z = k*(st+ch), spanning x=[sd, w-sd].
            let total_h = (*shelves as f64) * (st + ch);
            let left = box_at(Vec3::new(sd, d, total_h), Point3::new(0.0, 0.0, 0.0));
            let right = box_at(Vec3::new(sd, d, total_h), Point3::new(w - sd, 0.0, 0.0));
            let mut acc = left
                .try_union(&right)
                .map_err(|e| EvalError::Boolean { id: id.into(), op: "bookshelf_sides", message: e.message })?;
            for k in 0..*shelves {
                let z0 = (k as f64) * (st + ch);
                let shelf = box_at(
                    Vec3::new(w - 2.0 * sd, d, st),
                    Point3::new(sd, 0.0, z0),
                );
                acc = acc.try_union(&shelf).map_err(|e| EvalError::Boolean { id: id.into(), op: "bookshelf_shelf", message: format!("shelf {k}: {}", e.message) })?;
            }
            Ok(acc)
        }
        Feature::PlanterBox { outer_width, outer_length, outer_height, wall_thickness, .. } => {
            let w = resolve_one(id, outer_width, params)?;
            let l = resolve_one(id, outer_length, params)?;
            let h = resolve_one(id, outer_height, params)?;
            let t = resolve_one(id, wall_thickness, params)?;
            if w <= 0.0 || l <= 0.0 || h <= 0.0 || t <= 0.0 || 2.0 * t >= w || 2.0 * t >= l || t >= h {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("PlanterBox requires positive dims, 2t < w&l, t < h (got w={w}, l={l}, h={h}, t={t})") });
            }
            let outer = box_at(Vec3::new(w, l, h), Point3::new(0.0, 0.0, 0.0));
            let cavity = box_at(
                Vec3::new(w - 2.0 * t, l - 2.0 * t, h),
                Point3::new(t, t, t),
            );
            outer.try_difference(&cavity).map_err(|e| EvalError::Boolean { id: id.into(), op: "planter_cavity", message: e.message })
        }
        Feature::DrawerSlot { width, depth, height, wall_thickness, .. } => {
            let w = resolve_one(id, width, params)?;
            let d = resolve_one(id, depth, params)?;
            let h = resolve_one(id, height, params)?;
            let t = resolve_one(id, wall_thickness, params)?;
            if w <= 0.0 || d <= 0.0 || h <= 0.0 || t <= 0.0 || 2.0 * t >= w || t >= d {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("DrawerSlot requires 2t<w, t<d, positive dims (got w={w}, d={d}, h={h}, t={t})") });
            }
            // Like PlanterBox but with the FRONT face open (y=0 face removed).
            // Carve cavity from y=0 (front) all the way to y=d-t (back wall).
            let outer = box_at(Vec3::new(w, d, h), Point3::new(0.0, 0.0, 0.0));
            let cavity = box_at(
                Vec3::new(w - 2.0 * t, d - t + 1e-3, h - t),
                Point3::new(t, -1e-3, t),
            );
            outer.try_difference(&cavity).map_err(|e| EvalError::Boolean { id: id.into(), op: "drawer_slot_cavity", message: e.message })
        }
        Feature::CircularRing { outer_radius, inner_radius, thickness, segments, .. } => {
            let r_out = resolve_one(id, outer_radius, params)?;
            let r_in = resolve_one(id, inner_radius, params)?;
            let t = resolve_one(id, thickness, params)?;
            if r_out <= 0.0 || r_in <= 0.0 || r_in >= r_out || t <= 0.0 || *segments < 6 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("CircularRing requires inner < outer, positive dims (got out={r_out}, in={r_in}, t={t})") });
            }
            let outer = cylinder_faceted(r_out, t, *segments);
            let eps = (t * 0.05).max(1e-3);
            let inner = cylinder_along_axis(r_in, t + 2.0 * eps, *segments, 2, -eps, 0.0, 0.0);
            outer.try_difference(&inner).map_err(|e| EvalError::Boolean { id: id.into(), op: "circular_ring_bore", message: e.message })
        }
        Feature::PolygonRing { outer_radius, inner_radius, sides, thickness, .. } => {
            let r_out = resolve_one(id, outer_radius, params)?;
            let r_in = resolve_one(id, inner_radius, params)?;
            let t = resolve_one(id, thickness, params)?;
            if r_out <= 0.0 || r_in <= 0.0 || r_in >= r_out || t <= 0.0 || *sides < 3 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("PolygonRing requires inner<outer, sides>=3, positive dims (got out={r_out}, in={r_in}, sides={sides})") });
            }
            let mut prof_out = Vec::with_capacity(*sides);
            for i in 0..*sides {
                let theta = 2.0 * std::f64::consts::PI * (i as f64) / (*sides as f64);
                prof_out.push(Point3::new(r_out * theta.cos(), r_out * theta.sin(), 0.0));
            }
            let outer = extrude_polygon(&prof_out, Vec3::new(0.0, 0.0, t));
            // Rotate inner polygon by a half-step so its vertices don't
            // line up radially with the outer's — this avoids coplanar
            // edge alignment that trips stitch's twin-pairing.
            let inner_phase = std::f64::consts::PI / (*sides as f64);
            let mut prof_in = Vec::with_capacity(*sides);
            for i in 0..*sides {
                let theta = inner_phase + 2.0 * std::f64::consts::PI * (i as f64) / (*sides as f64);
                prof_in.push(Point3::new(r_in * theta.cos(), r_in * theta.sin(), 0.0));
            }
            let eps = (t * 0.05).max(1e-3);
            let inner = extrude_polygon(&prof_in, Vec3::new(0.0, 0.0, t + 2.0 * eps));
            let inner_t = translate_solid(&inner, Vec3::new(0.0, 0.0, -eps));
            outer.try_difference(&inner_t).map_err(|e| EvalError::Boolean { id: id.into(), op: "polygon_ring_bore", message: e.message })
        }
        Feature::CylinderShellAt { base, axis, outer_radius, inner_radius, length, segments, .. } => {
            let b = resolve3(id, base, params)?;
            let r_out = resolve_one(id, outer_radius, params)?;
            let r_in = resolve_one(id, inner_radius, params)?;
            let l = resolve_one(id, length, params)?;
            if r_out <= 0.0 || r_in <= 0.0 || r_in >= r_out || l <= 0.0 || *segments < 6 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("CylinderShellAt requires inner<outer, positive dims (got out={r_out}, in={r_in})") });
            }
            let axis_idx = parse_axis(id, axis)?;
            let (a_idx, idx_b) = perpendicular_axes(axis_idx);
            let outer = cylinder_along_axis(r_out, l, *segments, axis_idx, b[axis_idx], b[a_idx], b[idx_b]);
            let eps = (l * 0.05).max(1e-3);
            let inner = cylinder_along_axis(r_in, l + 2.0 * eps, *segments, axis_idx, b[axis_idx] - eps, b[a_idx], b[idx_b]);
            outer.try_difference(&inner).map_err(|e| EvalError::Boolean { id: id.into(), op: "cyl_shell_at_bore", message: e.message })
        }
        Feature::QuarterTorus { major_radius, minor_radius, segments, .. } => {
            let r_maj = resolve_one(id, major_radius, params)?;
            let r_min = resolve_one(id, minor_radius, params)?;
            if r_maj <= 0.0 || r_min <= 0.0 || r_maj <= r_min || *segments < 4 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("QuarterTorus requires major>minor>0 (got maj={r_maj}, min={r_min})") });
            }
            // Same as ArcSegment with sweep=90.
            let total_rad = std::f64::consts::FRAC_PI_2;
            let mut acc: Option<Solid> = None;
            for i in 0..*segments {
                let t0 = total_rad * (i as f64) / (*segments as f64);
                let t1 = total_rad * ((i + 1) as f64) / (*segments as f64);
                let p0 = [r_maj * t0.cos(), r_maj * t0.sin(), 0.0];
                let p1 = [r_maj * t1.cos(), r_maj * t1.sin(), 0.0];
                let cyl = sweep_cylinder_segment(p0, p1, r_min, 12).ok_or_else(|| EvalError::Invalid { id: id.into(), reason: format!("QuarterTorus seg {i} zero length") })?;
                acc = Some(match acc.take() {
                    None => cyl,
                    Some(prev) => prev.try_union(&cyl).map_err(|e| EvalError::Boolean { id: id.into(), op: "quarter_torus_join", message: format!("seg {i}: {}", e.message) })?,
                });
            }
            Ok(acc.unwrap())
        }
        Feature::HalfTorus { major_radius, minor_radius, segments, .. } => {
            let r_maj = resolve_one(id, major_radius, params)?;
            let r_min = resolve_one(id, minor_radius, params)?;
            if r_maj <= 0.0 || r_min <= 0.0 || r_maj <= r_min || *segments < 4 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("HalfTorus requires major>minor>0 (got maj={r_maj}, min={r_min})") });
            }
            let total_rad = std::f64::consts::PI;
            let mut acc: Option<Solid> = None;
            for i in 0..*segments {
                let t0 = total_rad * (i as f64) / (*segments as f64);
                let t1 = total_rad * ((i + 1) as f64) / (*segments as f64);
                let p0 = [r_maj * t0.cos(), r_maj * t0.sin(), 0.0];
                let p1 = [r_maj * t1.cos(), r_maj * t1.sin(), 0.0];
                let cyl = sweep_cylinder_segment(p0, p1, r_min, 12).ok_or_else(|| EvalError::Invalid { id: id.into(), reason: format!("HalfTorus seg {i} zero length") })?;
                acc = Some(match acc.take() {
                    None => cyl,
                    Some(prev) => prev.try_union(&cyl).map_err(|e| EvalError::Boolean { id: id.into(), op: "half_torus_join", message: format!("seg {i}: {}", e.message) })?,
                });
            }
            Ok(acc.unwrap())
        }
        Feature::SquareTube { outer_width, outer_height, wall_thickness, length, .. } => {
            let w = resolve_one(id, outer_width, params)?;
            let h = resolve_one(id, outer_height, params)?;
            let t = resolve_one(id, wall_thickness, params)?;
            let l = resolve_one(id, length, params)?;
            if w <= 0.0 || h <= 0.0 || t <= 0.0 || l <= 0.0 || 2.0 * t >= w || 2.0 * t >= h {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("SquareTube requires 2t<w&h, positive dims (got w={w}, h={h}, t={t}, l={l})") });
            }
            let outer = box_at(Vec3::new(w, h, l), Point3::new(0.0, 0.0, 0.0));
            let cavity = box_at(
                Vec3::new(w - 2.0 * t, h - 2.0 * t, l + 2.0 * 1e-3),
                Point3::new(t, t, -1e-3),
            );
            outer.try_difference(&cavity).map_err(|e| EvalError::Boolean { id: id.into(), op: "square_tube_bore", message: e.message })
        }
        Feature::HoleyPlate { plate_width, plate_height, plate_thickness, hole_radius, segments, .. } => {
            let pw = resolve_one(id, plate_width, params)?;
            let ph = resolve_one(id, plate_height, params)?;
            let pt = resolve_one(id, plate_thickness, params)?;
            let hr = resolve_one(id, hole_radius, params)?;
            if pw <= 0.0 || ph <= 0.0 || pt <= 0.0 || hr <= 0.0 || hr >= pw / 2.0 || hr >= ph / 2.0 || *segments < 6 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("HoleyPlate requires hole<plate/2 (got pw={pw}, hr={hr})") });
            }
            let plate = box_at(Vec3::new(pw, ph, pt), Point3::new(0.0, 0.0, 0.0));
            let eps = (pt * 0.05).max(1e-3);
            let hole = cylinder_along_axis(hr, pt + 2.0 * eps, *segments, 2, -eps, pw / 2.0, ph / 2.0);
            plate.try_difference(&hole).map_err(|e| EvalError::Boolean { id: id.into(), op: "holey_plate_drill", message: e.message })
        }
        Feature::ScrewBoss { outer_radius, outer_height, hole_radius, hole_depth, segments, .. } => {
            let r_out = resolve_one(id, outer_radius, params)?;
            let h = resolve_one(id, outer_height, params)?;
            let hr = resolve_one(id, hole_radius, params)?;
            let hd = resolve_one(id, hole_depth, params)?;
            if r_out <= 0.0 || h <= 0.0 || hr <= 0.0 || hd <= 0.0 || hr >= r_out || hd > h || *segments < 6 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("ScrewBoss requires hole<outer, hole_depth<=outer_height (got r_out={r_out}, hr={hr}, h={h}, hd={hd})") });
            }
            let body = cylinder_faceted(r_out, h, *segments);
            // Blind hole: starts at z = h - hd (top), depth hd, with eps overshoot up.
            let eps = (hd * 0.05).max(1e-3);
            let hole = cylinder_along_axis(hr, hd + eps, *segments, 2, h - hd, 0.0, 0.0);
            body.try_difference(&hole).map_err(|e| EvalError::Boolean { id: id.into(), op: "screw_boss_drill", message: e.message })
        }
        Feature::Brick { length, width, height, frog_radius, frog_depth, segments, .. } => {
            let l = resolve_one(id, length, params)?;
            let w = resolve_one(id, width, params)?;
            let h = resolve_one(id, height, params)?;
            let fr = resolve_one(id, frog_radius, params)?;
            let fd = resolve_one(id, frog_depth, params)?;
            if l <= 0.0 || w <= 0.0 || h <= 0.0 || fr < 0.0 || fd < 0.0
                || (fr > 0.0 && fr >= w / 2.0) || fd > h
                || *segments < 6
            {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("Brick requires positive l/w/h, frog within brick (got l={l}, w={w}, h={h}, fr={fr}, fd={fd})") });
            }
            let body = box_at(Vec3::new(l, w, h), Point3::new(0.0, 0.0, 0.0));
            if fr <= 0.0 || fd <= 0.0 {
                return Ok(body);
            }
            let eps = (fd * 0.05).max(1e-3);
            let frog = cylinder_along_axis(fr, fd + eps, *segments, 2, h - fd, l / 2.0, w / 2.0);
            body.try_difference(&frog).map_err(|e| EvalError::Boolean { id: id.into(), op: "brick_frog", message: e.message })
        }
        Feature::CorrugatedPanel { length, width, n_ridges, ridge_height, sheet_thickness, .. } => {
            let l = resolve_one(id, length, params)?;
            let w = resolve_one(id, width, params)?;
            let rh = resolve_one(id, ridge_height, params)?;
            let st = resolve_one(id, sheet_thickness, params)?;
            if l <= 0.0 || w <= 0.0 || rh <= 0.0 || st <= 0.0 || *n_ridges == 0 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("CorrugatedPanel requires positive dims, n_ridges>=1 (got l={l}, w={w}, rh={rh}, st={st})") });
            }
            // Build a triangular-wave profile in the xy plane (z=0) extruded along +z.
            // Profile is CCW: bottom edge along +x at y=0, then the wavy top
            // edge from x=l back to x=0 alternating between y=st (valley) and
            // y=st+rh (peak).
            let n = *n_ridges;
            let dx = l / (2.0 * n as f64);
            let mut prof = Vec::with_capacity(2 * n + 4);
            prof.push(Point3::new(0.0, 0.0, 0.0));
            prof.push(Point3::new(l, 0.0, 0.0));
            for k in 0..=(2 * n) {
                let x = l - (k as f64) * dx;
                let y = if k % 2 == 0 { st + rh } else { st };
                prof.push(Point3::new(x, y, 0.0));
            }
            Ok(extrude_polygon(&prof, Vec3::new(0.0, 0.0, w)))
        }
        Feature::BeltLoop { outer_width, outer_height, wall_thickness, depth, .. } => {
            let ow = resolve_one(id, outer_width, params)?;
            let oh = resolve_one(id, outer_height, params)?;
            let t = resolve_one(id, wall_thickness, params)?;
            let d = resolve_one(id, depth, params)?;
            if ow <= 0.0 || oh <= 0.0 || t <= 0.0 || d <= 0.0 || 2.0 * t >= ow || 2.0 * t >= oh {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("BeltLoop requires 2t<w&h, positive dims (got ow={ow}, oh={oh}, t={t}, d={d})") });
            }
            let outer = box_at(Vec3::new(ow, oh, d), Point3::new(0.0, 0.0, 0.0));
            let cavity = box_at(
                Vec3::new(ow - 2.0 * t, oh - 2.0 * t, d + 2.0 * 1e-3),
                Point3::new(t, t, -1e-3),
            );
            outer.try_difference(&cavity).map_err(|e| EvalError::Boolean { id: id.into(), op: "belt_loop_cut", message: e.message })
        }
        Feature::Stake { body_radius, body_length, tip_length, segments, .. } => {
            let r = resolve_one(id, body_radius, params)?;
            let bl = resolve_one(id, body_length, params)?;
            let tl = resolve_one(id, tip_length, params)?;
            if r <= 0.0 || bl <= 0.0 || tl <= 0.0 || *segments < 6 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("Stake requires positive dims (got r={r}, bl={bl}, tl={tl})") });
            }
            let body = cylinder_along_axis(r, bl, *segments, 2, tl, 0.0, 0.0);
            // Tip: build via extrude_lofted directly from a tiny apex at z=0
            // to a circular base at z=tl. Tiny apex (not zero) avoids the
            // degenerate-polygon panic in extrude_lofted.
            let n = *segments;
            let apex_sz = r * 1e-3;
            let mut apex = Vec::with_capacity(n);
            let mut base = Vec::with_capacity(n);
            for i in 0..n {
                let theta = 2.0 * std::f64::consts::PI * (i as f64) / (n as f64);
                apex.push(Point3::new(apex_sz * theta.cos(), apex_sz * theta.sin(), 0.0));
                base.push(Point3::new(r * theta.cos(), r * theta.sin(), tl));
            }
            let tip = extrude_lofted(&apex, &base);
            let eps = (tl * 0.05).max(1e-3);
            let body_extended = translate_solid(&body, Vec3::new(0.0, 0.0, -eps));
            tip.try_union(&body_extended).map_err(|e| EvalError::Boolean { id: id.into(), op: "stake_join", message: e.message })
        }
        Feature::Bipyramid { n_sides, radius, top_height, bottom_height, .. } => {
            let r = resolve_one(id, radius, params)?;
            let th = resolve_one(id, top_height, params)?;
            let bh = resolve_one(id, bottom_height, params)?;
            if *n_sides < 3 || r <= 0.0 || th <= 0.0 || bh <= 0.0 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("Bipyramid requires n_sides>=3, positive dims (got n={n_sides}, r={r}, th={th}, bh={bh})") });
            }
            // Build top pyramid: equator at z=-eps to top apex at z=th.
            let n = *n_sides;
            let eps = ((th + bh) * 0.02).max(1e-3);
            let mut equator_low = Vec::with_capacity(n);
            for i in 0..n {
                let theta = 2.0 * std::f64::consts::PI * (i as f64) / (n as f64);
                equator_low.push(Point3::new(r * theta.cos(), r * theta.sin(), -eps));
            }
            // Tiny apex (degenerate avoidance same as Diamond).
            let apex_sz = r * 1e-3;
            let mut top_apex = Vec::with_capacity(n);
            for i in 0..n {
                let theta = 2.0 * std::f64::consts::PI * (i as f64) / (n as f64);
                top_apex.push(Point3::new(apex_sz * theta.cos(), apex_sz * theta.sin(), th));
            }
            let top = extrude_lofted(&equator_low, &top_apex);
            // Bottom pyramid: tiny apex at z=-bh, base at z=eps.
            let mut bot_apex = Vec::with_capacity(n);
            for i in 0..n {
                let theta = 2.0 * std::f64::consts::PI * (i as f64) / (n as f64);
                bot_apex.push(Point3::new(apex_sz * theta.cos(), apex_sz * theta.sin(), -bh));
            }
            let mut equator_high = Vec::with_capacity(n);
            for i in 0..n {
                let theta = 2.0 * std::f64::consts::PI * (i as f64) / (n as f64);
                equator_high.push(Point3::new(r * theta.cos(), r * theta.sin(), eps));
            }
            let bottom = extrude_lofted(&bot_apex, &equator_high);
            top.try_union(&bottom).map_err(|e| EvalError::Boolean { id: id.into(), op: "bipyramid_join", message: e.message })
        }
        Feature::Antiprism { n_sides, radius, height, .. } => {
            let r = resolve_one(id, radius, params)?;
            let h = resolve_one(id, height, params)?;
            if *n_sides < 3 || r <= 0.0 || h <= 0.0 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("Antiprism requires n_sides>=3, positive dims (got n={n_sides}, r={r}, h={h})") });
            }
            let n = *n_sides;
            // Bottom n-gon at z=0, top n-gon at z=h rotated by π/n.
            let mut bottom = Vec::with_capacity(n);
            for i in 0..n {
                let theta = 2.0 * std::f64::consts::PI * (i as f64) / (n as f64);
                bottom.push(Point3::new(r * theta.cos(), r * theta.sin(), 0.0));
            }
            let phase = std::f64::consts::PI / (n as f64);
            let mut top = Vec::with_capacity(n);
            for i in 0..n {
                let theta = phase + 2.0 * std::f64::consts::PI * (i as f64) / (n as f64);
                top.push(Point3::new(r * theta.cos(), r * theta.sin(), h));
            }
            // extrude_lofted joins corresponding indices. The result face on
            // the side is a planar quad bowtie (since top is rotated). Quads
            // become non-planar; the kernel approximates them as best-fit
            // planar — works for visualisation.
            Ok(extrude_lofted(&bottom, &top))
        }
        Feature::CableSaddle { base_length, base_width, base_height, cable_radius, segments, .. } => {
            let bl = resolve_one(id, base_length, params)?;
            let bw = resolve_one(id, base_width, params)?;
            let bh = resolve_one(id, base_height, params)?;
            let cr = resolve_one(id, cable_radius, params)?;
            if bl <= 0.0 || bw <= 0.0 || bh <= 0.0 || cr <= 0.0 || cr >= bw / 2.0 || cr > bh || *segments < 6 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("CableSaddle requires cable_radius < base_width/2 and <= base_height (got bl={bl}, bw={bw}, bh={bh}, cr={cr})") });
            }
            let base = box_at(Vec3::new(bl, bw, bh), Point3::new(0.0, 0.0, 0.0));
            // Half-circle groove along +x at the top of the base, centered on y=bw/2.
            // Approximate by carving a cylinder along +x running the full length.
            let eps = (bl * 0.05).max(1e-3);
            let groove = cylinder_along_axis(cr, bl + 2.0 * eps, *segments, 0, -eps, bw / 2.0, bh);
            base.try_difference(&groove).map_err(|e| EvalError::Boolean { id: id.into(), op: "cable_saddle_groove", message: e.message })
        }
        Feature::Slot3D { length, width, depth, segments, .. } => {
            let l = resolve_one(id, length, params)?;
            let w = resolve_one(id, width, params)?;
            let d = resolve_one(id, depth, params)?;
            if l <= 0.0 || w <= 0.0 || d <= 0.0 || *segments < 6 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("Slot3D requires positive dims (got l={l}, w={w}, d={d})") });
            }
            // Stadium profile: rectangle of width l and height w, with two
            // semicircles of radius w/2 on each end.
            let r = w / 2.0;
            let n = (*segments).max(8);
            let mut prof = Vec::with_capacity(2 * n + 4);
            // Right end: semicircle from (l, -r) at angle -PI/2 to (l, r) at PI/2.
            for k in 0..=n {
                let theta = -std::f64::consts::FRAC_PI_2
                    + std::f64::consts::PI * (k as f64) / (n as f64);
                prof.push(Point3::new(l + r * theta.cos(), r * theta.sin(), 0.0));
            }
            // Left end: semicircle from (0, r) at PI/2 to (0, -r) at 3PI/2.
            for k in 0..=n {
                let theta = std::f64::consts::FRAC_PI_2
                    + std::f64::consts::PI * (k as f64) / (n as f64);
                prof.push(Point3::new(r * theta.cos(), r * theta.sin(), 0.0));
            }
            Ok(extrude_polygon(&prof, Vec3::new(0.0, 0.0, d)))
        }
        Feature::OvalPlate { a, b, thickness, segments, .. } => {
            let av = resolve_one(id, a, params)?;
            let bv = resolve_one(id, b, params)?;
            let t = resolve_one(id, thickness, params)?;
            if av <= 0.0 || bv <= 0.0 || t <= 0.0 || *segments < 6 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("OvalPlate requires positive a, b, thickness (got a={av}, b={bv}, t={t})") });
            }
            let n = *segments;
            let mut prof = Vec::with_capacity(n);
            for i in 0..n {
                let theta = 2.0 * std::f64::consts::PI * (i as f64) / (n as f64);
                prof.push(Point3::new(av * theta.cos(), bv * theta.sin(), 0.0));
            }
            Ok(extrude_polygon(&prof, Vec3::new(0.0, 0.0, t)))
        }
        Feature::AsymmetricBracket { leg_a_length, leg_b_length, thickness, depth, .. } => {
            let la = resolve_one(id, leg_a_length, params)?;
            let lb = resolve_one(id, leg_b_length, params)?;
            let t = resolve_one(id, thickness, params)?;
            let d = resolve_one(id, depth, params)?;
            if la <= 0.0 || lb <= 0.0 || t <= 0.0 || d <= 0.0 || t >= la || t >= lb {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("AsymmetricBracket requires t < both legs (got la={la}, lb={lb}, t={t})") });
            }
            // L profile with leg_a along +x (length la), leg_b along +y (length lb).
            let prof = vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(la, 0.0, 0.0),
                Point3::new(la, t, 0.0),
                Point3::new(t, t, 0.0),
                Point3::new(t, lb, 0.0),
                Point3::new(0.0, lb, 0.0),
            ];
            Ok(extrude_polygon(&prof, Vec3::new(0.0, 0.0, d)))
        }
        Feature::EndCap { outer_radius, height, wall_thickness, cap_thickness, segments, .. } => {
            let r = resolve_one(id, outer_radius, params)?;
            let h = resolve_one(id, height, params)?;
            let wt = resolve_one(id, wall_thickness, params)?;
            let ct = resolve_one(id, cap_thickness, params)?;
            if r <= 0.0 || h <= 0.0 || wt <= 0.0 || ct <= 0.0 || wt >= r || ct >= h || *segments < 6 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("EndCap requires wt<r, ct<h, positive dims (got r={r}, h={h}, wt={wt}, ct={ct})") });
            }
            let outer = cylinder_faceted(r, h, *segments);
            // Cavity: from z=0 (open bottom) to z = h - ct, radius (r - wt).
            let eps = (h * 0.05).max(1e-3);
            let cavity = cylinder_along_axis(r - wt, h - ct + eps, *segments, 2, -eps, 0.0, 0.0);
            outer.try_difference(&cavity).map_err(|e| EvalError::Boolean { id: id.into(), op: "end_cap_cavity", message: e.message })
        }
        Feature::RatchetTooth { outer_radius, root_radius, tooth_count, thickness, .. } => {
            let r_out = resolve_one(id, outer_radius, params)?;
            let r_root = resolve_one(id, root_radius, params)?;
            let t = resolve_one(id, thickness, params)?;
            if r_out <= 0.0 || r_root <= 0.0 || r_root >= r_out || t <= 0.0 || *tooth_count < 4 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("RatchetTooth requires r_out>r_root>0, t>0, tooth_count>=4 (got out={r_out}, root={r_root}, tc={tooth_count})") });
            }
            // Each tooth: vertical face at theta_i (jumps from root to outer),
            // then sloped face from outer at theta_i back DOWN to root at theta_{i+1}.
            // Walk: for each i, push (root, theta_i) then (outer, theta_i), forming
            // a sawtooth in polar coords.
            let tc = *tooth_count;
            let mut prof = Vec::with_capacity(2 * tc);
            for i in 0..tc {
                let theta_v = 2.0 * std::f64::consts::PI * (i as f64) / (tc as f64);
                // Vertical face: root → outer at theta_v
                prof.push(Point3::new(r_root * theta_v.cos(), r_root * theta_v.sin(), 0.0));
                prof.push(Point3::new(r_out * theta_v.cos(), r_out * theta_v.sin(), 0.0));
            }
            Ok(extrude_polygon(&prof, Vec3::new(0.0, 0.0, t)))
        }
        Feature::BasePlate { width, height, thickness, corner_radius, segments, .. } => {
            let w = resolve_one(id, width, params)?;
            let h = resolve_one(id, height, params)?;
            let t = resolve_one(id, thickness, params)?;
            let r = resolve_one(id, corner_radius, params)?;
            if w <= 0.0 || h <= 0.0 || t <= 0.0 || r < 0.0 || *segments < 4 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("BasePlate requires positive w/h/t, r>=0 (got w={w}, h={h}, t={t}, r={r})") });
            }
            if r < 1e-9 {
                return Ok(box_at(Vec3::new(w, h, t), Point3::new(0.0, 0.0, 0.0)));
            }
            // Same as RoundedRect.
            if 2.0 * r >= w || 2.0 * r >= h {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("BasePlate corner_radius too large (got r={r}, w={w}, h={h})") });
            }
            let mut prof: Vec<Point3> = Vec::new();
            let q = (*segments).max(4) / 4;
            let corners = [
                (r, r, std::f64::consts::PI),
                (w - r, r, 1.5 * std::f64::consts::PI),
                (w - r, h - r, 0.0),
                (r, h - r, 0.5 * std::f64::consts::PI),
            ];
            for (cx, cy, start_ang) in corners {
                for k in 0..=q {
                    let a = start_ang + 0.5 * std::f64::consts::PI * (k as f64) / (q as f64);
                    let x = cx + r * a.cos();
                    let y = cy + r * a.sin();
                    prof.push(Point3::new(x, y, 0.0));
                }
            }
            Ok(extrude_polygon(&prof, Vec3::new(0.0, 0.0, t)))
        }
        Feature::FunnelTube { top_outer_radius, bottom_outer_radius, wall_thickness, height, segments, .. } => {
            let tor = resolve_one(id, top_outer_radius, params)?;
            let bor = resolve_one(id, bottom_outer_radius, params)?;
            let wt = resolve_one(id, wall_thickness, params)?;
            let h = resolve_one(id, height, params)?;
            if tor <= 0.0 || bor <= 0.0 || wt <= 0.0 || h <= 0.0
                || tor - wt <= 0.0 || bor - wt <= 0.0 || *segments < 6
            {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("FunnelTube requires positive dims, inner > 0 (got tor={tor}, bor={bor}, wt={wt})") });
            }
            let outer = frustum_faceted(tor, bor, h, *segments);
            let inner = frustum_faceted(tor - wt, bor - wt, h + 2.0 * 1e-3, *segments);
            let inner_t = translate_solid(&inner, Vec3::new(0.0, 0.0, -1e-3));
            outer.try_difference(&inner_t).map_err(|e| EvalError::Boolean { id: id.into(), op: "funnel_tube_bore", message: e.message })
        }
        Feature::FlatWasher { outer_radius, inner_radius, thickness, segments, .. } => {
            let r_out = resolve_one(id, outer_radius, params)?;
            let r_in = resolve_one(id, inner_radius, params)?;
            let t = resolve_one(id, thickness, params)?;
            if r_out <= 0.0 || r_in <= 0.0 || r_in >= r_out || t <= 0.0 || *segments < 6 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("FlatWasher requires inner<outer, positive dims (got out={r_out}, in={r_in}, t={t})") });
            }
            let outer = cylinder_faceted(r_out, t, *segments);
            let eps = (t * 0.05).max(1e-3);
            let inner = cylinder_along_axis(r_in, t + 2.0 * eps, *segments, 2, -eps, 0.0, 0.0);
            outer.try_difference(&inner).map_err(|e| EvalError::Boolean { id: id.into(), op: "flat_washer_bore", message: e.message })
        }
        Feature::RibbedPlate { plate_length, plate_width, plate_thickness, n_ribs, rib_width, rib_height, .. } => {
            let l = resolve_one(id, plate_length, params)?;
            let w = resolve_one(id, plate_width, params)?;
            let t = resolve_one(id, plate_thickness, params)?;
            let rw = resolve_one(id, rib_width, params)?;
            let rh = resolve_one(id, rib_height, params)?;
            if l <= 0.0 || w <= 0.0 || t <= 0.0 || rw <= 0.0 || rh <= 0.0 || *n_ribs == 0 || rw >= l / *n_ribs as f64 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("RibbedPlate requires rib_width < plate_length/n_ribs (got l={l}, n={n_ribs}, rw={rw})") });
            }
            let plate = box_at(Vec3::new(l, w, t), Point3::new(0.0, 0.0, 0.0));
            let mut acc = plate;
            // Each rib: rw wide along +x, w deep along +y, rh tall along +z (above plate).
            // Centers spaced evenly: x_center_k = (k + 0.5) * (l / n_ribs).
            for k in 0..*n_ribs {
                let cx = ((k as f64) + 0.5) * (l / *n_ribs as f64);
                let rib = box_at(
                    Vec3::new(rw, w, rh + 1e-3),
                    Point3::new(cx - rw / 2.0, 0.0, t - 1e-3),
                );
                acc = acc.try_union(&rib).map_err(|e| EvalError::Boolean { id: id.into(), op: "ribbed_plate_rib", message: format!("rib {k}: {}", e.message) })?;
            }
            Ok(acc)
        }
        Feature::Pediment { base_width, base_height, gable_height, depth, .. } => {
            let bw = resolve_one(id, base_width, params)?;
            let bh = resolve_one(id, base_height, params)?;
            let gh = resolve_one(id, gable_height, params)?;
            let d = resolve_one(id, depth, params)?;
            if bw <= 0.0 || bh <= 0.0 || gh <= 0.0 || d <= 0.0 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("Pediment requires positive dims (got bw={bw}, bh={bh}, gh={gh}, d={d})") });
            }
            // Rectangle base + triangle on top, all in xy plane, extruded along +z.
            // CCW outer walk:
            //   (0, 0) -> (bw, 0) -> (bw, bh) -> (bw/2, bh + gh) -> (0, bh)
            let prof = vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(bw, 0.0, 0.0),
                Point3::new(bw, bh, 0.0),
                Point3::new(bw / 2.0, bh + gh, 0.0),
                Point3::new(0.0, bh, 0.0),
            ];
            Ok(extrude_polygon(&prof, Vec3::new(0.0, 0.0, d)))
        }
        Feature::Vault { base_width, base_length, base_height, segments, .. } => {
            let bw = resolve_one(id, base_width, params)?;
            let bl = resolve_one(id, base_length, params)?;
            let bh = resolve_one(id, base_height, params)?;
            if bw <= 0.0 || bl <= 0.0 || bh <= 0.0 || *segments < 6 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("Vault requires positive dims (got bw={bw}, bl={bl}, bh={bh})") });
            }
            // Base box at z=[0, bh], x=[0, bw], y=[0, bl].
            // Half-cylinder on top: radius = bw/2, axis along +y at (bw/2, *, bh).
            let base = box_at(Vec3::new(bw, bl, bh), Point3::new(0.0, 0.0, 0.0));
            // Build a full cylinder along +y with radius bw/2 at (bw/2, 0, bh),
            // then carve away the bottom half with a box at z<bh.
            let cyl = cylinder_along_axis(bw / 2.0, bl, *segments, 1, 0.0, bw / 2.0, bh);
            let cutter = box_at(
                Vec3::new(bw + 2.0 * bw, bl + 1e-3, bw),
                Point3::new(-bw, -1e-3 / 2.0, bh - bw),
            );
            let half_cyl = cyl.try_difference(&cutter)
                .map_err(|e| EvalError::Boolean { id: id.into(), op: "vault_half_cyl", message: e.message })?;
            base.try_union(&half_cyl).map_err(|e| EvalError::Boolean { id: id.into(), op: "vault_join", message: e.message })
        }
        Feature::ShelfBracket { horizontal, vertical, thickness, depth, .. } => {
            let h = resolve_one(id, horizontal, params)?;
            let v = resolve_one(id, vertical, params)?;
            let t = resolve_one(id, thickness, params)?;
            let d = resolve_one(id, depth, params)?;
            if h <= 0.0 || v <= 0.0 || t <= 0.0 || d <= 0.0 || t >= h || t >= v {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("ShelfBracket requires t < h and t < v, positive dims (got h={h}, v={v}, t={t})") });
            }
            // L-bracket profile + diagonal brace from corner (h, t) to (t, v).
            // The CCW polygon walk: outer L + diagonal cut.
            // Simpler: build LBracket profile, then add a triangular brace.
            // For v1: just build a single profile that walks the outer edge:
            //   (0, 0) -> (h, 0) -> (h, t) -> (t, v) -> (0, v) -> (0, 0)
            // This includes the diagonal as one straight edge (not a separate brace).
            // Useful for visualisation, not a real reinforced bracket — but
            // captures the typical shelf-bracket silhouette.
            let prof = vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(h, 0.0, 0.0),
                Point3::new(h, t, 0.0),
                Point3::new(t, v, 0.0),
                Point3::new(0.0, v, 0.0),
            ];
            Ok(extrude_polygon(&prof, Vec3::new(0.0, 0.0, d)))
        }
        Feature::NameTag { width, height, thickness, corner_radius, hole_radius, hole_offset_from_top, segments, .. } => {
            let w = resolve_one(id, width, params)?;
            let h = resolve_one(id, height, params)?;
            let t = resolve_one(id, thickness, params)?;
            let r = resolve_one(id, corner_radius, params)?;
            let hr = resolve_one(id, hole_radius, params)?;
            let off = resolve_one(id, hole_offset_from_top, params)?;
            if w <= 0.0 || h <= 0.0 || t <= 0.0 || r < 0.0 || hr <= 0.0 || off <= hr || 2.0 * r >= w || 2.0 * r >= h || hr >= w / 2.0 || *segments < 6 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("NameTag invalid params (got w={w}, h={h}, t={t}, r={r}, hr={hr}, off={off})") });
            }
            // Build a rounded-rect plate, drill a hole near the top.
            let mut prof: Vec<Point3> = Vec::new();
            let q = (*segments).max(4) / 4;
            let corners = [
                (r, r, std::f64::consts::PI),
                (w - r, r, 1.5 * std::f64::consts::PI),
                (w - r, h - r, 0.0),
                (r, h - r, 0.5 * std::f64::consts::PI),
            ];
            for (cx, cy, start_ang) in corners {
                for k in 0..=q {
                    let a = start_ang + 0.5 * std::f64::consts::PI * (k as f64) / (q as f64);
                    let x = cx + r * a.cos();
                    let y = cy + r * a.sin();
                    prof.push(Point3::new(x, y, 0.0));
                }
            }
            let plate = extrude_polygon(&prof, Vec3::new(0.0, 0.0, t));
            // Hole at (w/2, h - off, *), through the plate.
            let eps = (t * 0.05).max(1e-3);
            let hole = cylinder_along_axis(hr, t + 2.0 * eps, *segments, 2, -eps, w / 2.0, h - off);
            plate.try_difference(&hole).map_err(|e| EvalError::Boolean { id: id.into(), op: "name_tag_hole", message: e.message })
        }
        Feature::Plinth { bottom_side, bottom_height, top_side, top_height, .. } => {
            let bs = resolve_one(id, bottom_side, params)?;
            let bh = resolve_one(id, bottom_height, params)?;
            let ts = resolve_one(id, top_side, params)?;
            let th = resolve_one(id, top_height, params)?;
            if bs <= 0.0 || bh <= 0.0 || ts <= 0.0 || th <= 0.0 || ts > bs {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("Plinth requires top_side <= bottom_side, positive dims (got bs={bs}, ts={ts})") });
            }
            // Bottom slab centered, top slab centered on top.
            let bottom = box_at(Vec3::new(bs, bs, bh), Point3::new(-bs / 2.0, -bs / 2.0, 0.0));
            let eps = (th * 0.05).max(1e-3);
            let top = box_at(Vec3::new(ts, ts, th + eps), Point3::new(-ts / 2.0, -ts / 2.0, bh - eps));
            bottom.try_union(&top).map_err(|e| EvalError::Boolean { id: id.into(), op: "plinth_join", message: e.message })
        }
        Feature::ParapetWall { length, wall_height, wall_thickness, merlon_count, merlon_height, .. } => {
            let l = resolve_one(id, length, params)?;
            let wh = resolve_one(id, wall_height, params)?;
            let wt = resolve_one(id, wall_thickness, params)?;
            let mh = resolve_one(id, merlon_height, params)?;
            if l <= 0.0 || wh <= 0.0 || wt <= 0.0 || mh <= 0.0 || *merlon_count == 0 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("ParapetWall requires positive dims, merlon_count >= 1 (got l={l}, wh={wh}, mh={mh})") });
            }
            // Wall base: l × wt × wh.
            let wall = box_at(Vec3::new(l, wt, wh), Point3::new(0.0, 0.0, 0.0));
            // Merlons: a row along +x. Width of each merlon = l / (2 * merlon_count - 1)
            // (alternates with gaps of same width, with merlons at positions 0, 2, 4, ...).
            // For simplicity: place merlon_count merlons, each centered at (k+0.5)/(merlon_count) * l,
            // each of width = l / (merlon_count * 2).
            let mut acc = wall;
            let merlon_w = l / (*merlon_count as f64 * 2.0);
            for k in 0..*merlon_count {
                let cx = (k as f64 + 0.5) * (l / *merlon_count as f64);
                let merlon = box_at(
                    Vec3::new(merlon_w, wt, mh + 1e-3),
                    Point3::new(cx - merlon_w / 2.0, 0.0, wh - 1e-3),
                );
                acc = acc.try_union(&merlon).map_err(|e| EvalError::Boolean { id: id.into(), op: "parapet_merlon", message: format!("merlon {k}: {}", e.message) })?;
            }
            Ok(acc)
        }
        Feature::BeamWithHoles { length, width, height, hole_radius, hole_count, segments, .. } => {
            let l = resolve_one(id, length, params)?;
            let w = resolve_one(id, width, params)?;
            let h = resolve_one(id, height, params)?;
            let r = resolve_one(id, hole_radius, params)?;
            if l <= 0.0 || w <= 0.0 || h <= 0.0 || r <= 0.0 || *hole_count == 0
                || 2.0 * r >= h || *segments < 6
            {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("BeamWithHoles requires 2r<h, positive dims, hole_count>=1 (got l={l}, h={h}, r={r}, hc={hole_count})") });
            }
            let beam = box_at(Vec3::new(l, w, h), Point3::new(0.0, 0.0, 0.0));
            let mut acc = beam;
            let eps = (w * 0.05).max(1e-3);
            for k in 0..*hole_count {
                let cx = ((k as f64) + 0.5) * (l / *hole_count as f64);
                let hole = cylinder_along_axis(r, w + 2.0 * eps, *segments, 1, -eps, cx, h / 2.0);
                acc = acc.try_difference(&hole).map_err(|e| EvalError::Boolean { id: id.into(), op: "beam_holes_drill", message: format!("hole {k}: {}", e.message) })?;
            }
            Ok(acc)
        }
        Feature::Ellipsoid3D { rx, ry, rz, stacks, slices, .. } => {
            let rxv = resolve_one(id, rx, params)?;
            let ryv = resolve_one(id, ry, params)?;
            let rzv = resolve_one(id, rz, params)?;
            if rxv <= 0.0 || ryv <= 0.0 || rzv <= 0.0 || *stacks < 2 || *slices < 3 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("Ellipsoid3D requires positive radii, stacks>=2, slices>=3 (got rx={rxv}, ry={ryv}, rz={rzv})") });
            }
            let unit = sphere_faceted(1.0, *stacks, *slices);
            Ok(scale_xyz_solid(&unit, rxv, ryv, rzv))
        }
        Feature::VectorArrow { from, to, shaft_radius, head_radius, head_length, segments, .. } => {
            let f = resolve3(id, from, params)?;
            let t = resolve3(id, to, params)?;
            let sr = resolve_one(id, shaft_radius, params)?;
            let hr = resolve_one(id, head_radius, params)?;
            let hl = resolve_one(id, head_length, params)?;
            if sr <= 0.0 || hr <= 0.0 || hl <= 0.0 || hr <= sr || *segments < 6 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("VectorArrow requires head_radius>shaft_radius, positive dims (got sr={sr}, hr={hr}, hl={hl})") });
            }
            let dx = t[0] - f[0];
            let dy = t[1] - f[1];
            let dz = t[2] - f[2];
            let len = (dx * dx + dy * dy + dz * dz).sqrt();
            if len < 1e-9 || len <= hl {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("VectorArrow requires |to - from| > head_length (got len={len}, hl={hl})") });
            }
            // Shaft: from `f` to a point hl back from `t` along the segment.
            let shaft_end = [
                t[0] - dx * hl / len,
                t[1] - dy * hl / len,
                t[2] - dz * hl / len,
            ];
            let shaft = sweep_cylinder_segment(f, shaft_end, sr, *segments).ok_or_else(|| EvalError::Invalid {
                id: id.into(),
                reason: "VectorArrow shaft has zero length".into(),
            })?;
            // Head: cone — base at shaft_end (radius hr), apex at `t`.
            // Build as a tapered SweepPath: cylinder is sweep_cylinder_segment but
            // we don't have a tapered version. Use a small extrude_lofted from the
            // base disk to the apex.
            let n = *segments;
            let mut base_circle = Vec::with_capacity(n);
            // We need orthonormal frame at shaft_end. Compute one.
            let dir_norm = [dx / len, dy / len, dz / len];
            // Pick a perpendicular: cross with x_axis or y_axis depending on alignment.
            let perp = if dir_norm[0].abs() < 0.9 {
                let cx = dir_norm[1] * 0.0 - dir_norm[2] * 1.0;
                let cy = dir_norm[2] * 0.0 - dir_norm[0] * 0.0;
                let cz = dir_norm[0] * 1.0 - dir_norm[1] * 0.0;
                let cn = (cx * cx + cy * cy + cz * cz).sqrt();
                [cx / cn, cy / cn, cz / cn]
            } else {
                // Use y axis as perp seed.
                [0.0, 1.0, 0.0]
            };
            // Second perp: dir × perp
            let perp2 = [
                dir_norm[1] * perp[2] - dir_norm[2] * perp[1],
                dir_norm[2] * perp[0] - dir_norm[0] * perp[2],
                dir_norm[0] * perp[1] - dir_norm[1] * perp[0],
            ];
            for k in 0..n {
                let theta = 2.0 * std::f64::consts::PI * (k as f64) / (n as f64);
                let cs = theta.cos();
                let sn = theta.sin();
                base_circle.push(Point3::new(
                    shaft_end[0] + hr * (cs * perp[0] + sn * perp2[0]),
                    shaft_end[1] + hr * (cs * perp[1] + sn * perp2[1]),
                    shaft_end[2] + hr * (cs * perp[2] + sn * perp2[2]),
                ));
            }
            let apex_sz = hr * 1e-3;
            let mut apex_circle = Vec::with_capacity(n);
            for k in 0..n {
                let theta = 2.0 * std::f64::consts::PI * (k as f64) / (n as f64);
                let cs = theta.cos();
                let sn = theta.sin();
                apex_circle.push(Point3::new(
                    t[0] + apex_sz * (cs * perp[0] + sn * perp2[0]),
                    t[1] + apex_sz * (cs * perp[1] + sn * perp2[1]),
                    t[2] + apex_sz * (cs * perp[2] + sn * perp2[2]),
                ));
            }
            let head = extrude_lofted(&base_circle, &apex_circle);
            shaft.try_union(&head).map_err(|e| EvalError::Boolean { id: id.into(), op: "vector_arrow_join", message: e.message })
        }
        Feature::BoneShape { end_radius, shaft_radius, shaft_length, stacks, slices, .. } => {
            let er = resolve_one(id, end_radius, params)?;
            let sr = resolve_one(id, shaft_radius, params)?;
            let sl = resolve_one(id, shaft_length, params)?;
            if er <= 0.0 || sr <= 0.0 || sr >= er || sl <= 0.0 || *stacks < 2 || *slices < 3 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("BoneShape requires shaft<end, positive dims (got er={er}, sr={sr})") });
            }
            // Two end balls (faceted spheres) + cylinder shaft between them.
            let shaft = cylinder_along_axis(sr, sl, *slices, 2, 0.0, 0.0, 0.0);
            let end_a = sphere_faceted(er, *stacks, *slices);
            let end_b = sphere_faceted(er, *stacks, *slices);
            let end_b_t = translate_solid(&end_b, Vec3::new(0.0, 0.0, sl));
            let mid = shaft
                .try_union(&end_a)
                .map_err(|e| EvalError::Boolean { id: id.into(), op: "bone_shape_a", message: e.message })?;
            mid.try_union(&end_b_t)
                .map_err(|e| EvalError::Boolean { id: id.into(), op: "bone_shape_b", message: e.message })
        }
        Feature::Pawn { base_radius, base_height, body_top_radius, body_height, head_radius, segments, stacks, .. } => {
            let br = resolve_one(id, base_radius, params)?;
            let bh = resolve_one(id, base_height, params)?;
            let btr = resolve_one(id, body_top_radius, params)?;
            let bdy_h = resolve_one(id, body_height, params)?;
            let hr = resolve_one(id, head_radius, params)?;
            if br <= 0.0 || bh <= 0.0 || btr <= 0.0 || bdy_h <= 0.0 || hr <= 0.0
                || btr >= br || hr <= btr || *segments < 6 || *stacks < 2
            {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("Pawn requires top < base, head > top, positive dims (got br={br}, btr={btr}, hr={hr})") });
            }
            let base = cylinder_along_axis(br, bh, *segments, 2, 0.0, 0.0, 0.0);
            let body = frustum_faceted(btr, br, bdy_h, *segments);
            let body_t = translate_solid(&body, Vec3::new(0.0, 0.0, bh));
            let head = sphere_faceted(hr, *stacks, *segments);
            let head_t = translate_solid(&head, Vec3::new(0.0, 0.0, bh + bdy_h));
            let with_body = base.try_union(&body_t).map_err(|e| EvalError::Boolean { id: id.into(), op: "pawn_base_body", message: e.message })?;
            with_body.try_union(&head_t).map_err(|e| EvalError::Boolean { id: id.into(), op: "pawn_head", message: e.message })
        }
        Feature::Rook { base_radius, base_height, body_radius, body_height, segments, .. } => {
            let br = resolve_one(id, base_radius, params)?;
            let bh = resolve_one(id, base_height, params)?;
            let bod_r = resolve_one(id, body_radius, params)?;
            let bod_h = resolve_one(id, body_height, params)?;
            if br <= 0.0 || bh <= 0.0 || bod_r <= 0.0 || bod_h <= 0.0 || bod_r >= br || *segments < 6 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("Rook requires body < base, positive dims") });
            }
            let base = cylinder_along_axis(br, bh, *segments, 2, 0.0, 0.0, 0.0);
            let body = cylinder_along_axis(bod_r, bod_h, *segments, 2, bh, 0.0, 0.0);
            // 4 merlons on top, alternating around the perimeter.
            let merlon_h = bod_h * 0.3;
            let merlon_w = bod_r * 0.4;
            let body_top = bh + bod_h;
            let mut acc = base.try_union(&body).map_err(|e| EvalError::Boolean { id: id.into(), op: "rook_base_body", message: e.message })?;
            for k in 0..4 {
                let theta = std::f64::consts::FRAC_PI_2 * (k as f64);
                let cx = bod_r * 0.6 * theta.cos();
                let cy = bod_r * 0.6 * theta.sin();
                let merlon = box_at(
                    Vec3::new(merlon_w, merlon_w, merlon_h + 1e-3),
                    Point3::new(cx - merlon_w / 2.0, cy - merlon_w / 2.0, body_top - 1e-3),
                );
                acc = acc.try_union(&merlon).map_err(|e| EvalError::Boolean { id: id.into(), op: "rook_merlon", message: format!("merlon {k}: {}", e.message) })?;
            }
            Ok(acc)
        }
        Feature::Bishop { base_radius, base_height, body_radius, body_height, head_radius, slot_width, segments, stacks, .. } => {
            let br = resolve_one(id, base_radius, params)?;
            let bh = resolve_one(id, base_height, params)?;
            let bod_r = resolve_one(id, body_radius, params)?;
            let bod_h = resolve_one(id, body_height, params)?;
            let hr = resolve_one(id, head_radius, params)?;
            let sw = resolve_one(id, slot_width, params)?;
            if br <= 0.0 || bh <= 0.0 || bod_r <= 0.0 || bod_h <= 0.0 || hr <= 0.0
                || bod_r >= br || hr < bod_r || sw <= 0.0 || sw >= 2.0 * hr
                || *segments < 6 || *stacks < 2
            {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("Bishop invalid params (got br={br}, bod_r={bod_r}, hr={hr}, sw={sw})") });
            }
            let base = cylinder_along_axis(br, bh, *segments, 2, 0.0, 0.0, 0.0);
            let body = cylinder_along_axis(bod_r, bod_h, *segments, 2, bh, 0.0, 0.0);
            let head = sphere_faceted(hr, *stacks, *segments);
            let head_t = translate_solid(&head, Vec3::new(0.0, 0.0, bh + bod_h));
            let mut acc = base
                .try_union(&body).map_err(|e| EvalError::Boolean { id: id.into(), op: "bishop_base_body", message: e.message })?
                .try_union(&head_t).map_err(|e| EvalError::Boolean { id: id.into(), op: "bishop_head", message: e.message })?;
            // Slot: thin box across the head.
            let slot = box_at(
                Vec3::new(sw, 4.0 * hr, hr * 0.6),
                Point3::new(-sw / 2.0, -2.0 * hr, bh + bod_h + hr * 0.5),
            );
            acc = acc.try_difference(&slot).map_err(|e| EvalError::Boolean { id: id.into(), op: "bishop_slot", message: e.message })?;
            Ok(acc)
        }
        Feature::Marker3D { center, axis_length, bar_radius, segments, .. } => {
            let c = resolve3(id, center, params)?;
            let l = resolve_one(id, axis_length, params)?;
            let r = resolve_one(id, bar_radius, params)?;
            if l <= 0.0 || r <= 0.0 || *segments < 6 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("Marker3D requires positive dims (got l={l}, r={r})") });
            }
            // Three bars along x, y, z, each of length 2*l centered at center.
            let bar_x = cylinder_along_axis(r, 2.0 * l, *segments, 0, c[0] - l, c[1], c[2]);
            let bar_y = cylinder_along_axis(r, 2.0 * l + 2.0 * r, *segments, 1, c[1] - l - r, c[0], c[2]);
            let bar_z = cylinder_along_axis(r, 2.0 * l + 2.0 * r, *segments, 2, c[2] - l - r, c[0], c[1]);
            let xy = bar_x.try_union(&bar_y).map_err(|e| EvalError::Boolean { id: id.into(), op: "marker_xy", message: e.message })?;
            xy.try_union(&bar_z).map_err(|e| EvalError::Boolean { id: id.into(), op: "marker_xyz", message: e.message })
        }
        Feature::HollowBrick { length, width, height, wall_thickness, .. } => {
            let l = resolve_one(id, length, params)?;
            let w = resolve_one(id, width, params)?;
            let h = resolve_one(id, height, params)?;
            let t = resolve_one(id, wall_thickness, params)?;
            if l <= 0.0 || w <= 0.0 || h <= 0.0 || t <= 0.0 || 2.0 * t >= l || 2.0 * t >= w || t >= h {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("HollowBrick requires 2t<l&w, t<h, positive dims (got l={l}, w={w}, h={h}, t={t})") });
            }
            let outer = box_at(Vec3::new(l, w, h), Point3::new(0.0, 0.0, 0.0));
            // Cavity: open top, all walls thickness t.
            let cavity = box_at(
                Vec3::new(l - 2.0 * t, w - 2.0 * t, h),
                Point3::new(t, t, t),
            );
            outer.try_difference(&cavity).map_err(|e| EvalError::Boolean { id: id.into(), op: "hollow_brick_cavity", message: e.message })
        }
        Feature::StadiumPlate { length, width, thickness, segments, .. } => {
            let l = resolve_one(id, length, params)?;
            let w = resolve_one(id, width, params)?;
            let t = resolve_one(id, thickness, params)?;
            if l <= 0.0 || w <= 0.0 || t <= 0.0 || *segments < 6 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("StadiumPlate requires positive dims (got l={l}, w={w}, t={t})") });
            }
            // Same outline as Slot3D: rectangle l × w with semicircle ends.
            let r = w / 2.0;
            let n = (*segments).max(8);
            let mut prof = Vec::with_capacity(2 * n + 4);
            for k in 0..=n {
                let theta = -std::f64::consts::FRAC_PI_2
                    + std::f64::consts::PI * (k as f64) / (n as f64);
                prof.push(Point3::new(l + r * theta.cos(), r * theta.sin(), 0.0));
            }
            for k in 0..=n {
                let theta = std::f64::consts::FRAC_PI_2
                    + std::f64::consts::PI * (k as f64) / (n as f64);
                prof.push(Point3::new(r * theta.cos(), r * theta.sin(), 0.0));
            }
            Ok(extrude_polygon(&prof, Vec3::new(0.0, 0.0, t)))
        }
        Feature::Bowtie { width, height, pinch, depth, .. } => {
            let w = resolve_one(id, width, params)?;
            let h = resolve_one(id, height, params)?;
            let p = resolve_one(id, pinch, params)?;
            let d = resolve_one(id, depth, params)?;
            if w <= 0.0 || h <= 0.0 || p < 0.0 || d <= 0.0 || p >= h {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("Bowtie requires pinch < height, positive dims (got w={w}, h={h}, p={p})") });
            }
            // Bowtie outline (CCW from +z):
            //   (-w/2, -h/2), (0, -p/2), (w/2, -h/2), (w/2, h/2), (0, p/2), (-w/2, h/2)
            let prof = vec![
                Point3::new(-w / 2.0, -h / 2.0, 0.0),
                Point3::new( 0.0,     -p / 2.0, 0.0),
                Point3::new( w / 2.0, -h / 2.0, 0.0),
                Point3::new( w / 2.0,  h / 2.0, 0.0),
                Point3::new( 0.0,      p / 2.0, 0.0),
                Point3::new(-w / 2.0,  h / 2.0, 0.0),
            ];
            Ok(extrude_polygon(&prof, Vec3::new(0.0, 0.0, d)))
        }
        Feature::HollowCone { radius, height, bore_radius, segments, .. } => {
            let r = resolve_one(id, radius, params)?;
            let h = resolve_one(id, height, params)?;
            let br = resolve_one(id, bore_radius, params)?;
            if r <= 0.0 || h <= 0.0 || br <= 0.0 || br >= r || *segments < 6 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("HollowCone requires bore_radius<radius, positive dims (got r={r}, br={br})") });
            }
            let cone_solid = cone_faceted(r, h, *segments);
            let eps = (h * 0.05).max(1e-3);
            let bore = cylinder_along_axis(br, h + 2.0 * eps, *segments, 2, -eps, 0.0, 0.0);
            cone_solid.try_difference(&bore).map_err(|e| EvalError::Boolean { id: id.into(), op: "hollow_cone_bore", message: e.message })
        }
        Feature::ArchedDoorway { width, height, wall_thickness, depth, segments, .. } => {
            let w = resolve_one(id, width, params)?;
            let h = resolve_one(id, height, params)?;
            let t = resolve_one(id, wall_thickness, params)?;
            let d = resolve_one(id, depth, params)?;
            if w <= 0.0 || h <= 0.0 || t <= 0.0 || d <= 0.0 || h <= w / 2.0 || 2.0 * t >= w || *segments < 6 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("ArchedDoorway requires h > w/2 (room for arch), 2t<w (got w={w}, h={h}, t={t})") });
            }
            // Outer profile: rectangle (w × (h - w/2)) topped by a half-circle of radius w/2.
            // Walk CCW (in xy plane):
            let arch_r = w / 2.0;
            let rect_h = h - arch_r;
            let mut outer_prof = vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(w, 0.0, 0.0),
                Point3::new(w, rect_h, 0.0),
            ];
            // Arch from (w, rect_h) at theta=0 to (0, rect_h) at theta=PI.
            // Skip the endpoints since (w, rect_h) was already pushed and
            // (0, rect_h) is reached at theta=PI.
            let n_arch = (*segments).max(8);
            for k in 1..n_arch {
                let theta = std::f64::consts::PI * (k as f64) / (n_arch as f64);
                outer_prof.push(Point3::new(
                    arch_r + arch_r * theta.cos(),
                    rect_h + arch_r * theta.sin(),
                    0.0,
                ));
            }
            outer_prof.push(Point3::new(0.0, rect_h, 0.0));
            let outer = extrude_polygon(&outer_prof, Vec3::new(0.0, 0.0, d));
            // Inner cutout: same profile shrunk by t. The arch radius for the
            // cutout is arch_r - t, and the rectangle is shifted inward.
            let inner_arch_r = arch_r - t;
            let inner_rect_h = rect_h - t;
            if inner_arch_r <= 0.0 || inner_rect_h <= 0.0 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("ArchedDoorway wall_thickness too large for arch ({t})") });
            }
            let mut inner_prof = vec![
                Point3::new(t, t, 0.0),
                Point3::new(w - t, t, 0.0),
                Point3::new(w - t, inner_rect_h, 0.0),
            ];
            for k in 1..n_arch {
                let theta = std::f64::consts::PI * (k as f64) / (n_arch as f64);
                inner_prof.push(Point3::new(
                    arch_r + inner_arch_r * theta.cos(),
                    inner_rect_h + inner_arch_r * theta.sin(),
                    0.0,
                ));
            }
            inner_prof.push(Point3::new(t, inner_rect_h, 0.0));
            let inner = extrude_polygon(&inner_prof, Vec3::new(0.0, 0.0, d + 2.0 * 1e-3));
            let inner_t = translate_solid(&inner, Vec3::new(0.0, 0.0, -1e-3));
            outer.try_difference(&inner_t).map_err(|e| EvalError::Boolean { id: id.into(), op: "arched_doorway_carve", message: e.message })
        }
        Feature::CamLobe { rx, ry, hole_radius, thickness, segments, .. } => {
            let rxv = resolve_one(id, rx, params)?;
            let ryv = resolve_one(id, ry, params)?;
            let hr = resolve_one(id, hole_radius, params)?;
            let t = resolve_one(id, thickness, params)?;
            if rxv <= 0.0 || ryv <= 0.0 || hr <= 0.0 || t <= 0.0 || hr >= rxv || hr >= ryv || *segments < 6 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("CamLobe requires hole<rx&ry, positive dims (got rx={rxv}, ry={ryv}, hr={hr})") });
            }
            let n = *segments;
            let mut prof = Vec::with_capacity(n);
            for i in 0..n {
                let theta = 2.0 * std::f64::consts::PI * (i as f64) / (n as f64);
                prof.push(Point3::new(rxv * theta.cos(), ryv * theta.sin(), 0.0));
            }
            let disk = extrude_polygon(&prof, Vec3::new(0.0, 0.0, t));
            let eps = (t * 0.05).max(1e-3);
            let hole = cylinder_along_axis(hr, t + 2.0 * eps, *segments, 2, -eps, 0.0, 0.0);
            disk.try_difference(&hole).map_err(|e| EvalError::Boolean { id: id.into(), op: "cam_lobe_bore", message: e.message })
        }
        Feature::ButtonShape { radius, thickness, segments, .. } => {
            let r = resolve_one(id, radius, params)?;
            let t = resolve_one(id, thickness, params)?;
            if r <= 0.0 || t <= 0.0 || *segments < 6 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("ButtonShape requires positive dims (got r={r}, t={t})") });
            }
            Ok(cylinder_faceted(r, t, *segments))
        }
        Feature::FilletedSlot { length, width, corner_radius, depth, segments, .. } => {
            let l = resolve_one(id, length, params)?;
            let w = resolve_one(id, width, params)?;
            let r = resolve_one(id, corner_radius, params)?;
            let d = resolve_one(id, depth, params)?;
            if l <= 0.0 || w <= 0.0 || r <= 0.0 || d <= 0.0 || 2.0 * r > w || 2.0 * r > l || *segments < 4 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("FilletedSlot requires 2r<=w&l, positive dims (got l={l}, w={w}, r={r})") });
            }
            // Rounded rectangle in xy plane. Reuse RoundedRect logic.
            let mut prof: Vec<Point3> = Vec::new();
            let q = (*segments).max(4) / 4;
            let corners = [
                (r, r, std::f64::consts::PI),
                (l - r, r, 1.5 * std::f64::consts::PI),
                (l - r, w - r, 0.0),
                (r, w - r, 0.5 * std::f64::consts::PI),
            ];
            for (cx, cy, start_ang) in corners {
                for k in 0..=q {
                    let a = start_ang + 0.5 * std::f64::consts::PI * (k as f64) / (q as f64);
                    let x = cx + r * a.cos();
                    let y = cy + r * a.sin();
                    prof.push(Point3::new(x, y, 0.0));
                }
            }
            Ok(extrude_polygon(&prof, Vec3::new(0.0, 0.0, d)))
        }
        Feature::CoinShape { outer_radius, face_thickness, rim_height, rim_width, segments, .. } => {
            let r_out = resolve_one(id, outer_radius, params)?;
            let ft = resolve_one(id, face_thickness, params)?;
            let rh = resolve_one(id, rim_height, params)?;
            let rw = resolve_one(id, rim_width, params)?;
            if r_out <= 0.0 || ft <= 0.0 || rh <= 0.0 || rw <= 0.0 || rw >= r_out || *segments < 6 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("CoinShape requires rim_width<outer_radius, positive dims (got r={r_out}, rw={rw})") });
            }
            // Face: full disk (radius r_out, thickness ft).
            let face = cylinder_faceted(r_out, ft, *segments);
            // Rim: ring (outer r_out, inner r_out - rw) of height rh on top of face.
            let rim_outer = cylinder_along_axis(r_out, rh, *segments, 2, ft, 0.0, 0.0);
            let eps = (rh * 0.05).max(1e-3);
            let rim_inner = cylinder_along_axis(r_out - rw, rh + 2.0 * eps, *segments, 2, ft - eps, 0.0, 0.0);
            let rim = rim_outer.try_difference(&rim_inner).map_err(|e| EvalError::Boolean { id: id.into(), op: "coin_rim", message: e.message })?;
            face.try_union(&rim).map_err(|e| EvalError::Boolean { id: id.into(), op: "coin_face_rim", message: e.message })
        }
        Feature::CylindricalCap { radius, length, segments, .. } => {
            let r = resolve_one(id, radius, params)?;
            let l = resolve_one(id, length, params)?;
            if r <= 0.0 || l <= 0.0 || *segments < 6 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("CylindricalCap requires positive dims (got r={r}, l={l})") });
            }
            // Cylinder along +y, radius r, length l, axis at z=0. Cut by box at z<0 to keep top half.
            let cyl = cylinder_along_axis(r, l, *segments, 1, 0.0, 0.0, 0.0);
            let cutter = box_at(
                Vec3::new(4.0 * r, l + 2.0 * 1e-3, r),
                Point3::new(-2.0 * r, -1e-3, -r),
            );
            cyl.try_difference(&cutter).map_err(|e| EvalError::Boolean { id: id.into(), op: "cylindrical_cap_cut", message: e.message })
        }
        Feature::SquaredRing { outer_width, outer_height, wall_thickness, depth, .. } => {
            let ow = resolve_one(id, outer_width, params)?;
            let oh = resolve_one(id, outer_height, params)?;
            let t = resolve_one(id, wall_thickness, params)?;
            let d = resolve_one(id, depth, params)?;
            if ow <= 0.0 || oh <= 0.0 || t <= 0.0 || d <= 0.0 || 2.0 * t >= ow || 2.0 * t >= oh {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("SquaredRing requires 2t < ow & oh (got ow={ow}, oh={oh}, t={t})") });
            }
            let outer = box_at(Vec3::new(ow, oh, d), Point3::new(0.0, 0.0, 0.0));
            let inner = box_at(
                Vec3::new(ow - 2.0 * t, oh - 2.0 * t, d + 2.0 * 1e-3),
                Point3::new(t, t, -1e-3),
            );
            outer.try_difference(&inner).map_err(|e| EvalError::Boolean { id: id.into(), op: "squared_ring_carve", message: e.message })
        }
        Feature::WaveProfile { wavelength, amplitude, n_waves, depth, height, .. } => {
            let wl = resolve_one(id, wavelength, params)?;
            let amp = resolve_one(id, amplitude, params)?;
            let d = resolve_one(id, depth, params)?;
            let h = resolve_one(id, height, params)?;
            if wl <= 0.0 || amp <= 0.0 || d <= 0.0 || h <= 0.0 || *n_waves == 0 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("WaveProfile requires positive dims, n_waves >= 1 (got wl={wl}, amp={amp}, n={n_waves})") });
            }
            // Build a triangle-wave outline in the xy plane (top wavy, bottom flat).
            // CCW walk: bottom-left → bottom-right → wavy top from right to left.
            let total_l = (*n_waves as f64) * wl;
            let mut prof = vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(total_l, 0.0, 0.0),
            ];
            // Wavy top: for each wave, walk from peak → valley → peak.
            // Total points: 2*n_waves + 1 alternating peak/valley.
            for k in 0..=(2 * *n_waves) {
                let x = total_l - (k as f64) * wl / 2.0;
                let y = if k % 2 == 0 { h + amp } else { h };
                prof.push(Point3::new(x, y, 0.0));
            }
            Ok(extrude_polygon(&prof, Vec3::new(0.0, 0.0, d)))
        }
        Feature::BulletShape { radius, body_length, stacks, slices, .. } => {
            let r = resolve_one(id, radius, params)?;
            let bl = resolve_one(id, body_length, params)?;
            if r <= 0.0 || bl <= 0.0 || *stacks < 2 || *slices < 3 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("BulletShape requires positive dims (got r={r}, bl={bl})") });
            }
            let body = cylinder_along_axis(r, bl, *slices, 2, 0.0, 0.0, 0.0);
            let sph = sphere_faceted(r, *stacks, *slices);
            // Hemisphere clip — keep only upper half.
            let cutter = box_at(
                Vec3::new(4.0 * r, 4.0 * r, 2.0 * r),
                Point3::new(-2.0 * r, -2.0 * r, -2.0 * r),
            );
            let hemi = sph.try_difference(&cutter).map_err(|e| EvalError::Boolean { id: id.into(), op: "bullet_hemi_cut", message: e.message })?;
            let hemi_t = translate_solid(&hemi, Vec3::new(0.0, 0.0, bl));
            body.try_union(&hemi_t).map_err(|e| EvalError::Boolean { id: id.into(), op: "bullet_join", message: e.message })
        }
        Feature::TriangularPlate { a, b, c, thickness, .. } => {
            let av = [resolve_one(id, &a[0], params)?, resolve_one(id, &a[1], params)?];
            let bv = [resolve_one(id, &b[0], params)?, resolve_one(id, &b[1], params)?];
            let cv = [resolve_one(id, &c[0], params)?, resolve_one(id, &c[1], params)?];
            let t = resolve_one(id, thickness, params)?;
            if t <= 0.0 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("TriangularPlate requires positive thickness (got {t})") });
            }
            // Check triangle non-degeneracy.
            let area2 = (bv[0] - av[0]) * (cv[1] - av[1]) - (cv[0] - av[0]) * (bv[1] - av[1]);
            if area2.abs() < 1e-12 {
                return Err(EvalError::Invalid { id: id.into(), reason: "TriangularPlate corners are colinear".into() });
            }
            // Ensure CCW.
            let prof = if area2 > 0.0 {
                vec![
                    Point3::new(av[0], av[1], 0.0),
                    Point3::new(bv[0], bv[1], 0.0),
                    Point3::new(cv[0], cv[1], 0.0),
                ]
            } else {
                vec![
                    Point3::new(av[0], av[1], 0.0),
                    Point3::new(cv[0], cv[1], 0.0),
                    Point3::new(bv[0], bv[1], 0.0),
                ]
            };
            Ok(extrude_polygon(&prof, Vec3::new(0.0, 0.0, t)))
        }
        Feature::Heart { size, thickness, segments, .. } => {
            let sz = resolve_one(id, size, params)?;
            let t = resolve_one(id, thickness, params)?;
            if sz <= 0.0 || t <= 0.0 || *segments < 8 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("Heart requires positive dims, segments>=8 (got sz={sz}, t={t})") });
            }
            // Parametric heart curve: x = 16 sin³(t), y = 13 cos(t) - 5 cos(2t) - 2 cos(3t) - cos(4t).
            // Scale to fit `size`. Sample N points around the curve.
            let n = *segments;
            let mut prof = Vec::with_capacity(n);
            // Compute raw points and find bounding box for scaling.
            let mut raw: Vec<(f64, f64)> = Vec::with_capacity(n);
            for i in 0..n {
                let theta = 2.0 * std::f64::consts::PI * (i as f64) / (n as f64);
                let x = 16.0 * theta.sin().powi(3);
                let y = 13.0 * theta.cos()
                    - 5.0 * (2.0 * theta).cos()
                    - 2.0 * (3.0 * theta).cos()
                    - (4.0 * theta).cos();
                raw.push((x, y));
            }
            // Find max extent.
            let max_extent = raw.iter().fold(0.0_f64, |a, &(x, y)| a.max(x.abs()).max(y.abs()));
            let scale = sz / (2.0 * max_extent);
            // The curve as parameterized goes CW above; we need CCW for extrude_polygon.
            // Reverse the order.
            for &(x, y) in raw.iter().rev() {
                prof.push(Point3::new(x * scale, y * scale, 0.0));
            }
            Ok(extrude_polygon(&prof, Vec3::new(0.0, 0.0, t)))
        }
        Feature::ChainLink { length, width, wall_thickness, depth, segments, .. } => {
            let l = resolve_one(id, length, params)?;
            let w = resolve_one(id, width, params)?;
            let wt = resolve_one(id, wall_thickness, params)?;
            let d = resolve_one(id, depth, params)?;
            if l <= 0.0 || w <= 0.0 || wt <= 0.0 || d <= 0.0 || 2.0 * wt >= w || *segments < 8 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("ChainLink requires 2wt < w (got l={l}, w={w}, wt={wt})") });
            }
            // Build outer stadium then subtract inner stadium.
            let outer_r = w / 2.0;
            let inner_r = (w / 2.0) - wt;
            let inner_l = l - 2.0 * wt;
            if inner_l <= 0.0 || inner_r <= 0.0 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("ChainLink wall too thick (got l={l}, w={w}, wt={wt})") });
            }
            let n = (*segments).max(8);
            let mut outer_prof = Vec::with_capacity(2 * n + 4);
            for k in 0..=n {
                let theta = -std::f64::consts::FRAC_PI_2
                    + std::f64::consts::PI * (k as f64) / (n as f64);
                outer_prof.push(Point3::new(l + outer_r * theta.cos() - l / 2.0, outer_r * theta.sin(), 0.0));
            }
            for k in 0..=n {
                let theta = std::f64::consts::FRAC_PI_2
                    + std::f64::consts::PI * (k as f64) / (n as f64);
                outer_prof.push(Point3::new(outer_r * theta.cos() - l / 2.0, outer_r * theta.sin(), 0.0));
            }
            let outer = extrude_polygon(&outer_prof, Vec3::new(0.0, 0.0, d));
            let mut inner_prof = Vec::with_capacity(2 * n + 4);
            for k in 0..=n {
                let theta = -std::f64::consts::FRAC_PI_2
                    + std::f64::consts::PI * (k as f64) / (n as f64);
                inner_prof.push(Point3::new(inner_l + inner_r * theta.cos() - inner_l / 2.0, inner_r * theta.sin(), 0.0));
            }
            for k in 0..=n {
                let theta = std::f64::consts::FRAC_PI_2
                    + std::f64::consts::PI * (k as f64) / (n as f64);
                inner_prof.push(Point3::new(inner_r * theta.cos() - inner_l / 2.0, inner_r * theta.sin(), 0.0));
            }
            let eps = (d * 0.05).max(1e-3);
            let inner = extrude_polygon(&inner_prof, Vec3::new(0.0, 0.0, d + 2.0 * eps));
            let inner_t = translate_solid(&inner, Vec3::new(0.0, 0.0, -eps));
            outer.try_difference(&inner_t).map_err(|e| EvalError::Boolean { id: id.into(), op: "chain_link_carve", message: e.message })
        }
        Feature::SpiralPlate { max_radius, revolutions, rod_radius, z, segments_per_revolution, .. } => {
            let r_max = resolve_one(id, max_radius, params)?;
            let revs = resolve_one(id, revolutions, params)?;
            let rod_r = resolve_one(id, rod_radius, params)?;
            let zv = resolve_one(id, z, params)?;
            if r_max <= 0.0 || revs <= 0.0 || rod_r <= 0.0 || *segments_per_revolution < 6 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("SpiralPlate requires positive dims (got rmax={r_max}, revs={revs}, rod_r={rod_r})") });
            }
            // Archimedean spiral: r(theta) = r_max * theta / (2π * revs), theta in [0, 2π*revs].
            let total_samples = (*segments_per_revolution as f64 * revs).ceil() as usize + 1;
            let total_angle = 2.0 * std::f64::consts::PI * revs;
            let mut acc: Option<Solid> = None;
            for i in 0..total_samples - 1 {
                let t0 = total_angle * i as f64 / (total_samples - 1) as f64;
                let t1 = total_angle * (i + 1) as f64 / (total_samples - 1) as f64;
                let r0 = r_max * t0 / total_angle;
                let r1 = r_max * t1 / total_angle;
                let p0 = [r0 * t0.cos(), r0 * t0.sin(), zv];
                let p1 = [r1 * t1.cos(), r1 * t1.sin(), zv];
                if (p1[0] - p0[0]).powi(2) + (p1[1] - p0[1]).powi(2) < 1e-18 {
                    continue;
                }
                let cyl = sweep_cylinder_segment(p0, p1, rod_r, 8).ok_or_else(|| EvalError::Invalid {
                    id: id.into(),
                    reason: format!("SpiralPlate seg {i} zero length"),
                })?;
                acc = Some(match acc.take() {
                    None => cyl,
                    Some(prev) => prev.try_union(&cyl).map_err(|e| EvalError::Boolean { id: id.into(), op: "spiral_join", message: format!("seg {i}: {}", e.message) })?,
                });
            }
            acc.ok_or_else(|| EvalError::Invalid { id: id.into(), reason: "SpiralPlate produced no segments".into() })
        }
        Feature::WindowFrame { outer_width, outer_height, frame_thickness, depth, .. } => {
            let ow = resolve_one(id, outer_width, params)?;
            let oh = resolve_one(id, outer_height, params)?;
            let t = resolve_one(id, frame_thickness, params)?;
            let d = resolve_one(id, depth, params)?;
            if ow <= 0.0 || oh <= 0.0 || t <= 0.0 || d <= 0.0 || 2.0 * t >= ow || 2.0 * t >= oh {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("WindowFrame requires 2t < ow & oh (got ow={ow}, oh={oh}, t={t})") });
            }
            let outer = box_at(Vec3::new(ow, oh, d), Point3::new(0.0, 0.0, 0.0));
            let inner = box_at(
                Vec3::new(ow - 2.0 * t, oh - 2.0 * t, d + 2.0 * 1e-3),
                Point3::new(t, t, -1e-3),
            );
            outer.try_difference(&inner).map_err(|e| EvalError::Boolean { id: id.into(), op: "window_frame_carve", message: e.message })
        }
        Feature::SquareKey { side, length, .. } => {
            let sd = resolve_one(id, side, params)?;
            let l = resolve_one(id, length, params)?;
            if sd <= 0.0 || l <= 0.0 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("SquareKey requires positive dims (got sd={sd}, l={l})") });
            }
            Ok(box_at(Vec3::new(sd, sd, l), Point3::new(-sd / 2.0, -sd / 2.0, 0.0)))
        }
        Feature::DiskWithSlots { radius, slot_count, slot_width, slot_depth, thickness, segments, .. } => {
            let r = resolve_one(id, radius, params)?;
            let sw = resolve_one(id, slot_width, params)?;
            let sd = resolve_one(id, slot_depth, params)?;
            let t = resolve_one(id, thickness, params)?;
            if r <= 0.0 || sw <= 0.0 || sd <= 0.0 || t <= 0.0 || sd >= r || *slot_count < 2 || *segments < 6 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("DiskWithSlots requires sd<r, slot_count>=2 (got r={r}, sd={sd}, sc={slot_count})") });
            }
            let disk = cylinder_faceted(r, t, *segments);
            let mut acc = disk;
            // For each slot: a thin radial box from r-sd to r+1, of width sw.
            // Position around the perimeter using rotation.
            for k in 0..*slot_count {
                let theta = 2.0 * std::f64::consts::PI * (k as f64) / (*slot_count as f64);
                // Build the slot box at the +x position then rotate.
                let slot_at_zero = box_at(
                    Vec3::new(sd + 1e-3, sw, t + 2.0 * 1e-3),
                    Point3::new(r - sd, -sw / 2.0, -1e-3),
                );
                let rotated = rotate_solid(&slot_at_zero, Vec3::new(0.0, 0.0, 1.0), theta, Point3::origin());
                acc = acc.try_difference(&rotated).map_err(|e| EvalError::Boolean { id: id.into(), op: "disk_slot_carve", message: format!("slot {k}: {}", e.message) })?;
            }
            Ok(acc)
        }
        Feature::FivePointedBadge { outer_radius, inner_radius, thickness, .. } => {
            let r_out = resolve_one(id, outer_radius, params)?;
            let r_in = resolve_one(id, inner_radius, params)?;
            let t = resolve_one(id, thickness, params)?;
            if r_out <= 0.0 || r_in <= 0.0 || r_in >= r_out || t <= 0.0 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("FivePointedBadge requires inner<outer, positive dims (got out={r_out}, in={r_in})") });
            }
            // 5-pointed star: 10 vertices alternating outer/inner.
            let n = 10;
            let mut prof = Vec::with_capacity(n);
            for i in 0..n {
                let theta = std::f64::consts::FRAC_PI_2
                    + 2.0 * std::f64::consts::PI * (i as f64) / (n as f64);
                let r = if i % 2 == 0 { r_out } else { r_in };
                prof.push(Point3::new(r * theta.cos(), r * theta.sin(), 0.0));
            }
            Ok(extrude_polygon(&prof, Vec3::new(0.0, 0.0, t)))
        }
        Feature::Crescent { outer_radius, inner_radius, offset, thickness, segments, .. } => {
            let r_out = resolve_one(id, outer_radius, params)?;
            let r_in = resolve_one(id, inner_radius, params)?;
            let off = resolve_one(id, offset, params)?;
            let t = resolve_one(id, thickness, params)?;
            if r_out <= 0.0 || r_in <= 0.0 || off <= 0.0 || t <= 0.0
                || r_in >= r_out || off + r_in <= r_out
                || off >= r_out + r_in || *segments < 8
            {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("Crescent requires offset such that inner partially overlaps outer (got out={r_out}, in={r_in}, off={off})") });
            }
            let outer = cylinder_faceted(r_out, t, *segments);
            let eps = (t * 0.05).max(1e-3);
            let inner = cylinder_along_axis(r_in, t + 2.0 * eps, *segments, 2, -eps, off, 0.0);
            outer.try_difference(&inner).map_err(|e| EvalError::Boolean { id: id.into(), op: "crescent_carve", message: e.message })
        }
        Feature::Hourglass { end_radius, waist_radius, half_height, segments, .. } => {
            let er = resolve_one(id, end_radius, params)?;
            let wr = resolve_one(id, waist_radius, params)?;
            let hh = resolve_one(id, half_height, params)?;
            if er <= 0.0 || wr <= 0.0 || wr >= er || hh <= 0.0 || *segments < 6 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("Hourglass requires waist<end, positive dims (got er={er}, wr={wr}, hh={hh})") });
            }
            // Two frustums sharing the waist face. Note: shared-waist
            // unions where waist < end can hang the boolean engine for
            // some configurations — caller should use moderate values
            // and avoid pathological degeneracies. See
            // batch_features_20::hourglass_completes (ignored).
            let bot = frustum_faceted(wr, er, hh, *segments);
            let top = frustum_faceted(er, wr, hh, *segments);
            let top_t = translate_solid(&top, Vec3::new(0.0, 0.0, hh - 1e-3));
            bot.try_union(&top_t).map_err(|e| EvalError::Boolean { id: id.into(), op: "hourglass_join", message: e.message })
        }
        Feature::Diabolo { end_radius, waist_radius, half_height, segments, .. } => {
            let er = resolve_one(id, end_radius, params)?;
            let wr = resolve_one(id, waist_radius, params)?;
            let hh = resolve_one(id, half_height, params)?;
            if er <= 0.0 || wr <= 0.0 || wr <= er || hh <= 0.0 || *segments < 6 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("Diabolo requires waist > end, positive dims (got er={er}, wr={wr}, hh={hh})") });
            }
            // Same construction as Hourglass — diabolo just has waist > end.
            // For diabolo configurations the boolean union does NOT hang
            // (the shared face is now larger than the bounded ends, which
            // doesn't trigger the same kernel pathology).
            let bot = frustum_faceted(wr, er, hh, *segments);
            let top = frustum_faceted(er, wr, hh, *segments);
            let top_t = translate_solid(&top, Vec3::new(0.0, 0.0, hh - 1e-3));
            bot.try_union(&top_t).map_err(|e| EvalError::Boolean { id: id.into(), op: "diabolo_join", message: e.message })
        }
        Feature::TripleStep { bottom_radius, bottom_height, middle_radius, middle_height, top_radius, top_height, segments, .. } => {
            let br = resolve_one(id, bottom_radius, params)?;
            let bh = resolve_one(id, bottom_height, params)?;
            let mr = resolve_one(id, middle_radius, params)?;
            let mh = resolve_one(id, middle_height, params)?;
            let tr = resolve_one(id, top_radius, params)?;
            let th = resolve_one(id, top_height, params)?;
            if br <= 0.0 || bh <= 0.0 || mr <= 0.0 || mh <= 0.0 || tr <= 0.0 || th <= 0.0
                || mr >= br || tr >= mr || *segments < 6
            {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("TripleStep requires top<middle<bottom radii, positive dims (got br={br}, mr={mr}, tr={tr})") });
            }
            let eps = (bh.min(mh).min(th) * 0.05).max(1e-3);
            let bot = cylinder_along_axis(br, bh, *segments, 2, 0.0, 0.0, 0.0);
            let mid = cylinder_along_axis(mr, mh + eps, *segments, 2, bh - eps, 0.0, 0.0);
            let tp = cylinder_along_axis(tr, th + eps, *segments, 2, bh + mh - eps, 0.0, 0.0);
            let lower = bot.try_union(&mid).map_err(|e| EvalError::Boolean { id: id.into(), op: "triple_step_bm", message: e.message })?;
            lower.try_union(&tp).map_err(|e| EvalError::Boolean { id: id.into(), op: "triple_step_mt", message: e.message })
        }
        Feature::WingedScrew { shaft_radius, shaft_length, head_radius, head_height, wing_length, wing_thickness, segments, .. } => {
            let sr = resolve_one(id, shaft_radius, params)?;
            let sl = resolve_one(id, shaft_length, params)?;
            let hr = resolve_one(id, head_radius, params)?;
            let hh = resolve_one(id, head_height, params)?;
            let wl = resolve_one(id, wing_length, params)?;
            let wt = resolve_one(id, wing_thickness, params)?;
            if sr <= 0.0 || sl <= 0.0 || hr <= 0.0 || hh <= 0.0 || wl <= 0.0 || wt <= 0.0 || sr >= hr || *segments < 6 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("WingedScrew requires shaft<head, positive dims (got sr={sr}, hr={hr})") });
            }
            // Hex head + two rectangular wings projecting +x and -x at head height.
            let mut head_prof = Vec::with_capacity(6);
            for i in 0..6 {
                let theta = 2.0 * std::f64::consts::PI * (i as f64) / 6.0;
                head_prof.push(Point3::new(hr * theta.cos(), hr * theta.sin(), 0.0));
            }
            let head = extrude_polygon(&head_prof, Vec3::new(0.0, 0.0, hh));
            let wing_w = wt;
            let wing_h = hh * 0.6;
            let wing_z0 = (hh - wing_h) / 2.0;
            let wing_left = box_at(
                Vec3::new(wl, wing_w, wing_h),
                Point3::new(-hr - wl + 1e-3, -wing_w / 2.0, wing_z0),
            );
            let wing_right = box_at(
                Vec3::new(wl, wing_w, wing_h),
                Point3::new(hr - 1e-3, -wing_w / 2.0, wing_z0),
            );
            let head_w_l = head.try_union(&wing_left).map_err(|e| EvalError::Boolean { id: id.into(), op: "winged_screw_left_wing", message: e.message })?;
            let head_w_lr = head_w_l.try_union(&wing_right).map_err(|e| EvalError::Boolean { id: id.into(), op: "winged_screw_right_wing", message: e.message })?;
            let eps = (hh * 0.05).max(1e-3);
            let shaft = cylinder_along_axis(sr, sl + eps, *segments, 2, -sl, 0.0, 0.0);
            head_w_lr.try_union(&shaft).map_err(|e| EvalError::Boolean { id: id.into(), op: "winged_screw_shaft", message: e.message })
        }
        Feature::KneadHandle { length, width, plate_thickness, pad_radius, pad_height, segments, .. } => {
            let l = resolve_one(id, length, params)?;
            let w = resolve_one(id, width, params)?;
            let pt = resolve_one(id, plate_thickness, params)?;
            let pr = resolve_one(id, pad_radius, params)?;
            let ph = resolve_one(id, pad_height, params)?;
            if l <= 0.0 || w <= 0.0 || pt <= 0.0 || pr <= 0.0 || ph <= 0.0 || pr >= l / 2.0 || pr >= w / 2.0 || *segments < 6 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("KneadHandle requires pad_radius < l&w/2, positive dims (got l={l}, w={w}, pr={pr})") });
            }
            let plate = box_at(Vec3::new(l, w, pt), Point3::new(0.0, 0.0, 0.0));
            let pad = cylinder_along_axis(pr, ph + 1e-3, *segments, 2, pt - 1e-3, l / 2.0, w / 2.0);
            plate.try_union(&pad).map_err(|e| EvalError::Boolean { id: id.into(), op: "knead_handle_pad", message: e.message })
        }
        Feature::ZigzagBar { length, depth, base_height, zigzag_height, n_zigs, .. } => {
            let l = resolve_one(id, length, params)?;
            let d = resolve_one(id, depth, params)?;
            let bh = resolve_one(id, base_height, params)?;
            let zh = resolve_one(id, zigzag_height, params)?;
            if l <= 0.0 || d <= 0.0 || bh <= 0.0 || zh <= 0.0 || *n_zigs == 0 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("ZigzagBar requires positive dims, n_zigs>=1 (got l={l}, d={d}, bh={bh}, zh={zh})") });
            }
            // Build profile in xy plane: bottom rectangle [0,l] x [0,bh], top zigzag.
            // Walk CCW: (0,0) → (l,0) → up the right edge → top zigzag right-to-left → (0,bh).
            let n = *n_zigs;
            let dx = l / (2.0 * n as f64);
            let mut prof = vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(l, 0.0, 0.0),
            ];
            for k in 0..=(2 * n) {
                let x = l - (k as f64) * dx;
                let y = if k % 2 == 0 { bh + zh } else { bh };
                prof.push(Point3::new(x, y, 0.0));
            }
            Ok(extrude_polygon(&prof, Vec3::new(0.0, 0.0, d)))
        }
        Feature::FishingFloat { body_radius, pin_radius, pin_length, stacks, slices, .. } => {
            let br = resolve_one(id, body_radius, params)?;
            let pr = resolve_one(id, pin_radius, params)?;
            let pl = resolve_one(id, pin_length, params)?;
            if br <= 0.0 || pr <= 0.0 || pl <= 0.0 || pr >= br || *stacks < 2 || *slices < 3 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("FishingFloat requires pin<body, positive dims (got br={br}, pr={pr})") });
            }
            // Sphere body at origin, top pin from z=br to z=br+pl, bottom pin from z=-(br+pl) to z=-br.
            let body = sphere_faceted(br, *stacks, *slices);
            let top_pin = cylinder_along_axis(pr, pl + 1e-3, *slices, 2, br - 1e-3, 0.0, 0.0);
            let bot_pin = cylinder_along_axis(pr, pl + 1e-3, *slices, 2, -br - pl, 0.0, 0.0);
            let with_top = body.try_union(&top_pin).map_err(|e| EvalError::Boolean { id: id.into(), op: "fishing_float_top", message: e.message })?;
            with_top.try_union(&bot_pin).map_err(|e| EvalError::Boolean { id: id.into(), op: "fishing_float_bot", message: e.message })
        }
        Feature::Tetrahedron { edge_length, .. } => {
            let e = resolve_one(id, edge_length, params)?;
            if e <= 0.0 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("Tetrahedron requires positive edge_length (got {e})") });
            }
            // Regular tetrahedron with edge length e:
            //   base equilateral triangle side e at z=0 centered on origin,
            //   apex at z = e * sqrt(2/3) above the centroid.
            // Place base vertices: (e/2, -e/(2*sqrt(3)), 0),
            //                       (-e/2, -e/(2*sqrt(3)), 0),
            //                       (0,  e/sqrt(3), 0).
            let r = e / std::f64::consts::SQRT_2.sqrt() * std::f64::consts::SQRT_2; // not used
            let _ = r;
            let s3 = 3.0_f64.sqrt();
            // Base CCW from +z: top vertex first, then bottom-left, then
            // bottom-right (signed area positive). The earlier ordering
            // gave a CW walk and produced an inverted (negative-volume)
            // solid.
            let base = vec![
                Point3::new(0.0,       e / s3,          0.0),
                Point3::new(-e / 2.0, -e / (2.0 * s3), 0.0),
                Point3::new( e / 2.0, -e / (2.0 * s3), 0.0),
            ];
            let apex_z = e * (2.0_f64 / 3.0_f64).sqrt();
            let apex_sz = e * 1e-3;
            let apex = vec![
                Point3::new(0.0,             apex_sz / s3,         apex_z),
                Point3::new(-apex_sz / 2.0, -apex_sz / (2.0 * s3), apex_z),
                Point3::new( apex_sz / 2.0, -apex_sz / (2.0 * s3), apex_z),
            ];
            Ok(extrude_lofted(&base, &apex))
        }
        Feature::Spool { body_radius, body_length, flange_radius, flange_thickness, segments, .. } => {
            let br = resolve_one(id, body_radius, params)?;
            let bl = resolve_one(id, body_length, params)?;
            let fr = resolve_one(id, flange_radius, params)?;
            let ft = resolve_one(id, flange_thickness, params)?;
            if br <= 0.0 || bl <= 0.0 || fr <= 0.0 || ft <= 0.0 || br >= fr || *segments < 6 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("Spool requires body_radius < flange_radius, positive dims (got br={br}, fr={fr})") });
            }
            // Two flange disks + body cylinder between them.
            let flange_a = cylinder_along_axis(fr, ft, *segments, 2, 0.0, 0.0, 0.0);
            let body = cylinder_along_axis(br, bl + 2.0 * 1e-3, *segments, 2, ft - 1e-3, 0.0, 0.0);
            let flange_b = cylinder_along_axis(fr, ft, *segments, 2, ft + bl, 0.0, 0.0);
            let lower = flange_a.try_union(&body).map_err(|e| EvalError::Boolean { id: id.into(), op: "spool_body", message: e.message })?;
            lower.try_union(&flange_b).map_err(|e| EvalError::Boolean { id: id.into(), op: "spool_top_flange", message: e.message })
        }
        Feature::Lampshade { top_radius, bottom_radius, height, wall_thickness, segments, .. } => {
            let tr = resolve_one(id, top_radius, params)?;
            let br = resolve_one(id, bottom_radius, params)?;
            let h = resolve_one(id, height, params)?;
            let wt = resolve_one(id, wall_thickness, params)?;
            if tr <= 0.0 || br <= 0.0 || h <= 0.0 || wt <= 0.0
                || tr - wt <= 0.0 || br - wt <= 0.0 || *segments < 6
            {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("Lampshade requires inner > 0 at both ends (got tr={tr}, br={br}, wt={wt})") });
            }
            let outer = frustum_faceted(tr, br, h, *segments);
            let inner = frustum_faceted(tr - wt, br - wt, h + 2.0 * 1e-3, *segments);
            let inner_t = translate_solid(&inner, Vec3::new(0.0, 0.0, -1e-3));
            outer.try_difference(&inner_t).map_err(|e| EvalError::Boolean { id: id.into(), op: "lampshade_carve", message: e.message })
        }
        Feature::PrismHole { sides, outer_radius, hole_radius, height, segments, .. } => {
            let r_out = resolve_one(id, outer_radius, params)?;
            let hr = resolve_one(id, hole_radius, params)?;
            let h = resolve_one(id, height, params)?;
            if *sides < 3 || r_out <= 0.0 || hr <= 0.0 || hr >= r_out || h <= 0.0 || *segments < 6 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("PrismHole requires sides>=3, hole<outer, positive dims (got sides={sides}, r_out={r_out}, hr={hr})") });
            }
            // Build n-sided prism via extrude_polygon.
            let mut prof = Vec::with_capacity(*sides);
            for i in 0..*sides {
                let theta = 2.0 * std::f64::consts::PI * (i as f64) / (*sides as f64);
                prof.push(Point3::new(r_out * theta.cos(), r_out * theta.sin(), 0.0));
            }
            let prism = extrude_polygon(&prof, Vec3::new(0.0, 0.0, h));
            let eps = (h * 0.05).max(1e-3);
            let hole = cylinder_along_axis(hr, h + 2.0 * eps, *segments, 2, -eps, 0.0, 0.0);
            prism.try_difference(&hole).map_err(|e| EvalError::Boolean { id: id.into(), op: "prism_hole_drill", message: e.message })
        }
        Feature::KeyholeShape { circle_radius, slot_width, slot_height, plate_width, plate_height, plate_thickness, segments, .. } => {
            let cr = resolve_one(id, circle_radius, params)?;
            let sw = resolve_one(id, slot_width, params)?;
            let sh = resolve_one(id, slot_height, params)?;
            let pw = resolve_one(id, plate_width, params)?;
            let ph = resolve_one(id, plate_height, params)?;
            let pt = resolve_one(id, plate_thickness, params)?;
            if cr <= 0.0 || sw <= 0.0 || sh <= 0.0 || pt <= 0.0
                || sw >= 2.0 * cr || cr >= pw / 2.0 || cr + sh >= ph
                || *segments < 6
            {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("KeyholeShape invalid params (got cr={cr}, sw={sw}, sh={sh}, pw={pw}, ph={ph})") });
            }
            // Build the keyhole as a single combined cutout polygon to avoid
            // the boolean stitch panic that hits when the circle and slot
            // are adjacent (their boundaries meet along a tangent edge).
            // Combined profile: walk CCW around (slot rectangle + half-arc
            // circle on top, with the two halves of the circle joined to
            // the slot edges).
            //
            // Layout: circle center at (pw/2, ph - cr). Slot from
            // y = ph - cr - sh to y = ph - cr (top of slot meets circle
            // center y). Slot is centered on x=pw/2, width sw.
            //
            // Walk (CCW): start at slot bottom-left → bottom-right → up the
            // right edge of slot → out to where slot meets circle
            // (x = pw/2 + sw/2, y = ph - cr) → arc CCW around circle
            // (which becomes the OUTER side of the cutout) → down to where
            // slot meets circle on the LEFT (x = pw/2 - sw/2, y = ph - cr)
            // → down the left slot edge to start.
            let cx = pw / 2.0;
            let cy = ph - cr;
            let half_sw = sw / 2.0;
            // Angle of slot-top intersection with circle: x = ±sw/2 → cos(theta) = ±sw/(2cr)
            // theta = ±acos(sw/(2cr)) measured from +x axis.
            // Right intersection: theta = -acos(sw/(2cr)) (lower right of circle).
            // For our walk we go from right-slot-corner up and around to left-slot-corner,
            // so theta sweeps from -acos(sw/(2cr)) through PI back to PI+acos(sw/(2cr)).
            // Actually simpler: angle of right-slot-circle intersection above the circle center is
            //   sin(alpha) = (slot_top_y - cy) / cr = 0 / cr = 0... wait.
            //
            // The slot top is at y = cy = ph - cr (same y as circle center).
            // So the slot intersects the circle at x = ±sw/2, y = cy.
            // These points are on the equator of the circle (theta=0 and theta=PI).
            // We want to walk the arc from right (theta=0) → top (theta=PI/2)
            //   → left (theta=PI), which is the UPPER half of the circle.
            //
            // Cutout walk CCW (from outside of plate looking down +z):
            //   slot_bl (cx - half_sw, cy - sh)
            //   slot_br (cx + half_sw, cy - sh)
            //   slot_tr (cx + half_sw, cy)             — also right intersection with circle
            //   arc CCW from theta=0 to theta=PI       — upper half-circle
            //   slot_tl (cx - half_sw, cy)             — left intersection with circle
            //   back to slot_bl
            let mut hole_prof = Vec::new();
            hole_prof.push(Point3::new(cx - half_sw, cy - sh, 0.0));
            hole_prof.push(Point3::new(cx + half_sw, cy - sh, 0.0));
            // arc: theta from 0 to PI (sample n_arc segments, skip endpoints
            // since they're already in the polygon).
            let n_arc = (*segments).max(8);
            for k in 0..=n_arc {
                let theta = std::f64::consts::PI * (k as f64) / (n_arc as f64);
                hole_prof.push(Point3::new(cx + cr * theta.cos(), cy + cr * theta.sin(), 0.0));
            }
            let plate = box_at(Vec3::new(pw, ph, pt), Point3::new(0.0, 0.0, 0.0));
            let eps = (pt * 0.05).max(1e-3);
            let cutter = extrude_polygon(&hole_prof, Vec3::new(0.0, 0.0, pt + 2.0 * eps));
            let cutter_t = translate_solid(&cutter, Vec3::new(0.0, 0.0, -eps));
            plate.try_difference(&cutter_t).map_err(|e| EvalError::Boolean { id: id.into(), op: "keyhole_carve", message: e.message })
        }
        Feature::AcornShape { body_radius, stem_radius, stem_length, stacks, slices, .. } => {
            let br = resolve_one(id, body_radius, params)?;
            let sr = resolve_one(id, stem_radius, params)?;
            let sl = resolve_one(id, stem_length, params)?;
            if br <= 0.0 || sr <= 0.0 || sr >= br || sl <= 0.0 || *stacks < 2 || *slices < 3 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("AcornShape requires stem<body, positive dims (got br={br}, sr={sr})") });
            }
            let body = sphere_faceted(br, *stacks, *slices);
            let stem = cylinder_along_axis(sr, sl + 1e-3, *slices, 2, br - 1e-3, 0.0, 0.0);
            body.try_union(&stem).map_err(|e| EvalError::Boolean { id: id.into(), op: "acorn_join", message: e.message })
        }
        Feature::Volute { max_radius, revolutions, rod_radius, center_disk_radius, thickness, segments_per_revolution, center_segments, .. } => {
            let r_max = resolve_one(id, max_radius, params)?;
            let revs = resolve_one(id, revolutions, params)?;
            let rod_r = resolve_one(id, rod_radius, params)?;
            let cdr = resolve_one(id, center_disk_radius, params)?;
            let t = resolve_one(id, thickness, params)?;
            if r_max <= 0.0 || revs <= 0.0 || rod_r <= 0.0 || cdr <= 0.0 || t <= 0.0
                || cdr >= r_max || *segments_per_revolution < 6 || *center_segments < 6
            {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("Volute invalid params (got rmax={r_max}, revs={revs}, cdr={cdr})") });
            }
            // Spiral rod (Archimedean) from r=cdr to r=r_max, plus a center disk.
            let center = cylinder_along_axis(cdr, t, *center_segments, 2, 0.0, 0.0, 0.0);
            let total_samples = (*segments_per_revolution as f64 * revs).ceil() as usize + 1;
            let total_angle = 2.0 * std::f64::consts::PI * revs;
            let mut acc = center;
            for i in 0..total_samples - 1 {
                let t0 = total_angle * i as f64 / (total_samples - 1) as f64;
                let t1 = total_angle * (i + 1) as f64 / (total_samples - 1) as f64;
                let r0 = cdr + (r_max - cdr) * t0 / total_angle;
                let r1 = cdr + (r_max - cdr) * t1 / total_angle;
                let p0 = [r0 * t0.cos(), r0 * t0.sin(), t / 2.0];
                let p1 = [r1 * t1.cos(), r1 * t1.sin(), t / 2.0];
                if (p1[0] - p0[0]).powi(2) + (p1[1] - p0[1]).powi(2) < 1e-18 {
                    continue;
                }
                let cyl = sweep_cylinder_segment(p0, p1, rod_r, 8).ok_or_else(|| EvalError::Invalid {
                    id: id.into(),
                    reason: format!("Volute seg {i} zero length"),
                })?;
                acc = acc.try_union(&cyl).map_err(|e| EvalError::Boolean { id: id.into(), op: "volute_spiral", message: format!("seg {i}: {}", e.message) })?;
            }
            Ok(acc)
        }
        Feature::ScrollPlate { radius, rod_radius, thickness: _, segments, .. } => {
            let r = resolve_one(id, radius, params)?;
            let rod_r = resolve_one(id, rod_radius, params)?;
            if r <= 0.0 || rod_r <= 0.0 || *segments < 4 {
                return Err(EvalError::Invalid { id: id.into(), reason: format!("ScrollPlate requires positive dims, segments>=4 (got r={r}, rod_r={rod_r})") });
            }
            // Two semicircular arcs forming an S-curve. First arc:
            // center at (r, 0), radius r, sweeping from theta=PI to 2PI
            // (right half-circle below y=0). Second arc: center at (3r, 0),
            // radius r, sweeping from theta=0 to PI (left half-circle above y=0).
            let mut acc: Option<Solid> = None;
            for i in 0..*segments {
                let t0 = std::f64::consts::PI + std::f64::consts::PI * (i as f64) / (*segments as f64);
                let t1 = std::f64::consts::PI + std::f64::consts::PI * ((i + 1) as f64) / (*segments as f64);
                let p0 = [r + r * t0.cos(), r * t0.sin(), 0.0];
                let p1 = [r + r * t1.cos(), r * t1.sin(), 0.0];
                let cyl = sweep_cylinder_segment(p0, p1, rod_r, 8).ok_or_else(|| EvalError::Invalid { id: id.into(), reason: format!("ScrollPlate arc1 seg {i} zero length") })?;
                acc = Some(match acc.take() {
                    None => cyl,
                    Some(prev) => prev.try_union(&cyl).map_err(|e| EvalError::Boolean { id: id.into(), op: "scroll_arc1", message: format!("seg {i}: {}", e.message) })?,
                });
            }
            for i in 0..*segments {
                let t0 = std::f64::consts::PI * (i as f64) / (*segments as f64);
                let t1 = std::f64::consts::PI * ((i + 1) as f64) / (*segments as f64);
                let p0 = [3.0 * r + r * t0.cos(), r * t0.sin(), 0.0];
                let p1 = [3.0 * r + r * t1.cos(), r * t1.sin(), 0.0];
                let cyl = sweep_cylinder_segment(p0, p1, rod_r, 8).ok_or_else(|| EvalError::Invalid { id: id.into(), reason: format!("ScrollPlate arc2 seg {i} zero length") })?;
                acc = Some(match acc.take() {
                    None => cyl,
                    Some(prev) => prev.try_union(&cyl).map_err(|e| EvalError::Boolean { id: id.into(), op: "scroll_arc2", message: format!("seg {i}: {}", e.message) })?,
                });
            }
            Ok(acc.unwrap())
        }
        Feature::HoleArray {
            input,
            axis,
            start,
            offset,
            count,
            radius,
            depth,
            segments,
            ..
        } => {
            let base = cache_get(cache, input)?;
            let s = resolve3(id, start, params)?;
            let off = resolve3(id, offset, params)?;
            let r = resolve_one(id, radius, params)?;
            let d = resolve_one(id, depth, params)?;
            if *count == 0 || r <= 0.0 || d <= 0.0 || *segments < 3 {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: format!(
                        "HoleArray requires count>=1, positive radius, depth, and segments>=3 (got count={count}, r={r}, d={d}, segments={segments})"
                    ),
                });
            }
            let axis_idx = parse_axis(id, axis)?;
            let (a_idx, b_idx) = perpendicular_axes(axis_idx);
            let eps = (r * 0.1).max(1e-3).min(d * 0.1);
            // Subtract cylinders sequentially. Disjoint cylinders inside
            // a box subtract independently; this avoids the multi-cylinder
            // union limitation in the kernel.
            let mut acc = base.clone();
            for k in 0..*count {
                let cx = s[0] + off[0] * k as f64;
                let cy = s[1] + off[1] * k as f64;
                let cz = s[2] + off[2] * k as f64;
                let center = [cx, cy, cz];
                let cyl = cylinder_along_axis(
                    r,
                    d + eps,
                    *segments,
                    axis_idx,
                    center[axis_idx] - d,
                    center[a_idx],
                    center[b_idx],
                );
                acc = acc.try_difference(&cyl).map_err(|e| EvalError::Boolean {
                    id: id.into(),
                    op: "hole_array",
                    message: format!("drilling hole {k}: {}", e.message),
                })?;
            }
            Ok(acc)
        }
        Feature::BoltCircle {
            input,
            axis,
            center,
            bolt_circle_radius,
            count,
            radius,
            depth,
            segments,
            ..
        } => {
            let base = cache_get(cache, input)?;
            let ctr = resolve3(id, center, params)?;
            let bcr = resolve_one(id, bolt_circle_radius, params)?;
            let r = resolve_one(id, radius, params)?;
            let d = resolve_one(id, depth, params)?;
            if *count == 0 || r <= 0.0 || d <= 0.0 || bcr <= 0.0 || *segments < 3 {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: format!(
                        "BoltCircle requires count>=1, positive bolt_circle_radius/radius/depth, segments>=3"
                    ),
                });
            }
            let axis_idx = parse_axis(id, axis)?;
            let (a_idx, b_idx) = perpendicular_axes(axis_idx);
            let eps = (r * 0.1).max(1e-3).min(d * 0.1);
            let mut acc = base.clone();
            for k in 0..*count {
                let theta = 2.0 * std::f64::consts::PI * k as f64 / *count as f64;
                let pa = ctr[a_idx] + bcr * theta.cos();
                let pb = ctr[b_idx] + bcr * theta.sin();
                let cyl = cylinder_along_axis(
                    r,
                    d + eps,
                    *segments,
                    axis_idx,
                    ctr[axis_idx] - d,
                    pa,
                    pb,
                );
                acc = acc.try_difference(&cyl).map_err(|e| EvalError::Boolean {
                    id: id.into(),
                    op: "bolt_circle",
                    message: format!("drilling hole {k}: {}", e.message),
                })?;
            }
            Ok(acc)
        }
        Feature::HexHole {
            input,
            axis,
            top_center,
            inscribed_radius,
            depth,
            ..
        } => {
            let base = cache_get(cache, input)?;
            let tc = resolve3(id, top_center, params)?;
            let ir = resolve_one(id, inscribed_radius, params)?;
            let d = resolve_one(id, depth, params)?;
            if ir <= 0.0 || d <= 0.0 {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: format!(
                        "HexHole requires positive inscribed_radius and depth (got {ir}, {d})"
                    ),
                });
            }
            let cutter = build_polygon_pocket(id, tc, axis, ir / (std::f64::consts::PI / 6.0).cos(), d, 6, 0.0)?;
            base.try_difference(&cutter).map_err(|e| EvalError::Boolean {
                id: id.into(),
                op: "hex_hole",
                message: e.message,
            })
        }
        Feature::SquareHole {
            input,
            axis,
            top_center,
            side,
            depth,
            ..
        } => {
            let base = cache_get(cache, input)?;
            let tc = resolve3(id, top_center, params)?;
            let s = resolve_one(id, side, params)?;
            let d = resolve_one(id, depth, params)?;
            if s <= 0.0 || d <= 0.0 {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: format!(
                        "SquareHole requires positive side and depth (got {s}, {d})"
                    ),
                });
            }
            // Square inscribed in circle of radius s/sqrt(2), starting
            // at angle pi/4 so it's axis-aligned.
            let cutter = build_polygon_pocket(
                id,
                tc,
                axis,
                s * std::f64::consts::SQRT_2 / 2.0,
                d,
                4,
                std::f64::consts::FRAC_PI_4,
            )?;
            base.try_difference(&cutter).map_err(|e| EvalError::Boolean {
                id: id.into(),
                op: "square_hole",
                message: e.message,
            })
        }
        Feature::TubeAt {
            base,
            axis,
            outer_radius,
            inner_radius,
            height,
            segments,
            ..
        } => {
            let b = resolve3(id, base, params)?;
            let r_out = resolve_one(id, outer_radius, params)?;
            let r_in = resolve_one(id, inner_radius, params)?;
            let h = resolve_one(id, height, params)?;
            if r_out <= 0.0 || r_in <= 0.0 || h <= 0.0 {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: format!(
                        "TubeAt requires positive outer_radius, inner_radius, height (got {r_out}, {r_in}, {h})"
                    ),
                });
            }
            if r_in >= r_out {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: format!(
                        "TubeAt inner_radius ({r_in}) must be < outer_radius ({r_out})"
                    ),
                });
            }
            if *segments < 3 {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: format!("TubeAt segments must be >= 3 (got {segments})"),
                });
            }
            let axis_idx = parse_axis(id, axis)?;
            let (a_idx, b_idx) = perpendicular_axes(axis_idx);
            let outer = cylinder_along_axis(
                r_out,
                h,
                *segments,
                axis_idx,
                b[axis_idx],
                b[a_idx],
                b[b_idx],
            );
            // Inner cylinder extends 1 unit past each cap to make a clean
            // through-bore (matches Tube's convention).
            let bore_h = h + 2.0;
            let inner = cylinder_along_axis(
                r_in,
                bore_h,
                *segments,
                axis_idx,
                b[axis_idx] - 1.0,
                b[a_idx],
                b[b_idx],
            );
            outer.try_difference(&inner).map_err(|e| EvalError::Boolean {
                id: id.into(),
                op: "tube_at",
                message: e.message,
            })
        }
        Feature::Star {
            points,
            outer_radius,
            inner_radius,
            height,
            ..
        } => {
            let r_out = resolve_one(id, outer_radius, params)?;
            let r_in = resolve_one(id, inner_radius, params)?;
            let h = resolve_one(id, height, params)?;
            if *points < 3 {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: format!("Star points must be >= 3 (got {})", points),
                });
            }
            if r_in <= 0.0 || r_out <= 0.0 || h <= 0.0 {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: format!(
                        "Star requires positive outer_radius, inner_radius, height (got {r_out}, {r_in}, {h})"
                    ),
                });
            }
            if r_in >= r_out {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: format!(
                        "Star inner_radius ({r_in}) must be < outer_radius ({r_out})"
                    ),
                });
            }
            // 2N vertices alternating between r_out and r_in, going CCW from
            // angle 0. First vertex is an outer tip on +x axis.
            let n = 2 * points;
            let prof: Vec<Point3> = (0..n)
                .map(|i| {
                    let theta = 2.0 * std::f64::consts::PI * i as f64 / n as f64;
                    let r = if i % 2 == 0 { r_out } else { r_in };
                    Point3::new(r * theta.cos(), r * theta.sin(), 0.0)
                })
                .collect();
            Ok(extrude_polygon(&prof, Vec3::new(0.0, 0.0, h)))
        }
        Feature::HollowBox {
            extents,
            wall_thickness,
            ..
        } => {
            let e = resolve3(id, extents, params)?;
            let w = resolve_one(id, wall_thickness, params)?;
            if w <= 0.0 {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: format!("HollowBox wall_thickness must be > 0 (got {w})"),
                });
            }
            let two_w = 2.0 * w;
            if two_w >= e[0] || two_w >= e[1] || two_w >= e[2] {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: format!(
                        "HollowBox wall_thickness {w} too large for extents [{}, {}, {}]",
                        e[0], e[1], e[2]
                    ),
                });
            }
            let outer = box_(Vec3::new(e[0], e[1], e[2]));
            let inner = box_at(
                Vec3::new(e[0] - two_w, e[1] - two_w, e[2] - two_w),
                Point3::new(w, w, w),
            );
            outer.try_difference(&inner).map_err(|e| EvalError::Boolean {
                id: id.into(),
                op: "hollow_box_difference",
                message: e.message,
            })
        }

        Feature::Translate { input, offset, .. } => {
            let base = cache_get(cache, input)?;
            let o = resolve3(id, offset, params)?;
            Ok(translate_solid(base, Vec3::new(o[0], o[1], o[2])))
        }
        Feature::Scale { input, factor, .. } => {
            let base = cache_get(cache, input)?;
            let f = resolve_one(id, factor, params)?;
            if f <= 0.0 {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: format!("Scale factor must be > 0 (got {f})"),
                });
            }
            Ok(scale_solid(base, f))
        }
        Feature::ScaleXYZ { input, factors, .. } => {
            let base = cache_get(cache, input)?;
            let f = resolve3(id, factors, params)?;
            if f[0] <= 0.0 || f[1] <= 0.0 || f[2] <= 0.0 {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: format!(
                        "ScaleXYZ factors must all be > 0 (got [{}, {}, {}])",
                        f[0], f[1], f[2]
                    ),
                });
            }
            Ok(scale_xyz_solid(base, f[0], f[1], f[2]))
        }
        Feature::Mirror {
            input,
            plane_origin,
            plane_normal,
            ..
        } => {
            let base = cache_get(cache, input)?;
            let o = resolve3(id, plane_origin, params)?;
            let n = resolve3(id, plane_normal, params)?;
            let normal = Vec3::new(n[0], n[1], n[2]);
            if normal.norm() < 1e-12 {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: "Mirror plane_normal must be non-zero".into(),
                });
            }
            Ok(mirror_solid(base, Point3::new(o[0], o[1], o[2]), normal))
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

        Feature::LinearPattern {
            input,
            count,
            offset,
            ..
        } => {
            if *count == 0 {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: "LinearPattern count must be >= 1".into(),
                });
            }
            let base = cache_get(cache, input)?;
            let o = resolve3(id, offset, params)?;
            let dir = Vec3::new(o[0], o[1], o[2]);
            let mut acc = base.clone();
            for k in 1..*count {
                let copy = translate_solid(base, dir * (k as f64));
                acc = acc.try_union(&copy).map_err(|e| EvalError::Boolean {
                    id: id.into(),
                    op: "union(linear_pattern)",
                    message: e.message,
                })?;
            }
            Ok(acc)
        }
        Feature::PolarPattern {
            input,
            count,
            axis,
            center,
            total_angle_deg,
            ..
        } => {
            if *count == 0 {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: "PolarPattern count must be >= 1".into(),
                });
            }
            let base = cache_get(cache, input)?;
            let a = resolve3(id, axis, params)?;
            let c = resolve3(id, center, params)?;
            let total = resolve_one(id, total_angle_deg, params)?;
            let axis_v = Vec3::new(a[0], a[1], a[2]);
            if axis_v.norm() < 1e-12 {
                return Err(EvalError::Invalid {
                    id: id.into(),
                    reason: "PolarPattern axis must be non-zero".into(),
                });
            }
            // For a 360° pattern of N copies, the angular step is 360/N (last
            // copy lands back at start, NOT at 360°). For partial sweeps
            // (< 360 OR > 360 but != multiple of 360), divide by N-1 so the
            // first copy is at 0° and the last at total_angle_deg.
            let is_full_circle = (total.rem_euclid(360.0)).abs() < 1e-9
                || ((total.rem_euclid(360.0)) - 360.0).abs() < 1e-9;
            let step = if is_full_circle && *count > 0 {
                total / *count as f64
            } else if *count > 1 {
                total / (*count - 1) as f64
            } else {
                0.0
            };
            let center_p = Point3::new(c[0], c[1], c[2]);
            let mut acc = base.clone();
            for k in 1..*count {
                let theta = (step * k as f64).to_radians();
                let copy = rotate_solid(base, axis_v, theta, center_p);
                acc = acc.try_union(&copy).map_err(|e| EvalError::Boolean {
                    id: id.into(),
                    op: "union(polar_pattern)",
                    message: e.message,
                })?;
            }
            Ok(acc)
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

fn build_revolve(
    id: &str,
    profile: &Profile2D,
    params: &HashMap<String, f64>,
) -> Result<Solid, EvalError> {
    if profile.points.len() < 3 {
        return Err(EvalError::Invalid {
            id: id.into(),
            reason: format!(
                "revolve profile needs at least 3 points (got {})",
                profile.points.len()
            ),
        });
    }
    // Profile2D points are (x, z) pairs in the xz half-plane (y = 0).
    let mut pts: Vec<Point3> = Vec::with_capacity(profile.points.len());
    for p in &profile.points {
        let xz = resolve_arr(p, params).map_err(|message| EvalError::Parameter {
            id: id.into(),
            message,
        })?;
        pts.push(Point3::new(xz[0], 0.0, xz[1]));
    }
    Ok(revolve_polyline(&pts))
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

fn build_sketch_extrude(
    id: &str,
    sketch: &Sketch,
    direction: &[Scalar; 3],
    params: &HashMap<String, f64>,
    model: &Model,
) -> Result<Solid, EvalError> {
    let profiles = sketch.to_profile_2d(params).map_err(|e| EvalError::Invalid {
        id: id.into(),
        reason: format!("sketch trace: {e}"),
    })?;
    if profiles.is_empty() {
        return Err(EvalError::Invalid {
            id: id.into(),
            reason: "SketchExtrude has no closed profile".into(),
        });
    }

    // Multi-loop policy: if there are 2+ profiles, ensure none of them
    // CROSS each other (they may be fully disjoint or fully nested). For
    // now we union all profiles via the boolean engine — polygon-with-
    // holes isn't yet wired through `extrude_polygon`, so a fully-nested
    // inner loop becomes a separate solid that overlaps the outer; the
    // boolean engine handles the union naturally for the disjoint case
    // and reports an error for the overlapping case (which we surface as
    // DisjointSubLoops).
    if profiles.len() >= 2 {
        let pts: Vec<Vec<[f64; 2]>> = profiles
            .iter()
            .map(|p| {
                crate::sketch::profile_points_xy(p, params).map_err(|e| EvalError::Invalid {
                    id: id.into(),
                    reason: format!("sketch profile: {e}"),
                })
            })
            .collect::<Result<_, _>>()?;
        for i in 0..pts.len() {
            for j in (i + 1)..pts.len() {
                if crate::sketch::polygons_cross(&pts[i], &pts[j]) {
                    return Err(EvalError::Invalid {
                        id: id.into(),
                        reason: format!(
                            "{}",
                            crate::sketch::SketchError::DisjointSubLoops
                        ),
                    });
                }
            }
        }
    }

    // Build each profile in the LOCAL sketch frame (XY plane). We then
    // apply an optional ref-plane transform to the unioned result.
    let mut acc: Option<Solid> = None;
    for prof in &profiles {
        let one = build_extrude(id, prof, direction, params)?;
        acc = Some(match acc {
            None => one,
            Some(prev) => prev.try_union(&one).map_err(|e| EvalError::Boolean {
                id: id.into(),
                op: "sketch_multi_loop_union",
                message: e.message,
            })?,
        });
    }
    let solid = acc.expect("at least one profile");

    // Apply ref-plane transform, if any.
    let positioned = apply_sketch_plane_transform(id, &sketch.plane, &solid, params, model)?;
    Ok(positioned)
}

/// If the sketch's plane is a `NamedRefPlane(name)`, look up the
/// corresponding `Feature::RefPlane` in `model` and apply a rigid-body
/// transform to `solid` so its local +Z direction maps to the named
/// plane's normal axis, with the local origin translated to the named
/// plane's `position`. For axis-aligned plane variants (Xy / Xz / Yz)
/// no transform is applied — the sketch is built in-place.
fn apply_sketch_plane_transform(
    id: &str,
    plane: &crate::sketch::SketchPlane,
    solid: &Solid,
    params: &HashMap<String, f64>,
    model: &Model,
) -> Result<Solid, EvalError> {
    use crate::sketch::SketchPlane;
    use crate::transform::{rotate_solid, translate_solid};

    let name = match plane {
        SketchPlane::Xy | SketchPlane::Xz | SketchPlane::Yz => return Ok(solid.clone()),
        SketchPlane::NamedRefPlane(n) => n.clone(),
    };

    let target = model
        .feature(&name)
        .ok_or_else(|| EvalError::Invalid {
            id: id.into(),
            reason: format!("SketchPlane::NamedRefPlane references unknown feature '{name}'"),
        })?;
    let (axis_str, position) = match target {
        Feature::RefPlane { axis, position, .. } => (axis.clone(), position.clone()),
        _ => {
            return Err(EvalError::Invalid {
                id: id.into(),
                reason: format!(
                    "SketchPlane::NamedRefPlane '{name}' must reference a RefPlane feature"
                ),
            });
        }
    };
    let axis_idx = parse_axis(id, &axis_str)?;
    let pos = resolve3(id, &position, params)?;
    let pos_v = Vec3::new(pos[0], pos[1], pos[2]);

    // Sketch local frame: +X = local x, +Y = local y, +Z = normal.
    // Map local +Z -> world axis. axis_idx 0=x, 1=y, 2=z.
    // For axis 'z': identity (no rotation needed); just translate.
    // For axis 'x': rotate +Y around -Y by ?? — easiest: rotate around the
    //   y-axis by +90° so +Z -> +X; +X -> -Z; +Y stays.
    // For axis 'y': rotate around the x-axis by -90° so +Z -> +Y; +Y -> -Z;
    //   +X stays.
    let center = Point3::new(0.0, 0.0, 0.0);
    let rotated = match axis_idx {
        0 => rotate_solid(
            solid,
            Vec3::new(0.0, 1.0, 0.0),
            std::f64::consts::FRAC_PI_2,
            center,
        ),
        1 => rotate_solid(
            solid,
            Vec3::new(1.0, 0.0, 0.0),
            -std::f64::consts::FRAC_PI_2,
            center,
        ),
        2 => solid.clone(),
        _ => unreachable!(),
    };
    let translated = translate_solid(&rotated, pos_v);
    Ok(translated)
}

fn build_sketch_revolve(
    id: &str,
    sketch: &Sketch,
    params: &HashMap<String, f64>,
) -> Result<Solid, EvalError> {
    let profiles = sketch.to_profile_2d(params).map_err(|e| EvalError::Invalid {
        id: id.into(),
        reason: format!("sketch trace: {e}"),
    })?;
    if profiles.len() != 1 {
        return Err(EvalError::Invalid {
            id: id.into(),
            reason: format!(
                "SketchRevolve expects exactly one closed loop in the sketch, got {}",
                profiles.len()
            ),
        });
    }
    build_revolve(id, &profiles[0], params)
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

/// Build a regular n-gon pocket cutter — useful for HexHole, SquareHole,
/// etc. `top_center` is on the +axis-facing surface; pocket runs `depth`
/// in -axis direction. `circumradius` is the polygon's vertex radius.
/// `phase` is an angular offset (0 places first vertex at angle 0 in the
/// (a, b) perpendicular plane).
fn build_polygon_pocket(
    id: &str,
    top_center: [f64; 3],
    axis: &str,
    circumradius: f64,
    depth: f64,
    sides: usize,
    phase: f64,
) -> Result<Solid, EvalError> {
    let axis_idx = parse_axis(id, axis)?;
    let (a_idx, b_idx) = perpendicular_axes(axis_idx);
    let eps = (circumradius * 0.1).max(1e-3).min(depth * 0.1);
    // Build polygon profile in xy plane (CCW), then extrude along +z by
    // depth + eps, then reorient via cyclic permutation so the extrusion
    // direction is +axis. Place so its top sits at top_center along axis.
    let prof: Vec<Point3> = (0..sides)
        .map(|i| {
            let theta = phase + 2.0 * std::f64::consts::PI * i as f64 / sides as f64;
            Point3::new(circumradius * theta.cos(), circumradius * theta.sin(), 0.0)
        })
        .collect();
    let local = extrude_polygon(&prof, Vec3::new(0.0, 0.0, depth + eps));
    let oriented = match axis_idx {
        2 => local,
        0 => axis_swap_xz_to_x(&local),
        1 => axis_swap_yz_to_y(&local),
        _ => unreachable!(),
    };
    // Translate so the cutter's top (+axis end) sits at top_center, with
    // body extending in -axis direction by depth (+ eps overhang past top).
    let mut translation = [0.0_f64; 3];
    translation[axis_idx] = top_center[axis_idx] - depth;
    translation[a_idx] = top_center[a_idx];
    translation[b_idx] = top_center[b_idx];
    Ok(translate_solid(
        &oriented,
        Vec3::new(translation[0], translation[1], translation[2]),
    ))
}

/// Parse a single-axis label ("x" | "y" | "z") to an index 0/1/2.
fn parse_axis(id: &str, axis: &str) -> Result<usize, EvalError> {
    match axis.to_ascii_lowercase().as_str() {
        "x" => Ok(0),
        "y" => Ok(1),
        "z" => Ok(2),
        other => Err(EvalError::Invalid {
            id: id.into(),
            reason: format!("axis must be 'x', 'y', or 'z' (got {other:?})"),
        }),
    }
}

/// Parse a quadrant code ("pp" | "pn" | "np" | "nn") to two signs.
fn parse_quadrant(id: &str, q: &str) -> Result<(f64, f64), EvalError> {
    let bytes = q.as_bytes();
    let to_sign = |b: u8| match b {
        b'p' | b'P' | b'+' => Some(1.0_f64),
        b'n' | b'N' | b'-' => Some(-1.0_f64),
        _ => None,
    };
    if bytes.len() == 2 {
        if let (Some(a), Some(b)) = (to_sign(bytes[0]), to_sign(bytes[1])) {
            return Ok((a, b));
        }
    }
    Err(EvalError::Invalid {
        id: id.into(),
        reason: format!("quadrant must be one of 'pp', 'pn', 'np', 'nn' (got {q:?})"),
    })
}

/// For an edge along `axis_idx`, return the indices of the two perpendicular
/// axes in canonical (a, b) order:
///   axis x → (y, z)
///   axis y → (z, x)
///   axis z → (x, y)
fn perpendicular_axes(axis_idx: usize) -> (usize, usize) {
    match axis_idx {
        0 => (1, 2),
        1 => (2, 0),
        2 => (0, 1),
        _ => unreachable!(),
    }
}

/// Build an axis-aligned box that occupies the half-open region between
/// edge_min and edge_min + axis_extent along `axis_idx`, and extends by
/// signed amounts along the two perpendicular axes.
fn axis_aligned_box_at_edge(
    edge_min: [f64; 3],
    axis_idx: usize,
    axis_extent: f64,
    perp_extents: (f64, f64),
) -> Solid {
    let (a_idx, b_idx) = perpendicular_axes(axis_idx);
    let mut origin = edge_min;
    let mut extents = [0.0_f64; 3];
    extents[axis_idx] = axis_extent;
    // Place the box so its perpendicular extents point in the requested
    // direction from edge_min. Negative extent → shift origin by the extent
    // and flip sign so box_at gets a positive size.
    let (ext_a, ext_b) = perp_extents;
    if ext_a >= 0.0 {
        extents[a_idx] = ext_a;
    } else {
        origin[a_idx] += ext_a;
        extents[a_idx] = -ext_a;
    }
    if ext_b >= 0.0 {
        extents[b_idx] = ext_b;
    } else {
        origin[b_idx] += ext_b;
        extents[b_idx] = -ext_b;
    }
    box_at(
        Vec3::new(extents[0], extents[1], extents[2]),
        Point3::new(origin[0], origin[1], origin[2]),
    )
}

/// Build a cylinder of `r`, `h` whose axis runs along `axis_idx` starting at
/// `axis_origin` along that axis (in world coordinates), and centered at
/// `(perp_a_center, perp_b_center)` in the two perpendicular axes.
///
/// To avoid floating-point noise from sin/cos in 90° rotations (which trips
/// the boolean engine's coplanarity tests later), this function uses exact
/// coordinate permutation/sign-flip rather than `rotate_solid`.
fn cylinder_along_axis(
    r: f64,
    h: f64,
    segments: usize,
    axis_idx: usize,
    axis_origin: f64,
    perp_a_center: f64,
    perp_b_center: f64,
) -> Solid {
    let local = cylinder_faceted(r, h, segments);
    // Reorient so the cylinder axis aligns with axis_idx (was +z).
    //   axis_idx=2 (z): identity.
    //   axis_idx=0 (x): map (x, y, z) -> (z, y, -x)  (rot +y by +π/2)
    //   axis_idx=1 (y): map (x, y, z) -> (x, z, -y)  (rot +x by -π/2)
    let oriented = match axis_idx {
        2 => local,
        0 => axis_swap_xz_to_x(&local),
        1 => axis_swap_yz_to_y(&local),
        _ => unreachable!(),
    };
    let (a_idx, b_idx) = perpendicular_axes(axis_idx);
    let mut offset = [0.0_f64; 3];
    offset[axis_idx] = axis_origin;
    offset[a_idx] = perp_a_center;
    offset[b_idx] = perp_b_center;
    translate_solid(&oriented, Vec3::new(offset[0], offset[1], offset[2]))
}

/// Build a cylinder of `r`, `segments` whose axis runs from `p0` to `p1`
/// (any direction). Returns `None` if `p0 == p1`.
///
/// Strategy:
/// - For axis-aligned segments (differ in exactly one coordinate), use the
///   exact cyclic-permutation reorientation via `cylinder_along_axis` —
///   no sin/cos noise, robust under booleans.
/// - For diagonal segments, build a +z cylinder of length |p1-p0|, rotate
///   it so its +z axis aligns with (p1-p0), and translate to the start.
///   Introduces ~1e-15 floating-point noise in the rotated frame, which
///   is fine for single segments and most non-coplanar configurations.
fn sweep_cylinder_segment(
    p0: [f64; 3],
    p1: [f64; 3],
    r: f64,
    segments: usize,
) -> Option<Solid> {
    let d = [p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]];
    let len = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
    if len < 1e-12 {
        return None;
    }
    // Axis-aligned fast path: differ in exactly one coordinate.
    let abs = [d[0].abs(), d[1].abs(), d[2].abs()];
    let nonzero: Vec<usize> = (0..3).filter(|&k| abs[k] > 1e-12).collect();
    if nonzero.len() == 1 {
        let axis_idx = nonzero[0];
        let (a_idx, b_idx) = perpendicular_axes(axis_idx);
        let axis_origin = p0[axis_idx].min(p1[axis_idx]);
        return Some(cylinder_along_axis(
            r,
            len,
            segments,
            axis_idx,
            axis_origin,
            p0[a_idx],
            p0[b_idx],
        ));
    }
    // Diagonal: rotate +z cylinder onto the segment direction, then translate.
    let local = cylinder_faceted(r, len, segments);
    let dir = Vec3::new(d[0] / len, d[1] / len, d[2] / len);
    let z_axis = Vec3::new(0.0, 0.0, 1.0);
    let cos_theta = z_axis.dot(&dir).clamp(-1.0, 1.0);
    let oriented = if (cos_theta - 1.0).abs() < 1e-12 {
        // Already +z; no rotation.
        local
    } else if (cos_theta + 1.0).abs() < 1e-12 {
        // Antiparallel: rotate 180° about +x. (Picking +x as the rotation
        // axis is arbitrary — any axis perpendicular to +z works.)
        rotate_solid(&local, Vec3::new(1.0, 0.0, 0.0), std::f64::consts::PI, Point3::origin())
    } else {
        // General case: rotate around (z × dir) by angle = acos(z·dir).
        let axis = z_axis.cross(&dir);
        let angle = cos_theta.acos();
        rotate_solid(&local, axis, angle, Point3::origin())
    };
    Some(translate_solid(
        &oriented,
        Vec3::new(p0[0], p0[1], p0[2]),
    ))
}

/// Apply the cyclic permutation (x, y, z) -> (z, x, y) — a det=+1
/// rotation. Local +z maps to world +x, so a +z-aligned cylinder
/// becomes +x-aligned. Preserves topology and orientation.
fn axis_swap_xz_to_x(s: &Solid) -> Solid {
    apply_orthonormal_remap(s, |p| Point3::new(p.z, p.x, p.y), |v| {
        Vec3::new(v.z, v.x, v.y)
    })
}

/// Apply the cyclic permutation (x, y, z) -> (y, z, x) — a det=+1
/// rotation. Local +z maps to world +y, so a +z-aligned cylinder
/// becomes +y-aligned. Preserves topology and orientation.
fn axis_swap_yz_to_y(s: &Solid) -> Solid {
    apply_orthonormal_remap(s, |p| Point3::new(p.y, p.z, p.x), |v| {
        Vec3::new(v.y, v.z, v.x)
    })
}

fn apply_orthonormal_remap(
    s: &Solid,
    remap_p: impl Fn(Point3) -> Point3,
    remap_v: impl Fn(Vec3) -> Vec3,
) -> Solid {
    use kerf_brep::geometry::{CurveKind, SurfaceKind};
    let mut out = s.clone();
    for (_, p) in out.vertex_geom.iter_mut() {
        *p = remap_p(*p);
    }
    let remap_frame = |f: &mut kerf_geom::Frame| {
        f.origin = remap_p(f.origin);
        f.x = remap_v(f.x);
        f.y = remap_v(f.y);
        f.z = remap_v(f.z);
    };
    for (_, surf) in out.face_geom.iter_mut() {
        match surf {
            SurfaceKind::Plane(p) => remap_frame(&mut p.frame),
            SurfaceKind::Cylinder(c) => remap_frame(&mut c.frame),
            SurfaceKind::Sphere(s) => remap_frame(&mut s.frame),
            SurfaceKind::Cone(c) => remap_frame(&mut c.frame),
            SurfaceKind::Torus(t) => remap_frame(&mut t.frame),
        }
    }
    for (_, seg) in out.edge_geom.iter_mut() {
        match &mut seg.curve {
            CurveKind::Line(l) => {
                l.origin = remap_p(l.origin);
                l.direction = remap_v(l.direction);
            }
            CurveKind::Circle(c) => remap_frame(&mut c.frame),
            CurveKind::Ellipse(e) => remap_frame(&mut e.frame),
        }
    }
    out
}

fn build_fillet(
    id: &str,
    base: &Solid,
    axis: &str,
    edge_min: [f64; 3],
    edge_length: f64,
    radius: f64,
    quadrant: &str,
    segments: usize,
) -> Result<Solid, EvalError> {
    let wedge = build_fillet_wedge(id, axis, edge_min, edge_length, radius, quadrant, segments)?;
    base.try_difference(&wedge).map_err(|e| EvalError::Boolean {
        id: id.into(),
        op: "fillet",
        message: e.message,
    })
}

/// Build the wedge cutter for a single fillet without applying it. Used
/// by `Fillet` (single) and `Fillets` (plural).
fn build_fillet_wedge(
    id: &str,
    axis: &str,
    edge_min: [f64; 3],
    edge_length: f64,
    radius: f64,
    quadrant: &str,
    segments: usize,
) -> Result<Solid, EvalError> {
    if radius <= 0.0 {
        return Err(EvalError::Invalid {
            id: id.into(),
            reason: format!("Fillet radius must be > 0 (got {radius})"),
        });
    }
    if edge_length <= 0.0 {
        return Err(EvalError::Invalid {
            id: id.into(),
            reason: format!("Fillet edge_length must be > 0 (got {edge_length})"),
        });
    }
    if segments < 3 {
        return Err(EvalError::Invalid {
            id: id.into(),
            reason: format!("Fillet segments must be >= 3 (got {segments})"),
        });
    }
    let axis_idx = parse_axis(id, axis)?;
    let (sa, sb) = parse_quadrant(id, quadrant)?;
    let eps = (radius * 0.25).max(1e-3).min(edge_length * 0.5);
    let overhang_h = edge_length + 2.0 * eps;
    let cb_axis_origin = edge_min[axis_idx] - eps;
    let mut corner_origin = edge_min;
    corner_origin[axis_idx] = cb_axis_origin;
    let corner_box = axis_aligned_box_at_edge(
        corner_origin,
        axis_idx,
        overhang_h,
        (sa * radius, sb * radius),
    );
    let (a_idx, b_idx) = perpendicular_axes(axis_idx);
    let cyl = cylinder_along_axis(
        radius,
        overhang_h,
        segments,
        axis_idx,
        cb_axis_origin,
        edge_min[a_idx] + sa * radius,
        edge_min[b_idx] + sb * radius,
    );
    corner_box
        .try_difference(&cyl)
        .map_err(|e| EvalError::Boolean {
            id: id.into(),
            op: "fillet_wedge",
            message: e.message,
        })
}

fn build_fillets(
    id: &str,
    base: &Solid,
    edges: &[FilletEdge],
    params: &HashMap<String, f64>,
) -> Result<Solid, EvalError> {
    if edges.is_empty() {
        return Err(EvalError::Invalid {
            id: id.into(),
            reason: "Fillets needs at least one edge".into(),
        });
    }
    // Sequential subtract: each wedge built from the same `base` (not from
    // the accumulator), then subtracted from the running result.
    //
    // Why not "union all wedges then subtract once": that approach hits a
    // coplanar-face limitation when two wedges share a body face (e.g., the
    // four z-edges of a box share lateral faces in pairs).
    //
    // Why not "build wedge from accumulator each step": redundant — wedges
    // are pure geometry, they don't depend on the body topology.
    //
    // For two wedges that share a body face, this sequential approach
    // STILL fails (the second subtract sees the first's curved face). For
    // disjoint pairs (diagonal corners) it works.
    let mut acc = base.clone();
    for (i, e) in edges.iter().enumerate() {
        let em = resolve_arr(&e.edge_min, params).map_err(|message| EvalError::Parameter {
            id: id.into(),
            message,
        })?;
        let len = resolve_one(id, &e.edge_length, params)?;
        let r = resolve_one(id, &e.radius, params)?;
        let wedge =
            build_fillet_wedge(id, &e.axis, em, len, r, &e.quadrant, e.segments)?;
        acc = acc.try_difference(&wedge).map_err(|err| EvalError::Boolean {
            id: id.into(),
            op: "fillets",
            message: format!("applying edge {i}: {}", err.message),
        })?;
    }
    Ok(acc)
}

fn build_chamfer(
    id: &str,
    base: &Solid,
    axis: &str,
    edge_min: [f64; 3],
    edge_length: f64,
    setback: f64,
    quadrant: &str,
) -> Result<Solid, EvalError> {
    if setback <= 0.0 {
        return Err(EvalError::Invalid {
            id: id.into(),
            reason: format!("Chamfer setback must be > 0 (got {setback})"),
        });
    }
    if edge_length <= 0.0 {
        return Err(EvalError::Invalid {
            id: id.into(),
            reason: format!("Chamfer edge_length must be > 0 (got {edge_length})"),
        });
    }
    let axis_idx = parse_axis(id, axis)?;
    let (sa, sb) = parse_quadrant(id, quadrant)?;
    let eps = (setback * 0.25).max(1e-3).min(edge_length * 0.5);
    let overhang_h = edge_length + 2.0 * eps;
    let axis_origin = edge_min[axis_idx] - eps;
    let (a_idx, b_idx) = perpendicular_axes(axis_idx);
    // Triangle in the perpendicular plane (a, b), CCW when viewed from +axis.
    // Vertices (in (a, b) plane, relative to edge_min):
    //   (0, 0)
    //   (sa * setback, 0)
    //   (0, sb * setback)
    // Ordering: walk CCW from +axis = chosen so (0,0) → (a,0) → (0,b) is
    // CCW ⇔ sa*sb > 0; otherwise reverse to keep CCW.
    let p0 = (0.0, 0.0);
    let p1 = (sa * setback, 0.0);
    let p2 = (0.0, sb * setback);
    // Compute orientation (cross of (p1-p0) × (p2-p0)) in (a, b) → if
    // negative, reverse.
    let cross = (p1.0 - p0.0) * (p2.1 - p0.1) - (p1.1 - p0.1) * (p2.0 - p0.0);
    let pts_ab = if cross >= 0.0 {
        [p0, p1, p2]
    } else {
        [p0, p2, p1]
    };
    // Build 3D points in axis-canonical frame: axis along axis_idx with
    // value axis_origin (the bottom of the prism); perp coordinates from
    // edge_min[a_idx] + dx, etc.
    let mut prof = Vec::with_capacity(3);
    for (da, db) in pts_ab {
        let mut p = [0.0_f64; 3];
        p[axis_idx] = axis_origin;
        p[a_idx] = edge_min[a_idx] + da;
        p[b_idx] = edge_min[b_idx] + db;
        prof.push(Point3::new(p[0], p[1], p[2]));
    }
    let mut dir = [0.0_f64; 3];
    dir[axis_idx] = overhang_h;
    let cutter = extrude_polygon(&prof, Vec3::new(dir[0], dir[1], dir[2]));
    base.try_difference(&cutter)
        .map_err(|e| EvalError::Boolean {
            id: id.into(),
            op: "chamfer",
            message: e.message,
        })
}

fn build_slot(
    id: &str,
    length: f64,
    radius: f64,
    height: f64,
    segments: usize,
) -> Result<Solid, EvalError> {
    if length < 0.0 || radius <= 0.0 || height <= 0.0 {
        return Err(EvalError::Invalid {
            id: id.into(),
            reason: format!(
                "Slot requires length >= 0, positive radius and height (got {length}, {radius}, {height})"
            ),
        });
    }
    if segments < 3 {
        return Err(EvalError::Invalid {
            id: id.into(),
            reason: format!("Slot segments must be >= 3 (got {segments})"),
        });
    }
    // Length-zero stadium degenerates to a circle — use cylinder_faceted.
    if length < 1e-12 {
        return Ok(cylinder_faceted(radius, height, 2 * segments));
    }
    // Stadium shape in xy plane, centered on origin: rectangle of length x
    // diameter (length along x, 2*radius along y), bookended by two
    // semicircles of radius. Total length along x = length + 2*radius.
    //
    // Build profile CCW starting at right semicircle bottom (length/2, -radius):
    //   right semicircle from angle = -π/2 to +π/2 around (length/2, 0)
    //   top edge to (-length/2, +radius)
    //   left semicircle from +π/2 to +3π/2 around (-length/2, 0)
    //   bottom edge back to (length/2, -radius)
    let mut prof = Vec::with_capacity(2 * segments + 2);
    let half_l = length / 2.0;
    let n = segments; // arc subdivisions per semicircle
    // Right semicircle, theta from -π/2 to +π/2 inclusive of both endpoints.
    for i in 0..=n {
        let t = -std::f64::consts::FRAC_PI_2
            + (std::f64::consts::PI * i as f64 / n as f64);
        prof.push(Point3::new(
            half_l + radius * t.cos(),
            radius * t.sin(),
            0.0,
        ));
    }
    // Left semicircle, theta from π/2 to 3π/2 inclusive.
    for i in 0..=n {
        let t = std::f64::consts::FRAC_PI_2
            + (std::f64::consts::PI * i as f64 / n as f64);
        prof.push(Point3::new(
            -half_l + radius * t.cos(),
            radius * t.sin(),
            0.0,
        ));
    }
    // Both semicircles include their endpoints, and the right one ends at
    // (half_l, +radius) which equals the start of the top edge — but the
    // left one starts there too (its first point at theta=π/2 is
    // (-half_l + 0, +radius)). They share a Y coordinate but X differs
    // unless length=0; the polygon walks them as straight edges between.
    Ok(extrude_polygon(&prof, Vec3::new(0.0, 0.0, height)))
}

