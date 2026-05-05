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

        let mut result = build(feature, &self.parameters, cache)?;
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

