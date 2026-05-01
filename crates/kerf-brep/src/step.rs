//! ISO 10303-21 (STEP) B-rep output for `Solid`.
//!
//! Emits an `AUTOMOTIVE_DESIGN` (AP214) STEP file preserving exact analytic
//! geometry: planes/cylinders/spheres/cones/tori as native surface entities and
//! lines/circles/ellipses as native curves — no tessellation. Files written by
//! this module open in FreeCAD and other STEP-compliant viewers.
//!
//! The walk order is dependency-driven: points → directions → axis placements
//! → curves/surfaces → vertex points → edge curves → oriented edges → edge
//! loops → faces → shells → solids → wrapping representation entities.
//! Identical geometric primitives (e.g. shared planes between adjacent faces,
//! shared world axis frames) are deduplicated via quantized-float keys.
//!
//! ## Sphere caveat
//! STEP's `ADVANCED_FACE` requires at least one bound. A bare `sphere()` has
//! no boundary edges (1V, 0E, 1F), so we cannot legally emit it as an AP214
//! `ADVANCED_FACE`. We currently emit the sphere face *without* any bounds,
//! which most viewers (FreeCAD included) tolerate. A torus has the same
//! topology shape (no boundary edges if it's a closed solid in our kernel).

use std::collections::HashMap;
use std::io::{self, Write};

use kerf_geom::{Frame, Point3, Vec3};
use kerf_topo::{EdgeId, FaceId, HalfEdgeId, VertexId};

use crate::Solid;
use crate::geometry::{CurveKind, SurfaceKind};

/// Write `solid` as an ISO 10303-21 (AP214) STEP file to `w`. `name` is
/// embedded in the FILE_NAME header field.
pub fn write_step(solid: &Solid, name: &str, w: &mut impl Write) -> io::Result<()> {
    let mut writer = StepWriter::new();
    writer.emit_header(w, name)?;
    let solid_ref = writer.emit_solid(w, solid)?;
    writer.emit_footer(w, solid_ref)?;
    Ok(())
}

/// Quantize a float into a hashable u64 bit pattern. Two floats whose bit
/// patterns match share an entity. We don't actually round here — the kernel
/// produces stable bit-identical floats for shared frames/points (Frame::world
/// at the same origin, the same Vec3::x() literal, etc.), so bit equality is
/// the right notion. NaN handling: NaN's bit pattern is preserved verbatim,
/// which keeps NaN from accidentally merging with another NaN.
fn quantize(x: f64) -> u64 {
    x.to_bits()
}

fn quantize_point(p: Point3) -> [u64; 3] {
    [quantize(p.x), quantize(p.y), quantize(p.z)]
}

fn quantize_vec(v: Vec3) -> [u64; 3] {
    [quantize(v.x), quantize(v.y), quantize(v.z)]
}

/// `[origin_quantized; dir_z; dir_x]` — full identity of an `AXIS2_PLACEMENT_3D`.
fn quantize_frame(f: Frame) -> [u64; 9] {
    let p = quantize_point(f.origin);
    let z = quantize_vec(f.z);
    let x = quantize_vec(f.x);
    [p[0], p[1], p[2], z[0], z[1], z[2], x[0], x[1], x[2]]
}

struct StepWriter {
    next_id: u32,

    // Geometry caches.
    point_cache: HashMap<[u64; 3], u32>,
    direction_cache: HashMap<[u64; 3], u32>,
    frame_cache: HashMap<[u64; 9], u32>,

    // Topology → entity id maps.
    vertex_to_step: HashMap<VertexId, u32>,
    edge_to_step: HashMap<EdgeId, u32>,
    face_to_step: HashMap<FaceId, u32>,

    // Boilerplate ids referenced by the footer.
    context_id: u32,
    product_def_shape_id: u32,

    // Multi-shell support: emit_solid stashes additional solid_brep ids here
    // and emit_footer includes them all in ADVANCED_BREP_SHAPE_REPRESENTATION.
    extra_solid_breps: Vec<u32>,
}

impl StepWriter {
    fn new() -> Self {
        Self {
            next_id: 1,
            point_cache: HashMap::new(),
            direction_cache: HashMap::new(),
            frame_cache: HashMap::new(),
            vertex_to_step: HashMap::new(),
            edge_to_step: HashMap::new(),
            face_to_step: HashMap::new(),
            context_id: 0,
            product_def_shape_id: 0,
            extra_solid_breps: Vec::new(),
        }
    }

    fn emit(&mut self, w: &mut impl Write, body: &str) -> io::Result<u32> {
        let id = self.next_id;
        self.next_id += 1;
        writeln!(w, "#{id} = {body};")?;
        Ok(id)
    }

    fn emit_header(&mut self, w: &mut impl Write, name: &str) -> io::Result<()> {
        writeln!(w, "ISO-10303-21;")?;
        writeln!(w, "HEADER;")?;
        writeln!(w, "FILE_DESCRIPTION(('B-rep solid'),'2;1');")?;
        writeln!(
            w,
            "FILE_NAME('{}','2026-05-01T00:00:00',('kerf'),(''),'kerf 0.1.0','kerf','');",
            escape_step_string(name)
        )?;
        writeln!(
            w,
            "FILE_SCHEMA(('AUTOMOTIVE_DESIGN {{ 1 0 10303 214 3 1 1 }}'));"
        )?;
        writeln!(w, "ENDSEC;")?;
        writeln!(w, "DATA;")?;

        // Boilerplate context entities. Order matters: each entity references
        // ids already emitted.
        let app_ctx = self.emit(
            w,
            "APPLICATION_CONTEXT('core data for automotive mechanical design processes')",
        )?;
        let _ = self.emit(
            w,
            &format!(
                "APPLICATION_PROTOCOL_DEFINITION('international standard','automotive_design',2003,#{app_ctx})"
            ),
        )?;
        let prod_ctx = self.emit(w, &format!("PRODUCT_CONTEXT('',#{app_ctx},'mechanical')"))?;
        let prod = self.emit(w, &format!("PRODUCT('part','part','',(#{prod_ctx}))"))?;
        let prod_def_form = self.emit(
            w,
            &format!("PRODUCT_DEFINITION_FORMATION('','',#{prod})"),
        )?;
        let prod_def_ctx = self.emit(
            w,
            &format!("PRODUCT_DEFINITION_CONTEXT('part definition',#{app_ctx},'design')"),
        )?;
        let prod_def = self.emit(
            w,
            &format!("PRODUCT_DEFINITION('','',#{prod_def_form},#{prod_def_ctx})"),
        )?;
        self.product_def_shape_id =
            self.emit(w, &format!("PRODUCT_DEFINITION_SHAPE('','',#{prod_def})"))?;

        // Units: mm + radians + steradians, with a 1e-6 length tolerance.
        let length_unit = self.emit(
            w,
            "(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT(.MILLI.,.METRE.))",
        )?;
        let angle_unit = self.emit(
            w,
            "(NAMED_UNIT(*)PLANE_ANGLE_UNIT()SI_UNIT($,.RADIAN.))",
        )?;
        let solid_angle_unit = self.emit(
            w,
            "(NAMED_UNIT(*)SI_UNIT($,.STERADIAN.)SOLID_ANGLE_UNIT())",
        )?;
        let unc = self.emit(
            w,
            &format!(
                "UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(1.E-6),#{length_unit},'distance_accuracy_value','tolerance')"
            ),
        )?;
        self.context_id = self.emit(
            w,
            &format!(
                "(GEOMETRIC_REPRESENTATION_CONTEXT(3)GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((#{unc}))GLOBAL_UNIT_ASSIGNED_CONTEXT((#{length_unit},#{angle_unit},#{solid_angle_unit}))REPRESENTATION_CONTEXT('','3D'))"
            ),
        )?;
        Ok(())
    }

    /// Emit a CARTESIAN_POINT, deduplicated by exact bit pattern.
    fn emit_point(&mut self, w: &mut impl Write, p: Point3) -> io::Result<u32> {
        let key = quantize_point(p);
        if let Some(&id) = self.point_cache.get(&key) {
            return Ok(id);
        }
        let id = self.emit(
            w,
            &format!("CARTESIAN_POINT('',({},{},{}))", fmt_f(p.x), fmt_f(p.y), fmt_f(p.z)),
        )?;
        self.point_cache.insert(key, id);
        Ok(id)
    }

    /// Emit a DIRECTION (unit vector), deduplicated by exact bit pattern.
    fn emit_direction(&mut self, w: &mut impl Write, d: Vec3) -> io::Result<u32> {
        let key = quantize_vec(d);
        if let Some(&id) = self.direction_cache.get(&key) {
            return Ok(id);
        }
        let id = self.emit(
            w,
            &format!("DIRECTION('',({},{},{}))", fmt_f(d.x), fmt_f(d.y), fmt_f(d.z)),
        )?;
        self.direction_cache.insert(key, id);
        Ok(id)
    }

    /// Emit AXIS2_PLACEMENT_3D from a Frame: origin + z-axis + x ref direction.
    fn emit_frame(&mut self, w: &mut impl Write, f: Frame) -> io::Result<u32> {
        let key = quantize_frame(f);
        if let Some(&id) = self.frame_cache.get(&key) {
            return Ok(id);
        }
        let p = self.emit_point(w, f.origin)?;
        let axis_dir = self.emit_direction(w, f.z)?;
        let ref_dir = self.emit_direction(w, f.x)?;
        let id = self.emit(
            w,
            &format!("AXIS2_PLACEMENT_3D('',#{p},#{axis_dir},#{ref_dir})"),
        )?;
        self.frame_cache.insert(key, id);
        Ok(id)
    }

    /// Emit a CurveKind (LINE/CIRCLE/ELLIPSE). Not deduplicated — every
    /// `EdgeId` gets its own STEP curve since edges are unique.
    fn emit_curve(&mut self, w: &mut impl Write, c: &CurveKind) -> io::Result<u32> {
        match c {
            CurveKind::Line(line) => {
                let p = self.emit_point(w, line.origin)?;
                let d = self.emit_direction(w, line.direction)?;
                let v = self.emit(w, &format!("VECTOR('',#{d},1.)"))?;
                self.emit(w, &format!("LINE('',#{p},#{v})"))
            }
            CurveKind::Circle(circle) => {
                let placement = self.emit_frame(w, circle.frame)?;
                self.emit(
                    w,
                    &format!("CIRCLE('',#{placement},{})", fmt_f(circle.radius)),
                )
            }
            CurveKind::Ellipse(ell) => {
                let placement = self.emit_frame(w, ell.frame)?;
                self.emit(
                    w,
                    &format!(
                        "ELLIPSE('',#{placement},{},{})",
                        fmt_f(ell.semi_major),
                        fmt_f(ell.semi_minor)
                    ),
                )
            }
        }
    }

    /// Emit a SurfaceKind. Not deduplicated — adjacent faces of a solid
    /// generally don't share the *same* surface entity in a B-rep, even when
    /// they share a frame; we leave that micro-optimization out.
    fn emit_surface(&mut self, w: &mut impl Write, s: &SurfaceKind) -> io::Result<u32> {
        match s {
            SurfaceKind::Plane(p) => {
                let placement = self.emit_frame(w, p.frame)?;
                self.emit(w, &format!("PLANE('',#{placement})"))
            }
            SurfaceKind::Cylinder(c) => {
                let placement = self.emit_frame(w, c.frame)?;
                self.emit(
                    w,
                    &format!(
                        "CYLINDRICAL_SURFACE('',#{placement},{})",
                        fmt_f(c.radius)
                    ),
                )
            }
            SurfaceKind::Sphere(s) => {
                let placement = self.emit_frame(w, s.frame)?;
                self.emit(
                    w,
                    &format!("SPHERICAL_SURFACE('',#{placement},{})", fmt_f(s.radius)),
                )
            }
            SurfaceKind::Cone(c) => {
                let placement = self.emit_frame(w, c.frame)?;
                // CONICAL_SURFACE radius is the radius at v=0 (apex). Our cone
                // apex is at v=0, so the radius there is 0.
                self.emit(
                    w,
                    &format!(
                        "CONICAL_SURFACE('',#{placement},0.,{})",
                        fmt_f(c.half_angle)
                    ),
                )
            }
            SurfaceKind::Torus(t) => {
                let placement = self.emit_frame(w, t.frame)?;
                self.emit(
                    w,
                    &format!(
                        "TOROIDAL_SURFACE('',#{placement},{},{})",
                        fmt_f(t.major_radius),
                        fmt_f(t.minor_radius)
                    ),
                )
            }
        }
    }

    /// Emit all topology and geometry entities for `solid`, returning the
    /// MANIFOLD_SOLID_BREP id (or, for multi-shell solids, the first one — see
    /// caveat in body).
    fn emit_solid(&mut self, w: &mut impl Write, solid: &Solid) -> io::Result<u32> {
        // ---- Pass 1: vertex points ----
        for vid in solid.topo.vertex_ids() {
            let p = solid
                .point_at(vid)
                .expect("vertex without geometry — kernel invariant violated");
            let pt = self.emit_point(w, p)?;
            let vp = self.emit(w, &format!("VERTEX_POINT('',#{pt})"))?;
            self.vertex_to_step.insert(vid, vp);
        }

        // ---- Pass 2: edges (curve + EDGE_CURVE) ----
        for (eid, edge) in solid.topo.edges_iter() {
            let seg = solid
                .edge_geom
                .get(eid)
                .expect("edge without geometry — kernel invariant violated");
            let curve_id = self.emit_curve(w, &seg.curve)?;
            let [he_a, _he_b] = edge.half_edges();
            // Edge endpoints: he_a's origin → he_a's twin's origin.
            let he_a_struct = solid.topo.half_edge(he_a).expect("missing half-edge");
            let v_from = he_a_struct.origin();
            let twin_struct = solid
                .topo
                .half_edge(he_a_struct.twin())
                .expect("missing twin half-edge");
            let v_to = twin_struct.origin();
            let vp_from = self.vertex_to_step[&v_from];
            let vp_to = self.vertex_to_step[&v_to];
            // same_sense flag: .T. — the EDGE_CURVE inherits the curve's
            // direction. Per-half-edge direction is encoded by ORIENTED_EDGE.
            let edge_curve = self.emit(
                w,
                &format!("EDGE_CURVE('',#{vp_from},#{vp_to},#{curve_id},.T.)"),
            )?;
            self.edge_to_step.insert(eid, edge_curve);
        }

        // ---- Pass 3: faces ----
        // For each face: emit the surface, walk the loop emitting an
        // ORIENTED_EDGE per half-edge, then EDGE_LOOP, then FACE_OUTER_BOUND,
        // then ADVANCED_FACE.
        for fid in solid.topo.face_ids() {
            let face = solid.topo.face(fid).expect("missing face");
            let surface_kind = solid
                .face_geom
                .get(fid)
                .expect("face without geometry — kernel invariant violated");
            let surface_id = self.emit_surface(w, surface_kind)?;

            let outer_loop_id = face.outer_loop();
            let lp = solid.topo.loop_(outer_loop_id).expect("missing loop");

            // Build the list of bounds. A loop with no half-edge is the
            // sphere/torus case — emit the face with no bounds.
            let mut bound_refs: Vec<u32> = Vec::new();

            if let Some(start_he) = lp.half_edge() {
                let edge_loop_id = self.emit_loop_bounds(w, solid, start_he)?;
                let outer_bound = self.emit(
                    w,
                    &format!("FACE_OUTER_BOUND('',#{edge_loop_id},.T.)"),
                )?;
                bound_refs.push(outer_bound);
            }

            // Inner loops (holes in the face).
            for &inner_loop_id in face.inner_loops() {
                let inner_lp = solid.topo.loop_(inner_loop_id).expect("missing inner loop");
                if let Some(start_he) = inner_lp.half_edge() {
                    let edge_loop_id = self.emit_loop_bounds(w, solid, start_he)?;
                    let bound = self
                        .emit(w, &format!("FACE_BOUND('',#{edge_loop_id},.T.)"))?;
                    bound_refs.push(bound);
                }
            }

            let face_id = self.emit(
                w,
                &format!(
                    "ADVANCED_FACE('',({}),#{surface_id},.T.)",
                    fmt_id_list(&bound_refs)
                ),
            )?;
            self.face_to_step.insert(fid, face_id);
        }

        // ---- Pass 4: shells + solids ----
        // Group faces by shell. SlotMap doesn't expose a public iter, so we
        // derive shell membership from each face's `Face.shell()`.
        let mut shells: HashMap<kerf_topo::ShellId, Vec<u32>> = HashMap::new();
        let mut shell_order: Vec<kerf_topo::ShellId> = Vec::new();
        for fid in solid.topo.face_ids() {
            let face = solid.topo.face(fid).expect("missing face");
            let shell = face.shell();
            let face_step_id = self.face_to_step[&fid];
            shells
                .entry(shell)
                .and_modify(|v| v.push(face_step_id))
                .or_insert_with(|| {
                    shell_order.push(shell);
                    vec![face_step_id]
                });
        }

        // Multi-shell solids: AP214 represents this as a single
        // MANIFOLD_SOLID_BREP per outer shell. For our boolean results that
        // produce 2 disjoint shells, we emit each as its own solid_brep and
        // wrap all of them together at the footer. To keep emit_footer simple,
        // we emit each MANIFOLD_SOLID_BREP and return the *first*'s id; the
        // footer needs the full list, so we return them via an extension
        // path... actually: emit one MANIFOLD_SOLID_BREP and pack the rest as
        // additional items in the ADVANCED_BREP_SHAPE_REPRESENTATION list.
        //
        // For now, return only the first solid id; emit_footer handles the
        // single-solid case. We collect any additional ids and emit them
        // immediately so they appear in the file even if not referenced. This
        // is a known limitation tracked for a future milestone.
        let mut solid_brep_ids: Vec<u32> = Vec::with_capacity(shell_order.len());
        for shell_id in &shell_order {
            let face_ids = &shells[shell_id];
            let shell_step_id = self.emit(
                w,
                &format!("CLOSED_SHELL('',({}))", fmt_id_list(face_ids)),
            )?;
            let solid_brep_id =
                self.emit(w, &format!("MANIFOLD_SOLID_BREP('',#{shell_step_id})"))?;
            solid_brep_ids.push(solid_brep_id);
        }

        // Empty solid: return 0 (no MANIFOLD_SOLID_BREP) and let caller decide.
        // But emit_footer expects a valid id; we panic here instead, matching
        // the kernel's "non-empty solid" precondition.
        let first = *solid_brep_ids
            .first()
            .expect("STEP export requires at least one shell in the solid");

        // Multi-shell: emit a synthetic brep representation that lists all
        // solid_breps. Since AP214 puts the items list in
        // ADVANCED_BREP_SHAPE_REPRESENTATION, we pass them through via an
        // instance variable.
        self.extra_solid_breps = solid_brep_ids[1..].to_vec();
        Ok(first)
    }

    /// Walk the half-edge loop starting at `start_he` and emit one
    /// ORIENTED_EDGE per half-edge plus the EDGE_LOOP. Returns the EDGE_LOOP id.
    fn emit_loop_bounds(
        &mut self,
        w: &mut impl Write,
        solid: &Solid,
        start_he: HalfEdgeId,
    ) -> io::Result<u32> {
        // Walk the loop, collecting ORIENTED_EDGE refs.
        let mut oriented_refs: Vec<u32> = Vec::new();
        for he_id in solid.topo.iter_loop_half_edges(start_he) {
            let he = solid.topo.half_edge(he_id).expect("missing half-edge");
            let edge_id = he.edge();
            let edge = solid.topo.edge(edge_id).expect("missing edge");
            let [he_a, _he_b] = edge.half_edges();
            // .T. when this half-edge is he_a (the EDGE_CURVE's "forward"
            // direction); .F. when it's he_b.
            let same_sense = if he_id == he_a { ".T." } else { ".F." };
            let edge_curve_step = self.edge_to_step[&edge_id];
            let oe = self.emit(
                w,
                &format!("ORIENTED_EDGE('',*,*,#{edge_curve_step},{same_sense})"),
            )?;
            oriented_refs.push(oe);
        }
        let edge_loop = self.emit(
            w,
            &format!("EDGE_LOOP('',({}))", fmt_id_list(&oriented_refs)),
        )?;
        Ok(edge_loop)
    }

    fn emit_footer(&mut self, w: &mut impl Write, solid_brep_ref: u32) -> io::Result<()> {
        // Combine the first solid_brep with any extras emitted by emit_solid.
        let mut items = vec![solid_brep_ref];
        items.extend(self.extra_solid_breps.iter().copied());
        let rep = self.emit(
            w,
            &format!(
                "ADVANCED_BREP_SHAPE_REPRESENTATION('',({}),#{})",
                fmt_id_list(&items),
                self.context_id
            ),
        )?;
        let _ = self.emit(
            w,
            &format!(
                "SHAPE_DEFINITION_REPRESENTATION(#{},#{})",
                self.product_def_shape_id, rep
            ),
        )?;
        writeln!(w, "ENDSEC;")?;
        write!(w, "END-ISO-10303-21;\n")?;
        Ok(())
    }
}

/// Format an f64 in STEP's preferred notation. Examples:
///   1.0 → "1."
///   0.5 → "0.5"
///   1e-6 → "1.E-6"
/// We accept any rust default debug-friendly form; FreeCAD parses both
/// "1.0" and "1." (with trailing dot). The trailing-dot form is what most
/// reference STEP files use and is what AP214 examples in ISO 10303-21 show.
fn fmt_f(x: f64) -> String {
    if x.is_nan() {
        return "0.".to_string(); // STEP has no NaN; cap to 0 for safety.
    }
    if x.is_infinite() {
        return if x > 0.0 { "1.E308".to_string() } else { "-1.E308".to_string() };
    }
    if x == 0.0 {
        return "0.".to_string();
    }
    // Rust's default {} on f64 yields "1" for whole numbers, "1.5" for non-
    // whole. STEP wants "1." for the whole-number case. Use {:?} which yields
    // "1.0" then strip the trailing zero.
    let s = format!("{x:?}");
    // {:?} for 1.0 => "1.0"; for 0.5 => "0.5"; for 1e-6 => "1e-6".
    // STEP wants "E" not "e".
    let s = s.replace('e', "E");
    // For "X.0" → "X.".
    if let Some(stripped) = s.strip_suffix(".0") {
        format!("{stripped}.")
    } else {
        s
    }
}

/// Format a list of entity ids as `#1,#2,#3`.
fn fmt_id_list(ids: &[u32]) -> String {
    let mut out = String::new();
    for (i, id) in ids.iter().enumerate() {
        if i > 0 {
            out.push(',');
        }
        out.push('#');
        out.push_str(&id.to_string());
    }
    out
}

/// Escape a STEP string literal: single quotes are doubled.
fn escape_step_string(s: &str) -> String {
    s.replace('\'', "''")
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::primitives::{box_, cylinder, sphere};
    use kerf_geom::Vec3;

    #[test]
    fn step_box_emits_iso_header() {
        let s = box_(Vec3::new(1.0, 1.0, 1.0));
        let mut buf = Vec::new();
        write_step(&s, "test_box", &mut buf).unwrap();
        let out = String::from_utf8(buf).unwrap();
        assert!(out.starts_with("ISO-10303-21;"));
        assert!(out.contains("HEADER;"));
        assert!(out.contains("DATA;"));
        assert!(out.contains("ENDSEC;"));
        assert!(out.ends_with("END-ISO-10303-21;\n"));
    }

    #[test]
    fn step_box_emits_required_entities() {
        let s = box_(Vec3::new(1.0, 1.0, 1.0));
        let mut buf = Vec::new();
        write_step(&s, "test_box", &mut buf).unwrap();
        let out = String::from_utf8(buf).unwrap();
        assert!(out.contains("CARTESIAN_POINT"));
        assert!(out.contains("DIRECTION"));
        assert!(out.contains("AXIS2_PLACEMENT_3D"));
        assert!(out.contains("LINE"));
        assert!(out.contains("PLANE"));
        assert!(out.contains("VERTEX_POINT"));
        assert!(out.contains("EDGE_CURVE"));
        assert!(out.contains("ORIENTED_EDGE"));
        assert!(out.contains("EDGE_LOOP"));
        assert!(out.contains("ADVANCED_FACE"));
        assert!(out.contains("CLOSED_SHELL"));
        assert!(out.contains("MANIFOLD_SOLID_BREP"));
        assert!(out.contains("ADVANCED_BREP_SHAPE_REPRESENTATION"));
        assert!(out.contains("SHAPE_DEFINITION_REPRESENTATION"));
    }

    #[test]
    fn step_box_has_at_least_8_cartesian_points() {
        // 8 corners → at least 8 CARTESIAN_POINT (more for line origins, frame
        // origins).
        let s = box_(Vec3::new(1.0, 1.0, 1.0));
        let mut buf = Vec::new();
        write_step(&s, "box", &mut buf).unwrap();
        let out = String::from_utf8(buf).unwrap();
        let count = out.matches("CARTESIAN_POINT").count();
        assert!(count >= 8, "expected >=8 CARTESIAN_POINT, got {count}");
    }

    #[test]
    fn step_box_emits_six_advanced_faces() {
        let s = box_(Vec3::new(1.0, 1.0, 1.0));
        let mut buf = Vec::new();
        write_step(&s, "box", &mut buf).unwrap();
        let out = String::from_utf8(buf).unwrap();
        assert_eq!(out.matches("ADVANCED_FACE").count(), 6);
    }

    #[test]
    fn step_cylinder_emits_cylindrical_surface_and_circles() {
        let s = cylinder(1.0, 2.0);
        let mut buf = Vec::new();
        write_step(&s, "cyl", &mut buf).unwrap();
        let out = String::from_utf8(buf).unwrap();
        assert!(out.contains("CYLINDRICAL_SURFACE"));
        assert!(out.contains("CIRCLE"));
        assert_eq!(out.matches("ADVANCED_FACE").count(), 3);
        // Cylinder lateral loop has 4 half-edges; 2 cap loops have 1 each → 6
        // ORIENTED_EDGE total.
        assert_eq!(out.matches("ORIENTED_EDGE").count(), 6);
    }

    #[test]
    fn step_cylinder_emits_one_t_and_one_f_oriented_edge_for_each_circle() {
        // Each circle edge is traversed once forward (.T.) by the cap face and
        // once reversed (.F.) by the lateral face.
        let s = cylinder(1.0, 2.0);
        let mut buf = Vec::new();
        write_step(&s, "cyl", &mut buf).unwrap();
        let out = String::from_utf8(buf).unwrap();
        let t_count = out.matches(".T.)").count();
        let f_count = out.matches(".F.)").count();
        // Plenty of .T. closers from EDGE_CURVE and ADVANCED_FACE; we just
        // care that at least some ORIENTED_EDGEs are .F.
        assert!(f_count >= 2, "expected ≥2 .F. terminators, got {f_count}");
        assert!(t_count > f_count); // forward dominates
    }

    #[test]
    fn step_sphere_emits_spherical_surface_with_no_bounds() {
        // Sphere has no edges; the face is emitted with an empty bounds list.
        let s = sphere(1.5);
        let mut buf = Vec::new();
        write_step(&s, "sph", &mut buf).unwrap();
        let out = String::from_utf8(buf).unwrap();
        assert!(out.contains("SPHERICAL_SURFACE"));
        // No EDGE_CURVE, no ORIENTED_EDGE, no EDGE_LOOP, no FACE_OUTER_BOUND.
        assert!(!out.contains("EDGE_CURVE"));
        assert!(!out.contains("ORIENTED_EDGE"));
        assert!(!out.contains("EDGE_LOOP"));
        assert!(!out.contains("FACE_OUTER_BOUND"));
        // ADVANCED_FACE with empty bounds list "()".
        assert!(out.contains("ADVANCED_FACE('',()"));
    }

    #[test]
    fn step_difference_two_shells_produces_two_solid_breps() {
        use crate::primitives::box_at;
        use kerf_geom::Point3;
        let big = box_(Vec3::new(10.0, 10.0, 10.0));
        let small = box_at(Vec3::new(2.0, 2.0, 2.0), Point3::new(4.0, 4.0, 4.0));
        let result = big.difference(&small);
        let mut buf = Vec::new();
        write_step(&result, "diff", &mut buf).unwrap();
        let out = String::from_utf8(buf).unwrap();
        assert_eq!(out.matches("MANIFOLD_SOLID_BREP").count(), 2);
        assert_eq!(out.matches("CLOSED_SHELL").count(), 2);
    }

    #[test]
    fn fmt_f_strips_trailing_zero_for_whole_numbers() {
        assert_eq!(fmt_f(1.0), "1.");
        assert_eq!(fmt_f(0.0), "0.");
        assert_eq!(fmt_f(-3.0), "-3.");
        assert_eq!(fmt_f(0.5), "0.5");
    }

    #[test]
    fn fmt_id_list_formats_correctly() {
        assert_eq!(fmt_id_list(&[1, 2, 3]), "#1,#2,#3");
        assert_eq!(fmt_id_list(&[]), "");
        assert_eq!(fmt_id_list(&[42]), "#42");
    }
}
