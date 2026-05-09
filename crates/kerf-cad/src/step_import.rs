//! STEP (ISO 10303-21) AP203/AP214 importer for the planar-faceted polyhedral
//! subset.
//!
//! Reads the ASCII STEP entity types kerf's own [`kerf_brep::write_step`] emits
//! for box-and-extrude-style geometry — `CARTESIAN_POINT`, `VERTEX_POINT`,
//! `LINE`, `EDGE_CURVE`, `ORIENTED_EDGE`, `EDGE_LOOP`, `FACE_OUTER_BOUND`,
//! `PLANE`, `AXIS2_PLACEMENT_3D`, `ADVANCED_FACE`, `CLOSED_SHELL`,
//! `MANIFOLD_SOLID_BREP` — and reconstructs a triangulated [`Solid`] suitable
//! for the viewer.
//!
//! ## Scope
//!
//! Curved surfaces (`CYLINDRICAL_SURFACE`, `SPHERICAL_SURFACE`,
//! `B_SPLINE_SURFACE`, NURBS, …), trimmed-surface curves, and topology with
//! voids (`BREP_WITH_VOIDS`) are **out of scope** for this iteration. The
//! parser returns [`StepImportError::Unsupported`] when it sees one of those.
//!
//! ## Pipeline
//!
//! 1. **Lex** the STEP text into raw entity records keyed by `#N`.
//! 2. **Parse** each record's parameter list into a small AST
//!    ([`Param::Ref`], [`Param::Num`], [`Param::Str`], [`Param::List`],
//!    [`Param::Enum`], [`Param::Null`], [`Param::Star`]).
//! 3. **Resolve** the topology: walk every `ADVANCED_FACE`, follow its
//!    `FACE_OUTER_BOUND` → `EDGE_LOOP` → `ORIENTED_EDGE` → `EDGE_CURVE`
//!    chain, and produce an ordered ring of [`Point3`] vertices for each
//!    face.
//! 4. **Triangulate** each face by fan-from-vertex-0 and feed the resulting
//!    triangle soup to [`kerf_brep::from_triangles`], which handles vertex
//!    dedup and half-edge pairing.

use std::collections::HashMap;

use kerf_brep::{from_triangles, Solid};
use kerf_geom::Point3;
use thiserror::Error;

use crate::feature::Feature;
use crate::model::Model;

#[derive(Debug, Error)]
pub enum StepImportError {
    #[error("parse error at line {line}: {message}")]
    Parse { line: usize, message: String },
    #[error("unsupported entity '{kind}' at #{id} — only planar polyhedral STEP is supported")]
    Unsupported { id: u64, kind: String },
    #[error("missing reference: #{id} not found in file")]
    MissingRef { id: u64 },
    #[error("topology error: {0}")]
    Topology(String),
}

/// Import the first solid in `input` (an ISO 10303-21 STEP text) as a kerf
/// [`Solid`].
///
/// Errors fall into one of [`StepImportError`]'s variants. The caller is
/// expected to surface the message — it carries the entity id and a hint
/// about which sub-feature is unsupported.
pub fn import_step(input: &str) -> Result<Solid, StepImportError> {
    let entities = lex_entities(input)?;
    // Build raw-body map, keyed by id. We lazily parse parameters only for
    // entity types the resolver actually walks (MANIFOLD_SOLID_BREP and
    // friends). This avoids choking on the boilerplate AP214 unit /
    // representation-context entities that use complex-entity syntax with
    // typed parameters (e.g. `LENGTH_MEASURE(1.E-6)`) that our minimal
    // parameter grammar doesn't model.
    let mut raw: HashMap<u64, RawEntity> = HashMap::with_capacity(entities.len());
    for r in entities {
        raw.insert(r.id, r);
    }

    let resolver = Resolver { raw: &raw };
    let triangles = resolver.collect_triangles()?;
    if triangles.is_empty() {
        return Err(StepImportError::Topology(
            "no MANIFOLD_SOLID_BREP found in DATA section".into(),
        ));
    }
    let solid = from_triangles(&triangles).map_err(|e| {
        StepImportError::Topology(format!("triangle-soup → solid failed: {e}"))
    })?;
    Ok(solid)
}

/// Import a STEP file's first solid as a single-feature [`Model`], ready for
/// the viewer's evaluator. The imported geometry becomes a
/// [`Feature::ImportedMesh`] with the given `feature_id` (commonly
/// `"imported"`).
///
/// We tessellate the parsed triangles (already stored as a triangle list in
/// `triangles` under the hood) into a (vertices, indices) pair so the model
/// JSON-round-trips. Vertex dedup uses a 1 µm grid, matching
/// [`kerf_brep::from_triangles`]'s convention.
pub fn import_step_to_model(
    input: &str,
    feature_id: &str,
) -> Result<Model, StepImportError> {
    let entities = lex_entities(input)?;
    let mut raw: HashMap<u64, RawEntity> = HashMap::with_capacity(entities.len());
    for r in entities {
        raw.insert(r.id, r);
    }
    let resolver = Resolver { raw: &raw };
    let triangles = resolver.collect_triangles()?;
    if triangles.is_empty() {
        return Err(StepImportError::Topology(
            "no MANIFOLD_SOLID_BREP found in DATA section".into(),
        ));
    }
    // Dedup vertices on the same 1µm grid the kernel uses internally.
    let grid = 1.0e6;
    let mut keys: HashMap<(i64, i64, i64), usize> = HashMap::new();
    let mut vertices: Vec<[f64; 3]> = Vec::new();
    let mut indices: Vec<[usize; 3]> = Vec::with_capacity(triangles.len());
    for tri in &triangles {
        let mut idx = [0usize; 3];
        for (k, p) in tri.iter().enumerate() {
            let key = (
                (p.x * grid).round() as i64,
                (p.y * grid).round() as i64,
                (p.z * grid).round() as i64,
            );
            let i = *keys.entry(key).or_insert_with(|| {
                let n = vertices.len();
                vertices.push([p.x, p.y, p.z]);
                n
            });
            idx[k] = i;
        }
        // Skip degenerate triangles whose dedup collapsed two corners into
        // one — `from_triangles` would otherwise reject the whole soup.
        if idx[0] == idx[1] || idx[1] == idx[2] || idx[0] == idx[2] {
            continue;
        }
        indices.push(idx);
    }
    let model = Model::new()
        .try_add(Feature::ImportedMesh {
            id: feature_id.into(),
            vertices,
            indices,
        })
        .map_err(|e| StepImportError::Topology(format!("model insert: {e}")))?;
    Ok(model)
}

// ---------------------------------------------------------------------------
// Lexing — split DATA section into raw entity records.
// ---------------------------------------------------------------------------

#[derive(Debug)]
struct RawEntity {
    id: u64,
    kind: String,
    body: String, // text inside the outermost (...)
    line: usize,
}

/// Slice the input into per-entity records of the form `#N = TYPE(...);`.
///
/// Tolerant of arbitrary whitespace and line wrapping inside an entity. Stops
/// at `ENDSEC;` after `DATA;` and skips everything before `DATA;`.
fn lex_entities(input: &str) -> Result<Vec<RawEntity>, StepImportError> {
    // Find DATA; section.
    let bytes = input.as_bytes();
    let data_start = find_keyword(input, "DATA;").ok_or_else(|| StepImportError::Parse {
        line: 0,
        message: "missing DATA; section".into(),
    })?;
    let data_end = find_keyword_from(input, "ENDSEC;", data_start + 5)
        .unwrap_or(bytes.len());

    // Compute a 0-based byte → line lookup for diagnostics.
    let line_of = |pos: usize| -> usize {
        // 1-based line number.
        bytes[..pos.min(bytes.len())]
            .iter()
            .filter(|&&b| b == b'\n')
            .count()
            + 1
    };

    let mut entities = Vec::new();
    let mut i = data_start + 5; // past "DATA;"
    while i < data_end {
        // Skip whitespace + comments. STEP comments are /* ... */.
        i = skip_ws_and_comments(input, i, data_end);
        if i >= data_end {
            break;
        }
        if !input[i..].starts_with('#') {
            // Allow stray top-level tokens? Grammar forbids it, but be
            // forgiving on whitespace mismatch.
            return Err(StepImportError::Parse {
                line: line_of(i),
                message: format!(
                    "expected '#<id>' at top of DATA section, got {:?}",
                    input[i..].chars().next().unwrap_or('?')
                ),
            });
        }
        let id_start_line = line_of(i);
        i += 1;
        // Parse integer id.
        let id_text_start = i;
        while i < data_end && input.as_bytes()[i].is_ascii_digit() {
            i += 1;
        }
        if i == id_text_start {
            return Err(StepImportError::Parse {
                line: id_start_line,
                message: "expected integer after '#'".into(),
            });
        }
        let id: u64 = input[id_text_start..i].parse().map_err(|_| StepImportError::Parse {
            line: id_start_line,
            message: format!("invalid id '#{}'", &input[id_text_start..i]),
        })?;
        // Skip ws to '='.
        i = skip_ws_and_comments(input, i, data_end);
        if i >= data_end || input.as_bytes()[i] != b'=' {
            return Err(StepImportError::Parse {
                line: line_of(i),
                message: format!("expected '=' after '#{id}'"),
            });
        }
        i += 1;
        i = skip_ws_and_comments(input, i, data_end);
        // Parse TYPE NAME (uppercase identifier, may contain '_'). Some
        // entities are "complex" — written as `(SUBTYPE_A() SUBTYPE_B())`.
        // Those are out of scope; we recognise the leading `(` and report
        // Unsupported using a synthetic "COMPLEX_ENTITY" name.
        let kind: String;
        if i < data_end && input.as_bytes()[i] == b'(' {
            kind = "COMPLEX_ENTITY".into();
        } else {
            let kind_start = i;
            while i < data_end {
                let c = input.as_bytes()[i];
                if c.is_ascii_uppercase() || c.is_ascii_digit() || c == b'_' {
                    i += 1;
                } else {
                    break;
                }
            }
            if i == kind_start {
                return Err(StepImportError::Parse {
                    line: line_of(i),
                    message: format!("expected entity type name after '#{id} ='"),
                });
            }
            kind = input[kind_start..i].to_string();
        }
        i = skip_ws_and_comments(input, i, data_end);
        if i >= data_end || input.as_bytes()[i] != b'(' {
            return Err(StepImportError::Parse {
                line: line_of(i),
                message: format!("expected '(' after '{kind}' for #{id}"),
            });
        }
        // Capture the parenthesised body.
        let body_start = i;
        let body_end = find_matching_paren(input, i, data_end)?;
        let body = input[body_start + 1..body_end].to_string();
        i = body_end + 1;
        i = skip_ws_and_comments(input, i, data_end);
        if i >= data_end || input.as_bytes()[i] != b';' {
            return Err(StepImportError::Parse {
                line: line_of(i),
                message: format!("expected ';' to end entity #{id}"),
            });
        }
        i += 1;
        entities.push(RawEntity {
            id,
            kind,
            body,
            line: id_start_line,
        });
    }
    Ok(entities)
}

fn skip_ws_and_comments(s: &str, start: usize, end: usize) -> usize {
    let bytes = s.as_bytes();
    let mut i = start;
    loop {
        while i < end && (bytes[i] as char).is_whitespace() {
            i += 1;
        }
        // STEP block comments /* ... */
        if i + 1 < end && bytes[i] == b'/' && bytes[i + 1] == b'*' {
            i += 2;
            while i + 1 < end && !(bytes[i] == b'*' && bytes[i + 1] == b'/') {
                i += 1;
            }
            if i + 1 < end {
                i += 2;
            }
            continue;
        }
        break;
    }
    i
}

fn find_keyword(s: &str, kw: &str) -> Option<usize> {
    s.find(kw)
}

fn find_keyword_from(s: &str, kw: &str, from: usize) -> Option<usize> {
    s[from..].find(kw).map(|p| p + from)
}

/// Given `s[at] == '('`, return the index of the matching ')'. STEP strings
/// `'...'` and block comments `/*...*/` are paren-transparent; the matcher
/// must not count parens inside them.
fn find_matching_paren(s: &str, at: usize, end: usize) -> Result<usize, StepImportError> {
    let bytes = s.as_bytes();
    debug_assert_eq!(bytes[at], b'(');
    let mut depth = 0;
    let mut i = at;
    while i < end {
        let c = bytes[i];
        match c {
            b'(' => {
                depth += 1;
                i += 1;
            }
            b')' => {
                depth -= 1;
                if depth == 0 {
                    return Ok(i);
                }
                i += 1;
            }
            b'\'' => {
                // String literal: find closing quote, accounting for the
                // STEP escape `''`.
                i += 1;
                while i < end {
                    if bytes[i] == b'\'' {
                        if i + 1 < end && bytes[i + 1] == b'\'' {
                            i += 2;
                            continue;
                        }
                        i += 1;
                        break;
                    }
                    i += 1;
                }
            }
            b'/' if i + 1 < end && bytes[i + 1] == b'*' => {
                i += 2;
                while i + 1 < end && !(bytes[i] == b'*' && bytes[i + 1] == b'/') {
                    i += 1;
                }
                if i + 1 < end {
                    i += 2;
                }
            }
            _ => i += 1,
        }
    }
    Err(StepImportError::Parse {
        line: 0,
        message: "unmatched '(' in entity body".into(),
    })
}

// ---------------------------------------------------------------------------
// Param AST + parsing.
// ---------------------------------------------------------------------------

#[derive(Debug, Clone)]
#[allow(dead_code)]
enum Param {
    Ref(u64),
    Num(f64),
    Str(String),
    List(Vec<Param>),
    /// `.T.` / `.F.` / `.UNSPECIFIED.` etc.
    Enum(String),
    /// `$` — STEP "null / not given".
    Null,
    /// `*` — derived placeholder (used by ORIENTED_EDGE for the redundant
    /// vertex slots).
    Star,
}


/// Parse the text inside an entity's outer parens into a parameter list.
fn parse_params(body: &str, line: usize) -> Result<Vec<Param>, StepImportError> {
    let mut p = ParamParser {
        s: body.as_bytes(),
        i: 0,
        line,
    };
    let result = p.parse_list_inner()?;
    p.skip_ws();
    if p.i < p.s.len() {
        return Err(StepImportError::Parse {
            line,
            message: format!(
                "trailing content after parameters: {:?}",
                std::str::from_utf8(&p.s[p.i..]).unwrap_or("?")
            ),
        });
    }
    Ok(result)
}

struct ParamParser<'a> {
    s: &'a [u8],
    i: usize,
    line: usize,
}

impl<'a> ParamParser<'a> {
    fn skip_ws(&mut self) {
        loop {
            while self.i < self.s.len() && (self.s[self.i] as char).is_whitespace() {
                self.i += 1;
            }
            if self.i + 1 < self.s.len() && self.s[self.i] == b'/' && self.s[self.i + 1] == b'*' {
                self.i += 2;
                while self.i + 1 < self.s.len()
                    && !(self.s[self.i] == b'*' && self.s[self.i + 1] == b'/')
                {
                    self.i += 1;
                }
                if self.i + 1 < self.s.len() {
                    self.i += 2;
                }
                continue;
            }
            break;
        }
    }

    fn parse_list_inner(&mut self) -> Result<Vec<Param>, StepImportError> {
        let mut out = Vec::new();
        self.skip_ws();
        if self.i >= self.s.len() {
            return Ok(out);
        }
        loop {
            self.skip_ws();
            if self.i >= self.s.len() {
                break;
            }
            if self.s[self.i] == b')' {
                break;
            }
            let p = self.parse_param()?;
            out.push(p);
            self.skip_ws();
            if self.i < self.s.len() && self.s[self.i] == b',' {
                self.i += 1;
            } else {
                break;
            }
        }
        Ok(out)
    }

    fn parse_param(&mut self) -> Result<Param, StepImportError> {
        self.skip_ws();
        if self.i >= self.s.len() {
            return Err(StepImportError::Parse {
                line: self.line,
                message: "unexpected end of parameter list".into(),
            });
        }
        let c = self.s[self.i];
        match c {
            b'#' => {
                self.i += 1;
                let start = self.i;
                while self.i < self.s.len() && self.s[self.i].is_ascii_digit() {
                    self.i += 1;
                }
                if self.i == start {
                    return Err(StepImportError::Parse {
                        line: self.line,
                        message: "expected digits after '#'".into(),
                    });
                }
                let id: u64 = std::str::from_utf8(&self.s[start..self.i])
                    .unwrap()
                    .parse()
                    .map_err(|_| StepImportError::Parse {
                        line: self.line,
                        message: "invalid reference id".into(),
                    })?;
                Ok(Param::Ref(id))
            }
            b'\'' => {
                self.i += 1;
                let start = self.i;
                while self.i < self.s.len() {
                    if self.s[self.i] == b'\'' {
                        if self.i + 1 < self.s.len() && self.s[self.i + 1] == b'\'' {
                            self.i += 2;
                            continue;
                        }
                        break;
                    }
                    self.i += 1;
                }
                let raw = std::str::from_utf8(&self.s[start..self.i])
                    .map_err(|_| StepImportError::Parse {
                        line: self.line,
                        message: "string is not valid utf-8".into(),
                    })?
                    .replace("''", "'");
                if self.i >= self.s.len() {
                    return Err(StepImportError::Parse {
                        line: self.line,
                        message: "unterminated string".into(),
                    });
                }
                self.i += 1; // skip closing quote
                Ok(Param::Str(raw))
            }
            b'(' => {
                self.i += 1;
                let inner = self.parse_list_inner()?;
                self.skip_ws();
                if self.i >= self.s.len() || self.s[self.i] != b')' {
                    return Err(StepImportError::Parse {
                        line: self.line,
                        message: "missing ')' in nested list".into(),
                    });
                }
                self.i += 1;
                Ok(Param::List(inner))
            }
            b'.' => {
                self.i += 1;
                let start = self.i;
                while self.i < self.s.len() && self.s[self.i] != b'.' {
                    self.i += 1;
                }
                if self.i >= self.s.len() {
                    return Err(StepImportError::Parse {
                        line: self.line,
                        message: "unterminated enum literal".into(),
                    });
                }
                let v = std::str::from_utf8(&self.s[start..self.i])
                    .unwrap()
                    .to_string();
                self.i += 1;
                Ok(Param::Enum(v))
            }
            b'$' => {
                self.i += 1;
                Ok(Param::Null)
            }
            b'*' => {
                self.i += 1;
                Ok(Param::Star)
            }
            b'+' | b'-' | b'0'..=b'9' => {
                let start = self.i;
                if self.s[self.i] == b'+' || self.s[self.i] == b'-' {
                    self.i += 1;
                }
                while self.i < self.s.len()
                    && (self.s[self.i].is_ascii_digit()
                        || self.s[self.i] == b'.'
                        || self.s[self.i] == b'e'
                        || self.s[self.i] == b'E'
                        || self.s[self.i] == b'+'
                        || self.s[self.i] == b'-')
                {
                    // Stop the +/- consumption from eating the next param's sign
                    // by only allowing +/- right after e/E.
                    let prev = self.s[self.i - 1];
                    let cur = self.s[self.i];
                    if (cur == b'+' || cur == b'-') && prev != b'e' && prev != b'E' {
                        break;
                    }
                    self.i += 1;
                }
                let text = std::str::from_utf8(&self.s[start..self.i]).unwrap();
                let n: f64 = text.parse().map_err(|_| StepImportError::Parse {
                    line: self.line,
                    message: format!("invalid number '{text}'"),
                })?;
                Ok(Param::Num(n))
            }
            _ => Err(StepImportError::Parse {
                line: self.line,
                message: format!("unexpected char '{}' in params", c as char),
            }),
        }
    }
}

// ---------------------------------------------------------------------------
// Resolution: walk the entity graph and emit triangles.
// ---------------------------------------------------------------------------

const SUPPORTED_SURFACES: &[&str] = &["PLANE"];
const SUPPORTED_CURVES: &[&str] = &["LINE"];
const UNSUPPORTED_SURFACES: &[&str] = &[
    "CYLINDRICAL_SURFACE",
    "SPHERICAL_SURFACE",
    "CONICAL_SURFACE",
    "TOROIDAL_SURFACE",
    "B_SPLINE_SURFACE",
    "B_SPLINE_SURFACE_WITH_KNOTS",
    "BEZIER_SURFACE",
    "RATIONAL_B_SPLINE_SURFACE",
    "SURFACE_OF_REVOLUTION",
    "SURFACE_OF_LINEAR_EXTRUSION",
    "OFFSET_SURFACE",
];
const UNSUPPORTED_CURVES: &[&str] = &[
    "CIRCLE",
    "ELLIPSE",
    "B_SPLINE_CURVE",
    "B_SPLINE_CURVE_WITH_KNOTS",
    "BEZIER_CURVE",
    "RATIONAL_B_SPLINE_CURVE",
    "POLYLINE",
    "TRIMMED_CURVE",
];

struct Resolver<'a> {
    raw: &'a HashMap<u64, RawEntity>,
}

impl<'a> Resolver<'a> {
    fn get(&self, id: u64) -> Result<&'a RawEntity, StepImportError> {
        self.raw
            .get(&id)
            .ok_or(StepImportError::MissingRef { id })
    }

    /// Lazily parse the parameter list of `id`. Caller passes the entity's
    /// kind for error messages.
    fn params_of(&self, id: u64) -> Result<Vec<Param>, StepImportError> {
        let ent = self.get(id)?;
        parse_params(&ent.body, ent.line)
    }

    /// Walk every CLOSED_SHELL referenced by every MANIFOLD_SOLID_BREP and
    /// return a single triangle list spanning all of them.
    fn collect_triangles(&self) -> Result<Vec<[Point3; 3]>, StepImportError> {
        let mut tris = Vec::new();
        for (&id, ent) in self.raw {
            if ent.kind == "MANIFOLD_SOLID_BREP" {
                let params = parse_params(&ent.body, ent.line)?;
                // Args: ('label', #shell)
                let shell_id = first_ref(&params).ok_or_else(|| StepImportError::Topology(
                    format!("MANIFOLD_SOLID_BREP #{id} missing CLOSED_SHELL reference"),
                ))?;
                self.collect_shell(shell_id, &mut tris)?;
            }
        }
        Ok(tris)
    }

    fn collect_shell(
        &self,
        shell_id: u64,
        tris: &mut Vec<[Point3; 3]>,
    ) -> Result<(), StepImportError> {
        let ent = self.get(shell_id)?;
        if ent.kind != "CLOSED_SHELL" && ent.kind != "OPEN_SHELL" {
            return Err(StepImportError::Topology(format!(
                "expected CLOSED_SHELL at #{shell_id}, found '{}'",
                ent.kind
            )));
        }
        let params = self.params_of(shell_id)?;
        // Args: ('label', (#face, #face, ...))
        let face_list = first_list(&params).ok_or_else(|| StepImportError::Topology(
            format!("CLOSED_SHELL #{shell_id} missing face list"),
        ))?;
        for fp in face_list {
            if let Param::Ref(face_id) = fp {
                self.collect_face(*face_id, tris)?;
            }
        }
        Ok(())
    }

    fn collect_face(
        &self,
        face_id: u64,
        tris: &mut Vec<[Point3; 3]>,
    ) -> Result<(), StepImportError> {
        let ent = self.get(face_id)?;
        if ent.kind != "ADVANCED_FACE" && ent.kind != "FACE_SURFACE" {
            return Err(StepImportError::Unsupported {
                id: face_id,
                kind: ent.kind.clone(),
            });
        }
        let params = self.params_of(face_id)?;
        // Args: ('label', (#bound, #bound, ...), #surface, .T./.F.)
        let bound_list = first_list(&params).ok_or_else(|| StepImportError::Topology(
            format!("ADVANCED_FACE #{face_id} missing bound list"),
        ))?;
        let surface_ref = first_ref(&params).ok_or_else(|| StepImportError::Topology(
            format!("ADVANCED_FACE #{face_id} missing surface reference"),
        ))?;
        let face_sense = last_enum(&params).map(|s| s == "T").unwrap_or(true);

        // Reject unsupported surfaces with a clear error.
        let surface_ent = self.get(surface_ref)?;
        if UNSUPPORTED_SURFACES.contains(&surface_ent.kind.as_str())
            || !SUPPORTED_SURFACES.contains(&surface_ent.kind.as_str())
        {
            return Err(StepImportError::Unsupported {
                id: surface_ref,
                kind: surface_ent.kind.clone(),
            });
        }

        // Collect outer + inner ring point sequences.
        let mut outer: Option<Vec<Point3>> = None;
        let mut inners: Vec<Vec<Point3>> = Vec::new();
        for bp in bound_list {
            let bound_id = match bp {
                Param::Ref(r) => *r,
                _ => continue,
            };
            let bound_ent = self.get(bound_id)?;
            let is_outer = bound_ent.kind == "FACE_OUTER_BOUND";
            let is_inner = bound_ent.kind == "FACE_BOUND";
            if !is_outer && !is_inner {
                return Err(StepImportError::Unsupported {
                    id: bound_id,
                    kind: bound_ent.kind.clone(),
                });
            }
            let bound_params = self.params_of(bound_id)?;
            // FACE_OUTER_BOUND args: ('label', #edge_loop, .T./.F.)
            let loop_ref = first_ref(&bound_params).ok_or_else(|| StepImportError::Topology(
                format!("FACE_*_BOUND #{bound_id} missing EDGE_LOOP reference"),
            ))?;
            let bound_sense = last_enum(&bound_params).map(|s| s == "T").unwrap_or(true);
            let mut ring = self.collect_loop(loop_ref)?;
            if !bound_sense {
                ring.reverse();
            }
            if !face_sense {
                ring.reverse();
            }
            if is_outer {
                if outer.is_some() {
                    return Err(StepImportError::Topology(format!(
                        "ADVANCED_FACE #{face_id} has multiple outer bounds"
                    )));
                }
                outer = Some(ring);
            } else {
                inners.push(ring);
            }
        }

        let outer = outer.ok_or_else(|| StepImportError::Topology(format!(
            "ADVANCED_FACE #{face_id} has no FACE_OUTER_BOUND"
        )))?;

        // Inner loops (face holes) are out of scope for the polyhedral
        // subset: kerf's STEP export of a planar polyhedron never emits
        // them. Surface a clear error if encountered.
        if !inners.is_empty() {
            return Err(StepImportError::Topology(format!(
                "ADVANCED_FACE #{face_id} has {} inner FACE_BOUND(s) — face holes are out of scope",
                inners.len()
            )));
        }

        triangulate_fan(&outer, tris).map_err(|e| StepImportError::Topology(format!(
            "face #{face_id}: {e}"
        )))?;
        Ok(())
    }

    /// Resolve an EDGE_LOOP into an ordered ring of 3D points (one entry
    /// per loop vertex; the closing edge back to point 0 is implicit).
    fn collect_loop(&self, loop_id: u64) -> Result<Vec<Point3>, StepImportError> {
        let ent = self.get(loop_id)?;
        if ent.kind != "EDGE_LOOP" {
            return Err(StepImportError::Topology(format!(
                "expected EDGE_LOOP at #{loop_id}, found '{}'",
                ent.kind
            )));
        }
        let params = self.params_of(loop_id)?;
        let oedge_list = first_list(&params).ok_or_else(|| StepImportError::Topology(
            format!("EDGE_LOOP #{loop_id} missing oriented-edge list"),
        ))?;
        let mut ring: Vec<Point3> = Vec::with_capacity(oedge_list.len());
        for op in oedge_list {
            let oedge_id = match op {
                Param::Ref(r) => *r,
                _ => continue,
            };
            let oedge_ent = self.get(oedge_id)?;
            if oedge_ent.kind != "ORIENTED_EDGE" {
                return Err(StepImportError::Topology(format!(
                    "expected ORIENTED_EDGE at #{oedge_id}, found '{}'",
                    oedge_ent.kind
                )));
            }
            let oedge_params = self.params_of(oedge_id)?;
            // ORIENTED_EDGE args: ('label', *, *, #edge_curve, .T./.F.)
            let edge_ref = first_ref(&oedge_params).ok_or_else(|| StepImportError::Topology(
                format!("ORIENTED_EDGE #{oedge_id} missing EDGE_CURVE reference"),
            ))?;
            let sense = last_enum(&oedge_params).map(|s| s == "T").unwrap_or(true);
            let (start, end) = self.edge_endpoints(edge_ref)?;
            let from = if sense { start } else { end };
            ring.push(from);
        }
        Ok(ring)
    }

    /// Resolve an EDGE_CURVE into its ordered (start, end) `Point3` pair.
    fn edge_endpoints(
        &self,
        edge_id: u64,
    ) -> Result<(Point3, Point3), StepImportError> {
        let ent = self.get(edge_id)?;
        if ent.kind != "EDGE_CURVE" {
            return Err(StepImportError::Topology(format!(
                "expected EDGE_CURVE at #{edge_id}, found '{}'",
                ent.kind
            )));
        }
        let params = self.params_of(edge_id)?;
        // EDGE_CURVE args: ('label', #v_from, #v_to, #curve, .T./.F.)
        let refs: Vec<u64> = params
            .iter()
            .filter_map(|p| if let Param::Ref(r) = p { Some(*r) } else { None })
            .collect();
        if refs.len() < 3 {
            return Err(StepImportError::Topology(format!(
                "EDGE_CURVE #{edge_id} expected 3 refs (v_from, v_to, curve), got {}",
                refs.len()
            )));
        }
        let v_from = refs[0];
        let v_to = refs[1];
        let curve = refs[2];
        // Reject unsupported curves.
        let curve_ent = self.get(curve)?;
        if UNSUPPORTED_CURVES.contains(&curve_ent.kind.as_str())
            || !SUPPORTED_CURVES.contains(&curve_ent.kind.as_str())
        {
            return Err(StepImportError::Unsupported {
                id: curve,
                kind: curve_ent.kind.clone(),
            });
        }
        let p_from = self.vertex_point(v_from)?;
        let p_to = self.vertex_point(v_to)?;
        Ok((p_from, p_to))
    }

    /// Resolve a VERTEX_POINT into its CARTESIAN_POINT's coordinates.
    fn vertex_point(&self, vid: u64) -> Result<Point3, StepImportError> {
        let ent = self.get(vid)?;
        if ent.kind != "VERTEX_POINT" {
            return Err(StepImportError::Topology(format!(
                "expected VERTEX_POINT at #{vid}, found '{}'",
                ent.kind
            )));
        }
        let params = self.params_of(vid)?;
        let pt_ref = first_ref(&params).ok_or_else(|| StepImportError::Topology(
            format!("VERTEX_POINT #{vid} missing CARTESIAN_POINT reference"),
        ))?;
        self.cartesian_point(pt_ref)
    }

    fn cartesian_point(&self, pid: u64) -> Result<Point3, StepImportError> {
        let ent = self.get(pid)?;
        if ent.kind != "CARTESIAN_POINT" {
            return Err(StepImportError::Topology(format!(
                "expected CARTESIAN_POINT at #{pid}, found '{}'",
                ent.kind
            )));
        }
        let params = self.params_of(pid)?;
        // CARTESIAN_POINT args: ('label', (x, y, z))
        let coords = first_list(&params).ok_or_else(|| StepImportError::Topology(
            format!("CARTESIAN_POINT #{pid} missing coordinate list"),
        ))?;
        if coords.len() < 3 {
            return Err(StepImportError::Topology(format!(
                "CARTESIAN_POINT #{pid} expected 3 coordinates, got {}",
                coords.len()
            )));
        }
        let mut xyz = [0.0_f64; 3];
        for (i, c) in coords.iter().take(3).enumerate() {
            xyz[i] = match c {
                Param::Num(n) => *n,
                _ => {
                    return Err(StepImportError::Topology(format!(
                        "CARTESIAN_POINT #{pid} coordinate {i} is not a number"
                    )))
                }
            };
        }
        Ok(Point3::new(xyz[0], xyz[1], xyz[2]))
    }
}

fn first_ref(params: &[Param]) -> Option<u64> {
    params.iter().find_map(|p| match p {
        Param::Ref(r) => Some(*r),
        _ => None,
    })
}

fn first_list(params: &[Param]) -> Option<&[Param]> {
    params.iter().find_map(|p| match p {
        Param::List(l) => Some(l.as_slice()),
        _ => None,
    })
}

fn last_enum(params: &[Param]) -> Option<&str> {
    params.iter().rev().find_map(|p| match p {
        Param::Enum(e) => Some(e.as_str()),
        _ => None,
    })
}

/// Triangulate a polygon ring by fan from `ring[0]`. Skips degenerate triangles.
fn triangulate_fan(ring: &[Point3], out: &mut Vec<[Point3; 3]>) -> Result<(), String> {
    if ring.len() < 3 {
        return Err(format!("ring has only {} vertices", ring.len()));
    }
    let v0 = ring[0];
    for i in 1..(ring.len() - 1) {
        let v1 = ring[i];
        let v2 = ring[i + 1];
        // Skip exact-coincident triangles; from_triangles will reject any
        // remaining degeneracies.
        if points_equal(v0, v1) || points_equal(v1, v2) || points_equal(v0, v2) {
            continue;
        }
        out.push([v0, v1, v2]);
    }
    Ok(())
}

fn points_equal(a: Point3, b: Point3) -> bool {
    (a.x - b.x).abs() < 1e-9 && (a.y - b.y).abs() < 1e-9 && (a.z - b.z).abs() < 1e-9
}

// ---------------------------------------------------------------------------
// Tests.
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use kerf_brep::measure::solid_volume;

    /// Hand-crafted cube STEP retained as a sanity check that the parser
    /// works on inputs *not* produced by kerf's own writer. Uses the smallest
    /// possible AP214 vocabulary: only the 12 entity types we deliberately
    /// support — no APPLICATION_CONTEXT / unit boilerplate. Also makes the
    /// `unsupported_entity_returns_error` test reusable as a delta from this
    /// fixture.
    /// Cube vertices: 1-4 at z=0 (bottom), 5-8 at z=2 (top), in CCW order
    /// when viewed from +z. Edge curves use a single direction vector per
    /// axis, reused for parallel edges.
    ///
    /// Each face's EDGE_LOOP is wound CCW when viewed from outside the cube
    /// (so the inferred plane normal points outward), and `face_sense=.T.`
    /// matches kerf's writer convention. ORIENTED_EDGE sense is `.T.` if
    /// the loop walk goes start→end of the underlying EDGE_CURVE,
    /// `.F.` otherwise.
    const CUBE_STEP: &str = r#"ISO-10303-21;
HEADER;
FILE_DESCRIPTION(('cube'), '2;1');
FILE_NAME('cube', '2026-05-09T00:00:00', (''), (''), 'kerf', 'kerf', '');
FILE_SCHEMA(('AUTOMOTIVE_DESIGN { 1 0 10303 214 3 1 1 }'));
ENDSEC;
DATA;
#1 = CARTESIAN_POINT('', (0.0, 0.0, 0.0));
#2 = CARTESIAN_POINT('', (2.0, 0.0, 0.0));
#3 = CARTESIAN_POINT('', (2.0, 2.0, 0.0));
#4 = CARTESIAN_POINT('', (0.0, 2.0, 0.0));
#5 = CARTESIAN_POINT('', (0.0, 0.0, 2.0));
#6 = CARTESIAN_POINT('', (2.0, 0.0, 2.0));
#7 = CARTESIAN_POINT('', (2.0, 2.0, 2.0));
#8 = CARTESIAN_POINT('', (0.0, 2.0, 2.0));
#11 = VERTEX_POINT('', #1);
#12 = VERTEX_POINT('', #2);
#13 = VERTEX_POINT('', #3);
#14 = VERTEX_POINT('', #4);
#15 = VERTEX_POINT('', #5);
#16 = VERTEX_POINT('', #6);
#17 = VERTEX_POINT('', #7);
#18 = VERTEX_POINT('', #8);
#20 = DIRECTION('', (1.0, 0.0, 0.0));
#21 = DIRECTION('', (0.0, 1.0, 0.0));
#22 = DIRECTION('', (0.0, 0.0, 1.0));
#23 = VECTOR('', #20, 1.0);
#24 = VECTOR('', #21, 1.0);
#25 = VECTOR('', #22, 1.0);
#30 = LINE('', #1, #23);
#31 = LINE('', #1, #24);
#32 = LINE('', #1, #25);
#50 = EDGE_CURVE('', #11, #12, #30, .T.);
#51 = EDGE_CURVE('', #12, #13, #31, .T.);
#52 = EDGE_CURVE('', #14, #13, #30, .T.);
#53 = EDGE_CURVE('', #11, #14, #31, .T.);
#54 = EDGE_CURVE('', #11, #15, #32, .T.);
#55 = EDGE_CURVE('', #12, #16, #32, .T.);
#56 = EDGE_CURVE('', #13, #17, #32, .T.);
#57 = EDGE_CURVE('', #14, #18, #32, .T.);
#58 = EDGE_CURVE('', #15, #16, #30, .T.);
#59 = EDGE_CURVE('', #16, #17, #31, .T.);
#60 = EDGE_CURVE('', #18, #17, #30, .T.);
#61 = EDGE_CURVE('', #15, #18, #31, .T.);
/* Bottom face (z=0, outward normal -z): CCW from -z view = 1→4→3→2.
   E14(#53,T): v11→v14 via #53. E43: v14→v13 = #52.T (#52 is v14→v13).
   E32: v13→v12 = #51.F (#51 is v12→v13). E21: v12→v11 = #50.F (#50 is v11→v12). */
#100 = ORIENTED_EDGE('', *, *, #53, .T.);
#101 = ORIENTED_EDGE('', *, *, #52, .T.);
#102 = ORIENTED_EDGE('', *, *, #51, .F.);
#103 = ORIENTED_EDGE('', *, *, #50, .F.);
#104 = EDGE_LOOP('', (#100, #101, #102, #103));
/* Top face (z=2, outward normal +z): CCW from +z view = 5→6→7→8.
   Edges: E56(#58,T), E67(#59,T), E78(=E87=#60,F), E85(=E58=#61,F). */
#105 = ORIENTED_EDGE('', *, *, #58, .T.);
#106 = ORIENTED_EDGE('', *, *, #59, .T.);
#107 = ORIENTED_EDGE('', *, *, #60, .F.);
#108 = ORIENTED_EDGE('', *, *, #61, .F.);
#109 = EDGE_LOOP('', (#105, #106, #107, #108));
/* Front face (y=0, outward normal -y): CCW from -y view = 1→2→6→5.
   Edges: E12(#50,T), E26(#55,T), E65(=E56=#58,F), E51(=E15=#54,F). */
#110 = ORIENTED_EDGE('', *, *, #50, .T.);
#111 = ORIENTED_EDGE('', *, *, #55, .T.);
#112 = ORIENTED_EDGE('', *, *, #58, .F.);
#113 = ORIENTED_EDGE('', *, *, #54, .F.);
#114 = EDGE_LOOP('', (#110, #111, #112, #113));
/* Right face (x=2, outward normal +x): CCW from +x view = 2→3→7→6.
   Edges: E23(#51,T), E37(#56,T), E76(=E67=#59,F), E62(=E26=#55,F). */
#115 = ORIENTED_EDGE('', *, *, #51, .T.);
#116 = ORIENTED_EDGE('', *, *, #56, .T.);
#117 = ORIENTED_EDGE('', *, *, #59, .F.);
#118 = ORIENTED_EDGE('', *, *, #55, .F.);
#119 = EDGE_LOOP('', (#115, #116, #117, #118));
/* Back face (y=2, outward normal +y): CCW from +y view = 4→8→7→3.
   E48: v14→v18 = #57.T. E87: v18→v17 = #60.T (#60 is v18→v17).
   E73: v17→v13 = #56.F (#56 is v13→v17). E34: v13→v14 = #52.F (#52 is v14→v13). */
#120 = ORIENTED_EDGE('', *, *, #57, .T.);
#121 = ORIENTED_EDGE('', *, *, #60, .T.);
#122 = ORIENTED_EDGE('', *, *, #56, .F.);
#123 = ORIENTED_EDGE('', *, *, #52, .F.);
#124 = EDGE_LOOP('', (#120, #121, #122, #123));
/* Left face (x=0, outward normal -x): CCW from -x view = 1→5→8→4.
   Edges: E15(#54,T), E58(#61,T), E84(=E48=#57,F), E41(=E14=#53,F). */
#125 = ORIENTED_EDGE('', *, *, #54, .T.);
#126 = ORIENTED_EDGE('', *, *, #61, .T.);
#127 = ORIENTED_EDGE('', *, *, #57, .F.);
#128 = ORIENTED_EDGE('', *, *, #53, .F.);
#129 = EDGE_LOOP('', (#125, #126, #127, #128));
#150 = FACE_OUTER_BOUND('', #104, .T.);
#151 = FACE_OUTER_BOUND('', #109, .T.);
#152 = FACE_OUTER_BOUND('', #114, .T.);
#153 = FACE_OUTER_BOUND('', #119, .T.);
#154 = FACE_OUTER_BOUND('', #124, .T.);
#155 = FACE_OUTER_BOUND('', #129, .T.);
#170 = AXIS2_PLACEMENT_3D('', #1, #22, #20);
#171 = AXIS2_PLACEMENT_3D('', #5, #22, #20);
#172 = AXIS2_PLACEMENT_3D('', #1, #21, #20);
#173 = AXIS2_PLACEMENT_3D('', #2, #20, #21);
#174 = AXIS2_PLACEMENT_3D('', #4, #21, #20);
#175 = AXIS2_PLACEMENT_3D('', #1, #20, #21);
#180 = PLANE('', #170);
#181 = PLANE('', #171);
#182 = PLANE('', #172);
#183 = PLANE('', #173);
#184 = PLANE('', #174);
#185 = PLANE('', #175);
#200 = ADVANCED_FACE('', (#150), #180, .T.);
#201 = ADVANCED_FACE('', (#151), #181, .T.);
#202 = ADVANCED_FACE('', (#152), #182, .T.);
#203 = ADVANCED_FACE('', (#153), #183, .T.);
#204 = ADVANCED_FACE('', (#154), #184, .T.);
#205 = ADVANCED_FACE('', (#155), #185, .T.);
#300 = CLOSED_SHELL('', (#200, #201, #202, #203, #204, #205));
#400 = MANIFOLD_SOLID_BREP('', #300);
ENDSEC;
END-ISO-10303-21;
"#;

    #[test]
    fn import_cube_volume_is_8() {
        let s = import_step(CUBE_STEP).expect("import");
        let v = solid_volume(&s);
        assert!(
            (v - 8.0).abs() < 1e-6,
            "cube volume = {v}, expected 8.0"
        );
    }

    #[test]
    fn import_cube_face_count() {
        let s = import_step(CUBE_STEP).expect("import");
        // 6 quad faces, each fan-triangulated into 2 triangles → 12 faces.
        assert_eq!(s.face_count(), 12);
    }

    #[test]
    fn round_trip_through_kerf_step_export() {
        // Export a kerf-cad Box → STEP, re-import → same vertex / face count
        // (after re-triangulation: 6 quads × 2 triangles = 12 triangles
        // which dedup back to 8 vertices).
        use crate::feature::Feature;
        use crate::scalar::Scalar;
        use crate::Model;
        use kerf_brep::write_step;

        let m = Model::new().add(Feature::Box {
            id: "b".into(),
            extents: [Scalar::lit(2.0), Scalar::lit(2.0), Scalar::lit(2.0)],
        });
        let solid = m.evaluate("b").unwrap();
        let mut buf: Vec<u8> = Vec::new();
        write_step(&solid, "b", &mut buf).unwrap();
        let text = String::from_utf8(buf).unwrap();
        let imported = import_step(&text).expect("re-import kerf STEP");
        assert_eq!(imported.vertex_count(), 8, "cube has 8 verts");
        // Triangulation produces 12 tri-faces.
        assert_eq!(imported.face_count(), 12);
        let v = solid_volume(&imported);
        assert!((v - 8.0).abs() < 1e-6, "round-trip volume = {v}");
    }

    #[test]
    fn unsupported_entity_returns_error() {
        // Same prelude, but face #200 references a CYLINDRICAL_SURFACE.
        let bad = r#"ISO-10303-21;
HEADER;
FILE_DESCRIPTION(('x'),'2;1');
FILE_NAME('x','',(''),(''),'','','');
FILE_SCHEMA(('AUTOMOTIVE_DESIGN { 1 0 10303 214 3 1 1 }'));
ENDSEC;
DATA;
#1 = CARTESIAN_POINT('', (0.0, 0.0, 0.0));
#2 = DIRECTION('', (1.0, 0.0, 0.0));
#3 = DIRECTION('', (0.0, 0.0, 1.0));
#4 = AXIS2_PLACEMENT_3D('', #1, #3, #2);
#5 = CYLINDRICAL_SURFACE('', #4, 1.0);
#10 = VERTEX_POINT('', #1);
#11 = VERTEX_POINT('', #1);
#12 = LINE('', #1, #2);
#13 = EDGE_CURVE('', #10, #11, #12, .T.);
#14 = ORIENTED_EDGE('', *, *, #13, .T.);
#15 = EDGE_LOOP('', (#14));
#16 = FACE_OUTER_BOUND('', #15, .T.);
#17 = ADVANCED_FACE('', (#16), #5, .T.);
#18 = CLOSED_SHELL('', (#17));
#19 = MANIFOLD_SOLID_BREP('', #18);
ENDSEC;
END-ISO-10303-21;
"#;
        let err = import_step(bad).expect_err("expected unsupported");
        match err {
            StepImportError::Unsupported { kind, .. } => {
                assert_eq!(kind, "CYLINDRICAL_SURFACE");
            }
            other => panic!("expected Unsupported, got {other:?}"),
        }
    }

    #[test]
    fn missing_reference_returns_error() {
        let bad = r#"ISO-10303-21;
HEADER;
FILE_DESCRIPTION(('x'),'2;1');
FILE_NAME('x','',(''),(''),'','','');
FILE_SCHEMA(('AUTOMOTIVE_DESIGN { 1 0 10303 214 3 1 1 }'));
ENDSEC;
DATA;
#1 = MANIFOLD_SOLID_BREP('', #999);
ENDSEC;
END-ISO-10303-21;
"#;
        let err = import_step(bad).expect_err("expected missing ref");
        match err {
            StepImportError::MissingRef { id } => assert_eq!(id, 999),
            other => panic!("expected MissingRef, got {other:?}"),
        }
    }

    #[test]
    fn malformed_file_returns_parse_error_with_line_number() {
        let bad = "ISO-10303-21;\nHEADER;\nENDSEC;\nDATA;\n#1 = NO_PARENS;\nENDSEC;\nEND-ISO-10303-21;\n";
        let err = import_step(bad).expect_err("expected parse error");
        match err {
            StepImportError::Parse { line, message } => {
                assert!(line >= 1, "line number {line} should be sensible");
                assert!(message.contains('('), "message should mention '(': {message}");
            }
            other => panic!("expected Parse, got {other:?}"),
        }
    }

    #[test]
    fn missing_data_section_is_parse_error() {
        let bad = "ISO-10303-21;\nHEADER;\nENDSEC;\nEND-ISO-10303-21;\n";
        let err = import_step(bad).expect_err("expected parse error");
        match err {
            StepImportError::Parse { message, .. } => {
                assert!(message.contains("DATA"), "message: {message}");
            }
            other => panic!("expected Parse, got {other:?}"),
        }
    }

    #[test]
    fn empty_data_section_yields_topology_error() {
        let bad =
            "ISO-10303-21;\nHEADER;\nENDSEC;\nDATA;\nENDSEC;\nEND-ISO-10303-21;\n";
        let err = import_step(bad).expect_err("expected topology error");
        match err {
            StepImportError::Topology(_) => {}
            other => panic!("expected Topology, got {other:?}"),
        }
    }
}
