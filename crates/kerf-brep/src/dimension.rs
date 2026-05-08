//! Dimensioned 2D drawings: orthographic projections, distance/angle/projection
//! helpers, and an SVG renderer that overlays dimension annotations on a
//! solid's silhouette.
//!
//! This is the back-end half of the "Drawings" capability tracked in STATUS.md.
//! The viewer can either drive these helpers via the WASM API to compute
//! measurements (so it doesn't need to re-implement the math in TypeScript),
//! or it can ask for a fully-rendered SVG and embed it directly.
//!
//! ## View conventions
//!
//! - `Top`     looks down -Z; world (X, Y) → screen (x, -y) (Y up on paper).
//! - `Front`   looks along -Y; world (X, Z) → screen (x, -z).
//! - `Side`    (right side) looks along -X; world (Y, Z) → screen (-y, -z).
//! - `Iso`     a fixed isometric direction; uses the same mapping as the
//!             viewer's drawings.ts ISO cell so the SVG and PNG are
//!             visually consistent.
//!
//! Screen coordinates are in *world units* — the SVG renderer applies a
//! `viewport` transform to lay them out on the page, with Y flipped because
//! SVG Y points down.

use crate::tessellate::tessellate;
use crate::Solid;
use kerf_geom::{Point3, Vec3};

// --- View kinds ---------------------------------------------------------

/// Which orthographic / isometric projection to use.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ViewKind {
    Top,
    Front,
    Side,
    Iso,
}

impl ViewKind {
    /// Parse a case-insensitive view name. Used by the WASM bindings so
    /// the front-end can pass `"top"` / `"front"` / `"side"` / `"iso"`.
    pub fn from_str(name: &str) -> Option<Self> {
        match name.trim().to_ascii_lowercase().as_str() {
            "top" => Some(ViewKind::Top),
            "front" => Some(ViewKind::Front),
            "side" | "right" => Some(ViewKind::Side),
            "iso" | "isometric" => Some(ViewKind::Iso),
            _ => None,
        }
    }

    pub fn as_str(self) -> &'static str {
        match self {
            ViewKind::Top => "top",
            ViewKind::Front => "front",
            ViewKind::Side => "side",
            ViewKind::Iso => "iso",
        }
    }
}

// --- Pure math: distance, angle, projection -----------------------------

/// Euclidean distance between two world-space points.
pub fn distance(p1: Point3, p2: Point3) -> f64 {
    (p2 - p1).norm()
}

/// Angle between two vectors in radians, in [0, pi]. Returns 0 if either
/// vector has zero length (degenerate; callers can decide how to surface).
pub fn angle_between_vectors(a: Vec3, b: Vec3) -> f64 {
    let na = a.norm();
    let nb = b.norm();
    if na <= f64::EPSILON || nb <= f64::EPSILON {
        return 0.0;
    }
    let cos = (a.dot(&b) / (na * nb)).clamp(-1.0, 1.0);
    cos.acos()
}

/// Angle (radians) at `vertex` formed by rays to `a` and `b`.
pub fn angle_at_vertex(a: Point3, vertex: Point3, b: Point3) -> f64 {
    angle_between_vectors(a - vertex, b - vertex)
}

/// Project a point onto a plane defined by an origin and a normal.
/// The normal need not be unit-length; degenerate normals (zero length)
/// fall back to returning `p` unchanged.
pub fn project_to_plane(p: Point3, plane_normal: Vec3, plane_origin: Point3) -> Point3 {
    let n2 = plane_normal.norm_squared();
    if n2 <= f64::EPSILON {
        return p;
    }
    let v = p - plane_origin;
    let t = v.dot(&plane_normal) / n2;
    p - plane_normal * t
}

/// Project a 3D point into 2D screen coordinates for the given view.
///
/// Returned pair is `(u, v)` in *world units*: u points right on the page,
/// v points up on the page. The SVG renderer flips v to SVG's Y-down later.
pub fn to_2d_view(p: Point3, view: ViewKind) -> (f64, f64) {
    match view {
        // Looking down -Z: paper X = world X, paper Y = world Y.
        ViewKind::Top => (p.x, p.y),
        // Looking along -Y: paper X = world X, paper Y = world Z.
        ViewKind::Front => (p.x, p.z),
        // Looking along -X (right side): paper X = world Y, paper Y = world Z.
        // Note: drawings.ts uses world Y as the horizontal axis on the right
        // view; we follow the same convention so the projections agree.
        ViewKind::Side => (p.y, p.z),
        // Iso: simple cabinet-style projection that matches the spirit of
        // the viewer's ISO camera (forward = (-1, -0.7, -1.2).normalize(),
        // up = +Z). We don't try to be pixel-identical with three.js — this
        // is just a 2D fallback for the SVG renderer.
        ViewKind::Iso => {
            // 30-degree isometric: a standard CAD convention.
            let c = (30.0_f64).to_radians().cos();
            let s = (30.0_f64).to_radians().sin();
            let u = (p.x - p.y) * c;
            let v = p.z + (p.x + p.y) * s;
            (u, v)
        }
    }
}

// --- Dimension annotations ---------------------------------------------

/// A single dimension annotation. The points are in world space; the
/// renderer projects them through the requested view.
#[derive(Debug, Clone)]
pub struct Dimension {
    pub from: Point3,
    pub to: Point3,
    pub kind: DimensionKind,
}

#[derive(Debug, Clone)]
pub enum DimensionKind {
    /// Straight linear distance |to - from|.
    Linear,
    /// Radial distance from a fixed center (e.g. a hole). The annotation
    /// renders as a leader line from `center` to `to` (the point on the
    /// circle), labelled with R<value>.
    RadialFromCenter { center: Point3 },
    /// Angular dimension at `vertex`, between rays to `from` and `to`.
    /// Renders as an arc plus a degree label.
    Angular { vertex: Point3 },
}

/// Viewport spec for the SVG renderer. The renderer fits the projected
/// silhouette into `(width - 2*padding) x (height - 2*padding)` while
/// preserving aspect ratio.
#[derive(Debug, Clone, Copy)]
pub struct ViewportSpec {
    pub width: f64,
    pub height: f64,
    pub padding: f64,
}

impl ViewportSpec {
    pub fn new(width: f64, height: f64, padding: f64) -> Self {
        Self { width, height, padding }
    }

    pub fn default_a4_ish() -> Self {
        Self { width: 480.0, height: 360.0, padding: 36.0 }
    }
}

// --- Silhouette extraction ---------------------------------------------

/// Project the solid's tessellation into the requested view and return the
/// outer silhouette as a closed polygon in *world units* (paper coords).
///
/// Strategy: tessellate, project every triangle vertex, take the convex
/// hull. The convex hull is a strict superset of the silhouette for
/// non-convex solids, but it's cheap, robust, and produces a recognizable
/// outline for the common case (rectangular / cylindrical parts). The
/// `render_dimensioned_view` doc-comment calls out the limitation; a
/// future improvement is to walk the edge graph and emit only edges whose
/// adjacent face normals have opposite signs in the view direction.
pub fn projected_silhouette(solid: &Solid, view: ViewKind, segments: usize) -> Vec<(f64, f64)> {
    let soup = tessellate(solid, segments.max(3));
    let mut pts: Vec<(f64, f64)> = Vec::with_capacity(soup.triangles.len() * 3);
    for tri in &soup.triangles {
        for v in tri {
            pts.push(to_2d_view(*v, view));
        }
    }
    if pts.is_empty() {
        // Degenerate / empty solid — return an empty polygon, the renderer
        // just falls back to drawing the dimensions with no outline.
        return pts;
    }
    convex_hull(pts)
}

/// Andrew's monotone-chain convex hull. Returns the hull in CCW order
/// without duplicating the start point.
fn convex_hull(mut pts: Vec<(f64, f64)>) -> Vec<(f64, f64)> {
    pts.sort_by(|a, b| {
        a.0.partial_cmp(&b.0)
            .unwrap_or(std::cmp::Ordering::Equal)
            .then(a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal))
    });
    pts.dedup_by(|a, b| (a.0 - b.0).abs() < 1e-12 && (a.1 - b.1).abs() < 1e-12);
    let n = pts.len();
    if n < 2 {
        return pts;
    }
    let cross = |o: (f64, f64), a: (f64, f64), b: (f64, f64)| -> f64 {
        (a.0 - o.0) * (b.1 - o.1) - (a.1 - o.1) * (b.0 - o.0)
    };
    let mut lower: Vec<(f64, f64)> = Vec::new();
    for &p in &pts {
        while lower.len() >= 2 && cross(lower[lower.len() - 2], lower[lower.len() - 1], p) <= 0.0 {
            lower.pop();
        }
        lower.push(p);
    }
    let mut upper: Vec<(f64, f64)> = Vec::new();
    for &p in pts.iter().rev() {
        while upper.len() >= 2 && cross(upper[upper.len() - 2], upper[upper.len() - 1], p) <= 0.0 {
            upper.pop();
        }
        upper.push(p);
    }
    lower.pop();
    upper.pop();
    lower.extend(upper);
    lower
}

/// Axis-aligned bounding box of a 2D point cloud as `(min_u, min_v, max_u, max_v)`.
fn bbox_2d(pts: &[(f64, f64)]) -> (f64, f64, f64, f64) {
    if pts.is_empty() {
        return (0.0, 0.0, 0.0, 0.0);
    }
    let mut min_u = f64::INFINITY;
    let mut min_v = f64::INFINITY;
    let mut max_u = f64::NEG_INFINITY;
    let mut max_v = f64::NEG_INFINITY;
    for &(u, v) in pts {
        if u < min_u { min_u = u; }
        if v < min_v { min_v = v; }
        if u > max_u { max_u = u; }
        if v > max_v { max_v = v; }
    }
    (min_u, min_v, max_u, max_v)
}

// --- SVG renderer ------------------------------------------------------

/// Render `solid` to an SVG string with the requested view, dimensions,
/// and viewport.
///
/// Output layout:
/// - One root `<svg>` with `viewBox = "0 0 width height"`.
/// - The silhouette as a single `<polygon class="kerf-silhouette">`, or
///   nothing if the solid is empty.
/// - One `<g class="kerf-dim">` per dimension, containing dimension lines,
///   arrowheads (paths), and a `<text>` with the measured value.
///
/// Limitations (doc'd here so the front-end knows what to expect):
/// - The silhouette is the convex hull of the tessellated vertices, not a
///   true outline. Concave parts (e.g. an L-bracket viewed from above)
///   render as their bounding shape.
/// - Hidden lines / dashed edges are not drawn.
/// - The renderer does no font management; dimension text uses generic
///   `font-family="sans-serif"`.
pub fn render_dimensioned_view(
    solid: &Solid,
    view: ViewKind,
    dimensions: &[Dimension],
    viewport: ViewportSpec,
) -> String {
    let segments = 24;
    let silhouette = projected_silhouette(solid, view, segments);

    // Compute extents that include both the silhouette and every dimension
    // point, so dimensions placed off the part itself still fit on the page.
    let mut all_pts: Vec<(f64, f64)> = silhouette.clone();
    for d in dimensions {
        all_pts.push(to_2d_view(d.from, view));
        all_pts.push(to_2d_view(d.to, view));
        if let DimensionKind::RadialFromCenter { center } = &d.kind {
            all_pts.push(to_2d_view(*center, view));
        }
        if let DimensionKind::Angular { vertex } = &d.kind {
            all_pts.push(to_2d_view(*vertex, view));
        }
    }
    let (min_u, min_v, max_u, max_v) = bbox_2d(&all_pts);

    // Compute uniform scale that maps the world bbox to the inner viewport.
    let inner_w = (viewport.width - 2.0 * viewport.padding).max(1.0);
    let inner_h = (viewport.height - 2.0 * viewport.padding).max(1.0);
    let span_u = (max_u - min_u).max(1e-9);
    let span_v = (max_v - min_v).max(1e-9);
    let scale = (inner_w / span_u).min(inner_h / span_v);

    // Center the projected content inside the viewport.
    let used_w = span_u * scale;
    let used_h = span_v * scale;
    let off_x = viewport.padding + (inner_w - used_w) * 0.5;
    let off_y = viewport.padding + (inner_h - used_h) * 0.5;

    // Closure: world (u, v) → SVG (x, y). SVG Y points down, so flip v.
    let to_svg = |u: f64, v: f64| -> (f64, f64) {
        let x = off_x + (u - min_u) * scale;
        let y = off_y + (max_v - v) * scale;
        (x, y)
    };

    let mut s = String::new();
    s.push_str(&format!(
        "<svg xmlns=\"http://www.w3.org/2000/svg\" viewBox=\"0 0 {w} {h}\" \
         width=\"{w}\" height=\"{h}\" font-family=\"sans-serif\" font-size=\"11\">\n",
        w = viewport.width,
        h = viewport.height,
    ));
    // Background + view label.
    s.push_str("  <rect width=\"100%\" height=\"100%\" fill=\"#ffffff\"/>\n");
    s.push_str(&format!(
        "  <text x=\"{x}\" y=\"{y}\" fill=\"#161b22\" font-weight=\"bold\">{label}</text>\n",
        x = 8.0,
        y = 16.0,
        label = view.as_str().to_uppercase(),
    ));

    // Silhouette polygon.
    if silhouette.len() >= 3 {
        let pts_str: Vec<String> = silhouette
            .iter()
            .map(|&(u, v)| {
                let (x, y) = to_svg(u, v);
                format!("{:.3},{:.3}", x, y)
            })
            .collect();
        s.push_str(&format!(
            "  <polygon class=\"kerf-silhouette\" points=\"{}\" \
             fill=\"#c8d3df\" fill-opacity=\"0.5\" stroke=\"#222831\" stroke-width=\"1\"/>\n",
            pts_str.join(" ")
        ));
    }

    // Dimensions.
    for d in dimensions {
        match &d.kind {
            DimensionKind::Linear => render_linear(&mut s, d, view, &to_svg),
            DimensionKind::RadialFromCenter { center } => {
                render_radial(&mut s, d, *center, view, &to_svg)
            }
            DimensionKind::Angular { vertex } => {
                render_angular(&mut s, d, *vertex, view, &to_svg)
            }
        }
    }

    s.push_str("</svg>\n");
    s
}

fn render_linear<F: Fn(f64, f64) -> (f64, f64)>(
    out: &mut String,
    d: &Dimension,
    view: ViewKind,
    to_svg: &F,
) {
    let value = distance(d.from, d.to);
    let (u1, v1) = to_2d_view(d.from, view);
    let (u2, v2) = to_2d_view(d.to, view);
    let (x1, y1) = to_svg(u1, v1);
    let (x2, y2) = to_svg(u2, v2);
    let mx = (x1 + x2) * 0.5;
    let my = (y1 + y2) * 0.5;
    out.push_str("  <g class=\"kerf-dim kerf-dim-linear\" stroke=\"#3f6cba\" fill=\"#3f6cba\">\n");
    out.push_str(&format!(
        "    <line x1=\"{x1:.3}\" y1=\"{y1:.3}\" x2=\"{x2:.3}\" y2=\"{y2:.3}\" stroke-width=\"1\"/>\n"
    ));
    write_arrowhead(out, x2, y2, x1, y1);
    write_arrowhead(out, x1, y1, x2, y2);
    out.push_str(&format!(
        "    <text x=\"{mx:.3}\" y=\"{my:.3}\" dy=\"-4\" text-anchor=\"middle\" stroke=\"none\">{val:.3}</text>\n",
        val = value,
    ));
    out.push_str("  </g>\n");
}

fn render_radial<F: Fn(f64, f64) -> (f64, f64)>(
    out: &mut String,
    d: &Dimension,
    center: Point3,
    view: ViewKind,
    to_svg: &F,
) {
    let value = distance(center, d.to);
    let (cu, cv) = to_2d_view(center, view);
    let (u2, v2) = to_2d_view(d.to, view);
    let (cx, cy) = to_svg(cu, cv);
    let (x2, y2) = to_svg(u2, v2);
    out.push_str("  <g class=\"kerf-dim kerf-dim-radial\" stroke=\"#3f6cba\" fill=\"#3f6cba\">\n");
    out.push_str(&format!(
        "    <line x1=\"{cx:.3}\" y1=\"{cy:.3}\" x2=\"{x2:.3}\" y2=\"{y2:.3}\" stroke-width=\"1\"/>\n"
    ));
    write_arrowhead(out, cx, cy, x2, y2);
    let mx = (cx + x2) * 0.5;
    let my = (cy + y2) * 0.5;
    out.push_str(&format!(
        "    <text x=\"{mx:.3}\" y=\"{my:.3}\" dy=\"-4\" text-anchor=\"middle\" stroke=\"none\">R{val:.3}</text>\n",
        val = value,
    ));
    out.push_str("  </g>\n");
}

fn render_angular<F: Fn(f64, f64) -> (f64, f64)>(
    out: &mut String,
    d: &Dimension,
    vertex: Point3,
    view: ViewKind,
    to_svg: &F,
) {
    let radians = angle_at_vertex(d.from, vertex, d.to);
    let degrees = radians.to_degrees();
    let (vu, vv) = to_2d_view(vertex, view);
    let (au, av) = to_2d_view(d.from, view);
    let (bu, bv) = to_2d_view(d.to, view);
    let (vx, vy) = to_svg(vu, vv);
    let (ax, ay) = to_svg(au, av);
    let (bx, by) = to_svg(bu, bv);
    out.push_str("  <g class=\"kerf-dim kerf-dim-angular\" stroke=\"#3f6cba\" fill=\"#3f6cba\">\n");
    out.push_str(&format!(
        "    <line x1=\"{vx:.3}\" y1=\"{vy:.3}\" x2=\"{ax:.3}\" y2=\"{ay:.3}\" stroke-width=\"1\"/>\n"
    ));
    out.push_str(&format!(
        "    <line x1=\"{vx:.3}\" y1=\"{vy:.3}\" x2=\"{bx:.3}\" y2=\"{by:.3}\" stroke-width=\"1\"/>\n"
    ));
    out.push_str(&format!(
        "    <text x=\"{vx:.3}\" y=\"{vy:.3}\" dy=\"-6\" text-anchor=\"middle\" stroke=\"none\">{deg:.2}°</text>\n",
        deg = degrees,
    ));
    out.push_str("  </g>\n");
}

/// Append an arrowhead (small filled triangle) at `(tip_x, tip_y)`,
/// pointing away from `(tail_x, tail_y)`.
fn write_arrowhead(out: &mut String, tail_x: f64, tail_y: f64, tip_x: f64, tip_y: f64) {
    let dx = tip_x - tail_x;
    let dy = tip_y - tail_y;
    let len = (dx * dx + dy * dy).sqrt();
    if len < 1e-9 {
        return;
    }
    let ux = dx / len;
    let uy = dy / len;
    let head = 6.0_f64;
    // Two base corners perpendicular to the shaft.
    let base_x = tip_x - ux * head;
    let base_y = tip_y - uy * head;
    let half = head * 0.4;
    let p1x = base_x + (-uy) * half;
    let p1y = base_y + ux * half;
    let p2x = base_x - (-uy) * half;
    let p2y = base_y - ux * half;
    out.push_str(&format!(
        "    <path d=\"M{tip_x:.3},{tip_y:.3} L{p1x:.3},{p1y:.3} L{p2x:.3},{p2y:.3} Z\" stroke=\"none\"/>\n"
    ));
}

// -----------------------------------------------------------------------
// Tests
// -----------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::primitives::box_at;
    use kerf_geom::{Point3, Vec3};

    #[test]
    fn distance_between_known_points() {
        let p1 = Point3::new(0.0, 0.0, 0.0);
        let p2 = Point3::new(3.0, 4.0, 0.0);
        assert!((distance(p1, p2) - 5.0).abs() < 1e-12);

        let p3 = Point3::new(1.0, 2.0, 2.0);
        let p4 = Point3::new(1.0, 2.0, 2.0);
        assert!(distance(p3, p4).abs() < 1e-12);

        let p5 = Point3::new(-1.0, -1.0, -1.0);
        let p6 = Point3::new(1.0, 1.0, 1.0);
        assert!((distance(p5, p6) - (12.0_f64).sqrt()).abs() < 1e-12);
    }

    #[test]
    fn angle_between_orthogonal_and_parallel() {
        let x = Vec3::new(1.0, 0.0, 0.0);
        let y = Vec3::new(0.0, 1.0, 0.0);
        let mx = Vec3::new(-1.0, 0.0, 0.0);
        let two_x = Vec3::new(2.0, 0.0, 0.0);
        // Orthogonal → pi/2.
        assert!((angle_between_vectors(x, y) - std::f64::consts::FRAC_PI_2).abs() < 1e-12);
        // Parallel-same → 0.
        assert!(angle_between_vectors(x, two_x).abs() < 1e-12);
        // Anti-parallel → pi.
        assert!((angle_between_vectors(x, mx) - std::f64::consts::PI).abs() < 1e-12);
        // Zero-length input → 0 (degenerate; documented).
        assert_eq!(angle_between_vectors(Vec3::zeros(), x), 0.0);
    }

    #[test]
    fn angle_at_vertex_right_angle() {
        let v = Point3::new(0.0, 0.0, 0.0);
        let a = Point3::new(1.0, 0.0, 0.0);
        let b = Point3::new(0.0, 1.0, 0.0);
        let theta = angle_at_vertex(a, v, b);
        assert!((theta - std::f64::consts::FRAC_PI_2).abs() < 1e-12);
    }

    #[test]
    fn project_to_plane_round_trip() {
        // A point above the XY plane projects to its (x, y, 0) shadow,
        // and projecting again is idempotent.
        let p = Point3::new(2.0, -3.0, 5.0);
        let n = Vec3::new(0.0, 0.0, 1.0);
        let o = Point3::new(0.0, 0.0, 0.0);
        let q = project_to_plane(p, n, o);
        assert!((q.x - 2.0).abs() < 1e-12);
        assert!((q.y - (-3.0)).abs() < 1e-12);
        assert!(q.z.abs() < 1e-12);
        // Idempotence.
        let q2 = project_to_plane(q, n, o);
        assert!((q - q2).norm() < 1e-12);

        // Non-axis plane: project a point onto the plane through origin
        // with normal (1, 1, 0) / sqrt(2). The result should satisfy
        // (q - o) . n == 0.
        let n2 = Vec3::new(1.0, 1.0, 0.0);
        let q3 = project_to_plane(Point3::new(3.0, 0.0, 7.0), n2, o);
        let residual = (q3 - o).dot(&n2);
        assert!(residual.abs() < 1e-12, "residual {}", residual);

        // Degenerate normal: should return p unchanged.
        let q_deg = project_to_plane(p, Vec3::zeros(), o);
        assert!((q_deg - p).norm() < 1e-12);
    }

    #[test]
    fn to_2d_view_axis_mapping() {
        let p = Point3::new(2.0, -3.0, 5.0);
        assert_eq!(to_2d_view(p, ViewKind::Top), (2.0, -3.0));
        assert_eq!(to_2d_view(p, ViewKind::Front), (2.0, 5.0));
        assert_eq!(to_2d_view(p, ViewKind::Side), (-3.0, 5.0));
        // Iso just needs to be deterministic and non-degenerate.
        let (u, v) = to_2d_view(p, ViewKind::Iso);
        assert!(u.is_finite() && v.is_finite());
    }

    #[test]
    fn view_kind_parse() {
        assert_eq!(ViewKind::from_str("top"), Some(ViewKind::Top));
        assert_eq!(ViewKind::from_str("FRONT"), Some(ViewKind::Front));
        assert_eq!(ViewKind::from_str(" Side "), Some(ViewKind::Side));
        assert_eq!(ViewKind::from_str("right"), Some(ViewKind::Side));
        assert_eq!(ViewKind::from_str("ISO"), Some(ViewKind::Iso));
        assert_eq!(ViewKind::from_str("oblique"), None);
    }

    #[test]
    fn render_top_view_of_box_contains_dim_line_and_text() {
        // 2x4x1 box centered at origin. Top view sees the 2x4 rectangle.
        let s = box_at(Vec3::new(2.0, 4.0, 1.0), Point3::new(0.0, 0.0, 0.0));
        let from = Point3::new(-1.0, -2.0, 0.5);
        let to = Point3::new(1.0, -2.0, 0.5);
        let dim = Dimension { from, to, kind: DimensionKind::Linear };
        let svg = render_dimensioned_view(
            &s,
            ViewKind::Top,
            std::slice::from_ref(&dim),
            ViewportSpec::default_a4_ish(),
        );
        // Header
        assert!(svg.starts_with("<svg"), "missing <svg> root: {}", &svg[..40.min(svg.len())]);
        assert!(svg.contains("</svg>"));
        // Silhouette polygon
        assert!(svg.contains("kerf-silhouette"), "no silhouette polygon");
        assert!(svg.contains("<polygon"));
        // Dimension group + line + arrowhead path
        assert!(svg.contains("kerf-dim-linear"), "no linear-dim group");
        assert!(svg.contains("<line"), "no <line> for dimension");
        // Measurement value: |to - from| = 2.0, formatted with 3 decimals.
        assert!(svg.contains(">2.000<"), "expected dim value 2.000 in text:\n{}", svg);
        // View label
        assert!(svg.contains(">TOP<"), "expected view label TOP");
    }

    #[test]
    fn render_radial_dimension_renders_r_prefix() {
        let s = box_at(Vec3::new(4.0, 4.0, 1.0), Point3::new(0.0, 0.0, 0.0));
        let dim = Dimension {
            from: Point3::new(0.0, 0.0, 0.0),
            to: Point3::new(2.0, 0.0, 0.0),
            kind: DimensionKind::RadialFromCenter { center: Point3::new(0.0, 0.0, 0.0) },
        };
        let svg = render_dimensioned_view(
            &s,
            ViewKind::Top,
            std::slice::from_ref(&dim),
            ViewportSpec::default_a4_ish(),
        );
        assert!(svg.contains("kerf-dim-radial"));
        assert!(svg.contains(">R2.000<"), "expected R2.000 in:\n{}", svg);
    }

    #[test]
    fn render_angular_dimension_emits_degrees() {
        let s = box_at(Vec3::new(2.0, 2.0, 1.0), Point3::new(0.0, 0.0, 0.0));
        // 90 degrees at origin between +X ray and +Y ray.
        let dim = Dimension {
            from: Point3::new(1.0, 0.0, 0.0),
            to: Point3::new(0.0, 1.0, 0.0),
            kind: DimensionKind::Angular { vertex: Point3::new(0.0, 0.0, 0.0) },
        };
        let svg = render_dimensioned_view(
            &s,
            ViewKind::Top,
            std::slice::from_ref(&dim),
            ViewportSpec::default_a4_ish(),
        );
        assert!(svg.contains("kerf-dim-angular"));
        // Degrees value is 90.00 (with two decimals).
        assert!(svg.contains(">90.00°<"), "expected 90.00° in:\n{}", svg);
    }

    #[test]
    fn empty_solid_renders_blank_svg() {
        // No silhouette polygon when solid is empty.
        let s = Solid::new();
        let svg = render_dimensioned_view(&s, ViewKind::Top, &[], ViewportSpec::default_a4_ish());
        assert!(svg.starts_with("<svg"));
        assert!(svg.contains("</svg>"));
        // No silhouette node when there's nothing to draw.
        assert!(!svg.contains("kerf-silhouette"));
    }

    #[test]
    fn convex_hull_of_unit_square() {
        let pts = vec![
            (0.0, 0.0),
            (1.0, 0.0),
            (1.0, 1.0),
            (0.0, 1.0),
            (0.5, 0.5),  // interior, should be discarded
        ];
        let hull = convex_hull(pts);
        assert_eq!(hull.len(), 4);
        // All four corners must appear.
        let has = |u: f64, v: f64| hull.iter().any(|&(a, b)| (a - u).abs() < 1e-9 && (b - v).abs() < 1e-9);
        assert!(has(0.0, 0.0));
        assert!(has(1.0, 0.0));
        assert!(has(1.0, 1.0));
        assert!(has(0.0, 1.0));
    }
}
