//! Triangulate a convex polygon by fanning from its first vertex.

use kerf_geom::Point3;

/// Fan-triangulate a convex polygon. Returns a list of triangles.
/// For a polygon with n vertices, returns n-2 triangles.
pub fn fan_triangulate(poly: &[Point3]) -> Vec<[Point3; 3]> {
    if poly.len() < 3 { return Vec::new(); }
    let v0 = poly[0];
    let mut tris = Vec::with_capacity(poly.len() - 2);
    for i in 1..(poly.len() - 1) {
        tris.push([v0, poly[i], poly[i + 1]]);
    }
    tris
}

/// Fan-triangulate with reversed orientation (for flipped output).
pub fn fan_triangulate_reversed(poly: &[Point3]) -> Vec<[Point3; 3]> {
    if poly.len() < 3 { return Vec::new(); }
    let v0 = poly[0];
    let mut tris = Vec::with_capacity(poly.len() - 2);
    for i in 1..(poly.len() - 1) {
        tris.push([v0, poly[i + 1], poly[i]]);
    }
    tris
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn triangle_passes_through_unchanged() {
        let poly = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ];
        let tris = fan_triangulate(&poly);
        assert_eq!(tris.len(), 1);
        assert_eq!(tris[0], [poly[0], poly[1], poly[2]]);
    }

    #[test]
    fn quad_yields_two_triangles() {
        let poly = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ];
        let tris = fan_triangulate(&poly);
        assert_eq!(tris.len(), 2);
    }

    #[test]
    fn reversed_flips_winding() {
        let poly = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ];
        let fwd = fan_triangulate(&poly);
        let rev = fan_triangulate_reversed(&poly);
        assert_eq!(fwd[0][1], rev[0][2]);
        assert_eq!(fwd[0][2], rev[0][1]);
    }
}
