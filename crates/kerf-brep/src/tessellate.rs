//! Tessellate a Solid into a FaceSoup of triangles. v1 supports the
//! cylinder primitive's topology (planar caps + cylindrical lateral).
//!
//! For planar cap faces bounded by a single circular edge: fan-triangulate
//! around the circle center using `lateral_segments` divisions.
//!
//! For cylindrical lateral faces: emit a quad strip (2 triangles per segment).
//!
//! For other planar faces (polygon prisms etc.): fan-triangulate from vertex 0.

use std::f64::consts::TAU;

use kerf_geom::{Curve as _, Surface as _};

use crate::booleans::FaceSoup;
use crate::geometry::{CurveKind, SurfaceKind};
use crate::Solid;
use kerf_topo::{EdgeId, FaceId};

/// Tessellate a Solid into triangles. `lateral_segments` is the polygon
/// resolution for cylindrical faces and circular cap faces (must be ≥ 3).
///
/// Triangle winding is outward-normal CCW as seen from outside the solid.
pub fn tessellate(solid: &Solid, lateral_segments: usize) -> FaceSoup {
    let (soup, _) = tessellate_with_face_index(solid, lateral_segments);
    soup
}

/// Same as [`tessellate`], plus a parallel `Vec<u32>` recording which face
/// index each triangle belongs to. Face indices are sequential starting at 0
/// in the order `solid.topo.face_ids()` yields face IDs. Used by the
/// browser viewer for face-level picking.
pub fn tessellate_with_face_index(solid: &Solid, lateral_segments: usize) -> (FaceSoup, Vec<u32>) {
    debug_assert!(lateral_segments >= 3);
    let mut soup = FaceSoup::default();
    let mut face_index: Vec<u32> = Vec::new();
    let mut next_face: u32 = 0;

    for face_id in solid.topo.face_ids() {
        let before = soup.triangles.len();
        tessellate_one_face_into(&mut soup, solid, face_id, lateral_segments);
        let added = soup.triangles.len() - before;
        for _ in 0..added {
            face_index.push(next_face);
        }
        next_face = next_face.saturating_add(1);
    }
    (soup, face_index)
}

fn tessellate_one_face_into(
    soup: &mut FaceSoup,
    solid: &Solid,
    face_id: kerf_topo::FaceId,
    lateral_segments: usize,
) {
        let surface = match solid.face_geom.get(face_id) {
            Some(s) => s,
            None => return,
        };
        match surface {
            SurfaceKind::Plane(plane) => {
                let edges = collect_face_edges(solid, face_id);
                if edges.len() == 1 {
                    // Single-edge face → circular cap.
                    if let Some(circle_edge_id) = edges.first() {
                        if let Some(seg) = solid.edge_geom.get(*circle_edge_id) {
                            if let CurveKind::Circle(c) = &seg.curve {
                                let center = c.frame.origin;
                                let dt = TAU / lateral_segments as f64;
                                for i in 0..lateral_segments {
                                    let t0 = i as f64 * dt;
                                    let t1 = (i + 1) as f64 * dt;
                                    let p0 = c.point_at(t0);
                                    let p1 = c.point_at(t1);
                                    // plane.frame.z is outward normal.
                                    // For top face (normal +z): CCW winding when looking down +z
                                    //   → [center, p0, p1] is CCW (p1 is CCW of p0).
                                    // For bottom face (normal -z): [center, p1, p0].
                                    if plane.frame.z.z > 0.0 {
                                        soup.triangles.push([center, p0, p1]);
                                    } else {
                                        soup.triangles.push([center, p1, p0]);
                                    }
                                }
                                return;
                            }
                        }
                    }
                }
                // Polygonal planar face: fan from vertex 0. Use the RAW loop
                // traversal (no winding normalization) so cavity faces in
                // stitched results emit CCW-from-cavity-outward triangles —
                // matches the convention the rest of the pipeline assumes.
                if let Some(poly) = crate::booleans::face_polygon_raw(solid, face_id) {
                    if poly.len() >= 3 {
                        for i in 1..(poly.len() - 1) {
                            soup.triangles.push([poly[0], poly[i], poly[i + 1]]);
                        }
                    }
                }
            }
            SurfaceKind::Cylinder(cyl) => {
                // Lateral strip: for each angular segment emit two triangles.
                // Height comes from the seam edge's range length.
                let edges = collect_face_edges(solid, face_id);
                let height = edges
                    .iter()
                    .filter_map(|eid| solid.edge_geom.get(*eid))
                    .find_map(|seg| match &seg.curve {
                        CurveKind::Line(_) => Some(seg.range.1 - seg.range.0),
                        _ => None,
                    })
                    .unwrap_or(1.0);
                let dt = TAU / lateral_segments as f64;
                for i in 0..lateral_segments {
                    let t0 = i as f64 * dt;
                    let t1 = (i + 1) as f64 * dt;
                    let p0_bot = cyl.point_at(t0, 0.0);
                    let p1_bot = cyl.point_at(t1, 0.0);
                    let p0_top = cyl.point_at(t0, height);
                    let p1_top = cyl.point_at(t1, height);
                    // Outward normal points away from the axis. Winding CCW from outside:
                    //   [p0_bot, p1_bot, p1_top] and [p0_bot, p1_top, p0_top].
                    soup.triangles.push([p0_bot, p1_bot, p1_top]);
                    soup.triangles.push([p0_bot, p1_top, p0_top]);
                }
            }
            SurfaceKind::Cone(_cone_surf) => {
                // Walk face loop. Count circle edges:
                //   1 circle → true cone (apex on solid). Fan from apex.
                //   2 circles → frustum (apex off solid). Quad-strip between circles.
                let edges = collect_face_edges(solid, face_id);
                let circles: Vec<&kerf_geom::Circle> = edges
                    .iter()
                    .filter_map(|eid| solid.edge_geom.get(*eid))
                    .filter_map(|seg| match &seg.curve {
                        CurveKind::Circle(c) => Some(c),
                        _ => None,
                    })
                    .collect();

                let dt = TAU / lateral_segments as f64;
                if circles.len() == 1 {
                    let c = circles[0];
                    let apex = find_apex_in_face(solid, face_id);
                    for i in 0..lateral_segments {
                        let t0 = i as f64 * dt;
                        let t1 = ((i + 1) % lateral_segments) as f64 * dt;
                        let p0 = c.point_at(t0);
                        let p1 = c.point_at(t1);
                        soup.triangles.push([apex, p0, p1]);
                    }
                } else if circles.len() >= 2 {
                    // Frustum: top and bottom circles. Sort by z so winding is correct.
                    let mut c_top = circles[0];
                    let mut c_bot = circles[1];
                    if c_top.frame.origin.z < c_bot.frame.origin.z {
                        std::mem::swap(&mut c_top, &mut c_bot);
                    }
                    for i in 0..lateral_segments {
                        let t0 = i as f64 * dt;
                        let t1 = ((i + 1) % lateral_segments) as f64 * dt;
                        let p0_bot = c_bot.point_at(t0);
                        let p1_bot = c_bot.point_at(t1);
                        let p0_top = c_top.point_at(t0);
                        let p1_top = c_top.point_at(t1);
                        soup.triangles.push([p0_bot, p1_bot, p1_top]);
                        soup.triangles.push([p0_bot, p1_top, p0_top]);
                    }
                }
            }
            SurfaceKind::Sphere(sph) => {
                use std::f64::consts::PI;
                let polar_segs = (lateral_segments / 2).max(2);
                let dlat = PI / polar_segs as f64;
                let dlon = TAU / lateral_segments as f64;
                for j in 0..polar_segs {
                    let v0 = j as f64 * dlat;
                    let v1 = (j + 1) as f64 * dlat;
                    let north = j == 0;
                    let south = j == polar_segs - 1;
                    for i in 0..lateral_segments {
                        let u0 = i as f64 * dlon;
                        let u1 = ((i + 1) % lateral_segments) as f64 * dlon;
                        let p00 = sph.point_at(u0, v0);
                        let p10 = sph.point_at(u1, v0);
                        let p11 = sph.point_at(u1, v1);
                        let p01 = sph.point_at(u0, v1);
                        // CCW from outward (the radial outward normal at each
                        // sphere point): u goes CCW around z-axis as seen from
                        // +z, and v increases going from north pole (v=0) to
                        // south pole (v=π). At the equator looking outward we
                        // see u to the LEFT and v DOWN, so CCW order is
                        // p00 → p01 → p11 → p10.
                        if north {
                            soup.triangles.push([p00, p01, p11]);
                        } else if south {
                            soup.triangles.push([p00, p01, p10]);
                        } else {
                            soup.triangles.push([p00, p01, p11]);
                            soup.triangles.push([p00, p11, p10]);
                        }
                    }
                }
            }
            SurfaceKind::Torus(tor) => {
                let major_segs = lateral_segments;
                let minor_segs = (lateral_segments / 2).max(2);
                let du = TAU / major_segs as f64;
                let dv = TAU / minor_segs as f64;
                for i in 0..major_segs {
                    let u0 = i as f64 * du;
                    let u1 = ((i + 1) % major_segs) as f64 * du;
                    for j in 0..minor_segs {
                        let v0 = j as f64 * dv;
                        let v1 = ((j + 1) % minor_segs) as f64 * dv;
                        let p00 = tor.point_at(u0, v0);
                        let p10 = tor.point_at(u1, v0);
                        let p11 = tor.point_at(u1, v1);
                        let p01 = tor.point_at(u0, v1);
                        soup.triangles.push([p00, p10, p11]);
                        soup.triangles.push([p00, p11, p01]);
                    }
                }
            }
        }
}

/// Walk a cone face's outer loop to find the apex vertex position.
/// The apex is the origin of the seam half-edge that points to a non-self-loop vertex
/// (i.e., the half-edge whose origin is the apex vertex, not on the base circle).
fn find_apex_in_face(solid: &Solid, face_id: FaceId) -> kerf_geom::Point3 {
    use kerf_geom::Point3;
    let Some(face) = solid.topo.face(face_id) else {
        return Point3::origin();
    };
    let Some(loop_) = solid.topo.loop_(face.outer_loop()) else {
        return Point3::origin();
    };
    let Some(start) = loop_.half_edge() else {
        return Point3::origin();
    };
    let mut cur = start;
    loop {
        let Some(he) = solid.topo.half_edge(cur) else {
            break;
        };
        // The apex half-edge has a Line curve on its edge.
        let eid = he.edge();
        if let Some(seg) = solid.edge_geom.get(eid) {
            if matches!(&seg.curve, CurveKind::Line(_)) {
                // origin of this half-edge: if the line goes apex→bot, origin is apex.
                // apex is the vertex that is NOT on the base circle (z == height, not 0).
                let origin_vid = he.origin();
                if let Some(pos) = solid.vertex_geom.get(origin_vid) {
                    // The apex is at z = height > 0; base is at z = 0.
                    // Return whichever endpoint is not on the base (z ≠ 0).
                    if pos.z.abs() > 1e-10 {
                        return *pos;
                    }
                    // Otherwise the twin endpoint is apex; get dest vertex via twin.
                    let twin_id = he.twin();
                    if let Some(twin) = solid.topo.half_edge(twin_id) {
                        let dest_vid = twin.origin();
                        if let Some(dest_pos) = solid.vertex_geom.get(dest_vid) {
                            return *dest_pos;
                        }
                    }
                }
            }
        }
        cur = he.next();
        if cur == start {
            break;
        }
    }
    Point3::origin()
}

/// Walk a face's outer loop and collect the distinct edge IDs (de-duplicated
/// in case of self-loop — a self-loop visits the same edge twice in its
/// next-chain, but we only want one entry).
fn collect_face_edges(solid: &Solid, face_id: FaceId) -> Vec<EdgeId> {
    let mut edges: Vec<EdgeId> = Vec::new();
    let Some(face) = solid.topo.face(face_id) else {
        return edges;
    };
    let Some(loop_) = solid.topo.loop_(face.outer_loop()) else {
        return edges;
    };
    let Some(start) = loop_.half_edge() else {
        return edges;
    };
    let mut cur = start;
    loop {
        let Some(he) = solid.topo.half_edge(cur) else {
            break;
        };
        let eid = he.edge();
        if !edges.contains(&eid) {
            edges.push(eid);
        }
        cur = he.next();
        if cur == start {
            break;
        }
    }
    edges
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::primitives::{cone, cylinder, sphere, torus};

    #[test]
    fn cone_tessellation_has_expected_triangle_count() {
        // 16 segments: bot fan = 16 tris, lateral fan = 16 tris. Total = 32.
        let s = cone(1.0, 2.0);
        let soup = tessellate(&s, 16);
        assert_eq!(soup.triangles.len(), 32);
    }

    #[test]
    fn cylinder_tessellation_has_expected_triangle_count() {
        // For 16 lateral segments: top fan = 16 tris, bottom fan = 16 tris,
        // lateral strip = 32 tris (2 per quad × 16). Total = 64.
        let s = cylinder(1.0, 2.0);
        let soup = tessellate(&s, 16);
        assert_eq!(soup.triangles.len(), 64);
    }

    #[test]
    fn cylinder_to_stl_writes_valid_file() {
        let s = cylinder(1.0, 2.0);
        let soup = tessellate(&s, 12);
        let mut buf = Vec::new();
        crate::write_binary(&soup, "cylinder", &mut buf).unwrap();
        // 12 segments → 12+12+24 = 48 triangles. 80+4+50*48 = 2484 bytes.
        assert_eq!(buf.len(), 80 + 4 + 50 * 48);
    }

    #[test]
    fn sphere_tessellation_has_expected_triangle_count() {
        // 16 lateral × 8 polar:
        //   north row: 16 triangles (1 per longitude)
        //   middle 6 rows: 16 * 6 * 2 = 192 triangles
        //   south row: 16 triangles
        //   total: 16 + 192 + 16 = 224
        let s = sphere(1.0);
        let soup = tessellate(&s, 16);
        assert_eq!(soup.triangles.len(), 224);
    }

    #[test]
    fn sphere_tessellation_produces_valid_stl() {
        let s = sphere(1.0);
        let soup = tessellate(&s, 12);
        let mut buf = Vec::new();
        crate::write_binary(&soup, "sphere", &mut buf).unwrap();
        // 12 segs, 6 polar: 12 + 12*4*2 + 12 = 12 + 96 + 12 = 120 tris
        assert_eq!(buf.len(), 80 + 4 + 50 * 120);
    }

    #[test]
    fn torus_tessellation_has_expected_triangle_count() {
        // 16 major × 8 minor: 16 * 8 * 2 = 256 triangles.
        let s = torus(3.0, 1.0);
        let soup = tessellate(&s, 16);
        assert_eq!(soup.triangles.len(), 256);
    }
}
