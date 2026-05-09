//! Mass properties of a triangulated solid: volume, surface area, centroid,
//! inertia tensor, and principal axes/moments.
//!
//! All formulas assume a **uniformly-dense solid with unit density (ρ = 1)**.
//! The solid must be a closed, consistently-wound manifold (kerf convention:
//! outward CCW normals); the results for open or inconsistently-wound meshes
//! are meaningless.
//!
//! ## Method
//! Every face is fan-triangulated from its first vertex. Each triangle
//! (p0, p1, p2) together with the origin O forms a signed tetrahedron.
//! Summing signed tetrahedra contributions gives:
//!
//! * **Volume** — via the divergence theorem (same as `solid_volume`).
//! * **Surface area** — ½ |edge1 × edge2| per triangle.
//! * **Centroid** — ∑ tet_vol_signed * (p0+p1+p2)/4  /  total_volume.
//! * **Inertia tensor** — standard tetrahedron-integral formula (Tonon 2004).
//! * **Principal moments + axes** — Jacobi eigendecomposition of the
//!   covariance-shifted inertia tensor at the centroid.



use crate::Solid;

/// Complete mass properties of a solid, assuming unit density.
#[derive(Clone, Debug, PartialEq)]
pub struct MassProperties {
    /// Volume of the solid (mm³ if coordinates are in mm).
    pub volume: f64,
    /// Total surface area (mm² if coordinates are in mm).
    pub surface_area: f64,
    /// Volume-weighted centroid (centre of mass) [x, y, z].
    pub centroid: [f64; 3],
    /// 3×3 inertia tensor at the centroid (row-major).
    pub inertia_tensor: [[f64; 3]; 3],
    /// Principal moments of inertia (I₁ ≤ I₂ ≤ I₃).
    pub principal_moments: [f64; 3],
    /// Corresponding unit principal axes (row i is the i-th eigenvector).
    pub principal_axes: [[f64; 3]; 3],
    /// Axis-aligned bounding box minimum [x, y, z].
    pub aabb_min: [f64; 3],
    /// Axis-aligned bounding box maximum [x, y, z].
    pub aabb_max: [f64; 3],
}

/// Compute mass properties for a solid, assuming uniform unit density.
///
/// For an empty solid (no faces) returns zeroed-out `MassProperties` with
/// principal axes set to the identity matrix.
pub fn mass_properties(solid: &Solid) -> MassProperties {
    // Accumulate per-triangle contributions.
    // For each face we fan-triangulate from the first vertex.
    let mut vol = 0.0_f64;
    let mut area = 0.0_f64;

    // First moment: cx_acc = ∑ tet_signed_vol * (p0+p1+p2+O)/4
    // Since O = origin, the "/4" denominator's 4th term vanishes → (p0+p1+p2)/4.
    let mut cx_acc = 0.0_f64;
    let mut cy_acc = 0.0_f64;
    let mut cz_acc = 0.0_f64;

    // Inertia tensor accumulator (6 independent entries of symmetric 3×3).
    // Using Tonon (2004) formula for a tetrahedron with one vertex at origin.
    let mut ixx = 0.0_f64;
    let mut iyy = 0.0_f64;
    let mut izz = 0.0_f64;
    let mut ixy = 0.0_f64;
    let mut ixz = 0.0_f64;
    let mut iyz = 0.0_f64;

    // Bounding box.
    let mut bb_min = [f64::MAX; 3];
    let mut bb_max = [f64::MIN; 3];

    for face_id in solid.topo.face_ids() {
        let Some(face) = solid.topo.face(face_id) else { continue };
        let Some(lp) = solid.topo.loop_(face.outer_loop()) else { continue };
        let Some(start) = lp.half_edge() else { continue };

        // Collect the polygon vertices by walking the half-edge loop.
        let mut polygon: Vec<[f64; 3]> = Vec::new();
        let mut cur = start;
        loop {
            let Some(he) = solid.topo.half_edge(cur) else { break };
            let v = he.origin();
            let Some(p) = solid.vertex_geom.get(v) else { break };
            polygon.push([p.x, p.y, p.z]);
            // Update AABB.
            bb_min[0] = bb_min[0].min(p.x);
            bb_min[1] = bb_min[1].min(p.y);
            bb_min[2] = bb_min[2].min(p.z);
            bb_max[0] = bb_max[0].max(p.x);
            bb_max[1] = bb_max[1].max(p.y);
            bb_max[2] = bb_max[2].max(p.z);
            cur = he.next();
            if cur == start { break; }
        }
        if polygon.len() < 3 { continue; }

        // Fan-triangulate from polygon[0].
        let p0 = polygon[0];
        for i in 1..polygon.len() - 1 {
            let p1 = polygon[i];
            let p2 = polygon[i + 1];

            // ---- Surface area ----
            let e1 = [p1[0]-p0[0], p1[1]-p0[1], p1[2]-p0[2]];
            let e2 = [p2[0]-p0[0], p2[1]-p0[1], p2[2]-p0[2]];
            let cross = [
                e1[1]*e2[2] - e1[2]*e2[1],
                e1[2]*e2[0] - e1[0]*e2[2],
                e1[0]*e2[1] - e1[1]*e2[0],
            ];
            let cross_len = (cross[0]*cross[0] + cross[1]*cross[1] + cross[2]*cross[2]).sqrt();
            area += 0.5 * cross_len;

            // ---- Signed volume of tetrahedron (origin, p0, p1, p2) ----
            // v_tet = (1/6) * p0 · (p1 × p2)
            let tet = (p0[0]*cross[0] + p0[1]*cross[1] + p0[2]*cross[2]) / 6.0;
            vol += tet;

            // ---- Centroid accumulation ----
            // c contribution = tet_vol * (p0+p1+p2)/4
            // (The origin O is the 4th tet vertex; divided by 4 vertices.)
            let w = tet;
            cx_acc += w * (p0[0] + p1[0] + p2[0]);
            cy_acc += w * (p0[1] + p1[1] + p2[1]);
            cz_acc += w * (p0[2] + p1[2] + p2[2]);

            // ---- Inertia tensor accumulation (Tonon 2004) ----
            // For tetrahedron with vertices at origin, a=(x1,y1,z1), b, c:
            // The density-weighted second moments contribute.
            //
            // a=p0, b=p1, c=p2 for the tet (O,a,b,c).
            // det = 6 * tet_vol (signed)
            let det = 6.0 * tet; // = p0 · (p1 × p2)

            let (x1,y1,z1) = (p0[0],p0[1],p0[2]);
            let (x2,y2,z2) = (p1[0],p1[1],p1[2]);
            let (x3,y3,z3) = (p2[0],p2[1],p2[2]);

            // Tonon (2004) formula — second moment integrals over tet.
            // Each integral is expressed as det * (sum of symmetric combos) / 60
            // for diagonal terms, and det * (sum of cross combos) / 120 for off-diag.
            let f_x2 = x1*x1 + x1*x2 + x2*x2 + x1*x3 + x2*x3 + x3*x3;
            let f_y2 = y1*y1 + y1*y2 + y2*y2 + y1*y3 + y2*y3 + y3*y3;
            let f_z2 = z1*z1 + z1*z2 + z2*z2 + z1*z3 + z2*z3 + z3*z3;

            let f_xy = 2.0*x1*y1 + x2*y1 + x3*y1 + x1*y2 + 2.0*x2*y2 + x3*y2
                     + x1*y3 + x2*y3 + 2.0*x3*y3;
            let f_xz = 2.0*x1*z1 + x2*z1 + x3*z1 + x1*z2 + 2.0*x2*z2 + x3*z2
                     + x1*z3 + x2*z3 + 2.0*x3*z3;
            let f_yz = 2.0*y1*z1 + y2*z1 + y3*z1 + y1*z2 + 2.0*y2*z2 + y3*z2
                     + y1*z3 + y2*z3 + 2.0*y3*z3;

            ixx += det * f_x2 / 60.0;
            iyy += det * f_y2 / 60.0;
            izz += det * f_z2 / 60.0;
            ixy += det * f_xy / 120.0;
            ixz += det * f_xz / 120.0;
            iyz += det * f_yz / 120.0;
        }
    }

    // Handle empty solid.
    if vol.abs() < 1e-30 {
        let identity = [[1.0,0.0,0.0],[0.0,1.0,0.0],[0.0,0.0,1.0]];
        let aabb_min = if bb_min[0] == f64::MAX { [0.0;3] } else { bb_min };
        let aabb_max = if bb_max[0] == f64::MIN { [0.0;3] } else { bb_max };
        return MassProperties {
            volume: 0.0,
            surface_area: area,
            centroid: [0.0; 3],
            inertia_tensor: [[0.0;3];3],
            principal_moments: [0.0; 3],
            principal_axes: identity,
            aabb_min,
            aabb_max,
        };
    }

    // ---- Centroid ----
    let inv4v = 1.0 / (4.0 * vol);
    let cx = cx_acc * inv4v;
    let cy = cy_acc * inv4v;
    let cz = cz_acc * inv4v;

    // ---- Inertia tensor at origin → shift to centroid (parallel-axis theorem) ----
    // I_com = I_origin − m * (|c|² δᵢⱼ − cᵢcⱼ)   (where m = vol for unit density)
    // Note: Ixx from Tonon is the second moment ∫x² dm, not the inertia component.
    // Inertia Ixx_inertia = ∫(y²+z²)dm = ∫y²dm + ∫z²dm = Iyy_2nd + Izz_2nd
    // We have ixx, iyy, izz as second moments of x, y, z respectively.
    let ixx_inertia = iyy + izz;
    let iyy_inertia = ixx + izz;
    let izz_inertia = ixx + iyy;
    let ixy_inertia = -ixy;
    let ixz_inertia = -ixz;
    let iyz_inertia = -iyz;

    // Parallel-axis shift: I_com = I_origin − m*(d²δ − dᵢdⱼ)
    let m = vol; // unit density
    let d2 = cx*cx + cy*cy + cz*cz;

    let ixx_com = ixx_inertia - m * (d2 - cx*cx);
    let iyy_com = iyy_inertia - m * (d2 - cy*cy);
    let izz_com = izz_inertia - m * (d2 - cz*cz);
    let ixy_com = ixy_inertia - m * (-cx*cy);
    let ixz_com = ixz_inertia - m * (-cx*cz);
    let iyz_com = iyz_inertia - m * (-cy*cz);

    let tensor = [
        [ixx_com, ixy_com, ixz_com],
        [ixy_com, iyy_com, iyz_com],
        [ixz_com, iyz_com, izz_com],
    ];

    // ---- Eigendecomposition via Jacobi iteration ----
    let (principal_moments, principal_axes) = jacobi_eigen_3x3(tensor);

    let aabb_min = if bb_min[0] == f64::MAX { [0.0;3] } else { bb_min };
    let aabb_max = if bb_max[0] == f64::MIN { [0.0;3] } else { bb_max };

    MassProperties {
        volume: vol,
        surface_area: area,
        centroid: [cx, cy, cz],
        inertia_tensor: tensor,
        principal_moments,
        principal_axes,
        aabb_min,
        aabb_max,
    }
}

/// Jacobi iterative eigendecomposition for a 3×3 real symmetric matrix.
///
/// Returns `(eigenvalues, eigenvectors)` where `eigenvalues[i]` is sorted
/// ascending and `eigenvectors[i]` is the corresponding unit eigenvector
/// (stored as row `i`).  Converges in ≤ 50 sweeps for any non-degenerate
/// 3×3 symmetric matrix.
fn jacobi_eigen_3x3(m: [[f64; 3]; 3]) -> ([f64; 3], [[f64; 3]; 3]) {
    // Working copies — `a` will be diagonalised in place.
    let mut a = m;
    // `v` accumulates the rotation: starts as identity, columns → eigenvectors.
    let mut v = [[1.0_f64, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];

    const MAX_SWEEPS: usize = 50;
    const EPS: f64 = 1e-15;

    for _ in 0..MAX_SWEEPS {
        // Find the off-diagonal element with the largest absolute value.
        let mut p = 0usize;
        let mut q = 1usize;
        let mut max_off = a[0][1].abs();
        let candidates = [(0,1),(0,2),(1,2)];
        for &(i,j) in &candidates {
            let val = a[i][j].abs();
            if val > max_off { max_off = val; p = i; q = j; }
        }
        if max_off < EPS { break; }

        // Compute the Jacobi rotation angle.
        let theta = 0.5 * (a[q][q] - a[p][p]) / a[p][q];
        let t = if theta >= 0.0 {
            1.0 / (theta + (1.0 + theta*theta).sqrt())
        } else {
            1.0 / (theta - (1.0 + theta*theta).sqrt())
        };
        let c = 1.0 / (1.0 + t*t).sqrt();
        let s = t * c;
        let tau = s / (1.0 + c);

        // Apply Jacobi rotation to `a`.
        let a_pp = a[p][p];
        let a_qq = a[q][q];
        let a_pq = a[p][q];
        a[p][p] = a_pp - t * a_pq;
        a[q][q] = a_qq + t * a_pq;
        a[p][q] = 0.0;
        a[q][p] = 0.0;
        for r in 0..3 {
            if r == p || r == q { continue; }
            let a_rp = a[r][p];
            let a_rq = a[r][q];
            a[r][p] = a_rp - s * (a_rq + tau * a_rp);
            a[p][r] = a[r][p];
            a[r][q] = a_rq + s * (a_rp - tau * a_rq);
            a[q][r] = a[r][q];
        }

        // Accumulate rotation in `v`.
        for r in 0..3 {
            let v_rp = v[r][p];
            let v_rq = v[r][q];
            v[r][p] = v_rp - s * (v_rq + tau * v_rp);
            v[r][q] = v_rq + s * (v_rp - tau * v_rq);
        }
    }

    // Extract diagonal (eigenvalues) from `a`.
    let eigs = [a[0][0], a[1][1], a[2][2]];

    // Sort eigenvalues ascending; sort corresponding eigenvector columns of `v`.
    // Eigenvector for eigenvalue i is column i of v → row i of transposed.
    // We sort by eigenvalue and permute columns of v accordingly.
    let mut indices = [0usize, 1, 2];
    indices.sort_by(|&i, &j| eigs[i].partial_cmp(&eigs[j]).unwrap_or(std::cmp::Ordering::Equal));

    let sorted_eigs = [eigs[indices[0]], eigs[indices[1]], eigs[indices[2]]];
    // Rows of result = columns of v reordered.
    let axes = [
        [v[0][indices[0]], v[1][indices[0]], v[2][indices[0]]],
        [v[0][indices[1]], v[1][indices[1]], v[2][indices[1]]],
        [v[0][indices[2]], v[1][indices[2]], v[2][indices[2]]],
    ];

    (sorted_eigs, axes)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::primitives::{box_, box_at, sphere_faceted};
    use kerf_geom::{Vec3};
    use std::f64::consts::PI;

    const TOL: f64 = 1e-6;

    // Helper: build a box with given dimensions, origin at (0,0,0).
    fn rect_box(w: f64, d: f64, h: f64) -> Solid {
        crate::primitives::box_(Vec3::new(w, d, h))
    }

    #[test]
    fn unit_cube_volume_area_centroid() {
        let s = rect_box(1.0, 1.0, 1.0);
        let mp = mass_properties(&s);

        // Volume = 1, surface area = 6.
        assert!((mp.volume - 1.0).abs() < TOL, "volume={}", mp.volume);
        assert!((mp.surface_area - 6.0).abs() < TOL, "area={}", mp.surface_area);

        // Centroid at (0.5, 0.5, 0.5) — kerf box_ places corner at origin.
        assert!((mp.centroid[0] - 0.5).abs() < TOL, "cx={}", mp.centroid[0]);
        assert!((mp.centroid[1] - 0.5).abs() < TOL, "cy={}", mp.centroid[1]);
        assert!((mp.centroid[2] - 0.5).abs() < TOL, "cz={}", mp.centroid[2]);

        // For a unit cube I_principal = 1/6 for all three (cube is isotropic).
        let [i1, i2, i3] = mp.principal_moments;
        let expected = 1.0 / 6.0;
        assert!((i1 - expected).abs() < TOL, "I1={i1}");
        assert!((i2 - expected).abs() < TOL, "I2={i2}");
        assert!((i3 - expected).abs() < TOL, "I3={i3}");
    }

    #[test]
    fn rectangular_prism_2x4x6() {
        // Box with extents 2×4×6; kerf box_ places corner at origin.
        let s = rect_box(2.0, 4.0, 6.0);
        let mp = mass_properties(&s);

        // Volume = 2*4*6 = 48.
        assert!((mp.volume - 48.0).abs() < TOL, "volume={}", mp.volume);

        // Surface area = 2*(2*4 + 2*6 + 4*6) = 2*(8+12+24) = 88.
        assert!((mp.surface_area - 88.0).abs() < TOL, "area={}", mp.surface_area);

        // Centroid at (1, 2, 3).
        assert!((mp.centroid[0] - 1.0).abs() < TOL, "cx={}", mp.centroid[0]);
        assert!((mp.centroid[1] - 2.0).abs() < TOL, "cy={}", mp.centroid[1]);
        assert!((mp.centroid[2] - 3.0).abs() < TOL, "cz={}", mp.centroid[2]);

        // Principal moments for a solid rectangular cuboid a×b×c, mass m:
        //   Ix = m(b²+c²)/12, Iy = m(a²+c²)/12, Iz = m(a²+b²)/12
        // with a=2,b=4,c=6, m=48:
        //   Ix = 48*(16+36)/12 = 48*52/12 = 208
        //   Iy = 48*(4+36)/12  = 48*40/12 = 160
        //   Iz = 48*(4+16)/12  = 48*20/12 = 80
        let [i1, i2, i3] = mp.principal_moments; // sorted ascending
        assert!((i1 - 80.0).abs() < 1.0, "I1={i1} expected≈80");
        assert!((i2 - 160.0).abs() < 1.0, "I2={i2} expected≈160");
        assert!((i3 - 208.0).abs() < 1.0, "I3={i3} expected≈208");

        // Bounding box.
        assert!((mp.aabb_min[0] - 0.0).abs() < TOL);
        assert!((mp.aabb_max[0] - 2.0).abs() < TOL);
        assert!((mp.aabb_max[1] - 4.0).abs() < TOL);
        assert!((mp.aabb_max[2] - 6.0).abs() < TOL);
    }

    #[test]
    fn faceted_sphere_volume_and_isotropic_inertia() {
        // Faceted sphere at origin, radius 5, high enough resolution.
        let s = sphere_faceted(5.0, 32, 64);
        let mp = mass_properties(&s);

        // Volume ≈ 4/3 π r³ within 5%.
        let expected_vol = (4.0 / 3.0) * PI * 5.0_f64.powi(3);
        let vol_err = (mp.volume - expected_vol).abs() / expected_vol;
        assert!(vol_err < 0.05, "sphere volume err={vol_err:.4}");

        // For a uniform sphere the three principal moments should be equal
        // (to within tesselation error): I = 2/5 m r².
        let [i1, i2, i3] = mp.principal_moments;
        let expected_i = 0.4 * mp.volume * 25.0; // 2/5 * m * r²
        let rel_err_1 = (i1 - expected_i).abs() / expected_i;
        let rel_err_3 = (i3 - expected_i).abs() / expected_i;
        assert!(rel_err_1 < 0.05, "I1={i1} expected≈{expected_i}");
        assert!(rel_err_3 < 0.05, "I3={i3} expected≈{expected_i}");
        // All three should be close to each other (isotropy).
        let spread = (i3 - i1).abs() / expected_i;
        assert!(spread < 0.01, "I spread={spread:.5} for sphere");
    }

    #[test]
    fn mass_properties_deterministic() {
        let s = rect_box(3.0, 5.0, 7.0);
        let mp1 = mass_properties(&s);
        let mp2 = mass_properties(&s);
        assert_eq!(mp1.volume, mp2.volume);
        assert_eq!(mp1.centroid, mp2.centroid);
        assert_eq!(mp1.principal_moments, mp2.principal_moments);
    }

    #[test]
    fn empty_solid_returns_zero() {
        let s = Solid::new();
        let mp = mass_properties(&s);
        assert_eq!(mp.volume, 0.0);
        assert_eq!(mp.surface_area, 0.0);
        assert_eq!(mp.centroid, [0.0; 3]);
    }
}
