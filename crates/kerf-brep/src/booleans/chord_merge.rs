//! M38 attempt 2 — post-classify chord-merge with unit tests on the canonical
//! `union(box, box-corner)` geometry before integrating.
//!
//! Strategy: for each pair of coplanar adjacent kept-and-dropped polygons,
//! merge them by removing the shared chord (one or more consecutive edges)
//! and walking around the dropped polygon's outer perimeter instead.
//!
//! The previous attempt over-merged because it didn't ensure transitive
//! consistency across all sibling pairs. This pass iterates until fixpoint,
//! but only merges when the chord is unambiguously a kept-vs-dropped pair
//! sibling.

use kerf_geom::{Point3, Tolerance};

use crate::booleans::FaceClassification;
use crate::booleans::stitch::KeptFace;
use crate::geometry::SurfaceKind;

/// Two `Point3`s equal within tolerance (with a relaxation factor — kerf's
/// default 1e-9 is tighter than typical CAD precision).
fn pt_eq(a: Point3, b: Point3, tol: &Tolerance) -> bool {
    (a - b).norm() <= tol.point_eq * 1000.0
}

/// Are two surfaces the same plane (within tolerance)? Same direction OR
/// opposite-direction normals both qualify — both indicate coplanarity.
fn coplanar(s1: &SurfaceKind, s2: &SurfaceKind, tol: &Tolerance) -> bool {
    match (s1, s2) {
        (SurfaceKind::Plane(p1), SurfaceKind::Plane(p2)) => {
            let n1 = p1.frame.z;
            let n2 = p2.frame.z;
            let parallel = (n1.dot(&n2).abs() - 1.0).abs() < 1e-6;
            if !parallel {
                return false;
            }
            let d = (p2.frame.origin - p1.frame.origin).dot(&n1);
            d.abs() < tol.point_eq * 1000.0
        }
        _ => false,
    }
}

/// Find an index `i` in `poly` such that `poly[i]==a` and `poly[(i+1)%n]==b`.
fn find_directed_edge(poly: &[Point3], a: Point3, b: Point3, tol: &Tolerance) -> Option<usize> {
    let n = poly.len();
    for i in 0..n {
        if pt_eq(poly[i], a, tol) && pt_eq(poly[(i + 1) % n], b, tol) {
            return Some(i);
        }
    }
    None
}

/// Merge two adjacent coplanar polygons sharing a chord (one or more
/// consecutive edges going opposite directions in the two polygons).
///
/// `kept` traverses A→…→B in CCW order along the chord. `dropped` traverses
/// the chord in reverse order (B→…→A in CCW). The merged polygon walks the
/// outer perimeter: kept's vertices up to A, then dropped's vertices from
/// past A around back to B (skipping the chord), then kept's vertices from
/// past B onward.
///
/// Returns `None` if no shared chord exists.
pub fn merge_along_shared_chord(
    kept: &[Point3],
    dropped: &[Point3],
    tol: &Tolerance,
) -> Option<Vec<Point3>> {
    let n_k = kept.len();
    let n_d = dropped.len();
    if n_k < 3 || n_d < 3 {
        return None;
    }

    // Find ANY edge in `kept` whose reverse appears in `dropped`. That's
    // the chord's first edge; we'll extend forward in `kept` (and backward
    // in `dropped`) to find its full extent.
    let mut chord_start_k: Option<usize> = None;
    let mut chord_end_k: usize = 0;
    let mut chord_start_d: usize = 0;
    let mut chord_end_d: usize = 0;
    for ki in 0..n_k {
        let a = kept[ki];
        let b = kept[(ki + 1) % n_k];
        if let Some(dj) = find_directed_edge(dropped, b, a, tol) {
            chord_start_k = Some(ki);
            chord_end_k = (ki + 1) % n_k;
            chord_start_d = dj;
            chord_end_d = (dj + 1) % n_d;
            break;
        }
    }
    let mut chord_start_k = chord_start_k?;

    // Extend the chord backward (toward kept's start). While the previous
    // edge in `kept` is also part of the chord — i.e., its reverse appears
    // in `dropped` immediately AFTER the current chord_start_d.
    loop {
        let prev_k = (chord_start_k + n_k - 1) % n_k;
        let next_d = (chord_end_d) % n_d;
        // The previous kept edge is kept[prev_k] → kept[chord_start_k].
        // Its reverse would be kept[chord_start_k] → kept[prev_k].
        // We want this reverse to be the dropped edge ending at chord_end_d
        // (i.e., dropped[next_d] == kept[prev_k]).
        if pt_eq(dropped[next_d], kept[prev_k], tol) && prev_k != chord_end_k {
            chord_start_k = prev_k;
            chord_end_d = (chord_end_d + 1) % n_d;
        } else {
            break;
        }
    }

    // Extend the chord forward (toward kept's end). While the next edge in
    // `kept` is part of the chord — its reverse appears immediately BEFORE
    // chord_start_d in `dropped`.
    loop {
        let next_k = (chord_end_k + 1) % n_k;
        let prev_d = (chord_start_d + n_d - 1) % n_d;
        if pt_eq(kept[next_k], dropped[prev_d], tol) && next_k != chord_start_k {
            chord_end_k = next_k;
            chord_start_d = prev_d;
        } else {
            break;
        }
    }

    // Build merged polygon:
    //   kept[i..chord_start_k] (kept[chord_start_k] is "A", chord's first vertex)
    //   then walk dropped from past chord_end_d around to chord_start_d (which is "B")
    //   then kept from past chord_end_k onward back to start
    let mut merged: Vec<Point3> = Vec::with_capacity(n_k + n_d);

    // Walk kept from index 0 up to and including chord_start_k.
    let mut idx = 0usize;
    while idx != chord_start_k {
        merged.push(kept[idx]);
        idx = (idx + 1) % n_k;
    }
    merged.push(kept[chord_start_k]); // = "A"

    // Walk dropped from past chord_end_d to chord_start_d, exclusive of
    // chord_end_d but INCLUSIVE of chord_start_d (which is "B").
    let mut d_idx = (chord_end_d + 1) % n_d;
    while d_idx != chord_start_d {
        merged.push(dropped[d_idx]);
        d_idx = (d_idx + 1) % n_d;
    }
    merged.push(dropped[chord_start_d]); // = "B"

    // Walk kept from past chord_end_k back to 0.
    let mut k_idx = (chord_end_k + 1) % n_k;
    while k_idx != 0 {
        merged.push(kept[k_idx]);
        k_idx = (k_idx + 1) % n_k;
    }

    if merged.len() < 3 {
        return None;
    }
    Some(merged)
}

/// Does any kept face NOT coplanar with `surface` contain the directed edge
/// `a → b` OR `b → a` (i.e., the actual edge, not just both vertices)?
/// Used to gate the merge: if the chord IS an edge of a non-coplanar kept
/// face, merging the chord away would erase a real boundary.
fn chord_used_by_other_kept_face(
    a: Point3,
    b: Point3,
    surface: &SurfaceKind,
    kept: &[KeptFace],
    tol: &Tolerance,
) -> bool {
    for kf in kept {
        if coplanar(&kf.surface, surface, tol) {
            continue;
        }
        if find_directed_edge(&kf.polygon, a, b, tol).is_some()
            || find_directed_edge(&kf.polygon, b, a, tol).is_some()
        {
            return true;
        }
    }
    false
}

/// Same as [`chord_merge_pass`] but only merges kept-with-dropped pairs from
/// the SAME source solid (both from A or both from B). Cross-solid pairs are
/// skipped — they're typically OnBoundary coincident faces whose merge
/// produces overlapping kept polygons.
pub fn chord_merge_pass_same_source(
    kept: &mut Vec<KeptFace>,
    kept_source: &[u8],
    dropped: &[KeptFace],
    dropped_source: &[u8],
    dropped_cls: &[FaceClassification],
    tol: &Tolerance,
) -> usize {
    let mut available: Vec<bool> = vec![true; dropped.len()];
    let mut merge_count = 0usize;
    loop {
        let mut changed = false;
        for ki in 0..kept.len() {
            for di in 0..dropped.len() {
                if !available[di] {
                    continue;
                }
                if kept_source[ki] != dropped_source[di] {
                    continue;
                }
                // Only merge "Inside" dropped siblings — those are dropped
                // because their region is interior to the result. OnBoundary
                // dropped faces are coincident-with-other-solid duplicates;
                // their region is ALREADY covered by another kept face from
                // the other solid. Merging absorbs them spuriously.
                if !matches!(dropped_cls[di], FaceClassification::Inside) {
                    continue;
                }
                if !coplanar(&kept[ki].surface, &dropped[di].surface, tol) {
                    continue;
                }
                let kp = &kept[ki].polygon;
                let dp = &dropped[di].polygon;
                let n_k = kp.len();
                let mut chord: Option<(Point3, Point3)> = None;
                for j in 0..n_k {
                    let a = kp[j];
                    let b = kp[(j + 1) % n_k];
                    if find_directed_edge(dp, b, a, tol).is_some() {
                        chord = Some((a, b));
                        break;
                    }
                }
                let Some((chord_a, chord_b)) = chord else {
                    continue;
                };
                if chord_used_by_other_kept_face(chord_a, chord_b, &kept[ki].surface, kept, tol) {
                    continue;
                }
                if let Some(merged) = merge_along_shared_chord(kp, dp, tol) {
                    kept[ki].polygon = merged;
                    available[di] = false;
                    merge_count += 1;
                    changed = true;
                    break;
                }
            }
        }
        if !changed {
            break;
        }
    }
    merge_count
}

/// Iteratively merge every coplanar (kept, dropped) pair sharing a chord.
/// Skips merges where the chord's endpoints are also present in a
/// non-coplanar kept face — that means the chord is a real boundary edge
/// (e.g., a corner-cut difference's cavity wall) and merging would erase
/// real geometry. Once a dropped polygon is consumed, it's removed from
/// the pool. Iterates until fixpoint.
pub fn chord_merge_pass(kept: &mut Vec<KeptFace>, dropped: &[KeptFace], tol: &Tolerance) {
    let mut available: Vec<bool> = vec![true; dropped.len()];
    loop {
        let mut changed = false;
        for ki in 0..kept.len() {
            for di in 0..dropped.len() {
                if !available[di] {
                    continue;
                }
                if !coplanar(&kept[ki].surface, &dropped[di].surface, tol) {
                    continue;
                }
                // Find a candidate chord between kept[ki] and dropped[di].
                let kp = &kept[ki].polygon;
                let dp = &dropped[di].polygon;
                let n_k = kp.len();
                let mut chord: Option<(Point3, Point3)> = None;
                for j in 0..n_k {
                    let a = kp[j];
                    let b = kp[(j + 1) % n_k];
                    if find_directed_edge(dp, b, a, tol).is_some() {
                        chord = Some((a, b));
                        break;
                    }
                }
                let Some((chord_a, chord_b)) = chord else {
                    continue;
                };
                // If a non-coplanar kept face also has both chord endpoints,
                // the chord is a real boundary edge; don't merge.
                if chord_used_by_other_kept_face(chord_a, chord_b, &kept[ki].surface, kept, tol)
                {
                    continue;
                }
                if let Some(merged) = merge_along_shared_chord(kp, dp, tol) {
                    kept[ki].polygon = merged;
                    available[di] = false;
                    changed = true;
                    break;
                }
            }
        }
        if !changed {
            break;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// L-shape (kept) plus inner square (dropped) merge into the original
    /// 2×2 square (with two collinear chord-introduced vertices).
    #[test]
    fn merge_l_with_inner_square_recovers_original() {
        let l = vec![
            Point3::new(2.0, 0.0, 0.0),
            Point3::new(2.0, 2.0, 0.0),
            Point3::new(2.0, 2.0, 1.0), // chord vertex on y=2 boundary
            Point3::new(2.0, 1.0, 1.0), // interior chord corner
            Point3::new(2.0, 1.0, 2.0), // chord vertex on z=2 boundary
            Point3::new(2.0, 0.0, 2.0),
        ];
        let inner = vec![
            Point3::new(2.0, 2.0, 1.0), // shares with L's index 2 (reversed direction)
            Point3::new(2.0, 2.0, 2.0), // ← this is the missing corner
            Point3::new(2.0, 1.0, 2.0), // shares with L's index 4
            Point3::new(2.0, 1.0, 1.0), // shares with L's index 3
        ];
        let merged = merge_along_shared_chord(&l, &inner, &Tolerance::default())
            .expect("merge should succeed");
        // Expected merged perimeter (CCW from +x outward normal):
        //   (2,0,0), (2,2,0), (2,2,1), (2,2,2), (2,1,2), (2,0,2)
        assert_eq!(merged.len(), 6, "merged should have 6 vertices");
        assert!(pt_eq(merged[0], Point3::new(2.0, 0.0, 0.0), &Tolerance::default()));
        assert!(pt_eq(merged[1], Point3::new(2.0, 2.0, 0.0), &Tolerance::default()));
        assert!(pt_eq(merged[2], Point3::new(2.0, 2.0, 1.0), &Tolerance::default()));
        assert!(pt_eq(merged[3], Point3::new(2.0, 2.0, 2.0), &Tolerance::default()));
        assert!(pt_eq(merged[4], Point3::new(2.0, 1.0, 2.0), &Tolerance::default()));
        assert!(pt_eq(merged[5], Point3::new(2.0, 0.0, 2.0), &Tolerance::default()));
    }

    /// Two simple half-square polygons sharing one edge merge into the full square.
    #[test]
    fn merge_two_half_squares_recovers_full_square() {
        // Left half: (0,0), (1,0), (1,1), (0,1). Right half: (1,0), (2,0), (2,1), (1,1).
        // Shared edge: (1,0)→(1,1) in left, (1,1)→(1,0) in right.
        let left = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ];
        let right = vec![
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(2.0, 0.0, 0.0),
            Point3::new(2.0, 1.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
        ];
        let merged = merge_along_shared_chord(&left, &right, &Tolerance::default())
            .expect("merge should succeed");
        assert_eq!(merged.len(), 6, "merged has 6 vertices (2 collinear)");
    }
}
