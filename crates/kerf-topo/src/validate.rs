//! Topology validator. Walks the half-edge graph and checks invariants.

use thiserror::Error;

use crate::id::HalfEdgeId;
use crate::solid::Solid;

#[derive(Debug, Error, PartialEq)]
pub enum ValidationError {
    #[error("half-edge {0:?} twin mismatch: twin(twin) != self")]
    AsymmetricTwin(HalfEdgeId),
    #[error("half-edge {0:?} next/prev mismatch")]
    NextPrevMismatch(HalfEdgeId),
    #[error("loop walk did not close within {0} steps")]
    LoopWalkOpen(usize),
    #[error("euler invariant violated: V={v} E={e} F={f} R={r} S={s}; expected {expected}, got {got}")]
    EulerInvariant { v: i64, e: i64, f: i64, r: i64, s: i64, expected: i64, got: i64 },
}

pub fn validate(solid: &Solid) -> Result<(), ValidationError> {
    // 1. Twin symmetry.
    for (id, he) in &solid.half_edges {
        let twin = solid
            .half_edges
            .get(he.twin)
            .ok_or(ValidationError::AsymmetricTwin(id))?;
        if twin.twin != id {
            return Err(ValidationError::AsymmetricTwin(id));
        }
    }
    // 2. Next/prev consistency.
    for (id, he) in &solid.half_edges {
        let next_he = solid
            .half_edges
            .get(he.next)
            .ok_or(ValidationError::NextPrevMismatch(id))?;
        if next_he.prev != id {
            return Err(ValidationError::NextPrevMismatch(id));
        }
    }
    // 3. Loop closure.
    for (_lid, lp) in &solid.loops {
        let Some(start) = lp.half_edge else { continue; };
        let max_steps = solid.half_edges.len() + 1;
        let mut cur = start;
        let mut steps = 0;
        loop {
            steps += 1;
            if steps > max_steps {
                return Err(ValidationError::LoopWalkOpen(steps));
            }
            cur = solid.half_edges[cur].next;
            if cur == start {
                break;
            }
        }
    }
    // 4. Euler-Poincaré (with rings, genus assumed 0).
    let v = solid.vertex_count() as i64;
    let e = solid.edge_count() as i64;
    let f = solid.face_count() as i64;
    let s = solid.shell_count() as i64;
    let r: i64 = solid
        .faces
        .iter()
        .map(|(_, f)| f.inner_loops.len() as i64)
        .sum();
    let lhs = v - e + f - r;
    let rhs = 2 * s;
    if lhs != rhs {
        return Err(ValidationError::EulerInvariant {
            v, e, f, r, s, expected: rhs, got: lhs,
        });
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn empty_solid_passes_validation() {
        let s = Solid::new();
        validate(&s).unwrap();
    }

    #[test]
    fn mvfs_solid_passes_validation() {
        let mut s = Solid::new();
        s.mvfs();
        validate(&s).unwrap();
    }

    #[test]
    fn mvfs_then_mev_passes_validation() {
        let mut s = Solid::new();
        let r = s.mvfs();
        let _m = s.mev_at_lone_vertex(r.loop_, r.vertex);
        validate(&s).unwrap();
    }

    #[test]
    fn mvfs_mev_mev_mef_passes_validation() {
        let mut s = Solid::new();
        let r = s.mvfs();
        let m1 = s.mev_at_lone_vertex(r.loop_, r.vertex);
        let m2 = s.mev(r.loop_, m1.half_edges.1);
        let h_a = m1.half_edges.0;
        let h_b = m2.half_edges.1;
        let _mef = s.mef(h_a, h_b);
        validate(&s).unwrap();
    }
}
