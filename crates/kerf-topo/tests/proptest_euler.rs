//! Property test: random sequences of legal Euler ops preserve the validator.

use kerf_topo::{HalfEdgeId, Solid, validate};
use proptest::prelude::*;

proptest! {
    #[test]
    fn invariant_holds_under_mev_chain(num_ops in 0usize..15) {
        let mut s = Solid::new();
        let r = s.mvfs();
        let mut last_he: Option<HalfEdgeId> = None;
        let mut last_vertex = r.vertex;
        for _ in 0..num_ops {
            let m = match last_he {
                Some(h) => s.mev(r.loop_, h),
                None => s.mev_at_lone_vertex(r.loop_, last_vertex),
            };
            last_vertex = m.vertex;
            last_he = Some(m.half_edges.1);
        }
        validate(&s).unwrap();
    }
}
