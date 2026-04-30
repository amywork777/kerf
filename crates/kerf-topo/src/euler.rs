//! Mäntylä Euler operators. Each operator preserves V - E + F - R = 2(S - G).

use slotmap::Key;

use crate::entity::{Face, Loop, Shell, Vertex};
use crate::id::{FaceId, LoopId, ShellId, SolidId, VertexId};
use crate::solid::Solid;

#[derive(Clone, Copy, Debug)]
pub struct MvfsResult {
    pub solid: SolidId,
    pub shell: ShellId,
    pub face: FaceId,
    pub loop_: LoopId,
    pub vertex: VertexId,
}

impl Solid {
    /// Make Vertex-Face-Solid: bootstrap a minimal solid (1 vertex, 1 empty-loop
    /// face, 1 shell, 1 solid). Euler: V=1, E=0, F=1, R=0, S=1 → 1-0+1-0 = 2 = 2*(1-0). ✓
    pub fn mvfs(&mut self) -> MvfsResult {
        let solid_id = self.solids.insert(());
        if self.solid_id.is_none() {
            self.solid_id = Some(solid_id);
        }
        let shell_id = self.shells.insert(Shell { faces: Vec::new(), solid: solid_id });
        let face_id = self.faces.insert(Face {
            outer_loop: LoopId::null(),
            inner_loops: Vec::new(),
            shell: shell_id,
        });
        let loop_id = self.loops.insert(Loop { half_edge: None, face: face_id });
        self.faces[face_id].outer_loop = loop_id;
        self.shells[shell_id].faces.push(face_id);
        let vertex_id = self.vertices.insert(Vertex { outgoing: None });

        MvfsResult {
            solid: solid_id,
            shell: shell_id,
            face: face_id,
            loop_: loop_id,
            vertex: vertex_id,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mvfs_creates_one_of_each_top_level_entity() {
        let mut s = Solid::new();
        let r = s.mvfs();
        assert_eq!(s.vertex_count(), 1);
        assert_eq!(s.edge_count(), 0);
        assert_eq!(s.face_count(), 1);
        assert_eq!(s.shell_count(), 1);
        assert_eq!(s.face(r.face).unwrap().outer_loop, r.loop_);
        assert_eq!(s.face(r.face).unwrap().shell, r.shell);
        assert_eq!(s.shell(r.shell).unwrap().faces, vec![r.face]);
        assert_eq!(s.loop_(r.loop_).unwrap().half_edge, None);
        assert_eq!(s.vertex(r.vertex).unwrap().outgoing, None);
    }

    #[test]
    fn euler_invariant_holds_after_mvfs() {
        let mut s = Solid::new();
        s.mvfs();
        let v = s.vertex_count() as i64;
        let e = s.edge_count() as i64;
        let f = s.face_count() as i64;
        let s_count = s.shell_count() as i64;
        assert_eq!(v - e + f, 2 * s_count);
    }
}
