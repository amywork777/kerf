//! Boolean operation selection: which classified faces to keep per op.

use kerf_topo::FaceId;

use crate::booleans::FaceClassification;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BooleanOp {
    Union,
    Intersection,
    Difference, // a - b
}

/// Should this face from solid A be kept, given its classification against B?
pub fn keep_a_face(cls: FaceClassification, op: BooleanOp) -> bool {
    use BooleanOp::*;
    use FaceClassification::*;
    match (op, cls) {
        (Union, Outside) => true,
        (Union, Inside) => false,
        (Union, OnBoundary) => true, // include one copy from A
        (Union, OnBoundaryOpposite) => true, // touching: A's outer face stays
        (Intersection, Outside) => false,
        (Intersection, Inside) => true,
        (Intersection, OnBoundary) => true,
        (Intersection, OnBoundaryOpposite) => false, // touching: ∩ is empty here
        (Difference, Outside) => true,
        (Difference, Inside) => false,
        // OnBoundary faces from A are shared with B (coplanar-coincident).
        // For A - B, the shared boundary face is interior to B's enclosed
        // region (when normals agree) or on the cavity wall (opposite). The
        // pragma here drops it on the A side and re-emits it via B's flipped
        // face when needed — matches half-overlap difference geometry.
        (Difference, OnBoundary) => false,
        // Touching: A's face is the outer boundary of the result; B's
        // touches but is not subtracted (no volumetric overlap).
        (Difference, OnBoundaryOpposite) => true,
    }
}

/// Should this face from solid B be kept, given its classification against A?
/// For Difference, kept faces from B are flipped (their normals are inverted).
pub fn keep_b_face(cls: FaceClassification, op: BooleanOp) -> bool {
    use BooleanOp::*;
    use FaceClassification::*;
    match (op, cls) {
        (Union, Outside) => true,
        (Union, Inside) => false,
        (Union, OnBoundary) => false, // A already contributed it
        (Union, OnBoundaryOpposite) => true, // touching: B's outer face also stays
        (Intersection, Outside) => false,
        (Intersection, Inside) => true,
        (Intersection, OnBoundary) => false,
        (Intersection, OnBoundaryOpposite) => false, // empty intersection
        (Difference, Outside) => false,
        (Difference, Inside) => true, // flipped at output time
        (Difference, OnBoundary) => false,
        (Difference, OnBoundaryOpposite) => false, // touching: B contributes nothing
    }
}

/// For Difference, faces from B are emitted with reversed orientation.
pub fn flip_b_face(op: BooleanOp) -> bool {
    matches!(op, BooleanOp::Difference)
}

/// Carrier for selected face IDs from the two inputs.
#[derive(Clone, Debug, Default)]
pub struct SelectedFaces {
    pub a: Vec<FaceId>,
    pub b: Vec<FaceId>,
    pub b_flipped: bool,
}

#[cfg(test)]
mod tests {
    use super::*;
    use FaceClassification::*;

    #[test]
    fn union_keeps_outside_faces() {
        assert!(keep_a_face(Outside, BooleanOp::Union));
        assert!(keep_b_face(Outside, BooleanOp::Union));
        assert!(!keep_a_face(Inside, BooleanOp::Union));
        assert!(!keep_b_face(Inside, BooleanOp::Union));
    }

    #[test]
    fn intersection_keeps_inside_faces() {
        assert!(keep_a_face(Inside, BooleanOp::Intersection));
        assert!(keep_b_face(Inside, BooleanOp::Intersection));
        assert!(!keep_a_face(Outside, BooleanOp::Intersection));
        assert!(!keep_b_face(Outside, BooleanOp::Intersection));
    }

    #[test]
    fn difference_keeps_a_outside_and_b_inside_flipped() {
        assert!(keep_a_face(Outside, BooleanOp::Difference));
        assert!(!keep_a_face(Inside, BooleanOp::Difference));
        assert!(keep_b_face(Inside, BooleanOp::Difference));
        assert!(!keep_b_face(Outside, BooleanOp::Difference));
        assert!(flip_b_face(BooleanOp::Difference));
    }
}
