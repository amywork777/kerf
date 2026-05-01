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
        (Intersection, Outside) => false,
        (Intersection, Inside) => true,
        (Intersection, OnBoundary) => true,
        (Difference, Outside) => true,
        (Difference, Inside) => false,
        (Difference, OnBoundary) => true,
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
        (Intersection, Outside) => false,
        (Intersection, Inside) => true,
        (Intersection, OnBoundary) => false,
        (Difference, Outside) => false,
        (Difference, Inside) => true, // flipped at output time
        (Difference, OnBoundary) => false,
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
