//! Solid constructors. v1: box_, extrude_polygon. M13: cylinder. M14: cone. M15: sphere. M16: torus. M17: frustum. M20: revolve. M25+: capsule_faceted.

pub mod box_;
pub mod capsule;
pub mod cone;
pub mod cylinder;
pub mod cylinder_faceted;
pub mod extrude;
pub mod frustum;
pub mod revolve;
pub mod sphere;
pub mod torus;

pub use box_::{box_, box_at};
pub use capsule::{capsule_faceted, capsule_volume_analytic};
pub use cone::cone;
pub use cylinder::cylinder;
pub use cylinder_faceted::cylinder_faceted;
pub use extrude::extrude_polygon;
pub use frustum::frustum;
pub use revolve::revolve_polyline;
pub use sphere::sphere;
pub use torus::torus;
