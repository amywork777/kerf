//! Core geometric types: points, vectors, axes, and orthonormal frames.

use serde::{Deserialize, Serialize};

pub type Point3 = nalgebra::Point3<f64>;
pub type Vec3 = nalgebra::Vector3<f64>;

/// A directed line: a point on the line plus a unit direction.
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct Axis {
    pub origin: Point3,
    pub direction: Vec3, // unit length
}

impl Axis {
    /// Construct from origin + direction. Returns `None` if `direction` has zero magnitude.
    pub fn new(origin: Point3, direction: Vec3) -> Option<Self> {
        Some(Axis {
            origin,
            direction: direction.try_normalize(0.0)?,
        })
    }
}

/// An orthonormal right-handed frame: origin + (x, y, z) unit basis.
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct Frame {
    pub origin: Point3,
    pub x: Vec3,
    pub y: Vec3,
    pub z: Vec3,
}

impl Frame {
    /// Construct a frame from an origin, an x direction, and an approximate y direction.
    /// `y_hint` is reprojected to be exactly perpendicular to `x`.
    /// Returns `None` if `x` has zero magnitude or `y_hint` is collinear with `x`.
    /// Non-finite inputs are not explicitly checked; behavior is unspecified for NaN/Inf.
    pub fn from_x_yhint(origin: Point3, x: Vec3, y_hint: Vec3) -> Option<Self> {
        let x = x.try_normalize(0.0)?;
        let z = x.cross(&y_hint).try_normalize(0.0)?;
        let y = z.cross(&x); // already unit by construction
        Some(Frame { origin, x, y, z })
    }

    /// Frame whose x/y/z are world x/y/z, anchored at `origin`.
    pub fn world(origin: Point3) -> Self {
        Frame {
            origin,
            x: Vec3::x(),
            y: Vec3::y(),
            z: Vec3::z(),
        }
    }

    /// World-coordinates of a point given in this frame's local coordinates.
    pub fn point_from_local(&self, lx: f64, ly: f64, lz: f64) -> Point3 {
        self.origin + lx * self.x + ly * self.y + lz * self.z
    }

    /// Local coordinates of a world-coordinates point.
    pub fn local_of(&self, p: Point3) -> (f64, f64, f64) {
        let d = p - self.origin;
        (d.dot(&self.x), d.dot(&self.y), d.dot(&self.z))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn frame_world_round_trips_origin_and_axes() {
        let f = Frame::world(Point3::new(1.0, 2.0, 3.0));
        let p = f.point_from_local(4.0, 5.0, 6.0);
        assert_relative_eq!(p, Point3::new(5.0, 7.0, 9.0));
        let (lx, ly, lz) = f.local_of(p);
        assert_relative_eq!(lx, 4.0);
        assert_relative_eq!(ly, 5.0);
        assert_relative_eq!(lz, 6.0);
    }

    #[test]
    fn frame_from_x_yhint_orthonormalizes() {
        let f = Frame::from_x_yhint(
            Point3::origin(),
            Vec3::new(2.0, 0.0, 0.0), // x will normalize
            Vec3::new(0.5, 1.0, 0.0), // not unit, not perp
        )
        .unwrap();
        assert_relative_eq!(f.x.dot(&f.y), 0.0, epsilon = 1e-12);
        assert_relative_eq!(f.y.dot(&f.z), 0.0, epsilon = 1e-12);
        assert_relative_eq!(f.x.dot(&f.z), 0.0, epsilon = 1e-12);
        assert_relative_eq!(f.x.norm(), 1.0, epsilon = 1e-12);
        assert_relative_eq!(f.y.norm(), 1.0, epsilon = 1e-12);
        assert_relative_eq!(f.z.norm(), 1.0, epsilon = 1e-12);
        assert_relative_eq!(f.x.cross(&f.y), f.z, epsilon = 1e-12);
    }

    #[test]
    fn frame_from_x_yhint_rejects_collinear_inputs() {
        assert!(
            Frame::from_x_yhint(
                Point3::origin(),
                Vec3::new(1.0, 0.0, 0.0),
                Vec3::new(2.0, 0.0, 0.0),
            )
            .is_none()
        );
    }

    #[test]
    fn axis_new_normalizes_direction_and_rejects_zero() {
        let a = Axis::new(Point3::origin(), Vec3::new(0.0, 3.0, 4.0)).unwrap();
        assert_relative_eq!(a.direction.norm(), 1.0, epsilon = 1e-12);
        assert_relative_eq!(a.direction, Vec3::new(0.0, 0.6, 0.8), epsilon = 1e-12);
        assert!(Axis::new(Point3::origin(), Vec3::zeros()).is_none());
    }
}
