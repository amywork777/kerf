//! JSON serialization / deserialization for `brep::Solid`.
//!
//! # Example
//! ```rust,no_run
//! use kerf_brep::{primitives::box_, serde_io::{read_json, write_json}};
//! use kerf_geom::Vec3;
//!
//! let solid = box_(Vec3::new(1.0, 2.0, 3.0));
//! let mut buf: Vec<u8> = Vec::new();
//! write_json(&solid, &mut buf).unwrap();
//! let solid2 = read_json(&mut buf.as_slice()).unwrap();
//! assert_eq!(solid.vertex_count(), solid2.vertex_count());
//! ```

use std::io::{Read, Write};

use crate::solid::Solid;

/// Serialize `solid` as JSON and write it to `w`.
pub fn write_json(solid: &Solid, w: &mut impl Write) -> serde_json::Result<()> {
    let bytes = serde_json::to_vec(solid)?;
    w.write_all(&bytes).map_err(serde_json::Error::io)
}

/// Deserialize a `Solid` from JSON read from `r`.
pub fn read_json(r: &mut impl Read) -> serde_json::Result<Solid> {
    let mut buf = Vec::new();
    r.read_to_end(&mut buf).map_err(serde_json::Error::io)?;
    serde_json::from_slice(&buf)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::primitives::{box_, box_at, cylinder};
    use kerf_geom::{Point3, Vec3};

    #[test]
    fn box_round_trips_through_json() {
        let s = box_(Vec3::new(1.0, 2.0, 3.0));
        let mut buf = Vec::new();
        write_json(&s, &mut buf).unwrap();
        let s2 = read_json(&mut buf.as_slice()).unwrap();
        assert_eq!(s.vertex_count(), s2.vertex_count());
        assert_eq!(s.edge_count(), s2.edge_count());
        assert_eq!(s.face_count(), s2.face_count());
        kerf_topo::validate(&s2.topo).unwrap();
    }

    #[test]
    fn cylinder_round_trips_through_json() {
        let s = cylinder(1.0, 2.0);
        let mut buf = Vec::new();
        write_json(&s, &mut buf).unwrap();
        let s2 = read_json(&mut buf.as_slice()).unwrap();
        assert_eq!(s.vertex_count(), s2.vertex_count());
        kerf_topo::validate(&s2.topo).unwrap();
    }

    #[test]
    fn boolean_output_round_trips_through_json() {
        let a = box_(Vec3::new(2.0, 2.0, 2.0));
        let b = box_at(Vec3::new(2.0, 2.0, 2.0), Point3::new(1.0, 0.0, 0.0));
        let r = a.union(&b);
        let mut buf = Vec::new();
        write_json(&r, &mut buf).unwrap();
        let r2 = read_json(&mut buf.as_slice()).unwrap();
        assert_eq!(r.face_count(), r2.face_count());
        kerf_topo::validate(&r2.topo).unwrap();
    }

    #[test]
    fn json_box_reports_byte_size() {
        let s = box_(Vec3::new(1.0, 2.0, 3.0));
        let mut buf = Vec::new();
        write_json(&s, &mut buf).unwrap();
        // Just print for information; no assertion on exact size.
        eprintln!("Serialized box JSON size: {} bytes", buf.len());
        assert!(!buf.is_empty());
    }
}
