//! Serialization round-trip integrity: write to STL/JSON/OBJ, read back,
//! compare. Catches encoder/decoder bugs that silently corrupt geometry.

use std::io::Cursor;

use kerf_brep::primitives::{box_, box_at, cylinder_faceted, extrude_polygon};
use kerf_brep::{
    read_obj_to_solid, read_stl_binary_to_solid, solid_volume, tessellate, write_binary, write_obj,
    Solid,
};
use kerf_geom::{Point3, Vec3};

const VOL_TOL: f64 = 1e-6;

/// STL round-trip: tessellate → write_binary → read_stl_binary_to_solid →
/// volume must match within tolerance.
fn stl_roundtrip(s: &Solid, label: &str) {
    let soup = tessellate(s, 24);
    let mut buf = Vec::new();
    write_binary(&soup, label, &mut buf).expect("write_binary");
    let mut reader = Cursor::new(buf);
    let read_back = read_stl_binary_to_solid(&mut reader).expect("read_stl_binary_to_solid");

    let v_in = solid_volume(s);
    let v_out = solid_volume(&read_back);
    assert!(
        (v_in - v_out).abs() < VOL_TOL.max(v_in.abs() * 1e-3),
        "{label}: STL round-trip volume drift: in={v_in} out={v_out} diff={}",
        (v_in - v_out).abs()
    );
}

#[test]
fn box_stl_roundtrip_preserves_volume() {
    stl_roundtrip(&box_(Vec3::new(2.0, 3.0, 4.0)), "box");
}

#[test]
fn box_at_stl_roundtrip_preserves_volume() {
    stl_roundtrip(
        &box_at(Vec3::new(1.5, 2.5, 0.7), Point3::new(-3.0, 4.0, 7.0)),
        "box-at",
    );
}

#[test]
fn cylinder_stl_roundtrip_preserves_volume() {
    stl_roundtrip(&cylinder_faceted(0.6, 3.0, 24), "cylinder");
}

#[test]
fn tri_prism_stl_roundtrip_preserves_volume() {
    stl_roundtrip(
        &extrude_polygon(
            &[
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(2.0, 0.0, 0.0),
                Point3::new(1.0, 1.5, 0.0),
            ],
            Vec3::new(0.0, 0.0, 2.0),
        ),
        "tri-prism",
    );
}

#[test]
fn boolean_result_stl_roundtrip_preserves_volume() {
    let a = box_(Vec3::new(2.0, 2.0, 2.0));
    let b = box_at(Vec3::new(2.0, 2.0, 2.0), Point3::new(1.0, 0.0, 0.0));
    let r = a.try_union(&b).unwrap();
    stl_roundtrip(&r, "box ∪ box-shift");

    let r = a.try_intersection(&b).unwrap();
    stl_roundtrip(&r, "box ∩ box-shift");

    let r = a.try_difference(&b).unwrap();
    stl_roundtrip(&r, "box − box-shift");
}

#[test]
fn nested_diff_stl_roundtrip_preserves_volume() {
    // 2-shell solid (outer + cavity) — roundtrip should preserve both.
    let big = box_(Vec3::new(10.0, 10.0, 10.0));
    let small = box_at(Vec3::new(2.0, 2.0, 2.0), Point3::new(4.0, 4.0, 4.0));
    let r = big.try_difference(&small).unwrap();
    stl_roundtrip(&r, "big − nested small");
}

#[test]
fn obj_roundtrip_preserves_volume_for_box() {
    let s = box_(Vec3::new(2.0, 3.0, 4.0));
    let soup = tessellate(&s, 24);
    let mut buf = Vec::new();
    write_obj(&soup, "box", &mut buf).expect("write_obj");
    let mut reader = Cursor::new(buf);
    let read_back = read_obj_to_solid(&mut reader).expect("read_obj_to_solid");

    let v_in = solid_volume(&s);
    let v_out = solid_volume(&read_back);
    assert!(
        (v_in - v_out).abs() < VOL_TOL,
        "OBJ round-trip volume drift: in={v_in} out={v_out}"
    );
}
