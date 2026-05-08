//! Batch 11: Cup, Bottle, TableLeg, ChairLeg, Bookshelf, PlanterBox,
//! DrawerSlot.

use std::f64::consts::PI;

use kerf_brep::measure::solid_volume;
use kerf_cad::{Feature, Model, Scalar};

fn lit(x: f64) -> Scalar { Scalar::lit(x) }

fn faceted_cyl_vol(r: f64, h: f64, segs: usize) -> f64 {
    0.5 * segs as f64 * r * r * (2.0 * PI / segs as f64).sin() * h
}

#[test]
fn cup_volume_matches_outer_minus_cavity() {
    let r = 2.0; let h = 4.0; let w = 0.3; let segs = 16;
    let m = Model::new().add(Feature::Cup {
        id: "c".into(),
        outer_radius: lit(r), height: lit(h), wall_thickness: lit(w),
        segments: segs,
    });
    let s = m.evaluate("c").unwrap();
    let v = solid_volume(&s);
    let outer_v = faceted_cyl_vol(r, h, segs);
    let cavity_v = faceted_cyl_vol(r - w, h - w, segs);
    let exp = outer_v - cavity_v;
    let rel = (v - exp).abs() / exp;
    assert!(rel < 0.02, "cup v={v}, exp={exp}, rel={rel}");
}

#[test]
fn bottle_volume_matches_two_cylinders() {
    let br = 1.0; let bh = 3.0; let nr = 0.3; let nh = 1.0; let segs = 16;
    let m = Model::new().add(Feature::Bottle {
        id: "b".into(),
        body_radius: lit(br), body_height: lit(bh),
        neck_radius: lit(nr), neck_height: lit(nh),
        segments: segs,
    });
    let s = m.evaluate("b").unwrap();
    let v = solid_volume(&s);
    let exp = faceted_cyl_vol(br, bh, segs) + faceted_cyl_vol(nr, nh, segs);
    let rel = (v - exp).abs() / exp;
    assert!(rel < 0.02);
}

#[test]
fn table_leg_volume_matches_frustum() {
    let br = 0.5; let tr = 0.3; let h = 4.0; let segs = 16;
    let m = Model::new().add(Feature::TableLeg {
        id: "tl".into(),
        bottom_radius: lit(br), top_radius: lit(tr),
        height: lit(h), segments: segs,
    });
    let s = m.evaluate("tl").unwrap();
    let v = solid_volume(&s);
    let a_top = 0.5 * segs as f64 * tr * tr * (2.0 * PI / segs as f64).sin();
    let a_bot = 0.5 * segs as f64 * br * br * (2.0 * PI / segs as f64).sin();
    let exp = h / 3.0 * (a_top + a_bot + (a_top * a_bot).sqrt());
    let rel = (v - exp).abs() / exp;
    assert!(rel < 0.05);
}

#[test]
fn chair_leg_volume_matches_box() {
    let m = Model::new().add(Feature::ChairLeg {
        id: "cl".into(),
        width: lit(0.5), depth: lit(0.5), height: lit(3.0),
    });
    let s = m.evaluate("cl").unwrap();
    let v = solid_volume(&s);
    assert!((v - 0.75).abs() < 1e-12);
}

#[test]
fn bookshelf_completes() {
    let m = Model::new().add(Feature::Bookshelf {
        id: "bs".into(),
        width: lit(8.0), depth: lit(2.0),
        shelves: 4, shelf_thickness: lit(0.2),
        clear_height: lit(2.0), side_thickness: lit(0.3),
    });
    let s = m.evaluate("bs").unwrap();
    assert!(solid_volume(&s) > 0.0);
}

#[test]
fn planter_box_volume_below_full_box() {
    let w = 4.0; let l = 5.0; let h = 3.0; let t = 0.3;
    let m = Model::new().add(Feature::PlanterBox {
        id: "p".into(),
        outer_width: lit(w), outer_length: lit(l),
        outer_height: lit(h), wall_thickness: lit(t),
    });
    let s = m.evaluate("p").unwrap();
    let v = solid_volume(&s);
    let outer = w * l * h;
    let cavity = (w - 2.0 * t) * (l - 2.0 * t) * (h - t);
    let exp = outer - cavity;
    let rel = (v - exp).abs() / exp;
    assert!(rel < 0.02);
}

#[test]
fn drawer_slot_completes() {
    let m = Model::new().add(Feature::DrawerSlot {
        id: "d".into(),
        width: lit(4.0), depth: lit(3.0), height: lit(2.0),
        wall_thickness: lit(0.3),
    });
    let s = m.evaluate("d").unwrap();
    assert!(solid_volume(&s) > 0.0);
}

#[test]
fn batch_11_round_trips_via_json() {
    let m = Model::new()
        .add(Feature::Cup { id: "c".into(), outer_radius: lit(2.0), height: lit(4.0), wall_thickness: lit(0.3), segments: 16 })
        .add(Feature::Bottle { id: "b".into(), body_radius: lit(1.0), body_height: lit(3.0), neck_radius: lit(0.3), neck_height: lit(1.0), segments: 16 })
        .add(Feature::TableLeg { id: "tl".into(), bottom_radius: lit(0.5), top_radius: lit(0.3), height: lit(4.0), segments: 16 })
        .add(Feature::ChairLeg { id: "cl".into(), width: lit(0.5), depth: lit(0.5), height: lit(3.0) })
        .add(Feature::Bookshelf { id: "bs".into(), width: lit(8.0), depth: lit(2.0), shelves: 4, shelf_thickness: lit(0.2), clear_height: lit(2.0), side_thickness: lit(0.3) })
        .add(Feature::PlanterBox { id: "p".into(), outer_width: lit(4.0), outer_length: lit(5.0), outer_height: lit(3.0), wall_thickness: lit(0.3) })
        .add(Feature::DrawerSlot { id: "d".into(), width: lit(4.0), depth: lit(3.0), height: lit(2.0), wall_thickness: lit(0.3) });
    let json = m.to_json_string().unwrap();
    let parsed = Model::from_json_str(&json).unwrap();
    assert_eq!(json, parsed.to_json_string().unwrap());
}
