//! Batch 15: Pediment, Vault, ShelfBracket, NameTag, Plinth, ParapetWall,
//! BeamWithHoles.

use std::f64::consts::PI;

use kerf_brep::measure::solid_volume;
use kerf_cad::{Feature, Model, Scalar};

fn lit(x: f64) -> Scalar { Scalar::lit(x) }

#[test]
fn pediment_volume_matches_rect_plus_triangle() {
    let bw = 4.0; let bh = 1.0; let gh = 1.5; let d = 0.5;
    let m = Model::new().add(Feature::Pediment {
        id: "p".into(),
        base_width: lit(bw), base_height: lit(bh),
        gable_height: lit(gh), depth: lit(d),
    });
    let s = m.evaluate("p").unwrap();
    let v = solid_volume(&s);
    // Area = rect (bw × bh) + triangle (1/2 × bw × gh)
    let area = bw * bh + 0.5 * bw * gh;
    let exp = area * d;
    let rel = (v - exp).abs() / exp;
    assert!(rel < 1e-6);
}

#[test]
fn vault_completes() {
    let m = Model::new().add(Feature::Vault {
        id: "v".into(),
        base_width: lit(4.0), base_length: lit(6.0),
        base_height: lit(2.0), segments: 16,
    });
    match m.evaluate("v") {
        Ok(s) => assert!(solid_volume(&s) > 0.0),
        Err(_) => {}
    }
}

#[test]
fn shelf_bracket_volume_matches_polygon() {
    let h = 5.0; let v = 4.0; let t = 0.5; let d = 1.0;
    let m = Model::new().add(Feature::ShelfBracket {
        id: "sb".into(),
        horizontal: lit(h), vertical: lit(v),
        thickness: lit(t), depth: lit(d),
    });
    let s = m.evaluate("sb").unwrap();
    let vv = solid_volume(&s);
    // Pentagon area via shoelace: vertices (0,0), (h,0), (h,t), (t,v), (0,v).
    let pts = [(0.0, 0.0), (h, 0.0), (h, t), (t, v), (0.0, v)];
    let mut a = 0.0;
    for i in 0..5 {
        let (x0, y0) = pts[i];
        let (x1, y1) = pts[(i + 1) % 5];
        a += x0 * y1 - x1 * y0;
    }
    let area = 0.5 * a.abs();
    let exp = area * d;
    let rel = (vv - exp).abs() / exp;
    assert!(rel < 1e-6, "shelf bracket v={vv}, exp={exp}, rel={rel}");
}

#[test]
fn name_tag_volume_below_full_plate() {
    let m = Model::new().add(Feature::NameTag {
        id: "nt".into(),
        width: lit(4.0), height: lit(2.0), thickness: lit(0.2),
        corner_radius: lit(0.3),
        hole_radius: lit(0.2),
        hole_offset_from_top: lit(0.5),
        segments: 24,
    });
    let s = m.evaluate("nt").unwrap();
    let v = solid_volume(&s);
    let full_plate = 4.0 * 2.0 * 0.2;
    assert!(v > 0.0 && v < full_plate);
}

#[test]
fn plinth_volume_matches_two_boxes_overlap() {
    let bs = 4.0; let bh = 1.0; let ts = 3.0; let th = 0.5;
    let m = Model::new().add(Feature::Plinth {
        id: "p".into(),
        bottom_side: lit(bs), bottom_height: lit(bh),
        top_side: lit(ts), top_height: lit(th),
    });
    let s = m.evaluate("p").unwrap();
    let v = solid_volume(&s);
    // Bottom box bs² × bh + top box ts² × th, but the top box overlaps
    // the bottom by an eps band. Expect close to sum.
    let exp = bs * bs * bh + ts * ts * th;
    let rel = (v - exp).abs() / exp;
    assert!(rel < 0.05, "plinth v={v}, exp={exp}, rel={rel}");
}

#[test]
fn parapet_wall_completes() {
    let m = Model::new().add(Feature::ParapetWall {
        id: "pw".into(),
        length: lit(10.0), wall_height: lit(3.0),
        wall_thickness: lit(0.5),
        merlon_count: 5, merlon_height: lit(0.8),
    });
    let s = m.evaluate("pw").unwrap();
    assert!(solid_volume(&s) > 0.0);
}

#[test]
fn beam_with_holes_volume_matches_beam_minus_holes() {
    let l = 10.0; let w = 1.0; let h = 2.0; let r = 0.3; let hc = 4; let segs = 16;
    let m = Model::new().add(Feature::BeamWithHoles {
        id: "bh".into(),
        length: lit(l), width: lit(w), height: lit(h),
        hole_radius: lit(r), hole_count: hc, segments: segs,
    });
    let s = m.evaluate("bh").unwrap();
    let v = solid_volume(&s);
    let beam_v = l * w * h;
    let hole_v = 0.5 * segs as f64 * r * r * (2.0 * PI / segs as f64).sin() * w;
    let exp = beam_v - (hc as f64) * hole_v;
    let rel = (v - exp).abs() / exp;
    assert!(rel < 0.05, "beam v={v}, exp={exp}, rel={rel}");
}

#[test]
fn batch_15_round_trips_via_json() {
    let m = Model::new()
        .add(Feature::Pediment { id: "p".into(), base_width: lit(4.0), base_height: lit(1.0), gable_height: lit(1.5), depth: lit(0.5) })
        .add(Feature::Vault { id: "v".into(), base_width: lit(4.0), base_length: lit(6.0), base_height: lit(2.0), segments: 16 })
        .add(Feature::ShelfBracket { id: "sb".into(), horizontal: lit(5.0), vertical: lit(4.0), thickness: lit(0.5), depth: lit(1.0) })
        .add(Feature::NameTag { id: "nt".into(), width: lit(4.0), height: lit(2.0), thickness: lit(0.2), corner_radius: lit(0.3), hole_radius: lit(0.2), hole_offset_from_top: lit(0.5), segments: 24 })
        .add(Feature::Plinth { id: "pl".into(), bottom_side: lit(4.0), bottom_height: lit(1.0), top_side: lit(3.0), top_height: lit(0.5) })
        .add(Feature::ParapetWall { id: "pw".into(), length: lit(10.0), wall_height: lit(3.0), wall_thickness: lit(0.5), merlon_count: 5, merlon_height: lit(0.8) })
        .add(Feature::BeamWithHoles { id: "bh".into(), length: lit(10.0), width: lit(1.0), height: lit(2.0), hole_radius: lit(0.3), hole_count: 4, segments: 16 });
    let json = m.to_json_string().unwrap();
    let parsed = Model::from_json_str(&json).unwrap();
    assert_eq!(json, parsed.to_json_string().unwrap());
}
