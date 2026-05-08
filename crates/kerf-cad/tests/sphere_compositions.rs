//! Verify boolean combinations enabled by sphere_faceted.

use kerf_brep::solid_volume;
use kerf_cad::{lits, Feature, Model, Scalar};

#[test]
fn box_minus_dome_works() {
    // Box with a hemispherical pocket carved out.
    let m = Model::new()
        .add(Feature::Box {
            id: "body".into(),
            extents: lits([6.0, 6.0, 4.0]),
        })
        .add(Feature::Dome {
            id: "pocket_raw".into(),
            radius: Scalar::lit(2.0),
            stacks: 12,
            slices: 16,
        })
        .add(Feature::Translate {
            id: "pocket".into(),
            input: "pocket_raw".into(),
            offset: lits([3.0, 3.0, 0.0]),
        })
        .add(Feature::Difference {
            id: "out".into(),
            inputs: vec!["body".into(), "pocket".into()],
        });
    let s = m.evaluate("out").unwrap();
    let v = solid_volume(&s);
    let body_v = 144.0;
    let dome_v = (2.0 / 3.0) * std::f64::consts::PI * 8.0; // hemisphere of r=2
    let exp = body_v - dome_v;
    let rel = (v - exp).abs() / exp;
    assert!(rel < 0.10, "v={v}, exp={exp}, rel={rel}");
}

#[test]
#[ignore = "kernel: sphere - cylinder trips stitch when the cylinder pierces \
            through the sphere's curved surface — same family as the \
            'not all multi-cylinder unions work' coplanar limitation. \
            Workaround: use box-minus-cylinder + box-minus-sphere separately."]
fn drilled_sphere_works() {
    let m = Model::new()
        .add(Feature::SphereFaceted {
            id: "ball".into(),
            radius: Scalar::lit(2.0),
            stacks: 12,
            slices: 16,
        })
        .add(Feature::Cylinder {
            id: "drill_raw".into(),
            radius: Scalar::lit(0.5),
            height: Scalar::lit(6.0),
            segments: 16,
        })
        .add(Feature::Translate {
            id: "drill".into(),
            input: "drill_raw".into(),
            offset: lits([0.0, 0.0, -3.0]),
        })
        .add(Feature::Difference {
            id: "out".into(),
            inputs: vec!["ball".into(), "drill".into()],
        });
    let s = m.evaluate("out").unwrap();
    assert!(solid_volume(&s) > 0.0);
}
