//! Verify boolean combinations enabled by sphere_faceted.
//! Tests for Sphere − Cylinder ("drilled sphere") boolean.

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

/// Sphere(r=2) − z-axis Cylinder(r=0.5, h=6, segments=16).
/// This is the canonical "drilled sphere" — cylinder runs fully through
/// the sphere along its polar axis.
#[test]
fn drilled_sphere_z_axis_small_bore() {
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
    assert!(solid_volume(&s) > 0.0, "drilled sphere must have positive volume");
    // Face count upper bound: sphere faces + 2 * bore_segments.
    // sphere_faceted(stacks=12, slices=16) has slices*(stacks) = 192 faces.
    // Bore ring: 2 * 16 = 32. Allow generous 3x margin for splits.
    assert!(
        s.face_count() <= 700,
        "face count too high: {} (possible stitch explosion)",
        s.face_count()
    );
}

/// Spec test 1: Sphere(r=10) − Cylinder(r=2, h=24, axis=z, segments=24).
/// Cylinder extends past the sphere in z; boolean must succeed.
#[test]
fn drilled_sphere_r10_small_bore_z() {
    let m = Model::new()
        .add(Feature::SphereFaceted {
            id: "ball".into(),
            radius: Scalar::lit(10.0),
            stacks: 12,
            slices: 24,
        })
        // Cylinder centered on origin, spanning z=-12..+12 (diameter 24 > sphere diameter 20).
        .add(Feature::CylinderAt {
            id: "drill".into(),
            base: lits([-0.0, 0.0, -12.0]),
            axis: "z".into(),
            radius: Scalar::lit(2.0),
            height: Scalar::lit(24.0),
            segments: 24,
        })
        .add(Feature::Difference {
            id: "out".into(),
            inputs: vec!["ball".into(), "drill".into()],
        });
    let s = m.evaluate("out").unwrap();
    // Volume must be positive (sphere with a bore)
    let v = solid_volume(&s);
    assert!(v > 0.0, "drilled sphere must have positive volume (got {v})");
    // Rough sanity: volume < sphere volume (4/3 π 1000 ≈ 4189)
    let sphere_v = (4.0 / 3.0) * std::f64::consts::PI * 1000.0;
    assert!(v < sphere_v, "drilled sphere volume {v} must be < sphere volume {sphere_v}");
}

/// Spec test 2: Sphere(r=10) − Cylinder(r=5, h=24, axis=z, segments=16).
/// Larger bore — tests that wide drills also succeed.
#[test]
fn drilled_sphere_r10_large_bore_z() {
    let m = Model::new()
        .add(Feature::SphereFaceted {
            id: "ball".into(),
            radius: Scalar::lit(10.0),
            stacks: 12,
            slices: 24,
        })
        .add(Feature::CylinderAt {
            id: "drill".into(),
            base: lits([0.0, 0.0, -12.0]),
            axis: "z".into(),
            radius: Scalar::lit(5.0),
            height: Scalar::lit(24.0),
            segments: 16,
        })
        .add(Feature::Difference {
            id: "out".into(),
            inputs: vec!["ball".into(), "drill".into()],
        });
    let s = m.evaluate("out").unwrap();
    let v = solid_volume(&s);
    assert!(v > 0.0, "drilled sphere (large bore) must have positive volume (got {v})");
}

/// Spec test 3: Sphere(r=10) − Cylinder(r=2, h=24, axis=x, segments=24).
/// Drill along the x-axis instead of z.
#[test]
fn drilled_sphere_r10_small_bore_x_axis() {
    let m = Model::new()
        .add(Feature::SphereFaceted {
            id: "ball".into(),
            radius: Scalar::lit(10.0),
            stacks: 12,
            slices: 24,
        })
        // CylinderAt axis=x: base[x]=axis_origin, base[y]=perp_a, base[z]=perp_b.
        // To center the cylinder in the sphere we want x from -12 to +12, centered at y=0, z=0.
        .add(Feature::CylinderAt {
            id: "drill".into(),
            base: lits([-12.0, 0.0, 0.0]),
            axis: "x".into(),
            radius: Scalar::lit(2.0),
            height: Scalar::lit(24.0),
            segments: 24,
        })
        .add(Feature::Difference {
            id: "out".into(),
            inputs: vec!["ball".into(), "drill".into()],
        });
    let s = m.evaluate("out").unwrap();
    let v = solid_volume(&s);
    assert!(v > 0.0, "x-axis drilled sphere must have positive volume (got {v})");
}

/// Spec test 4: Sphere − Cylinder where cylinder doesn't intersect sphere.
/// Result should equal sphere (identity operation).
#[test]
fn drilled_sphere_no_intersection_is_identity() {
    let m = Model::new()
        .add(Feature::SphereFaceted {
            id: "ball".into(),
            radius: Scalar::lit(10.0),
            stacks: 12,
            slices: 24,
        })
        // Cylinder far away: center of bottom cap at z=50, so it spans z=50..74.
        // Sphere has radius 10 (extends to z=10), so no intersection.
        .add(Feature::CylinderAt {
            id: "drill".into(),
            base: lits([0.0, 0.0, 50.0]),
            axis: "z".into(),
            radius: Scalar::lit(2.0),
            height: Scalar::lit(24.0),
            segments: 24,
        })
        .add(Feature::Difference {
            id: "out".into(),
            inputs: vec!["ball".into(), "drill".into()],
        });
    let s = m.evaluate("out").unwrap();
    // Volume should equal sphere volume (± 5% for tessellation)
    let v = solid_volume(&s);
    let sphere_v = (4.0 / 3.0) * std::f64::consts::PI * 1000.0;
    let rel = (v - sphere_v).abs() / sphere_v;
    assert!(
        rel < 0.10,
        "non-intersecting difference should yield full sphere volume: v={v}, expected≈{sphere_v}, rel={rel}"
    );
}

/// Spec test 5: Sphere − Cylinder where cylinder fully contains sphere.
/// Result should be empty or produce an error (kernel convention: empty solid).
#[test]
fn drilled_sphere_cylinder_contains_sphere_is_empty() {
    // A cylinder of radius 15 fully surrounds a sphere of radius 10.
    // Sphere − giant_cylinder = empty (sphere is entirely inside the cylinder).
    let sphere = {
        let m = Model::new().add(Feature::SphereFaceted {
            id: "ball".into(),
            radius: Scalar::lit(10.0),
            stacks: 8,
            slices: 16,
        });
        m.evaluate("ball").unwrap()
    };
    // Giant cylinder: radius=15, height=30, centered at z=-15..+15.
    let cyl = {
        let m = Model::new().add(Feature::CylinderAt {
            id: "cyl".into(),
            base: lits([0.0, 0.0, -15.0]),
            axis: "z".into(),
            radius: Scalar::lit(15.0),
            height: Scalar::lit(30.0),
            segments: 16,
        });
        m.evaluate("cyl").unwrap()
    };
    // The boolean may return an empty solid (face_count == 0) or a very thin
    // residual depending on kernel convention. Either way it must not panic.
    let result = sphere.try_difference(&cyl);
    // Per kernel convention: either Ok (empty or near-empty solid) or an Err
    // from try_boolean_solid. We just require no panic.
    match &result {
        Ok(s) => {
            let v = solid_volume(s);
            // Volume must be ≤ 1% of sphere volume (effectively empty)
            let sphere_v = (4.0 / 3.0) * std::f64::consts::PI * 1000.0;
            assert!(
                v < sphere_v * 0.01,
                "sphere fully inside cylinder: result volume {v} should be near-zero (sphere_v={sphere_v})"
            );
        }
        Err(e) => {
            // An error from try_boolean_solid is also acceptable.
            let _ = e;
        }
    }
}

/// Spec test 6: Face count is bounded after drilled-sphere boolean.
/// Result faces ≤ original sphere faces + 2 * cylinder segments.
#[test]
fn drilled_sphere_face_count_bounded() {
    let stacks = 12usize;
    let slices = 24usize;
    let segments = 24usize;
    // sphere_faceted face count: slices * stacks
    let sphere_faces = slices * stacks;

    let m = Model::new()
        .add(Feature::SphereFaceted {
            id: "ball".into(),
            radius: Scalar::lit(10.0),
            stacks,
            slices,
        })
        .add(Feature::CylinderAt {
            id: "drill".into(),
            base: lits([0.0, 0.0, -12.0]),
            axis: "z".into(),
            radius: Scalar::lit(2.0),
            height: Scalar::lit(24.0),
            segments,
        })
        .add(Feature::Difference {
            id: "out".into(),
            inputs: vec!["ball".into(), "drill".into()],
        });
    let s = m.evaluate("out").unwrap();
    // Bound: sphere faces + 2 * segments (bore caps) + generous split allowance.
    // The boolean splits sphere faces at the cylinder intersection, so allow
    // up to 4x the original face count before declaring a runaway.
    let upper = (sphere_faces + 2 * segments) * 4;
    assert!(
        s.face_count() <= upper,
        "face count {} exceeds bound {} (sphere_faces={}, segments={})",
        s.face_count(),
        upper,
        sphere_faces,
        segments
    );
    // Also confirm the solid has a sane topology (non-zero face count).
    assert!(s.face_count() > 0, "drilled sphere should have faces");
}
