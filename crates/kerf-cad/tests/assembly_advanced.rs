//! Step ASSEMBLY-3 — advanced mates + sub-assembly composition + AABB
//! interference detection.
//!
//! Adds:
//! - `Mate::Symmetry` — instance_b is the mirror image of instance_a.
//! - `Mate::Width` — distance from B's centroid to A's axis line.
//! - `Mate::PathMate` — instance follows a polyline at parameter t.
//! - `Mate::Lock` — fully constrains B to A.
//! - `AssemblyRef::Path(...)` resolved via `Assembly::evaluate_with_loader`.
//! - `Assembly::detect_interference` — pairs whose AABBs overlap, with
//!   overlap volume.
//!
//! These tests sit alongside `tests/assembly.rs` (PR #7) and
//! `tests/assembly_completion.rs` (PR #17).

use std::collections::HashMap;

use kerf_cad::{
    lits, Assembly, AssemblyError, AssemblyRef, AxisRef, Feature, Instance, Mate, Model, Pose,
    Scalar,
};

fn approx_v(a: kerf_geom::Vec3, b: kerf_geom::Vec3, eps: f64) -> bool {
    (a - b).norm() <= eps
}

fn unit_cube_model() -> Model {
    Model::new().add(Feature::Box {
        id: "body".into(),
        extents: lits([1.0, 1.0, 1.0]),
    })
}

// ---------------------------------------------------------------------------
// Symmetry
// ---------------------------------------------------------------------------

#[test]
fn symmetry_mate_mirrors_lid() {
    // Base ("lid") is at world (3, 0, 5). Mirror plane is the y-z plane
    // (x=0) with normal +x, defined in instance "frame" (which is at
    // identity, so the plane is in world space). The mirrored instance
    // should sit at (-3, 0, 5).
    let asm = Assembly::new()
        .with_instance(Instance {
            id: "frame".into(),
            model: AssemblyRef::Inline(Box::new(unit_cube_model())),
            target: None,
            default_pose: Pose::identity(),
        

            pinned: false,})
        .with_instance(Instance {
            id: "lid_left".into(),
            model: AssemblyRef::Inline(Box::new(unit_cube_model())),
            target: None,
            default_pose: Pose::at(3.0, 0.0, 5.0),
        

            pinned: false,})
        .with_instance(Instance {
            id: "lid_right".into(),
            model: AssemblyRef::Inline(Box::new(unit_cube_model())),
            target: None,
            default_pose: Pose::at(99.0, 99.0, 99.0), // garbage, will be overwritten
        

            pinned: false,})
        .with_mate(Mate::Symmetry {
            instance_a: "lid_left".into(),
            plane_origin: lits([0.0, 0.0, 0.0]),
            plane_normal: lits([1.0, 0.0, 0.0]),
            instance_b: "lid_right".into(),
        });

    let poses = asm.solve_poses().expect("solve");
    let right = poses.get("lid_right").unwrap();
    assert!(
        approx_v(right.translation, kerf_geom::Vec3::new(-3.0, 0.0, 5.0), 1e-9),
        "lid_right translation = {:?}, want (-3, 0, 5)",
        right.translation
    );
}

#[test]
fn symmetry_round_trip_json() {
    let asm = Assembly::new()
        .with_instance(Instance {
            id: "frame".into(),
            model: AssemblyRef::Inline(Box::new(unit_cube_model())),
            target: None,
            default_pose: Pose::identity(),
        

            pinned: false,})
        .with_instance(Instance {
            id: "left".into(),
            model: AssemblyRef::Inline(Box::new(unit_cube_model())),
            target: None,
            default_pose: Pose::at(2.5, 1.0, 0.0),
        

            pinned: false,})
        .with_instance(Instance {
            id: "right".into(),
            model: AssemblyRef::Inline(Box::new(unit_cube_model())),
            target: None,
            default_pose: Pose::identity(),
        

            pinned: false,})
        .with_mate(Mate::Symmetry {
            instance_a: "left".into(),
            plane_origin: lits([0.0, 0.0, 0.0]),
            plane_normal: lits([1.0, 0.0, 0.0]),
            instance_b: "right".into(),
        });

    let json = asm.to_json_string().expect("to_json");
    let asm2 = Assembly::from_json_str(&json).expect("from_json");

    let p1 = asm.solve_poses().expect("solve");
    let p2 = asm2.solve_poses().expect("solve2");

    for id in ["left", "right", "frame"] {
        assert!(
            approx_v(
                p1.get(id).unwrap().translation,
                p2.get(id).unwrap().translation,
                1e-12,
            ),
            "translation mismatch for instance '{id}' after round-trip",
        );
    }

    // Specifically, "right" should be at (-2.5, 1.0, 0.0).
    let r = p2.get("right").unwrap();
    assert!(approx_v(
        r.translation,
        kerf_geom::Vec3::new(-2.5, 1.0, 0.0),
        1e-9,
    ));
}

// ---------------------------------------------------------------------------
// Width
// ---------------------------------------------------------------------------

#[test]
fn width_mate_distance_validates() {
    // A's axis is the world z-axis. B's centroid starts at (5, 0, 7).
    // Width mate sets B's distance from the axis to 2.0. Expected: B's
    // centroid lands at (2, 0, 7) (along the +x radial direction).
    let asm = Assembly::new()
        .with_instance(Instance {
            id: "shaft".into(),
            model: AssemblyRef::Inline(Box::new(unit_cube_model())),
            target: None,
            default_pose: Pose::identity(),
        

            pinned: false,})
        .with_instance(Instance {
            id: "satellite".into(),
            model: AssemblyRef::Inline(Box::new(unit_cube_model())),
            target: None,
            default_pose: Pose::at(5.0, 0.0, 7.0),
        

            pinned: false,})
        .with_mate(Mate::Width {
            instance_a: "shaft".into(),
            axis_a: AxisRef {
                origin: lits([0.0, 0.0, 0.0]),
                direction: lits([0.0, 0.0, 1.0]),
            },
            instance_b: "satellite".into(),
            distance: Scalar::lit(2.0),
        });

    let poses = asm.solve_poses().expect("solve");
    let s = poses.get("satellite").unwrap();
    assert!(
        approx_v(s.translation, kerf_geom::Vec3::new(2.0, 0.0, 7.0), 1e-9),
        "satellite translation = {:?}, want (2, 0, 7)",
        s.translation
    );

    // Verify perpendicular distance from satellite's translation to the
    // z-axis is exactly 2.0.
    let perp_dist = (s.translation.x.powi(2) + s.translation.y.powi(2)).sqrt();
    assert!((perp_dist - 2.0).abs() < 1e-9, "perp_dist = {perp_dist}");
}

#[test]
fn width_mate_negative_distance_rejected() {
    let asm = Assembly::new()
        .with_instance(Instance {
            id: "a".into(),
            model: AssemblyRef::Inline(Box::new(unit_cube_model())),
            target: None,
            default_pose: Pose::identity(),
        

            pinned: false,})
        .with_instance(Instance {
            id: "b".into(),
            model: AssemblyRef::Inline(Box::new(unit_cube_model())),
            target: None,
            default_pose: Pose::at(1.0, 0.0, 0.0),
        

            pinned: false,})
        .with_mate(Mate::Width {
            instance_a: "a".into(),
            axis_a: AxisRef {
                origin: lits([0.0, 0.0, 0.0]),
                direction: lits([0.0, 0.0, 1.0]),
            },
            instance_b: "b".into(),
            distance: Scalar::lit(-0.5),
        });

    let err = asm.solve_poses().expect_err("negative distance must be rejected");
    match err {
        AssemblyError::Mate(kerf_cad::MateError::Invalid(_, _)) => {}
        other => panic!("expected Invalid, got {other:?}"),
    }
}

// ---------------------------------------------------------------------------
// PathMate
// ---------------------------------------------------------------------------

#[test]
fn path_mate_at_t_zero_at_start() {
    // Polyline (0,0,0) → (10,0,0) → (10,10,0). At t=0, instance lands
    // exactly on the first waypoint regardless of starting pose.
    let asm = Assembly::new()
        .with_instance(Instance {
            id: "follower".into(),
            model: AssemblyRef::Inline(Box::new(unit_cube_model())),
            target: None,
            default_pose: Pose::at(99.0, 99.0, 99.0),
        

            pinned: false,})
        .with_mate(Mate::PathMate {
            instance: "follower".into(),
            path: vec![
                lits([0.0, 0.0, 0.0]),
                lits([10.0, 0.0, 0.0]),
                lits([10.0, 10.0, 0.0]),
            ],
            parameter: Scalar::lit(0.0),
        });

    let poses = asm.solve_poses().expect("solve");
    let f = poses.get("follower").unwrap();
    assert!(approx_v(f.translation, kerf_geom::Vec3::zeros(), 1e-12));
}

#[test]
fn path_mate_at_t_one_at_end() {
    let asm = Assembly::new()
        .with_instance(Instance {
            id: "follower".into(),
            model: AssemblyRef::Inline(Box::new(unit_cube_model())),
            target: None,
            default_pose: Pose::identity(),
        

            pinned: false,})
        .with_mate(Mate::PathMate {
            instance: "follower".into(),
            path: vec![
                lits([0.0, 0.0, 0.0]),
                lits([10.0, 0.0, 0.0]),
                lits([10.0, 10.0, 0.0]),
            ],
            parameter: Scalar::lit(1.0),
        });

    let poses = asm.solve_poses().expect("solve");
    let f = poses.get("follower").unwrap();
    assert!(approx_v(
        f.translation,
        kerf_geom::Vec3::new(10.0, 10.0, 0.0),
        1e-12,
    ));
}

#[test]
fn path_mate_at_t_half_arc_length() {
    // Polyline length = 10 + 10 = 20. t = 0.5 → arc 10 → exactly at the
    // first elbow (10, 0, 0).
    let asm = Assembly::new()
        .with_instance(Instance {
            id: "follower".into(),
            model: AssemblyRef::Inline(Box::new(unit_cube_model())),
            target: None,
            default_pose: Pose::identity(),
        

            pinned: false,})
        .with_mate(Mate::PathMate {
            instance: "follower".into(),
            path: vec![
                lits([0.0, 0.0, 0.0]),
                lits([10.0, 0.0, 0.0]),
                lits([10.0, 10.0, 0.0]),
            ],
            parameter: Scalar::lit(0.5),
        });

    let poses = asm.solve_poses().expect("solve");
    let f = poses.get("follower").unwrap();
    assert!(approx_v(
        f.translation,
        kerf_geom::Vec3::new(10.0, 0.0, 0.0),
        1e-9,
    ));
}

#[test]
fn path_mate_out_of_range_rejected() {
    let asm = Assembly::new()
        .with_instance(Instance {
            id: "f".into(),
            model: AssemblyRef::Inline(Box::new(unit_cube_model())),
            target: None,
            default_pose: Pose::identity(),
        

            pinned: false,})
        .with_mate(Mate::PathMate {
            instance: "f".into(),
            path: vec![lits([0.0, 0.0, 0.0]), lits([1.0, 0.0, 0.0])],
            parameter: Scalar::lit(1.5),
        });

    let err = asm.solve_poses().expect_err("t > 1 must be rejected");
    match err {
        AssemblyError::Mate(kerf_cad::MateError::Invalid(_, _)) => {}
        other => panic!("expected Invalid, got {other:?}"),
    }
}

// ---------------------------------------------------------------------------
// Lock
// ---------------------------------------------------------------------------

#[test]
fn lock_mate_freezes_instance_b() {
    // A is at (4, 5, 6) with no rotation. Lock mate forces B to match
    // A's pose exactly, regardless of its default.
    let asm = Assembly::new()
        .with_instance(Instance {
            id: "a".into(),
            model: AssemblyRef::Inline(Box::new(unit_cube_model())),
            target: None,
            default_pose: Pose::at(4.0, 5.0, 6.0),
        

            pinned: false,})
        .with_instance(Instance {
            id: "b".into(),
            model: AssemblyRef::Inline(Box::new(unit_cube_model())),
            target: None,
            default_pose: Pose::at(99.0, -42.0, 13.0),
        

            pinned: false,})
        .with_mate(Mate::Lock {
            instance_a: "a".into(),
            instance_b: "b".into(),
        });

    let poses = asm.solve_poses().expect("solve");
    let a = poses.get("a").unwrap();
    let b = poses.get("b").unwrap();

    assert!(
        approx_v(a.translation, b.translation, 1e-12),
        "a={:?}, b={:?}",
        a.translation,
        b.translation,
    );
    assert!((a.rotation_angle - b.rotation_angle).abs() < 1e-12);
}

#[test]
fn lock_mate_with_rotated_a_propagates_rotation() {
    // A has rotation π/3 about +y; B is at identity. After lock, B
    // must match A's rotation.
    let asm = Assembly::new()
        .with_instance(Instance {
            id: "a".into(),
            model: AssemblyRef::Inline(Box::new(unit_cube_model())),
            target: None,
            default_pose: Pose {
                translation: lits([1.0, 2.0, 3.0]),
                rotation_axis: lits([0.0, 1.0, 0.0]),
                rotation_angle: Scalar::lit(std::f64::consts::FRAC_PI_3),
            },
        

            pinned: false,})
        .with_instance(Instance {
            id: "b".into(),
            model: AssemblyRef::Inline(Box::new(unit_cube_model())),
            target: None,
            default_pose: Pose::identity(),
        

            pinned: false,})
        .with_mate(Mate::Lock {
            instance_a: "a".into(),
            instance_b: "b".into(),
        });

    let poses = asm.solve_poses().expect("solve");
    let b = poses.get("b").unwrap();
    // Apply B's pose to local +x; it should match A's rotated +x.
    let bx = b.apply_vector(kerf_geom::Vec3::x()).normalize();
    let want_x = kerf_geom::Vec3::new(
        std::f64::consts::FRAC_PI_3.cos(),
        0.0,
        -std::f64::consts::FRAC_PI_3.sin(),
    );
    assert!(approx_v(bx, want_x, 1e-9), "bx = {:?}, want {:?}", bx, want_x);
}

// ---------------------------------------------------------------------------
// Sub-assembly composition via loader
// ---------------------------------------------------------------------------

#[test]
fn sub_assembly_loads_from_loader() {
    // Build a sub-assembly with one instance of a unit cube, serialise
    // it to JSON, then build a top-level assembly that references the
    // sub-assembly through `AssemblyRef::Path`. The loader returns the
    // parsed sub-assembly. After evaluation, the top-level instance
    // should contain a single posed solid with the cube's geometry,
    // shifted by the top-level instance's pose.
    let sub = Assembly::new().with_instance(Instance {
        id: "core".into(),
        model: AssemblyRef::Inline(Box::new(unit_cube_model())),
        target: None,
        default_pose: Pose::identity(),
    

        pinned: false,});
    let sub_json = sub.to_json_string().expect("to_json");

    // The fixture: a HashMap<String, String> from "filename" → JSON.
    let mut fs: HashMap<String, String> = HashMap::new();
    fs.insert("sub.json".into(), sub_json);

    let top = Assembly::new().with_instance(Instance {
        id: "outer".into(),
        model: AssemblyRef::Path("sub.json".into()),
        target: None,
        default_pose: Pose::at(10.0, 0.0, 0.0),
    

        pinned: false,});

    // Evaluate without loader: should fail with UnresolvedRef.
    let err = top.evaluate().expect_err("no-loader evaluate must fail");
    match err {
        AssemblyError::UnresolvedRef(name) => assert_eq!(name, "outer"),
        other => panic!("expected UnresolvedRef, got {other:?}"),
    }

    // Evaluate WITH loader: should succeed.
    let loader = |path: &str| -> Result<Assembly, AssemblyError> {
        match fs.get(path) {
            Some(j) => Assembly::from_json_str(j),
            None => Err(AssemblyError::UnresolvedRef(format!("not found: {path}"))),
        }
    };
    let solids = top.evaluate_with_loader(loader).expect("evaluate");
    assert_eq!(solids.len(), 1);
    assert_eq!(solids[0].0, "outer");

    // The returned posed cube should have all its vertices' x in
    // [10, 11] (cube is unit, base at origin, top-level pose translates
    // by +10 along x).
    let s = &solids[0].1;
    let mut min_x = f64::INFINITY;
    let mut max_x = f64::NEG_INFINITY;
    for vid in s.topo.vertex_ids() {
        if let Some(p) = s.vertex_geom.get(vid) {
            min_x = min_x.min(p.x);
            max_x = max_x.max(p.x);
        }
    }
    assert!(
        (min_x - 10.0).abs() < 1e-9 && (max_x - 11.0).abs() < 1e-9,
        "x range = [{min_x}, {max_x}], want [10, 11]",
    );
}

#[test]
fn sub_assembly_loader_error_propagates() {
    // Loader explicitly returns a UnresolvedRef; the top-level
    // evaluate_with_loader must surface it (rebadged as the containing
    // instance's id).
    let top = Assembly::new().with_instance(Instance {
        id: "missing_outer".into(),
        model: AssemblyRef::Path("nope.json".into()),
        target: None,
        default_pose: Pose::identity(),
    

        pinned: false,});
    let loader =
        |_path: &str| -> Result<Assembly, AssemblyError> { Err(AssemblyError::UnresolvedRef("nope.json".into())) };
    let err = top.evaluate_with_loader(loader).expect_err("must fail");
    match err {
        AssemblyError::UnresolvedRef(id) => assert_eq!(id, "missing_outer"),
        other => panic!("expected UnresolvedRef, got {other:?}"),
    }
}

// ---------------------------------------------------------------------------
// Interference detection
// ---------------------------------------------------------------------------

#[test]
fn interference_detection_finds_overlapping_aabb() {
    // Two unit cubes both at the origin → AABBs identical → overlap
    // volume = 1.0.
    let asm = Assembly::new()
        .with_instance(Instance {
            id: "alpha".into(),
            model: AssemblyRef::Inline(Box::new(unit_cube_model())),
            target: None,
            default_pose: Pose::identity(),
        

            pinned: false,})
        .with_instance(Instance {
            id: "beta".into(),
            model: AssemblyRef::Inline(Box::new(unit_cube_model())),
            target: None,
            default_pose: Pose::at(0.5, 0.0, 0.0),
        

            pinned: false,});
    // Without any mates, default poses are kept.
    let params: HashMap<String, f64> = HashMap::new();
    let pairs = asm.detect_interference(&params).expect("detect");
    assert_eq!(pairs.len(), 1);
    let (a, b, vol) = &pairs[0];
    assert_eq!(a, "alpha");
    assert_eq!(b, "beta");
    // alpha is x∈[0,1]; beta is x∈[0.5, 1.5]; overlap x∈[0.5,1]=0.5.
    // y and z both fully overlap (extent 1.0). Volume = 0.5 * 1 * 1 = 0.5.
    assert!((vol - 0.5).abs() < 1e-9, "vol = {vol}, want 0.5");
}

#[test]
fn interference_no_overlap_returns_empty() {
    // Two unit cubes spaced 5.0 apart → no AABB overlap → empty list.
    let asm = Assembly::new()
        .with_instance(Instance {
            id: "alpha".into(),
            model: AssemblyRef::Inline(Box::new(unit_cube_model())),
            target: None,
            default_pose: Pose::identity(),
        

            pinned: false,})
        .with_instance(Instance {
            id: "beta".into(),
            model: AssemblyRef::Inline(Box::new(unit_cube_model())),
            target: None,
            default_pose: Pose::at(5.0, 0.0, 0.0),
        

            pinned: false,});
    let params: HashMap<String, f64> = HashMap::new();
    let pairs = asm.detect_interference(&params).expect("detect");
    assert!(pairs.is_empty(), "expected no pairs, got {pairs:?}");
}

#[test]
fn interference_three_instances_detects_two_pairs() {
    // alpha @ (0,0,0), beta @ (0.5,0,0), gamma @ (10,0,0). alpha-beta
    // overlap; gamma is far away.
    let asm = Assembly::new()
        .with_instance(Instance {
            id: "alpha".into(),
            model: AssemblyRef::Inline(Box::new(unit_cube_model())),
            target: None,
            default_pose: Pose::identity(),
        

            pinned: false,})
        .with_instance(Instance {
            id: "beta".into(),
            model: AssemblyRef::Inline(Box::new(unit_cube_model())),
            target: None,
            default_pose: Pose::at(0.5, 0.0, 0.0),
        

            pinned: false,})
        .with_instance(Instance {
            id: "gamma".into(),
            model: AssemblyRef::Inline(Box::new(unit_cube_model())),
            target: None,
            default_pose: Pose::at(10.0, 0.0, 0.0),
        

            pinned: false,});
    let params: HashMap<String, f64> = HashMap::new();
    let pairs = asm.detect_interference(&params).expect("detect");
    assert_eq!(pairs.len(), 1);
    assert_eq!(pairs[0].0, "alpha");
    assert_eq!(pairs[0].1, "beta");
}
