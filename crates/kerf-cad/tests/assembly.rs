//! Step ASSEMBLY — multi-body data model + simple mates.
//!
//! Covers:
//! - Coincident mate translates instance_b so two points line up.
//! - Concentric mate aligns two cylinders' axes (rotation + translation).
//! - Distance mate places instance_b at a fixed distance from a point on A.
//! - Multi-instance: assembly with three boxes and overlapping mates.
//! - JSON round-trip preserves assembly + reproduces solved poses.
//! - Over-constrained mates produce a typed error carrying the mate index.

use kerf_brep::solid_volume;
use kerf_cad::{
    apply_pose_to_solid, lits, Assembly, AssemblyError, AssemblyRef, AxisRef, Feature, Instance,
    Mate, MateError, Model, Pose, ResolvedPose, Scalar,
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

fn unit_cylinder_model(segments: usize) -> Model {
    Model::new().add(Feature::Cylinder {
        id: "body".into(),
        radius: Scalar::lit(0.5),
        height: Scalar::lit(2.0),
        segments,
    })
}

#[test]
fn assembly_two_boxes_coincident() {
    // Base at origin, lid initially elsewhere; mate forces lid's bottom-center
    // to sit on top-center of base.
    let asm = Assembly::new()
        .with_instance(Instance {
            id: "base".into(),
            model: AssemblyRef::Inline(Box::new(unit_cube_model())),
            target: None,
            default_pose: Pose::identity(),
        

            pinned: false,})
        .with_instance(Instance {
            id: "lid".into(),
            model: AssemblyRef::Inline(Box::new(unit_cube_model())),
            target: None,
            default_pose: Pose::at(10.0, -3.0, 7.5),
        

            pinned: false,})
        .with_mate(Mate::Coincident {
            instance_a: "base".into(),
            point_a: lits([0.5, 0.5, 1.0]), // top center of base
            instance_b: "lid".into(),
            point_b: lits([0.5, 0.5, 0.0]), // bottom center of lid (in lid's local frame)
        });

    let poses = asm.solve_poses().expect("solve");

    let lid = poses.get("lid").unwrap();
    // Lid's local (0.5, 0.5, 0) in world = lid translation + (0.5, 0.5, 0)
    let want = kerf_geom::Vec3::new(0.0, 0.0, 1.0);
    assert!(
        approx_v(lid.translation, want, 1e-12),
        "lid translation = {:?}, want {:?}",
        lid.translation,
        want
    );
    // Base stays at default identity.
    let base = poses.get("base").unwrap();
    assert!(approx_v(base.translation, kerf_geom::Vec3::zeros(), 1e-12));
}

#[test]
fn assembly_concentric_cylinders() {
    // Two cylinders. Base cylinder is along world +z at the origin. Lid
    // cylinder is initially rotated 90° about +y so its axis points along
    // +x. Concentric mate should rotate it back so it points along +z and
    // moves it onto the same world line.
    let asm = Assembly::new()
        .with_instance(Instance {
            id: "base".into(),
            model: AssemblyRef::Inline(Box::new(unit_cylinder_model(16))),
            target: None,
            default_pose: Pose::identity(),
        

            pinned: false,})
        .with_instance(Instance {
            id: "lid".into(),
            model: AssemblyRef::Inline(Box::new(unit_cylinder_model(16))),
            target: None,
            default_pose: Pose {
                translation: lits([5.0, 4.0, 3.0]),
                rotation_axis: lits([0.0, 1.0, 0.0]),
                rotation_angle: Scalar::lit(std::f64::consts::FRAC_PI_2),
            },
        

            pinned: false,})
        .with_mate(Mate::Concentric {
            instance_a: "base".into(),
            axis_a: AxisRef {
                origin: lits([0.0, 0.0, 0.0]),
                direction: lits([0.0, 0.0, 1.0]),
            },
            instance_b: "lid".into(),
            axis_b: AxisRef {
                origin: lits([0.0, 0.0, 0.0]),
                direction: lits([0.0, 0.0, 1.0]),
            },
        });

    let poses = asm.solve_poses().expect("solve");
    let lid = poses.get("lid").expect("lid pose");

    // After solving, the lid's local +z axis transformed should be along world +z.
    let world_dir = lid.apply_vector(kerf_geom::Vec3::z()).normalize();
    assert!(
        approx_v(world_dir, kerf_geom::Vec3::z(), 1e-9),
        "lid axis dir in world = {:?}, want +z",
        world_dir
    );
    // Lid's local (0,0,0) in world should lie on the line x=0, y=0.
    let world_origin = lid.apply_point(kerf_geom::Vec3::zeros());
    assert!(
        world_origin.x.abs() < 1e-9 && world_origin.y.abs() < 1e-9,
        "lid origin in world = {:?}, want on z-axis",
        world_origin
    );
}

#[test]
fn assembly_distance_mate() {
    // Two boxes; place lid 5.0 units from base's corner along the
    // (initial) approach direction.
    let asm = Assembly::new()
        .with_instance(Instance {
            id: "base".into(),
            model: AssemblyRef::Inline(Box::new(unit_cube_model())),
            target: None,
            default_pose: Pose::identity(),
        

            pinned: false,})
        .with_instance(Instance {
            id: "satellite".into(),
            model: AssemblyRef::Inline(Box::new(unit_cube_model())),
            target: None,
            // Initially placed 2.0 along +x of base's corner.
            default_pose: Pose::at(2.0, 0.0, 0.0),
        

            pinned: false,})
        .with_mate(Mate::Distance {
            instance_a: "base".into(),
            point_a: lits([0.0, 0.0, 0.0]),
            instance_b: "satellite".into(),
            point_b: lits([0.0, 0.0, 0.0]),
            value: Scalar::lit(5.0),
        });

    let poses = asm.solve_poses().expect("solve");
    let s = poses.get("satellite").unwrap();
    // The satellite was at (2,0,0) and the direction A->B was +x; the
    // solver moves it to (5,0,0).
    assert!(
        approx_v(s.translation, kerf_geom::Vec3::new(5.0, 0.0, 0.0), 1e-12),
        "satellite translation = {:?}, want (5,0,0)",
        s.translation
    );
    // Verify distance is 5.0.
    let d = (s.translation - kerf_geom::Vec3::zeros()).norm();
    assert!((d - 5.0).abs() < 1e-12);
}

#[test]
fn assembly_round_trip_json() {
    let asm = Assembly::new()
        .with_parameter("offset", 1.5)
        .with_instance(Instance {
            id: "base".into(),
            model: AssemblyRef::Inline(Box::new(unit_cube_model())),
            target: None,
            default_pose: Pose::identity(),
        

            pinned: false,})
        .with_instance(Instance {
            id: "lid".into(),
            model: AssemblyRef::Inline(Box::new(unit_cube_model())),
            target: None,
            default_pose: Pose {
                translation: [Scalar::expr("$offset"), Scalar::lit(0.0), Scalar::lit(2.0)],
                rotation_axis: lits([0.0, 0.0, 1.0]),
                rotation_angle: Scalar::lit(0.0),
            },
        

            pinned: false,})
        .with_mate(Mate::Coincident {
            instance_a: "base".into(),
            point_a: lits([0.5, 0.5, 1.0]),
            instance_b: "lid".into(),
            point_b: lits([0.5, 0.5, 0.0]),
        });

    let json = asm.to_json_string().expect("to_json");
    let asm2 = Assembly::from_json_str(&json).expect("from_json");

    let p1 = asm.solve_poses().expect("solve original");
    let p2 = asm2.solve_poses().expect("solve roundtripped");

    for id in ["base", "lid"] {
        let a = p1.get(id).unwrap();
        let b = p2.get(id).unwrap();
        assert!(
            approx_v(a.translation, b.translation, 1e-12),
            "{id} translations differ: {:?} vs {:?}",
            a.translation,
            b.translation
        );
    }

    // Evaluate both — both should produce the same number of solids of the
    // same volume.
    let v1 = asm.evaluate().expect("eval");
    let v2 = asm2.evaluate().expect("eval2");
    assert_eq!(v1.len(), v2.len());
    for ((id1, s1), (id2, s2)) in v1.iter().zip(v2.iter()) {
        assert_eq!(id1, id2);
        let vol1 = solid_volume(s1);
        let vol2 = solid_volume(s2);
        assert!(
            (vol1 - vol2).abs() < 1e-9,
            "instance {id1} volume differs: {vol1} vs {vol2}"
        );
        // Each unit cube has volume 1.
        assert!((vol1 - 1.0).abs() < 1e-9);
    }
}

#[test]
fn assembly_overconstrained_rejects() {
    // Two mates pin the same instance to two contradictory positions.
    let asm = Assembly::new()
        .with_instance(Instance {
            id: "base".into(),
            model: AssemblyRef::Inline(Box::new(unit_cube_model())),
            target: None,
            default_pose: Pose::identity(),
        

            pinned: false,})
        .with_instance(Instance {
            id: "lid".into(),
            model: AssemblyRef::Inline(Box::new(unit_cube_model())),
            target: None,
            default_pose: Pose::at(10.0, 10.0, 10.0),
        

            pinned: false,})
        // First mate: lid's local (0,0,0) should equal base's local (0,0,1).
        .with_mate(Mate::Coincident {
            instance_a: "base".into(),
            point_a: lits([0.0, 0.0, 1.0]),
            instance_b: "lid".into(),
            point_b: lits([0.0, 0.0, 0.0]),
        })
        // Second mate (contradiction): lid's local (0,0,0) should also
        // equal base's local (5,5,5). Can't be both.
        .with_mate(Mate::Coincident {
            instance_a: "base".into(),
            point_a: lits([5.0, 5.0, 5.0]),
            instance_b: "lid".into(),
            point_b: lits([0.0, 0.0, 0.0]),
        });

    let err = asm.solve_poses().expect_err("should be over-constrained");
    match err {
        AssemblyError::Mate(MateError::OverConstrained(idx, _msg)) => {
            assert_eq!(idx, 1, "expected mate 1 to fail");
        }
        other => panic!("expected OverConstrained, got {other:?}"),
    }
}

#[test]
fn assembly_evaluate_produces_posed_solids() {
    // Two boxes with a coincident mate. After evaluate, solids are
    // returned in the resolved-pose world frame. Verify that the lid's
    // bounding-box minimum z is 1.0 (it sits on top of the unit base).
    let asm = Assembly::new()
        .with_instance(Instance {
            id: "base".into(),
            model: AssemblyRef::Inline(Box::new(unit_cube_model())),
            target: None,
            default_pose: Pose::identity(),
        

            pinned: false,})
        .with_instance(Instance {
            id: "lid".into(),
            model: AssemblyRef::Inline(Box::new(unit_cube_model())),
            target: None,
            default_pose: Pose::at(0.0, 0.0, 10.0),
        

            pinned: false,})
        .with_mate(Mate::Coincident {
            instance_a: "base".into(),
            point_a: lits([0.0, 0.0, 1.0]),
            instance_b: "lid".into(),
            point_b: lits([0.0, 0.0, 0.0]),
        });

    let solids = asm.evaluate().expect("evaluate");
    assert_eq!(solids.len(), 2);

    let (lid_id, lid_solid) = &solids[1];
    assert_eq!(lid_id, "lid");

    // Walk every vertex of the lid solid; its z-min should be 1.0,
    // its z-max should be 2.0.
    let mut zmin = f64::INFINITY;
    let mut zmax = f64::NEG_INFINITY;
    for (_, p) in lid_solid.vertex_geom.iter() {
        zmin = zmin.min(p.z);
        zmax = zmax.max(p.z);
    }
    assert!((zmin - 1.0).abs() < 1e-9, "zmin = {zmin}, want 1.0");
    assert!((zmax - 2.0).abs() < 1e-9, "zmax = {zmax}, want 2.0");
}

#[test]
fn assembly_default_pose_only() {
    // No mates — every instance just sits at its default pose.
    let asm = Assembly::new()
        .with_instance(Instance {
            id: "a".into(),
            model: AssemblyRef::Inline(Box::new(unit_cube_model())),
            target: None,
            default_pose: Pose::at(1.0, 2.0, 3.0),
        

            pinned: false,})
        .with_instance(Instance {
            id: "b".into(),
            model: AssemblyRef::Inline(Box::new(unit_cube_model())),
            target: None,
            default_pose: Pose::at(-5.0, 0.0, 0.0),
        

            pinned: false,});

    let poses = asm.solve_poses().expect("solve");
    assert_eq!(poses.len(), 2);
    let a = poses.get("a").unwrap();
    let b = poses.get("b").unwrap();
    assert!(approx_v(a.translation, kerf_geom::Vec3::new(1.0, 2.0, 3.0), 1e-12));
    assert!(approx_v(b.translation, kerf_geom::Vec3::new(-5.0, 0.0, 0.0), 1e-12));
}

#[test]
fn assembly_three_instances_chained_coincident() {
    // base at origin; mid coincident with top of base; tip coincident with
    // top of mid. Each is a unit cube. Tip's translation should be (0,0,2).
    let asm = Assembly::new()
        .with_instance(Instance {
            id: "base".into(),
            model: AssemblyRef::Inline(Box::new(unit_cube_model())),
            target: None,
            default_pose: Pose::identity(),
        

            pinned: false,})
        .with_instance(Instance {
            id: "mid".into(),
            model: AssemblyRef::Inline(Box::new(unit_cube_model())),
            target: None,
            default_pose: Pose::at(50.0, 50.0, 50.0),
        

            pinned: false,})
        .with_instance(Instance {
            id: "tip".into(),
            model: AssemblyRef::Inline(Box::new(unit_cube_model())),
            target: None,
            default_pose: Pose::at(-50.0, -50.0, -50.0),
        

            pinned: false,})
        .with_mate(Mate::Coincident {
            instance_a: "base".into(),
            point_a: lits([0.0, 0.0, 1.0]),
            instance_b: "mid".into(),
            point_b: lits([0.0, 0.0, 0.0]),
        })
        .with_mate(Mate::Coincident {
            instance_a: "mid".into(),
            point_a: lits([0.0, 0.0, 1.0]),
            instance_b: "tip".into(),
            point_b: lits([0.0, 0.0, 0.0]),
        });

    let poses = asm.solve_poses().expect("solve");
    let mid = poses.get("mid").unwrap();
    let tip = poses.get("tip").unwrap();
    assert!(approx_v(mid.translation, kerf_geom::Vec3::new(0.0, 0.0, 1.0), 1e-12));
    assert!(approx_v(tip.translation, kerf_geom::Vec3::new(0.0, 0.0, 2.0), 1e-12));
}

#[test]
fn assembly_apply_pose_helper_translates_solid() {
    // Sanity-check the apply_pose_to_solid helper directly.
    let m = unit_cube_model();
    let s = m.evaluate("body").expect("eval");
    let pose = ResolvedPose {
        translation: kerf_geom::Vec3::new(10.0, 20.0, 30.0),
        rotation_axis: kerf_geom::Vec3::z(),
        rotation_angle: 0.0,
    };
    let posed = apply_pose_to_solid(&s, pose);
    let mut zmin = f64::INFINITY;
    for (_, p) in posed.vertex_geom.iter() {
        zmin = zmin.min(p.z);
    }
    assert!((zmin - 30.0).abs() < 1e-9);
    // Volume is preserved.
    let v0 = solid_volume(&s);
    let v1 = solid_volume(&posed);
    assert!((v0 - v1).abs() < 1e-9);
}

#[test]
fn assembly_unknown_instance_in_mate_errors() {
    let asm = Assembly::new()
        .with_instance(Instance {
            id: "base".into(),
            model: AssemblyRef::Inline(Box::new(unit_cube_model())),
            target: None,
            default_pose: Pose::identity(),
        

            pinned: false,})
        .with_mate(Mate::Coincident {
            instance_a: "base".into(),
            point_a: lits([0.0, 0.0, 0.0]),
            instance_b: "ghost".into(),
            point_b: lits([0.0, 0.0, 0.0]),
        });

    let err = asm.solve_poses().expect_err("should error");
    match err {
        AssemblyError::Mate(MateError::UnknownInstance(name, idx)) => {
            assert_eq!(name, "ghost");
            assert_eq!(idx, 0);
        }
        other => panic!("expected UnknownInstance, got {other:?}"),
    }
}

#[test]
fn assembly_distance_already_satisfied_passes() {
    // A second (redundant but consistent) Distance mate after a first one
    // should not be over-constrained.
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
            default_pose: Pose::at(2.0, 0.0, 0.0),
        

            pinned: false,})
        .with_mate(Mate::Distance {
            instance_a: "a".into(),
            point_a: lits([0.0, 0.0, 0.0]),
            instance_b: "b".into(),
            point_b: lits([0.0, 0.0, 0.0]),
            value: Scalar::lit(5.0),
        })
        // Same distance — already satisfied — should pass.
        .with_mate(Mate::Distance {
            instance_a: "a".into(),
            point_a: lits([0.0, 0.0, 0.0]),
            instance_b: "b".into(),
            point_b: lits([0.0, 0.0, 0.0]),
            value: Scalar::lit(5.0),
        });

    let poses = asm.solve_poses().expect("solve");
    let b = poses.get("b").unwrap();
    assert!((b.translation.norm() - 5.0).abs() < 1e-12);
}
