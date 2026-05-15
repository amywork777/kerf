//! Step ASSEMBLY-2 — completing the mate solver.
//!
//! Adds:
//! - ParallelPlane mate (orient B's plane parallel to A's, with offset)
//! - AngleMate (set angle between two axes)
//! - TangentMate (cylinder-on-plane, sphere-on-plane, sphere-on-sphere)
//! - Cycle-aware iterative solver: A→B→C→A topologies that the simple
//!   "freeze-after-first-move" pass would mark over-constrained.
//!
//! These tests sit alongside `tests/assembly.rs` (PR #7).

use kerf_cad::{
    lits, Assembly, AssemblyError, AssemblyRef, AxisRef, Feature, Instance, Mate, MateError,
    Model, Pose, Scalar, SurfaceRef,
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
// ParallelPlane
// ---------------------------------------------------------------------------

#[test]
fn parallel_plane_mate() {
    // Base plane: z=0, normal +z. Lid plane: locally z=0 with normal +z,
    // but lid is initially rotated 90° about +y so its plane normal is
    // pointing along +x in the world. With offset 1.5, the lid's plane
    // origin should land 1.5 above base plane along +z, and the lid's
    // plane normal should point +z again.
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
            default_pose: Pose {
                translation: lits([5.0, 4.0, 3.0]),
                rotation_axis: lits([0.0, 1.0, 0.0]),
                rotation_angle: Scalar::lit(std::f64::consts::FRAC_PI_2),
            },
        

            pinned: false,})
        .with_mate(Mate::ParallelPlane {
            instance_a: "base".into(),
            plane_a_origin: lits([0.0, 0.0, 0.0]),
            plane_a_normal: lits([0.0, 0.0, 1.0]),
            instance_b: "lid".into(),
            plane_b_origin: lits([0.0, 0.0, 0.0]),
            plane_b_normal: lits([0.0, 0.0, 1.0]),
            offset: Scalar::lit(1.5),
        });

    let poses = asm.solve_poses().expect("solve");
    let lid = poses.get("lid").unwrap();

    let world_normal = lid.apply_vector(kerf_geom::Vec3::z()).normalize();
    assert!(
        approx_v(world_normal, kerf_geom::Vec3::z(), 1e-9),
        "lid normal in world = {:?}, want +z",
        world_normal
    );
    let world_origin = lid.apply_point(kerf_geom::Vec3::zeros());
    assert!(
        (world_origin.z - 1.5).abs() < 1e-9,
        "lid plane origin z = {}, want 1.5",
        world_origin.z
    );
}

#[test]
fn parallel_plane_zero_offset_is_coplanar() {
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
            default_pose: Pose::at(0.0, 0.0, 5.0),
        

            pinned: false,})
        .with_mate(Mate::ParallelPlane {
            instance_a: "a".into(),
            plane_a_origin: lits([0.0, 0.0, 0.0]),
            plane_a_normal: lits([0.0, 0.0, 1.0]),
            instance_b: "b".into(),
            plane_b_origin: lits([0.0, 0.0, 0.0]),
            plane_b_normal: lits([0.0, 0.0, 1.0]),
            offset: Scalar::lit(0.0),
        });

    let poses = asm.solve_poses().expect("solve");
    let b = poses.get("b").unwrap();
    let world_origin = b.apply_point(kerf_geom::Vec3::zeros());
    assert!(world_origin.z.abs() < 1e-9);
}

// ---------------------------------------------------------------------------
// AngleMate
// ---------------------------------------------------------------------------

#[test]
fn angle_mate_90deg() {
    // Axis A is +x. Axis B (in B's local frame) is +x; B's default pose
    // is identity, so B's axis in world is +x (parallel to A → angle 0).
    // After the 90° mate, B's axis should be perpendicular to A's, i.e.
    // somewhere in the y-z plane.
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
            default_pose: Pose::at(3.0, 0.0, 0.0),
        

            pinned: false,})
        .with_mate(Mate::AngleMate {
            instance_a: "a".into(),
            axis_a: AxisRef {
                origin: lits([0.0, 0.0, 0.0]),
                direction: lits([1.0, 0.0, 0.0]),
            },
            instance_b: "b".into(),
            axis_b: AxisRef {
                origin: lits([0.0, 0.0, 0.0]),
                direction: lits([1.0, 0.0, 0.0]),
            },
            angle: Scalar::lit(std::f64::consts::FRAC_PI_2),
        });

    let poses = asm.solve_poses().expect("solve");
    let b = poses.get("b").unwrap();
    let dir = b.apply_vector(kerf_geom::Vec3::x()).normalize();
    let dot = dir.dot(&kerf_geom::Vec3::x()).abs();
    assert!(
        dot < 1e-9,
        "B's axis dot with A's axis = {dot}, want 0 (perpendicular)"
    );
}

#[test]
fn angle_mate_45deg_with_param() {
    // Use an assembly parameter to drive the target angle. Initial B
    // axis is +x (parallel to A). Mate sets angle to π/4.
    let asm = Assembly::new()
        .with_parameter("theta", std::f64::consts::FRAC_PI_4)
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
            default_pose: Pose::at(0.0, 0.0, 0.0),
        

            pinned: false,})
        .with_mate(Mate::AngleMate {
            instance_a: "a".into(),
            axis_a: AxisRef {
                origin: lits([0.0, 0.0, 0.0]),
                direction: lits([1.0, 0.0, 0.0]),
            },
            instance_b: "b".into(),
            axis_b: AxisRef {
                origin: lits([0.0, 0.0, 0.0]),
                direction: lits([1.0, 0.0, 0.0]),
            },
            angle: Scalar::expr("$theta"),
        });

    let poses = asm.solve_poses().expect("solve");
    let b = poses.get("b").unwrap();
    let dir = b.apply_vector(kerf_geom::Vec3::x()).normalize();
    let dot = dir.dot(&kerf_geom::Vec3::x()).clamp(-1.0, 1.0);
    let actual_angle = dot.acos();
    assert!(
        (actual_angle - std::f64::consts::FRAC_PI_4).abs() < 1e-9,
        "actual angle = {actual_angle}, want π/4"
    );
}

#[test]
fn angle_mate_already_satisfied_no_op() {
    // B starts at identity; A and B both point +x. Mate is "angle 0",
    // already satisfied. Pose should be unchanged.
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
            default_pose: Pose::at(2.0, 3.0, 4.0),
        

            pinned: false,})
        .with_mate(Mate::AngleMate {
            instance_a: "a".into(),
            axis_a: AxisRef {
                origin: lits([0.0, 0.0, 0.0]),
                direction: lits([1.0, 0.0, 0.0]),
            },
            instance_b: "b".into(),
            axis_b: AxisRef {
                origin: lits([0.0, 0.0, 0.0]),
                direction: lits([1.0, 0.0, 0.0]),
            },
            angle: Scalar::lit(0.0),
        });

    let poses = asm.solve_poses().expect("solve");
    let b = poses.get("b").unwrap();
    assert!(approx_v(b.translation, kerf_geom::Vec3::new(2.0, 3.0, 4.0), 1e-12));
}

// ---------------------------------------------------------------------------
// TangentMate
// ---------------------------------------------------------------------------

#[test]
fn tangent_mate_cylinder_on_plane() {
    // Plane: z=0, normal +z. Cylinder of radius 0.5 with axis along +x
    // in B's local frame. After tangent mate, the cylinder line should
    // sit at z=0.5 with axis perpendicular to the plane normal.
    let asm = Assembly::new()
        .with_instance(Instance {
            id: "table".into(),
            model: AssemblyRef::Inline(Box::new(unit_cube_model())),
            target: None,
            default_pose: Pose::identity(),
        

            pinned: false,})
        .with_instance(Instance {
            id: "rod".into(),
            model: AssemblyRef::Inline(Box::new(unit_cube_model())),
            target: None,
            default_pose: Pose::at(2.0, 1.0, 5.0),
        

            pinned: false,})
        .with_mate(Mate::TangentMate {
            instance_a: "table".into(),
            surface_a: SurfaceRef::Plane {
                origin: lits([0.0, 0.0, 0.0]),
                normal: lits([0.0, 0.0, 1.0]),
            },
            instance_b: "rod".into(),
            surface_b: SurfaceRef::Cylinder {
                origin: lits([0.0, 0.0, 0.0]),
                axis: lits([1.0, 0.0, 0.0]),
                radius: Scalar::lit(0.5),
            },
        });

    let poses = asm.solve_poses().expect("solve");
    let rod = poses.get("rod").unwrap();
    // Cylinder's local +x axis transformed to world should be
    // perpendicular to +z (the plane's normal).
    let cax_world = rod.apply_vector(kerf_geom::Vec3::x()).normalize();
    let z_dot = cax_world.dot(&kerf_geom::Vec3::z()).abs();
    assert!(z_dot < 1e-9, "cylinder axis · plane normal = {z_dot}, want 0");
    // Cylinder origin (local 0,0,0) transformed to world should be at
    // signed distance 0.5 from the plane along +z.
    let cax_origin_world = rod.apply_point(kerf_geom::Vec3::zeros());
    assert!(
        (cax_origin_world.z - 0.5).abs() < 1e-9,
        "cylinder origin z = {}, want 0.5",
        cax_origin_world.z
    );
}

#[test]
fn tangent_mate_sphere_on_plane() {
    // Plane: z=0, normal +z. Sphere of radius 1.2 centered at lid's
    // local origin. After tangent mate, sphere center should be at
    // z = 1.2 in world.
    let asm = Assembly::new()
        .with_instance(Instance {
            id: "floor".into(),
            model: AssemblyRef::Inline(Box::new(unit_cube_model())),
            target: None,
            default_pose: Pose::identity(),
        

            pinned: false,})
        .with_instance(Instance {
            id: "ball".into(),
            model: AssemblyRef::Inline(Box::new(unit_cube_model())),
            target: None,
            default_pose: Pose::at(3.0, -2.0, 7.0),
        

            pinned: false,})
        .with_mate(Mate::TangentMate {
            instance_a: "floor".into(),
            surface_a: SurfaceRef::Plane {
                origin: lits([0.0, 0.0, 0.0]),
                normal: lits([0.0, 0.0, 1.0]),
            },
            instance_b: "ball".into(),
            surface_b: SurfaceRef::Sphere {
                center: lits([0.0, 0.0, 0.0]),
                radius: Scalar::lit(1.2),
            },
        });

    let poses = asm.solve_poses().expect("solve");
    let ball = poses.get("ball").unwrap();
    let center_world = ball.apply_point(kerf_geom::Vec3::zeros());
    // Translation along +z should drop center to 1.2; x/y untouched.
    assert!(
        (center_world.z - 1.2).abs() < 1e-9,
        "sphere center z = {}, want 1.2",
        center_world.z
    );
    // X and Y of the sphere center should not have moved (the mate
    // only translates along the plane normal).
    assert!((center_world.x - 3.0).abs() < 1e-9);
    assert!((center_world.y - (-2.0)).abs() < 1e-9);
}

#[test]
fn tangent_mate_sphere_on_sphere() {
    // Sphere A radius 2 at origin. Sphere B radius 1 at (10, 0, 0). After
    // external tangent, B's center should be at (3, 0, 0): same direction,
    // distance 3 = 2 + 1.
    let asm = Assembly::new()
        .with_instance(Instance {
            id: "big".into(),
            model: AssemblyRef::Inline(Box::new(unit_cube_model())),
            target: None,
            default_pose: Pose::identity(),
        

            pinned: false,})
        .with_instance(Instance {
            id: "small".into(),
            model: AssemblyRef::Inline(Box::new(unit_cube_model())),
            target: None,
            default_pose: Pose::at(10.0, 0.0, 0.0),
        

            pinned: false,})
        .with_mate(Mate::TangentMate {
            instance_a: "big".into(),
            surface_a: SurfaceRef::Sphere {
                center: lits([0.0, 0.0, 0.0]),
                radius: Scalar::lit(2.0),
            },
            instance_b: "small".into(),
            surface_b: SurfaceRef::Sphere {
                center: lits([0.0, 0.0, 0.0]),
                radius: Scalar::lit(1.0),
            },
        });

    let poses = asm.solve_poses().expect("solve");
    let small = poses.get("small").unwrap();
    let cb = small.apply_point(kerf_geom::Vec3::zeros());
    assert!(
        approx_v(cb, kerf_geom::Vec3::new(3.0, 0.0, 0.0), 1e-12),
        "small sphere center = {:?}, want (3,0,0)",
        cb
    );
}

#[test]
fn tangent_mate_unsupported_combo_returns_not_implemented() {
    // Cylinder-cylinder is not supported.
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
            default_pose: Pose::at(5.0, 0.0, 0.0),
        

            pinned: false,})
        .with_mate(Mate::TangentMate {
            instance_a: "a".into(),
            surface_a: SurfaceRef::Cylinder {
                origin: lits([0.0, 0.0, 0.0]),
                axis: lits([0.0, 0.0, 1.0]),
                radius: Scalar::lit(1.0),
            },
            instance_b: "b".into(),
            surface_b: SurfaceRef::Cylinder {
                origin: lits([0.0, 0.0, 0.0]),
                axis: lits([0.0, 0.0, 1.0]),
                radius: Scalar::lit(0.5),
            },
        });

    let err = asm.solve_poses().expect_err("should be NotImplemented");
    match err {
        AssemblyError::Mate(MateError::NotImplemented(idx, _)) => {
            assert_eq!(idx, 0);
        }
        other => panic!("expected NotImplemented, got {other:?}"),
    }
}

// ---------------------------------------------------------------------------
// Cycle-aware solving
// ---------------------------------------------------------------------------

#[test]
fn cycle_three_instances_converges() {
    // Three instances with mates that close a *consistent* triangular
    // cycle. The simple sequential solver would freeze each instance
    // after one mate and either silently drop the third constraint
    // (if its instance_b wasn't yet frozen) or report over-constrained.
    //
    // Cycle:
    //   mate 0: A's (1,0,0) coincident with B's (0,0,0)
    //                                       → B at A + (1,0,0)
    //   mate 1: B's (0,1,0) coincident with C's (0,0,0)
    //                                       → C at B + (0,1,0)
    //   mate 2: C's (-1,-1,0) coincident with A's (0,0,0)
    //                                       → world(C) + (-1,-1,0) =
    //                                         world(A.0) = A.translation.
    //
    // Substituting: world(C) = A + (1,1,0), so mate 2 says
    //   A + (1,1,0) + (-1,-1,0) = A + (0,0,0) = A.translation. Consistent
    //   for any choice of A. Iterative relaxation should converge no
    //   matter where A starts.
    let asm = Assembly::new()
        .with_instance(Instance {
            id: "a".into(),
            model: AssemblyRef::Inline(Box::new(unit_cube_model())),
            target: None,
            default_pose: Pose::at(0.0, 0.0, 0.0),
        

            pinned: false,})
        .with_instance(Instance {
            id: "b".into(),
            model: AssemblyRef::Inline(Box::new(unit_cube_model())),
            target: None,
            default_pose: Pose::at(10.0, -3.0, 4.0), // far from the truth
        

            pinned: false,})
        .with_instance(Instance {
            id: "c".into(),
            model: AssemblyRef::Inline(Box::new(unit_cube_model())),
            target: None,
            default_pose: Pose::at(-5.0, 7.0, 2.0), // also far
        

            pinned: false,})
        .with_mate(Mate::Coincident {
            instance_a: "a".into(),
            point_a: lits([1.0, 0.0, 0.0]),
            instance_b: "b".into(),
            point_b: lits([0.0, 0.0, 0.0]),
        })
        .with_mate(Mate::Coincident {
            instance_a: "b".into(),
            point_a: lits([0.0, 1.0, 0.0]),
            instance_b: "c".into(),
            point_b: lits([0.0, 0.0, 0.0]),
        })
        // Closes the loop consistently.
        .with_mate(Mate::Coincident {
            instance_a: "c".into(),
            point_a: lits([-1.0, -1.0, 0.0]),
            instance_b: "a".into(),
            point_b: lits([0.0, 0.0, 0.0]),
        });

    let poses = asm.solve_poses().expect("consistent cycle should converge");

    // Verify all three mates are satisfied to within 1e-6 (iterative
    // tolerance — looser than the strict tolerance, but tight enough
    // for assembly purposes).
    let pa = poses.get("a").unwrap().apply_point(kerf_geom::Vec3::new(1.0, 0.0, 0.0));
    let pb = poses.get("b").unwrap().apply_point(kerf_geom::Vec3::zeros());
    assert!(
        approx_v(pa, pb, 1e-6),
        "mate 0 not satisfied: A.(1,0,0)={:?} B.(0)={:?}",
        pa,
        pb
    );

    let pb1 = poses.get("b").unwrap().apply_point(kerf_geom::Vec3::new(0.0, 1.0, 0.0));
    let pc = poses.get("c").unwrap().apply_point(kerf_geom::Vec3::zeros());
    assert!(
        approx_v(pb1, pc, 1e-6),
        "mate 1 not satisfied: B.(0,1,0)={:?} C.(0)={:?}",
        pb1,
        pc
    );

    let pc1 = poses.get("c").unwrap().apply_point(kerf_geom::Vec3::new(-1.0, -1.0, 0.0));
    let pa1 = poses.get("a").unwrap().apply_point(kerf_geom::Vec3::zeros());
    assert!(
        approx_v(pc1, pa1, 1e-6),
        "mate 2 not satisfied: C.(-1,-1,0)={:?} A.(0)={:?}",
        pc1,
        pa1
    );
}

#[test]
fn cycle_overconstrained_rejects() {
    // Three instances forming a cycle — but with INCONSISTENT distances
    // along a triangle. mate 0: A→B at distance 5. mate 1: B→C at
    // distance 5. mate 2: C→A at distance 100 (impossible — three
    // points colinear at offsets of 5+5=10 can't be 100 apart).
    let asm = Assembly::new()
        .with_instance(Instance {
            id: "a".into(),
            model: AssemblyRef::Inline(Box::new(unit_cube_model())),
            target: None,
            default_pose: Pose::at(0.0, 0.0, 0.0),
        

            pinned: false,})
        .with_instance(Instance {
            id: "b".into(),
            model: AssemblyRef::Inline(Box::new(unit_cube_model())),
            target: None,
            default_pose: Pose::at(1.0, 0.0, 0.0),
        

            pinned: false,})
        .with_instance(Instance {
            id: "c".into(),
            model: AssemblyRef::Inline(Box::new(unit_cube_model())),
            target: None,
            default_pose: Pose::at(2.0, 0.0, 0.0),
        

            pinned: false,})
        .with_mate(Mate::Coincident {
            instance_a: "a".into(),
            point_a: lits([0.0, 0.0, 0.0]),
            instance_b: "b".into(),
            point_b: lits([0.0, 0.0, 0.0]),
        })
        .with_mate(Mate::Coincident {
            instance_a: "b".into(),
            point_a: lits([5.0, 0.0, 0.0]),
            instance_b: "c".into(),
            point_b: lits([0.0, 0.0, 0.0]),
        })
        // Closes the cycle inconsistently: A and C must be at the same
        // point, but mates 0+1 force them 5 apart.
        .with_mate(Mate::Coincident {
            instance_a: "a".into(),
            point_a: lits([0.0, 0.0, 0.0]),
            instance_b: "c".into(),
            point_b: lits([0.0, 0.0, 0.0]),
        });

    let err = asm.solve_poses().expect_err("inconsistent cycle should reject");
    match err {
        AssemblyError::Mate(MateError::OverConstrained(_, _))
        | AssemblyError::Mate(MateError::CycleDidNotConverge { .. }) => {}
        other => panic!(
            "expected OverConstrained or CycleDidNotConverge, got {other:?}"
        ),
    }
}

#[test]
fn cycle_two_instances_self_consistent() {
    // Two-instance cycle is degenerate (just two parallel mates between
    // the same pair). Consistent: both mates pin B's local (0,0,0) to
    // A's local (1,0,0). The cycle detector picks this up because the
    // second mate creates a multi-edge between A and B (already in the
    // same component). Iterative refinement should converge instantly.
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
            default_pose: Pose::at(7.0, 0.0, 0.0),
        

            pinned: false,})
        .with_mate(Mate::Coincident {
            instance_a: "a".into(),
            point_a: lits([1.0, 0.0, 0.0]),
            instance_b: "b".into(),
            point_b: lits([0.0, 0.0, 0.0]),
        })
        // Same constraint, redundant: should be detected as a cycle and
        // converge instantly to the consistent solution.
        .with_mate(Mate::Coincident {
            instance_a: "a".into(),
            point_a: lits([1.0, 0.0, 0.0]),
            instance_b: "b".into(),
            point_b: lits([0.0, 0.0, 0.0]),
        });

    let poses = asm.solve_poses().expect("redundant-cycle should converge");
    let b = poses.get("b").unwrap();
    assert!(approx_v(b.translation, kerf_geom::Vec3::new(1.0, 0.0, 0.0), 1e-9));
}

#[test]
fn parallel_plane_round_trip_json() {
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
            default_pose: Pose::at(2.0, 3.0, 4.0),
        

            pinned: false,})
        .with_mate(Mate::ParallelPlane {
            instance_a: "base".into(),
            plane_a_origin: lits([0.0, 0.0, 0.0]),
            plane_a_normal: lits([0.0, 0.0, 1.0]),
            instance_b: "lid".into(),
            plane_b_origin: lits([0.0, 0.0, 0.0]),
            plane_b_normal: lits([0.0, 0.0, 1.0]),
            offset: Scalar::lit(0.75),
        });

    let json = asm.to_json_string().expect("to_json");
    let asm2 = Assembly::from_json_str(&json).expect("from_json");

    let p1 = asm.solve_poses().expect("solve");
    let p2 = asm2.solve_poses().expect("solve2");

    for id in ["base", "lid"] {
        assert!(approx_v(
            p1.get(id).unwrap().translation,
            p2.get(id).unwrap().translation,
            1e-12
        ));
    }
}

#[test]
fn angle_mate_invalid_negative_angle_rejected() {
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
            default_pose: Pose::identity(),
        

            pinned: false,})
        .with_mate(Mate::AngleMate {
            instance_a: "a".into(),
            axis_a: AxisRef {
                origin: lits([0.0, 0.0, 0.0]),
                direction: lits([1.0, 0.0, 0.0]),
            },
            instance_b: "b".into(),
            axis_b: AxisRef {
                origin: lits([0.0, 0.0, 0.0]),
                direction: lits([1.0, 0.0, 0.0]),
            },
            angle: Scalar::lit(-0.1),
        });

    let err = asm.solve_poses().expect_err("negative angle is invalid");
    match err {
        AssemblyError::Mate(MateError::Invalid(_, _)) => {}
        other => panic!("expected Invalid, got {other:?}"),
    }
}
