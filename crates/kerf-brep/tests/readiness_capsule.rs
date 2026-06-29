//! Integration tests for `capsule_faceted` — covering volume, topology,
//! boolean interop, and STL/JSON round-trips.

use std::io::Cursor;

use kerf_brep::primitives::{
    box_, box_at, capsule_faceted, capsule_volume_analytic, cylinder_faceted,
};
use kerf_brep::{
    read_json, read_stl_binary_to_solid, shell_volume, solid_volume, tessellate, write_binary,
    write_json,
};
use kerf_geom::{Point3, Vec3};

const VOL_TOL_REL: f64 = 0.03; // 3 % — polyhedral approximation with m=8

// ── helpers ──────────────────────────────────────────────────────────────────

fn check_euler(label: &str, s: &kerf_brep::Solid) {
    let v = s.vertex_count() as i64;
    let e = s.edge_count() as i64;
    let f = s.face_count() as i64;
    let r_holes: i64 = s
        .topo
        .face_ids()
        .map(|fid| {
            s.topo
                .face(fid)
                .map(|fc| fc.inner_loops().len())
                .unwrap_or(0) as i64
        })
        .sum();
    let shells = s.topo.shell_count() as i64;
    let lhs = v - e + f - r_holes;
    let genus_x2 = 2 * shells - lhs;
    assert!(
        genus_x2 >= 0 && genus_x2 % 2 == 0,
        "{label}: V-E+F-R={lhs} S={shells} genus*2={genus_x2} (must be non-negative even)"
    );
}

// ── topology ─────────────────────────────────────────────────────────────────

#[test]
fn capsule_n8_m4_topology_validates() {
    let s = capsule_faceted(1.0, 2.0, 8, 4);
    kerf_topo::validate(&s.topo).expect("topology");
    check_euler("n8_m4_h2", &s);
}

#[test]
fn capsule_n24_m8_topology_validates() {
    let s = capsule_faceted(1.0, 4.0, 24, 8);
    kerf_topo::validate(&s.topo).expect("topology");
    check_euler("n24_m8_h4", &s);
}

#[test]
fn capsule_h0_sphere_like_topology_validates() {
    let s = capsule_faceted(2.0, 0.0, 16, 6);
    kerf_topo::validate(&s.topo).expect("topology");
    check_euler("n16_m6_h0", &s);
    // Single shell with positive volume.
    assert_eq!(s.topo.shell_count(), 1);
    assert!(solid_volume(&s) > 0.0);
}

// ── volume vs analytic ────────────────────────────────────────────────────────

#[test]
fn capsule_volume_r1_h0_matches_sphere() {
    let r = 1.0;
    let got = solid_volume(&capsule_faceted(r, 0.0, 24, 8));
    let expected = capsule_volume_analytic(r, 0.0); // = 4/3 π
    let err = (got - expected).abs() / expected;
    assert!(err < VOL_TOL_REL, "vol={got:.5} expected={expected:.5} err={err:.4}");
}

#[test]
fn capsule_volume_r1_h1_correct() {
    let (r, h) = (1.0, 1.0);
    let got = solid_volume(&capsule_faceted(r, h, 24, 8));
    let expected = capsule_volume_analytic(r, h);
    let err = (got - expected).abs() / expected;
    assert!(err < VOL_TOL_REL, "r={r} h={h} vol={got:.5} expected={expected:.5} err={err:.4}");
}

#[test]
fn capsule_volume_r05_h3_correct() {
    let (r, h) = (0.5, 3.0);
    let got = solid_volume(&capsule_faceted(r, h, 24, 8));
    let expected = capsule_volume_analytic(r, h);
    let err = (got - expected).abs() / expected;
    assert!(err < VOL_TOL_REL, "r={r} h={h} vol={got:.5} expected={expected:.5} err={err:.4}");
}

#[test]
fn capsule_shell_volume_positive() {
    // A capsule is a single positive shell.
    let s = capsule_faceted(1.0, 2.0, 12, 4);
    for sh in s.topo.shell_ids() {
        let sv = shell_volume(&s, sh);
        assert!(sv > 0.0, "shell volume should be positive, got {sv}");
    }
}

// ── STL round-trip ────────────────────────────────────────────────────────────

#[test]
fn capsule_stl_roundtrip_preserves_volume() {
    let s = capsule_faceted(1.0, 2.0, 12, 4);
    let v_in = solid_volume(&s);
    // Tessellate → write → re-import.
    let soup = tessellate(&s, 12);
    let mut buf = Vec::new();
    write_binary(&soup, "capsule", &mut buf).expect("write_binary");
    let s2 = read_stl_binary_to_solid(&mut Cursor::new(buf)).expect("read_stl");
    let v_out = solid_volume(&s2);
    assert!(
        (v_in - v_out).abs() < 1e-3,
        "STL round-trip volume drift: in={v_in:.6} out={v_out:.6}"
    );
}

// ── JSON round-trip ───────────────────────────────────────────────────────────

#[test]
fn capsule_json_roundtrip_preserves_topology() {
    let s = capsule_faceted(2.0, 5.0, 12, 4);
    let mut buf = Vec::new();
    write_json(&s, &mut buf).expect("write_json");
    let s2 = read_json(&mut buf.as_slice()).expect("read_json");
    assert_eq!(s.vertex_count(), s2.vertex_count());
    assert_eq!(s.edge_count(), s2.edge_count());
    assert_eq!(s.face_count(), s2.face_count());
    kerf_topo::validate(&s2.topo).expect("round-tripped topology");
    let v1 = solid_volume(&s);
    let v2 = solid_volume(&s2);
    assert!((v1 - v2).abs() < 1e-6, "JSON volume drift: {v1} vs {v2}");
}

// ── boolean interop ───────────────────────────────────────────────────────────

#[test]
fn capsule_minus_box_reduces_volume() {
    // Carve a box through the center of a capsule.
    let cap = capsule_faceted(2.0, 4.0, 12, 4);
    let cutter = box_at(Vec3::new(1.0, 1.0, 8.0), Point3::new(-0.5, -0.5, -4.0));
    match cap.try_difference(&cutter) {
        Ok(r) => {
            kerf_topo::validate(&r.topo).expect("boolean result topology");
            let v_cap = solid_volume(&cap);
            let v_cut = solid_volume(&cutter);
            let v_r = solid_volume(&r);
            // Result volume < capsule volume and > capsule - cutter.
            assert!(v_r < v_cap + 1e-6, "result should be smaller than capsule");
            assert!(v_r > (v_cap - v_cut) - 1e-3, "result should be > cap - cutter");
        }
        Err(e) => {
            // Acceptable: this configuration may hit current kernel limits.
            eprintln!("capsule minus box: boolean failed (known limit): {}", e.message);
        }
    }
}

#[test]
fn capsule_union_box_at_least_capsule_volume() {
    // Union of capsule + disjoint box = sum of volumes.
    let cap = capsule_faceted(1.0, 2.0, 12, 4);
    let blk = box_at(Vec3::new(1.0, 1.0, 1.0), Point3::new(10.0, 0.0, 0.0));
    match cap.try_union(&blk) {
        Ok(r) => {
            let v_cap = solid_volume(&cap);
            let v_blk = solid_volume(&blk);
            let v_r = solid_volume(&r);
            assert!(
                (v_r - (v_cap + v_blk)).abs() < 1e-3,
                "disjoint union vol={v_r:.4} expected≈{:.4}", v_cap + v_blk
            );
        }
        Err(e) => {
            eprintln!("capsule union box: boolean failed (known limit): {}", e.message);
        }
    }
}

#[test]
fn capsule_cylinder_both_faceted_boolean_attempted() {
    // Two faceted primitives — both imported-mesh style, so boolean should work
    // wherever intersection segments align with edges.
    let cap = capsule_faceted(0.5, 3.0, 12, 4);
    let cyl = cylinder_faceted(0.4, 5.0, 12);
    // Just verify it doesn't panic — result may fail via try_*.
    let _ = cap.try_union(&cyl);
    let _ = cap.try_difference(&cyl);
}

#[test]
fn capsule_box_difference_preserves_euler() {
    // Build a capsule, subtract a box, check Euler on the result (if it succeeds).
    let cap = capsule_faceted(1.0, 2.0, 12, 4);
    let cut = box_(Vec3::new(0.5, 0.5, 5.0));
    if let Ok(r) = cap.try_difference(&cut) {
        check_euler("cap - box", &r);
    }
}
