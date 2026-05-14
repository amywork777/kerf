//! Integration tests for 3MF and GLTF export — both library API and CLI.

use std::env;
use std::fs;
use std::process::Command;

use kerf_brep::primitives::box_;
use kerf_brep::tessellate::tessellate;
use kerf_brep::{write_3mf, write_gltf};
use kerf_geom::Vec3;

// ---------------------------------------------------------------------------
// Shared minimal model JSON (same as cli.rs tests)
// ---------------------------------------------------------------------------

const MODEL_JSON: &str = r#"{
  "features": [
    {"kind": "Box", "id": "body", "extents": [10.0, 5.0, 2.0]},
    {"kind": "Cylinder", "id": "hole", "radius": 1.0, "height": 5.0, "segments": 16},
    {"kind": "Translate", "id": "hole_pos", "input": "hole", "offset": [5.0, 2.5, -1.0]},
    {"kind": "Difference", "id": "out", "inputs": ["body", "hole_pos"]}
  ]
}"#;

fn cli_path() -> &'static str {
    env!("CARGO_BIN_EXE_kerf-cad")
}

// ---------------------------------------------------------------------------
// Library-level tests
// ---------------------------------------------------------------------------

/// export_3mf produces a valid ZIP (bytes start with PK\x03\x04).
#[test]
fn export_3mf_produces_pk_magic() {
    let solid = box_(Vec3::new(1.0, 1.0, 1.0));
    let soup = tessellate(&solid, 8);
    let bytes = write_3mf(&soup, "box").unwrap();
    assert!(
        bytes.starts_with(b"PK\x03\x04"),
        "expected ZIP magic PK\\x03\\x04, got {:?}",
        &bytes[..4.min(bytes.len())]
    );
}

/// export_gltf produces a valid GLB (bytes start with "glTF").
#[test]
fn export_gltf_produces_gltf_magic() {
    let solid = box_(Vec3::new(1.0, 1.0, 1.0));
    let soup = tessellate(&solid, 8);
    let bytes = write_gltf(&soup, "box").unwrap();
    assert!(
        bytes.starts_with(b"glTF"),
        "expected GLB magic 'glTF', got {:?}",
        &bytes[..4.min(bytes.len())]
    );
}

/// Round-trip vertex count for 3MF: parse out vertex XML elements, count them,
/// confirm they match the deduplicated unique vertex count from the soup.
#[test]
fn export_3mf_roundtrip_vertex_count_box() {
    let solid = box_(Vec3::new(2.0, 3.0, 4.0));
    let soup = tessellate(&solid, 8);

    // Count unique vertices in the soup (mirrors what write_3mf does).
    let tol = kerf_geom::Tolerance::default();
    let mut unique: Vec<kerf_geom::Point3> = Vec::new();
    for tri in &soup.triangles {
        for &p in tri {
            if !unique.iter().any(|q| (p - *q).norm() < tol.point_eq) {
                unique.push(p);
            }
        }
    }
    let expected = unique.len();

    let bytes = write_3mf(&soup, "box").unwrap();
    let cursor = std::io::Cursor::new(&bytes);
    let mut zip = zip::ZipArchive::new(cursor).expect("valid ZIP");
    let mut model_file = zip.by_name("3D/3dmodel.model").expect("3dmodel.model present");
    let mut content = String::new();
    std::io::Read::read_to_string(&mut model_file, &mut content).unwrap();
    let actual = content.matches("<vertex").count();
    assert_eq!(
        actual, expected,
        "3MF vertex count {actual} != expected {expected}"
    );
}

/// Round-trip vertex count for GLTF: parse the GLB JSON chunk, extract the
/// POSITION accessor count, confirm it matches expected unique vertices.
#[test]
fn export_gltf_roundtrip_vertex_count_box() {
    let solid = box_(Vec3::new(2.0, 3.0, 4.0));
    let soup = tessellate(&solid, 8);

    // Count unique vertices in the soup.
    let tol = kerf_geom::Tolerance::default();
    let mut unique: Vec<kerf_geom::Point3> = Vec::new();
    for tri in &soup.triangles {
        for &p in tri {
            if !unique.iter().any(|q| (p - *q).norm() < tol.point_eq) {
                unique.push(p);
            }
        }
    }
    let expected = unique.len();

    let bytes = write_gltf(&soup, "box").unwrap();
    let actual = kerf_brep::export_gltf::glb_vertex_count(&bytes).unwrap();
    assert_eq!(
        actual, expected,
        "GLB vertex count {actual} != expected {expected}"
    );
}

// ---------------------------------------------------------------------------
// CLI integration tests
// ---------------------------------------------------------------------------

/// CLI writes a 3MF file when the output extension is .3mf.
#[test]
fn cli_writes_3mf_when_extension_is_3mf() {
    let dir = env::temp_dir();
    let model_path = dir.join("kerf-cad-cli-3mf-test.json");
    let out_path = dir.join("kerf-cad-cli-3mf-test.3mf");
    fs::write(&model_path, MODEL_JSON).unwrap();
    let _ = fs::remove_file(&out_path);

    let out = Command::new(cli_path())
        .arg(&model_path)
        .arg("out")
        .arg(&out_path)
        .output()
        .expect("run kerf-cad");

    assert!(
        out.status.success(),
        "CLI failed (status={:?}). stderr:\n{}",
        out.status,
        String::from_utf8_lossy(&out.stderr)
    );

    let bytes = fs::read(&out_path).expect("3mf file written");
    assert!(
        bytes.starts_with(b"PK\x03\x04"),
        "expected ZIP magic, got {:?}",
        &bytes[..4.min(bytes.len())]
    );

    let _ = fs::remove_file(&model_path);
    let _ = fs::remove_file(&out_path);
}

/// CLI writes a GLB file when the output extension is .gltf.
#[test]
fn cli_writes_gltf_when_extension_is_gltf() {
    let dir = env::temp_dir();
    let model_path = dir.join("kerf-cad-cli-gltf-test.json");
    let out_path = dir.join("kerf-cad-cli-gltf-test.gltf");
    fs::write(&model_path, MODEL_JSON).unwrap();
    let _ = fs::remove_file(&out_path);

    let out = Command::new(cli_path())
        .arg(&model_path)
        .arg("out")
        .arg(&out_path)
        .output()
        .expect("run kerf-cad");

    assert!(
        out.status.success(),
        "CLI failed (status={:?}). stderr:\n{}",
        out.status,
        String::from_utf8_lossy(&out.stderr)
    );

    let bytes = fs::read(&out_path).expect("gltf/glb file written");
    assert!(
        bytes.starts_with(b"glTF"),
        "expected GLB magic, got {:?}",
        &bytes[..4.min(bytes.len())]
    );

    let _ = fs::remove_file(&model_path);
    let _ = fs::remove_file(&out_path);
}

/// CLI writes a GLB file when the output extension is .glb.
#[test]
fn cli_writes_gltf_when_extension_is_glb() {
    let dir = env::temp_dir();
    let model_path = dir.join("kerf-cad-cli-glb-test.json");
    let out_path = dir.join("kerf-cad-cli-glb-test.glb");
    fs::write(&model_path, MODEL_JSON).unwrap();
    let _ = fs::remove_file(&out_path);

    let out = Command::new(cli_path())
        .arg(&model_path)
        .arg("out")
        .arg(&out_path)
        .output()
        .expect("run kerf-cad");

    assert!(
        out.status.success(),
        "CLI failed (status={:?}). stderr:\n{}",
        out.status,
        String::from_utf8_lossy(&out.stderr)
    );

    let bytes = fs::read(&out_path).expect("glb file written");
    assert!(
        bytes.starts_with(b"glTF"),
        "expected GLB magic, got {:?}",
        &bytes[..4.min(bytes.len())]
    );

    let _ = fs::remove_file(&model_path);
    let _ = fs::remove_file(&out_path);
}
