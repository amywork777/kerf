//! Step 4 — end-to-end CLI: read JSON model, evaluate target, write STL.

use std::env;
use std::fs;
use std::process::Command;

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

#[test]
fn cli_reads_model_evaluates_target_writes_stl() {
    let dir = env::temp_dir();
    let model_path = dir.join("kerf-cad-cli-test.json");
    let stl_path = dir.join("kerf-cad-cli-test.stl");
    fs::write(&model_path, MODEL_JSON).unwrap();
    let _ = fs::remove_file(&stl_path);

    let out = Command::new(cli_path())
        .arg(&model_path)
        .arg("out")
        .arg(&stl_path)
        .output()
        .expect("run kerf-cad");
    assert!(
        out.status.success(),
        "CLI failed (status={:?}). stderr:\n{}\nstdout:\n{}",
        out.status,
        String::from_utf8_lossy(&out.stderr),
        String::from_utf8_lossy(&out.stdout)
    );

    let meta = fs::metadata(&stl_path).expect("stl file written");
    // Binary STL header is 80 bytes + 4-byte triangle count + N*50 bytes,
    // so any non-empty mesh is well over 84 bytes.
    assert!(meta.len() > 84, "stl file too small: {} bytes", meta.len());

    let _ = fs::remove_file(&model_path);
    let _ = fs::remove_file(&stl_path);
}

#[test]
fn cli_exits_nonzero_on_unknown_target() {
    let dir = env::temp_dir();
    let model_path = dir.join("kerf-cad-cli-bad-target.json");
    let stl_path = dir.join("kerf-cad-cli-bad-target.stl");
    fs::write(&model_path, MODEL_JSON).unwrap();

    let out = Command::new(cli_path())
        .arg(&model_path)
        .arg("does-not-exist")
        .arg(&stl_path)
        .output()
        .expect("run kerf-cad");
    assert!(!out.status.success());
    let err = String::from_utf8_lossy(&out.stderr);
    assert!(
        err.contains("does-not-exist") || err.contains("unknown"),
        "stderr should mention the bad id; got:\n{err}"
    );

    let _ = fs::remove_file(&model_path);
}

#[test]
fn cli_prints_usage_when_called_with_no_args() {
    let out = Command::new(cli_path())
        .output()
        .expect("run kerf-cad");
    assert!(!out.status.success());
    let combined = format!(
        "{}{}",
        String::from_utf8_lossy(&out.stdout),
        String::from_utf8_lossy(&out.stderr)
    );
    assert!(
        combined.to_lowercase().contains("usage"),
        "expected usage hint, got:\n{combined}"
    );
}
