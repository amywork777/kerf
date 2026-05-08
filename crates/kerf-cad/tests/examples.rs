//! Step 6 — every shipped example evaluates cleanly to a positive-volume Solid.

use std::fs;
use std::path::{Path, PathBuf};

use kerf_brep::solid_volume;
use kerf_cad::Model;

fn examples_dir() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("examples")
}

fn each_example() -> Vec<PathBuf> {
    fs::read_dir(examples_dir())
        .unwrap()
        .filter_map(|e| {
            let p = e.unwrap().path();
            (p.extension().map(|e| e == "json").unwrap_or(false)).then_some(p)
        })
        .collect()
}

#[test]
fn at_least_three_examples_exist() {
    let n = each_example().len();
    assert!(n >= 3, "expected at least 3 example models, found {n}");
}

#[test]
fn every_example_evaluates_to_positive_volume_via_out() {
    for path in each_example() {
        let m = Model::read_json_path(&path)
            .unwrap_or_else(|e| panic!("loading {}: {e}", path.display()));
        let s = m
            .evaluate("out")
            .unwrap_or_else(|e| panic!("evaluating {}: {e}", path.display()));
        let v = solid_volume(&s);
        assert!(
            v > 0.0,
            "{}: solid_volume should be positive, got {v}",
            path.display()
        );
    }
}
