//! extract_catalog: scan `crates/kerf-cad/src/feature.rs` and emit a markdown
//! catalog of every `Feature` variant.
//!
//! Wraps `kerf_cad::catalog` — which does the actual parsing + rendering —
//! into a small CLI. Defaults to writing `<repo-root>/docs/FEATURE_CATALOG.md`.
//!
//! Usage:
//!   cargo run -p kerf-cad --bin extract_catalog              # default path
//!   cargo run -p kerf-cad --bin extract_catalog -- /tmp/cat.md
//!
//! The same parsing code is exercised by `tests/catalog_examples.rs`, which
//! evaluates each variant's default example to verify it actually builds. So
//! this binary is also a drift detector: if you add a `Feature` variant
//! without a working default, `cargo test` fails before this binary ever
//! ships a stale catalog.

use std::fs;
use std::path::{Path, PathBuf};
use std::process::ExitCode;

use kerf_cad::catalog::{parse_variants, read_feature_rs, render_markdown};

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().collect();
    let out_path: PathBuf = if args.len() >= 2 {
        PathBuf::from(&args[1])
    } else {
        // Default: <repo-root>/docs/FEATURE_CATALOG.md.
        let manifest_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
        let repo_root = manifest_dir
            .parent()
            .and_then(Path::parent)
            .map(Path::to_path_buf)
            .unwrap_or(manifest_dir);
        repo_root.join("docs").join("FEATURE_CATALOG.md")
    };

    let src = match read_feature_rs() {
        Ok(s) => s,
        Err(e) => {
            eprintln!("failed to read feature.rs: {e}");
            return ExitCode::from(2);
        }
    };

    let variants = match parse_variants(&src) {
        Ok(v) => v,
        Err(e) => {
            eprintln!("parse error: {e}");
            return ExitCode::from(3);
        }
    };

    let md = render_markdown(&variants);

    if let Some(parent) = out_path.parent() {
        if !parent.as_os_str().is_empty() {
            if let Err(e) = fs::create_dir_all(parent) {
                eprintln!("failed to create {}: {e}", parent.display());
                return ExitCode::from(4);
            }
        }
    }

    if let Err(e) = fs::write(&out_path, md) {
        eprintln!("failed to write {}: {e}", out_path.display());
        return ExitCode::from(5);
    }

    eprintln!(
        "wrote {} variants to {}",
        variants.len(),
        out_path.display()
    );
    ExitCode::SUCCESS
}
