//! kerf-cad CLI: load a JSON model, evaluate a target feature, write STL.
//!
//! Usage: kerf-cad <model.json> <target_id> <output.stl> [--segments N]

use std::fs::File;
use std::io::BufWriter;
use std::path::PathBuf;
use std::process::ExitCode;

use kerf_brep::{stl::write_binary, tessellate::tessellate};
use kerf_cad::Model;

const DEFAULT_SEGMENTS: usize = 24;

fn main() -> ExitCode {
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("error: {e}");
            ExitCode::from(1)
        }
    }
}

struct Args {
    model: PathBuf,
    target: String,
    output: PathBuf,
    segments: usize,
}

fn parse_args() -> Result<Args, String> {
    let mut positional: Vec<String> = Vec::new();
    let mut segments = DEFAULT_SEGMENTS;
    let mut iter = std::env::args().skip(1);
    while let Some(a) = iter.next() {
        if a == "--segments" {
            let v = iter
                .next()
                .ok_or_else(|| "--segments requires a value".to_string())?;
            segments = v
                .parse::<usize>()
                .map_err(|e| format!("--segments: {e}"))?;
            if segments < 3 {
                return Err("--segments must be >= 3".into());
            }
        } else if a == "-h" || a == "--help" {
            return Err(usage());
        } else {
            positional.push(a);
        }
    }
    if positional.len() != 3 {
        return Err(usage());
    }
    Ok(Args {
        model: PathBuf::from(&positional[0]),
        target: positional[1].clone(),
        output: PathBuf::from(&positional[2]),
        segments,
    })
}

fn usage() -> String {
    "usage: kerf-cad <model.json> <target_id> <output.stl> [--segments N]".into()
}

fn run() -> Result<(), String> {
    let args = parse_args()?;
    let model = Model::read_json_path(&args.model)
        .map_err(|e| format!("loading {}: {e}", args.model.display()))?;
    let solid = model
        .evaluate(&args.target)
        .map_err(|e| format!("evaluating '{}': {e}", args.target))?;
    let soup = tessellate(&solid, args.segments);
    let f = File::create(&args.output)
        .map_err(|e| format!("creating {}: {e}", args.output.display()))?;
    let mut w = BufWriter::new(f);
    write_binary(&soup, &args.target, &mut w)
        .map_err(|e| format!("writing STL: {e}"))?;
    Ok(())
}
