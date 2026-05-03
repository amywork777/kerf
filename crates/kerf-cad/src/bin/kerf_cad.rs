//! kerf-cad CLI: load a JSON model, evaluate a target feature, write geometry.
//!
//! Output format is chosen by the file extension:
//!   .stl  → binary STL  (mesh)
//!   .obj  → Wavefront OBJ  (mesh)
//!   .step → STEP AP203/214 (B-rep)
//!
//! Usage: kerf-cad <model.json> <target_id> <output.{stl,obj,step}> [--segments N]

use std::fs::File;
use std::io::BufWriter;
use std::path::PathBuf;
use std::process::ExitCode;

use kerf_brep::{obj::write_obj, step::write_step, stl::write_binary, tessellate::tessellate, Solid};
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

#[derive(Clone, Copy)]
enum Format {
    Stl,
    Obj,
    Step,
}

fn format_for(path: &std::path::Path) -> Result<Format, String> {
    let ext = path
        .extension()
        .and_then(|e| e.to_str())
        .map(str::to_ascii_lowercase);
    match ext.as_deref() {
        Some("stl") => Ok(Format::Stl),
        Some("obj") => Ok(Format::Obj),
        Some("step") | Some("stp") => Ok(Format::Step),
        Some(other) => Err(format!(
            "unrecognised output extension '.{other}' — use .stl, .obj, .step, or .stp"
        )),
        None => Err("output path needs a .stl/.obj/.step/.stp extension".into()),
    }
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
    "usage: kerf-cad <model.json> <target_id> <output.{stl,obj,step}> [--segments N]".into()
}

fn run() -> Result<(), String> {
    let args = parse_args()?;
    let format = format_for(&args.output)?;
    let model = Model::read_json_path(&args.model)
        .map_err(|e| format!("loading {}: {e}", args.model.display()))?;
    let solid = model
        .evaluate(&args.target)
        .map_err(|e| format!("evaluating '{}': {e}", args.target))?;
    write_output(&args.output, &args.target, &solid, args.segments, format)
}

fn write_output(
    path: &std::path::Path,
    name: &str,
    solid: &Solid,
    segments: usize,
    format: Format,
) -> Result<(), String> {
    let f = File::create(path).map_err(|e| format!("creating {}: {e}", path.display()))?;
    let mut w = BufWriter::new(f);
    match format {
        Format::Stl => {
            let soup = tessellate(solid, segments);
            write_binary(&soup, name, &mut w).map_err(|e| format!("writing STL: {e}"))
        }
        Format::Obj => {
            let soup = tessellate(solid, segments);
            write_obj(&soup, name, &mut w).map_err(|e| format!("writing OBJ: {e}"))
        }
        Format::Step => write_step(solid, name, &mut w).map_err(|e| format!("writing STEP: {e}")),
    }
}
