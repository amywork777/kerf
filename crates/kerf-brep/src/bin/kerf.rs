//! `kerf` CLI: STL/OBJ boolean operations.
//!
//! Usage:
//!   kerf <union|intersection|difference> <a> <b> <out>
//!
//! Pass `-` for stdin / stdout. Format is detected from the file extension
//! (`.stl`, `.obj`); for stdin, the first bytes are sniffed.
//!
//! Examples:
//!   kerf union a.stl b.obj merged.stl
//!   kerf union a.stl b.stl - | kerf difference - c.stl out.stl
//!   cat a.obj | kerf intersection - b.stl out.obj

use std::fs::File;
use std::io::{self, BufReader, BufWriter, Read, Write};
use std::process::ExitCode;

use kerf_brep::tessellate::tessellate;
use kerf_brep::{
    from_triangles, read_obj, read_stl_auto, write_binary, write_obj, Solid,
};

#[derive(Copy, Clone, Debug)]
enum Format {
    Stl,
    Obj,
}

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().collect();
    if args.len() != 5 {
        eprintln!(
            "usage: {} <union|intersection|difference> <a> <b> <out>\n  \
             formats detected from extension (.stl, .obj); use '-' for stdin/stdout",
            args.first().map(|s| s.as_str()).unwrap_or("kerf")
        );
        return ExitCode::from(2);
    }

    let op_arg = args[1].as_str();
    let a_path = &args[2];
    let b_path = &args[3];
    let out_path = &args[4];

    let a = match load(a_path) {
        Ok(s) => s,
        Err(e) => {
            eprintln!("kerf: failed to load {a_path}: {e}");
            return ExitCode::from(1);
        }
    };
    let b = match load(b_path) {
        Ok(s) => s,
        Err(e) => {
            eprintln!("kerf: failed to load {b_path}: {e}");
            return ExitCode::from(1);
        }
    };
    eprintln!(
        "kerf: A = {}V/{}E/{}F   B = {}V/{}E/{}F",
        a.vertex_count(),
        a.edge_count(),
        a.face_count(),
        b.vertex_count(),
        b.edge_count(),
        b.face_count(),
    );

    let result = match op_arg {
        "union" => a.union(&b),
        "intersection" => a.intersection(&b),
        "difference" => a.difference(&b),
        other => {
            eprintln!("kerf: unknown op {other:?} (expected union, intersection, or difference)");
            return ExitCode::from(2);
        }
    };
    eprintln!(
        "kerf: result = {}V/{}E/{}F",
        result.vertex_count(),
        result.edge_count(),
        result.face_count(),
    );

    let soup = tessellate(&result, 8);
    let out_format = detect_format_for_output(out_path);
    let mut w: Box<dyn Write> = if out_path == "-" {
        Box::new(BufWriter::new(io::stdout().lock()))
    } else {
        match File::create(out_path) {
            Ok(f) => Box::new(BufWriter::new(f)),
            Err(e) => {
                eprintln!("kerf: failed to create {out_path}: {e}");
                return ExitCode::from(1);
            }
        }
    };
    let write_result = match out_format {
        Format::Obj => write_obj(&soup, "kerf_cli_output", &mut w),
        Format::Stl => write_binary(&soup, "kerf_cli_output", &mut w),
    };
    if let Err(e) = write_result {
        eprintln!("kerf: write failed: {e}");
        return ExitCode::from(1);
    }
    let dest = if out_path == "-" { "stdout" } else { out_path.as_str() };
    eprintln!(
        "kerf: wrote {} triangles ({:?}) → {dest}",
        soup.triangles.len(),
        out_format
    );
    ExitCode::SUCCESS
}

fn detect_format_from_path(path: &str) -> Option<Format> {
    let lower = path.to_ascii_lowercase();
    if lower.ends_with(".obj") {
        Some(Format::Obj)
    } else if lower.ends_with(".stl") {
        Some(Format::Stl)
    } else {
        None
    }
}

/// For output, default to STL if no extension recognised.
fn detect_format_for_output(path: &str) -> Format {
    detect_format_from_path(path).unwrap_or(Format::Stl)
}

/// Load a Solid from a file or stdin. STL/OBJ chosen by extension; stdin
/// content-sniffed (OBJ files typically start with `# ` or `v `; STL ASCII
/// with `solid `; STL binary with an opaque 80-byte header).
fn load(path: &str) -> Result<Solid, String> {
    let mut buf = Vec::new();
    if path == "-" {
        io::stdin()
            .lock()
            .read_to_end(&mut buf)
            .map_err(|e| e.to_string())?;
    } else {
        let mut f = BufReader::new(File::open(path).map_err(|e| e.to_string())?);
        f.read_to_end(&mut buf).map_err(|e| e.to_string())?;
    }

    let format = detect_format_from_path(path).unwrap_or_else(|| sniff_format(&buf));
    let soup = match format {
        Format::Obj => read_obj(&mut buf.as_slice()).map_err(|e| e.to_string())?,
        Format::Stl => read_stl_auto(&buf).map_err(|e| e.to_string())?,
    };
    from_triangles(&soup.triangles).map_err(|e| e.to_string())
}

/// Sniff input format from leading bytes. OBJ files have `v ` / `f ` / `# `
/// near the start; STL ASCII has `solid `; STL binary has anything else and
/// gets a binary parse attempt.
fn sniff_format(buf: &[u8]) -> Format {
    let head = std::str::from_utf8(&buf[..buf.len().min(256)]).unwrap_or("");
    let looks_obj = head
        .lines()
        .take(20)
        .any(|line| {
            let t = line.trim_start();
            t.starts_with("v ") || t.starts_with("f ") || t.starts_with("vn ") || t.starts_with("vt ")
        });
    if looks_obj {
        Format::Obj
    } else {
        Format::Stl
    }
}
