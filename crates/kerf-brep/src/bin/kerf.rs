//! `kerf` CLI: STL boolean operations.
//!
//! Usage:
//!   kerf <union|intersection|difference> <a.stl> <b.stl> <out.stl>
//!
//! Pass `-` for stdin / stdout. Inputs may be ASCII or binary STL; output is
//! always binary STL.
//!
//! Examples:
//!   kerf union a.stl b.stl out.stl
//!   kerf union a.stl b.stl - | kerf difference - c.stl out.stl
//!   cat a.stl | kerf intersection - b.stl out.stl

use std::fs::File;
use std::io::{self, BufReader, BufWriter, Read, Write};
use std::process::ExitCode;

use kerf_brep::tessellate::tessellate;
use kerf_brep::{read_stl_to_solid, write_binary, Solid};

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().collect();
    if args.len() != 5 {
        eprintln!(
            "usage: {} <union|intersection|difference> <a.stl> <b.stl> <out.stl>",
            args.first().map(|s| s.as_str()).unwrap_or("kerf")
        );
        return ExitCode::from(2);
    }

    let op_arg = args[1].as_str();
    let a_path = &args[2];
    let b_path = &args[3];
    let out_path = &args[4];

    let a = match load_stl(a_path) {
        Ok(s) => s,
        Err(e) => {
            eprintln!("kerf: failed to load {a_path}: {e}");
            return ExitCode::from(1);
        }
    };
    let b = match load_stl(b_path) {
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
    if let Err(e) = write_binary(&soup, "kerf_cli_output", &mut w) {
        eprintln!("kerf: write failed: {e}");
        return ExitCode::from(1);
    }
    if out_path == "-" {
        eprintln!("kerf: wrote {} triangles → stdout", soup.triangles.len());
    } else {
        eprintln!("kerf: wrote {} triangles → {out_path}", soup.triangles.len());
    }
    ExitCode::SUCCESS
}

fn load_stl(path: &str) -> Result<Solid, String> {
    let mut r: Box<dyn Read> = if path == "-" {
        Box::new(BufReader::new(io::stdin().lock()))
    } else {
        let f = File::open(path).map_err(|e| e.to_string())?;
        Box::new(BufReader::new(f))
    };
    read_stl_to_solid(&mut r).map_err(|e| e.to_string())
}
