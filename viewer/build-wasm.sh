#!/usr/bin/env bash
# Rebuild the kerf-cad-wasm crate and regenerate JS bindings into src/wasm/.
# Run after changing kerf-brep, kerf-cad, or kerf-cad-wasm.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

echo "==> cargo build --release --target wasm32-unknown-unknown -p kerf-cad-wasm"
cargo build --release --target wasm32-unknown-unknown -p kerf-cad-wasm

echo "==> wasm-bindgen → viewer/src/wasm/"
mkdir -p viewer/src/wasm
wasm-bindgen --target web \
  --out-dir viewer/src/wasm \
  target/wasm32-unknown-unknown/release/kerf_cad_wasm.wasm

echo "==> sync example JSON to viewer/public/examples/"
mkdir -p viewer/public/examples
cp crates/kerf-cad/examples/*.json viewer/public/examples/

echo "done — now: cd viewer && pnpm dev"
