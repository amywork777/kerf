#!/usr/bin/env bash
# Rebuild the kerf-cad-wasm crate and regenerate JS bindings into src/wasm/.
# Run after changing kerf-brep, kerf-cad, or kerf-cad-wasm.
#
# Size optimization pipeline:
#   1. cargo build --release  — workspace [profile.release] uses opt-level=z,
#      lto=fat, codegen-units=1, panic=abort, strip=true.
#   2. wasm-bindgen with --remove-name-section / --remove-producers-section
#      to strip the symbol names + toolchain producers section.
#   3. wasm-opt -Oz on the result if installed (binaryen). Recommended:
#      brew install binaryen, or `cargo install wasm-opt`.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

echo "==> cargo build --release --target wasm32-unknown-unknown -p kerf-cad-wasm"
cargo build --release --target wasm32-unknown-unknown -p kerf-cad-wasm

WASM_IN="target/wasm32-unknown-unknown/release/kerf_cad_wasm.wasm"
echo "    (raw cargo output: $(wc -c <"$WASM_IN") bytes)"

echo "==> wasm-bindgen --target web (with strip flags) → viewer/src/wasm/"
mkdir -p viewer/src/wasm
wasm-bindgen --target web \
  --remove-name-section \
  --remove-producers-section \
  --out-dir viewer/src/wasm \
  "$WASM_IN"

WASM_OUT="viewer/src/wasm/kerf_cad_wasm_bg.wasm"
echo "    (after wasm-bindgen: $(wc -c <"$WASM_OUT") bytes)"

if command -v wasm-opt >/dev/null 2>&1; then
  echo "==> wasm-opt -Oz --strip-debug --strip-producers"
  # Modern Rust emits memory.copy / sign-ext / nontrapping-float ops by
  # default. wasm-opt's validator rejects those unless we pass the matching
  # --enable-* flag (or pre-3.0 wasm-opt builds; binaryen 129 is post-3.0).
  # We enable the full default-feature set so the validator matches what
  # rustc emits.
  wasm-opt -Oz \
    --enable-bulk-memory \
    --enable-reference-types \
    --enable-mutable-globals \
    --enable-nontrapping-float-to-int \
    --enable-sign-ext \
    --strip-debug \
    --strip-producers \
    -o "$WASM_OUT.opt" "$WASM_OUT"
  mv "$WASM_OUT.opt" "$WASM_OUT"
  echo "    (after wasm-opt -Oz: $(wc -c <"$WASM_OUT") bytes)"
else
  echo "==> wasm-opt not found — skipping. Install with:"
  echo "      brew install binaryen          # macOS"
  echo "      cargo install wasm-opt         # cross-platform"
  echo "    Expected additional ~30-50% size reduction."
fi

echo "==> sync example JSON to viewer/public/examples/"
mkdir -p viewer/public/examples
cp crates/kerf-cad/examples/*.json viewer/public/examples/

echo "done — final wasm size: $(wc -c <"$WASM_OUT") bytes — now: cd viewer && pnpm dev"
