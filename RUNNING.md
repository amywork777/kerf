# Running kerf-cad

## Prerequisites

- Rust toolchain (stable), with `wasm32-unknown-unknown` target:
  ```
  rustup target add wasm32-unknown-unknown
  ```
- [wasm-pack](https://rustwasm.github.io/wasm-pack/) or `wasm-bindgen-cli`:
  ```
  cargo install wasm-bindgen-cli
  ```
- [pnpm](https://pnpm.io/) v9+
- Optional: `wasm-opt` for smaller WASM output (`brew install binaryen`)

## Local dev (browser)

```bash
# 1. Build WASM bindings (run once, then re-run when Rust sources change)
cd viewer && ./build-wasm.sh

# 2. Install JS deps
pnpm install

# 3. Start the dev server
pnpm dev
```

Open http://localhost:5173. The viewer hot-reloads on TypeScript changes; rerun `build-wasm.sh` after any Rust change.

## Local build (static site)

```bash
cd viewer
./build-wasm.sh
pnpm install
pnpm build        # output in viewer/dist/
pnpm preview      # serve the production build locally
```

## Desktop app (Tauri)

The `viewer/src-tauri/` scaffold wraps the same web build into a native window.

```bash
# Install Tauri system deps (first time only):
#   macOS:   xcode-select --install
#   Linux:   see https://tauri.app/start/prerequisites/
#   Windows: install WebView2 runtime

cd viewer
./build-wasm.sh          # build WASM first
pnpm install             # installs @tauri-apps/cli
pnpm tauri build         # produces a native installer in src-tauri/target/release/bundle/
```

The Tauri binary serves `viewer/dist/` as its frontend — no web server required at runtime.

## GitHub Pages auto-deploy

On every push to `integration/smart-merge`, the workflow at
`.github/workflows/deploy.yml` runs automatically:

1. Installs Rust + `wasm32-unknown-unknown` + `wasm-bindgen-cli`
2. Runs `viewer/build-wasm.sh` to compile the Rust crate to WASM
3. Runs `pnpm install && pnpm build` in the `viewer/` directory
4. Publishes `viewer/dist/` to the `gh-pages` branch via
   [peaceiris/actions-gh-pages](https://github.com/peaceiris/actions-gh-pages)

Enable GitHub Pages in repository Settings → Pages → Source: `gh-pages` branch, `/ (root)`.

## Running tests

```bash
# Rust unit tests (1114 tests across kerf-cad and kerf-brep)
cargo test

# Viewer TypeScript tests
cd viewer && pnpm test
```
