# Build a parametric mug — end-to-end walkthrough

Audience: someone who has never used kerf-cad before.
Time: about 5 minutes.
What you will do: open the browser viewer, load a mug model, scrub its
parameters, export a 3-view drawing PNG, save the JSON, and re-open it.
You will never see raw JSON.

---

## Prerequisites

- **Rust + Cargo** — `curl https://sh.rustup.rs | sh` if not installed.
- **wasm-pack dependencies** — `cargo install wasm-bindgen-cli` and
  optionally `brew install binaryen` (macOS) for `wasm-opt` size savings.
- **pnpm** — `npm i -g pnpm`.
- **Node 18+** — check with `node --version`.

---

## Step 1 — Get the viewer running (~2 minutes)

**1a.** Clone and enter the repo.

```
git clone https://github.com/amywork777/kerf
cd kerf/viewer
```

**1b.** Build the WASM kernel, install JS deps, and start the dev server.

```
./build-wasm.sh && pnpm install && pnpm dev
```

The build compiles the entire kerf boolean kernel to WebAssembly and copies
example JSON files into `viewer/public/examples/`.

**1c.** Open the URL that Vite prints — defaults to **http://localhost:5174**.

What you should see: a dark panel on the left labelled "kerf-cad viewer" with
a drop zone and a row of example links. The 3-D viewport is empty.

If it doesn't work: make sure `wasm-bindgen-cli` is installed
(`cargo install wasm-bindgen-cli`) and the `wasm32-unknown-unknown` target is
present (`rustup target add wasm32-unknown-unknown`).

---

## Step 2 — Load the starter mug (~15 seconds)

**2a.** In the left panel, click the **mug** link in the "Try one:" row.

What you should see: a mug — cylindrical body with a torus handle — appears in
the viewport. The "Target" dropdown at the top of the panel shows `mug`. The
parameter sliders for `h`, `r_outer`, `wall`, `handle_r`, and
`handle_thickness` appear automatically beneath it.

Alternatively, drag `viewer/public/examples/mug.json` from your file manager
into the drop zone, or click the drop zone to pick the file.

If it doesn't work: the example links only work after `./build-wasm.sh` has run
at least once (it copies the JSON files into `public/examples/`). If the mug
link is missing, run the script and refresh.

> Screenshot placeholder: mug loaded in viewer, torus handle visible.

---

## Step 3 — Scrub the parameters (~1 minute)

The five sliders control every dimension of the mug.

| Parameter | What it does | Default |
|---|---|---|
| `h` | Overall height of the cup | 90 mm |
| `r_outer` | Outer radius of the cup | 37 mm |
| `wall` | Wall and floor thickness | 3.5 mm |
| `handle_r` | Major radius of the torus handle | 22 mm |
| `handle_thickness` | Minor (tube) radius of the handle | 5 mm |

**3a.** Drag the `h` slider to the right. Watch the mug grow taller in real
time.

**3b.** Drag `r_outer` to make a wider or narrower mug.

**3c.** Try `handle_r` to change how far the handle protrudes from the body.

What you should see: the geometry re-evaluates in the browser on every slider
change. The status bar at the bottom of the panel shows vertex/face counts.

If it doesn't work: if the status bar shows a red error, the parameter
combination is invalid (for example, `wall` must be less than `r_outer`).
Reset to defaults with the **Reset params** button.

> Screenshot placeholder: slider panel mid-drag, geometry updated.

---

## Step 4 — Export a 3-view drawing PNG (~10 seconds)

**4a.** Click **Export 3-view PNG** (or press **D** on the keyboard).

What you should see: your browser downloads `mug.drawing.png` — a 4-panel
sheet with Front, Top, Right, and ISO views, each annotated with bounding-box
dimensions.

If it doesn't work: pop-up blockers sometimes intercept the download. Allow
downloads from localhost.

> Screenshot placeholder: the 3-view PNG sheet showing the mug.

---

## Step 5 — Save and reopen the model (~30 seconds)

**5a.** Click **Download JSON** (or press **S** on the keyboard). Your browser
downloads `model.kerf-cad.json` with the current slider values baked into the
`parameters` block.

**5b.** Drag that file back into the drop zone (or click the drop zone and
pick it).

What you should see: the mug reloads exactly as you left it, sliders set to the
saved values.

If it doesn't work: make sure the file you re-open is the one kerf-cad just
saved (look for `model.kerf-cad.json` in your Downloads folder, not the
original `mug.json`).

> Screenshot placeholder: viewer after round-trip reload, parameters
> matching the saved file.

---

## Step 6 — Export STL for 3-D printing (CLI, ~30 seconds)

The browser viewer is read-only — it cannot yet write STL directly. Use the
CLI for that.

**6a.** Build the native CLI (one-time, ~30 s):

```
cargo build --release -p kerf-cad --bin kerf-cad
```

**6b.** Export STL from the saved JSON:

```
./target/release/kerf-cad model.kerf-cad.json mug part.stl
```

Replace `model.kerf-cad.json` with the path to the file you downloaded in
step 5a.

What you should see: `part.stl` appears in the current directory (~28 KB for
the default parameters).

**6c.** Drag `part.stl` into your slicer (Bambu Studio, PrusaSlicer, Cura,
etc.) and print.

Other output formats work the same way — substitute the output extension:

```
./target/release/kerf-cad model.kerf-cad.json mug part.step
./target/release/kerf-cad model.kerf-cad.json mug part.obj
./target/release/kerf-cad model.kerf-cad.json mug part.3mf
```

If it doesn't work: make sure the `target` name (`mug`) matches the `"id"` of
the last feature in the JSON. Use `body` to export just the cup without the
handle.

---

## What is the model doing?

The mug is built from four features:

1. **Cup** (`body`) — an open-top cylindrical container: outer cylinder minus
   inner cavity, controlled by `r_outer`, `h`, and `wall`.
2. **Donut** (`handle_raw`) — a faceted torus placed at the origin, controlled
   by `handle_r` and `handle_thickness`.
3. **Translate** (`handle`) — moves the torus to the side of the cup
   (x offset 57 mm, z offset 45 mm to centre it on the cup's height).
4. **Union** (`mug`) — merges body and handle into a single solid.

The viewer evaluates only the feature you select in the Target dropdown. Pick
`body` to inspect the cup alone; pick `mug` for the complete model.

---

## What is coming

These features are queued in open pull requests and will land soon:

- **PropertyManager** (PR #50) — numeric input fields with expression support
  alongside the sliders; native "Save" that writes back to the original file
  without a browser download dialog.
- **Toolbar + Feature add** — buttons to add new features without editing JSON.
- **Section view** — real-time cross-section cut in the viewport.
- **Mass properties panel** — volume, surface area, centre of mass.
- **GD&T drawing sheet** — export a proper 2-D engineering drawing with
  tolerances, not just a PNG snapshot.
- **Assembly mates** — multi-part assemblies with geometric constraints.
- **Sketcher → extrude** — draw a 2-D profile in the browser and extrude it
  into a 3-D solid without leaving the viewer.

Until those PRs merge, the "Download JSON → CLI export" path in step 5–6 is
the recommended workflow for getting a printable STL.

---

*Screenshots are placeholders — real screenshots will be added once the viewer
UI stabilises post-PR-merge.*
