# Build a Parametric Mug

This tutorial walks you through building, dimensioning, and exporting a coffee mug.
Target time: 10 minutes. No CAD background needed.

## 1. Open the viewer

Run the development server:

```bash
cd viewer && pnpm dev
```

Then open `http://localhost:5174` in your browser. You will see the kerf-cad viewer:
left panel for loading models and adjusting parameters, a 3D canvas in the centre,
and a status line along the bottom of the left panel.

## 2. Drop the starter JSON

Copy the JSON below into a file called `mug.json`, then drag that file into the
**Drop JSON** zone in the left panel (or click the zone and pick the file).

```json
{
  "parameters": { "h": 100, "r": 40, "wall": 3 },
  "features": [
    { "kind": "Cylinder", "id": "outer", "radius": "$r", "height": "$h", "segments": 32 },
    { "kind": "Shell", "id": "shelled", "input": "outer", "thickness": "$wall" }
  ]
}
```

The mug body appears immediately: a solid white cylinder hollowed out by the Shell
feature.  The status line at the bottom of the left panel shows rebuild time, vertex /
edge / face counts, and the computed volume.

## 3. Scrub the radius

The left panel shows three sliders: **h**, **r**, and **wall**.

- Drag the **r** slider left and right — the mug widens and narrows in real time.
- Set **r** to roughly `35` for a slim travel mug, or `50` for a wide bowl.
- The status line updates on every scrub (typically < 5 ms).

To type an exact value, click the slider and use the arrow keys for fine control.

## 4. Add a handle

A handle is a small torus or box extruded from the side.  Use the **Open 2D sketcher**
button at the bottom of the left panel:

1. Click **Open 2D sketcher** — a floating canvas appears.
2. Draw a closed rectangular loop roughly 20 × 50 units.
3. Click **Extrude** and confirm the direction.  The sketcher emits a new feature JSON
   and loads it into the main viewer automatically.
4. You now have a feature tree with three entries (Cylinder, Shell, and the extrusion).
   Click any entry in the feature tree to set it as the active target and inspect it
   in isolation.

Tip: you can delete a feature by clicking the small **x** that appears when you hover
its row in the Feature tree.

## 5. Generate the 3-view drawing

Click **Export 3-view PNG** in the left panel (or press **D**).  A PNG with front,
top, and side orthographic projections is downloaded to your default folder.

While you are here, try the view presets:

| Key | View |
|-----|------|
| `1` | Isometric |
| `2` | Front |
| `3` | Top |
| `4` | Side |
| `W` | Toggle wireframe overlay |

## 6. Save and reopen

Press **S** (or click **Download JSON**) to save the model with your current parameter
values baked in.

To reopen later, drag the saved `.kerf-cad.json` file back into the drop zone.  All
parameters are restored exactly as you left them; scrub the sliders again and the mesh
rebuilds from the B-rep kernel in milliseconds.

---

**Next steps:** explore the built-in examples (bracket, tube, bolt\_circle) via the
"Try one:" links in the left panel, or read `docs/FEATURE_CATALOG.md` for the full
list of supported feature kinds.
