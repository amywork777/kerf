/**
 * view-presets.ts
 *
 * Save / load named camera view presets to localStorage.
 * A preset stores the camera position (x,y,z) and orbit target (x,y,z).
 */

export interface CameraState {
  position: [number, number, number];
  target: [number, number, number];
}

export interface ViewPreset {
  name: string;
  camera: CameraState;
  savedAt: number; // Date.now() timestamp
}

const STORAGE_KEY = "kerf-cad-view-presets";

/** Load all saved presets from localStorage. Returns [] if none / parse error. */
export function loadAllPresets(): ViewPreset[] {
  try {
    const raw = localStorage.getItem(STORAGE_KEY);
    if (!raw) return [];
    const parsed = JSON.parse(raw);
    if (!Array.isArray(parsed)) return [];
    return parsed as ViewPreset[];
  } catch {
    return [];
  }
}

/** Persist the full list of presets to localStorage. */
function saveAllPresets(presets: ViewPreset[]): void {
  localStorage.setItem(STORAGE_KEY, JSON.stringify(presets));
}

/**
 * Save (or overwrite) a named preset with the given camera state.
 * Returns the updated list of all presets.
 */
export function savePreset(name: string, camera: CameraState): ViewPreset[] {
  const all = loadAllPresets();
  const idx = all.findIndex((p) => p.name === name);
  const entry: ViewPreset = { name, camera, savedAt: Date.now() };
  if (idx >= 0) {
    all[idx] = entry;
  } else {
    all.push(entry);
  }
  saveAllPresets(all);
  return all;
}

/**
 * Load a preset by name. Returns the camera state or null if not found.
 */
export function loadPreset(name: string): CameraState | null {
  const all = loadAllPresets();
  const found = all.find((p) => p.name === name);
  return found?.camera ?? null;
}

/**
 * Delete a preset by name. Returns the updated list.
 */
export function deletePreset(name: string): ViewPreset[] {
  const all = loadAllPresets().filter((p) => p.name !== name);
  saveAllPresets(all);
  return all;
}

/**
 * Mount "Save view" + "Load view" UI into the given container element.
 *
 * @param container  The DOM element to append controls into.
 * @param getCameraState  Callback that returns current { position, target }.
 * @param applyCameraState  Callback invoked when the user loads a preset.
 */
export function mountViewPresetsUI(
  container: HTMLElement,
  getCameraState: () => CameraState,
  applyCameraState: (state: CameraState) => void,
): void {
  // -- Save row --
  const saveRow = document.createElement("div");
  saveRow.className = "view-preset-row";
  saveRow.style.cssText = "display:flex;gap:4px;margin-top:6px;";

  const nameInput = document.createElement("input");
  nameInput.type = "text";
  nameInput.placeholder = "preset name";
  nameInput.style.cssText =
    "flex:1;padding:4px 6px;background:var(--bg);color:var(--fg);" +
    "border:1px solid var(--line);border-radius:4px;font:inherit;font-size:11px;";

  const saveBtn = document.createElement("button");
  saveBtn.textContent = "Save view";
  saveBtn.style.cssText = "flex:0 0 auto;padding:4px 8px;font-size:11px;";

  saveRow.appendChild(nameInput);
  saveRow.appendChild(saveBtn);

  // -- Load row --
  const loadRow = document.createElement("div");
  loadRow.className = "view-preset-row";
  loadRow.style.cssText = "display:flex;gap:4px;margin-top:4px;";

  const presetSelect = document.createElement("select");
  presetSelect.style.cssText =
    "flex:1;padding:4px 6px;background:var(--bg);color:var(--fg);" +
    "border:1px solid var(--line);border-radius:4px;font:inherit;font-size:11px;";

  const loadBtn = document.createElement("button");
  loadBtn.textContent = "Load view";
  loadBtn.style.cssText = "flex:0 0 auto;padding:4px 8px;font-size:11px;";

  loadRow.appendChild(presetSelect);
  loadRow.appendChild(loadBtn);

  container.appendChild(saveRow);
  container.appendChild(loadRow);

  function refreshSelect() {
    const presets = loadAllPresets();
    presetSelect.innerHTML = "";
    if (presets.length === 0) {
      const opt = document.createElement("option");
      opt.value = "";
      opt.textContent = "(no saved views)";
      presetSelect.appendChild(opt);
    } else {
      for (const p of presets) {
        const opt = document.createElement("option");
        opt.value = p.name;
        opt.textContent = p.name;
        presetSelect.appendChild(opt);
      }
    }
  }

  refreshSelect();

  saveBtn.addEventListener("click", () => {
    const name = nameInput.value.trim();
    if (!name) return;
    const state = getCameraState();
    savePreset(name, state);
    refreshSelect();
    presetSelect.value = name;
    nameInput.value = "";
  });

  loadBtn.addEventListener("click", () => {
    const name = presetSelect.value;
    if (!name) return;
    const state = loadPreset(name);
    if (state) applyCameraState(state);
  });
}
