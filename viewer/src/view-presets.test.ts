/**
 * @vitest-environment jsdom
 */
import { describe, it, expect, beforeEach, vi } from "vitest";
import {
  savePreset,
  loadPreset,
  loadAllPresets,
  deletePreset,
  mountViewPresetsUI,
  type CameraState,
} from "./view-presets.js";

// Reset localStorage before each test so tests are isolated.
beforeEach(() => {
  localStorage.clear();
});

const cam1: CameraState = {
  position: [80, 60, 100],
  target: [0, 0, 0],
};

const cam2: CameraState = {
  position: [0, 200, 0],
  target: [10, 5, 3],
};

describe("savePreset / loadPreset", () => {
  it("save persists a preset and load retrieves it", () => {
    savePreset("myView", cam1);
    const result = loadPreset("myView");
    expect(result).toEqual(cam1);
  });

  it("load returns null for a missing key", () => {
    expect(loadPreset("nonexistent")).toBeNull();
  });

  it("overwriting an existing preset updates it in place", () => {
    savePreset("view1", cam1);
    savePreset("view1", cam2);
    expect(loadPreset("view1")).toEqual(cam2);
    // Should still be only one preset.
    expect(loadAllPresets()).toHaveLength(1);
  });

  it("multiple distinct presets are saved independently", () => {
    savePreset("a", cam1);
    savePreset("b", cam2);
    expect(loadPreset("a")).toEqual(cam1);
    expect(loadPreset("b")).toEqual(cam2);
    expect(loadAllPresets()).toHaveLength(2);
  });
});

describe("loadAllPresets", () => {
  it("returns empty array when nothing is stored", () => {
    expect(loadAllPresets()).toEqual([]);
  });

  it("returns all saved presets with correct shape", () => {
    savePreset("p1", cam1);
    savePreset("p2", cam2);
    const all = loadAllPresets();
    expect(all).toHaveLength(2);
    // Each entry has name, camera, savedAt.
    for (const p of all) {
      expect(typeof p.name).toBe("string");
      expect(typeof p.savedAt).toBe("number");
      expect(p.camera).toHaveProperty("position");
      expect(p.camera).toHaveProperty("target");
    }
  });
});

describe("deletePreset", () => {
  it("removes a preset by name", () => {
    savePreset("x", cam1);
    deletePreset("x");
    expect(loadPreset("x")).toBeNull();
    expect(loadAllPresets()).toHaveLength(0);
  });

  it("deleting a non-existent preset is a no-op", () => {
    savePreset("keep", cam1);
    deletePreset("ghost");
    expect(loadAllPresets()).toHaveLength(1);
  });
});

describe("mountViewPresetsUI", () => {
  it("appends save/load controls into the container", () => {
    const container = document.createElement("div");
    document.body.appendChild(container);
    mountViewPresetsUI(container, () => cam1, () => {});
    expect(container.querySelector("input[type='text']")).not.toBeNull();
    expect(container.querySelector("select")).not.toBeNull();
    // Two buttons: Save view + Load view.
    const buttons = container.querySelectorAll("button");
    expect(buttons.length).toBeGreaterThanOrEqual(2);
  });

  it("Save view button stores the camera state", () => {
    const container = document.createElement("div");
    document.body.appendChild(container);
    let capturedState: CameraState | null = null;

    mountViewPresetsUI(
      container,
      () => cam2,
      (s) => { capturedState = s; },
    );

    const nameInput = container.querySelector<HTMLInputElement>("input[type='text']")!;
    nameInput.value = "testPreset";

    const saveBtn = Array.from(container.querySelectorAll("button")).find(
      (b) => b.textContent?.includes("Save"),
    )!;
    saveBtn.click();

    expect(loadPreset("testPreset")).toEqual(cam2);
    // applyCameraState not called on save.
    expect(capturedState).toBeNull();
  });

  it("Load view button calls applyCameraState with the saved camera", () => {
    savePreset("preset99", cam1);

    const container = document.createElement("div");
    document.body.appendChild(container);
    const apply = vi.fn();

    mountViewPresetsUI(container, () => cam2, apply);

    const select = container.querySelector<HTMLSelectElement>("select")!;
    select.value = "preset99";

    const loadBtn = Array.from(container.querySelectorAll("button")).find(
      (b) => b.textContent?.includes("Load"),
    )!;
    loadBtn.click();

    expect(apply).toHaveBeenCalledWith(cam1);
  });

  it("select is populated with existing presets on mount", () => {
    savePreset("alpha", cam1);
    savePreset("beta", cam2);

    const container = document.createElement("div");
    document.body.appendChild(container);
    mountViewPresetsUI(container, () => cam1, () => {});

    const select = container.querySelector<HTMLSelectElement>("select")!;
    const options = Array.from(select.options).map((o) => o.value);
    expect(options).toContain("alpha");
    expect(options).toContain("beta");
  });
});
