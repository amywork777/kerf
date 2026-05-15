/**
 * @vitest-environment jsdom
 */
import { describe, it, expect, beforeEach, afterEach } from "vitest";
import {
  mountTutorialOverlay,
  resetTutorial,
  showTutorial,
  STORAGE_KEY,
} from "./tutorial-overlay.js";

function overlayPresent(): boolean {
  return document.getElementById("tutorial-overlay") !== null;
}

beforeEach(() => {
  // Clean DOM and storage before each test.
  document.body.innerHTML = "";
  localStorage.clear();
});

afterEach(() => {
  // Remove overlay if any test left it mounted.
  const el = document.getElementById("tutorial-overlay");
  if (el) el.remove();
});

describe("tutorial-overlay", () => {
  it("first call mounts the overlay", () => {
    mountTutorialOverlay();
    expect(overlayPresent()).toBe(true);
  });

  it("skip button dismisses overlay and sets localStorage flag", () => {
    mountTutorialOverlay();
    expect(overlayPresent()).toBe(true);

    const skip = document.getElementById("tutorial-skip") as HTMLButtonElement;
    expect(skip).not.toBeNull();
    skip.click();

    expect(overlayPresent()).toBe(false);
    expect(localStorage.getItem(STORAGE_KEY)).toBe("1");
  });

  it("second call with flag already set does not mount overlay", () => {
    localStorage.setItem(STORAGE_KEY, "1");
    mountTutorialOverlay();
    expect(overlayPresent()).toBe(false);
  });

  it("resetTutorial clears the localStorage flag", () => {
    localStorage.setItem(STORAGE_KEY, "1");
    resetTutorial();
    expect(localStorage.getItem(STORAGE_KEY)).toBeNull();
  });

  it("showTutorial mounts overlay unconditionally even when flag is set", () => {
    localStorage.setItem(STORAGE_KEY, "1");
    showTutorial();
    expect(overlayPresent()).toBe(true);
  });

  it("resetTutorial + mountTutorialOverlay re-shows the overlay", () => {
    // Simulate a user who dismissed and then clicks "Show tour".
    localStorage.setItem(STORAGE_KEY, "1");
    resetTutorial();
    mountTutorialOverlay();
    expect(overlayPresent()).toBe(true);
  });
});
