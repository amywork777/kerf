/**
 * @vitest-environment jsdom
 */
import { describe, it, expect, beforeEach } from "vitest";
import { mountErrorOverlay, extractFeatureIdFromError } from "../overlay.js";

describe("error overlay banner", () => {
  let stage: HTMLElement;

  beforeEach(() => {
    document.body.innerHTML = "";
    stage = document.createElement("div");
    stage.id = "stage";
    document.body.appendChild(stage);
  });

  it("mounts a hidden node into the stage", () => {
    const ov = mountErrorOverlay(stage);
    expect(stage.querySelector(".error-overlay")).toBe(ov.element);
    expect(ov.isVisible()).toBe(false);
    expect(ov.element.hidden).toBe(true);
  });

  it("show() reveals the banner with the message", () => {
    const ov = mountErrorOverlay(stage);
    ov.show("evaluate 'foo': bad axis");
    expect(ov.isVisible()).toBe(true);
    expect(ov.element.querySelector(".eo-msg")?.textContent).toBe(
      "evaluate 'foo': bad axis",
    );
  });

  it("show() displays the offending feature id when provided", () => {
    const ov = mountErrorOverlay(stage);
    ov.show("oops", "feat42");
    const featEl = ov.element.querySelector(".eo-feature") as HTMLElement;
    expect(featEl.hidden).toBe(false);
    expect(featEl.textContent).toContain("feat42");
  });

  it("hide() and clicking the banner both dismiss it", () => {
    const ov = mountErrorOverlay(stage);
    ov.show("oops");
    expect(ov.isVisible()).toBe(true);
    ov.element.click();
    expect(ov.isVisible()).toBe(false);
    // hide() while already hidden is a noop, not an error.
    expect(() => ov.hide()).not.toThrow();
  });

  it("calling show() again updates the message in place (idempotent)", () => {
    const ov = mountErrorOverlay(stage);
    ov.show("first");
    ov.show("second");
    expect(ov.element.querySelector(".eo-msg")?.textContent).toBe("second");
    expect(ov.isVisible()).toBe(true);
  });

  it("extractFeatureIdFromError parses 'evaluate '<id>': ...' messages", () => {
    expect(extractFeatureIdFromError("evaluate 'foo': bad axis")).toBe("foo");
    expect(extractFeatureIdFromError("evaluate \"bar\": missing input")).toBe("bar");
    expect(extractFeatureIdFromError("parse error: bad json")).toBeNull();
  });
});
