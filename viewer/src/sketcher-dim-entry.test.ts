// @vitest-environment jsdom
import { describe, it, expect, afterEach } from "vitest";
import { promptDistance } from "./sketcher-dim-entry.js";

function fireKey(input: HTMLElement, key: string) {
  input.dispatchEvent(new KeyboardEvent("keydown", { key, bubbles: true }));
}

afterEach(() => {
  // Clean up any lingering inputs from a test that resolved synchronously.
  document.querySelectorAll(".sk-dim-entry").forEach((el) => el.remove());
});

describe("promptDistance", () => {
  it("creates an <input> element in the DOM", () => {
    const p = promptDistance({ x: 100, y: 200 }, 5);
    const input = document.querySelector(".sk-dim-entry") as HTMLInputElement | null;
    expect(input).not.toBeNull();
    expect(input?.tagName).toBe("INPUT");
    // Resolve the promise so we don't leak it (simulate Escape).
    fireKey(input!, "Escape");
    return p.then((v) => expect(v).toBeNull());
  });

  it("returns the entered value when Enter is pressed", async () => {
    const p = promptDistance({ x: 50, y: 50 }, 3.14);
    const input = document.querySelector(".sk-dim-entry") as HTMLInputElement;
    expect(input).not.toBeNull();

    // Change the value then press Enter.
    input.value = "42.5";
    fireKey(input, "Enter");

    const result = await p;
    expect(result).toBeCloseTo(42.5);
  });

  it("returns null when Escape is pressed", async () => {
    const p = promptDistance({ x: 50, y: 50 }, 10);
    const input = document.querySelector(".sk-dim-entry") as HTMLInputElement;

    fireKey(input, "Escape");

    const result = await p;
    expect(result).toBeNull();
  });

  it("removes the input from the DOM after confirmation", async () => {
    const p = promptDistance({ x: 0, y: 0 }, 1);
    const input = document.querySelector(".sk-dim-entry") as HTMLInputElement;
    input.value = "7";
    fireKey(input, "Enter");
    await p;
    expect(document.querySelector(".sk-dim-entry")).toBeNull();
  });

  it("removes the input from the DOM after cancel", async () => {
    const p = promptDistance({ x: 0, y: 0 }, 1);
    const input = document.querySelector(".sk-dim-entry") as HTMLInputElement;
    fireKey(input, "Escape");
    await p;
    expect(document.querySelector(".sk-dim-entry")).toBeNull();
  });

  it("pre-fills the input with the default value (formatted to 4dp)", () => {
    const p = promptDistance({ x: 0, y: 0 }, 2.5);
    const input = document.querySelector(".sk-dim-entry") as HTMLInputElement;
    expect(parseFloat(input.value)).toBeCloseTo(2.5);
    // cleanup
    fireKey(input, "Escape");
    return p;
  });

  it("positions the input near the anchor", () => {
    const p = promptDistance({ x: 123, y: 456 }, 0);
    const input = document.querySelector(".sk-dim-entry") as HTMLInputElement;
    // style.left should incorporate the x anchor offset (123 + 12 = 135)
    expect(input.style.left).toContain("135");
    // style.top should incorporate the y anchor offset (456 - 10 = 446)
    expect(input.style.top).toContain("446");
    fireKey(input, "Escape");
    return p;
  });
});
