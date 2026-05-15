/**
 * Tests for viewer/src/file-system.ts — native + fallback file pickers.
 *
 * @vitest-environment jsdom
 */
import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";
import { createFileHandle } from "./file-system.js";

const ACCEPT = {
  description: "kerf model",
  mimes: ["application/json"],
  extensions: [".json"],
};

beforeEach(() => {
  // jsdom doesn't ship URL.createObjectURL — stub it for the anchor-download path.
  if (!(URL as unknown as { createObjectURL?: unknown }).createObjectURL) {
    (URL as unknown as { createObjectURL: () => string }).createObjectURL = () => "blob:test";
    (URL as unknown as { revokeObjectURL: () => void }).revokeObjectURL = () => {};
  }
});

afterEach(() => {
  delete (window as unknown as Record<string, unknown>).showSaveFilePicker;
  delete (window as unknown as Record<string, unknown>).showOpenFilePicker;
});

describe("createFileHandle — fallback mode (no native APIs)", () => {
  it("isNativeSupported returns false when window APIs absent", () => {
    const h = createFileHandle();
    expect(h.isNativeSupported()).toBe(false);
  });

  it("hasHandle starts false; currentName starts null", () => {
    const h = createFileHandle();
    expect(h.hasHandle()).toBe(false);
    expect(h.currentName()).toBe(null);
  });

  it("saveAs in fallback mode triggers an anchor download and sets currentName", async () => {
    const clicks: HTMLAnchorElement[] = [];
    const origCreate = document.createElement.bind(document);
    vi.spyOn(document, "createElement").mockImplementation((tag) => {
      const el = origCreate(tag);
      if (tag === "a") {
        clicks.push(el as HTMLAnchorElement);
        // Stub the click so jsdom doesn't actually navigate.
        (el as HTMLAnchorElement).click = vi.fn();
      }
      return el;
    });
    const h = createFileHandle();
    const result = await h.saveAs('{"x":1}', "model.json", ACCEPT);
    expect(result?.name).toBe("model.json");
    expect(h.currentName()).toBe("model.json");
    expect(clicks.length).toBeGreaterThanOrEqual(1);
    expect(clicks[0].download).toBe("model.json");
  });

  it("saveExisting falls through to saveAs when no handle is cached", async () => {
    const origCreate = document.createElement.bind(document);
    vi.spyOn(document, "createElement").mockImplementation((tag) => {
      const el = origCreate(tag);
      if (tag === "a") (el as HTMLAnchorElement).click = vi.fn();
      return el;
    });
    const h = createFileHandle();
    const result = await h.saveExisting('{"y":2}', "fallback.json", ACCEPT);
    expect(result?.name).toBe("fallback.json");
  });
});

describe("createFileHandle — native mode (mocked)", () => {
  it("isNativeSupported returns true when both pickers exist", () => {
    (window as unknown as Record<string, unknown>).showSaveFilePicker = () => {};
    (window as unknown as Record<string, unknown>).showOpenFilePicker = () => {};
    const h = createFileHandle();
    expect(h.isNativeSupported()).toBe(true);
  });

  it("saveAs caches the handle and reuses it on saveExisting", async () => {
    const written: string[] = [];
    const fakeHandle = {
      name: "cube.json",
      getFile: async () => new File(["{}"], "cube.json"),
      createWritable: async () => ({
        write: async (s: string) => {
          written.push(s);
        },
        close: async () => {},
      }),
    };
    (window as unknown as Record<string, unknown>).showSaveFilePicker = vi.fn(
      async () => fakeHandle,
    );
    (window as unknown as Record<string, unknown>).showOpenFilePicker = vi.fn();

    const h = createFileHandle();
    const r1 = await h.saveAs('{"a":1}', "cube.json", ACCEPT);
    expect(r1?.name).toBe("cube.json");
    expect(h.hasHandle()).toBe(true);
    expect(written).toEqual(['{"a":1}']);

    const r2 = await h.saveExisting('{"a":2}', "cube.json", ACCEPT);
    expect(r2?.name).toBe("cube.json");
    expect(written).toEqual(['{"a":1}', '{"a":2}']);
    // showSaveFilePicker was only called the first time.
    expect(
      (window as unknown as Record<string, unknown>).showSaveFilePicker,
    ).toHaveBeenCalledTimes(1);
  });

  it("user cancel (AbortError) returns null instead of throwing", async () => {
    (window as unknown as Record<string, unknown>).showSaveFilePicker = vi.fn(async () => {
      const e = new Error("user cancelled");
      (e as Error & { name: string }).name = "AbortError";
      throw e;
    });
    (window as unknown as Record<string, unknown>).showOpenFilePicker = vi.fn();

    const h = createFileHandle();
    const result = await h.saveAs('{"x":1}', "x.json", ACCEPT);
    expect(result).toBe(null);
    expect(h.hasHandle()).toBe(false);
  });

  it("open caches the handle and returns the file contents", async () => {
    // jsdom's File doesn't implement .text(); fake one that does.
    const fakeFile = {
      text: async () => '{"loaded":true}',
    };
    const fakeHandle = {
      name: "loaded.json",
      getFile: async () => fakeFile as unknown as File,
      createWritable: async () => ({
        write: async () => {},
        close: async () => {},
      }),
    };
    (window as unknown as Record<string, unknown>).showOpenFilePicker = vi.fn(
      async () => [fakeHandle],
    );
    (window as unknown as Record<string, unknown>).showSaveFilePicker = vi.fn();

    const h = createFileHandle();
    const opened = await h.open(ACCEPT);
    expect(opened?.name).toBe("loaded.json");
    expect(opened?.text).toBe('{"loaded":true}');
    expect(h.hasHandle()).toBe(true);
    expect(h.currentName()).toBe("loaded.json");
  });
});
