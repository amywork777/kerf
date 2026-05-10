import { describe, it, expect, beforeEach } from "vitest";
import { PickHistory } from "./pick-history.js";

describe("PickHistory", () => {
  let h: PickHistory;

  beforeEach(() => {
    h = new PickHistory(10);
  });

  it("starts empty", () => {
    expect(h.size).toBe(0);
    expect(h.current()).toBeNull();
    expect(h.canGoBack).toBe(false);
    expect(h.canGoForward).toBe(false);
  });

  it("push N > maxSize items — oldest dropped, only maxSize retained", () => {
    const ph = new PickHistory(10);
    for (let i = 0; i < 15; i++) {
      ph.push({ kind: "face", id: i });
    }
    // Should have exactly 10 entries; the oldest 5 were dropped.
    expect(ph.size).toBe(10);
    // Current should be the last pushed item.
    expect(ph.current()).toEqual({ kind: "face", id: 14 });
  });

  it("back returns the previous selection", () => {
    h.push({ kind: "face", id: 1 });
    h.push({ kind: "face", id: 2 });
    h.push({ kind: "edge", id: 3 });

    // Currently at id=3; go back one.
    const prev = h.back();
    expect(prev).toEqual({ kind: "face", id: 2 });
    expect(h.current()).toEqual({ kind: "face", id: 2 });
  });

  it("double-back returns the entry two steps earlier", () => {
    h.push({ kind: "face", id: 10 });
    h.push({ kind: "vertex", id: 20 });
    h.push({ kind: "edge", id: 30 });

    h.back(); // now at id=20
    const earlier = h.back(); // now at id=10
    expect(earlier).toEqual({ kind: "face", id: 10 });
    expect(h.current()).toEqual({ kind: "face", id: 10 });
  });

  it("back at the beginning returns null", () => {
    h.push({ kind: "face", id: 5 });
    expect(h.back()).toBeNull();
    expect(h.current()).toEqual({ kind: "face", id: 5 });
  });

  it("forward after back returns next entry", () => {
    h.push({ kind: "face", id: 1 });
    h.push({ kind: "face", id: 2 });
    h.back(); // at id=1
    const fwd = h.forward();
    expect(fwd).toEqual({ kind: "face", id: 2 });
  });

  it("forward at end returns null", () => {
    h.push({ kind: "face", id: 1 });
    expect(h.forward()).toBeNull();
  });

  it("push after back clears forward history", () => {
    h.push({ kind: "face", id: 1 });
    h.push({ kind: "face", id: 2 });
    h.back(); // at id=1
    h.push({ kind: "vertex", id: 99 }); // clears forward (id=2)
    expect(h.canGoForward).toBe(false);
    expect(h.current()).toEqual({ kind: "vertex", id: 99 });
    expect(h.size).toBe(2); // id=1 + id=99
  });

  it("clear resets everything", () => {
    h.push({ kind: "face", id: 1 });
    h.push({ kind: "edge", id: 2 });
    h.clear();
    expect(h.size).toBe(0);
    expect(h.current()).toBeNull();
    expect(h.canGoBack).toBe(false);
  });

  it("respects a smaller custom maxSize", () => {
    const small = new PickHistory(3);
    for (let i = 0; i < 5; i++) {
      small.push({ kind: "face", id: i });
    }
    expect(small.size).toBe(3);
    expect(small.current()).toEqual({ kind: "face", id: 4 });
  });
});
