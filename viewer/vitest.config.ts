import { defineConfig } from "vitest/config";

export default defineConfig({
  test: {
    // Default environment is node; individual test files override with
    // the `@vitest-environment jsdom` doc-comment directive as needed
    // (e.g. sketcher-dim-entry.test.ts which needs a DOM).
    environment: "node",
    environmentMatchGlobs: [
      // sketcher-dim-entry tests require a DOM.
      ["src/sketcher-dim-entry.test.ts", "jsdom"],
    ],
    include: ["src/**/*.test.ts"],
    // The wasm bindings file is a generated ESM module that isn't
    // safe to import from a Node test runner — exclude it explicitly
    // so Vitest doesn't try to type-check it.
<<<<<<< HEAD
    // sketch_state.test.ts uses a custom console-based runner (pnpm test)
    // and is not compatible with Vitest's test suite format.
=======
    // sketch_state.test.ts uses a hand-rolled test framework (pre-vitest); exclude
    // it from vitest and run it separately via the test:legacy script.
>>>>>>> 6f76b06 (feat(viewer): visual constraint glyphs in the 2D sketcher)
    exclude: ["**/node_modules/**", "src/wasm/**", "src/sketch_state.test.ts"],
  },
});
