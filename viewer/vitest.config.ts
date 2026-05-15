import { defineConfig } from "vitest/config";

export default defineConfig({
  test: {
    // Default to jsdom for files that need DOM (overrideable per-file
    // with `@vitest-environment jsdom` directives).
    environment: "node",
    include: ["src/**/*.test.ts"],
    // The wasm bindings file is a generated ESM module that isn't
    // safe to import from a Node test runner — exclude it explicitly
    // so Vitest doesn't try to type-check it.
    // sketch_state.test.ts uses a hand-rolled test framework (pre-vitest); exclude
    // it from vitest and run it separately via the test:legacy script.
    exclude: ["**/node_modules/**", "src/wasm/**", "src/sketch_state.test.ts"],
  },
});
