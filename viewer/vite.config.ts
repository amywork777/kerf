import { defineConfig } from "vite";

export default defineConfig({
  server: { port: 5174 },
  build: {
    target: "es2022",
    rollupOptions: {
      // The WASM bindings are generated at build time by build-wasm.sh.
      // Mark as external so `pnpm build` succeeds without the WASM artefact.
      external: [/\/wasm\/kerf_cad_wasm\.js/],
    },
  },
  optimizeDeps: { exclude: ["./src/wasm/kerf_cad_wasm.js"] },
  assetsInclude: ["**/*.wasm"],
  test: {
    environment: "jsdom",
    include: ["src/**/*.test.ts"],
  },
});
