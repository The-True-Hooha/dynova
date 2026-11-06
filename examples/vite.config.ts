import { defineConfig } from "vite";
import { resolve } from "path";

export default defineConfig({
  resolve: {
    alias: {
      "@dynova/sdk": resolve(__dirname, "../sdk/typescript/src/index.ts"),
      "@dynova/canvas2d": resolve(__dirname, "../renderer/canvas2d/src/index.ts"),
      "@dynova/three": resolve(__dirname, "../renderer/three/src/index.ts"),
      "@dynova/wasm": resolve(__dirname, "../sdk/typescript/wasm"),
    },
  },
  server: {
    port: 5173,
    strictPort: true,
    headers: {
      "Cross-Origin-Opener-Policy": "same-origin",
      "Cross-Origin-Embedder-Policy": "require-corp",
    },
  },
  optimizeDeps: {
    exclude: ["dynova-core"],
  },
});
