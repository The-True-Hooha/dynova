import { defineConfig, type Connect, type Plugin } from "vite";
import { resolve } from "path";
import { existsSync, readFileSync } from "fs";

const dracoDir = resolve(__dirname, "../renderer/three/node_modules/three/examples/jsm/libs/draco/gltf");

// Serves the Draco decoder files at /draco/ so GLTFLoader can decode Draco-compressed glTF in the demos.
// Serves the Draco decoder files at /draco/ so GLTFLoader can decode Draco-compressed glTF in the demos.
const serveDraco: Connect.NextHandleFunction = (req, res, next) => {
  const name = (req.url ?? "").split("?")[0].replace(/^\//, "");
  const file = resolve(dracoDir, name);
  if (!/^[\w.]+$/.test(name) || !file.startsWith(dracoDir) || !existsSync(file)) return next();
  res.setHeader("Content-Type", name.endsWith(".wasm") ? "application/wasm" : "text/javascript");
  res.setHeader("Cross-Origin-Resource-Policy", "same-origin");
  res.end(readFileSync(file));
};

const dracoDecoder: Plugin = {
  name: "dynova-draco-decoder",
  configureServer: (server) => void server.middlewares.use("/draco/", serveDraco),
  configurePreviewServer: (server) => void server.middlewares.use("/draco/", serveDraco),
};

export default defineConfig({
  plugins: [dracoDecoder],
  resolve: {
    alias: {
      "@dynova/sdk": resolve(__dirname, "../sdk/typescript/src/index.ts"),
      "@dynova/canvas2d": resolve(__dirname, "../renderer/canvas2d/src/index.ts"),
      "@dynova/three/loaders": resolve(__dirname, "../renderer/three/src/loaders.ts"),
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
