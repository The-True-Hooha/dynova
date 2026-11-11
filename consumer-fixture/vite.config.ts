import { defineConfig } from 'vite';

const headers = process.env.NO_ISOLATION ? {} : { 'Cross-Origin-Opener-Policy': 'same-origin', 'Cross-Origin-Embedder-Policy': 'require-corp' };

export default defineConfig({
    server: { headers },
    preview: { headers },
    // A dev server pre-bundles dependencies, which would move the worker and wasm away from the files they are loaded relative to.
    optimizeDeps: { exclude: ['@dynova/sdk'] },
});
