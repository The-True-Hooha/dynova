import { defineConfig, type Plugin } from 'vite';

const headers = process.env.NO_ISOLATION ? {} : { 'Cross-Origin-Opener-Policy': 'same-origin', 'Cross-Origin-Embedder-Policy': 'require-corp' };

// `headers` is not sent on a 304 Not Modified, and WebKit refuses a module worker whose revalidated script lacks the
// cross-origin-embedder-policy header, which breaks a page that starts several workers from one script.
const onEveryResponse: Plugin = {
    name: 'isolation-headers',
    configureServer: (server) => void server.middlewares.use((_req, res, next) => { for (const [k, v] of Object.entries(headers)) res.setHeader(k, v); next(); }),
    configurePreviewServer: (server) => void server.middlewares.use((_req, res, next) => { for (const [k, v] of Object.entries(headers)) res.setHeader(k, v); next(); }),
};

export default defineConfig({
    plugins: [onEveryResponse],
    server: { headers },
    preview: { headers },
    // A dev server pre-bundles dependencies, which would move the worker and wasm away from the files they are loaded relative to.
    optimizeDeps: { exclude: ['@dynova/sdk'] },
});
