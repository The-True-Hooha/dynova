# Packaging

The four packages build to ESM with type declarations and can be installed into an app outside this repository. They are **not published to npm**; installing means packing tarballs.

| package | contents | needs |
|---|---|---|
| `@dynova/sdk` | the API, `./web` (`launch`), `./worker`, and the release wasm (threaded and single-threaded) | nothing |
| `@dynova/three` | Three.js adapter, colliders, assembly import, `./loaders` | `three` (0.170 tested), `@types/three` for TypeScript, `@dynova/sdk` |
| `@dynova/canvas2d` | Canvas2D adapter | `@dynova/sdk` |
| `@dynova/inspector` | debug panel | `@dynova/sdk` |

## Build and install

```bash
make package          # release wasm (both builds), then each package's dist/ (a plain `pnpm build` of the SDK refuses: it could ship a debug wasm)
cd sdk/typescript && pnpm pack      # repeat in renderer/three, renderer/canvas2d, inspector; a missing or unbuilt dist stops the pack
```

In the consuming app, install the tarballs. The adapters depend on the SDK as `^0.1.0` (one shared copy, so `instanceof` checks across packages hold), which no registry serves yet, so point every copy of it at the tarball with npm `overrides`, or for pnpm an `overrides` entry in `pnpm-workspace.yaml` (both tested; yarn's `resolutions` is not):

```json
{
  "dependencies": { "@dynova/sdk": "file:./dynova-sdk-0.1.0.tgz", "@dynova/three": "file:./dynova-three-0.1.0.tgz", "three": "^0.170.0" },
  "overrides": { "@dynova/sdk": "file:./dynova-sdk-0.1.0.tgz" }
}
```

In the repository the packages keep `exports` pointing at TypeScript source so the examples, tests and type checks need no build; `pnpm pack` swaps in the `dist/` exports from `publishConfig`.

## How the wasm and worker are found

`@dynova/sdk/web` creates the worker with `new URL('./sim.worker.js', import.meta.url)`, and the worker and the single-threaded fallback load their wasm relative to themselves. Vite's production build copies the worker and both wasm files into the output with hashed names; nothing needs configuring. The Vite dev server works without configuration too (tested with and without `optimizeDeps: { exclude: ['@dynova/sdk'] }`; the exclusion is not needed with Vite 8). A webpack 5 production build was checked once by hand (a bundle of the consumer app with the worker and both wasm files emitted as chunks, run in Chromium and WebKit over shared memory); that check is not automated. Rollup, esbuild and Next.js are untested, and esbuild alone does not bundle workers. Module workers need Chrome 80, Firefox 114 or Safari 15 or newer, and the worker must come from the page's origin, so a CDN `base` on another origin breaks it. TypeScript needs `moduleResolution` `bundler`, `node16` or `nodenext` to resolve the subpath entries (`@dynova/sdk/web`, `@dynova/three/loaders`).

The sources reach the wasm through `../wasm`; the SDK's build script rewrites those two imports for `dist/` and fails if any `../wasm` reference is left.

## The consumer test

```bash
make consumer-test        # installs with npm
make consumer-test-pnpm   # the same, installing with pnpm (workspace-level `overrides`)
```

packs the four packages into a temporary directory, installs them with `npm` into a clean app copied from `consumer-fixture/` (outside the pnpm workspace), type-checks it (and checks the packed `.d.ts` files with library checks on, so a declaration that points at a missing type fails),  bundles it with Vite, and runs `e2e-consumer/consumer.spec.ts` in Chromium and WebKit against `vite preview` and the Vite dev server. It checks four modes: an isolated page (the world runs in a worker over shared memory, the Three.js and Canvas2D adapters read it and the view paints each entity where it is, the inspector lists its entities and joint states cross the worker boundary), an isolated page where `SharedArrayBuffer` is removed (the same code on the calling thread; a non-isolated page cannot reach this because browsers hide it there), a page without the isolation headers (the error names the headers), and the Vite dev server running the packaged world over shared memory. It needs network access for `npm install`.

## Not done

- Publishing to npm and a release process. The licence is MIT, and the packages are no longer `private`, so `pnpm publish` would not refuse: do not publish without a deliberate decision (name ownership, versioning).
- Windows (the build and consumer scripts shell out to `pnpm` and `npm`).
- Rollup, esbuild-only and Next.js builds, and yarn installs, are untested; Firefox is untested here as everywhere.
- Versions are `0.1.0` and the API is not declared stable.
