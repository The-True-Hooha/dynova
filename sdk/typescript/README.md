# @dynova/sdk

A shared-memory simulation runtime for the web. Rust (WebAssembly) owns the world and steps it, in a Worker when the page allows shared memory; JavaScript reads positions from shared memory and sends commands through a ring buffer.

```ts
import { launch } from '@dynova/sdk/web';

const world = await launch({ capacity: 1024, ringCapacity: 256 }, { physics: true });
const body = world.spawn({ y: 5, hx: 0.5, hy: 0.5, hz: 0.5, invMass: 1 });
world.spawn({ y: -0.5, hx: 20, hy: 0.5, hz: 20, invMass: 0 });

function frame() {
    const transforms = world.readTransforms();   // [x y z qx qy qz qw] per entity id
    // draw with an adapter, or read transforms[body.id * 7 + 1] yourself
    requestAnimationFrame(frame);
}
frame();
```

`launch` is `launchWorld` with this package's worker and wasm wired in. It needs a bundler that follows `new URL('./file', import.meta.url)` and bundles module workers. Tested: a Vite production build, the Vite dev server (no extra configuration; excluding `@dynova/sdk` from `optimizeDeps` is not needed) and a webpack 5 production build, each in Chromium and WebKit. Module workers need Chrome 80, Firefox 114 or Safari 15 or newer, and the worker file must be served from your own origin (a CDN `base` on another origin breaks `new Worker`).

## Pages that cannot use shared memory

- **Isolated page** (`Cross-Origin-Opener-Policy: same-origin` and `Cross-Origin-Embedder-Policy: require-corp`): the simulation runs in a Worker over shared memory.
- **Isolated but without `SharedArrayBuffer`** (for example an embedder that removes it): the simulation runs on the calling thread; call `world.step()` yourself each frame (`world.mode === 'fallback'`). A page that is not isolated gets the error below instead, because browsers hide `SharedArrayBuffer` there.
- **Supported but not isolated**: `launch` throws `SharedMemoryUnavailableError` saying which headers to serve.

## What is in the package

`.` (the API: `World`, command and view classes, `launchWorld`, joints, sensors, `Assembly`, controllers), `./web` (`launch`), `./worker` (the simulation worker, for a custom loader). The release wasm for both builds ships inside it (about 240 kB each, 100 kB gzipped).

Adapters: `@dynova/three`, `@dynova/canvas2d`. Debug panel: `@dynova/inspector` (`new InspectorView(world.memory, world.layout.headerByteOffset)`).

Not published to npm yet; see `docs/packaging.md` in the repository for building and installing the tarballs.
