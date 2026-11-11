import { launchWorld, type FallbackLoad, type LaunchOptions } from './launch.js';
import type { World, WorldConfig } from './World.js';

export type WebLaunchOptions = Omit<LaunchOptions, 'config' | 'createWorker' | 'loadFallback'>;

async function loadFallback(config: WorldConfig): Promise<FallbackLoad> {
    // The single-threaded build is only fetched when the page cannot use shared memory.
    const glue = await import('../wasm-st/dynova_core.js');
    const exports = await glue.default() as unknown as { memory: WebAssembly.Memory };
    return { runtime: new glue.Runtime(config.capacity, config.ringCapacity), memory: exports.memory };
}

/**
 * `launchWorld` with the package's own worker and wasm already wired, for a bundler that follows `new URL(..., import.meta.url)` and
 * bundles module workers. Tested with Vite (production build and dev server) and webpack 5. On an isolated page the simulation runs in a Worker over shared memory;
 * on an isolated page where SharedArrayBuffer is nevertheless missing it runs on the calling thread; on a page that supports it but lacks the isolation headers it
 * throws `SharedMemoryUnavailableError`.
 */
export function launch(config: WorldConfig, options: WebLaunchOptions = {}): Promise<World> {
    return launchWorld({
        ...options,
        config,
        createWorker: () => new Worker(new URL('./sim.worker.js', import.meta.url), { type: 'module' }),
        loadFallback,
    });
}
