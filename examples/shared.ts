import { launchWorld, type FallbackLoad, type World, type WorldConfig } from '@dynova/sdk';

async function loadFallback(config: WorldConfig): Promise<FallbackLoad> {
    const glue = await import('../sdk/typescript/wasm-st/dynova_core.js');
    const exports = await glue.default() as unknown as { memory: WebAssembly.Memory };
    return { runtime: new glue.Runtime(config.capacity, config.ringCapacity), memory: exports.memory };
}

export function createWorld(config: WorldConfig, physics: boolean): Promise<World> {
    return launchWorld({
        config,
        physics,
        createWorker: () => new Worker(new URL('../sdk/typescript/src/sim.worker.ts', import.meta.url), { type: 'module' }),
        loadFallback,
    });
}
