import { createMemory, detectMode, MemoryMode } from './memory.js';
import { MAX_CAPACITY } from './CommandRing.js';
import { World, WorldConfig, WorldLayout, RuntimePointers, layoutFromRuntime } from './World.js';

export interface SimInit {
    type: 'init';
    memory: WebAssembly.Memory;
    capacity: number;
    ringCapacity: number;
    physics: boolean;
    hz: number;
    autoRun: boolean;
}

export type SimMessage =
    | { type: 'ready'; layout: WorldLayout }
    | { type: 'error'; message: string };

export interface FallbackRuntime extends RuntimePointers {
    step(): void;
    enable_physics(): void;
}

export interface FallbackLoad {
    runtime: FallbackRuntime;
    memory: WebAssembly.Memory;
}

export interface LaunchOptions {
    config: WorldConfig;
    physics?: boolean;
    hz?: number;
    autoRun?: boolean;
    initialPages?: number;
    maximumPages?: number;
    createWorker: () => Worker;
    loadFallback: (config: WorldConfig) => Promise<FallbackLoad>;
}

export class SharedMemoryUnavailableError extends Error {
    constructor(readonly reason: 'headers-missing') {
        super(
            'SharedArrayBuffer is supported but unavailable: the page is not cross-origin isolated. ' +
            'Serve it with "Cross-Origin-Opener-Policy: same-origin" and "Cross-Origin-Embedder-Policy: require-corp".',
        );
        this.name = 'SharedMemoryUnavailableError';
    }
}

function validate(opts: LaunchOptions): void {
    const { capacity, ringCapacity } = opts.config;
    const hz = opts.hz ?? 60;
    if (!Number.isInteger(capacity) || capacity < 1 || capacity > MAX_CAPACITY) {
        throw new RangeError(`capacity must be an integer in [1, ${MAX_CAPACITY}], got ${capacity}`);
    }
    if (!Number.isInteger(ringCapacity) || ringCapacity < 1 || (ringCapacity & (ringCapacity - 1)) !== 0) {
        throw new RangeError(`ringCapacity must be a power of two, got ${ringCapacity}`);
    }
    if (!Number.isFinite(hz) || hz <= 0) throw new RangeError(`hz must be > 0, got ${hz}`);
}

export async function launchWorld(opts: LaunchOptions): Promise<World> {
    validate(opts);
    const mode: MemoryMode = detectMode();
    if (mode === 'headers-missing') throw new SharedMemoryUnavailableError('headers-missing');

    if (mode === 'fallback') {
        const { runtime, memory } = await opts.loadFallback(opts.config);
        if (opts.physics) runtime.enable_physics();
        return World.attachSync(opts.config, memory, mode, layoutFromRuntime(runtime), runtime);
    }

    const { memory } = createMemory(opts.initialPages ?? 256, opts.maximumPages ?? 16384);
    const worker = opts.createWorker();
    let world: World | null = null;
    const fail = (message: string) => world?.markFatal(message);
    const layout = await new Promise<WorldLayout>((resolve, reject) => {
        worker.onmessage = (e: MessageEvent<SimMessage>) => {
            if (e.data.type === 'ready') resolve(e.data.layout);
            else if (e.data.type === 'error') { fail(e.data.message); reject(new Error(e.data.message)); }
        };
        worker.onerror = (e) => { fail(e.message); reject(new Error(e.message)); };
        const init: SimInit = {
            type: 'init',
            memory,
            capacity: opts.config.capacity,
            ringCapacity: opts.config.ringCapacity,
            physics: opts.physics ?? false,
            hz: opts.hz ?? 60,
            autoRun: opts.autoRun ?? true,
        };
        worker.postMessage(init);
    }).catch((err) => {
        worker.terminate();
        throw err;
    });
    world = World.attach(opts.config, memory, mode, layout);
    world.setDisposer(() => worker.terminate());
    return world;
}
