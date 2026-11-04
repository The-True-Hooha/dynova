import { describe, it, expect, afterEach } from 'vitest';
import { launchWorld, SharedMemoryUnavailableError, type FallbackRuntime } from '../launch.js';
import { fakeLayout } from './helpers.js';

const config = { capacity: 8, ringCapacity: 4 };
const realSab = globalThis.SharedArrayBuffer;
afterEach(() => { globalThis.SharedArrayBuffer = realSab; });

function fakeRuntime(): FallbackRuntime & { stepped: number; physics: boolean } {
    const { layout } = fakeLayout(config);
    return {
        stepped: 0,
        physics: false,
        step() { this.stepped++; },
        enable_physics() { this.physics = true; },
        transforms_buf_a_ptr: () => layout.transformAByteOffset,
        transforms_buf_b_ptr: () => layout.transformBByteOffset,
        transforms_write_index_ptr: () => layout.writeIndexByteOffset,
        ring_slots_ptr: () => layout.ringSlotsByteOffset,
        ring_head_ptr: () => layout.ringHeadByteOffset,
        ring_tail_ptr: () => layout.ringTailByteOffset,
        id_counter_ptr: () => layout.idCounterByteOffset,
        generations_ptr: () => layout.generationsByteOffset,
    };
}

const neverWorker = () => { throw new Error('worker must not be created'); };

describe('launchWorld', () => {
    it('headers-missing throws a config error naming COOP and COEP, creating nothing', async () => {
        globalThis.SharedArrayBuffer = class { constructor() { throw new Error('blocked'); } } as never;
        let loaded = false;
        const p = launchWorld({ config, createWorker: neverWorker, loadFallback: async () => { loaded = true; throw new Error('no'); } });
        await expect(p).rejects.toBeInstanceOf(SharedMemoryUnavailableError);
        await expect(p).rejects.toThrow(/Cross-Origin-Opener-Policy.*Cross-Origin-Embedder-Policy/);
        expect(loaded).toBe(false);
    });

    it('no SharedArrayBuffer uses the sync fallback and enables physics on request', async () => {
        // @ts-expect-error simulate an old browser
        globalThis.SharedArrayBuffer = undefined;
        const rt = fakeRuntime();
        const memory = new WebAssembly.Memory({ initial: 1, maximum: 2 });
        const world = await launchWorld({
            config, physics: true, createWorker: neverWorker,
            loadFallback: async () => ({ runtime: rt, memory }),
        });
        expect(world.mode).toBe('fallback');
        expect(rt.physics).toBe(true);
        world.step();
        expect(rt.stepped).toBe(1);
    });

    it('fallback load failure rejects instead of hanging', async () => {
        // @ts-expect-error simulate an old browser
        globalThis.SharedArrayBuffer = undefined;
        await expect(launchWorld({
            config, createWorker: neverWorker,
            loadFallback: async () => { throw new Error('wasm 404'); },
        })).rejects.toThrow('wasm 404');
    });

    it('worker error message rejects the launch', async () => {
        const worker = { postMessage() { this.onmessage?.({ data: { type: 'error', message: 'bad wasm' } }); }, terminate() {}, onmessage: null as ((e: unknown) => void) | null, onerror: null };
        await expect(launchWorld({
            config, createWorker: () => worker as unknown as Worker,
            loadFallback: async () => { throw new Error('unused'); },
        })).rejects.toThrow('bad wasm');
    });

    it('worker ready message resolves to a shared-mode world using the posted layout', async () => {
        const { layout } = fakeLayout(config);
        const worker = { postMessage() { this.onmessage?.({ data: { type: 'ready', layout } }); }, onmessage: null as ((e: unknown) => void) | null, onerror: null };
        const world = await launchWorld({
            config, initialPages: 1, maximumPages: 1, createWorker: () => worker as unknown as Worker,
            loadFallback: async () => { throw new Error('unused'); },
        });
        expect(world.mode).toBe('shared');
        expect(world.layout).toEqual(layout);
    });

    it.each([
        ['ringCapacity 3', { capacity: 8, ringCapacity: 3 }, undefined],
        ['ringCapacity 0', { capacity: 8, ringCapacity: 0 }, undefined],
        ['capacity 0', { capacity: 0, ringCapacity: 4 }, undefined],
        ['capacity 1.5', { capacity: 1.5, ringCapacity: 4 }, undefined],
        ['hz 0', { capacity: 8, ringCapacity: 4 }, 0],
        ['hz NaN', { capacity: 8, ringCapacity: 4 }, NaN],
    ])('rejects invalid config before touching a worker: %s', async (_n, cfg, hz) => {
        await expect(launchWorld({ config: cfg, hz, createWorker: neverWorker, loadFallback: async () => { throw new Error('x'); } }))
            .rejects.toBeInstanceOf(RangeError);
    });

    it('terminates the worker when it reports an error during startup', async () => {
        let terminated = 0;
        const worker = {
            postMessage() { this.onmessage?.({ data: { type: 'error', message: 'boom' } }); },
            terminate() { terminated++; },
            onmessage: null as ((e: unknown) => void) | null, onerror: null,
        };
        await expect(launchWorld({ config, createWorker: () => worker as unknown as Worker, loadFallback: async () => { throw new Error('u'); } }))
            .rejects.toThrow('boom');
        expect(terminated).toBe(1);
    });

    it('worker error after ready marks the world dead; dispose terminates the worker', async () => {
        const { layout } = fakeLayout(config);
        let terminated = 0;
        const worker = {
            postMessage() { this.onmessage?.({ data: { type: 'ready', layout } }); },
            terminate() { terminated++; },
            onmessage: null as ((e: unknown) => void) | null, onerror: null,
        };
        const world = await launchWorld({
            config, initialPages: 1, maximumPages: 1, createWorker: () => worker as unknown as Worker,
            loadFallback: async () => { throw new Error('u'); },
        });
        worker.onmessage?.({ data: { type: 'error', message: 'unreachable executed' } });
        expect(world.failure).toBe('unreachable executed');
        expect(() => world.spawn()).toThrow('unreachable executed');
        world.dispose();
        expect(terminated).toBe(1);
    });
});
