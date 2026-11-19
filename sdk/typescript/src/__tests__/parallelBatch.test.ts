import { describe, it, expect, vi } from 'vitest';
import { readFileSync } from 'node:fs';
import { World, layoutFromRuntime } from '../World.js';
import { ParallelBatch, partition, type BatchWorker } from '../parallelBatch.js';
import { createBatchHost, type BatchReply, type BatchRequest, type HostBatch } from '../batchHost.js';
import { SHAPE_SPHERE } from '../CommandRing.js';

const glue = await import('../../wasm-st/dynova_core.js');
const wasm = glue.initSync({ module: readFileSync(new URL('../../wasm-st/dynova_core_bg.wasm', import.meta.url)) });
const config = { capacity: 16, ringCapacity: 64 };

/** A worker that runs the real `createBatchHost` over the real wasm in this thread; `wrap` lets a test change how a world steps. */
class InProcessWorker implements BatchWorker {
    onmessage: ((event: { data: BatchReply }) => void) | null = null;
    onerror: ((event: { message: string }) => void) | null = null;
    terminated = 0;
    /** A worker that never answers a step. */
    mute = false;
    tamper: (reply: BatchReply) => BatchReply = (r) => r;
    private host: (request: BatchRequest) => void;

    constructor(wrap: (batch: HostBatch) => HostBatch = (b) => b, now?: () => number, forcedRing?: number) {
        this.host = createBatchHost(
            (w, c, r) => wrap(new glue.Batch(w, c, forcedRing ?? r) as unknown as HostBatch),
            (reply) => queueMicrotask(() => !(this.mute && reply.type === 'stepped') && this.onmessage?.({ data: this.tamper(reply.type === 'ready' ? { ...reply, memory: wasm.memory } : reply) })),
            now,
        );
    }

    postMessage(request: BatchRequest): void { this.host(request); }
    terminate(): void { this.terminated++; }
}

function scene(w: World, i: number) {
    w.spawn({ x: 0, y: -1, z: 0, hx: 20, hy: 1, hz: 20, invMass: 0 });
    w.spawn({ x: 0, y: 3 + i, z: 0, shape: SHAPE_SPHERE, hx: 0.5, invMass: 1 });
}

function alone(i: number, ticks: number): number[] {
    const rt = new glue.Runtime(config.capacity, config.ringCapacity);
    rt.enable_physics();
    const w = World.attachSync(config, wasm.memory, 'fallback', layoutFromRuntime(rt), rt);
    scene(w, i);
    for (let t = 0; t < ticks; t++) w.step();
    return Array.from(w.readTransforms().slice(0, 14));
}

const launch = (worlds: number, workers: InProcessWorker[], cfg = config) =>
    ParallelBatch.launch({ config: cfg, worlds, workers: workers.length, physics: true, createWorker: () => workers[made++] });
let made = 0;
const fresh = (n: number, make: (k: number) => InProcessWorker = () => new InProcessWorker()) => { made = 0; return Array.from({ length: n }, (_, k) => make(k)); };

describe('partition', () => {
    it('shares worlds out contiguously and evenly, never leaving a worker empty', () => {
        expect(partition(10, 4)).toEqual([3, 3, 2, 2]);
        expect(partition(3, 8)).toEqual([1, 1, 1]);
        expect(partition(1, 1)).toEqual([1]);
        expect(partition(256, 7).reduce((a, b) => a + b, 0)).toBe(256);
        for (const c of [partition(256, 7), partition(9, 2)]) expect(Math.max(...c) - Math.min(...c)).toBeLessThanOrEqual(1);
    });

    it('rejects counts that are not positive integers', () => {
        for (const bad of [0, -1, 1.5, NaN, Infinity]) {
            expect(() => partition(bad, 2)).toThrow(RangeError);
            expect(() => partition(2, bad)).toThrow(RangeError);
        }
    });
});

describe('a ParallelBatch over in-process workers running the real wasm', () => {
    it('every world, whichever worker holds it, ends as the same world run alone', async () => {
        const workers = fresh(3);
        const batch = await launch(7, workers);
        expect(batch.size).toBe(7);
        expect(batch.workerCount).toBe(3);
        batch.worlds.forEach(scene);
        for (let t = 0; t < 40; t++) expect(await batch.step()).toEqual([]);
        batch.worlds.forEach((w, i) => expect(Array.from(w.readTransforms().slice(0, 14)), `world ${i}`).toEqual(alone(i, 40)));
        expect(alone(0, 40)).not.toEqual(alone(6, 40));
    });

    it('records each world its own measured step time', async () => {
        let clock = 0;
        const workers = fresh(2, () => new InProcessWorker(
            (b) => ({ ...b, world_count: () => b.world_count(), world_layout: (i) => b.world_layout(i), enable_physics: () => b.enable_physics(), step_all: () => b.step_all(), step_world: (i) => { clock += (i + 1) * 2; return b.step_world(i); } }),
            () => clock,
        ));
        const batch = await launch(5, workers);
        const spies = batch.worlds.map((w) => vi.spyOn(w, 'finishStep'));
        await batch.step();
        // Local world k of a worker advances the clock by 2 * (k + 1) ms; the host reports microseconds.
        expect(spies.map((s) => s.mock.calls[0][0])).toEqual([2000, 4000, 6000, 2000, 4000]);
    });

    it('a worker that traps fails its worlds only, and the others keep stepping', async () => {
        const workers = fresh(3, (k) => new InProcessWorker(k === 1 ? (b) => ({ ...b, world_count: () => b.world_count(), world_layout: (i) => b.world_layout(i), enable_physics: () => b.enable_physics(), step_all: () => b.step_all(), step_world: () => { throw new Error('unreachable'); } }) : undefined));
        const batch = await launch(6, workers);
        batch.worlds.forEach(scene);
        expect(await batch.step()).toEqual([2, 3]);
        expect(await batch.step()).toEqual([2, 3]);
        batch.worlds.forEach((w, i) => {
            if (i === 2 || i === 3) {
                expect(w.failure).toMatch(/worker failed: unreachable/);
                expect(() => w.spawn({})).toThrow(/worker failed/);
            } else {
                expect(w.failure).toBeNull();
                expect(Array.from(w.readTransforms().slice(0, 14)), `world ${i}`).toEqual(alone(i, 2));
            }
        });
        expect(workers.map((w) => w.terminated)).toEqual([0, 1, 0]);
    });

    it('rejects once every worker has failed', async () => {
        const broken = (b: HostBatch): HostBatch => ({ ...b, world_count: () => b.world_count(), world_layout: (i) => b.world_layout(i), enable_physics: () => b.enable_physics(), step_all: () => b.step_all(), step_world: () => { throw new Error('boom'); } });
        const workers = fresh(2, () => new InProcessWorker(broken));
        const batch = await launch(2, workers);
        expect(await batch.step()).toEqual([0, 1]);
        await expect(batch.step()).rejects.toThrow('every worker has failed');
    });

    it('an error while finishing one world still lets every other world finish, then is thrown', async () => {
        const workers = fresh(2);
        const batch = await launch(4, workers);
        vi.spyOn(batch.worlds[0], 'finishStep').mockImplementation(() => { throw new Error('world 0 broke'); });
        const rest = [1, 2, 3].map((i) => vi.spyOn(batch.worlds[i], 'finishStep'));
        await expect(batch.step()).rejects.toThrow('world 0 broke');
        rest.forEach((s) => expect(s).toHaveBeenCalledTimes(1));
    });

    it('does not start a step while one is running', async () => {
        const workers = fresh(1);
        const batch = await launch(1, workers);
        const first = batch.step();
        await expect(batch.step()).rejects.toThrow('already running');
        await first;
        await expect(batch.step()).resolves.toEqual([]);
    });

    it('dispose terminates every worker once and fails every world', async () => {
        const workers = fresh(3);
        const batch = await launch(5, workers);
        batch.dispose();
        expect(workers.map((w) => w.terminated)).toEqual([1, 1, 1]);
        batch.worlds.forEach((w) => expect(w.failure).toBe('batch disposed'));
    });

    it('a worker that cannot build its share fails the launch and terminates every worker made so far', async () => {
        const workers = fresh(3, (k) => new InProcessWorker(k === 2 ? () => { throw new Error('the memory for this batch is not available'); } : undefined));
        await expect(launch(6, workers)).rejects.toThrow('not available');
        expect(workers.map((w) => w.terminated)).toEqual([1, 1, 1]);
    });

    it('refuses a config whose ring is not the ring the worker built', async () => {
        const workers = fresh(1, () => new InProcessWorker(undefined, undefined, 64));
        await expect(launch(1, workers, { capacity: 16, ringCapacity: 1024 })).rejects.toThrow(/ring of 64 slots/);
        expect(workers[0].terminated).toBe(1);
    });

    it('a worker error event fails its worlds like a trap does', async () => {
        const workers = fresh(2);
        const batch = await launch(2, workers);
        const pending = batch.step();
        workers[1].onerror?.({ message: 'worker died' });
        expect(await pending).toEqual([1]);
        expect(batch.worlds[1].failure).toMatch(/worker died/);
    });

    it('dispose while a step is waiting resolves that step instead of leaving it pending, and terminates each worker once', async () => {
        const workers = fresh(2);
        const batch = await launch(4, workers);
        workers.forEach((w) => { w.mute = true; });
        const pending = batch.step();
        batch.dispose();
        expect(await pending).toEqual([0, 1, 2, 3]);
        expect(workers.map((w) => w.terminated)).toEqual([1, 1]);
        await expect(batch.step()).rejects.toThrow('every worker has failed');
    });

    it('a worker that cannot get its memory is not left running when the launch fails', async () => {
        const made: InProcessWorker[] = [];
        await expect(ParallelBatch.launch({
            config, worlds: 2, workers: 2, initialPages: 2, maximumPages: 1,
            createWorker: () => { const w = new InProcessWorker(); made.push(w); return w; },
        })).rejects.toThrow();
        expect(made.filter((w) => w.terminated === 0), 'no worker left alive').toEqual([]);
    });

    it('a worker that fails between steps is failed at once, and the next step skips it', async () => {
        const workers = fresh(2);
        const batch = await launch(4, workers);
        batch.worlds.forEach(scene);
        workers[1].onerror?.({ message: 'died between steps' });
        expect(batch.worlds[2].failure).toMatch(/died between steps/);
        expect(batch.worlds[0].failure).toBeNull();
        expect(workers[1].terminated).toBe(1);
        expect(await batch.step()).toEqual([2, 3]);
        expect(Array.from(batch.worlds[0].readTransforms().slice(0, 14))).toEqual(alone(0, 1));
    });

    it('an error message that arrives between steps fails the worker too', async () => {
        const workers = fresh(2);
        const batch = await launch(2, workers);
        workers[0].onmessage?.({ data: { type: 'error', message: 'out of memory' } });
        expect(batch.worlds[0].failure).toMatch(/out of memory/);
        expect(await batch.step()).toEqual([0]);
    });

    it('a step reply with the wrong number of times fails that worker', async () => {
        const workers = fresh(2);
        workers[1].tamper = (r) => (r.type === 'stepped' ? { ...r, micros: r.micros.slice(1) } : r);
        const batch = await launch(4, workers);
        expect(await batch.step()).toEqual([2, 3]);
        expect(batch.worlds[2].failure).toMatch(/times for 2 worlds/);
    });

    it('a world its owner failed is skipped by the step, which still finishes the healthy worlds', async () => {
        const workers = fresh(2);
        const batch = await launch(4, workers);
        batch.worlds[0].markFatal('owner gave up');
        const skipped = vi.spyOn(batch.worlds[0], 'finishStep');
        const finished = vi.spyOn(batch.worlds[1], 'finishStep');
        expect(await batch.step()).toEqual([]);
        expect(skipped).not.toHaveBeenCalled();
        expect(finished).toHaveBeenCalledTimes(1);
    });
});

describe('the batch host', () => {
    const fakeBatch = (stepWorld: (i: number) => boolean): HostBatch => ({
        world_count: () => 2, enable_physics() {}, step_all() {}, world_layout: () => new Uint32Array(7), step_world: stepWorld,
    });
    const init = { type: 'init', memory: wasm.memory, worlds: 2, capacity: 8, ringCapacity: 16, physics: false } as const;

    it('treats a world the batch does not have as a failure and stays failed afterwards', () => {
        const replies: BatchReply[] = [];
        const host = createBatchHost(() => fakeBatch((i) => i === 0), (r) => replies.push(r));
        host(init);
        host({ type: 'step' });
        host({ type: 'step' });
        expect(replies.map((r) => r.type)).toEqual(['ready', 'error', 'error']);
        expect(replies[1]).toMatchObject({ message: 'world 1 is not in the batch' });
        expect(replies[2]).toMatchObject({ message: 'this worker has already failed' });
    });

    it('answers a step before init with an error', () => {
        const replies: BatchReply[] = [];
        createBatchHost(() => fakeBatch(() => true), (r) => replies.push(r))({ type: 'step' });
        expect(replies).toEqual([{ type: 'error', message: 'step before init' }]);
    });
});
