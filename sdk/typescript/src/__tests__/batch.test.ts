import { describe, it, expect, vi } from 'vitest';
import { readFileSync } from 'node:fs';
import { World, layoutFromRuntime } from '../World.js';
import { WorldBatch, type BatchRuntime } from '../batch.js';
import { SHAPE_SPHERE } from '../CommandRing.js';

// The real single-threaded wasm module (make build-st), not a stand-in: the batch is a wasm export.
const glue = await import('../../wasm-st/dynova_core.js');
const wasm = glue.initSync({ module: readFileSync(new URL('../../wasm-st/dynova_core_bg.wasm', import.meta.url)) });
const config = { capacity: 16, ringCapacity: 64 };

function batchOf(worlds: number): { batch: BatchRuntime; set: WorldBatch } {
    const batch = new glue.Batch(worlds, config.capacity, config.ringCapacity);
    batch.enable_physics();
    return { batch, set: new WorldBatch(config, wasm.memory, 'fallback', batch) };
}

function alone(drop: number, force: number | null, ticks: number): Float32Array {
    const rt = new glue.Runtime(config.capacity, config.ringCapacity);
    rt.enable_physics();
    const w = World.attachSync(config, wasm.memory, 'fallback', layoutFromRuntime(rt), rt);
    scene(w, drop, force);
    for (let i = 0; i < ticks; i++) w.step();
    return w.readTransforms().slice();
}

function scene(w: World, drop: number, force: number | null) {
    w.spawn({ x: 0, y: -1, z: 0, hx: 20, hy: 1, hz: 20, invMass: 0 });
    const ball = w.spawn({ x: 0, y: drop, z: 0, shape: SHAPE_SPHERE, hx: 0.5, invMass: 1 });
    if (force !== null) w.force(ball, force, 0, 0);
}

describe('a batch of real wasm worlds', () => {
    it('steps every world to the state the same world reaches alone', () => {
        const drops = [3, 4, 5, 6];
        const { set } = batchOf(drops.length);
        drops.forEach((d, i) => scene(set.worlds[i], d, null));
        for (let t = 0; t < 40; t++) set.step();
        drops.forEach((d, i) => {
            expect(Array.from(set.worlds[i].readTransforms()), `world ${i}`).toEqual(Array.from(alone(d, null, 40)));
        });
        expect(set.worlds[0].readTransforms()[8]).not.toBe(set.worlds[3].readTransforms()[8]);
    });

    it('a force sent to one world moves that world only', () => {
        const { set } = batchOf(3);
        set.worlds.forEach((w, i) => scene(w, 2, i === 1 ? 50 : null));
        for (let t = 0; t < 20; t++) set.step();
        const x = (i: number) => set.worlds[i].readTransforms()[7];
        expect(x(0)).toBe(0);
        expect(x(2)).toBe(0);
        expect(x(1)).toBeGreaterThan(0.1);
        expect(Array.from(set.worlds[1].readTransforms())).toEqual(Array.from(alone(2, 50, 20)));
    });

    it('each world allocates its own entity ids from zero', () => {
        const { set } = batchOf(2);
        const a = set.worlds[0].spawn({ x: 0, y: 5, z: 0, hx: 0.5, invMass: 1 });
        const b = set.worlds[1].spawn({ x: 0, y: 5, z: 0, hx: 0.5, invMass: 1 });
        expect([a.id, b.id]).toEqual([0, 0]);
    });

    it('rejects sizes outside the budget and layouts past the last world', () => {
        expect(() => new glue.Batch(0, 8, 16)).toThrow(/at least one world/);
        expect(() => new glue.Batch(1, 8, 12)).toThrow(/power of two/);
        expect(() => new glue.Batch(4_000_000_000, 8, 16)).toThrow(/MiB/);
        // One body each is still 64 KiB of tables per world: this used to trap the module instead of throwing.
        expect(() => new glue.Batch(524_288, 1, 1)).toThrow(/MiB/);
        expect(() => new glue.Batch(100_000, 1, 1)).toThrow(/MiB/);
        const batch = new glue.Batch(2, 8, 16);
        expect(batch.world_layout(2)).toBeUndefined();
        expect(batch.world_layout(2 ** 32 - 1)).toBeUndefined();
    });

    it('a stale wasm header size fails loudly instead of reading the wrong words', () => {
        const fake: BatchRuntime = { world_count: () => 1, enable_physics() {}, step_all() {}, world_layout: () => new Uint32Array([0, 999, 0, 0, 0, 0, 64]) };
        expect(() => new WorldBatch(config, wasm.memory, 'fallback', fake)).toThrow(/rebuild the wasm package/);
    });

    it('a ring larger than the real one is refused before it can write into the next world', () => {
        const batch = new glue.Batch(2, 8, 16);
        expect(() => new WorldBatch({ capacity: 8, ringCapacity: 1024 }, wasm.memory, 'fallback', batch)).toThrow(/ring of 16 slots/);
    });

    it('one world growing keeps its own state and leaves the others as they were', () => {
        const { set } = batchOf(2);
        scene(set.worlds[0], 3, null);
        scene(set.worlds[1], 3, null);
        set.worlds[0].requestGrow(48);
        for (let t = 0; t < 10; t++) set.step();
        // copies: a later wasm allocation may grow the memory and detach a view
        const a = set.worlds[0].readTransforms().slice();
        const b = set.worlds[1].readTransforms().slice();
        expect(a.length / 7).toBe(48);
        expect(b.length / 7).toBe(config.capacity);
        expect(Array.from(b)).toEqual(Array.from(alone(3, null, 10)));
        expect(Array.from(a.slice(0, 14))).toEqual(Array.from(b.slice(0, 14)));
    });

    it('every step finishes every world once, with the batch time shared out', () => {
        const { set } = batchOf(4);
        const spies = set.worlds.map((w) => vi.spyOn(w, 'finishStep'));
        const now = vi.spyOn(performance, 'now');
        now.mockReturnValueOnce(1000).mockReturnValueOnce(1800);
        set.step();
        now.mockRestore();
        spies.forEach((spy) => {
            expect(spy).toHaveBeenCalledTimes(1);
            expect(spy.mock.calls[0][0]).toBeCloseTo(200_000, 6);
        });
    });

    it('an error finishing one world does not stop the others finishing, and is rethrown', () => {
        const { set } = batchOf(3);
        vi.spyOn(set.worlds[0], 'finishStep').mockImplementation(() => { throw new Error('world 0 broke'); });
        const rest = [1, 2].map((i) => vi.spyOn(set.worlds[i], 'finishStep'));
        expect(() => set.step()).toThrow('world 0 broke');
        rest.forEach((spy) => expect(spy).toHaveBeenCalledTimes(1));
    });

    it('a trap in the wasm step fails every world and rethrows; dispose fails them too and frees the batch', () => {
        let freed = 0;
        const real = new glue.Batch(2, config.capacity, config.ringCapacity);
        const trapping: BatchRuntime = {
            world_count: () => real.world_count(),
            enable_physics: () => real.enable_physics(),
            world_layout: (i) => real.world_layout(i),
            step_all: () => { throw new Error('unreachable'); },
            free: () => { freed++; },
        };
        const set = new WorldBatch(config, wasm.memory, 'fallback', trapping);
        expect(() => set.step()).toThrow('unreachable');
        set.worlds.forEach((w) => expect(w.failure).toMatch(/trapped/));
        const fresh = new WorldBatch(config, wasm.memory, 'fallback', trapping);
        fresh.dispose();
        expect(freed).toBe(1);
        fresh.worlds.forEach((w) => expect(w.failure).toBe('batch disposed'));
    });

    it('stepping a world of a batch on its own is refused with a message that names the batch', () => {
        const { set } = batchOf(1);
        expect(() => set.worlds[0].step()).toThrow(/WorldBatch/);
    });
});
