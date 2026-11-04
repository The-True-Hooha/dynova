import { describe, it, expect } from 'vitest';
import { World } from '../World.js';
import { makeWorld } from './helpers.js';

const config = { capacity: 128, ringCapacity: 64 };

describe('World', () => {
    it('creates with static factory', () => {
        const world = makeWorld(config).world;
        expect(world.allocatedEntities).toBe(0);
        expect(world.pendingCommands).toBe(0);
    });

    it('spawn returns handle with id=0 gen=0 first', () => {
        const world = makeWorld(config).world;
        expect(world.spawn()).toEqual({ id: 0, gen: 0 });
    });

    it('spawn returns incrementing ids', () => {
        const world = makeWorld(config).world;
        expect(world.spawn().id).toBe(0);
        expect(world.spawn().id).toBe(1);
        expect(world.spawn().id).toBe(2);
    });

    it('spawn enqueues a command', () => {
        const world = makeWorld(config).world;
        world.spawn({ x: 1, y: 2, z: 3 });
        expect(world.pendingCommands).toBe(1);
    });

    it('destroy re-enqueues id with incremented gen', () => {
        const world = makeWorld(config).world;
        const h = world.spawn();
        world.destroy(h);
        const h2 = world.spawn();
        expect(h2.id).toBe(h.id);
        expect(h2.gen).toBe(1);
    });

    it('force enqueues a command', () => {
        const world = makeWorld(config).world;
        const h = world.spawn();
        world.force(h, 0, -9.8, 0);
        expect(world.pendingCommands).toBe(2);
    });

    it('spawn 10k entities without throwing', () => {
        const world = makeWorld({ capacity: 10_000, ringCapacity: 16384 }).world;
        for (let i = 0; i < 10_000; i++) world.spawn();
        expect(world.allocatedEntities).toBe(10_000);
        expect(world.pendingCommands).toBe(10_000);
    });

    it('throws when ring is full', () => {
        const world = makeWorld({ capacity: 128, ringCapacity: 4 }).world;
        world.spawn(); world.spawn(); world.spawn(); world.spawn();
        expect(() => world.spawn()).toThrow('CommandRing full');
    });

    it('backpressureDrops increments on ring-full rejection', () => {
        const world = makeWorld({ capacity: 128, ringCapacity: 2 }).world;
        world.spawn(); world.spawn();
        expect(world.backpressureDrops).toBe(0);
        expect(() => world.spawn()).toThrow();
        expect(world.backpressureDrops).toBe(1);
        expect(() => world.spawn()).toThrow();
        expect(world.backpressureDrops).toBe(2);
    });

    it('destroy of a stale handle throws and enqueues nothing', () => {
        const world = makeWorld(config).world;
        const h = world.spawn();
        world.destroy(h);
        const pending = world.pendingCommands;
        expect(() => world.destroy(h)).toThrow('stale handle');
        expect(world.pendingCommands).toBe(pending);
    });

    it('force on a stale handle throws', () => {
        const world = makeWorld(config).world;
        const h = world.spawn();
        world.destroy(h);
        expect(() => world.force(h, 1, 0, 0)).toThrow('stale handle');
    });

    it('spawn past entity capacity throws RangeError and enqueues nothing', () => {
        const world = makeWorld({ capacity: 2, ringCapacity: 8 }).world;
        world.spawn(); world.spawn();
        expect(() => world.spawn()).toThrow(RangeError);
        expect(world.pendingCommands).toBe(2);
    });

    it('failed ring push returns the id to the allocator', () => {
        const world = makeWorld({ capacity: 4, ringCapacity: 1 }).world;
        const first = world.spawn();
        expect(() => world.spawn()).toThrow('CommandRing full');
        expect(first.id).toBe(0);
        expect(world.allocatedEntities).toBe(2);
    });

    it('step() throws in worker mode', () => {
        expect(() => makeWorld(config).world.step()).toThrow('fallback mode');
    });

    it('fallback step() re-acquires views after the non-shared buffer is detached by growth', () => {
        const { layout, memory } = makeWorld({ capacity: 4, ringCapacity: 4 });
        const plain = new WebAssembly.Memory({ initial: 1, maximum: 4 });
        new Uint8Array(plain.buffer).set(new Uint8Array(memory.buffer.slice(0, 65536)));
        let calls = 0;
        const world = World.attachSync({ capacity: 4, ringCapacity: 4 }, plain, 'fallback', layout, {
            step: () => { calls++; plain.grow(1); },
        });
        world.spawn();
        world.step();
        expect(calls).toBe(1);
        expect(() => world.spawn()).not.toThrow();
        expect(() => world.readTransforms()[0]).not.toThrow();
        expect(world.pendingCommands).toBe(2);
    });

    it('reacquireIfGrown is a no-op when the buffer did not change', () => {
        const { world } = makeWorld(config);
        expect(world.reacquireIfGrown()).toBe(false);
    });

    it('ring-full rollback keeps the generation so the retried spawn is not stale', () => {
        const { world, memory, layout } = makeWorld({ capacity: 4, ringCapacity: 1 });
        world.spawn();
        const head = new Int32Array(memory.buffer, layout.ringHeadByteOffset, 1);
        expect(() => world.spawn()).toThrow('CommandRing full');
        Atomics.store(head, 0, 1);
        const retried = world.spawn();
        expect(retried).toEqual({ id: 1, gen: 0 });
    });

    it('dispose() runs the disposer once and blocks further commands', () => {
        const { world } = makeWorld(config);
        let calls = 0;
        world.setDisposer(() => { calls++; });
        world.dispose();
        world.dispose();
        expect(calls).toBe(1);
        expect(() => world.spawn()).toThrow('world unavailable');
    });

    it('markFatal makes spawn/destroy/force throw and keeps the first message', () => {
        const { world } = makeWorld(config);
        const h = world.spawn();
        world.markFatal('unreachable executed');
        world.markFatal('second');
        expect(world.failure).toBe('unreachable executed');
        expect(() => world.spawn()).toThrow('unreachable executed');
        expect(() => world.destroy(h)).toThrow('unreachable executed');
        expect(() => world.force(h, 0, 0, 0)).toThrow('unreachable executed');
    });

    it('fallback spawn after out-of-band growth re-acquires instead of writing a detached view', () => {
        const { layout, memory } = makeWorld({ capacity: 4, ringCapacity: 4 });
        const plain = new WebAssembly.Memory({ initial: 1, maximum: 4 });
        new Uint8Array(plain.buffer).set(new Uint8Array(memory.buffer.slice(0, 65536)));
        const world = World.attachSync({ capacity: 4, ringCapacity: 4 }, plain, 'fallback', layout, { step() {} });
        plain.grow(1);
        expect(() => world.spawn()).not.toThrow();
        plain.grow(1);
        expect(() => world.readTransforms()).not.toThrow();
    });
});
