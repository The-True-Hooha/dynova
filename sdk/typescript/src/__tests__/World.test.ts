import { describe, it, expect } from 'vitest';
import { World } from '../World.js';
import { makeWorld, makeSyncWorld } from './helpers.js';

const config = { capacity: 128, ringCapacity: 64 };
const tight = { capacity: 8, ringCapacity: 2, maxDeferredCommands: 0 };

describe('World basics', () => {
    it('starts empty', () => {
        const { world } = makeWorld(config);
        expect(world.allocatedEntities).toBe(0);
        expect(world.pendingCommands).toBe(0);
        expect(world.capacity).toBe(128);
    });

    it('spawn returns incrementing handles starting at id 0 gen 0', () => {
        const { world } = makeWorld(config);
        expect(world.spawn()).toEqual({ id: 0, gen: 0 });
        expect(world.spawn().id).toBe(1);
        expect(world.spawn().id).toBe(2);
    });

    it('spawn, force and destroy enqueue commands', () => {
        const { world } = makeWorld(config);
        const h = world.spawn({ x: 1, y: 2, z: 3 });
        world.force(h, 0, -9.8, 0);
        world.destroy(h);
        expect(world.pendingCommands).toBe(3);
    });

    it('destroy then spawn reuses the id with the next generation', () => {
        const { world } = makeWorld(config);
        const h = world.spawn();
        world.destroy(h);
        expect(world.spawn()).toEqual({ id: h.id, gen: 1 });
    });

    it('spawn 10k entities without throwing', () => {
        const { world } = makeWorld({ capacity: 10_000, ringCapacity: 16384 }, { pages: 64 });
        for (let i = 0; i < 10_000; i++) world.spawn();
        expect(world.allocatedEntities).toBe(10_000);
        expect(world.pendingCommands).toBe(10_000);
    });

    it('destroy of a stale handle throws and enqueues nothing', () => {
        const { world } = makeWorld(config);
        const h = world.spawn();
        world.destroy(h);
        const pending = world.pendingCommands;
        expect(() => world.destroy(h)).toThrow('stale handle');
        expect(world.pendingCommands).toBe(pending);
    });

    it('force on a stale handle throws', () => {
        const { world } = makeWorld(config);
        const h = world.spawn();
        world.destroy(h);
        expect(() => world.force(h, 1, 0, 0)).toThrow('stale handle');
    });

    it('spawn past entity capacity throws RangeError and enqueues nothing', () => {
        const { world } = makeWorld({ capacity: 2, ringCapacity: 8 });
        world.spawn(); world.spawn();
        expect(() => world.spawn()).toThrow(RangeError);
        expect(world.pendingCommands).toBe(2);
    });

    it('step() throws in worker mode', () => {
        expect(() => makeWorld(config).world.step()).toThrow('fallback mode');
    });

    it('dispose() runs the disposer once and blocks further commands', () => {
        const { world } = makeWorld(config);
        let calls = 0;
        world.setDisposer(() => { calls++; });
        world.dispose();
        world.dispose();
        expect(calls).toBe(1);
        expect(() => world.spawn()).toThrow('world unavailable');
        expect(() => world.readTransforms()).toThrow('world unavailable');
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
});

describe('World backpressure (bounded deferral)', () => {
    it('with no deferral allowed, a full ring throws and counts a drop per rejection', () => {
        const { world } = makeWorld(tight);
        world.spawn(); world.spawn();
        expect(world.backpressureDrops).toBe(0);
        expect(() => world.spawn()).toThrow('CommandRing full');
        expect(world.backpressureDrops).toBe(1);
        expect(() => world.spawn()).toThrow();
        expect(world.backpressureDrops).toBe(2);
    });

    it('ring-full rollback keeps the generation so the retried spawn is not stale', () => {
        const { world, rt } = makeWorld({ capacity: 4, ringCapacity: 1, maxDeferredCommands: 0 });
        world.spawn();
        expect(() => world.spawn()).toThrow('CommandRing full');
        rt.drain();
        expect(world.spawn()).toEqual({ id: 1, gen: 0 });
    });

    it('defers when the ring is full, keeps order, and nothing is dropped', () => {
        const { world, rt } = makeWorld({ capacity: 16, ringCapacity: 2, maxDeferredCommands: 100 });
        const hs = Array.from({ length: 10 }, (_, i) => world.spawn({ x: i }));
        expect(world.deferredCommands).toBe(8);
        expect(world.totalDeferred).toBe(8);
        expect(world.backpressureDrops).toBe(0);
        const seen: number[] = [];
        for (let i = 0; i < 20 && world.pendingCommands > 0; i++) {
            rt.drain();
            const buf = world.readTransforms();
            for (let i = 0; i < 10; i++) if (buf[i * 7 + 6] === 1 && !seen.includes(i)) seen.push(i);
        }
        expect(seen.sort((a, b) => a - b)).toEqual(hs.map((h) => h.id));
        expect(world.deferredCommands).toBe(0);
    });

    it('throws once the deferred queue itself is full and releases the id', () => {
        const { world } = makeWorld({ capacity: 16, ringCapacity: 1, maxDeferredCommands: 2 });
        for (let i = 0; i < 3; i++) world.spawn();
        expect(() => world.spawn()).toThrow('CommandRing full');
        expect(world.backpressureDrops).toBe(1);
        expect(world.deferredCommands).toBe(2);
    });

    it('new commands queue behind deferred ones even when the ring has room again', () => {
        const { world, rt } = makeWorld({ capacity: 16, ringCapacity: 2, maxDeferredCommands: 100 });
        world.spawn({ x: 1 }); world.spawn({ x: 2 }); world.spawn({ x: 3 });
        rt.drain();
        world.spawn({ x: 4 });
        rt.drain();
        rt.drain();
        const buf = world.readTransforms();
        expect([0, 1, 2, 3].map((i) => buf[i * 7])).toEqual([1, 2, 3, 4]);
    });

    it('destroy of an entity whose spawn is still deferred preserves spawn-then-destroy order', () => {
        const { world, rt } = makeWorld({ capacity: 16, ringCapacity: 1, maxDeferredCommands: 100 });
        world.spawn();
        const h = world.spawn({ x: 5 });
        world.destroy(h);
        for (let i = 0; i < 10 && world.pendingCommands > 0; i++) { rt.drain(); world.flush(); }
        expect(world.pendingCommands).toBe(0);
        const buf = world.readTransforms();
        expect(buf[7 + 6]).toBe(0);
    });

    it('flush reports how many commands remain deferred', () => {
        const { world, rt } = makeWorld({ capacity: 16, ringCapacity: 2, maxDeferredCommands: 100 });
        for (let i = 0; i < 5; i++) world.spawn();
        expect(world.flush()).toBe(3);
        rt.drain();
        expect(world.flush()).toBe(1);
        rt.drain();
        expect(world.flush()).toBe(0);
    });
});

describe('World growth', () => {
    it('requestGrow reaches the runtime; capacity and the event update when the world observes it', () => {
        const { world, rt } = makeWorld({ capacity: 2, ringCapacity: 8 });
        const events: unknown[] = [];
        world.onBufferGrowth = (e) => events.push(e);
        world.spawn(); world.spawn();
        expect(() => world.spawn()).toThrow(RangeError);
        world.requestGrow(6);
        expect(world.capacity).toBe(2);
        rt.drain();
        world.readTransforms();
        expect(world.capacity).toBe(6);
        expect(events).toEqual([{ epoch: 1, capacity: 6 }]);
        expect(world.growthEvents).toBe(1);
        expect(() => { for (let i = 0; i < 4; i++) world.spawn(); }).not.toThrow();
    });

    it('spawn observes growth on its own, without a read', () => {
        const { world, rt } = makeWorld({ capacity: 1, ringCapacity: 8 });
        world.spawn();
        world.requestGrow(3);
        rt.drain();
        expect(() => world.spawn()).not.toThrow();
        expect(world.capacity).toBe(3);
    });

    it('transforms written before growth are still readable after it, at their old ids', () => {
        const { world, rt } = makeWorld({ capacity: 2, ringCapacity: 8 });
        world.spawn({ x: 11 }); world.spawn({ x: 22 });
        rt.drain();
        world.requestGrow(4);
        rt.drain();
        const buf = world.readTransforms();
        expect(buf[0]).toBe(11);
        expect(buf[7]).toBe(22);
        expect(buf.length).toBe(4 * 7);
    });

    it.each([[2], [1], [0], [-5], [2.5], [NaN], [Infinity], [(1 << 20) + 1]])('requestGrow(%s) is rejected up front', (n) => {
        const { world } = makeWorld({ capacity: 2, ringCapacity: 8 });
        expect(() => world.requestGrow(n)).toThrow(RangeError);
        expect(world.pendingCommands).toBe(0);
    });

    it('repeated growth keeps firing events with increasing epochs', () => {
        const { world, rt } = makeWorld({ capacity: 1, ringCapacity: 8 }, { pages: 16 });
        const epochs: number[] = [];
        world.onBufferGrowth = (e) => epochs.push(e.epoch);
        for (const c of [2, 4, 8]) { world.requestGrow(c); rt.drain(); world.readTransforms(); }
        expect(epochs).toEqual([1, 2, 3]);
        expect(world.capacity).toBe(8);
    });

    it('fallback step() re-acquires views after non-shared memory growth detaches the buffer', () => {
        const { world, memory } = makeSyncWorld({ capacity: 4, ringCapacity: 4 }, () => {});
        world.spawn();
        const before = memory.buffer;
        memory.grow(1);
        expect(before.byteLength).toBe(0);
        world.step();
        expect(() => world.spawn()).not.toThrow();
        expect(() => world.readTransforms()).not.toThrow();
        expect(world.pendingCommands).toBe(2);
    });

    it('fallback spawn after out-of-band growth re-acquires instead of writing a detached view', () => {
        const { world, memory } = makeSyncWorld({ capacity: 4, ringCapacity: 4 }, () => {});
        memory.grow(1);
        expect(() => world.spawn()).not.toThrow();
        memory.grow(1);
        expect(() => world.readTransforms()).not.toThrow();
    });

    it('syncLayout is a no-op when nothing changed', () => {
        const { world } = makeWorld(config);
        expect(world.syncLayout()).toBe(false);
    });

    it('fallback step() flushes deferred commands after the runtime drained the ring', () => {
        const { world } = makeSyncWorld({ capacity: 8, ringCapacity: 2, maxDeferredCommands: 100 }, (rt) => rt.drain());
        for (let i = 0; i < 6; i++) world.spawn({ x: i + 1 });
        for (let i = 0; i < 4 && world.pendingCommands > 0; i++) world.step();
        expect(world.pendingCommands).toBe(0);
        const buf = world.readTransforms();
        expect([0, 1, 2, 3, 4, 5].map((i) => buf[i * 7])).toEqual([1, 2, 3, 4, 5, 6]);
    });

    it('requestGrow rejects a request not larger than one already pending', () => {
        const { world, rt } = makeWorld({ capacity: 2, ringCapacity: 8 });
        world.requestGrow(8);
        expect(() => world.requestGrow(8)).toThrow(RangeError);
        expect(() => world.requestGrow(5)).toThrow(RangeError);
        expect(() => world.requestGrow(9)).not.toThrow();
        rt.drain();
        world.readTransforms();
        expect(world.capacity).toBe(9);
        expect(() => world.requestGrow(10)).not.toThrow();
    });

    it('a throwing onBufferGrowth handler does not break the call that observed the growth', async () => {
        const { world, rt } = makeWorld({ capacity: 2, ringCapacity: 8 });
        world.onBufferGrowth = () => { throw new Error('handler boom'); };
        world.requestGrow(4);
        rt.drain();
        const uncaught: unknown[] = [];
        const onErr = (e: unknown) => uncaught.push(e);
        const proc = (globalThis as unknown as { process: { once(e: string, f: (x: unknown) => void): void; off(e: string, f: (x: unknown) => void): void } }).process;
        proc.once('uncaughtException', onErr);
        expect(() => world.readTransforms()).not.toThrow();
        expect(world.capacity).toBe(4);
        await new Promise((r) => setTimeout(r, 10));
        proc.off('uncaughtException', onErr);
        expect(String(uncaught[0])).toContain('handler boom');
    });

    it('deferred queue compaction keeps order and loses nothing across thousands of commands', () => {
        const { world, rt } = makeWorld({ capacity: 8192, ringCapacity: 64, maxDeferredCommands: 100_000 }, { pages: 64 });
        const n = 5000;
        for (let i = 0; i < n; i++) world.spawn({ x: i + 1 });
        expect(world.deferredCommands).toBe(n - 64);
        const order: number[] = [];
        const seen = new Set<number>();
        for (let guard = 0; guard < 500 && world.pendingCommands > 0; guard++) {
            rt.drain();
            world.flush();
            const buf = world.readTransforms();
            for (let id = 0; id < n; id++) if (buf[id * 7 + 6] === 1 && !seen.has(id)) { seen.add(id); order.push(id); }
        }
        expect(world.pendingCommands).toBe(0);
        expect(order.length).toBe(n);
        expect(order).toEqual([...order].sort((a, b) => a - b));
        expect(buf0(world)).toBe(1);
    });
    it('setGravity enqueues a command, validates input, and respects backpressure order', () => {
        const { world } = makeWorld(config);
        world.setGravity(0, -9.81, 0);
        expect(world.pendingCommands).toBe(1);
        for (const bad of [[NaN, 0, 0], [0, Infinity, 0], [0, 0, 1e6]] as const) {
            expect(() => world.setGravity(bad[0], bad[1], bad[2])).toThrow(RangeError);
        }
        expect(world.pendingCommands).toBe(1);
    });

    it('runtimeStats reads ring depth, dropped contacts, overruns and contacts from the header', () => {
        const { world, rt } = makeWorld(config);
        Atomics.store(rt.header, 12, 5);
        Atomics.store(rt.header, 13, 2);
        Atomics.store(rt.header, 14, 7);
        Atomics.store(rt.header, 15, 11);
        expect(world.runtimeStats).toEqual({ ringPending: 5, droppedContacts: 2, workerOverruns: 7, contacts: 11 });
    });

    describe('registerHull', () => {
        const cube = () => {
            const p: number[] = [];
            for (const x of [-1, 1]) for (const y of [-1, 1]) for (const z of [-1, 1]) p.push(x, y, z);
            return p;
        };

        function ringTags(mem: WebAssembly.Memory, layout: { ringSlotsByteOffset: number }, n: number) {
            const v = new DataView(mem.buffer);
            return Array.from({ length: n }, (_, i) => v.getUint8(layout.ringSlotsByteOffset + i * 64));
        }

        it('chunks 4 vertices per command and ends with a commit, returning sequential ids', () => {
            const { world, memory, layout } = makeWorld({ capacity: 8, ringCapacity: 16 });
            expect(world.registerHull(cube())).toBe(0);
            expect(world.pendingCommands).toBe(3);
            expect(ringTags(memory, layout, 3)).toEqual([5, 5, 6]);
            expect(world.registerHull([...cube(), 0, 0, 2])).toBe(1);
            expect(world.pendingCommands).toBe(7);
        });

        it('keeps chunk order across ring backpressure so the commit is last', () => {
            const { world, rt, memory, layout } = makeWorld({ capacity: 8, ringCapacity: 2, maxDeferredCommands: 100 });
            world.registerHull(cube());
            const seen: number[] = [];
            for (let i = 0; i < 10 && world.pendingCommands > 0; i++) {
                seen.push(...ringTags(memory, layout, 0));
                const head = new Int32Array(memory.buffer, layout.ringHeadByteOffset, 1);
                const tail = new Int32Array(memory.buffer, layout.ringTailByteOffset, 1);
                const v = new DataView(memory.buffer);
                for (let h = Atomics.load(head, 0); h !== Atomics.load(tail, 0); h++) seen.push(v.getUint8(layout.ringSlotsByteOffset + (h & 1) * 64));
                rt.drain();
                world.flush();
            }
            expect(seen).toEqual([5, 5, 6]);
        });

        it.each([
            ['fewer than 4 vertices', [0, 0, 0, 1, 0, 0, 0, 1, 0]],
            ['not a multiple of 3', [...Array(13).fill(0)]],
            ['more than 32 vertices', [...Array(99).fill(1)]],
            ['NaN', (() => { const p = [...Array(12).fill(1)]; p[4] = NaN; return p; })()],
            ['Infinity', (() => { const p = [...Array(12).fill(1)]; p[7] = Infinity; return p; })()],
            ['out of range', (() => { const p = [...Array(12).fill(1)]; p[0] = 2e4; return p; })()],
        ])('rejects %s before touching the ring', (_n, pts) => {
            const { world } = makeWorld({ capacity: 8, ringCapacity: 16 });
            expect(() => world.registerHull(pts)).toThrow(RangeError);
            expect(world.pendingCommands).toBe(0);
        });

        it('stops at the hull limit and does not consume an id when the ring has no room', () => {
            const tiny = makeWorld({ capacity: 8, ringCapacity: 2, maxDeferredCommands: 0 }).world;
            expect(() => tiny.registerHull(cube())).toThrow('CommandRing full');
            expect(tiny.pendingCommands).toBe(0);
            const big = makeWorld({ capacity: 8, ringCapacity: 4096, maxDeferredCommands: 100 }, { pages: 16 }).world;
            for (let i = 0; i < 1024; i++) big.registerHull(cube());
            expect(() => big.registerHull(cube())).toThrow(RangeError);
        });

        it('commit carries the total vertex count', () => {
            const { world, memory, layout } = makeWorld({ capacity: 8, ringCapacity: 16 });
            world.registerHull([...cube(), 0, 0, 2]);
            expect(new DataView(memory.buffer).getUint8(layout.ringSlotsByteOffset + 3 * 64 + 5)).toBe(9);
        });

        it.each([[undefined], [-1], [0.5], [2 ** 32], [5]])('spawn with SHAPE_HULL rejects hull id %s', (hull) => {
            const { world } = makeWorld({ capacity: 8, ringCapacity: 16 });
            world.registerHull(cube());
            expect(() => world.spawn({ shape: 3, hull: hull as number })).toThrow(RangeError);
            expect(world.allocatedEntities).toBe(0);
        });

        it('a spawn can reference the returned hull id', () => {
            const { world, memory, layout } = makeWorld({ capacity: 8, ringCapacity: 16 });
            const hull = world.registerHull(cube());
            world.spawn({ shape: 3, hull });
            const v = new DataView(memory.buffer);
            expect(v.getUint32(layout.ringSlotsByteOffset + 3 * 64 + 60, true)).toBe(hull);
        });
    });
});

function buf0(world: import('../World.js').World) { return world.readTransforms()[0]; }
