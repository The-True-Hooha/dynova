import { describe, it, expect } from 'vitest';
import { readFileSync } from 'node:fs';
import { World, layoutFromRuntime } from '../World.js';
import { CommandRing, SLOT_BYTES, MAX_PARTICLES } from '../CommandRing.js';
import { WorldBatch, type BatchRuntime } from '../batch.js';

// The real single-threaded wasm module (make build-st): the particles are stepped and published by the shipped core.
const glue = await import('../../wasm-st/dynova_core.js');
const wasm = glue.initSync({ module: readFileSync(new URL('../../wasm-st/dynova_core_bg.wasm', import.meta.url)) });
const config = { capacity: 32, ringCapacity: 256 };

function world(gravity = true): World {
    const rt = new glue.Runtime(config.capacity, config.ringCapacity);
    rt.enable_physics();
    const w = World.attachSync(config, wasm.memory, 'fallback', layoutFromRuntime(rt), rt);
    if (!gravity) w.setGravity(0, 0, 0);
    return w;
}

const floor = (w: World) => w.spawn({ y: -0.5, hx: 20, hy: 0.5, hz: 20, invMass: 0 });

describe('particles through the SDK on the real wasm', () => {
    it('a rope pinned at one end hangs from it: one read returns every particle, the pin never moves', () => {
        const w = world();
        const g = w.createChain({ from: [0, 5, 0], to: [2, 5, 0], count: 11, pinFirst: true });
        expect(g).toEqual({ id: 0, first: 0, count: 11 });
        for (let t = 0; t < 360; t++) w.step();
        const p = w.readParticles();
        expect(p.length).toBe(11 * 4);
        expect([p[0], p[1], p[2]]).toEqual([0, 5, 0]);
        expect(p[3]).toBeCloseTo(0.02, 6);
        const tip = 10 * 4;
        expect(p[tip + 1]).toBeLessThan(3.4);
        expect(p[tip + 1]).toBeGreaterThan(2.9);
    });

    it('a cloth dropped on a box drapes over it and a soft block dropped on the floor rests on it', () => {
        const w = world();
        floor(w);
        w.spawn({ y: 0.6, hx: 0.4, hy: 0.5, hz: 0.4, invMass: 0 });
        const cloth = w.createCloth({ origin: [-1, 2.2, -1], u: [2, 0, 0], v: [0, 0, 2], nu: 21, nv: 21, radius: 0.06 });
        const block = w.createSoftBlock({ origin: [3, 1, 3], size: [0.8, 0.8, 0.8], n: [4, 4, 4], radius: 0.13 });
        expect([cloth.first, cloth.count, block.first, block.count]).toEqual([0, 441, 441, 64]);
        for (let t = 0; t < 600; t++) w.step();
        const p = w.readParticles();
        expect(p.length).toBe((441 + 64) * 4);
        expect(p[(10 * 21 + 10) * 4 + 1]).toBeGreaterThan(1.1);
        expect(p[(10 * 21 + 10) * 4 + 1]).toBeLessThan(1.25);
        let lowest = Infinity;
        for (let i = 441; i < 441 + 64; i++) lowest = Math.min(lowest, p[i * 4 + 1]);
        expect(lowest).toBeGreaterThan(0.1);
        expect(lowest).toBeLessThan(0.17);
    }, 30_000);

    it('a rope attached to a free body goes where the body goes, and detaching lets it go', () => {
        const w = world(false);
        const box = w.spawn({ x: 0, y: 0, z: 0, hx: 0.5, hy: 0.5, hz: 0.5, invMass: 1 });
        const rope = w.createChain({ from: [1, 0, 0], to: [2, 0, 0], count: 2, invMass: 1 });
        w.attachParticle(rope.first, box, [0.5, 0, 0]);
        w.force(box, 20, 0, 0);
        for (let t = 0; t < 30; t++) w.step();
        const x = w.readTransforms()[0];
        expect(x).toBeGreaterThan(0.5);
        expect(w.readParticles()[0]).toBeCloseTo(x + 0.5, 1);
        w.detachParticle(rope.first);
        w.force(box, -40, 0, 0);
        for (let t = 0; t < 30; t++) w.step();
        // Held, the particle would still be half a body width ahead of the box; free, it carries on while the box is driven back.
        expect(w.readParticles()[0] - w.readTransforms()[0]).toBeGreaterThan(1.5);
        expect(w.runtimeCounters.malformedDropped).toBe(0);
    });

    it('a seek republishes the particles of the sought tick, and none before the rope was made', () => {
        const w = world();
        w.record(true);
        w.step();
        for (let t = 0; t < 40; t++) {
            if (t === 20) w.createChain({ from: [0, 5, 0], to: [1, 5, 0], count: 5, pinFirst: true });
            w.step();
        }
        const end = Array.from(w.readParticles());
        expect(end.length).toBe(20);
        w.seek(10);
        w.step();
        expect(w.readParticles().length).toBe(0);
        w.seek(41);
        w.step();
        expect(Array.from(w.readParticles())).toEqual(end);
    });

    it('every bad request is refused with a message before anything is queued', () => {
        const w = world();
        const ok = { from: [0, 1, 0], to: [1, 1, 0], count: 4 } as const;
        expect(() => w.createChain({ ...ok, count: 1 })).toThrow(/2\.\.1024/);
        expect(() => w.createChain({ ...ok, to: [0, 1, 0] })).toThrow(/apart/);
        expect(() => w.createChain({ ...ok, radius: 0 })).toThrow(/radius/);
        expect(() => w.createChain({ ...ok, invMass: -1 })).toThrow(/invMass/);
        expect(() => w.createChain({ ...ok, stiffness: 0 })).toThrow(/stiffness/);
        expect(() => w.createChain({ ...ok, from: [Number.NaN, 0, 0] })).toThrow(/finite/);
        const sheet = { origin: [0, 0, 0], u: [1, 0, 0], v: [0, 0, 1], nu: 4, nv: 4 } as const;
        expect(() => w.createCloth({ ...sheet, nu: 65 })).toThrow(/2\.\.64/);
        expect(() => w.createCloth({ ...sheet, v: [2, 0, 0] })).toThrow(/span an area/);
        expect(() => w.createCloth({ ...sheet, pins: 32 })).toThrow(/mask/);
        expect(() => w.createCloth({ ...sheet, bendStiffness: 2 })).toThrow(/bendStiffness/);
        const block = { origin: [0, 0, 0], size: [1, 1, 1], n: [3, 3, 3] } as const;
        expect(() => w.createSoftBlock({ ...block, n: [13, 3, 3] })).toThrow(/2\.\.12/);
        expect(() => w.createSoftBlock({ ...block, size: [0, 1, 1] })).toThrow(/size/);
        const box = w.spawn({});
        expect(() => w.attachParticle(-1, box, [0, 0, 0])).toThrow(RangeError);
        expect(() => w.attachParticle(MAX_PARTICLES, box, [0, 0, 0])).toThrow(RangeError);
        expect(() => w.attachParticle(0, box, [Number.NaN, 0, 0])).toThrow(/offset/);
        expect(() => w.detachParticle(1.5)).toThrow(RangeError);
        const lattice = { origin: [0, 0, 0], n: [4, 4, 4], spacing: 0.05 } as const;
        expect(() => w.createFluid({ ...lattice, n: [33, 4, 4] })).toThrow(/2\.\.32/);
        expect(() => w.createFluid({ ...lattice, n: [1, 4, 4] })).toThrow(/2\.\.32/);
        expect(() => w.createFluid({ ...lattice, n: [4, 4] as never })).toThrow(/three counts/);
        expect(() => w.createFluid({ ...lattice, spacing: 0.009 })).toThrow(/spacing/);
        expect(() => w.createSand({ ...lattice, spacing: 1.5 })).toThrow(/spacing/);
        expect(() => w.createFluid({ ...lattice, spacing: Number.NaN })).toThrow(/spacing/);
        expect(() => w.createFluid({ ...lattice, origin: [0, 1e7, 0] })).toThrow(/origin/);
        expect(() => w.createFluid({ ...lattice, viscosity: 1.5 })).toThrow(/viscosity/);
        expect(() => w.createSand({ ...lattice, friction: -0.1 })).toThrow(/friction/);
        expect(() => w.createSand({ ...lattice, friction: Number.NaN })).toThrow(/friction/);
        expect(w.pendingCommands + w.deferredCommands).toBe(1);
        expect(w.createChain(ok).first).toBe(0);
        expect(() => { for (let i = 0; i < 20; i++) w.createSoftBlock({ origin: [0, 0, 0], size: [1, 1, 1], n: [12, 12, 12] }); }).toThrow(/at most 16384 particles/);
    });

    it('water spreads and sand holds its block on the same floor, and each group reads back where the host said it would', () => {
        const w = world();
        floor(w);
        const rope = w.createChain({ from: [0, 5, 0], to: [1, 5, 0], count: 11, pinFirst: true });
        const water = w.createFluid({ origin: [-3, 0.05, 0], n: [6, 6, 6], spacing: 0.05 });
        const sand = w.createSand({ origin: [3, 0.05, 0], n: [6, 6, 6], spacing: 0.05, friction: 1 });
        expect([rope, water, sand]).toEqual([{ id: 0, first: 0, count: 11 }, { id: 1, first: 11, count: 216 }, { id: 2, first: 227, count: 216 }]);
        for (let t = 0; t < 300; t++) w.step();
        const p = w.readParticles();
        const width = (g: { first: number; count: number }) => {
            let lo = Infinity, hi = -Infinity;
            for (let i = g.first; i < g.first + g.count; i++) { lo = Math.min(lo, p[i * 4]); hi = Math.max(hi, p[i * 4]); }
            return hi - lo;
        };
        expect(p[water.first * 4 + 3]).toBeCloseTo(0.025, 6);
        expect(width(water)).toBeGreaterThan(0.6);
        expect(width(sand)).toBeLessThan(0.5);
        expect(w.runtimeCounters.malformedDropped).toBe(0);
    }, 30_000);

    it('the viscosity and friction given to createFluid and createSand reach the ring slot, not a default', () => {
        const rt = new glue.Runtime(config.capacity, config.ringCapacity);
        rt.enable_physics();
        const layout = layoutFromRuntime(rt);
        const w = World.attachSync(config, wasm.memory, 'fallback', layout, rt);
        w.createSand({ origin: [0, 0, 0], n: [2, 2, 2], spacing: 0.05, friction: 0.3 });
        w.createFluid({ origin: [0, 0, 0], n: [2, 2, 2], spacing: 0.05, viscosity: 0.7 });
        const slot = (n: number) => new DataView(wasm.memory.buffer, layout.ringSlotsByteOffset + n * SLOT_BYTES, SLOT_BYTES);
        expect([slot(0).getUint8(0), slot(0).getFloat32(28, true)]).toEqual([26, Math.fround(0.3)]);
        expect([slot(1).getUint8(0), slot(1).getFloat32(28, true)]).toEqual([25, Math.fround(0.7)]);
    });

    it('a batch renders particles per world: each world reads its own', () => {
        const batch = new glue.Batch(2, config.capacity, config.ringCapacity);
        batch.enable_physics();
        const set = new WorldBatch(config, wasm.memory, 'fallback', batch as unknown as BatchRuntime);
        set.worlds[0].createChain({ from: [0, 5, 0], to: [1, 5, 0], count: 3, pinFirst: true });
        set.worlds[1].createChain({ from: [0, 9, 0], to: [1, 9, 0], count: 6, pinFirst: true });
        for (let t = 0; t < 10; t++) set.step();
        expect([set.worlds[0].readParticles().length, set.worlds[1].readParticles().length]).toEqual([12, 24]);
        expect(set.worlds[1].readParticles()[1]).toBe(9);
    });
});

describe('particle commands on the wire', () => {
    it('write the bytes the Rust ring decodes (pinned in python/tests/golden_particles_ts.json, also read by Rust and Python)', () => {
        const gold = JSON.parse(new TextDecoder().decode(readFileSync(new URL('../../../../python/tests/golden_particles_ts.json', import.meta.url)))) as Record<string, string>;
        const mem = new WebAssembly.Memory({ initial: 1 });
        const ring = new CommandRing(mem, 0, 64 * SLOT_BYTES, 64 * SLOT_BYTES + 4, 64);
        const hex = (n: number) => Array.from(new Uint8Array(mem.buffer, n * SLOT_BYTES, SLOT_BYTES)).map((b) => b.toString(16).padStart(2, '0')).join('');
        const writes: Array<[string, () => boolean]> = [
            ['chain', () => ring.particleChain({ id: 70000, from: [1, 2, 3], to: [-1, 2, 3.5], count: 300, pinFirst: true, pinLast: true, radius: 0.025, invMass: 2, stiffness: 0.75 })],
            ['cloth', () => ring.particleCloth({ id: 70000, origin: [1, 2, 3], u: [2, 0, 0.5], v: [0, 0, 3], nu: 64, nv: 17, pins: 22, radius: 0.04, invMass: 2, stiffness: 0.75, bendStiffness: 0.25 })],
            ['soft', () => ring.particleSoft({ id: 70000, origin: [1, 2, 3], size: [0.5, 1, 2], n: [4, 12, 2], radius: 0.1, invMass: 2, stiffness: 0.75 })],
            ['fluid', () => ring.particleFluid({ id: 70000, origin: [1, 2, 3], n: [4, 32, 2], spacing: 0.05, viscosity: 0.25 })],
            ['sand', () => ring.particleSand({ id: 70000, origin: [1, 2, 3], n: [4, 32, 2], spacing: 0.05, friction: 0.75 })],
            ['attach', () => ring.particleAttach({ particle: 70000, body: 123456, gen: 9, local: [0.5, -1, 2] })],
            ['detach', () => ring.particleDetach(5)],
        ];
        writes.forEach(([name, write], n) => {
            new Uint8Array(mem.buffer, n * SLOT_BYTES, SLOT_BYTES).fill(0xff);
            expect(write(), name).toBe(true);
            expect(hex(n), name).toBe(gold[name]);
        });
        expect(Object.keys(gold).sort()).toEqual(writes.map(([name]) => name).sort());
    });
});

describe('host checks judge what the core will see', () => {
    it('fluid and sand share one cap of 4096 particles, judged before anything is queued', () => {
        const w = world();
        const half = { origin: [0, 0, 0], n: [32, 32, 2], spacing: 0.05 } as const;
        expect(w.createFluid(half).count).toBe(2048);
        expect(w.createSand(half).first).toBe(2048);
        const queued = w.pendingCommands;
        expect(() => w.createFluid({ origin: [0, 0, 0], n: [2, 2, 2], spacing: 0.05 })).toThrow(/at most 4096 fluid and sand/);
        expect(() => w.createSand({ origin: [0, 0, 0], n: [2, 2, 2], spacing: 0.05 })).toThrow(/at most 4096 fluid and sand/);
        expect(w.pendingCommands).toBe(queued);
        expect(w.createChain({ from: [0, 0, 0], to: [1, 0, 0], count: 3 }).first).toBe(4096);
    });

    it('a reset or a seek back past a group gives its particles and its share of the fluid cap back, so the next group starts where the core will put it', () => {
        const w = world();
        w.checkpoint();
        w.step();
        const half = { origin: [0, 0, 0], n: [32, 32, 2], spacing: 0.05 } as const;
        w.createFluid(half);
        expect(w.createSand(half).first).toBe(2048);
        w.step();
        expect(w.readParticles().length).toBe(4096 * 4);
        w.reset();
        w.step();
        expect(w.readParticles().length).toBe(0);
        const again = w.createFluid(half);
        expect(again.first).toBe(0);
        w.step();
        expect(w.readParticles().length).toBe(2048 * 4);

        // Made in the same frame as the reset: the core keeps it, so it still counts against the cap.
        const x = world();
        x.checkpoint();
        x.step();
        x.createChain({ from: [0, 5, 0], to: [1, 5, 0], count: 100 });
        x.step();
        x.reset();
        x.createFluid(half);
        x.step();
        expect(x.readParticles().length).toBe(2048 * 4);
        expect(x.createSand(half).first).toBe(2048);
        expect(() => x.createFluid({ origin: [0, 0, 0], n: [2, 2, 2], spacing: 0.05 })).toThrow(/at most 4096 fluid and sand/);

        // A group that ends exactly where the core's count now stands survives the reset and keeps its share of the cap.
        const u = world();
        u.createFluid(half);
        u.checkpoint();
        u.step();
        u.createSand(half);
        u.step();
        u.reset();
        u.step();
        expect(u.createSand(half).first).toBe(2048);
        expect(() => u.createFluid({ origin: [0, 0, 0], n: [2, 2, 2], spacing: 0.05 })).toThrow(/at most 4096 fluid and sand/);

        const v = world();
        v.record(true);
        v.step();
        for (let t = 0; t < 30; t++) {
            if (t === 20) v.createChain({ from: [0, 5, 0], to: [1, 5, 0], count: 5, pinFirst: true });
            v.step();
        }
        v.seek(10);
        v.step();
        expect(v.createChain({ from: [0, 3, 0], to: [1, 3, 0], count: 3 }).first).toBe(0);
    });

    it('judges lattice limits on the f32 values: a spacing that rounds onto a bound passes, one past it does not', () => {
        const w = world();
        const lattice = { origin: [0, 0, 0], n: [2, 2, 2] } as const;
        expect(() => w.createFluid({ ...lattice, spacing: 0.0099999999 })).not.toThrow();
        expect(() => w.createFluid({ ...lattice, spacing: 1.0000000001 })).not.toThrow();
        expect(() => w.createFluid({ ...lattice, spacing: 0.05, viscosity: 1.0000000001 })).not.toThrow();
        expect(() => w.createSand({ ...lattice, spacing: 0.05, friction: 1.0000000001 })).not.toThrow();
        expect(() => w.createFluid({ ...lattice, spacing: 0.00999 })).toThrow(/spacing/);
        expect(() => w.createFluid({ ...lattice, spacing: 1.0001 })).toThrow(/spacing/);
        for (let t = 0; t < 2; t++) w.step();
        expect(w.runtimeCounters.malformedDropped).toBe(0);
    });

    it('refuses what rounds to nothing in f32, wrong-length vectors and a 4097th group, before queueing', () => {
        const w = world();
        expect(() => w.createChain({ from: [500000, 0, 0], to: [500000.001, 0, 0], count: 3 })).toThrow(/apart/);
        expect(() => w.createChain({ from: [0, 0, 0], to: [1, 0, 0], count: 3, stiffness: 1e-50 })).toThrow(/stiffness/);
        expect(() => w.createCloth({ origin: [0, 0, 0], u: [1e6, 1, 0], v: [1e6, 1.00000001, 0], nu: 3, nv: 3 })).toThrow(/span an area/);
        expect(() => w.createSoftBlock({ origin: [0, 0, 0], size: [1, 1, 1], n: [4, 4] as never })).toThrow(/three counts/);
        expect(() => w.createChain({ from: [0, 0] as never, to: [1, 0, 0], count: 3 })).toThrow(/3-vectors/);
        expect(w.pendingCommands).toBe(0);
        const wide = new glue.Runtime(8, 8192);
        wide.enable_physics();
        const big = World.attachSync({ capacity: 8, ringCapacity: 8192 }, wasm.memory, 'fallback', layoutFromRuntime(wide), wide);
        for (let i = 0; i < 4096; i++) big.createChain({ from: [0, 0, 0], to: [1, 0, 0], count: 2 });
        expect(() => big.createChain({ from: [0, 0, 0], to: [1, 0, 0], count: 2 })).toThrow(/4096 particle groups/);
    });
});
