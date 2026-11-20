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
        expect(w.pendingCommands + w.deferredCommands).toBe(1);
        expect(w.createChain(ok).first).toBe(0);
        expect(() => { for (let i = 0; i < 20; i++) w.createSoftBlock({ origin: [0, 0, 0], size: [1, 1, 1], n: [12, 12, 12] }); }).toThrow(/at most 16384 particles/);
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
