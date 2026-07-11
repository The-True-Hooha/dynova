import { describe, it, expect } from 'vitest';
import { CommandRing, SLOT_BYTES } from '../CommandRing.js';

function makeRing(capacity = 64) {
    const mem = new WebAssembly.Memory({ initial: 1 });
    const slotsOffset = 0;
    const headOffset  = capacity * SLOT_BYTES;
    const tailOffset  = headOffset + 4;
    return { mem, ring: new CommandRing(mem, slotsOffset, headOffset, tailOffset, capacity) };
}

function snapshotSlots(mem: WebAssembly.Memory, count: number): Uint8Array {
    return new Uint8Array(mem.buffer.slice(0, count * SLOT_BYTES));
}

const FIXED_SPAWNS = [
    { id: 0, gen: 0, x:  1.5,  y: -2.25, z: 0,    qx: 0, qy: 0, qz: 0, qw: 1 },
    { id: 1, gen: 0, x: -3.0,  y:  0.5,  z: 1.0,  qx: 0, qy: 0, qz: 0, qw: 1 },
    { id: 2, gen: 1, x:  0.0,  y:  0.0,  z: -9.8, qx: 0, qy: 0, qz: 0, qw: 1 },
    { id: 3, gen: 0, x:  100,  y:  200,  z: 300,  qx: 0, qy: 0, qz: 0, qw: 1 },
    { id: 4, gen: 2, x: -0.001,y:  0.001,z: 0,    qx: 0, qy: 0, qz: 0, qw: 1 },
];

describe('CommandRing encoding determinism', () => {
    it('identical spawn sequence produces identical slot bytes', () => {
        const { mem: mem1, ring: ring1 } = makeRing();
        const { mem: mem2, ring: ring2 } = makeRing();

        for (const s of FIXED_SPAWNS) { ring1.spawn(s); }
        for (const s of FIXED_SPAWNS) { ring2.spawn(s); }

        expect(snapshotSlots(mem1, FIXED_SPAWNS.length)).toEqual(snapshotSlots(mem2, FIXED_SPAWNS.length));
    });

    it('different spawn order produces different slot bytes', () => {
        const { mem: mem1, ring: ring1 } = makeRing();
        const { mem: mem2, ring: ring2 } = makeRing();

        for (const s of FIXED_SPAWNS) { ring1.spawn(s); }
        for (const s of [...FIXED_SPAWNS].reverse()) { ring2.spawn(s); }

        expect(snapshotSlots(mem1, FIXED_SPAWNS.length)).not.toEqual(snapshotSlots(mem2, FIXED_SPAWNS.length));
    });

    it('mixed command sequence is stable across runs', () => {
        function buildSequence() {
            const { mem, ring } = makeRing();
            ring.spawn({ id: 0, gen: 0, x: 1, y: 2, z: 3, qx: 0, qy: 0, qz: 0, qw: 1 });
            ring.spawn({ id: 1, gen: 0, x: 4, y: 5, z: 6, qx: 0, qy: 0, qz: 0, qw: 1 });
            ring.force({ id: 0, gen: 0, x: 0, y: -9.8, z: 0 });
            ring.destroy({ id: 1, gen: 0 });
            return new Uint8Array(mem.buffer.slice(0, 4 * SLOT_BYTES));
        }
        expect(buildSequence()).toEqual(buildSequence());
    });

    it('stale generation encodes differently from live generation', () => {
        const { mem: mem1, ring: ring1 } = makeRing();
        const { mem: mem2, ring: ring2 } = makeRing();

        ring1.spawn({ id: 0, gen: 0, x: 1, y: 2, z: 3, qx: 0, qy: 0, qz: 0, qw: 1 });
        ring2.spawn({ id: 0, gen: 1, x: 1, y: 2, z: 3, qx: 0, qy: 0, qz: 0, qw: 1 });

        expect(snapshotSlots(mem1, 1)).not.toEqual(snapshotSlots(mem2, 1));
    });
});
