import { describe, it, expect } from 'vitest';
import { CommandRing, TAG_SPAWN, TAG_DESTROY, TAG_FORCE, SLOT_BYTES } from '../CommandRing.js';

function makeRing(capacity = 16) {
    const mem = new WebAssembly.Memory({ initial: 1 });
    const slotsOffset = 0;
    const headOffset  = capacity * SLOT_BYTES;
    const tailOffset  = headOffset + 4;
    return { mem, ring: new CommandRing(mem, slotsOffset, headOffset, tailOffset, capacity) };
}

describe('CommandRing', () => {
    it('spawn encodes tag, id, gen, and position', () => {
        const { mem, ring } = makeRing();
        expect(ring.spawn({ id: 7, gen: 3, x: 1, y: 2, z: 3, qx: 0, qy: 0, qz: 0, qw: 1 })).toBe(true);
        expect(ring.pendingCount).toBe(1);
        const view = new DataView(mem.buffer);
        expect(view.getUint8(0)).toBe(TAG_SPAWN);
        expect(view.getUint32(1, true)).toBe(7);
        expect(view.getUint8(5)).toBe(3);
        expect(view.getFloat32(6,  true)).toBeCloseTo(1);
        expect(view.getFloat32(10, true)).toBeCloseTo(2);
        expect(view.getFloat32(14, true)).toBeCloseTo(3);
        expect(view.getFloat32(30, true)).toBeCloseTo(1);
    });

    it('destroy encodes tag, id, and gen', () => {
        const { mem, ring } = makeRing();
        ring.destroy({ id: 42, gen: 5 });
        const view = new DataView(mem.buffer);
        expect(view.getUint8(0)).toBe(TAG_DESTROY);
        expect(view.getUint32(1, true)).toBe(42);
        expect(view.getUint8(5)).toBe(5);
    });

    it('force encodes tag, id, gen, and vector', () => {
        const { mem, ring } = makeRing();
        ring.force({ id: 3, gen: 1, x: 9, y: 8, z: 7 });
        const view = new DataView(mem.buffer);
        expect(view.getUint8(0)).toBe(TAG_FORCE);
        expect(view.getUint32(1,  true)).toBe(3);
        expect(view.getUint8(5)).toBe(1);
        expect(view.getFloat32(6,  true)).toBeCloseTo(9);
        expect(view.getFloat32(10, true)).toBeCloseTo(8);
        expect(view.getFloat32(14, true)).toBeCloseTo(7);
    });

    it('returns false and does not advance tail when full', () => {
        const { ring } = makeRing(4);
        const desc = { id: 0, gen: 0, x: 0, y: 0, z: 0, qx: 0, qy: 0, qz: 0, qw: 1 };
        ring.spawn({ ...desc, id: 0 });
        ring.spawn({ ...desc, id: 1 });
        ring.spawn({ ...desc, id: 2 });
        ring.spawn({ ...desc, id: 3 });
        expect(ring.spawn({ ...desc, id: 4 })).toBe(false);
        expect(ring.pendingCount).toBe(4);
    });

    it('wraps around correctly', () => {
        const { mem, ring } = makeRing(4);
        const head = new Int32Array(mem.buffer, 4 * SLOT_BYTES, 1);
        const desc = { id: 0, gen: 0, x: 0, y: 0, z: 0, qx: 0, qy: 0, qz: 0, qw: 1 };
        ring.spawn({ ...desc, id: 0 });
        ring.spawn({ ...desc, id: 1 });
        Atomics.store(head, 0, 2);
        ring.spawn({ ...desc, id: 2 });
        ring.spawn({ ...desc, id: 3 });
        ring.spawn({ ...desc, id: 4 });
        expect(ring.pendingCount).toBe(3);
        const view = new DataView(mem.buffer);
        expect(view.getUint32(1, true)).toBe(4);
    });
});
