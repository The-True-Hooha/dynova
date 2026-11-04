import { describe, it, expect } from 'vitest';
import { CommandRing, TAG_SPAWN, TAG_DESTROY, TAG_FORCE, SHAPE_BOX, SHAPE_SPHERE, SHAPE_CAPSULE, TAG_GRAVITY, TAG_HULL_VERTS, TAG_HULL_COMMIT, SHAPE_HULL, SLOT_BYTES } from '../CommandRing.js';

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

    it('spawn bytes match the Rust cross-language fixture', () => {
        const { mem, ring } = makeRing();
        ring.spawn({
            id: 7, gen: 3, x: 1, y: 2, z: 3, qx: 0, qy: 0, qz: 0, qw: 1,
            shape: SHAPE_SPHERE, hx: 0.5, hy: 0.25, hz: 0.125, invMass: 2, restitution: 0.5,
        });
        const hex = Array.from(new Uint8Array(mem.buffer, 0, SLOT_BYTES))
            .map((b) => b.toString(16).padStart(2, '0')).join('');
        expect(hex).toBe(
            '00' + '07000000' + '03' +
            '0000803f' + '00000040' + '00004040' +
            '00000000' + '00000000' + '00000000' + '0000803f' +
            '01' + '00' +
            '0000003f' + '0000803e' + '0000003e' +
            '00000040' + '0000003f' +
            '0000003f' + '00000000',
        );
    });

    it('spawn defaults to a unit box with mass 1 and restitution 0.3', () => {
        const { mem, ring } = makeRing();
        ring.spawn({ id: 0, gen: 0, x: 0, y: 0, z: 0, qx: 0, qy: 0, qz: 0, qw: 1 });
        const view = new DataView(mem.buffer);
        expect(view.getUint8(34)).toBe(SHAPE_BOX);
        expect(view.getFloat32(36, true)).toBe(0.5);
        expect(view.getFloat32(40, true)).toBe(0.5);
        expect(view.getFloat32(44, true)).toBe(0.5);
        expect(view.getFloat32(48, true)).toBe(1);
        expect(view.getFloat32(52, true)).toBeCloseTo(0.3);
    });

    it('wrapped slot holds the newer command', () => {
        const { mem, ring } = makeRing(1);
        const head = new Int32Array(mem.buffer, SLOT_BYTES, 1);
        ring.spawn({ id: 1, gen: 1, x: 5, y: 5, z: 5, qx: 0, qy: 0, qz: 0, qw: 1, hx: 9, hy: 9, hz: 9 });
        Atomics.store(head, 0, 1);
        ring.destroy({ id: 2, gen: 2 });
        const view = new DataView(mem.buffer);
        expect(view.getUint8(0)).toBe(TAG_DESTROY);
        expect(view.getUint32(1, true)).toBe(2);
    });

    it('spawn encodes friction and capsule shape', () => {
        const { mem, ring } = makeRing();
        ring.spawn({ id: 0, gen: 0, x: 0, y: 0, z: 0, qx: 0, qy: 0, qz: 0, qw: 1, shape: SHAPE_CAPSULE, friction: 1.25 });
        const view = new DataView(mem.buffer);
        expect(view.getUint8(34)).toBe(SHAPE_CAPSULE);
        expect(view.getFloat32(56, true)).toBe(1.25);
    });

    it('spawn defaults friction to 0.5', () => {
        const { mem, ring } = makeRing();
        ring.spawn({ id: 0, gen: 0, x: 0, y: 0, z: 0, qx: 0, qy: 0, qz: 0, qw: 1 });
        expect(new DataView(mem.buffer).getFloat32(56, true)).toBe(0.5);
    });

    it('gravity encodes tag and vector at the force offsets, with the rest zeroed', () => {
        const { mem, ring } = makeRing(1);
        ring.spawn({ id: 9, gen: 9, x: 1, y: 1, z: 1, qx: 0, qy: 0, qz: 0, qw: 1, hx: 9 });
        Atomics.store(new Int32Array(mem.buffer, SLOT_BYTES, 1), 0, 1);
        expect(ring.gravity(0, -9.81, 0)).toBe(true);
        const view = new DataView(mem.buffer);
        expect(view.getUint8(0)).toBe(TAG_GRAVITY);
        expect(view.getFloat32(10, true)).toBeCloseTo(-9.81);
        expect(view.getUint32(1, true)).toBe(0);
        expect(view.getFloat32(36, true)).toBe(0);
    });

    it('hullVerts encodes tag, hull id, count and vertices; hullCommit encodes tag and id; slots are zeroed first', () => {
        const { mem, ring } = makeRing(1);
        ring.spawn({ id: 1, gen: 1, x: 5, y: 5, z: 5, qx: 0, qy: 0, qz: 0, qw: 1, hx: 9 });
        Atomics.store(new Int32Array(mem.buffer, SLOT_BYTES, 1), 0, 1);
        expect(ring.hullVerts(300, [1, 2, 3, -4, 5.5, 6])).toBe(true);
        const view = new DataView(mem.buffer);
        expect(view.getUint8(0)).toBe(TAG_HULL_VERTS);
        expect(view.getUint32(1, true)).toBe(300);
        expect(view.getUint8(5)).toBe(2);
        expect(view.getFloat32(6, true)).toBe(1);
        expect(view.getFloat32(22, true)).toBe(5.5);
        expect(view.getFloat32(26, true)).toBe(6);
        expect(view.getFloat32(30, true)).toBe(0);
        Atomics.store(new Int32Array(mem.buffer, SLOT_BYTES, 1), 0, 2);
        expect(ring.hullCommit(300, 8)).toBe(true);
        expect(view.getUint8(0)).toBe(TAG_HULL_COMMIT);
        expect(view.getUint32(1, true)).toBe(300);
        expect(view.getUint8(5)).toBe(8);
    });

    it('hullVerts rejects empty and oversized chunks and reports a full ring', () => {
        const { ring } = makeRing(1);
        expect(() => ring.hullVerts(0, [])).toThrow(RangeError);
        expect(() => ring.hullVerts(0, new Array(15).fill(0))).toThrow(RangeError);
        expect(() => ring.hullVerts(0, [1, 2])).toThrow(RangeError);
        expect(ring.hullCommit(0, 4)).toBe(true);
        expect(ring.hullVerts(0, [1, 2, 3])).toBe(false);
        expect(ring.hullCommit(0, 4)).toBe(false);
    });

    it('spawn carries the hull id at bytes 60 to 63', () => {
        const { mem, ring } = makeRing();
        ring.spawn({ id: 0, gen: 0, x: 0, y: 0, z: 0, qx: 0, qy: 0, qz: 0, qw: 1, shape: SHAPE_HULL, hull: 513 });
        const view = new DataView(mem.buffer);
        expect(view.getUint8(34)).toBe(SHAPE_HULL);
        expect(view.getUint32(60, true)).toBe(513);
    });

    it('a full 4-vertex chunk fills bytes 6 to 53 in order', () => {
        const { mem, ring } = makeRing();
        ring.hullVerts(1, Array.from({ length: 12 }, (_, i) => i + 1));
        const view = new DataView(mem.buffer);
        for (let i = 0; i < 12; i++) expect(view.getFloat32(6 + i * 4, true)).toBe(i + 1);
        expect(view.getFloat32(54, true)).toBe(0);
        expect(view.getUint8(5)).toBe(4);
    });
});
