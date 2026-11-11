import { describe, it, expect } from 'vitest';
import { CommandRing, TAG_SPAWN, TAG_DESTROY, TAG_FORCE, SHAPE_BOX, SHAPE_SPHERE, SHAPE_CAPSULE, TAG_GRAVITY, TAG_HULL_VERTS, TAG_HULL_COMMIT, SHAPE_HULL, SLOT_BYTES, TAG_TELEPORT, TAG_RECORD, TAG_SEEK, TAG_PAUSE, TAG_JOINT_CREATE, TAG_JOINT_DESTROY, TAG_JOINT_MOTOR, TAG_KINEMATIC_BATCH, TAG_RAYCAST, TAG_OVERLAP_SPHERE, KIN_SAMPLES_PER_SLOT, JOINT_HINGE } from '../CommandRing.js';

function makeRing(capacity = 16) {
    const mem = new WebAssembly.Memory({ initial: 1 });
    const slotsOffset = 0;
    const headOffset  = capacity * SLOT_BYTES;
    const tailOffset  = headOffset + 4;
    return { mem, ring: new CommandRing(mem, slotsOffset, headOffset, tailOffset, capacity) };
}

describe('CommandRing', () => {
    // The same wire values are asserted in core/src/command_ring.rs (wire_tags_are_pinned_to_the_values_the_typescript_ring_writes).
    it('pins the tag values of the recording commands', () => {
        expect([TAG_TELEPORT, TAG_RECORD, TAG_SEEK, TAG_PAUSE]).toEqual([8, 9, 10, 11]);
    });

    it('pins the tag values of the joint commands', () => {
        expect([TAG_JOINT_CREATE, TAG_JOINT_DESTROY, TAG_JOINT_MOTOR]).toEqual([12, 13, 14]);
    });

    // Same bytes as JOINT_FIXTURE_HEX in core/src/command_ring.rs.
    it('joint create bytes match the Rust cross-language fixture', () => {
        const { mem, ring } = makeRing();
        new Uint8Array(mem.buffer, 0, SLOT_BYTES).fill(0xff);
        expect(ring.jointCreate({ id: 7, kind: JOINT_HINGE, a: 3, genA: 1, b: 9, genB: 2, anchor: [1, 2, 3], axis: [0, 0, 1], limits: [-0.5, 0.75], motorVelocity: 2, motorMax: 10 })).toBe(true);
        const hex = Array.from(new Uint8Array(mem.buffer, 0, SLOT_BYTES)).map((b) => b.toString(16).padStart(2, '0')).join('');
        expect(hex).toBe('0c07000000' + '01' + '01' + '02' + '03000000' + '09000000' + '0000803f' + '00000040' + '00004040' + '00000000' + '00000000' + '0000803f' + '000000bf' + '0000403f' + '00000040' + '00002041' + '01' + '00000000000000');
    });

    it('joint create without limits leaves the flag and the limit words zero, and destroy and motor encode their payloads', () => {
        const { mem, ring } = makeRing();
        const view = new DataView(mem.buffer);
        ring.jointCreate({ id: 1, kind: 0, a: 0, genA: 0, b: 1, genB: 0, anchor: [0, 0, 0], axis: [0, 0, 0] });
        ring.jointDestroy(16_777_215);
        ring.jointMotor(5, -3.5, 12);
        expect([view.getUint8(56), view.getFloat32(40, true), view.getFloat32(44, true)]).toEqual([0, 0, 0]);
        const at = (n: number) => n * SLOT_BYTES;
        expect([view.getUint8(at(1)), view.getUint32(at(1) + 1, true)]).toEqual([TAG_JOINT_DESTROY, 16_777_215]);
        expect([view.getUint8(at(2)), view.getUint32(at(2) + 1, true), view.getFloat32(at(2) + 6, true), view.getFloat32(at(2) + 10, true)]).toEqual([TAG_JOINT_MOTOR, 5, -3.5, 12]);
    });

    it('a full ring refuses every joint command', () => {
        const { ring } = makeRing(1);
        expect(ring.jointDestroy(0)).toBe(true);
        expect([ring.jointCreate({ id: 0, kind: 0, a: 0, genA: 0, b: 1, genB: 0, anchor: [0, 0, 0], axis: [0, 0, 0] }), ring.jointDestroy(1), ring.jointMotor(0, 0, 0)]).toEqual([false, false, false]);
    });

    it('teleport encodes tag, id, gen and position and clears the rest of the slot', () => {
        const { mem, ring } = makeRing();
        new Uint8Array(mem.buffer, 0, SLOT_BYTES).fill(0xff);
        expect(ring.teleport({ id: 70000, gen: 255, x: 1.5, y: -2, z: 3 })).toBe(true);
        const view = new DataView(mem.buffer);
        expect([view.getUint8(0), view.getUint32(1, true), view.getUint8(5)]).toEqual([TAG_TELEPORT, 70000, 255]);
        expect([view.getFloat32(6, true), view.getFloat32(10, true), view.getFloat32(14, true)]).toEqual([1.5, -2, 3]);
        expect(view.getUint8(18)).toBe(0);
        expect(view.getUint8(63)).toBe(0);
    });

    it('record and pause encode a 0/1 flag at the payload byte', () => {
        const { mem, ring } = makeRing();
        const view = new DataView(mem.buffer);
        ring.record(true);
        ring.record(false);
        ring.pause(true);
        ring.pause(false);
        const at = (n: number) => [view.getUint8(n * SLOT_BYTES), view.getUint8(n * SLOT_BYTES + 6)];
        expect([at(0), at(1), at(2), at(3)]).toEqual([[TAG_RECORD, 1], [TAG_RECORD, 0], [TAG_PAUSE, 1], [TAG_PAUSE, 0]]);
    });

    it('seek encodes the tick as an unsigned 32-bit id field, including the maximum', () => {
        const { mem, ring } = makeRing();
        ring.seek(0);
        ring.seek(0xffffffff);
        const view = new DataView(mem.buffer);
        expect([view.getUint8(0), view.getUint32(1, true)]).toEqual([TAG_SEEK, 0]);
        expect([view.getUint8(SLOT_BYTES), view.getUint32(SLOT_BYTES + 1, true)]).toEqual([TAG_SEEK, 0xffffffff]);
    });

    it('the new commands report a full ring instead of overwriting', () => {
        const { ring } = makeRing();
        let n = 0;
        while (ring.seek(n)) n++;
        expect(n).toBeGreaterThan(0);
        expect([ring.teleport({ id: 0, gen: 0, x: 0, y: 0, z: 0 }), ring.record(true), ring.pause(true), ring.seek(1)]).toEqual([false, false, false, false]);
    });

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

    it('pins the kinematic batch tag and slot capacity', () => {
        expect([TAG_KINEMATIC_BATCH, KIN_SAMPLES_PER_SLOT]).toEqual([15, 3]);
    });

    // Same bytes as KIN_FIXTURE_HEX in core/src/command_ring.rs.
    it('kinematic batch bytes match the Rust cross-language fixture', () => {
        const { mem, ring } = makeRing();
        new Uint8Array(mem.buffer, 0, SLOT_BYTES).fill(0xff);
        expect(ring.kinematicBatch([
            { id: 5, gen: 1, ticks: 30, x: 1, y: 2, z: 3 },
            { id: 70_000, gen: 255, ticks: 65_535, x: -0.5, y: 0, z: 4 },
        ])).toBe(true);
        const hex = Array.from(new Uint8Array(mem.buffer, 0, SLOT_BYTES)).map((b) => b.toString(16).padStart(2, '0')).join('');
        expect(hex).toBe('0f' + '02' + '0000' + '05000000' + '01' + '00' + '1e00' + '0000803f' + '00000040' + '00004040'
            + '70110100' + 'ff' + '00' + 'ffff' + '000000bf' + '00000000' + '00008040' + '00000000'.repeat(5));
    });

    it('a kinematic batch takes one to three samples and reports a full ring without writing', () => {
        const { mem, ring } = makeRing(2);
        const s = { id: 1, gen: 0, ticks: 1, x: 0, y: 0, z: 0 };
        expect(() => ring.kinematicBatch([])).toThrow(RangeError);
        expect(() => ring.kinematicBatch([s, s, s, s])).toThrow(RangeError);
        expect(ring.kinematicBatch([s, s, s])).toBe(true);
        expect(ring.kinematicBatch([s])).toBe(true);
        const before = new Uint8Array(mem.buffer, 0, 2 * SLOT_BYTES).slice();
        expect(ring.kinematicBatch([{ ...s, id: 9 }])).toBe(false);
        expect(new Uint8Array(mem.buffer, 0, 2 * SLOT_BYTES)).toEqual(before);
    });

    it('pins the query tags', () => {
        expect([TAG_RAYCAST, TAG_OVERLAP_SPHERE]).toEqual([16, 17]);
    });

    // Same bytes as the query fixtures in core/src/command_ring.rs.
    it('query bytes match the Rust cross-language fixtures', () => {
        const hex = (mem: WebAssembly.Memory) => Array.from(new Uint8Array(mem.buffer, 0, SLOT_BYTES)).map((b) => b.toString(16).padStart(2, '0')).join('');
        const a = makeRing();
        new Uint8Array(a.mem.buffer, 0, SLOT_BYTES).fill(0xff);
        expect(a.ring.raycast({ id: 70_000, origin: [1, 2, 3], dir: [0, 0, -2], maxDistance: 50.5, ignore: 9 })).toBe(true);
        expect(hex(a.mem)).toBe('10' + '70110100' + '00' + '0000803f' + '00000040' + '00004040' + '00000000' + '00000000' + '000000c0' + '00004a42' + '09000000' + '00'.repeat(26));
        const b = makeRing();
        new Uint8Array(b.mem.buffer, 0, SLOT_BYTES).fill(0xff);
        expect(b.ring.overlapSphere({ id: 3, center: [-1, 0.5, 8], radius: 2.5, ignore: 0xffffffff })).toBe(true);
        expect(hex(b.mem)).toBe('11' + '03000000' + '00' + '000080bf' + '0000003f' + '00000041' + '00002040' + '00000000' + '00000000' + '00000000' + 'ffffffff' + '00'.repeat(26));
    });

    it('query writers report a full ring without touching it', () => {
        const { mem, ring } = makeRing(1);
        expect(ring.raycast({ id: 1, origin: [0, 0, 0], dir: [1, 0, 0], maxDistance: 1, ignore: 0xffffffff })).toBe(true);
        const before = new Uint8Array(mem.buffer, 0, SLOT_BYTES).slice();
        expect(ring.raycast({ id: 2, origin: [9, 9, 9], dir: [0, 1, 0], maxDistance: 2, ignore: 0 })).toBe(false);
        expect(ring.overlapSphere({ id: 3, center: [9, 9, 9], radius: 2, ignore: 0 })).toBe(false);
        expect(new Uint8Array(mem.buffer, 0, SLOT_BYTES)).toEqual(before);
    });
});
