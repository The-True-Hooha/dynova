import { describe, it, expect } from 'vitest';
import { Assembly } from '../assembly.js';
import { makeWorld } from './helpers.js';
import { SLOT_BYTES, JOINT_BALL, JOINT_DISTANCE, JOINT_FIXED, JOINT_HINGE, JOINT_SLIDER } from '../CommandRing.js';
import { H_JSTATE_SEQ, H_JSTATE_ROWS, H_JSTATE_N } from '../TransformView.js';

const config = { capacity: 64, ringCapacity: 64 };

function rig(origin?: [number, number, number]) {
    const made = makeWorld(config);
    const asm = new Assembly(made.world, origin);
    const slot = (n: number) => {
        const off = made.layout.ringSlotsByteOffset + n * SLOT_BYTES;
        const f = (at: number) => made.rt.dv.getFloat32(off + at, true);
        return { tag: made.rt.dv.getUint8(off), kind: made.rt.dv.getUint8(off + 5), pos: [f(6), f(10), f(14)], anchor: [f(16), f(20), f(24)], axis: [f(28), f(32), f(36)], limits: [f(40), f(44)], flag: made.rt.dv.getUint8(off + 56) };
    };
    return { ...made, asm, slot };
}

describe('Assembly coordinates', () => {
    it('removes the origin in 64 bits, so a far-away model keeps millimetres that 32 bits would round away', () => {
        const origin: [number, number, number] = [123_456_789.25, -987_654_321.5, 5_000_000.125];
        const { asm } = rig(origin);
        const p: [number, number, number] = [123_456_789.5, -987_654_321.0, 5_000_000.126];
        const local = asm.toLocal(p);
        expect(local[0]).toBe(0.25);
        expect(local[1]).toBe(0.5);
        expect(local[2]).toBeCloseTo(0.001, 5);
        expect(Math.fround(p[0]) - Math.fround(origin[0]), 'removing the origin after rounding to 32 bits loses it').not.toBe(0.25);
        const back = asm.toGlobal(local);
        expect([back[0], back[1]]).toEqual([123_456_789.5, -987_654_321.0]);
        expect(back[2]).toBeCloseTo(5_000_000.126, 3);
    });

    it('toGlobal inverts toLocal for values the 32-bit grid can hold', () => {
        const { asm } = rig([1000, 2000, 3000]);
        const g: [number, number, number] = [1000.5, 1999.25, 3004];
        expect(asm.toGlobal(asm.toLocal(g))).toEqual(g);
    });

    it('rejects a non-finite or malformed origin', () => {
        const { world } = rig();
        for (const o of [[NaN, 0, 0], [0, Infinity, 0], [0, 0]]) expect(() => new Assembly(world, o as never)).toThrow(RangeError);
    });

    it('defaults to the world origin, where it changes nothing', () => {
        const { asm } = rig();
        expect(asm.origin).toEqual([0, 0, 0]);
        expect(asm.toLocal([1.5, -2.25, 3])).toEqual([1.5, -2.25, 3]);
    });
});

describe('Assembly parts and mates', () => {
    it('adds a part at its assembly coordinates, converted, and keeps everything else as given', () => {
        const { asm, slot, world } = rig([500_000, 0, 0]);
        const before = world.pendingCommands;
        const h = asm.addPart('base', { x: 500_001.5, y: 2, z: -3, hx: 0.5, invMass: 0 });
        expect(asm.part('base')).toBe(h);
        expect(slot(before).pos).toEqual([1.5, 2, -3]);
        expect(asm.partNames).toEqual(['base']);
    });

    it('refuses a duplicate part or mate name and a mate naming a part that does not exist', () => {
        const { asm } = rig();
        asm.addPart('a', { invMass: 0 });
        asm.addPart('b');
        expect(() => asm.addPart('a')).toThrow(/already exists/);
        asm.addMate('m', { type: 'ball', a: 'a', b: 'b', at: [0, 0, 0] });
        expect(() => asm.addMate('m', { type: 'ball', a: 'a', b: 'b', at: [0, 0, 0] })).toThrow(/already exists/);
        expect(() => asm.addMate('n', { type: 'fixed', a: 'a', b: 'nope', at: [0, 0, 0] })).toThrow(/no part named 'nope'/);
        expect(() => asm.part('zzz')).toThrow(/no part named/);
        expect(() => asm.mate('zzz')).toThrow(/no mate named/);
        expect(asm.mateNames).toEqual(['m']);
    });

    it('turns each mate type into the matching joint, with anchors in local coordinates and the axis untouched', () => {
        const { asm, slot, world } = rig([1_000_000, 0, 0]);
        asm.addPart('base', { x: 1_000_000, invMass: 0 });
        asm.addPart('arm', { x: 1_000_002 });
        const at = () => world.pendingCommands - 1;
        asm.addMate('fix', { type: 'fixed', a: 'base', b: 'arm', at: [1_000_001, 0, 0] });
        expect([slot(at()).kind, slot(at()).anchor]).toEqual([JOINT_FIXED, [1, 0, 0]]);
        asm.addMate('ball', { type: 'ball', a: 'base', b: 'arm', at: [1_000_001, 0, 0] });
        expect(slot(at()).kind).toBe(JOINT_BALL);
        asm.addMate('rev', { type: 'revolute', a: 'base', b: 'arm', at: [1_000_000.5, 1, 0], axis: [0, 0, 2], range: [-1, 1] });
        expect(slot(at())).toMatchObject({ kind: JOINT_HINGE, anchor: [0.5, 1, 0], axis: [0, 0, 2], limits: [-1, 1], flag: 1 });
        asm.addMate('slide', { type: 'slider', a: 'base', b: 'arm', at: [1_000_001, 0, 0], axis: [1, 0, 0], range: [0, 3] });
        expect(slot(at())).toMatchObject({ kind: JOINT_SLIDER, axis: [1, 0, 0], limits: [0, 3], flag: 1 });
        asm.addMate('rod', { type: 'distance', a: 'base', b: 'arm', pointA: [1_000_000, 0, 0], pointB: [1_000_002, 0, 1], range: [-0.5, 0.5] });
        expect(slot(at())).toMatchObject({ kind: JOINT_DISTANCE, anchor: [0, 0, 0], axis: [2, 0, 1], limits: [-0.5, 0.5], flag: 1 });
        expect(asm.mateNames).toEqual(['fix', 'ball', 'rev', 'slide', 'rod']);
    });

    it('rejects an unknown mate type and passes the world validation through', () => {
        const { asm } = rig();
        asm.addPart('a', { invMass: 0 });
        asm.addPart('b');
        expect(() => asm.addMate('x', { type: 'weld', a: 'a', b: 'b', at: [0, 0, 0] } as never)).toThrow(/unknown mate type/);
        expect(() => asm.addMate('y', { type: 'revolute', a: 'a', b: 'b', at: [0, 0, 0], axis: [0, 0, 0] })).toThrow(RangeError);
        expect(() => asm.addMate('z', { type: 'distance', a: 'a', b: 'b', pointA: [0, 0, 0], pointB: [0, 0, 0] })).toThrow(RangeError);
        expect(asm.mateNames).toEqual([]);
    });

    it('a failed mate leaves no name behind, so the name can be used again', () => {
        const { asm } = rig();
        asm.addPart('a', { invMass: 0 });
        asm.addPart('b');
        expect(() => asm.addMate('m', { type: 'slider', a: 'a', b: 'b', at: [0, 0, 0], axis: [0, 0, 0] })).toThrow();
        expect(() => asm.addMate('m', { type: 'ball', a: 'a', b: 'b', at: [0, 0, 0] })).not.toThrow();
    });
});

describe('Assembly states and drives', () => {
    const publish = (rt: { header: Int32Array; memory: WebAssembly.Memory }, rows: Array<[number, number, number, number]>) => {
        const AT = 250_000;
        const u = new Uint32Array(rt.memory.buffer, AT, 64 * 4);
        const f = new Float32Array(rt.memory.buffer, AT, 64 * 4);
        Atomics.add(rt.header, H_JSTATE_SEQ, 1);
        rows.forEach(([id, kind, pos, vel], n) => { u[n * 4] = id; u[n * 4 + 1] = kind; f[n * 4 + 2] = pos; f[n * 4 + 3] = vel; });
        Atomics.store(rt.header, H_JSTATE_ROWS, AT);
        Atomics.store(rt.header, H_JSTATE_N, rows.length);
        Atomics.add(rt.header, H_JSTATE_SEQ, 1);
    };
    const gains = { kp: 4, kd: 0, maxSpeed: 2, maxForce: 30 };

    function built() {
        const r = rig();
        r.asm.addPart('base', { invMass: 0 });
        r.asm.addPart('arm', { x: 1 });
        r.asm.addPart('car', { x: 3 });
        r.asm.addMate('shoulder', { type: 'revolute', a: 'base', b: 'arm', at: [0, 0, 0], axis: [0, 0, 1] });
        r.asm.addMate('rail', { type: 'slider', a: 'base', b: 'car', at: [3, 0, 0], axis: [1, 0, 0] });
        r.asm.addMate('weld', { type: 'fixed', a: 'arm', b: 'car', at: [2, 0, 0] });
        return r;
    }

    it('reports a state per mate that has a free coordinate and none for ball and fixed mates', () => {
        const { asm, rt } = built();
        const shoulder = asm.mate('shoulder').id, rail = asm.mate('rail').id;
        publish(rt, [[shoulder, 1, 0.5, 0.125], [rail, 3, 0.25, -0.25]]);
        expect(asm.states()).toEqual({ shoulder: { position: 0.5, velocity: 0.125 }, rail: { position: 0.25, velocity: -0.25 } });
    });

    it('is empty before the runtime has published anything', () => {
        expect(built().asm.states()).toEqual({});
    });

    it('drives the named mates towards their targets and reports the ones it had no state for', () => {
        const { asm, rt, world } = built();
        publish(rt, [[asm.mate('shoulder').id, 1, 0, 0]]);
        const before = world.pendingCommands;
        const skipped = asm.drive({ shoulder: 0.25, rail: 1 }, gains);
        expect(skipped).toEqual(['rail']);
        expect(world.pendingCommands - before).toBe(1);
        expect(() => asm.drive({ nope: 1 }, gains)).toThrow(/no mate named 'nope'/);
    });

    it('refuses to drive a distance mate instead of queueing a command the runtime would reject', () => {
        const { asm, rt, world } = built();
        asm.addMate('rod', { type: 'distance', a: 'base', b: 'car', pointA: [0, 0, 0], pointB: [3, 0, 0] });
        publish(rt, [[asm.mate('shoulder').id, 1, 0, 0], [asm.mate('rod').id, 4, 0, 0]]);
        const before = world.pendingCommands;
        expect(() => asm.drive({ shoulder: 0.2, rod: 0.1 }, gains)).toThrow(/no motor/);
        expect(world.pendingCommands).toBe(before);
    });
});
