import { describe, it, expect } from 'vitest';
import { GpuReacher, REACHER, reacherParams, type ReacherScene } from '../GpuReacher.js';

const f = Math.fround;

describe('the constants the kernel reads', () => {
    const p = reacherParams(REACHER);

    it('are laid out as the shader declares them, one vec4 at a time', () => {
        expect(p.length).toBe(80);
        expect(Array.from(p.subarray(0, 2))).toEqual([f(1 / 60), 1]);
        expect(p[3]).toBe(20);
        expect(Array.from(p.subarray(4, 7))).toEqual([0, f(-9.81), 0]);
        expect(Array.from(p.subarray(8, 10))).toEqual([-2.5, 2.5]);
        expect(p[13]).toBe(32);
        expect(p[14]).toBe(4);
        expect(Array.from(p.subarray(20, 23))).toEqual([-0.5, 0, 0]);
        expect(Array.from(p.subarray(24, 27))).toEqual([0, 0, 1]);
        expect(Array.from(p.subarray(40, 44))).toEqual([0, 0, 0, 1]);
    });

    it('take the scalar inertia of the box the way the core does: mass times 2/9 of the squared half sizes', () => {
        const [hx, hy, hz] = REACHER.linkHalf;
        expect(p[2]).toBeCloseTo(1 / ((2 / 9) * (hx * hx + hy * hy + hz * hz)), 4);
    });

    it('spread damping and the joint bias over the substeps by repeated square roots', () => {
        expect(p[10]).toBeCloseTo(0.99 ** 0.25, 6);
        expect(p[11]).toBeCloseTo(0.97 ** 0.25, 6);
        expect(p[12]).toBeCloseTo(1 - 0.8 ** 0.25, 6);
    });

    it('take the hinge reference direction perpendicular to the axis', () => {
        const axis = [0, 1, 0] as const;
        const q = reacherParams({ ...REACHER, axis });
        const ref = Array.from(q.subarray(32, 35));
        expect(ref[0] * axis[0] + ref[1] * axis[1] + ref[2] * axis[2]).toBe(0);
        expect(Math.hypot(...ref)).toBeCloseTo(1, 6);
    });
});

describe('a scene the CPU core would refuse is refused', () => {
    it.each<[string, Partial<typeof REACHER>]>([
        ['a link with no inverse mass', { invMass: 0 }],
        ['a negative motor force', { motorMax: -20 }],
        ['a zero axis', { axis: [0, 0, 0] }],
        ['a NaN limit', { limits: [Number.NaN, 1] }],
        ['limits that exclude zero', { limits: [0.5, 1] }],
        ['limits past 3.1 rad', { limits: [-4, 1] }],
        ['a NaN step', { dt: Number.NaN }],
        ['a NaN position', { link: [Number.NaN, 0, 0] }],
        ['a wrong-length vector', { gravity: [0, -9.81] as never }],
        ['a flat link', { linkHalf: [0.5, 0, 0.1] }],
    ])('%s', (_, change) => {
        expect(() => reacherParams({ ...REACHER, ...change })).toThrow(RangeError);
    });

    it('accepts the arm scene and a motor force of zero', () => {
        expect(() => reacherParams({ ...REACHER, motorMax: 0 })).not.toThrow();
    });
});

describe('a batch is refused before it touches the device', () => {
    it.each([0, -1, 1.5, Number.NaN, 2 ** 21])('for %s worlds', async (n) => {
        await expect(GpuReacher.create({} as GPUDevice, n)).rejects.toThrow(RangeError);
    });
});

describe('the joint kind', () => {
    it('is a hinge unless the scene says slider, and the kernel reads it from the last float of the pivot vec4', () => {
        expect(reacherParams(REACHER)[47]).toBe(0);
        expect(reacherParams({ ...REACHER, kind: 'hinge' })[47]).toBe(0);
        expect(reacherParams({ ...REACHER, kind: 'slider' })[47]).toBe(1);
    });

    it('bounds limits by metres for a slider and radians for a hinge, as the core does', () => {
        expect(() => reacherParams({ ...REACHER, limits: [-5, 5] })).toThrow(RangeError);
        expect(() => reacherParams({ ...REACHER, kind: 'slider', limits: [-5, 5] })).not.toThrow();
        expect(() => reacherParams({ ...REACHER, kind: 'slider', limits: [-2e4, 1] })).toThrow(RangeError);
        expect(() => reacherParams({ ...REACHER, kind: 'slider', limits: [0.1, 1] })).toThrow(RangeError);
    });

    it('refuses an unknown kind', () => {
        expect(() => reacherParams({ ...REACHER, kind: 'ball' as 'hinge' })).toThrow(/kind/);
    });
});

describe('two floors', () => {
    const a = { pos: [0, -1, 0] as const, half: [4, 0.5, 4] as const, friction: 0.3, restitution: 0.2 };
    const b = { pos: [0, 1, 0] as const, half: [3, 0.25, 2] as const, rot: [0, 0, 0.6, 0.8] as const, friction: 0.9, restitution: 0.6 };

    it('put the second box after the first, with its own friction, rotation and restitution', () => {
        const p = reacherParams({ ...REACHER, floors: [a, b] });
        expect(Array.from(p.subarray(48, 52))).toEqual([0, -1, 0, 1]);
        expect(Array.from(p.subarray(68, 72))).toEqual([0, 1, 0, 1]);
        expect(Array.from(p.subarray(72, 76))).toEqual([3, 0.25, 2, f(0.9)]);
        expect(Array.from(p.subarray(76, 80))).toEqual([0, 0, f(0.6), f(0.8)]);
        expect(Array.from(p.subarray(64, 67))).toEqual([0, f(0.2), f(0.6)]);
    });

    it('make `floor` the same as a list of one, and leave the second slot empty', () => {
        expect(reacherParams({ ...REACHER, floors: [a] })).toEqual(reacherParams({ ...REACHER, floor: a }));
        expect(reacherParams({ ...REACHER, floors: [a] })[71]).toBe(0);
    });

    it('refuse a third box, both spellings at once, and a bad second box', () => {
        expect(() => reacherParams({ ...REACHER, floors: [a, a, a] })).toThrow(RangeError);
        expect(() => reacherParams({ ...REACHER, floor: a, floors: [a] })).toThrow(RangeError);
        expect(() => reacherParams({ ...REACHER, floors: [a, { ...b, half: [0, 1, 1] }] })).toThrow(RangeError);
    });
});

describe('a distance joint', () => {
    const rod: ReacherScene = { ...REACHER, kind: 'distance', pivot: [0, 0, 0], link: [0.3, -1.2, 0.1], anchor: [0, -0.1, 0], axis: [0, -1.15, 0.15], limits: null, motorMax: 0 };

    it('carries its kind, the rest length, the anchor offsets of both bodies and whether it has limits', () => {
        const p = reacherParams(rod);
        expect(p[47]).toBe(2);
        expect(p[27]).toBeCloseTo(Math.hypot(0, 1.05, 0.15), 6);
        expect(Array.from(p.subarray(16, 19))).toEqual([0, f(-0.1), 0]);
        expect(Array.from(p.subarray(20, 23))).toEqual([f(-0.3), f(0.05), f(0.05)]);
        expect(p[31]).toBe(0);
        expect(reacherParams({ ...REACHER, kind: 'slider' })[31]).toBe(0);
        expect(reacherParams(REACHER)[27]).toBe(0);
        const rope = reacherParams({ ...rod, limits: [-0.4, 0] });
        expect(rope[31]).toBe(1);
        expect(Array.from(rope.subarray(8, 10))).toEqual([f(-0.4), 0]);
    });

    it.each<[string, Partial<ReacherScene>]>([
        ['two points less than a millimetre apart', { axis: [0, -0.1, 0.0005] }],
        ['a NaN second point', { axis: [0, Number.NaN, 0] }],
        ['a motor', { motorMax: 5 }],
        ['limits that exclude zero', { limits: [0.1, 1] }],
        ['limits past 1e4 m', { limits: [-2e4, 1] }],
    ])('is refused with %s', (_, change) => {
        expect(() => reacherParams({ ...rod, ...change })).toThrow(RangeError);
    });

    it('is the only kind that may go without limits', () => {
        expect(() => reacherParams({ ...REACHER, limits: null })).toThrow(/limits/);
        expect(() => reacherParams({ ...REACHER, kind: 'slider', limits: null })).toThrow(/limits/);
    });
});

describe('the floor constants', () => {
    const floor = { pos: [0, -1, 0] as const, half: [4, 0.5, 4] as const };

    it('say there is no floor unless the scene has one', () => {
        expect(reacherParams(REACHER)[51]).toBe(0);
        expect(Array.from(reacherParams(REACHER).subarray(52, 60))).toEqual([0.5, 0.5, 0.5, 0.5, 0, 0, 0, 1]);
        const p = reacherParams({ ...REACHER, floor });
        expect(Array.from(p.subarray(48, 52))).toEqual([0, -1, 0, 1]);
        expect(Array.from(p.subarray(52, 56))).toEqual([4, 0.5, 4, 0.5]);
        expect(Array.from(p.subarray(56, 60))).toEqual([0, 0, 0, 1]);
    });

    it('carry the link half sizes, both frictions and both restitutions', () => {
        const p = reacherParams({ ...REACHER, floor: { ...floor, friction: 0.2, restitution: 0.6 }, linkFriction: 0.9, linkRestitution: 0.3 });
        expect(Array.from(p.subarray(60, 64))).toEqual([...REACHER.linkHalf.map(f), f(0.9)]);
        expect(p[55]).toBe(f(0.2));
        expect(p[64]).toBe(f(0.3));
        expect(p[65]).toBe(f(0.6));
    });

    it('normalise a rotation that is unit within tolerance', () => {
        const h = Math.sin(0.15), c = Math.cos(0.15);
        const p = reacherParams({ ...REACHER, floor: { ...floor, rot: [0, 0, h, c] } });
        expect(Math.hypot(...Array.from(p.subarray(56, 60)))).toBeCloseTo(1, 6);
    });

    it('refuse what the core refuses for a body', () => {
        for (const bad of [
            { ...floor, pos: [Number.NaN, 0, 0] as const },
            { ...floor, half: [0, 1, 1] as const },
            { ...floor, rot: [0, 0, 0, 2] as const },
            { ...floor, rot: [0, 0, Number.NaN, 1] as const },
            { ...floor, half: [1e5, 1, 1] as const },
            { ...floor, friction: 3 },
            { ...floor, restitution: -0.1 },
        ]) expect(() => reacherParams({ ...REACHER, floor: bad })).toThrow(RangeError);
        expect(() => reacherParams({ ...REACHER, linkFriction: 5 })).toThrow(RangeError);
        expect(() => reacherParams({ ...REACHER, linkRestitution: 2 })).toThrow(RangeError);
    });
});
