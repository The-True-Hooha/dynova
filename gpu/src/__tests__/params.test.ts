import { describe, it, expect } from 'vitest';
import { GpuReacher, REACHER, reacherParams } from '../GpuReacher.js';

const f = Math.fround;

describe('the constants the kernel reads', () => {
    const p = reacherParams(REACHER);

    it('are laid out as the shader declares them, one vec4 at a time', () => {
        expect(p.length).toBe(48);
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
