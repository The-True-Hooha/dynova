import { describe, it, expect } from 'vitest';
import { positionControl } from '../control.js';

const gains = { kp: 4, kd: 0.5, maxSpeed: 2, maxForce: 30 };

describe('positionControl', () => {
    it('asks for a speed towards the target, proportional to the error', () => {
        expect(positionControl({ position: 0, velocity: 0 }, 0.25, gains)).toEqual({ velocity: 1, maxForce: 30 });
        expect(positionControl({ position: 0, velocity: 0 }, -0.25, gains)).toEqual({ velocity: -1, maxForce: 30 });
    });

    it('asks for nothing at the target when standing still', () => {
        expect(positionControl({ position: 0.7, velocity: 0 }, 0.7, gains).velocity).toBe(0);
    });

    it('takes speed off in proportion to the joint speed, so it brakes before the target', () => {
        const free = positionControl({ position: 0, velocity: 0 }, 0.25, gains).velocity;
        const moving = positionControl({ position: 0, velocity: 1 }, 0.25, gains).velocity;
        expect(moving).toBeCloseTo(free - 0.5, 10);
        expect(positionControl({ position: 0.25, velocity: 1 }, 0.25, gains).velocity).toBeCloseTo(-0.5, 10);
    });

    it('clamps the speed to the limit in both directions', () => {
        expect(positionControl({ position: 0, velocity: 0 }, 100, gains).velocity).toBe(2);
        expect(positionControl({ position: 0, velocity: 0 }, -100, gains).velocity).toBe(-2);
        expect(positionControl({ position: 0, velocity: 50 }, 0, { ...gains, kd: 10 }).velocity).toBe(-2);
    });

    it('passes the force limit through and allows zero gains and a zero limit', () => {
        expect(positionControl({ position: 0, velocity: 0 }, 1, { kp: 0, kd: 0, maxSpeed: 2, maxForce: 0 })).toEqual({ velocity: 0, maxForce: 0 });
        expect(positionControl({ position: 0, velocity: 0 }, 1, { ...gains, maxSpeed: 0 }).velocity).toBe(0);
    });

    it('rejects non-finite numbers and negative gains', () => {
        const s = { position: 0, velocity: 0 };
        for (const bad of [NaN, Infinity, -Infinity]) {
            expect(() => positionControl({ ...s, position: bad }, 0, gains)).toThrow(RangeError);
            expect(() => positionControl({ ...s, velocity: bad }, 0, gains)).toThrow(RangeError);
            expect(() => positionControl(s, bad, gains)).toThrow(RangeError);
            expect(() => positionControl(s, 0, { ...gains, kp: bad })).toThrow(RangeError);
        }
        for (const key of ['kp', 'kd', 'maxSpeed', 'maxForce'] as const) expect(() => positionControl(s, 0, { ...gains, [key]: -1 })).toThrow(RangeError);
    });
});
