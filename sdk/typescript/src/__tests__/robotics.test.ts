import { describe, it, expect } from 'vitest';
import { armJoints, lookaheadIndex, pursue, solveArm, type PursuitOptions } from '../robotics.js';

const spec = { lengths: [1, 0.8, 0.5], limit: 2 };
const dist = (a: readonly number[], b: readonly number[]) => Math.hypot(a[0] - b[0], a[1] - b[1]);

describe('armJoints', () => {
    it('lays the links out along +x at zero angles', () => {
        expect(armJoints(spec.lengths, [0, 0, 0]).map(([x, y]) => [x, +y.toFixed(9)])).toEqual([[0, 0], [1, 0], [1.8, 0], [2.3, 0]]);
    });

    it('turns each joint relative to the link before it', () => {
        const p = armJoints([1, 1], [Math.PI / 2, -Math.PI / 2]);
        expect(p[1][0]).toBeCloseTo(0, 9);
        expect(p[1][1]).toBeCloseTo(1, 9);
        expect(p[2][0]).toBeCloseTo(1, 9);
        expect(p[2][1]).toBeCloseTo(1, 9);
    });

    it('is just the origin for an arm with no links', () => {
        expect(armJoints([], [])).toEqual([[0, 0]]);
    });
});

describe('solveArm', () => {
    it('reaches points inside the workspace within the tolerance and the joint limits', () => {
        for (const target of [[1.2, 1.0], [0.5, 1.6], [-0.4, 1.0], [2.0, 0.3], [0.0, -1.5]] as Array<[number, number]>) {
            const r = solveArm(spec, target, [0, 0, 0]);
            expect(r.reached, `${target}`).toBe(true);
            expect(dist(armJoints(spec.lengths, r.angles).pop()!, target)).toBeLessThan(0.01);
            for (const a of r.angles) expect(Math.abs(a)).toBeLessThanOrEqual(spec.limit + 1e-12);
        }
    });

    it('is deterministic: the same target and start give the same angles', () => {
        expect(solveArm(spec, [1.1, 0.9], [0, 0, 0])).toEqual(solveArm(spec, [1.1, 0.9], [0, 0, 0]));
    });

    it('reports a target past the reach as not reached and stretches towards it', () => {
        const r = solveArm(spec, [5, 0], [0, 0, 0]);
        expect(r.reached).toBe(false);
        expect(dist(armJoints(spec.lengths, r.angles).pop()!, [2.3, 0])).toBeLessThan(0.01);
    });

    it('reports a target inside the dead zone near the base as not reached', () => {
        expect(solveArm({ lengths: [1, 0.5], limit: 3 }, [0, 0], [0, 0]).reached).toBe(false);
    });

    it('never exceeds a tight joint limit', () => {
        const r = solveArm({ lengths: [1, 1], limit: 0.3 }, [0, 2], [0, 0]);
        expect(r.angles.every((a) => Math.abs(a) <= 0.3 + 1e-12)).toBe(true);
        expect(r.reached).toBe(false);
    });

    it('starts from the angles it is given and does not change them', () => {
        const start = [0.2, 0.1, 0.1];
        solveArm(spec, [1.2, 1.0], start);
        expect(start).toEqual([0.2, 0.1, 0.1]);
    });
});

const opts: PursuitOptions = { lookahead: 1, maxSpeed: 1, maxTurn: 1, arrive: 0.3, trackHalfWidth: 0.5, wheelRadius: 0.25 };
const path: Array<[number, number]> = [[2, 0], [4, 0], [4, 3]];

describe('lookaheadIndex', () => {
    it('picks the first waypoint farther than the lookahead, never before `from`', () => {
        expect(lookaheadIndex(path, 0, { x: 0, y: 0, yaw: 0 }, 1)).toBe(0);
        expect(lookaheadIndex(path, 0, { x: 1.5, y: 0, yaw: 0 }, 1)).toBe(1);
        expect(lookaheadIndex(path, 1, { x: 0, y: 0, yaw: 0 }, 1)).toBe(1);
    });

    it('needs a waypoint strictly farther than the lookahead, and chases the last one when none is', () => {
        expect(lookaheadIndex([[1, 0], [3, 0], [5, 0]], 0, { x: 0, y: 0, yaw: 0 }, 1)).toBe(1);
        expect(lookaheadIndex([[1, 0], [3, 0], [5, 0]], 0, { x: 0, y: 0, yaw: 0 }, 3)).toBe(2);
    });

    it('falls back to the last waypoint when all are close', () => {
        expect(lookaheadIndex(path, 0, { x: 4, y: 3, yaw: 0 }, 10)).toBe(2);
    });

    it('works on a one-point path', () => {
        expect(lookaheadIndex([[1, 1]], 0, { x: 0, y: 0, yaw: 0 }, 1)).toBe(0);
    });
});

describe('pursue', () => {
    it('drives both wheels equally when the goal is dead ahead', () => {
        const r = pursue(path, 0, { x: 0, y: 0, yaw: 0 }, opts);
        expect(r.left).toBeCloseTo(r.right, 10);
        expect(r.left).toBeCloseTo(opts.maxSpeed / opts.wheelRadius, 10);
        expect(r.done).toBe(false);
    });

    it('speeds up the right wheel to turn left towards a goal on the left, and the reverse', () => {
        const left = pursue([[0, 3]], 0, { x: 0, y: 0, yaw: 0 }, opts);
        expect(left.right).toBeGreaterThan(left.left);
        const right = pursue([[0, -3]], 0, { x: 0, y: 0, yaw: 0 }, opts);
        expect(right.left).toBeGreaterThan(right.right);
    });

    it('turns in proportion to the heading error before the limit, 0.6 per radian', () => {
        const big = { ...opts, maxTurn: 10, trackHalfWidth: 1, wheelRadius: 1 };
        const r = pursue([[Math.cos(0.5) * 3, Math.sin(0.5) * 3]], 0, { x: 0, y: 0, yaw: 0 }, big);
        expect(r.right - r.left).toBeCloseTo(2 * 0.6 * 0.5, 9);
    });

    it('limits the turn rate: the wheel speed difference never exceeds 2 * maxTurn * trackHalfWidth / wheelRadius', () => {
        const r = pursue([[-5, 0.01]], 0, { x: 0, y: 0, yaw: 0 }, opts);
        expect(Math.abs(r.right - r.left)).toBeLessThanOrEqual(2 * opts.maxTurn * opts.trackHalfWidth / opts.wheelRadius + 1e-9);
    });

    it('wraps the heading error instead of turning the long way round', () => {
        // Facing just short of -x, a goal just past -x on the other side of the +-pi seam is a small left turn, not a near-full right turn.
        const a = pursue([[-3, -0.2]], 0, { x: 0, y: 0, yaw: Math.PI - 0.1 }, opts);
        expect(a.right).toBeGreaterThan(a.left);
        expect(a.right - a.left).toBeLessThan(0.5);
        const b = pursue([[-3, 0.2]], 0, { x: 0, y: 0, yaw: -Math.PI + 0.1 }, opts);
        expect(b.left).toBeGreaterThan(b.right);
        expect(b.left - b.right).toBeLessThan(0.5);
    });

    it('slows in a sharp turn, to cos(1.2) of full speed for any heading error past 1.2 rad', () => {
        const ahead = pursue([[5, 0]], 0, { x: 0, y: 0, yaw: 0 }, opts);
        const behind = pursue([[-5, 0]], 0, { x: 0, y: 0, yaw: 0 }, { ...opts, maxTurn: 0 });
        expect(behind.left).toBeCloseTo(Math.cos(1.2) * ahead.left, 10);
        const aside = pursue([[0, 5]], 0, { x: 0, y: 0, yaw: 0 }, { ...opts, maxTurn: 0 });
        expect(aside.left).toBeCloseTo(Math.cos(1.2) * ahead.left, 10);
        const mild = pursue([[5, 1]], 0, { x: 0, y: 0, yaw: 0 }, { ...opts, maxTurn: 0 });
        expect(mild.left).toBeCloseTo(Math.cos(Math.atan2(1, 5)) * ahead.left, 10);
    });

    it('stops within the arrive distance of the last waypoint and reports done', () => {
        const r = pursue(path, 0, { x: 4, y: 2.8, yaw: 1.5 }, opts);
        expect(r).toEqual({ left: 0, right: 0, index: 2, done: true });
        expect(pursue(path, 0, { x: 4, y: 2.6, yaw: 1.5 }, opts).done).toBe(false);
        expect(pursue(path, 0, { x: 4, y: 2.5, yaw: 1.5 }, { ...opts, arrive: 0.5 }).done, 'exactly at the arrive distance is not yet arrived').toBe(false);
    });

    it('returns the waypoint it is chasing so the caller can keep it', () => {
        expect(pursue(path, 0, { x: 1.5, y: 0, yaw: 0 }, opts).index).toBe(1);
        expect(pursue(path, 1, { x: 0, y: 0, yaw: 0 }, opts).index).toBe(1);
    });
});
