import { describe, it, expect } from 'vitest';
import { readFileSync } from 'node:fs';
import { World, layoutFromRuntime } from '../World.js';
import { parseUrdf, spawnRobot, qrot, type SpawnedRobot } from '../robot.js';
import { trot, standing, DEFAULT_TROT } from '../legged.js';

const glue = await import('../../wasm-st/dynova_core.js');
const wasm = glue.initSync({ module: readFileSync(new URL('../../wasm-st/dynova_core_bg.wasm', import.meta.url)) });
const QUADRUPED = readFileSync(new URL('../../../../assets/robots/quadruped.urdf', import.meta.url), 'utf8');
const gains = { kp: 10, kd: 0.3, maxSpeed: 6, maxForce: 40 };

interface Run { x: number; z: number; y: number; maxTilt: number; minY: number; hash: string; atLimit: number }

function run(ticks: number, targets: (names: string[], seconds: number) => Record<string, number>, settle = 0): Run {
    const rt = new glue.Runtime(64, 1024);
    rt.enable_physics();
    const w = World.attachSync({ capacity: 64, ringCapacity: 1024 }, wasm.memory, 'fallback', layoutFromRuntime(rt), rt);
    w.spawn({ y: -1, hx: 100, hy: 1, hz: 50, invMass: 0, friction: 1 });
    const desc = parseUrdf(QUADRUPED);
    const bot: SpawnedRobot = spawnRobot(w, desc, { position: [0, 0.545, 0], friction: 1 });
    const names = desc.joints.map((j) => j.name);
    const trunk = bot.links.get('trunk')!;
    let maxTilt = 0, minY = Infinity, atLimit = 0;
    for (let t = 0; t < ticks; t++) {
        const target = targets(names, Math.max(0, t - settle) / 60);
        w.driveJoints(desc.joints.map((j) => ({ joint: bot.joints.get(j.name)!, target: target[j.name], gains })));
        w.step();
        const tr = w.readTransforms();
        const o = trunk.id * 7;
        const up = qrot(Array.from(tr.slice(o + 3, o + 7)) as [number, number, number, number], [0, 0, 1]);
        maxTilt = Math.max(maxTilt, Math.acos(Math.max(-1, Math.min(1, up[1]))));
        if (t >= settle) minY = Math.min(minY, tr[o + 1]);
    }
    for (const j of desc.joints) {
        const s = w.readJointStates().find((q) => q.joint === bot.joints.get(j.name)!.id)!;
        if (j.limit && (s.position <= j.limit.lower + 1e-3 || s.position >= j.limit.upper - 1e-3)) atLimit++;
    }
    const tr = w.readTransforms();
    const o = trunk.id * 7;
    return { x: tr[o], y: tr[o + 1], z: tr[o + 2], maxTilt, minY, atLimit, hash: Array.from(new Uint32Array(tr.buffer, tr.byteOffset, tr.length)).join(',') };
}

describe('the quadruped asset', () => {
    it('stands for 10,000 ticks: upright, at its standing height, in place, with no joint at a limit', () => {
        const r = run(10_000, (names) => standing(names));
        expect(r.y).toBeGreaterThan(0.48);
        expect(r.y).toBeLessThan(0.53);
        expect(r.maxTilt).toBeLessThan(0.05);
        expect(Math.hypot(r.x, r.z)).toBeLessThan(0.05);
        expect(r.atLimit).toBe(0);
    }, 60_000);

    it('trots forward without tipping, and the same run gives the same bits', () => {
        const gait = (names: string[], s: number) => trot(names, s);
        const a = run(1800, gait, 300);
        expect(a.x).toBeGreaterThan(4);
        expect(Math.abs(a.z)).toBeLessThan(2);
        expect(a.maxTilt).toBeLessThan(0.3);
        expect(a.minY).toBeGreaterThan(0.45);
        expect(run(1800, gait, 300).hash).toBe(a.hash);
    }, 60_000);

    it('without the stride the same lifting does not carry it far, so the sweep of the thighs is what moves it', () => {
        const still = run(1800, (n, s) => trot(n, s, { stride: 0 }), 300);
        expect(Math.abs(still.x)).toBeLessThan(1.5);
        expect(still.maxTilt).toBeLessThan(0.3);
        expect(run(1800, (n, s) => trot(n, s), 300).x - still.x).toBeGreaterThan(4);
    }, 60_000);
});

describe('trot', () => {
    const names = ['fl_abduct', 'fl_hip_pitch', 'fl_knee', 'hr_knee', 'tail'];
    it('starts at standing and eases in', () => {
        expect(Object.values(trot(names, 0, { ramp: 1 })).every((v) => v === 0)).toBe(true);
        const half = trot(names, 0.1, { ramp: 0.2, frequency: 1 }).fl_hip_pitch;
        const full = trot(names, 0.1, { ramp: 0, frequency: 1 }).fl_hip_pitch;
        expect(Math.abs(full)).toBeGreaterThan(0.1);
        expect(half).toBeCloseTo(full / 2, 12);
    });
    it('moves diagonal legs together and lifts a foot only in the second half of its cycle', () => {
        const all = ['fl_knee', 'hr_knee', 'fr_knee', 'hl_knee', 'fl_hip_pitch', 'hr_hip_pitch', 'fr_hip_pitch'];
        for (const t of [0.1, 0.2, 0.5, 0.9]) {
            const g = trot(all, t, { ramp: 0 });
            expect(g.fl_knee).toBe(g.hr_knee);
            expect(g.fl_hip_pitch).toBe(g.hr_hip_pitch);
            expect(g.fr_hip_pitch).toBeCloseTo(-g.fl_hip_pitch, 12);
        }
        expect(trot(['fl_knee'], 0.1, { ramp: 0, frequency: 1 }).fl_knee).toBe(0);
        expect(trot(['fl_knee'], 0.75, { ramp: 0, frequency: 1 }).fl_knee).toBeCloseTo(-DEFAULT_TROT.lift, 12);
    });
    it('refuses a negative or non-finite time and options', () => {
        expect(() => trot(names, -1)).toThrow(RangeError);
        expect(() => trot(names, Number.NaN)).toThrow(RangeError);
        expect(() => trot(names, 1, { stride: Number.POSITIVE_INFINITY })).toThrow(RangeError);
        expect(() => trot(names, 1, { ramp: -1 })).toThrow(RangeError);
        for (const frequency of [0, -1]) expect(() => trot(names, 1, { frequency })).toThrow(/positive frequency/);
    });
    it('only the twelve leg joints move, and a list with none is an error', () => {
        const g = trot(['fr_x_knee', 'frame_joint', 'FL_knee', 'fl_knee'], 0.75, { ramp: 0, frequency: 1 });
        expect(g).toMatchObject({ fr_x_knee: 0, frame_joint: 0, FL_knee: 0 });
        expect(g.fl_knee).not.toBe(0);
        expect(() => trot(['front_left_knee', 'tail'], 1)).toThrow(/no joint is named/);
    });
});
