import { describe, it, expect, vi } from 'vitest';
import { readFileSync } from 'node:fs';
import { WorldBatch } from '../batch.js';
import { BatchEnv, mulberry32, episodeSeed, type EnvSpec, type EnvResult } from '../env.js';
import { reacherEnv } from '../../../../examples/reacherEnv.js';

// The real single-threaded wasm module: reset, checkpoint and joint states all run in Rust.
const glue = await import('../../wasm-st/dynova_core.js');
const wasm = glue.initSync({ module: readFileSync(new URL('../../wasm-st/dynova_core_bg.wasm', import.meta.url)) });
const config = { capacity: 16, ringCapacity: 64 };

type Reacher = typeof reacherEnv extends EnvSpec<infer C> ? C : never;

function stepper(worlds: number, cfg = config, physics = true): WorldBatch {
    const batch = new glue.Batch(worlds, cfg.capacity, cfg.ringCapacity);
    if (physics) batch.enable_physics();
    return new WorldBatch(cfg, wasm.memory, 'fallback', batch);
}

const make = (worlds: number, seed = 0, spec: EnvSpec<Reacher> = reacherEnv) => BatchEnv.create(stepper(worlds), spec, { seed });
const copy = (r: EnvResult) => ({ obs: Array.from(r.obs), reward: Array.from(r.reward), terminated: Array.from(r.terminated), truncated: Array.from(r.truncated), resetting: Array.from(r.resetting) });
/** A fixed action that depends on the world and the step, so every world is driven differently. */
const action = (n: number, t: number) => Float32Array.from({ length: n }, (_, i) => Math.sin(t * 0.3 + i) * 3);

async function run(env: BatchEnv<Reacher>, steps: number, from = 0) {
    const out = [];
    for (let t = from; t < from + steps; t++) out.push(copy(await env.step(action(env.size, t))));
    return out;
}

describe('the seeded generator', () => {
    it('draws the values mulberry32 is known for, from integer arithmetic only', () => {
        const r = mulberry32(1);
        expect([r.next(), r.next(), r.next()]).toEqual([0.6270739405881613, 0.002735721180215478, 0.5274470399599522]);
        expect(mulberry32(0).next()).toBe(0.26642920868471265);
        expect(mulberry32(4294967295).next()).toBe(0.8964226141106337);
        expect(mulberry32(-1).next(), 'a negative seed is the same as its unsigned form').toBe(0.8964226141106337);
    });

    it('keeps range and int inside their bounds', () => {
        const r = mulberry32(99);
        for (let k = 0; k < 2000; k++) {
            const x = r.range(-2, 3);
            expect(x >= -2 && x < 3).toBe(true);
            const n = r.int(7);
            expect(Number.isInteger(n) && n >= 0 && n < 7).toBe(true);
        }
    });

    it('gives every world and episode its own seed, and pins the mixing', () => {
        const seen = new Set<number>();
        for (let w = 0; w < 64; w++) for (let e = 0; e < 32; e++) seen.add(episodeSeed(0, w, e));
        expect(seen.size).toBe(64 * 32);
        expect(episodeSeed(1, 3, 3)).not.toBe(episodeSeed(2, 3, 3));
        expect([episodeSeed(0, 0, 0), episodeSeed(0, 1, 0), episodeSeed(5, 3, 2)]).toEqual([2746606380, 4026859071, 1330923999]);
    });
});

describe('a BatchEnv over real wasm worlds', () => {
    it('starts every world at its checkpoint with the target its own seed draws', async () => {
        const env = await make(4, 11);
        const r = env.result;
        for (let i = 0; i < 4; i++) {
            const target = mulberry32(episodeSeed(11, i, 0)).range(-1.2, 1.2);
            expect(Array.from(r.obs.subarray(i * 3, i * 3 + 3)), `world ${i}`).toEqual([0, 0, Math.fround(target)]);
        }
        expect(Array.from(r.resetting)).toEqual([1, 1, 1, 1]);
        expect(Array.from(r.reward)).toEqual([0, 0, 0, 0]);
        expect(new Set(Array.from(r.obs).filter((_, k) => k % 3 === 2)).size, 'targets differ between worlds').toBe(4);
    });

    it('runs the same trajectory twice from the same seed, and another from a different seed', async () => {
        const a = await run(await make(3, 5), 120);
        const b = await run(await make(3, 5), 120);
        expect(b).toEqual(a);
        const c = await run(await make(3, 6), 120);
        expect(c).not.toEqual(a);
        expect(Math.max(...a.map((s) => Math.abs(s.obs[0]))), 'the actions move the arm').toBeGreaterThan(0.3);
    });

    it('gives a world the same history whatever number of worlds it runs beside', async () => {
        const small = await run(await make(2, 9), 120);
        const large = await run(await make(5, 9), 120);
        small.forEach((s, t) => {
            expect(s.obs, `step ${t}`).toEqual(large[t].obs.slice(0, 6));
            expect(s.reward).toEqual(large[t].reward.slice(0, 2));
        });
    });

    it('truncates at maxSteps and resets on the step after, ignoring that step\'s action', async () => {
        const short = { ...reacherEnv, maxSteps: 10 };
        const idle = new Float32Array(2);
        const env = await make(2, 3, short);
        const states = [];
        for (let t = 0; t < 9; t++) states.push(copy(await env.step(idle)));
        expect(states.every((s) => s.truncated.every((f) => f === 0))).toBe(true);
        const last = copy(await env.step(idle));
        expect([last.truncated, last.terminated, last.resetting]).toEqual([[1, 1], [0, 0], [0, 0]]);
        const afterA = copy(await env.step(Float32Array.of(3, -3)));
        expect(afterA.resetting).toEqual([1, 1]);
        expect(afterA.reward).toEqual([0, 0]);
        expect([afterA.truncated, afterA.terminated]).toEqual([[0, 0], [0, 0]]);
        for (let i = 0; i < 2; i++) {
            const target = Math.fround(mulberry32(episodeSeed(3, i, 1)).range(-1.2, 1.2));
            expect(afterA.obs.slice(i * 3, i * 3 + 3), `world ${i} starts episode 1 from the checkpoint`).toEqual([0, 0, target]);
        }
        let next = copy(await env.step(idle));
        for (let t = 1; t < 9; t++) next = copy(await env.step(idle));
        expect(next.truncated, 'the new episode counts its steps from zero').toEqual([0, 0]);
        expect(copy(await env.step(idle)).truncated).toEqual([1, 1]);
    });

    it('the action given on a reset step is ignored', async () => {
        const short = { ...reacherEnv, maxSteps: 1 };
        const a = await make(1, 4, short);
        const b = await make(1, 4, short);
        await a.step(Float32Array.of(0));
        await b.step(Float32Array.of(0));
        const ra = copy(await a.step(Float32Array.of(3)));
        const rb = copy(await b.step(Float32Array.of(-3)));
        expect(ra).toEqual(rb);
        expect(copy(await a.step(Float32Array.of(1)))).toEqual(copy(await b.step(Float32Array.of(1))));
    });

    it('terminates on the spec\'s say-so and flags that step only', async () => {
        const instant: EnvSpec<Reacher> = { ...reacherEnv, terminated: () => true };
        const env = await make(2, 1, instant);
        const s1 = copy(await env.step(new Float32Array(2)));
        expect([s1.terminated, s1.truncated, s1.resetting]).toEqual([[1, 1], [0, 0], [0, 0]]);
        expect(s1.reward.every((r) => r <= 0)).toBe(true);
        const s2 = copy(await env.step(new Float32Array(2)));
        expect([s2.terminated, s2.resetting, s2.reward]).toEqual([[0, 0], [1, 1], [0, 0]]);
    });

    it('a reset returns the world to where an episode starts and replays the same run', async () => {
        const env = await make(3, 21);
        const start = copy(env.result);
        const first = await run(env, 60);
        expect(first[59].obs).not.toEqual(start.obs);
        expect(copy(await env.reset())).toEqual(start);
        expect(await run(env, 60)).toEqual(first);
    });

    it('a new seed on reset changes the targets and the same seed again repeats them', async () => {
        const env = await make(2, 1);
        const one = copy(await env.reset(7));
        const two = copy(await env.reset(8));
        expect(two.obs).not.toEqual(one.obs);
        expect(copy(await env.reset(7))).toEqual(one);
    });

    it('moves the arm with the action and keeps handles working after resets', async () => {
        const env = await make(1, 2);
        for (let t = 0; t < 60; t++) await env.step(Float32Array.of(3));
        expect(env.result.obs[0]).toBeGreaterThan(0.3);
        await env.reset();
        for (let t = 0; t < 60; t++) await env.step(Float32Array.of(-3));
        expect(env.result.obs[0]).toBeLessThan(-0.3);
    });

    it('reuses its buffers instead of allocating a result per step', async () => {
        const env = await make(2, 1);
        const a = await env.step(new Float32Array(2));
        const obs = a.obs;
        const b = await env.step(new Float32Array(2));
        expect(b).toBe(a);
        expect(b.obs).toBe(obs);
    });

    it('rejects hostile calls with a message and leaves the environment usable', async () => {
        const env = await make(2, 1);
        await expect(env.step(new Float32Array(3))).rejects.toThrow(/expected 2 action values/);
        await expect(env.step(Float32Array.of(0, NaN))).rejects.toThrow(/action 1 \(world 1\) is not finite/);
        await expect(env.step(Float32Array.of(Infinity, 0))).rejects.toThrow(/action 0 \(world 0\) is not finite/);
        const pending = env.step(new Float32Array(2));
        await expect(env.step(new Float32Array(2))).rejects.toThrow(/already running/);
        await pending;
        await expect(env.step(new Float32Array(2))).resolves.toBeDefined();
    });

    it('refuses an empty batch and sizes that are not positive integers', async () => {
        for (const bad of [0, -1, 1.5, NaN]) {
            await expect(make(1, 0, { ...reacherEnv, obsSize: bad })).rejects.toThrow(/obsSize/);
            await expect(make(1, 0, { ...reacherEnv, actionSize: bad })).rejects.toThrow(/actionSize/);
            await expect(make(1, 0, { ...reacherEnv, maxSteps: bad })).rejects.toThrow(/maxSteps/);
        }
        await expect(BatchEnv.create({ worlds: [], step() {} }, reacherEnv)).rejects.toThrow(/at least one world/);
    });

    it('names the world and the value when an observation or a reward is not finite', async () => {
        const badObs: EnvSpec<Reacher> = { ...reacherEnv, observe: (w, i, c, out) => { reacherEnv.observe(w, i, c, out); if (i === 1) out[1] = NaN; } };
        await expect(make(2, 0, badObs)).rejects.toThrow(/world 1 observed a non-finite value at index 1/);
        const badReward: EnvSpec<Reacher> = { ...reacherEnv, reward: () => Infinity };
        const env = await make(1, 0, badReward);
        await expect(env.step(new Float32Array(1))).rejects.toThrow(/world 0 gave a non-finite reward/);
    });

    it('stops with the failing world named when a world has failed', async () => {
        const set = stepper(3);
        const env = await BatchEnv.create(set, reacherEnv, { seed: 1 });
        set.worlds[2].markFatal('worker lost');
        await expect(env.step(new Float32Array(3))).rejects.toThrow(/world 2 act: world unavailable: worker lost/);
        const set2 = stepper(3);
        const env2 = await BatchEnv.create(set2, reacherEnv);
        vi.spyOn(set2, 'step').mockImplementation(() => { set2.worlds[1].markFatal('worker lost'); });
        await expect(env2.reset()).rejects.toThrow(/worlds failed: 1: worker lost/);
    });

    it('checks every seed is an integer in [0, 2^32) and leaves the environment alone when one is not', async () => {
        for (const bad of [NaN, 1.5, -1, 2 ** 32, Infinity]) await expect(make(1, bad)).rejects.toThrow(/seed must be an integer/);
        const env = await make(2, 3);
        const before = copy(env.result);
        await expect(env.reset(NaN)).rejects.toThrow(/seed must be an integer/);
        expect(copy(env.result)).toEqual(before);
        await expect(env.step(new Float32Array(2))).resolves.toBeDefined();
        await expect(make(1, 2 ** 32 - 1)).resolves.toBeDefined();
    });

    it('terminated wins over truncated when both happen on one step', async () => {
        const env = await make(1, 0, { ...reacherEnv, maxSteps: 1, terminated: () => true });
        const r = copy(await env.step(new Float32Array(1)));
        expect([r.terminated, r.truncated]).toEqual([[1], [0]]);
    });
});

describe('what a BatchEnv does when things go wrong half way', () => {
    it('names the world and the hook when a hook throws, then refuses every later call', async () => {
        const boom: EnvSpec<Reacher> = { ...reacherEnv, act: (w, i, c, a) => { if (i === 1) throw new Error('boom'); reacherEnv.act(w, i, c, a); } };
        const env = await make(3, 1, boom);
        await expect(env.step(new Float32Array(3))).rejects.toThrow('world 1 act: boom');
        await expect(env.step(new Float32Array(3))).rejects.toThrow(/unusable after an earlier failure: world 1 act: boom/);
        await expect(env.reset()).rejects.toThrow(/unusable/);
    });

    it('is unusable after a reward hook throws, because the worlds already stepped', async () => {
        const env = await make(2, 1, { ...reacherEnv, reward: (_w, i) => { if (i === 0) throw new Error('bad reward'); return 0; } });
        await expect(env.step(new Float32Array(2))).rejects.toThrow('world 0 reward: bad reward');
        await expect(env.step(new Float32Array(2))).rejects.toThrow(/unusable/);
    });

    it('builds a scene of more commands than the ring holds, and its checkpoint is the scene as built, unmoved', async () => {
        const big = { capacity: 256, ringCapacity: 64 };
        const BODIES = 200;
        const counting: EnvSpec<null> = {
            obsSize: 2, actionSize: 1, maxSteps: 50,
            build(world) {
                world.spawn({ x: 0, y: -5, z: 0, hx: 50, hy: 1, hz: 50, invMass: 0 });
                for (let k = 0; k < BODIES; k++) world.spawn({ x: k * 2, y: 5, z: 0, hx: 0.4, hy: 0.4, hz: 0.4, invMass: 1 });
                return null;
            },
            act() {},
            observe(world, _i, _c, out) {
                const t = world.readTransforms();
                let alive = 0;
                for (let id = 0; id <= BODIES; id++) if (t[id * 7 + 6] !== 0) alive++;
                out[0] = alive;
                out[1] = t[1 * 7 + 1];
            },
            reward: () => 0,
            terminated: () => false,
        };
        const env = await BatchEnv.create(stepper(2, big), counting, { seed: 1 });
        expect(Array.from(env.result.obs)).toEqual([BODIES + 1, 5, BODIES + 1, 5]);
        for (let t = 0; t < 30; t++) await env.step(new Float32Array(2));
        expect(env.result.obs[1], 'the bodies fell during the episode').toBeLessThan(4.9);
        await env.reset();
        expect(Array.from(env.result.obs)).toEqual([BODIES + 1, 5, BODIES + 1, 5]);
    });

    it('refuses a hook that queues more commands than the ring takes in one step, and is unusable after', async () => {
        const flood: EnvSpec<Reacher> = { ...reacherEnv, act: (w, i, c, a) => { for (let k = 0; k < 70; k++) w.setJointMotor(c.joint, a[0], 20); } };
        const env = await make(1, 1, flood);
        await expect(env.step(new Float32Array(1))).rejects.toThrow(/world 0 queued more commands than its ring of 64 slots/);
        await expect(env.step(new Float32Array(1))).rejects.toThrow(/unusable/);
    });

    it('fails create when the worlds have no physics, since the checkpoint is refused', async () => {
        await expect(BatchEnv.create(stepper(1, config, false), reacherEnv)).rejects.toThrow(/world 0: \d+ command\(s\) were refused while it was built/);
    });

    it('fails when a reset is refused because a recording runs, instead of carrying on with an unreset world', async () => {
        const recording: EnvSpec<Reacher> = { ...reacherEnv, build: (w, i, n) => { w.record(true); return reacherEnv.build(w, i, n); } };
        await expect(make(1, 0, recording)).rejects.toThrow(/world 0: \d+ command\(s\) were refused while it was reset/);
    });

    it('lets a body spawned mid-episode vanish at the reset and hands out clean ids afterwards', async () => {
        const set = stepper(1);
        const env = await BatchEnv.create(set, reacherEnv, { seed: 2 });
        const world = set.worlds[0];
        for (let t = 0; t < 3; t++) await env.step(new Float32Array(1));
        const mid = world.spawn({ x: 7, y: 7, z: 0, hx: 0.5, hy: 0.5, hz: 0.5, invMass: 0 });
        await env.step(new Float32Array(1));
        expect(world.readTransforms()[mid.id * 7]).toBe(7);
        await env.reset();
        expect(world.readTransforms()[mid.id * 7 + 6], 'the slot is a tombstone again').toBe(0);
        const again = world.spawn({ x: -4, y: 2, z: 0, hx: 0.5, hy: 0.5, hz: 0.5, invMass: 0 });
        await env.step(new Float32Array(1));
        expect(again.id).toBe(mid.id);
        expect(world.readTransforms().slice(again.id * 7, again.id * 7 + 3)).toEqual(Float32Array.of(-4, 2, 0));
    });
});
