import { describe, it, expect } from 'vitest';
import { readFileSync } from 'node:fs';
import { CommandRing, SLOT_BYTES, SHAPE_SPHERE, JOINT_HINGE } from '../CommandRing.js';
import { World, layoutFromRuntime } from '../World.js';
import { WorldBatch } from '../batch.js';
import { BatchEnv, type EnvResult, type EnvSpec } from '../env.js';
import { reacherEnv } from '../../../../examples/reacherEnv.js';

// The Python package pins itself to these two files (python/tests/test_wire.py, python/tests/test_env.py). Pinning the TypeScript side to the same files means a
// change to either language that moves a ring byte, a world trajectory or an environment trajectory fails in both.
const read = (name: string) => JSON.parse(new TextDecoder().decode(readFileSync(new URL(`../../../../python/tests/${name}`, import.meta.url))));
const WIRE = read('golden_ts.json');
const ENV = read('golden_env_ts.json');

const glue = await import('../../wasm-st/dynova_core.js');
const wasm = glue.initSync({ module: readFileSync(new URL('../../wasm-st/dynova_core_bg.wasm', import.meta.url)) });

describe('what the Python tests pin', () => {
    it('the ring writes the bytes of golden_ts.json (destroy and force leave the unused tail as it was)', () => {
        const mem = new WebAssembly.Memory({ initial: 1 });
        const ring = new CommandRing(mem, 0, 64 * SLOT_BYTES, 64 * SLOT_BYTES + 4, 64);
        const hex = (n: number) => Array.from(new Uint8Array(mem.buffer, n * SLOT_BYTES, SLOT_BYTES)).map((b) => b.toString(16).padStart(2, '0')).join('');
        new Uint8Array(mem.buffer, 0, 64 * SLOT_BYTES).fill(0xff);
        const writes: Array<[string, () => boolean]> = [
            ['spawn', () => ring.spawn({ id: 7, gen: 2, x: 1.5, y: -2.25, z: 3, qx: 0.1, qy: 0.2, qz: 0.3, qw: 0.9, shape: SHAPE_SPHERE, hx: 0.4, hy: 0.5, hz: 0.6, invMass: 2, restitution: 0.25, friction: 0.75, hull: 5 })],
            ['spawn_defaults', () => ring.spawn({ id: 1, gen: 0, x: 0, y: 0, z: 0, qx: 0, qy: 0, qz: 0, qw: 1 })],
            ['destroy', () => ring.destroy({ id: 9, gen: 3 })],
            ['force', () => ring.force({ id: 4, gen: 1, x: 1, y: 2, z: 3 })],
            ['teleport', () => ring.teleport({ id: 4, gen: 1, x: -1, y: 5, z: 0.5 })],
            ['gravity', () => ring.gravity(0, -9.81, 0.5)],
            ['grow', () => ring.grow(48)],
            ['record_on', () => ring.record(true)],
            ['pause_off', () => ring.pause(false)],
            ['seek', () => ring.seek(1234)],
            ['checkpoint', () => ring.checkpoint()],
            ['reset', () => ring.reset()],
            ['joint_destroy', () => ring.jointDestroy(11)],
            ['joint_motor', () => ring.jointMotor(3, -2.5, 40)],
            ['joint_hinge', () => ring.jointCreate({ id: 7, kind: JOINT_HINGE, a: 3, genA: 1, b: 9, genB: 2, anchor: [1, 2, 3], axis: [0, 0, 1], limits: [-0.5, 0.75], motorVelocity: 2, motorMax: 10 })],
            ['joint_ball_nolimits', () => ring.jointCreate({ id: 8, kind: 0, a: 1, genA: 0, b: 2, genB: 0, anchor: [0.5, 0.5, 0.5], axis: [0, 0, 0] })],
        ];
        const prefix: Record<string, number> = { destroy: 6, force: 18 };
        writes.forEach(([name, write], n) => {
            expect(write(), name).toBe(true);
            const cut = (prefix[name] ?? SLOT_BYTES) * 2;
            expect(hex(n).slice(0, cut), name).toBe((WIRE[name] as string).slice(0, cut));
        });
        expect(Object.keys(WIRE).filter((k) => k !== 'run_bits').sort()).toEqual(writes.map(([name]) => name).sort());
    });

    it('a dropped ball ends with the bits of golden_ts.json', () => {
        const rt = new glue.Runtime(16, 64);
        rt.enable_physics();
        const w = World.attachSync({ capacity: 16, ringCapacity: 64 }, wasm.memory, 'fallback', layoutFromRuntime(rt), rt);
        w.spawn({ y: -1, hx: 20, hy: 1, hz: 20, invMass: 0 });
        w.spawn({ y: 3, shape: SHAPE_SPHERE, hx: 0.5, invMass: 1 });
        for (let i = 0; i < 40; i++) w.step();
        const t = w.readTransforms();
        expect(Array.from(new Uint32Array(t.buffer, t.byteOffset, 14)).join(',')).toBe(WIRE.run_bits);
    });

    it('both environment trajectories have the digests of golden_env_ts.json', async () => {
        const specs: Record<string, EnvSpec<any>> = {
            both_ends: { ...reacherEnv, maxSteps: 40, terminated: (_w: unknown, _i: number, _c: unknown, obs: Float32Array) => Math.abs(obs[0]) > 0.2 },
            default: reacherEnv,
        };
        for (const [name, spec] of Object.entries(specs)) {
            const cfg = { capacity: 16, ringCapacity: 64 };
            const batch = new glue.Batch(3, cfg.capacity, cfg.ringCapacity);
            batch.enable_physics();
            const env = await BatchEnv.create(new WorldBatch(cfg, wasm.memory, 'fallback', batch), spec, { seed: 5 });
            let h = 0xcbf29ce484222325n;
            const word = (v: number) => { h = ((h ^ BigInt(v >>> 0)) * 0x100000001b3n) & 0xffffffffffffffffn; };
            const feed = (r: EnvResult) => {
                for (const x of new Uint32Array(r.obs.buffer, r.obs.byteOffset, r.obs.length)) word(x);
                for (const x of new Uint32Array(r.reward.buffer, r.reward.byteOffset, r.reward.length)) word(x);
                for (const flags of [r.terminated, r.truncated, r.resetting]) for (const x of flags) word(x);
            };
            feed(env.result);
            let terminated = 0, truncated = 0, resets = 0;
            const first: number[] = [];
            for (let t = 0; t < 130; t++) {
                const r = await env.step(Float32Array.from({ length: 3 }, (_, i) => (((t * 7 + i * 13) % 11) - 5) * 0.6));
                feed(r);
                if (t < 2) first.push(...new Uint32Array(r.obs.buffer, r.obs.byteOffset, r.obs.length));
                for (let i = 0; i < 3; i++) { terminated += r.terminated[i]; truncated += r.truncated[i]; resets += r.resetting[i]; }
            }
            expect({ digest: h.toString(), terminated, truncated, resets, first }, name).toEqual(ENV[name]);
        }
    });
});
