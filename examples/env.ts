import { BatchEnv, ParallelBatch, WorldBatch, mulberry32 } from '@dynova/sdk';
import { reacherEnv } from './reacherEnv';

const config = { capacity: 16, ringCapacity: 64 };
// Ends an episode both ways: a swing past 0.2 rad either way terminates it, otherwise it is truncated after 40 steps.
const spec = { ...reacherEnv, maxSteps: 40, terminated: (_w: unknown, _i: number, _c: unknown, obs: Float32Array) => Math.abs(obs[0]) > 0.2 };
const STEPS = 130;
const makeWorker = () => new Worker(new URL('../sdk/typescript/src/batch.worker.ts', import.meta.url), { type: 'module' });

const actionAt = (n: number, t: number) => Float32Array.from({ length: n }, (_, i) => Math.sin(t * 0.3 + i) * 3);
const bits = (a: Float32Array) => Array.from(new Uint32Array(a.buffer, a.byteOffset, a.length)).join();

async function trajectory(env: BatchEnv<unknown>) {
    const rows: string[] = [bits(env.result.obs)];
    let resets = 0;
    let terminated = 0;
    let truncated = 0;
    for (let t = 0; t < STEPS; t++) {
        const r = await env.step(actionAt(env.size, t));
        rows.push(`${bits(r.obs)}|${bits(r.reward)}|${r.terminated.join('')}|${r.truncated.join('')}|${r.resetting.join('')}`);
        resets += r.resetting.reduce((a, b) => a + b, 0);
        terminated += r.terminated.reduce((a, b) => a + b, 0);
        truncated += r.truncated.reduce((a, b) => a + b, 0);
    }
    return { rows, resets, terminated, truncated };
}

async function rate(worlds: number, workers: number) {
    const batch = await ParallelBatch.launch({ config, worlds, workers, physics: true, createWorker: makeWorker });
    const env = await BatchEnv.create(batch, reacherEnv, { seed: 1 });
    await env.step(new Float32Array(worlds));
    const t0 = performance.now();
    for (let t = 0; t < 60; t++) await env.step(actionAt(worlds, t));
    const ms = performance.now() - t0;
    batch.dispose();
    return ms;
}

async function run() {
    const glue = await import('../sdk/typescript/wasm-st/dynova_core.js');
    const exports = await glue.default() as unknown as { memory: WebAssembly.Memory };
    const WORLDS = 12;
    const batch = new glue.Batch(WORLDS, config.capacity, config.ringCapacity);
    batch.enable_physics();
    const local = await BatchEnv.create(new WorldBatch(config, exports.memory, 'fallback', batch), spec, { seed: 5 });
    const a = await trajectory(local as BatchEnv<unknown>);

    const parallel = await ParallelBatch.launch({ config, worlds: WORLDS, workers: 4, physics: true, createWorker: makeWorker });
    const remote = await BatchEnv.create(parallel, spec, { seed: 5 });
    const b = await trajectory(remote as BatchEnv<unknown>);
    parallel.dispose();

    const firstDifferent = a.rows.findIndex((row, k) => row !== b.rows[k]);
    const rng = [mulberry32(1).next(), mulberry32(0).next()];
    const rate1 = await rate(256, 1);
    const rate4 = await rate(256, 4);
    return { worlds: WORLDS, steps: STEPS, firstDifferent, resets: a.resets, terminated: a.terminated, truncated: a.truncated, remoteResets: b.resets, rng, rate1Ms: rate1, rate4Ms: rate4, cores: navigator.hardwareConcurrency };
}

run().then(
    (r) => { (window as any).__result = r; document.getElementById('out')!.textContent = JSON.stringify(r); },
    (e) => { (window as any).__result = { error: String(e?.stack ?? e) }; document.getElementById('out')!.textContent = String(e?.stack ?? e); },
);
