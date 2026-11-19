import { ParallelBatch, World, layoutFromRuntime, SHAPE_SPHERE } from '@dynova/sdk';

const config = { capacity: 128, ringCapacity: 128 };
const TICKS = 150;

function scene(w: World, i: number, bodies: number) {
    w.spawn({ y: -1, hx: 30, hy: 1, hz: 30, invMass: 0, restitution: 0.1 });
    for (let k = 0; k < bodies; k++) {
        const x = (k % 8) * 1.2 - 4;
        const z = (Math.floor(k / 8) % 8) * 1.2 - 4;
        const y = 1 + Math.floor(k / 64) * 1.2 + i * 0.05;
        if (k % 3 === 0) w.spawn({ x, y, z, shape: SHAPE_SPHERE, hx: 0.5, invMass: 1, restitution: 0.2 });
        else w.spawn({ x, y, z, hx: 0.5, hy: 0.5, hz: 0.5, invMass: 1, restitution: 0.2 });
    }
}

const bits = (t: Float32Array) => Array.from(new Uint32Array(t.buffer, t.byteOffset, t.length)).join();
const makeWorker = () => new Worker(new URL('../sdk/typescript/src/batch.worker.ts', import.meta.url), { type: 'module' });

// Bodies are dropped from a height each run, so the 40 timed ticks are mostly falling and colliding rather than asleep.
async function timed(worlds: number, workers: number, bodies: number) {
    const heavy = { capacity: bodies + 8, ringCapacity: 1024 };
    const batch = await ParallelBatch.launch({ config: heavy, worlds, workers, physics: true, createWorker: makeWorker });
    batch.worlds.forEach((w, i) => scene(w, i + 20, bodies));
    await batch.step();
    const t0 = performance.now();
    for (let t = 0; t < 40; t++) await batch.step();
    const ms = performance.now() - t0;
    batch.dispose();
    return ms;
}

async function run() {
    const glue = await import('../sdk/typescript/wasm-st/dynova_core.js');
    const exports = await glue.default() as unknown as { memory: WebAssembly.Memory };
    const WORLDS = 12;
    const batch = await ParallelBatch.launch({ config, worlds: WORLDS, workers: 4, physics: true, createWorker: makeWorker });
    batch.worlds.forEach((w, i) => scene(w, i, 40));
    let failed: number[] = [];
    for (let t = 0; t < TICKS; t++) failed = await batch.step();
    const mismatched: number[] = [];
    for (let i = 0; i < WORLDS; i++) {
        const rt = new glue.Runtime(config.capacity, config.ringCapacity);
        rt.enable_physics();
        const solo = World.attachSync(config, exports.memory, 'fallback', layoutFromRuntime(rt), rt);
        scene(solo, i, 40);
        for (let t = 0; t < TICKS; t++) solo.step();
        if (bits(solo.readTransforms()) !== bits(batch.worlds[i].readTransforms())) mismatched.push(i);
    }
    const distinct = new Set(batch.worlds.map((w) => w.readTransforms()[7 * 20 + 1])).size;
    batch.dispose();
    const cores = navigator.hardwareConcurrency;
    const oneWorker = await timed(64, 1, 300);
    const fourWorkers = await timed(64, 4, 300);
    return { worlds: WORLDS, workers: batch.workerCount, failed, mismatched, distinct, cores, oneWorkerMs: oneWorker, fourWorkersMs: fourWorkers, userAgent: navigator.userAgent };
}

run().then(
    (r) => { (window as any).__result = r; document.getElementById('out')!.textContent = JSON.stringify(r); },
    (e) => { (window as any).__result = { error: String(e) }; document.getElementById('out')!.textContent = String(e); },
);
