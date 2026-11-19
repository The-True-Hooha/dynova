import { ParallelBatch, WorldBatch, World, SHAPE_SPHERE, lookAt, type BatchRuntime, type CameraPose } from '@dynova/sdk';

const config = { capacity: 64, ringCapacity: 128 };
const WORLDS = 64;
const camera = { width: 64, height: 64, fovY: Math.PI / 3, near: 0.1, far: 60 };
const makeWorker = () => new Worker(new URL('../sdk/typescript/src/batch.worker.ts', import.meta.url), { type: 'module' });

function scene(w: World, i: number) {
    w.spawn({ y: -1, hx: 30, hy: 1, hz: 30, invMass: 0 });
    for (let k = 0; k < 30; k++) {
        const x = (k % 6) * 1.5 - 3.75;
        const z = -2 - Math.floor(k / 6) * 1.5 - (i % 5) * 0.1;
        if (k % 3 === 0) w.spawn({ x, y: 0.5, z, shape: SHAPE_SPHERE, hx: 0.5, invMass: 0 });
        else w.spawn({ x, y: 0.5, z, hx: 0.5, hy: 0.5, hz: 0.5, invMass: 0 });
    }
}

const poseOf = (i: number): CameraPose => lookAt([(i % 7) * 0.2 - 0.6, 2, 4], [0, 0.5, -5]);
const bits = (d: Float32Array) => Array.from(new Uint32Array(d.buffer, d.byteOffset, d.length)).join();
const median = (xs: number[]) => xs.slice().sort((a, b) => a - b)[Math.floor(xs.length / 2)];

async function run() {
    const glue = await import('../sdk/typescript/wasm-st/dynova_core.js');
    const exports = await glue.default() as unknown as { memory: WebAssembly.Memory };

    const raw = new glue.Batch(WORLDS, config.capacity, config.ringCapacity);
    raw.enable_physics();
    const single = new WorldBatch(config, exports.memory, 'fallback', raw as unknown as BatchRuntime);
    single.worlds.forEach(scene);
    single.step();
    const poses = single.worlds.map((_, i) => poseOf(i));
    const reference = single.render(camera, poses);
    const referenceBits = Array.from({ length: WORLDS }, (_, i) => bits(reference.world(i).depth) + '|' + reference.world(i).id.join());
    const hits = reference.id.filter((id) => id !== 0xffffffff).length;
    const oneThread: number[] = [];
    for (let k = 0; k < 7; k++) {
        const t0 = performance.now();
        single.render(camera, poses);
        oneThread.push(performance.now() - t0);
    }

    const parallel = await ParallelBatch.launch({ config, worlds: WORLDS, workers: 4, physics: true, createWorker: makeWorker });
    parallel.worlds.forEach(scene);
    await parallel.step();
    const images = await parallel.render(camera, poses);
    const mismatched = images.flatMap((img, i) => (img && bits(img.depth) + '|' + img.id.join() === referenceBits[i] ? [] : [i]));
    const fourWorkers: number[] = [];
    for (let k = 0; k < 7; k++) {
        const t0 = performance.now();
        await parallel.render(camera, poses);
        fourWorkers.push(performance.now() - t0);
    }
    parallel.dispose();

    // A wall square to the camera: every pixel reads its distance, to a millimetre.
    const wallBatch = new glue.Batch(1, 8, 64);
    wallBatch.enable_physics();
    const walls = new WorldBatch({ capacity: 8, ringCapacity: 64 }, exports.memory, 'fallback', wallBatch as unknown as BatchRuntime);
    const wall = walls.worlds[0].spawn({ z: -7, hx: 50, hy: 50, hz: 0.5, invMass: 0 });
    walls.step();
    const img = walls.render(camera, [{ position: [0, 0, 0], rotation: [0, 0, 0, 1] }]).world(0);
    const wallError = Math.max(...img.depth.map((z) => Math.abs(z - 6.5)));
    const wallIds = new Set(img.id);

    return {
        worlds: WORLDS, pixels: camera.width * camera.height, hits, mismatched, wallError, wallIdsOk: wallIds.size === 1 && wallIds.has(wall.id),
        oneThreadMs: median(oneThread), fourWorkersMs: median(fourWorkers), cores: navigator.hardwareConcurrency, userAgent: navigator.userAgent,
    };
}

run().then(
    (r) => { (window as any).__result = r; document.getElementById('out')!.textContent = JSON.stringify(r); },
    (e) => { (window as any).__result = { error: String(e) }; document.getElementById('out')!.textContent = String(e); },
);
