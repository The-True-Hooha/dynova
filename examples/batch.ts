import { World, WorldBatch, layoutFromRuntime, SHAPE_SPHERE } from '@dynova/sdk';

const WORLDS = 8;
const TICKS = 200;
const config = { capacity: 64, ringCapacity: 128 };

// Each world drops its own stack from a different height; the same scene run alone is the reference.
function scene(w: World, i: number) {
    w.spawn({ y: -1, hx: 20, hy: 1, hz: 20, invMass: 0, restitution: 0.1 });
    for (let k = 0; k < 6; k++) w.spawn({ x: k * 0.2, y: 1 + k * 1.1 + i * 0.3, hx: 0.5, hy: 0.5, hz: 0.5, invMass: 1, restitution: 0.2 });
    for (let k = 0; k < 4; k++) w.spawn({ x: -2 + k, y: 4 + i, z: 0.3, shape: SHAPE_SPHERE, hx: 0.4, invMass: 1, restitution: 0.3 });
}

const bits = (t: Float32Array) => Array.from(new Uint32Array(t.buffer, t.byteOffset, t.length));

async function run() {
    const glue = await import('../sdk/typescript/wasm-st/dynova_core.js');
    const exports = await glue.default() as unknown as { memory: WebAssembly.Memory };
    const batch = new glue.Batch(WORLDS, config.capacity, config.ringCapacity);
    batch.enable_physics();
    const set = new WorldBatch(config, exports.memory, 'fallback', batch);
    set.worlds.forEach(scene);
    const start = performance.now();
    for (let t = 0; t < TICKS; t++) set.step();
    const batchMs = performance.now() - start;

    const mismatched: number[] = [];
    for (let i = 0; i < WORLDS; i++) {
        const rt = new glue.Runtime(config.capacity, config.ringCapacity);
        rt.enable_physics();
        const solo = World.attachSync(config, exports.memory, 'fallback', layoutFromRuntime(rt), rt);
        scene(solo, i);
        for (let t = 0; t < TICKS; t++) solo.step();
        if (bits(solo.readTransforms()).join() !== bits(set.worlds[i].readTransforms()).join()) mismatched.push(i);
    }
    const heights = set.worlds.map((w) => w.readTransforms()[7 * 6 + 1]);
    return { worlds: WORLDS, ticks: TICKS, mismatched, distinct: new Set(heights).size, batchMs, userAgent: navigator.userAgent };
}

run().then(
    (r) => { (window as any).__result = r; document.getElementById('out')!.textContent = JSON.stringify(r); },
    (e) => { (window as any).__result = { error: String(e) }; document.getElementById('out')!.textContent = String(e); },
);
