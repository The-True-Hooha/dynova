import { World, layoutFromRuntime } from '@dynova/sdk';

async function setup() {
    const glue = await import('../sdk/typescript/wasm-st/dynova_core.js');
    const exports = await glue.default() as unknown as { memory: WebAssembly.Memory };
    const rt = new glue.Runtime(64, 256);
    rt.enable_physics();
    return World.attachSync({ capacity: 64, ringCapacity: 256 }, exports.memory, 'fallback', layoutFromRuntime(rt), rt);
}

const CUBE = 0.3;

async function run() {
    const w = await setup();
    w.spawn({ y: -0.5, hx: 20, hy: 0.5, hz: 20, invMass: 0 });
    for (const s of [-1, 1]) {
        w.spawn({ x: s * 0.85, y: 0.5, hx: 0.25, hy: 1, hz: 1.1, invMass: 0 });
        w.spawn({ z: s * 0.85, y: 0.5, hx: 1.1, hy: 1, hz: 0.25, invMass: 0 });
    }
    const water = w.createFluid({ origin: [-0.375, 0.05, -0.375], n: [16, 12, 16], spacing: 0.05 });
    const sand = w.createSand({ origin: [3, 0.05, 0], n: [8, 8, 8], spacing: 0.05, friction: 0.6 });
    let cube: ReturnType<World['spawn']> | undefined;
    const times: number[] = [];
    for (let t = 0; t < 600; t++) {
        if (t === 150) cube = w.spawn({ y: 0.6, hx: CUBE / 2, hy: CUBE / 2, hz: CUBE / 2, invMass: 1 / (500 * CUBE ** 3) });
        const t0 = performance.now();
        w.step();
        if (t >= 450) times.push(performance.now() - t0);
    }
    const p = w.readParticles();
    const row = (i: number) => [p[i * 4], p[i * 4 + 1], p[i * 4 + 2]];
    const ys = Array.from({ length: water.count }, (_, k) => p[(water.first + k) * 4 + 1]).sort((a, b) => b - a);
    const top = ys.slice(0, Math.ceil(ys.length / 10));
    const surface = top.reduce((a, b) => a + b, 0) / top.length + 0.025;
    const cubeY = w.readTransforms()[cube!.id * 7 + 1];
    let leaked = 0, sandLowest = Infinity, sandMin = Infinity, sandMax = -Infinity;
    for (let k = 0; k < water.count; k++) {
        const [x, y, z] = row(water.first + k);
        if (Math.abs(x) > 0.6 || Math.abs(z) > 0.6 || y < 0) leaked++;
    }
    for (let k = 0; k < sand.count; k++) {
        const [x, y] = row(sand.first + k);
        sandLowest = Math.min(sandLowest, y); sandMin = Math.min(sandMin, x); sandMax = Math.max(sandMax, x);
    }
    times.sort((a, b) => a - b);
    return {
        particles: p.length / 4, leaked, surface, draft: surface - (cubeY - CUBE / 2), sandLowest, sandWidth: sandMax - sandMin,
        malformed: w.runtimeCounters.malformedDropped, medianStepMs: times[times.length >> 1], p95StepMs: times[Math.floor(times.length * 0.95)], userAgent: navigator.userAgent,
    };
}

run().then(
    (r) => { (window as any).__result = r; document.getElementById('out')!.textContent = JSON.stringify(r); },
    (e) => { (window as any).__result = { error: String(e) }; document.getElementById('out')!.textContent = String(e); },
);
