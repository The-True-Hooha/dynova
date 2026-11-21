import { World, layoutFromRuntime } from '@dynova/sdk';

async function setup() {
    const glue = await import('../sdk/typescript/wasm-st/dynova_core.js');
    const exports = await glue.default() as unknown as { memory: WebAssembly.Memory };
    const rt = new glue.Runtime(64, 256);
    rt.enable_physics();
    return World.attachSync({ capacity: 64, ringCapacity: 256 }, exports.memory, 'fallback', layoutFromRuntime(rt), rt);
}

async function run() {
    const w = await setup();
    w.spawn({ y: -0.5, hx: 20, hy: 0.5, hz: 20, invMass: 0 });
    w.spawn({ y: 0.6, hx: 0.4, hy: 0.5, hz: 0.4, invMass: 0 });
    const rope = w.createChain({ from: [5, 5, 0], to: [7, 5, 0], count: 11, pinFirst: true });
    const cloth = w.createCloth({ origin: [-1, 2.2, -1], u: [2, 0, 0], v: [0, 0, 2], nu: 21, nv: 21, radius: 0.06 });
    const block = w.createSoftBlock({ origin: [3, 1, 3], size: [0.8, 0.8, 0.8], n: [4, 4, 4], radius: 0.13 });
    const times: number[] = [];
    for (let t = 0; t < 600; t++) {
        const t0 = performance.now();
        w.step();
        if (t >= 300) times.push(performance.now() - t0);
    }
    const p = w.readParticles();
    const y = (i: number) => p[i * 4 + 1];
    times.sort((a, b) => a - b);
    return {
        particles: p.length / 4, pin: [p[rope.first * 4], y(rope.first)], ropeTip: y(rope.first + 10), clothMiddle: y(cloth.first + 10 * 21 + 10),
        blockLowest: Math.min(...Array.from({ length: block.count }, (_, k) => y(block.first + k))), medianStepMs: times[times.length >> 1], userAgent: navigator.userAgent,
    };
}

run().then(
    (r) => { (window as any).__result = r; document.getElementById('out')!.textContent = JSON.stringify(r); },
    (e) => { (window as any).__result = { error: String(e) }; document.getElementById('out')!.textContent = String(e); },
);
