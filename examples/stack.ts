import { TransformView, World, layoutFromRuntime } from '@dynova/sdk';

const COLS = 5;
const ROWS = 10;
const BOXES = COLS * ROWS;
const TICKS = 1500;

async function run() {
    const glue = await import('../sdk/typescript/wasm-st/dynova_core.js');
    const exports = await glue.default() as unknown as { memory: WebAssembly.Memory };
    const config = { capacity: BOXES + 1, ringCapacity: 64 };
    const rt = new glue.Runtime(config.capacity, config.ringCapacity);
    rt.enable_physics();
    const world = World.attachSync(config, exports.memory, 'fallback', layoutFromRuntime(rt), rt);
    world.spawn({ y: -0.5, hx: 30, hy: 0.5, hz: 30, invMass: 0, restitution: 0 });
    for (let r = 0; r < ROWS; r++) {
        for (let c = 0; c < COLS; c++) {
            const x = (c - (COLS - 1) / 2) * 1.02 + (r % 2 ? 0.5 : 0);
            world.spawn({ x, y: 0.5 + r * 1.02, hx: 0.5, hy: 0.5, hz: 0.5, invMass: 1, restitution: 0 });
        }
    }

    const times: number[] = [];
    for (let t = 0; t < TICKS; t++) {
        const t0 = performance.now();
        world.step();
        times.push(performance.now() - t0);
    }
    const view = new TransformView(exports.memory, rt.header_ptr());
    const buf = view.read();
    const ys = Array.from({ length: BOXES }, (_, i) => buf[(i + 1) * 7 + 1]);
    const sorted = [...times].sort((a, b) => a - b);
    return {
        avgMs: times.reduce((a, b) => a + b, 0) / times.length,
        p99Ms: sorted[Math.floor(sorted.length * 0.99)],
        maxMs: sorted[sorted.length - 1],
        finite: Array.from(buf).every(Number.isFinite),
        topY: Math.max(...ys),
        minY: Math.min(...ys),
        maxXZ: Math.max(...Array.from({ length: BOXES }, (_, i) => Math.max(Math.abs(buf[(i + 1) * 7]), Math.abs(buf[(i + 1) * 7 + 2])))),
        sleeping: rt.sleeping_count(),
    };
}

run().then(
    (r) => { (window as any).__result = r; document.getElementById('out')!.textContent = JSON.stringify(r); },
    (e) => { (window as any).__result = { error: String(e) }; document.getElementById('out')!.textContent = String(e); },
);
