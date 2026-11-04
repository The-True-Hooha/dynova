import { TransformView } from '@dynova/sdk';

const BOXES = 50;
const TICKS = 600;

async function run() {
    const glue = await import('../sdk/typescript/wasm-st/dynova_core.js');
    const exports = await glue.default() as unknown as { memory: WebAssembly.Memory };
    const rt = new glue.Runtime(BOXES + 1, 64);
    rt.enable_physics();
    rt.spawn_box(0, 0, -0.5, 0, 20, 0.5, 20, 0, 0);
    for (let i = 1; i <= BOXES; i++) rt.spawn_box(i, 0, 0.5 + (i - 1) * 1.02, 0, 0.5, 0.5, 0.5, 1, 0);

    const times: number[] = [];
    for (let t = 0; t < TICKS; t++) {
        const t0 = performance.now();
        rt.step();
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
        topY: ys[BOXES - 1],
        minY: Math.min(...ys),
        maxXZ: Math.max(...Array.from({ length: BOXES }, (_, i) => Math.max(Math.abs(buf[(i + 1) * 7]), Math.abs(buf[(i + 1) * 7 + 2])))),
        sleeping: rt.sleeping_count(),
    };
}

run().then(
    (r) => { (window as any).__result = r; document.getElementById('out')!.textContent = JSON.stringify(r); },
    (e) => { (window as any).__result = { error: String(e) }; document.getElementById('out')!.textContent = String(e); },
);
