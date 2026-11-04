import { TransformView } from '@dynova/sdk';

// Inputs are rounded to f32 stepwise, exactly like the native test. Same scene and hash as `physics_scene_matches_golden_hash` in core/src/runtime.rs.
async function run() {
    const glue = await import('../sdk/typescript/wasm-st/dynova_core.js');
    const exports = await glue.default() as unknown as { memory: WebAssembly.Memory };
    const rt = new glue.Runtime(64, 64);
    rt.enable_physics();
    rt.spawn_box(0, 0, -5, 0, 20, 0.5, 20, 0, 0.1);
    const f = Math.fround;
    for (let i = 1; i < 20; i++) {
        rt.spawn_box(i, f(f(i % 5) * f(1.1)), f(f(-4) + f(f(Math.floor(i / 5)) * f(1.1))), 0, 0.5, 0.5, 0.5, 1, 0.3);
    }
    for (let i = 20; i < 30; i++) {
        rt.spawn_sphere(i, f(f(f(i % 5) * f(1)) + f(0.3)), f(f(0) + f(Math.floor(i / 5))), 0.2, 0.4, 1, 0.2);
    }
    for (let i = 0; i < 300; i++) rt.step();

    const buf = new TransformView(exports.memory, rt.header_ptr()).read();
    const bits = new Uint32Array(1);
    const f32 = new Float32Array(bits.buffer);
    let h = 0n;
    for (let e = 0; e < 64; e++) {
        for (let k = 0; k < 3; k++) {
            f32[0] = buf[e * 7 + k];
            h = ((h * 1099511628211n) & 0xffffffffffffffffn) ^ BigInt(bits[0]);
        }
    }
    return { hash: h.toString(), userAgent: navigator.userAgent };
}

run().then(
    (r) => { (window as any).__result = r; document.getElementById('out')!.textContent = JSON.stringify(r); },
    (e) => { (window as any).__result = { error: String(e) }; document.getElementById('out')!.textContent = String(e); },
);
