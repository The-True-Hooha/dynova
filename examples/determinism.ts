import { TransformView, World, layoutFromRuntime, SHAPE_SPHERE } from '@dynova/sdk';

// Inputs are rounded to f32 stepwise, exactly like the native test. Same scene and hash as `physics_scene_matches_golden_hash` in core/src/runtime.rs.
async function run() {
    const glue = await import('../sdk/typescript/wasm-st/dynova_core.js');
    const exports = await glue.default() as unknown as { memory: WebAssembly.Memory };
    const config = { capacity: 64, ringCapacity: 64 };
    const rt = new glue.Runtime(config.capacity, config.ringCapacity);
    rt.enable_physics();
    const world = World.attachSync(config, exports.memory, 'fallback', layoutFromRuntime(rt), rt);
    const f = Math.fround;
    world.spawn({ y: -5, hx: 20, hy: 0.5, hz: 20, invMass: 0, restitution: 0.1 });
    for (let i = 1; i < 20; i++) {
        world.spawn({ x: f(f(i % 5) * f(1.1)), y: f(f(-4) + f(f(Math.floor(i / 5)) * f(1.1))), hx: 0.5, hy: 0.5, hz: 0.5, invMass: 1, restitution: 0.3 });
    }
    for (let i = 20; i < 30; i++) {
        world.spawn({ x: f(f(f(i % 5) * f(1)) + f(0.3)), y: f(f(0) + f(Math.floor(i / 5))), z: 0.2, shape: SHAPE_SPHERE, hx: 0.4, hy: 0.4, hz: 0.4, invMass: 1, restitution: 0.2 });
    }
    for (let i = 0; i < 300; i++) world.step();

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
    const scene = Array.from(glue.determinism_scene() as BigUint64Array, (v) => v.toString());
    const jointScene = Array.from(glue.determinism_joint_scene() as BigUint64Array, (v) => v.toString());
    const kinematicScene = Array.from(glue.determinism_kinematic_scene() as BigUint64Array, (v) => v.toString());
    const sensorScene = (glue.determinism_sensor_scene() as bigint).toString();
    return { hash: h.toString(), probe: glue.determinism_probe().toString(), scene, jointScene, kinematicScene, sensorScene, userAgent: navigator.userAgent };
}

run().then(
    (r) => { (window as any).__result = r; document.getElementById('out')!.textContent = JSON.stringify(r); },
    (e) => { (window as any).__result = { error: String(e) }; document.getElementById('out')!.textContent = String(e); },
);
