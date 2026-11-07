import { TransformView, World, layoutFromRuntime, SHAPE_SPHERE } from '@dynova/sdk';

const N = 6000;
const SIDE = 20;

async function run() {
    const glue = await import('../sdk/typescript/wasm-st/dynova_core.js');
    const exports = await glue.default() as unknown as { memory: WebAssembly.Memory };
    const config = { capacity: N + 8, ringCapacity: 1024 };
    const rt = new glue.Runtime(config.capacity, config.ringCapacity);
    rt.enable_physics();
    const world = World.attachSync(config, exports.memory, 'fallback', layoutFromRuntime(rt), rt);
    const direct = new TransformView(exports.memory, rt.header_ptr());
    const bufferBefore = exports.memory.buffer;

    for (let i = 0; i < N; i++) {
        world.spawn({
            x: (i % SIDE) * 0.99, y: (Math.floor(i / SIDE) % SIDE) * 0.99 + 0.5, z: Math.floor(i / (SIDE * SIDE)) * 0.99,
            shape: SHAPE_SPHERE, hx: 0.5, hy: 0.5, hz: 0.5, invMass: 1, restitution: 0,
        });
    }
    for (let t = 0; t < 12; t++) world.step();

    const grew = exports.memory.buffer !== bufferBefore;
    const viaWorld = world.readTransforms();
    const viaDirect = direct.read();
    const late = world.spawn({ x: 500, y: 500, z: 500, shape: SHAPE_SPHERE, hx: 0.5, hy: 0.5, hz: 0.5, invMass: 1, restitution: 0 });
    for (let t = 0; t < 3; t++) world.step();
    const after = world.readTransforms();
    const o = late.id * 7;
    return {
        grew,
        contacts: world.runtimeStats.contacts,
        droppedContacts: world.runtimeStats.droppedContacts,
        worldLen: viaWorld.length,
        directLen: viaDirect.length,
        expectedLen: config.capacity * 7,
        finite: Array.from(after).every(Number.isFinite),
        lateSpawnY: after[o + 1],
        malformed: world.runtimeCounters.malformedDropped,
    };
}

run().then(
    (r) => { (window as any).__result = r; document.getElementById('out')!.textContent = JSON.stringify(r); },
    (e) => { (window as any).__result = { error: String(e) }; document.getElementById('out')!.textContent = String(e); },
);
