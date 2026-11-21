import { TransformView, World, layoutFromRuntime, SHAPE_HULL } from '@dynova/sdk';

function cube(h: number): number[] {
    const p: number[] = [];
    for (const x of [-h, h]) for (const y of [-h, h]) for (const z of [-h, h]) p.push(x, y, z);
    return p;
}

async function run() {
    const glue = await import('../sdk/typescript/wasm-st/dynova_core.js');
    const exports = await glue.default() as unknown as { memory: WebAssembly.Memory };
    const config = { capacity: 16, ringCapacity: 64 };
    const rt = new glue.Runtime(config.capacity, config.ringCapacity);
    rt.enable_physics();
    const world = World.attachSync(config, exports.memory, 'fallback', layoutFromRuntime(rt), rt);

    const cubeHull = world.registerHull(cube(0.5));
    const tetraHull = world.registerHull([1, 1, 1, 1, -1, -1, -1, 1, -1, -1, -1, 1]);
    const flatHull = world.registerHull([0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 1, 0]);

    world.spawn({ y: -0.5, hx: 30, hy: 0.5, hz: 30, invMass: 0, restitution: 0 });
    const rows = 4;
    const ids: number[] = [];
    for (let r = 0; r < rows; r++) {
        for (let c = 0; c < rows - r; c++) {
            ids.push(world.spawn({ shape: SHAPE_HULL, hull: cubeHull, x: (c - (rows - 1 - r) / 2), y: 0.5 + r, invMass: 1, restitution: 0 }).id);
        }
    }
    const tetra = world.spawn({ shape: SHAPE_HULL, hull: tetraHull, x: 6, y: 4, invMass: 1, restitution: 0 }).id;
    const rejected = world.spawn({ shape: SHAPE_HULL, hull: flatHull, x: -6, y: 4, invMass: 1, restitution: 0 }).id;

    for (let i = 0; i < 900; i++) world.step();

    const buf = new TransformView(exports.memory, rt.header_ptr()).read();
    const at = (id: number) => Array.from(buf.slice(id * 7, id * 7 + 7));
    return {
        topY: at(ids[ids.length - 1])[1],
        topX: at(ids[ids.length - 1])[0],
        tetra: at(tetra),
        rejected: at(rejected),
        sleeping: rt.sleeping_count(),
        malformed: world.runtimeCounters.malformedDropped,
        finite: Array.from(buf).every(Number.isFinite),
    };
}

run().then(
    (r) => { (window as any).__result = r; document.getElementById('out')!.textContent = JSON.stringify(r); },
    (e) => { (window as any).__result = { error: String(e) }; document.getElementById('out')!.textContent = String(e); },
);
