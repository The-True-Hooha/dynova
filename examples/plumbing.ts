import { FLOATS_PER_TRANSFORM } from '@dynova/sdk';
import { createWorld } from './shared';

const COUNT = 10_000;

async function run() {
    const t0 = performance.now();
    const world = await createWorld({ capacity: COUNT, ringCapacity: 16384 }, false);

    const handles = [];
    for (let i = 0; i < COUNT; i++) {
        handles.push(world.spawn({ x: i, y: i * 2, z: -i }));
    }

    const deadline = performance.now() + 10_000;
    while (world.pendingCommands > 0) {
        if (world.mode === 'fallback') world.step();
        else await new Promise((r) => setTimeout(r, 5));
        if (performance.now() > deadline) throw new Error(`timeout, pending=${world.pendingCommands}`);
    }
    if (world.mode === 'fallback') world.step();
    else await new Promise((r) => setTimeout(r, 50));

    const buf = world.readTransforms();
    let bad = -1;
    for (let i = 0; i < COUNT; i++) {
        const o = i * FLOATS_PER_TRANSFORM;
        if (buf[o] !== i || buf[o + 1] !== i * 2 || buf[o + 2] !== -i || buf[o + 6] !== 1) { bad = i; break; }
    }

    return {
        mode: world.mode,
        crossOriginIsolated: self.crossOriginIsolated,
        spawned: handles.length,
        allocated: world.allocatedEntities,
        firstBadEntity: bad,
        ms: Math.round(performance.now() - t0),
    };
}

run().then(
    (r) => { (window as any).__result = r; document.getElementById('out')!.textContent = JSON.stringify(r); },
    (e) => { (window as any).__result = { error: String(e) }; document.getElementById('out')!.textContent = String(e); },
);
