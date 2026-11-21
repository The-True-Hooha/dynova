import { InspectorView, TransformView, World, layoutFromRuntime, SHAPE_SPHERE } from '@dynova/sdk';

const params = new URLSearchParams(location.search);
const N = Number(params.get('n') ?? 1000);
const INSPECT = params.get('inspector') === '1';
const ACTIVE_TICKS = 120;
const LATE_TICKS = 240;
const DROP_HEIGHT = 6;
const SPACING = 1.15;

let seed = 1;
const rand = () => ((seed = (Math.imul(seed, 1664525) + 1013904223) >>> 0) / 2 ** 32);

interface Summary { avgMs: number; p50Ms: number; p99Ms: number; maxMs: number }

function summarize(samples: Float64Array): Summary {
    const sorted = Float64Array.from(samples).sort();
    const at = (q: number) => sorted[Math.min(sorted.length - 1, Math.floor(sorted.length * q))];
    return { avgMs: samples.reduce((a, b) => a + b, 0) / samples.length, p50Ms: at(0.5), p99Ms: at(0.99), maxMs: sorted[sorted.length - 1] };
}

function bucketSummary(counts: Uint32Array) {
    let used = 0;
    let max = 0;
    let total = 0;
    for (const c of counts) {
        if (c === 0) continue;
        used++;
        total += c;
        if (c > max) max = c;
    }
    return { buckets: counts.length, used, maxPerBucket: max, meanPerUsed: used === 0 ? 0 : total / used };
}

async function run() {
    const glue = await import('../sdk/typescript/wasm-st/dynova_core.js');
    const exports = await glue.default() as unknown as { memory: WebAssembly.Memory };
    const config = { capacity: N + 8, ringCapacity: 1024 };
    const rt = new glue.Runtime(config.capacity, config.ringCapacity);
    rt.enable_physics();
    const world = World.attachSync(config, exports.memory, 'fallback', layoutFromRuntime(rt), rt);

    const side = Math.ceil(Math.cbrt(N));
    const half = (side * SPACING) / 2 + 3;
    world.spawn({ y: -0.5, hx: half, hy: 0.5, hz: half, invMass: 0, restitution: 0 });
    for (const [x, z, hx, hz] of [[half, 0, 0.5, half], [-half, 0, 0.5, half], [0, half, half, 0.5], [0, -half, half, 0.5]]) {
        world.spawn({ x, y: 10, z, hx, hy: 10, hz, invMass: 0, restitution: 0 });
    }
    for (let i = 0; i < N; i++) {
        const gx = i % side;
        const gz = Math.floor(i / side) % side;
        const gy = Math.floor(i / (side * side));
        const x = (gx - (side - 1) / 2) * SPACING + (rand() - 0.5) * 0.4;
        const z = (gz - (side - 1) / 2) * SPACING + (rand() - 0.5) * 0.4;
        const y = DROP_HEIGHT + gy * SPACING + rand() * 0.4;
        if (i % 2 === 0) world.spawn({ x, y, z, shape: SHAPE_SPHERE, hx: 0.5, hy: 0.5, hz: 0.5, invMass: 1, restitution: 0.1 });
        else world.spawn({ x, y, z, hx: 0.5, hy: 0.5, hz: 0.5, invMass: 1, restitution: 0.1 });
    }

    let drainTicks = 0;
    while (world.pendingCommands > 0 && drainTicks < 10_000) { world.step(); drainTicks++; }

    const inspector = new InspectorView(exports.memory, rt.header_ptr());
    const snapshotMs: number[] = [];
    let snapshotsOk = 0;
    let lastSnapshotContacts = 0;
    let peakContacts = 0;
    const timed = (ticks: number) => {
        const samples = new Float64Array(ticks);
        for (let t = 0; t < ticks; t++) {
            const t0 = performance.now();
            world.step();
            samples[t] = performance.now() - t0;
            peakContacts = Math.max(peakContacts, world.runtimeStats.contacts);
            if (INSPECT && t % 15 === 0) {
                const r0 = performance.now();
                const snap = inspector.snapshot();
                snapshotMs.push(performance.now() - r0);
                if (snap) {
                    snapshotsOk++;
                    lastSnapshotContacts = snap.contactCount;
                }
            }
        }
        return samples;
    };
    if (INSPECT) {
        world.setInspector(true);
        world.step();
    }
    const active = timed(ACTIVE_TICKS);
    const sleepingAfterActive = rt.sleeping_count();
    const late = timed(LATE_TICKS);
    const stats = world.runtimeStats;
    const counters = world.runtimeCounters;
    const buf = new TransformView(exports.memory, rt.header_ptr()).read();
    return {
        n: N,
        drainTicks,
        pending: world.pendingCommands,
        active: summarize(active),
        late: summarize(late),
        peakContacts,
        inspector: INSPECT,
        snapshotMs: snapshotMs.length === 0 ? null : snapshotMs.reduce((a, b) => a + b, 0) / snapshotMs.length,
        snapshotsTaken: snapshotMs.length,
        snapshotsOk,
        lastSnapshotContacts,
        snapshotMissed: inspector.missedReads,
        sleepingAfterActive,
        droppedContacts: stats.droppedContacts,
        malformedDropped: counters.malformedDropped,
        sleeping: rt.sleeping_count(),
        finite: Array.from(buf).every(Number.isFinite),
        buckets: bucketSummary(world.bucketHistogram()),
    };
}

run().then(
    (r) => { (window as any).__result = r; document.getElementById('out')!.textContent = JSON.stringify(r, null, 1); },
    (e) => { (window as any).__result = { error: String(e) }; document.getElementById('out')!.textContent = String(e); },
);
