import { launchWorld, type EntityHandle } from '@dynova/sdk';

const TARGET_TICKS = 10_000;
const MAX_CAPACITY = 4096;
const noRead = new URLSearchParams(location.search).has('noread');

// Deterministic PRNG so a failing run can be replayed.
let seed = Number(new URLSearchParams(location.search).get('seed') ?? 1) >>> 0;
const rand = () => ((seed = (Math.imul(seed, 1664525) + 1013904223) >>> 0) / 2 ** 32);

function inspect(buf: Float32Array, capacity: number, expectLive: Set<number> | null) {
    let torn = 0;
    let live = 0;
    const wrong: string[] = [];
    for (let id = 0; id < capacity; id++) {
        const o = id * 7;
        const empty = buf[o + 3] === 0 && buf[o + 4] === 0 && buf[o + 5] === 0 && buf[o + 6] === 0
            && buf[o] === 0 && buf[o + 1] === 0 && buf[o + 2] === 0;
        const full = buf[o] === 1 && buf[o + 1] === 2 && buf[o + 2] === 3
            && buf[o + 3] === 0 && buf[o + 4] === 0 && buf[o + 5] === 0 && buf[o + 6] === 1;
        if (!empty && !full) torn++;
        if (full) live++;
        if (expectLive && full !== expectLive.has(id) && wrong.length < 10) wrong.push(`${id}:expectedLive=${expectLive.has(id)},actuallyFull=${full}`);
    }
    return { torn, live, wrong };
}

async function run() {
    const world = await launchWorld({
        config: { capacity: 64, ringCapacity: 64, maxDeferredCommands: 1 << 20 },
        physics: false,
        hz: 3000,
        createWorker: () => new Worker(new URL('../sdk/typescript/src/sim.worker.ts', import.meta.url), { type: 'module' }),
        loadFallback: async () => { throw new Error('fuzz needs shared memory'); },
    });
    if (world.mode !== 'shared') throw new Error(`expected shared mode, got ${world.mode}`);

    const live = new Map<number, EntityHandle>();
    const hist = new Map<number, string[]>();
    const log = (id: number, m: string) => { const a = hist.get(id) ?? []; a.push(m); hist.set(id, a); };
    let growPending = false;
    let requestedGrows = 0;
    let observedGrows = 0;
    let tornReads = 0;
    let reads = 0;
    let spawned = 0;
    let destroyed = 0;
    world.onBufferGrowth = () => { observedGrows++; growPending = false; };

    const deadline = performance.now() + 90_000;
    while (world.runtimeCounters.ticks < TARGET_TICKS) {
        if (performance.now() > deadline) throw new Error(`timeout at tick ${world.runtimeCounters.ticks}`);
        const burst = 1 + Math.floor(rand() * 200);
        for (let i = 0; i < burst; i++) {
            const r = rand();
            if (r < 0.55) {
                if (world.allocatedEntities - live.size >= 0 && live.size >= world.capacity) {
                    if (!growPending && world.capacity < MAX_CAPACITY) {
                        growPending = true; requestedGrows++;
                        world.requestGrow(Math.min(MAX_CAPACITY, world.capacity * 2));
                    }
                    continue;
                }
                try {
                    const h = world.spawn({ x: 1, y: 2, z: 3 });
                    live.set(h.id, h);
                    log(h.id, `S${h.gen}@cap${world.capacity}`);
                    spawned++;
                } catch (e) {
                    if (!(e instanceof RangeError)) throw e;
                }
            } else if (live.size > 0) {
                const ids = [...live.keys()];
                const id = ids[Math.floor(rand() * ids.length)];
                log(id, `D${live.get(id)!.gen}`);
                world.destroy(live.get(id)!);
                live.delete(id);
                destroyed++;
            }
        }
        if (!noRead) {
            const buf = world.readTransforms();
            reads++;
            tornReads += inspect(buf, world.capacity, null).torn;
        } else {
            world.flush();
            world.syncLayout();
        }
        await new Promise((r) => setTimeout(r, 0));
    }
    return { spawned, destroyed, reads, tornReads, requestedGrows, observedGrows, live, world, hist };
}

run().then(
    async (s) => {
        const { world, live } = s;
        const drainDeadline = performance.now() + 60_000;
        while (world.pendingCommands > 0 && performance.now() < drainDeadline) {
            world.flush();
            await new Promise((r) => setTimeout(r, 5));
        }
        const pendingAtEnd = world.pendingCommands;
        const t0 = world.runtimeCounters.ticks;
        while (world.runtimeCounters.ticks < t0 + 5) await new Promise((r) => setTimeout(r, 5));
        const expected = new Set(live.keys());
        const final = inspect(world.readTransforms(), world.capacity, expected);
        const c = world.runtimeCounters;
        const result = {
            spawned: s.spawned, destroyed: s.destroyed, reads: s.reads, tornReads: s.tornReads,
            requestedGrows: s.requestedGrows, observedGrows: s.observedGrows, capacity: world.capacity,
            ticks: c.ticks, staleDropped: c.staleDropped, malformedDropped: c.malformedDropped,
            deferredTotal: world.totalDeferred, backpressureDrops: world.backpressureDrops,
            histories: final.wrong.map((w) => `${w} => ${(s.hist.get(Number(w.split(':')[0])) ?? []).slice(-8).join(' ')}`),
            pendingAtEnd,
            expectedLive: expected.size, actualLive: final.live, finalTorn: final.torn, mismatched: final.wrong,
        };
        (window as any).__result = result;
        document.getElementById('out')!.textContent = JSON.stringify(result, null, 1);
    },
    (e) => { (window as any).__result = { error: String(e) }; document.getElementById('out')!.textContent = String(e); },
);
