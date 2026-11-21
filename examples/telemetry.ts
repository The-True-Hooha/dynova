import { InspectorView, World, layoutFromRuntime } from '@dynova/sdk';
import type { EntityHandle } from '@dynova/sdk';
import { canvas2dAdapter } from '@dynova/canvas2d';
import { mountInspector, type InspectorControls } from '@dynova/inspector';

const params = new URLSearchParams(location.search);
const MAX_N = 10000;
const parsed = Math.floor(Number(params.get('n') ?? 2000));
const N = Number.isFinite(parsed) ? Math.min(Math.max(parsed, 0), MAX_N) : 2000;
const paused = params.has('paused');
const withPanel = params.get('panel') !== '0';

const PPU = 12;
const EVERY = 12;
const FRAMES = 120;
const COLUMNS = 50;
const SPACING = 1.3;
const VELOCITY_SCALE = 0.35;

function lcg(seed: number): () => number {
    let s = seed >>> 0;
    return () => {
        s = (Math.imul(s, 1664525) + 1013904223) >>> 0;
        return s / 4294967296;
    };
}

/** The stand-in for a recorded feed: per entity and frame, an offset from home, generated once and then only read. */
function recordFeed(n: number): Float32Array {
    const rand = lcg(7);
    const feed = new Float32Array(n * FRAMES * 2);
    for (let i = 0; i < n; i++) {
        const radius = 0.3 + rand() * 1.4, phase = rand() * 6.28, speed = 0.1 + rand() * 0.5;
        for (let k = 0; k < FRAMES; k++) {
            feed[(k * n + i) * 2] = Math.cos(phase + k * speed) * radius;
            feed[(k * n + i) * 2 + 1] = Math.sin(phase + k * speed * 0.8) * radius;
        }
    }
    return feed;
}

async function run() {
    const glue = await import('../sdk/typescript/wasm-st/dynova_core.js');
    const exports = await glue.default() as unknown as { memory: WebAssembly.Memory };
    const config = { capacity: N + 8, ringCapacity: 4096 };
    const rt = new glue.Runtime(config.capacity, config.ringCapacity);
    rt.enable_physics();
    const world = World.attachSync(config, exports.memory, 'fallback', layoutFromRuntime(rt), rt);
    world.setGravity(0, 0, 0);
    const scales = new Float32Array(config.capacity * 3).fill(0.6);

    const rows = Math.ceil(N / COLUMNS);
    const home = (i: number): [number, number] => [((i % COLUMNS) - (COLUMNS - 1) / 2) * SPACING, (Math.floor(i / COLUMNS) - (rows - 1) / 2) * SPACING];
    const handles: EntityHandle[] = [];
    for (let i = 0; i < N; i++) {
        const [x, y] = home(i);
        handles.push(world.spawn({ x, y, z: 0, hx: 0.3, hy: 0.3, hz: 0.3, invMass: 0 }));
        scales.set([0.6, 0.6, 0.6], handles[i].id * 3);
    }
    const feed = recordFeed(N);

    const canvas = document.getElementById('view') as HTMLCanvasElement;
    const ctx = canvas.getContext('2d')!;
    const view = { pixelsPerUnit: PPU, originX: canvas.width / 2, originY: canvas.height / 2 };
    const adapter = canvas2dAdapter(world, ctx, { scales, view, style: () => ({ shape: 'circle', color: '#44aaff' }) });
    const inspectorView = new InspectorView(exports.memory, rt.header_ptr());
    const controls: InspectorControls = {
        recording: () => world.recording,
        record: (on) => world.record(on),
        pause: (on) => world.pause(on),
        seek: (tick) => world.seek(tick),
        teleport: () => false,
    };
    if (withPanel) {
        mountInspector(document.getElementById('panel')!, { view: inspectorView, controls, setEnabled: (on) => world.setInspector(on) });
    } else {
        world.setInspector(true);
    }
    world.record(true);

    let overlayLines = 0;
    const drawOverlay = () => {
        const snap = inspectorView.snapshot();
        overlayLines = 0;
        if (!snap) return;
        ctx.strokeStyle = '#7fe0a0';
        ctx.lineWidth = 1;
        ctx.beginPath();
        for (const h of handles) {
            const b = snap.body(h.id);
            if (!b || (b.velocity[0] === 0 && b.velocity[1] === 0)) continue;
            const x = view.originX + b.position[0] * PPU, y = view.originY - b.position[1] * PPU;
            ctx.moveTo(x, y);
            ctx.lineTo(x + b.velocity[0] * PPU * VELOCITY_SCALE, y - b.velocity[1] * PPU * VELOCITY_SCALE);
            overlayLines++;
        }
        ctx.stroke();
    };
    const draw = () => {
        ctx.clearRect(0, 0, canvas.width, canvas.height);
        adapter.update();
        drawOverlay();
    };

    let frame = 0;
    let feeds = 0;
    let lastFedTick = -1;
    // The feed is a function of the tick alone, so a seek followed by a live step feeds the same samples. A paused world keeps its
    // tick, so the tick it last fed is remembered instead of queueing the same batch on every animation frame.
    const feedTick = () => {
        const tick = world.runtimeCounters.ticks;
        if (tick % EVERY !== 0 || tick === lastFedTick) return;
        lastFedTick = tick;
        feeds++;
        const k = ((tick / EVERY) | 0) % FRAMES;
        frame = k;
        world.moveKinematic(handles.map((handle, i) => {
            const [hx, hy] = home(i);
            return { handle, x: hx + feed[(k * N + i) * 2], y: hy + feed[(k * N + i) * 2 + 1], z: 0, ticks: EVERY };
        }));
    };
    const step = (n = 1) => {
        for (let i = 0; i < n; i++) {
            feedTick();
            world.step();
        }
        draw();
    };
    // A paused page still redraws, so the panel's scrub, pause and seek show on the canvas.
    const loop = () => {
        requestAnimationFrame(loop);
        if (paused) draw(); else step();
    };
    draw();
    requestAnimationFrame(loop);

    (window as any).__telemetry = {
        step,
        world,
        view: inspectorView,
        feedFrame: () => frame,
        feeds: () => feeds,
        home,
        feedOffset: (k: number, i: number): [number, number] => [feed[(k * N + i) * 2], feed[(k * N + i) * 2 + 1]],
        overlayLines: () => overlayLines,
        entities: N,
        draw,
        stats: () => ({ ticks: world.runtimeCounters.ticks, malformed: world.runtimeCounters.malformedDropped, stale: world.runtimeCounters.staleDropped }),
    };
    (window as any).__ready = true;
}

run().catch((e) => {
    (window as any).__error = String(e);
    document.body.append(String(e));
});
