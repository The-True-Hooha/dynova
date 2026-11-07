import { InspectorView, World, layoutFromRuntime, SHAPE_SPHERE } from '@dynova/sdk';
import { canvas2dAdapter } from '@dynova/canvas2d';
import { mountInspector, type InspectorPanel } from '@dynova/inspector';

const params = new URLSearchParams(location.search);
const N = Number(params.get('n') ?? 300);
const paused = params.has('paused');
const withPanel = params.get('panel') !== '0';

const PPU = 16;
const ARENA_HALF = 12;
const COLUMNS = 20;

async function run() {
    const glue = await import('../sdk/typescript/wasm-st/dynova_core.js');
    const exports = await glue.default() as unknown as { memory: WebAssembly.Memory };
    const config = { capacity: N + 8, ringCapacity: 512 };
    const rt = new glue.Runtime(config.capacity, config.ringCapacity);
    rt.enable_physics();
    const world = World.attachSync(config, exports.memory, 'fallback', layoutFromRuntime(rt), rt);
    const scales = new Float32Array(config.capacity * 3).fill(1);

    const statics: [number, number, number, number, number][] = [
        [0, -0.5, ARENA_HALF + 2, 0.5, 2],
        [-ARENA_HALF - 0.5, 8, 0.5, 9, 2],
        [ARENA_HALF + 0.5, 8, 0.5, 9, 2],
        [0, 8, ARENA_HALF + 2, 9, 0.5],
    ];
    for (const [x, y, hx, hy, hz] of statics) {
        const h = world.spawn({ x, y, hx, hy, hz, invMass: 0, restitution: 0 });
        scales.set([hx * 2, hy * 2, hz * 2], h.id * 3);
    }
    const sphere = new Set<number>();
    for (let i = 0; i < N; i++) {
        const isSphere = i % 2 === 0;
        const h = world.spawn({
            x: ((i % COLUMNS) - (COLUMNS - 1) / 2) * 1.1 + ((i * 13) % 7) * 0.02,
            y: 2 + Math.floor(i / COLUMNS) * 1.1,
            z: ((i * 7) % 3 - 1) * 0.3,
            shape: isSphere ? SHAPE_SPHERE : undefined,
            hx: 0.5, hy: 0.5, hz: 0.5, invMass: 1, restitution: 0.1,
        });
        if (isSphere) sphere.add(h.id);
    }

    const canvas = document.getElementById('view') as HTMLCanvasElement;
    const ctx = canvas.getContext('2d')!;
    const adapter = canvas2dAdapter(world, ctx, {
        scales,
        view: { pixelsPerUnit: PPU, originX: canvas.width / 2, originY: canvas.height - 24 },
        style: (id) => ({ shape: sphere.has(id) ? 'circle' : 'rect', color: sphere.has(id) ? '#44aaff' : id < 4 ? '#555d6e' : '#ffaa44' }),
    });

    let panel: InspectorPanel | null = null;
    const inspectorView = new InspectorView(exports.memory, rt.header_ptr());
    if (withPanel) {
        panel = mountInspector(document.getElementById('panel')!, {
            view: inspectorView,
            setEnabled: (on) => world.setInspector(on),
        });
    }

    const draw = () => {
        ctx.clearRect(0, 0, canvas.width, canvas.height);
        adapter.update();
    };
    const step = (n = 1) => {
        for (let i = 0; i < n; i++) world.step();
        draw();
    };
    const loop = () => {
        step();
        requestAnimationFrame(loop);
    };
    draw();
    if (!paused) requestAnimationFrame(loop);

    (window as any).__inspector = {
        step,
        panel,
        world,
        view: inspectorView,
        mountInspector,
        stats: () => ({ ticks: world.runtimeCounters.ticks, sleeping: rt.sleeping_count(), contacts: world.runtimeStats.contacts, ring: world.pendingCommands }),
    };
    (window as any).__ready = true;
}

run().catch((e) => {
    (window as any).__error = String(e);
    document.body.append(String(e));
});
