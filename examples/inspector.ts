import { InspectorView, World, layoutFromRuntime, JOINT_BALL, JOINT_HINGE, SHAPE_SPHERE } from '@dynova/sdk';
import { canvas2dAdapter } from '@dynova/canvas2d';
import { mountInspector, type InspectorControls, type InspectorPanel } from '@dynova/inspector';

const params = new URLSearchParams(location.search);
const N = Number(params.get('n') ?? 300);
const paused = params.has('paused');
const withPanel = params.get('panel') !== '0';
const withControls = params.get('controls') !== '0';
const withJoints = params.has('joints');
const CAPACITY = Number(params.get('capacity') ?? 0);
const RING = Number(params.get('ring') ?? 512);

const PPU = 16;
const ARENA_HALF = 12;
const COLUMNS = 20;

async function run() {
    const glue = await import('../sdk/typescript/wasm-st/dynova_core.js');
    const exports = await glue.default() as unknown as { memory: WebAssembly.Memory };
    const config = { capacity: Math.max(N + 8, CAPACITY), ringCapacity: RING };
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
    const dynamic: { h: ReturnType<World['spawn']>; x: number; y: number; z: number }[] = [];
    for (let i = 0; i < N; i++) {
        const isSphere = i % 2 === 0;
        const x = ((i % COLUMNS) - (COLUMNS - 1) / 2) * 1.1 + ((i * 13) % 7) * 0.02;
        const y = 2 + Math.floor(i / COLUMNS) * 1.1;
        // The backdrop wall sits at z = 0; a jointed body on each side of it would be pushed apart by the contacts.
        const z = withJoints && i < 3 ? 1.5 : ((i * 7) % 3 - 1) * 0.3;
        const h = world.spawn({
            x, y, z,
            shape: isSphere ? SHAPE_SPHERE : undefined,
            hx: 0.5, hy: 0.5, hz: 0.5, invMass: 1, restitution: 0.1,
        });
        if (isSphere) sphere.add(h.id);
        dynamic.push({ h, x, y, z });
    }
    if (withJoints) {
        const mid = (a: number, b: number): [number, number, number] => [(dynamic[a].x + dynamic[b].x) / 2, (dynamic[a].y + dynamic[b].y) / 2, (dynamic[a].z + dynamic[b].z) / 2];
        world.createJoint({ kind: JOINT_BALL, a: dynamic[0].h, b: dynamic[1].h, anchor: mid(0, 1) });
        world.createJoint({ kind: JOINT_HINGE, a: dynamic[1].h, b: dynamic[2].h, anchor: mid(1, 2), axis: [0, 0, 1] });
    }

    const canvas = document.getElementById('view') as HTMLCanvasElement;
    const ctx = canvas.getContext('2d')!;
    const view = { pixelsPerUnit: PPU, originX: canvas.width / 2, originY: canvas.height - 24 };
    const adapter = canvas2dAdapter(world, ctx, {
        scales,
        view,
        style: (id) => ({ shape: sphere.has(id) ? 'circle' : 'rect', color: sphere.has(id) ? '#44aaff' : id < 4 ? '#555d6e' : '#ffaa44' }),
    });

    let panel: InspectorPanel | null = null;
    const inspectorView = new InspectorView(exports.memory, rt.header_ptr());
    const controls: InspectorControls = {
        recording: () => world.recording,
        record: (on) => world.record(on),
        pause: (on) => world.pause(on),
        seek: (tick) => world.seek(tick),
        teleport: (id, x, y, z) => {
            const handle = world.handleOf(id);
            if (!handle) return false;
            try {
                world.teleport(handle, x, y, z);
                return true;
            } catch {
                // Out of range, a stale handle or a full command ring: the move is refused, the page keeps working.
                return false;
            }
        },
    };
    if (withPanel) {
        panel = mountInspector(document.getElementById('panel')!, {
            view: inspectorView,
            controls: withControls ? controls : undefined,
            setEnabled: (on) => world.setInspector(on),
        });
    }

    const STATIC_IDS = 4;
    const canvasPoint = (e: PointerEvent): [number, number] => {
        const r = canvas.getBoundingClientRect();
        return [((e.clientX - r.left) * canvas.width) / r.width, ((e.clientY - r.top) * canvas.height) / r.height];
    };
    const toWorld = (cx: number, cy: number): [number, number] => [(cx - view.originX) / PPU, (view.originY - cy) / PPU];
    const pick = (wx: number, wy: number): number | null => {
        const t = world.readTransforms();
        for (let id = config.capacity - 1; id >= 0; id--) {
            const o = id * 7;
            if (t[o + 3] === 0 && t[o + 4] === 0 && t[o + 5] === 0 && t[o + 6] === 0) continue;
            const dx = wx - t[o];
            const dy = wy - t[o + 1];
            const hx = scales[id * 3] / 2;
            const hy = scales[id * 3 + 1] / 2;
            const hit = sphere.has(id) ? dx * dx + dy * dy <= hx * hx : Math.abs(dx) <= hx && Math.abs(dy) <= hy;
            if (hit) return id;
        }
        return null;
    };
    let drag: { id: number; dx: number; dy: number; z: number } | null = null;
    canvas.addEventListener('pointerdown', (e) => {
        const [wx, wy] = toWorld(...canvasPoint(e));
        const id = pick(wx, wy);
        if (id === null) return;
        panel?.select(id);
        if (id < STATIC_IDS) return;
        const t = world.readTransforms();
        drag = { id, dx: wx - t[id * 7], dy: wy - t[id * 7 + 1], z: t[id * 7 + 2] };
        canvas.setPointerCapture(e.pointerId);
    });
    canvas.addEventListener('pointermove', (e) => {
        if (!drag || !withControls) return;
        const [wx, wy] = toWorld(...canvasPoint(e));
        controls.teleport(drag.id, wx - drag.dx, wy - drag.dy, drag.z);
    });
    const endDrag = () => { drag = null; };
    canvas.addEventListener('pointerup', endDrag);
    canvas.addEventListener('pointercancel', endDrag);

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
        screenOf: (id: number) => {
            const t = world.readTransforms();
            const r = canvas.getBoundingClientRect();
            return {
                x: r.left + ((view.originX + t[id * 7] * PPU) * r.width) / canvas.width,
                y: r.top + ((view.originY - t[id * 7 + 1] * PPU) * r.height) / canvas.height,
            };
        },
        stats: () => ({ ticks: world.runtimeCounters.ticks, sleeping: rt.sleeping_count(), contacts: world.runtimeStats.contacts, ring: world.pendingCommands }),
    };
    (window as any).__ready = true;
}

run().catch((e) => {
    (window as any).__error = String(e);
    document.body.append(String(e));
});
