import { BoxGeometry, InstancedMesh, MeshBasicMaterial } from 'three';
import { InspectorView, SharedMemoryUnavailableError, SHAPE_SPHERE, positionControl, type World } from '@dynova/sdk';
import { launch } from '@dynova/sdk/web';
import { canvas2dAdapter } from '@dynova/canvas2d';
import { threeAdapter, importAssembly, collidersFromScene } from '@dynova/three';
import { loadModelFiles } from '@dynova/three/loaders';
import { mountInspector } from '@dynova/inspector';

const BOXES = 20;

async function run() {
    const world: World = await launch({ capacity: BOXES + 8, ringCapacity: 256 }, { physics: true });
    const scales = new Float32Array((BOXES + 8) * 3).fill(1);
    const floor = world.spawn({ y: -0.5, hx: 10, hy: 0.5, hz: 10, invMass: 0 });
    scales.set([20, 1, 20], floor.id * 3);
    for (let i = 0; i < BOXES; i++) world.spawn({ x: (i % 5) - 2, y: 1 + Math.floor(i / 5) * 1.1, hx: 0.5, hy: 0.5, hz: 0.5, invMass: 1 });
    const ball = world.spawn({ x: 0, y: 8, shape: SHAPE_SPHERE, hx: 0.4, invMass: 1 });
    const a = world.spawn({ x: 6, y: 1, invMass: 0 });
    const b = world.spawn({ x: 7, y: 1, invMass: 1 });
    const joint = world.createJoint({ kind: 1, a, b, anchor: [6.5, 1, 0], axis: [0, 0, 1], motor: { velocity: 0, maxForce: 50 } });

    const canvas = document.getElementById('view') as HTMLCanvasElement;
    const ctx = canvas.getContext('2d')!;
    const view = { pixelsPerUnit: 20, originX: 200, originY: 280 };
    const flat = canvas2dAdapter(world, ctx, { scales, view });
    const mesh = new InstancedMesh(new BoxGeometry(1, 1, 1), new MeshBasicMaterial(), BOXES + 8);
    const three = threeAdapter(world, mesh, { scales });
    const inspector = new InspectorView(world.memory, world.layout.headerByteOffset);
    const panel = mountInspector(document.getElementById('panel')!, { view: inspector, setEnabled: (on) => world.setInspector(on) });

    const draw = () => {
        ctx.clearRect(0, 0, canvas.width, canvas.height);
        flat.update();
        three.update();
    };
    const loop = () => {
        // A world on the calling thread (no shared memory) is stepped by its owner; a worker steps its own.
        if (world.mode === 'fallback') world.step();
        draw();
        panel.update();
        requestAnimationFrame(loop);
    };
    requestAnimationFrame(loop);

    (window as any).__app = {
        world, mesh, ball, floor, inspector, joint,
        motor: () => positionControl({ position: 0, velocity: 0 }, 1, { kp: 1, kd: 0, maxSpeed: 1, maxForce: 1 }),
        imports: { importAssembly: typeof importAssembly, collidersFromScene: typeof collidersFromScene, loadModelFiles: typeof loadModelFiles },
        pixels: () => { const d = ctx.getImageData(0, 0, canvas.width, canvas.height).data; let n = 0; for (let i = 3; i < d.length; i += 4) if (d[i] !== 0) n++; return n; },
        isolated: crossOriginIsolated,
        // True when the Canvas2D view painted something at the screen position of entity `id`.
        drawnAt: (id: number) => { const t = world.readTransforms(); const x = Math.round(view.originX + t[id * 7] * view.pixelsPerUnit), y = Math.round(view.originY - t[id * 7 + 1] * view.pixelsPerUnit); return ctx.getImageData(x, y, 1, 1).data[3] !== 0; },
    };
    (window as any).__ready = true;
}

run().catch((e) => {
    (window as any).__error = { name: e instanceof SharedMemoryUnavailableError ? 'SharedMemoryUnavailableError' : String(e?.name), message: String(e?.message ?? e) };
});
