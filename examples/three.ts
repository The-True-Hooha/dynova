import { BoxGeometry, InstancedMesh, MeshNormalMaterial, Matrix4, PerspectiveCamera, Scene, WebGLRenderer } from 'three';
import { World, layoutFromRuntime } from '@dynova/sdk';
import { threeAdapter } from '@dynova/three';

const COLS = 5;
const ROWS = 10;
const BOXES = COLS * ROWS;

async function run() {
    const glue = await import('../sdk/typescript/wasm-st/dynova_core.js');
    const exports = await glue.default() as unknown as { memory: WebAssembly.Memory };
    const config = { capacity: BOXES + 1, ringCapacity: 128 };
    const rt = new glue.Runtime(config.capacity, config.ringCapacity);
    rt.enable_physics();
    const world = World.attachSync(config, exports.memory, 'fallback', layoutFromRuntime(rt), rt);

    world.spawn({ y: -0.5, hx: 30, hy: 0.5, hz: 30, invMass: 0, restitution: 0 });
    for (let r = 0; r < ROWS; r++) {
        for (let c = 0; c < COLS; c++) {
            world.spawn({ x: (c - (COLS - 1) / 2) * 1.02 + (r % 2 ? 0.5 : 0), y: 0.5 + r * 1.02, hx: 0.5, hy: 0.5, hz: 0.5, invMass: 1, restitution: 0 });
        }
    }

    const mesh = new InstancedMesh(new BoxGeometry(1, 1, 1), new MeshNormalMaterial(), BOXES + 1);
    const scales = new Float32Array((BOXES + 1) * 3).fill(1);
    scales.set([60, 1, 60], 0);
    const adapter = threeAdapter(world, mesh, { scales });
    const scene = new Scene();
    scene.add(mesh);
    const camera = new PerspectiveCamera(50, 320 / 240, 0.1, 100);
    camera.position.set(0, 6, 20);

    let rendered = false;
    try {
        const renderer = new WebGLRenderer({ canvas: document.getElementById('c') as HTMLCanvasElement });
        for (let i = 0; i < 400; i++) {
            world.step();
            adapter.update();
            renderer.render(scene, camera);
        }
        rendered = true;
    } catch {
        for (let i = 0; i < 400; i++) { world.step(); adapter.update(); }
    }

    const buf = world.readTransforms();
    const m = new Matrix4();
    let worst = 0;
    for (let i = 0; i <= BOXES; i++) {
        mesh.getMatrixAt(i, m);
        for (let k = 0; k < 3; k++) worst = Math.max(worst, Math.abs(m.elements[12 + k] - buf[i * 7 + k]));
    }
    return { rendered, worst, children: scene.children.length, meshChildren: mesh.children.length, floorWidth: (() => { mesh.getMatrixAt(0, m); return m.elements[0]; })(), boxWidth: (() => { mesh.getMatrixAt(1, m); return m.elements[0]; })(), topY: Math.max(...Array.from({ length: BOXES }, (_, i) => buf[(i + 1) * 7 + 1])) };
}

run().then(
    (r) => { (window as any).__result = r; document.getElementById('out')!.textContent = JSON.stringify(r); },
    (e) => { (window as any).__result = { error: String(e) }; document.getElementById('out')!.textContent = String(e); },
);
