import { World, layoutFromRuntime } from '@dynova/sdk';
import { canvas2dAdapter } from '@dynova/canvas2d';
import { importAssembly } from '@dynova/three';
import { GLTFLoader } from 'three/examples/jsm/loaders/GLTFLoader.js';
import { box, buildGlb } from '../renderer/three/src/testing/gltfFixture';

const params = new URLSearchParams(location.search);
const OFFSET = Number(params.get('offset') ?? 0);
const paused = params.has('paused');
if (!Number.isFinite(OFFSET)) throw new RangeError('offset must be a number');

const PPU = 120;
const GAINS = { kp: 4, kd: 0.2, maxSpeed: 2, maxForce: 200 };
const ROD_FROM: [number, number] = [1, 0];
const CARRIAGE: [number, number] = [3, 0.6];

/** Carriage position along the rail for a crank angle (the closed triangle the connecting rod makes). */
function carriageX(angle: number): number {
    const rod = Math.hypot(CARRIAGE[0] - ROD_FROM[0], CARRIAGE[1] - ROD_FROM[1]);
    const tip: [number, number] = [Math.cos(angle), Math.sin(angle)];
    return tip[0] + Math.sqrt(rod * rod - (CARRIAGE[1] - tip[1]) ** 2);
}

/** The file the demo reads. In a real project this is a .glb exported from a CAD or DCC tool with the extras set on the nodes. */
function authoredFile(): ArrayBuffer {
    return buildGlb([
        { name: 'base', ...box(0.1, 0.25, 0.1), translation: [-0.3, 0, 0], extras: { dynova: { static: true } } },
        { name: 'crank', ...box(0.44, 0.05, 0.1), translation: [0.5, 0, 0], extras: { dynova: { mate: { type: 'revolute', with: 'base', at: [-0.5, 0, 0], axis: [0, 0, 1], range: [-1.2, 1.2] } } } },
        {
            name: 'carriage', ...box(0.3, 0.2, 0.1), translation: [CARRIAGE[0], CARRIAGE[1], 0],
            extras: { dynova: { mate: [
                { type: 'slider', with: 'base', axis: [1, 0, 0], range: [-1.5, 0.5] },
                { type: 'distance', with: 'crank', from: [0.5, 0, 0], to: [0, 0, 0] },
            ] } },
        },
    ], { translation: [OFFSET, OFFSET, OFFSET] });
}

async function run() {
    const glue = await import('../sdk/typescript/wasm-st/dynova_core.js');
    const exports = await glue.default() as unknown as { memory: WebAssembly.Memory };
    const config = { capacity: 16, ringCapacity: 256 };
    const rt = new glue.Runtime(config.capacity, config.ringCapacity);
    rt.enable_physics();
    const world = World.attachSync(config, exports.memory, 'fallback', layoutFromRuntime(rt), rt);
    world.setGravity(0, 0, 0);
    world.record(true);

    const scene = await new Promise<import('three').Object3D>((resolve, reject) => new GLTFLoader().parse(authoredFile(), '', (g) => resolve(g.scene), reject));
    const imported = importAssembly(scene, world);
    const { assembly } = imported;

    const scales = new Float32Array(config.capacity * 3).fill(1);
    const colors: Record<string, string> = { base: '#555d6e', crank: '#44aaff', carriage: '#ffaa44' };
    const halves: Record<string, [number, number]> = { base: [0.1, 0.25], crank: [0.44, 0.05], carriage: [0.3, 0.2] };
    const styles = new Map<number, string>();
    for (const name of imported.parts) {
        const h = assembly.part(name);
        scales.set([halves[name][0] * 2, halves[name][1] * 2, 1], h.id * 3);
        styles.set(h.id, colors[name]);
    }

    const canvas = document.getElementById('view') as HTMLCanvasElement;
    const ctx = canvas.getContext('2d')!;
    const view = { pixelsPerUnit: PPU, originX: 120, originY: canvas.height / 2 };
    // The canvas draws local coordinates, so the view origin already sits at the model's centre; shift it back to the crank's pivot.
    const shift = assembly.toLocal([OFFSET, OFFSET, OFFSET]);
    view.originX += -shift[0] * PPU;
    view.originY += shift[1] * PPU;
    const adapter = canvas2dAdapter(world, ctx, { scales, view, style: (id) => ({ color: styles.get(id) ?? '#333b4a' }) });
    const status = document.getElementById('status')!;

    let target = 0;
    const step = (n = 1) => {
        for (let i = 0; i < n; i++) {
            world.step();
            assembly.drive({ crank: target }, GAINS);
        }
        ctx.clearRect(0, 0, canvas.width, canvas.height);
        adapter.update();
        const s = assembly.states();
        status.textContent = `${imported.parts.length} parts, ${imported.mates.length} mates, ${imported.errors.length} errors  crank ${s.crank?.position.toFixed(3) ?? '-'} rad`;
    };
    step(1);
    const loop = () => { step(); requestAnimationFrame(loop); };
    if (!paused) requestAnimationFrame(loop);

    (window as any).__imp = {
        world, assembly, step, imported: { parts: imported.parts, mates: imported.mates, errors: imported.errors, skipped: imported.skipped },
        setTarget: (t: number) => { target = t; },
        carriageX,
        carriageGlobalX: () => assembly.toGlobal([world.readTransforms()[assembly.part('carriage').id * 7], 0, 0])[0] - OFFSET,
    };
    (window as any).__ready = true;
}

run().catch((e) => {
    (window as any).__error = String(e);
    document.body.append(String(e));
});
