import { AmbientLight, BoxGeometry, DirectionalLight, InstancedMesh, Matrix4, PerspectiveCamera, Quaternion, Scene, SphereGeometry, Vector3, WebGLRenderer } from 'three';
import { World, layoutFromRuntime, TransformView } from '@dynova/sdk';
import { collidersFromScene, spawnColliders, type ColliderDesc } from '@dynova/three';
import { threeAdapter } from '@dynova/three';
import { canvas2dAdapter } from '@dynova/canvas2d';
import { BALLS, CONES, CRATES, buildLevel, buildProps, material, pyramidGeometry } from './stageScene';

const PPU = 14;
const ORIGIN_X = 320;
const ORIGIN_Y = 380;
const TICKS = 1200;
const SWAP_TO_FLAT = 400;
const SWAP_TO_GL = 800;

async function run() {
    const glue = await import('../sdk/typescript/wasm-st/dynova_core.js');
    const exports = await glue.default() as unknown as { memory: WebAssembly.Memory };

    const level = buildLevel();
    const props = buildProps();
    const levelScene = collidersFromScene(level);
    const propScene = collidersFromScene(props);
    const config = { capacity: levelScene.colliders.length + propScene.colliders.length + 4, ringCapacity: 256 };
    const rt = new glue.Runtime(config.capacity, config.ringCapacity);
    rt.enable_physics();
    const world = World.attachSync(config, exports.memory, 'fallback', layoutFromRuntime(rt), rt);
    const view = new TransformView(exports.memory, rt.header_ptr());

    spawnColliders(world, levelScene.colliders);
    const dynamics = spawnColliders(world, propScene.colliders, { body: () => ({ invMass: 1, restitution: 0.1 }) });

    const byName = new Map<string, ColliderDesc>(propScene.colliders.map((c) => [c.name, c]));
    const scales = new Float32Array(config.capacity * 3).fill(1);
    const kindOf = new Map<number, string>();
    for (const s of dynamics.spawned) {
        const shape = byName.get(s.name)!.shape;
        const dims = shape.kind === 'box' ? shape.half.map((h) => h * 2) : shape.kind === 'sphere' ? [shape.radius * 2, shape.radius * 2, shape.radius * 2] : [1, 1, 1];
        scales.set(dims, s.handle.id * 3);
        kindOf.set(s.handle.id, s.name);
    }
    const first = (prefix: string) => dynamics.spawned.find((s) => s.name.startsWith(prefix))!.handle.id;
    const firstProp = Math.min(...kindOf.keys());

    const cone = byName.get('cone0')!.shape;
    const coneGeometry = pyramidGeometry(8, 0.7, 1.6);
    if (cone.kind === 'hull') coneGeometry.translate(-cone.center[0], -cone.center[1], -cone.center[2]);
    const crates = new InstancedMesh(new BoxGeometry(1, 1, 1), material(0xffaa44), CRATES);
    const balls = new InstancedMesh(new SphereGeometry(0.5, 16, 12), material(0x44aaff), BALLS);
    const cones = new InstancedMesh(coneGeometry, material(0xdd5588), CONES);
    const glAdapters = [
        threeAdapter(world, crates, { idOffset: first('crate'), scales }),
        threeAdapter(world, balls, { idOffset: first('ball'), scales }),
        threeAdapter(world, cones, { idOffset: first('cone'), scales }),
    ];
    const scene = new Scene();
    scene.add(level, crates, balls, cones, new AmbientLight(0xffffff, 0.6));
    const sun = new DirectionalLight(0xffffff, 1.2);
    sun.position.set(5, 12, 6);
    scene.add(sun);
    const camera = new PerspectiveCamera(50, 640 / 400, 0.1, 100);
    camera.position.set(0, 9, 20);
    camera.lookAt(0, 1, 0);
    let renderer: WebGLRenderer | null = null;
    try { renderer = new WebGLRenderer({ canvas: document.getElementById('gl') as HTMLCanvasElement }); } catch { renderer = null; }

    const flat = document.getElementById('flat') as HTMLCanvasElement;
    const ctx = flat.getContext('2d')!;
    const colorOf = (id: number) => (kindOf.get(id)!.startsWith('crate') ? '#ffaa44' : kindOf.get(id)!.startsWith('ball') ? '#44aaff' : '#dd5588');
    const flatAdapter = canvas2dAdapter(world, ctx, {
        idOffset: firstProp,
        scales,
        view: { pixelsPerUnit: PPU, originX: ORIGIN_X, originY: ORIGIN_Y },
        style: (id) => ({ color: colorOf(id), shape: kindOf.get(id)!.startsWith('ball') ? 'circle' : 'rect' }),
    });

    const matrixMismatch = () => {
        const buf = view.read();
        const m = new Matrix4();
        const expected = new Matrix4();
        let worst = 0;
        for (const [mesh, prefix, count] of [[crates, 'crate', CRATES], [balls, 'ball', BALLS], [cones, 'cone', CONES]] as const) {
            for (let i = 0; i < count; i++) {
                const id = first(prefix) + i;
                mesh.getMatrixAt(i, m);
                const o = id * 7;
                expected.compose(
                    new Vector3(buf[o], buf[o + 1], buf[o + 2]),
                    new Quaternion(buf[o + 3], buf[o + 4], buf[o + 5], buf[o + 6]).normalize(),
                    new Vector3(scales[id * 3], scales[id * 3 + 1], scales[id * 3 + 2]),
                );
                for (let k = 0; k < 16; k++) worst = Math.max(worst, Math.abs(m.elements[k] - expected.elements[k]));
            }
        }
        return worst;
    };

    const live = { gl: 0, flat: 0 };
    let mismatchBefore = 0, mismatchAfter = 0, painted = 0, inBounds = 0, largestJump = 0;
    let previous: Float32Array | null = null;
    const sequence: string[] = [];

    for (let t = 0; t < TICKS; t++) {
        world.step();
        const useFlat = t >= SWAP_TO_FLAT && t < SWAP_TO_GL;
        if (sequence[sequence.length - 1] !== (useFlat ? 'canvas2d' : 'three')) sequence.push(useFlat ? 'canvas2d' : 'three');

        if (useFlat) {
            ctx.clearRect(0, 0, flat.width, flat.height);
            const res = flatAdapter.update();
            if (t === SWAP_TO_FLAT) live.flat = res.live;
        } else {
            let total = 0;
            for (const a of glAdapters) total += a.update().live;
            if (t === SWAP_TO_FLAT - 1) { live.gl = total; mismatchBefore = matrixMismatch(); }
            if (t === SWAP_TO_GL) mismatchAfter = matrixMismatch();
            renderer?.render(scene, camera);
        }

        const buf = view.read();
        if (previous) {
            for (const id of kindOf.keys()) {
                for (let k = 0; k < 3; k++) largestJump = Math.max(largestJump, Math.abs(buf[id * 7 + k] - previous[id * 7 + k]));
            }
        }
        previous = buf.slice();

        if (t === SWAP_TO_GL - 1) {
            for (const id of kindOf.keys()) {
                const sx = Math.round(ORIGIN_X + buf[id * 7] * PPU), sy = Math.round(ORIGIN_Y - buf[id * 7 + 1] * PPU);
                if (sx < 0 || sy < 0 || sx >= flat.width || sy >= flat.height) continue;
                inBounds++;
                if (ctx.getImageData(sx, sy, 1, 1).data[3] > 0) painted++;
            }
        }
    }

    return {
        sequence,
        rendered: renderer !== null,
        glLive: live.gl,
        flatLive: live.flat,
        props: kindOf.size,
        mismatchBefore,
        mismatchAfter,
        painted,
        inBounds,
        largestJump,
        ticks: world.runtimeCounters.ticks,
        malformed: world.runtimeCounters.malformedDropped,
        finite: Array.from(view.read()).every(Number.isFinite),
    };
}

run().then(
    (r) => { (window as any).__result = r; document.getElementById('out')!.textContent = JSON.stringify(r, null, 1); },
    (e) => { (window as any).__result = { error: String(e) }; document.getElementById('out')!.textContent = String(e); },
);
