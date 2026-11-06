import { AmbientLight, Box3, BoxGeometry, DirectionalLight, InstancedMesh, Mesh, MeshStandardMaterial, PerspectiveCamera, Scene, SphereGeometry, Vector3, WebGLRenderer, type Object3D, type Texture } from 'three';
import { World, layoutFromRuntime, TransformView } from '@dynova/sdk';
import { collidersFromScene, spawnColliders, type ColliderDesc, type SceneColliders } from '@dynova/three';
import { loadModelFiles, loadModelUrl, type LoadOptions, type ModelFile } from '@dynova/three/loaders';
import { threeAdapter } from '@dynova/three';
import { box, buildGlb, cube, pyramid, rampGrid, translated, type FixtureMesh } from '../renderer/three/src/testing/gltfFixture';
import { BALLS, CONES, CRATES, buildProps, pyramidGeometry } from './stageScene';

const params = new URLSearchParams(location.search);
const paused = params.has('paused');
const canvas = document.getElementById('c') as HTMLCanvasElement;
const out = document.getElementById('out')!;
const msg = document.getElementById('msg')!;
const loadOptions: LoadOptions = { dracoDecoderPath: '/draco/' };

function defaultLevel(): ArrayBuffer {
    const wall = (name: string, half: number[], at: number[]): FixtureMesh => ({ name, ...box(half[0], half[1], half[2]), translation: at, color: [0.45, 0.4, 0.35] });
    return buildGlb([
        { name: 'floor', ...box(6, 0.25, 6), translation: [0, -0.25, 0], color: [0.2, 0.45, 0.25] },
        wall('wall-n', [6.5, 1, 0.25], [0, 1, -6.25]),
        wall('wall-s', [6.5, 1, 0.25], [0, 1, 6.25]),
        wall('wall-w', [0.25, 1, 6], [-6.25, 1, 0]),
        wall('wall-e', [0.25, 1, 6], [6.25, 1, 0]),
        { name: 'ramp', ...rampGrid(5, 0.5), translation: [-5, 0, -2], color: [0.6, 0.4, 0.25] },
        { name: 'pillar', ...translated(pyramid(7, 1.2, 3), 0, 0, 0), translation: [3, 0, 3], color: [0.35, 0.35, 0.6] },
        { name: 'crate-static', ...cube(0.6), translation: [-1, 0.6, 4], color: [0.8, 0.6, 0.3] },
    ]);
}

const renderer = (() => { try { return new WebGLRenderer({ canvas }); } catch { return null; } })();
const scene = new Scene();
scene.add(new AmbientLight(0xffffff, 0.6));
const sun = new DirectionalLight(0xffffff, 1.3);
sun.position.set(6, 14, 8);
scene.add(sun);
const camera = new PerspectiveCamera(50, canvas.width / canvas.height, 0.05, 2000);

const glue = await import('../sdk/typescript/wasm-st/dynova_core.js');
const exports = await glue.default() as unknown as { memory: WebAssembly.Memory };

interface Running {
    runtime: InstanceType<typeof glue.Runtime>;
    step(): void;
    stats(): Record<string, unknown>;
    dispose(): void;
}
let running: Running | null = null;
let generation = 0;
let request = 0;

function start(root: Object3D, label: string, levelScene: SceneColliders): void {
    running?.dispose();
    running = null;
    const token = ++generation;
    const bounds = new Box3().setFromObject(root);
    const size = bounds.getSize(new Vector3());
    const centre = bounds.getCenter(new Vector3());

    const props = buildProps();
    const s = Math.min(3, Math.max(0.15, (Math.min(size.x, size.z) * 0.4) / 7));
    props.scale.setScalar(s);
    props.position.set(centre.x + s, bounds.max.y + 0.5, centre.z);
    props.updateMatrixWorld(true);
    const propScene = collidersFromScene(props);

    const config = { capacity: levelScene.colliders.length + propScene.colliders.length + 4, ringCapacity: 512 };
    const runtime = new glue.Runtime(config.capacity, config.ringCapacity);
    runtime.enable_physics();
    const world = World.attachSync(config, exports.memory, 'fallback', layoutFromRuntime(runtime), runtime);
    const view = new TransformView(exports.memory, runtime.header_ptr());
    const statics = spawnColliders(world, levelScene.colliders);
    const dynamics = spawnColliders(world, propScene.colliders, { body: () => ({ invMass: 1, restitution: 0.1 }) });

    const byName = new Map<string, ColliderDesc>(propScene.colliders.map((c) => [c.name, c]));
    const scales = new Float32Array(config.capacity * 3).fill(1);
    for (const sp of dynamics.spawned) {
        const shape = byName.get(sp.name)!.shape;
        scales.set(shape.kind === 'box' ? shape.half.map((h) => h * 2) : shape.kind === 'sphere' ? [shape.radius * 2, shape.radius * 2, shape.radius * 2] : [s, s, s], sp.handle.id * 3);
    }
    const first = (prefix: string) => dynamics.spawned.find((x) => x.name.startsWith(prefix))!.handle.id;
    const cone = byName.get('cone0')!.shape;
    const coneGeometry = pyramidGeometry(8, 0.7, 1.6);
    if (cone.kind === 'hull') coneGeometry.translate(-cone.center[0] / s, -cone.center[1] / s, -cone.center[2] / s);
    const mat = (c: number) => new MeshStandardMaterial({ color: c, flatShading: true });
    const meshes = [
        { mesh: new InstancedMesh(new BoxGeometry(1, 1, 1), mat(0xffaa44), CRATES), prefix: 'crate' },
        { mesh: new InstancedMesh(new SphereGeometry(0.5, 16, 12), mat(0x44aaff), BALLS), prefix: 'ball' },
        { mesh: new InstancedMesh(coneGeometry, mat(0xdd5588), CONES), prefix: 'cone' },
    ];
    const adapters = meshes.map(({ mesh, prefix }) => threeAdapter(world, mesh, { idOffset: first(prefix), scales }));

    scene.add(root, ...meshes.map((m) => m.mesh));
    const reach = Math.max(size.x, size.y, size.z, 1);
    camera.position.set(centre.x, bounds.max.y + reach * 0.9, centre.z + reach * 1.25);
    camera.lookAt(centre.x, bounds.min.y + reach * 0.1, centre.z);

    let ticks = 0;
    const ids = dynamics.spawned.map((x) => x.handle.id);
    const step = () => {
        world.step();
        ticks++;
        for (const a of adapters) a.update();
    };
    const hud = () => { out.textContent = `${label}  tick ${ticks}  asleep ${runtime.sleeping_count()}/${ids.length}  static ${statics.spawned.length} colliders  contacts ${world.runtimeStats.contacts}`; };
    const loop = () => {
        if (token !== generation) return;
        step();
        hud();
        renderer?.render(scene, camera);
        requestAnimationFrame(loop);
    };
    if (!paused) requestAnimationFrame(loop); else hud();

    running = {
        runtime,
        step,
        stats() {
            const buf = view.read();
            const ys = ids.map((id) => buf[id * 7 + 1]);
            const inside = ids.filter((id) => buf[id * 7] >= bounds.min.x && buf[id * 7] <= bounds.max.x && buf[id * 7 + 2] >= bounds.min.z && buf[id * 7 + 2] <= bounds.max.z).length;
            return {
                label,
                ticks,
                levelKinds: levelScene.colliders.map((c) => `${c.name}:${c.shape.kind}`),
                skipped: [...levelScene.skipped, ...propScene.skipped],
                failed: [...statics.failed, ...dynamics.failed],
                propCount: ids.length,
                malformed: world.runtimeCounters.malformedDropped,
                finite: Array.from(buf).every(Number.isFinite),
                minPropY: Math.min(...ys),
                floorY: bounds.min.y,
                propsInside: inside,
                propScale: s,
                coneInstanceScale: (() => {
                    const e = meshes[2].mesh.instanceMatrix.array;
                    return Math.hypot(e[0], e[1], e[2]);
                })(),
                sleeping: runtime.sleeping_count(),
            };
        },
        dispose() {
            scene.remove(root, ...meshes.map((m) => m.mesh));
            for (const { mesh } of meshes) { mesh.geometry.dispose(); (mesh.material as MeshStandardMaterial).dispose(); }
            disposeObject(root);
            world.dispose();
            runtime.free();
        },
    };
}

function disposeObject(root: Object3D): void {
    root.traverse((n) => {
        const m = n as Mesh;
        if (!m.isMesh) return;
        m.geometry.dispose();
        for (const material of Array.isArray(m.material) ? m.material : [m.material]) {
            for (const value of Object.values(material)) if (value && (value as Texture).isTexture) (value as Texture).dispose();
            material.dispose();
        }
    });
}

function fail(text: string): void {
    msg.textContent = text;
    (window as any).__error = text;
    (window as any).__errorCount = ((window as any).__errorCount ?? 0) + 1;
}

async function load(source: () => ReturnType<typeof loadModelUrl>, label: string, token = ++request): Promise<void> {
    const mine = token;
    const result = await source();
    if (mine !== request) return;
    if (!result.ok) return fail(`could not load ${label}: ${result.error}`);
    const levelScene = collidersFromScene(result.scene);
    if (levelScene.colliders.length === 0) {
        disposeObject(result.scene);
        return fail(`could not use ${label}: no meshes to collide with (${levelScene.skipped.length} skipped)`);
    }
    try {
        start(result.scene, label, levelScene);
    } catch (err) {
        running = null;
        disposeObject(result.scene);
        return fail(`could not start ${label}: ${err instanceof Error ? err.message : String(err)}`);
    }
    msg.textContent = `${label} (${result.format})`;
    (window as any).__error = undefined;
}

async function loadFiles(files: File[]): Promise<void> {
    if (files.length === 0) return;
    const token = ++request;
    const models: ModelFile[] = await Promise.all(files.map(async (f) => ({ name: f.name, data: await f.arrayBuffer() })));
    const label = files.length === 1 ? files[0].name : `${files.length} files`;
    await load(() => loadModelFiles(models, loadOptions), label, token);
}

document.getElementById('file')!.addEventListener('change', (e) => {
    const input = e.target as HTMLInputElement;
    const files = Array.from(input.files ?? []);
    input.value = '';
    void loadFiles(files);
});
addEventListener('dragover', (e) => { e.preventDefault(); document.body.classList.add('drag'); });
addEventListener('dragleave', (e) => { if (!e.relatedTarget) document.body.classList.remove('drag'); });
addEventListener('drop', (e) => {
    e.preventDefault();
    document.body.classList.remove('drag');
    void loadFiles(Array.from(e.dataTransfer?.files ?? []));
});

(window as any).__step = (n: number) => { for (let i = 0; i < n; i++) running?.step(); };
(window as any).__stats = () => running?.stats();

const url = params.get('url');
try {
    await (url
        ? load(() => loadModelUrl(url, loadOptions), url)
        : load(() => loadModelFiles([{ name: 'built-in.glb', data: defaultLevel() }], loadOptions), 'built-in level'));
} finally {
    (window as any).__ready = true;
}
