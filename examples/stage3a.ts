import {
    AmbientLight, BoxGeometry, BufferAttribute, BufferGeometry, DirectionalLight, Group, InstancedMesh, Matrix4, Mesh,
    MeshStandardMaterial, PerspectiveCamera, Scene, SphereGeometry, WebGLRenderer,
} from 'three';
import { World, layoutFromRuntime, TransformView } from '@dynova/sdk';
import { collidersFromScene, spawnColliders, type ColliderDesc } from '@dynova/three';
import { threeAdapter } from '@dynova/three';

const CRATES = 12;
const BALLS = 8;
const CONES = 6;
const PROPS = CRATES + BALLS + CONES;

function pyramidGeometry(sides: number, radius: number, height: number): BufferGeometry {
    const pos: number[] = [0, 0, 0, 0, height, 0];
    for (let k = 0; k < sides; k++) {
        const t = (k / sides) * Math.PI * 2;
        pos.push(radius * Math.cos(t), 0, radius * Math.sin(t));
    }
    const idx: number[] = [];
    for (let k = 0; k < sides; k++) {
        const a = 2 + k, b = 2 + ((k + 1) % sides);
        idx.push(0, b, a, 1, a, b);
    }
    const g = new BufferGeometry();
    g.setAttribute('position', new BufferAttribute(new Float32Array(pos), 3));
    g.setIndex(idx);
    g.computeVertexNormals();
    return g;
}

function prismGeometry(sides: number, radius: number, height: number): BufferGeometry {
    const g = pyramidGeometry(sides, radius, height);
    const pos = g.getAttribute('position') as BufferAttribute;
    for (let i = 0; i < pos.count; i++) if (pos.getY(i) === height) pos.setXYZ(i, 0, height, 0);
    return g;
}

const material = (color: number) => new MeshStandardMaterial({ color, flatShading: true });

function buildLevel(): Group {
    const level = new Group();
    const floor = new Mesh(new BoxGeometry(80, 1, 80), material(0x445544));
    floor.name = 'floor';
    floor.position.y = -0.5;

    const rampGeo = new BufferGeometry();
    const rampPts: number[] = [];
    for (let i = 0; i < 6; i++) for (let j = 0; j < 6; j++) rampPts.push(i, 0.5 * i, j);
    rampGeo.setAttribute('position', new BufferAttribute(new Float32Array(rampPts), 3));
    const tri: number[] = [];
    for (let i = 0; i < 5; i++) for (let j = 0; j < 5; j++) {
        const a = i * 6 + j, b = a + 1, c = a + 6, d = c + 1;
        tri.push(a, b, c, b, d, c);
    }
    rampGeo.setIndex(tri);
    rampGeo.computeVertexNormals();
    const ramp = new Mesh(rampGeo, material(0x886644));
    ramp.name = 'ramp';
    ramp.position.set(-9, 0, -3);

    const pillar = new Mesh(pyramidGeometry(7, 1.2, 3), material(0x555577));
    pillar.name = 'pillar';
    pillar.position.set(6, 0, 4);

    level.add(floor, ramp, pillar);
    return level;
}

function buildProps(): Group {
    const props = new Group();
    for (let i = 0; i < CRATES; i++) {
        const size = 0.6 + (i % 3) * 0.2;
        const m = new Mesh(new BoxGeometry(1, 1, 1), material(0xffaa44));
        m.name = `crate${i}`;
        m.scale.setScalar(size);
        m.position.set(-3 + (i % 4) * 1.6, 3 + Math.floor(i / 4) * 1.4, -2 + (i % 3));
        m.rotation.set(0.2 * i, 0.5 * i, 0);
        props.add(m);
    }
    for (let i = 0; i < BALLS; i++) {
        const m = new Mesh(new SphereGeometry(0.5, 16, 12), material(0x44aaff));
        m.name = `ball${i}`;
        m.scale.setScalar(0.5 + (i % 3) * 0.2);
        m.position.set(-8 + (i % 4) * 0.6, 6 + Math.floor(i / 4) * 1.2, -2.5 + (i % 2) * 2);
        props.add(m);
    }
    for (let i = 0; i < CONES; i++) {
        const m = new Mesh(pyramidGeometry(8, 0.7, 1.6), material(0xdd5588));
        m.name = `cone${i}`;
        m.position.set(2 + (i % 3) * 1.8, 5 + Math.floor(i / 3) * 1.6, -3 + (i % 2) * 5);
        m.rotation.set(0.4 * i, 0, 0.3 * i);
        props.add(m);
    }
    return props;
}

async function run() {
    const live = new URLSearchParams(location.search).has('live');
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

    const statics = spawnColliders(world, levelScene.colliders);
    const dynamics = spawnColliders(world, propScene.colliders, { body: () => ({ invMass: 1, restitution: 0.1 }) });

    const scales = new Float32Array(config.capacity * 3).fill(1);
    const byName = new Map<string, ColliderDesc>(propScene.colliders.map((c) => [c.name, c]));
    for (const s of dynamics.spawned) {
        const shape = byName.get(s.name)!.shape;
        const dims = shape.kind === 'box' ? shape.half.map((h) => h * 2) : shape.kind === 'sphere' ? [shape.radius * 2, shape.radius * 2, shape.radius * 2] : [1, 1, 1];
        scales.set(dims, s.handle.id * 3);
    }

    const first = (prefix: string) => dynamics.spawned.find((s) => s.name.startsWith(prefix))!.handle.id;
    const cone = byName.get('cone0')!.shape;
    const coneGeometry = pyramidGeometry(8, 0.7, 1.6);
    if (cone.kind === 'hull') coneGeometry.translate(-cone.center[0], -cone.center[1], -cone.center[2]);

    const crates = new InstancedMesh(new BoxGeometry(1, 1, 1), material(0xffaa44), CRATES);
    const balls = new InstancedMesh(new SphereGeometry(0.5, 16, 12), material(0x44aaff), BALLS);
    const cones = new InstancedMesh(coneGeometry, material(0xdd5588), CONES);
    const adapters = [
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
    try { renderer = new WebGLRenderer({ canvas: document.getElementById('c') as HTMLCanvasElement }); } catch { renderer = null; }
    const frame = () => {
        world.step();
        for (const a of adapters) a.update();
        renderer?.render(scene, camera);
    };

    const ids = dynamics.spawned.map((s) => s.handle.id);
    const lowest = new Map<number, number>(ids.map((id) => [id, Infinity]));
    const radiusOf = new Map<number, number>();
    for (const s of dynamics.spawned) {
        const shape = byName.get(s.name)!.shape;
        if (shape.kind === 'sphere') radiusOf.set(s.handle.id, shape.radius);
    }
    // Ramp plane: y = 0.5 * (x - rampX) for x in [rampX, rampX + 5], z in [-3, 2]; unit normal (-0.5, 1, 0) / sqrt(1.25).
    let rampSink = 0;
    let overRamp = 0;
    const totalTicks = 1200;
    for (let t = 0; t < totalTicks; t++) {
        frame();
        const buf = view.read();
        for (const id of ids) lowest.set(id, Math.min(lowest.get(id)!, buf[id * 7 + 1]));
        for (const [id, r] of radiusOf) {
            const x = buf[id * 7], y = buf[id * 7 + 1], z = buf[id * 7 + 2];
            if (x < -9 || x > -4 || z < -3 || z > 2) continue;
            overRamp++;
            rampSink = Math.max(rampSink, r - (y - 0.5 * (x + 9)) / Math.sqrt(1.25));
        }
    }

    const buf = view.read();
    const at = (name: string) => Array.from(buf.slice(dynamics.spawned.find((s) => s.name === name)!.handle.id * 7, dynamics.spawned.find((s) => s.name === name)!.handle.id * 7 + 7));
    const m = new Matrix4();
    let worst = 0;
    for (const [mesh, prefix, count] of [[crates, 'crate', CRATES], [balls, 'ball', BALLS], [cones, 'cone', CONES]] as const) {
        const base = first(prefix);
        for (let i = 0; i < count; i++) {
            mesh.getMatrixAt(i, m);
            for (let k = 0; k < 3; k++) worst = Math.max(worst, Math.abs(m.elements[12 + k] - buf[(base + i) * 7 + k]));
        }
    }

    if (live) {
        const loop = () => { frame(); requestAnimationFrame(loop); };
        requestAnimationFrame(loop);
    }

    return {
        rendered: renderer !== null,
        staticKinds: levelScene.colliders.map((c) => `${c.name}:${c.shape.kind}`),
        propKinds: Array.from(new Set(propScene.colliders.map((c) => c.shape.kind))).sort(),
        propCount: dynamics.spawned.length,
        skipped: [...levelScene.skipped, ...propScene.skipped],
        failed: [...statics.failed, ...dynamics.failed],
        malformed: world.runtimeCounters.malformedDropped,
        adapterMismatch: worst,
        minHeight: Math.min(...ids.map((id) => lowest.get(id)!)),
        finite: Array.from(buf).every(Number.isFinite),
        crate0: at('crate0'),
        ball0: at('ball0'),
        cone0: at('cone0'),
        sleeping: rt.sleeping_count(),
        contacts: world.runtimeStats.contacts,
        props: PROPS,
        rampSink,
        overRamp,
    };
}

run().then(
    (r) => { (window as any).__result = r; document.getElementById('out')!.textContent = JSON.stringify({ ...r, crate0: undefined, ball0: undefined, cone0: undefined }, null, 1); },
    (e) => { (window as any).__result = { error: String(e) }; document.getElementById('out')!.textContent = String(e); },
);
