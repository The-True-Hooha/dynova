import { BoxGeometry, BufferAttribute, BufferGeometry, Group, Mesh, MeshBasicMaterial, SphereGeometry, Vector3 } from 'three';
import { World, layoutFromRuntime, TransformView } from '@dynova/sdk';
import { collidersFromScene, spawnColliders } from '@dynova/three';

const HEIGHT = 4;

function pyramid(sides: number, radius: number, height: number): BufferGeometry {
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
    return g;
}

function mesh(name: string, geometry: BufferGeometry): Mesh {
    const m = new Mesh(geometry, new MeshBasicMaterial());
    m.name = name;
    return m;
}

async function boot(capacity: number) {
    const glue = await import('../sdk/typescript/wasm-st/dynova_core.js');
    const exports = await glue.default() as unknown as { memory: WebAssembly.Memory };
    const config = { capacity, ringCapacity: 128 };
    const rt = new glue.Runtime(config.capacity, config.ringCapacity);
    rt.enable_physics();
    const world = World.attachSync(config, exports.memory, 'fallback', layoutFromRuntime(rt), rt);
    return { rt, world, view: () => new TransformView(exports.memory, rt.header_ptr()).read() };
}

async function run() {
    const dynamic = (n: { name: string }) => (n.name === 'cone' || n.name === 'ball' ? { invMass: 1, restitution: 0 } : {});

    const a = await boot(4);
    const placed = new Group();
    const cone = mesh('cone', pyramid(8, 1, HEIGHT));
    cone.position.set(3, 5, -2);
    cone.rotation.set(0.4, 0.7, -0.3);
    placed.add(cone);
    const placedScene = collidersFromScene(placed);
    a.world.setGravity(0, 0, 0);
    const report = spawnColliders(a.world, placedScene.colliders, { body: dynamic });
    a.world.step();
    const got = Array.from(a.view().slice(report.spawned[0].handle.id * 7, report.spawned[0].handle.id * 7 + 3));
    const expected = new Vector3(0, HEIGHT / 4, 0).applyMatrix4(cone.matrixWorld);
    const malformedA = a.world.runtimeCounters.malformedDropped;

    const b = await boot(8);
    const level = new Group();
    const floor = mesh('floor', new BoxGeometry(20, 1, 20));
    floor.position.y = -0.5;
    const cone2 = mesh('cone', pyramid(8, 1, HEIGHT));
    cone2.position.set(0, 3, 0);
    const ball = mesh('ball', new SphereGeometry(0.5, 16, 12));
    ball.position.set(4, 4, 0);
    const rampGeo = new BufferGeometry();
    const rampPts: number[] = [];
    for (let i = 0; i < 6; i++) for (let j = 0; j < 6; j++) rampPts.push(i, 0.5 * i, j);
    rampGeo.setAttribute('position', new BufferAttribute(new Float32Array(rampPts), 3));
    const ramp = mesh('ramp', rampGeo);
    ramp.position.set(-8, 0, -3);
    level.add(floor, cone2, ball, ramp);
    const scene = collidersFromScene(level);
    const dropped = spawnColliders(b.world, scene.colliders, { body: dynamic });
    for (let i = 0; i < 900; i++) b.world.step();
    const buf = b.view();
    const at = (name: string) => {
        const id = dropped.spawned.find((s) => s.name === name)!.handle.id;
        return Array.from(buf.slice(id * 7, id * 7 + 7));
    };
    return {
        centroidError: Math.max(...got.map((v, i) => Math.abs(v - [expected.x, expected.y, expected.z][i]))),
        kinds: report.spawned.map((s) => s.shape),
        levelKinds: scene.colliders.map((c) => `${c.name}:${c.shape.kind}`),
        failed: [...report.failed, ...dropped.failed],
        skipped: [...placedScene.skipped, ...scene.skipped],
        malformed: b.world.runtimeCounters.malformedDropped + malformedA,
        cone: at('cone'),
        ball: at('ball'),
        sleeping: b.rt.sleeping_count(),
    };
}

run().then(
    (r) => { (window as any).__result = r; document.getElementById('out')!.textContent = JSON.stringify(r); },
    (e) => { (window as any).__result = { error: String(e) }; document.getElementById('out')!.textContent = String(e); },
);
