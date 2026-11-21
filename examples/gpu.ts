import { WorldBatch } from '@dynova/sdk';
import { JOINT_HINGE } from '@dynova/sdk';
import { GpuReacher, REACHER, STATE_FLOATS, type ReacherScene } from '@dynova/gpu';

const config = { capacity: 8, ringCapacity: 128 };
const params = new URLSearchParams(location.search);
const WORLDS = Number(params.get('n') ?? 64);
const CHECKS = [1, 10, 50, 100, 200];
const TICKS = CHECKS[CHECKS.length - 1];

// Part of the kick is along the hinge axis, so the link starts tilting out of its plane and the hinge has to pull it back.
const kick = (i: number): [number, number, number] => [0, (((i * 37) % 81) - 40) / 10, (((i * 13) % 21) - 10) / 20];
// `skew` moves the pivot off the origin, tilts the hinge axis, bends gravity, and changes mass and limits: what the origin scene's symmetry hides.
const SCENE: ReacherScene = params.get('scene') === 'skew'
    ? { ...REACHER, pivot: [0.5, 0.3, 0.2], link: [1, 0.3, 0.2], anchor: [0.6, 0.3, 0.2], axis: [0.3, 0.5, 0.8], gravity: [1, -9.81, 2], invMass: 3, limits: [-1, 0.7] }
    : REACHER;
const AMP = Number(params.get('amp') ?? 3);
const motor = (i: number, t: number) => AMP * Math.sin(0.05 * t + i * 0.3);

// What a caller can get wrong on the host side: each must be refused or serialised, never run on bad data.
async function probe(device: GPUDevice) {
    const g = await GpuReacher.create(device, 2);
    g.reset(Float32Array.from([0, 1, 0, 0, 2, 0]));
    g.step(3);
    const [a, b] = await Promise.all([g.read(), g.read()]);
    g.reset(Float32Array.from([0, 1, 0, 0, 2, 0]));
    g.setMotors(Float32Array.from([2, 2]));
    g.step(3);
    const early = g.readObs();
    g.step(40);
    const [snap, late] = await Promise.all([early, g.readObs()]);
    const refused = (f: () => void) => { try { f(); return false; } catch (e) { return e instanceof RangeError; } };
    const out = {
        concurrentReadsAgree: Array.from(a.state).join() === Array.from(b.state).join() && a.state.length === 48,
        readIsASnapshot: [0, 1, 2, 3].some((k) => Math.abs(snap[k] - late[k]) > 1e-3),
        nanMotor: refused(() => g.setMotors(Float32Array.from([0, Number.NaN]))),
        hugeMotor: refused(() => g.setMotors(Float32Array.from([0, 1e4]))),
        nanKick: refused(() => g.reset(Float32Array.from([0, 0, 0, 0, Number.NaN, 0]))),
        staticLink: await GpuReacher.create(device, 1, { ...REACHER, invMass: 0 }).then(() => false, (e) => e instanceof RangeError),
        usedAfterDestroy: false,
    };
    g.destroy();
    out.usedAfterDestroy = (() => { try { g.step(); return false; } catch { return true; } })();
    return out;
}

async function run() {
    const adapter = await navigator.gpu?.requestAdapter();
    if (!adapter) return { skipped: 'no WebGPU adapter', userAgent: navigator.userAgent };
    const device = await adapter.requestDevice();
    const glue = await import('../sdk/typescript/wasm-st/dynova_core.js');
    const exports = await glue.default() as unknown as { memory: WebAssembly.Memory };
    const batch = new glue.Batch(WORLDS, config.capacity, config.ringCapacity);
    batch.enable_physics();
    const set = new WorldBatch(config, exports.memory, 'fallback', batch);
    const ctx = set.worlds.map((w) => {
        w.setGravity(...SCENE.gravity);
        const pivot = w.spawn({ x: SCENE.pivot[0], y: SCENE.pivot[1], z: SCENE.pivot[2], hx: 0.1, hy: 0.1, hz: 0.1, invMass: 0 });
        const link = w.spawn({ x: SCENE.link[0], y: SCENE.link[1], z: SCENE.link[2], hx: SCENE.linkHalf[0], hy: SCENE.linkHalf[1], hz: SCENE.linkHalf[2], invMass: SCENE.invMass, friction: 0 });
        const joint = w.createJoint({ kind: JOINT_HINGE, a: pivot, b: link, anchor: SCENE.anchor, axis: SCENE.axis, limits: SCENE.limits, motor: { velocity: 0, maxForce: SCENE.motorMax } });
        return { link, joint };
    });
    set.worlds.forEach((w, i) => w.force(ctx[i].link, ...kick(i)));
    const gpu = await GpuReacher.create(device, WORLDS, SCENE);
    gpu.reset(Float32Array.from({ length: WORLDS * 3 }, (_, k) => kick(Math.floor(k / 3))[k % 3]));

    const checks: Array<{ tick: number; angle: number; velocity: number; position: number; rotation: number }> = [];
    let widest = 0, atLimit = 0;
    for (let t = 1; t <= TICKS; t++) {
        set.worlds.forEach((w, i) => w.setJointMotor(ctx[i].joint, motor(i, t), 20));
        gpu.setMotors(Float32Array.from({ length: WORLDS }, (_, i) => motor(i, t)));
        set.step();
        gpu.step();
        if (!CHECKS.includes(t)) continue;
        const { state, obs } = await gpu.read();
        let angle = 0, velocity = 0, position = 0, rotation = 0;
        set.worlds.forEach((w, i) => {
            const joint = w.readJointStates()[0];
            const body = w.readTransforms();
            const at = i * STATE_FLOATS;
            angle = Math.max(angle, Math.abs(joint.position - obs[i * 2]));
            velocity = Math.max(velocity, Math.abs(joint.velocity - obs[i * 2 + 1]));
            for (let k = 0; k < 3; k++) position = Math.max(position, Math.abs(body[7 + k] - state[at + k]));
            for (let k = 0; k < 4; k++) rotation = Math.max(rotation, Math.abs(body[7 + 3 + k] - state[at + 3 + k]));
            widest = Math.max(widest, Math.abs(joint.position));
            if (Math.abs(joint.position) > 2.45) atLimit++;
        });
        checks.push({ tick: t, angle, velocity, position, rotation });
    }
    const guards = await probe(device);
    const info = adapter.info;
    return { guards, worlds: WORLDS, checks, widest, atLimit, adapter: `${info.vendor} ${info.architecture}`, userAgent: navigator.userAgent };
}

run().then(
    (r) => { (window as any).__result = r; document.getElementById('out')!.textContent = JSON.stringify(r); },
    (e) => { (window as any).__result = { error: String(e) }; document.getElementById('out')!.textContent = String(e); },
);
