import { WorldBatch } from '@dynova/sdk';
import { JOINT_DISTANCE, JOINT_HINGE, JOINT_SLIDER } from '@dynova/sdk';
import { GpuReacher, REACHER, STATE_FLOATS, type FloorBox, type ReacherScene } from '@dynova/gpu';

const config = { capacity: 8, ringCapacity: 128 };
const params = new URLSearchParams(location.search);
const WORLDS = Number(params.get('n') ?? 64);
// `ticks=N` runs longer so that bodies reach sleep; the checks then sit at fractions of it.
const RUN = Number(params.get('ticks') ?? 200);
const CHECKS = RUN === 200 ? [1, 10, 50, 100, 200] : [1, 10, RUN / 4, RUN / 2, (RUN * 3) / 4, RUN];
const TICKS = CHECKS[CHECKS.length - 1];

// Part of the kick is along the hinge axis, so the link starts tilting out of its plane and the hinge has to pull it back.
// `coast=1` is a slider along x with limits a few metres away: the kick is a push along it at 0.05 to 0.4 m/s, which damping slows through the sleep speed of 0.3 m/s.
const COAST = params.get('coast') === '1';
const kick = (i: number): [number, number, number] => COAST ? [((i % 8) + 1) * 0.05, 0, 0] : [0, KICK * (((i * 37) % 81) - 40) / 10, KICK * (((i * 13) % 21) - 10) / 20];
// `skew` moves the pivot off the origin, tilts the hinge axis, bends gravity, and changes mass and limits: what the origin scene's symmetry hides.
const SKEW = params.get('scene') === 'skew';
// `joint=slider` swaps the hinge for a slider: the same bodies, the limits and the motor now act on the offset along the axis in metres.
const SLIDER = params.get('joint') === 'slider' || params.get('coast') === '1';
// `joint=rod` holds a corner-side point of the link at a fixed distance from the pivot's underside; `joint=rope` lets that distance shrink by 0.4 m but not grow. The link hangs clear of the pivot box,
// which the CPU would otherwise collide it with (a distance joint does not switch the pair's contact off).
const ROD = params.get('joint') === 'rod', ROPE = params.get('joint') === 'rope';
// `floor=<name>` adds a static box the link collides with. The tops sit where the swinging or falling link reaches them within the run.
const tilt = (a: number): [number, number, number, number] => [0, 0, Math.sin(a / 2), Math.cos(a / 2)];
const FLOORS: Record<string, FloorBox> = {
    // below the arm's tip: it meets the floor at an angle, an edge of the link's end against the face
    tip: { pos: [0, -1.1, 0], half: [3, 0.5, 3], friction: 0.5 },
    // under a slider falling along y: the link's flat face lands on the floor face, four contact points
    flat: { pos: [0, -0.85, 0], half: [3, 0.5, 3], friction: 0.6 },
    // a floor thinner than a substep's travel at 40 m/s under a slider: only the swept test keeps the link from passing through it
    thin: { pos: [0, -0.43, 0], half: [3, 0.02, 3], friction: 0.5 },
    // tilted about x and y, for the skewed scene: the contact normal and the ordering direction are off every axis
    twist: { pos: [1, -0.6, 0.2], half: [3, 0.5, 3], rot: [0.1, 0.2, 0.05, Math.sqrt(1 - 0.01 - 0.04 - 0.0025)], friction: 0.6 },
    // tilted 0.3 rad about z so the normal and the friction tangents leave the axes, and a bouncy surface
    slope: { pos: [0.4, -1.0, 0], half: [3, 0.5, 3], rot: tilt(0.3), friction: 0.8, restitution: 0.7 },
    // a post narrower than the link's end under a hanging link: the patch is the post's top, clipped by its edges, and a hanging link rests 0.015 into it
    // just under a hanging rod's link (its underside is at -1.25): a link that turns about its anchor point dips a corner into it
    deck: { pos: [0, -1.8, 0], half: [3, 0.5, 3], friction: 0.5 },
    block: { pos: [0, -1.085, 0], half: [0.04, 0.1, 0.15], friction: 0.5 },
};
const FLOOR = params.get('floor');
if (FLOOR && !FLOORS[FLOOR]) throw new Error(`unknown floor ${FLOOR}`);
const SCENE: ReacherScene = {
    ...(SKEW ? { ...REACHER, pivot: [0.5, 0.3, 0.2], link: [1, 0.3, 0.2], anchor: [0.6, 0.3, 0.2], axis: [0.3, 0.5, 0.8], gravity: [1, -9.81, 2], invMass: 3, limits: [-1, 0.7] } : REACHER),
    // `sleep=1` has no motor, so a link that comes to rest falls asleep as on the CPU.
    ...(params.get('sleep') === '1' ? { motorMax: 0 } : {}),
    // `lever=1` puts the anchor off the link's centre in y and z, so a push has a lever about every axis the slider must hold.
    ...(params.get('lever') === '1' ? { anchor: [0.1, 0.2, 0.1] } : {}),
    ...(SLIDER ? { kind: 'slider', limits: SKEW ? [-0.4, 0.3] : [-0.5, 0.3], axis: SKEW ? [0.3, 0.5, 0.8] : [0, 1, 0] } : {}),
    ...(ROD || ROPE ? { kind: 'distance', pivot: [0, 0, 0], link: [0.3, -1.2, 0.1], anchor: [0, -0.1, 0], axis: [0, -1.15, 0.15], limits: ROPE ? [-0.4, 0] : null, motorMax: 0 } : {}),
    ...(COAST ? { kind: 'slider', axis: [1, 0, 0], limits: [-3, 3] } : {}),
    ...(FLOOR ? { floor: FLOORS[FLOOR], linkFriction: 0.5, linkRestitution: params.get('bounce') === '1' ? 0.9 : 0 } : {}),
} as ReacherScene;
const AMP = Number(params.get('amp') ?? 3);
const KICK = Number(params.get('kick') ?? 1);
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
        concurrentReadsAgree: Array.from(a.state).join() === Array.from(b.state).join() && a.state.length === 2 * STATE_FLOATS,
        readIsASnapshot: [0, 1, 2, 3].some((k) => Math.abs(snap[k] - late[k]) > 1e-3),
        nanMotor: refused(() => g.setMotors(Float32Array.from([0, Number.NaN]))),
        hugeMotor: refused(() => g.setMotors(Float32Array.from([0, 1e4]))),
        nanKick: refused(() => g.reset(Float32Array.from([0, 0, 0, 0, Number.NaN, 0]))),
        staticLink: await GpuReacher.create(device, 1, { ...REACHER, invMass: 0 }).then(() => false, (e) => e instanceof RangeError),
        resetObs: false,
        usedAfterDestroy: false,
    };
    g.reset(Float32Array.from([0, 1, 0, 0, 2, 0]));
    out.resetObs = (await g.readObs()).every((v) => v === 0);
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
        const link = w.spawn({ x: SCENE.link[0], y: SCENE.link[1], z: SCENE.link[2], hx: SCENE.linkHalf[0], hy: SCENE.linkHalf[1], hz: SCENE.linkHalf[2], invMass: SCENE.invMass, friction: SCENE.linkFriction ?? 0, restitution: SCENE.linkRestitution ?? 0 });
        if (SCENE.floor) {
            const [qx, qy, qz, qw] = SCENE.floor.rot ?? [0, 0, 0, 1];
            w.spawn({ x: SCENE.floor.pos[0], y: SCENE.floor.pos[1], z: SCENE.floor.pos[2], qx, qy, qz, qw, hx: SCENE.floor.half[0], hy: SCENE.floor.half[1], hz: SCENE.floor.half[2], invMass: 0, friction: SCENE.floor.friction ?? 0.5, restitution: SCENE.floor.restitution ?? 0 });
        }
        const joint = w.createJoint({ kind: ROD || ROPE ? JOINT_DISTANCE : SLIDER ? JOINT_SLIDER : JOINT_HINGE, a: pivot, b: link, anchor: SCENE.anchor, ...(ROD || ROPE ? { anchorB: SCENE.axis } : { axis: SCENE.axis }), limits: SCENE.limits ?? undefined, ...(ROD || ROPE ? {} : { motor: { velocity: 0, maxForce: SCENE.motorMax } }) });
        return { link, joint };
    });
    set.worlds.forEach((w, i) => w.force(ctx[i].link, ...kick(i)));
    const gpu = await GpuReacher.create(device, WORLDS, SCENE);
    gpu.reset(Float32Array.from({ length: WORLDS * 3 }, (_, k) => kick(Math.floor(k / 3))[k % 3]));

    const checks: Array<{ tick: number; angle: number; velocity: number; position: number; rotation: number; asleep: number; gpuAsleepCpuMoving: number; cpuFrozenGpuAwake: number }> = [];
    // A sleeping body is frozen exactly, so a CPU body that moved over a tick was awake at its start, and the GPU must agree. The other way round proves less: a body
    // pressed against a limit can stand exactly still awake, until its quiet time is up.
    // Per world: the last tick the CPU pose changed and the first tick the GPU phase was asleep. A body that falls asleep mid-tick moves in that tick and is frozen after it,
    // so the GPU's tick is the CPU's last moving tick or the one after: a rule off by a few substeps shows even where the pose gap is below tolerance.
    const lastMoved = new Array<number>(WORLDS).fill(0), asleepAt = new Array<number>(WORLDS).fill(-1);
    let lastPose: Float32Array[] = set.worlds.map((w) => w.readTransforms().slice(7, 10));
    let before: { cpu: Float32Array[]; phase: number[] } | null = null;
    let widest = 0, atLimit = 0;
    // Worlds whose link had a corner inside the floor (grown by a hair) at the end of some tick: proof the scene reaches the contact path at all.
    const touched = new Array<boolean>(WORLDS).fill(false);
    const touchesFloor = (body: Float32Array) => {
        const fl = SCENE.floor!, fq = fl.rot ?? [0, 0, 0, 1];
        const rotate = (q: readonly number[], v: number[]) => {
            const [ux, uy, uz] = q, qw = q[3];
            const tx = 2 * (uy * v[2] - uz * v[1]), ty = 2 * (uz * v[0] - ux * v[2]), tz = 2 * (ux * v[1] - uy * v[0]);
            return [v[0] + qw * tx + (uy * tz - uz * ty), v[1] + qw * ty + (uz * tx - ux * tz), v[2] + qw * tz + (ux * ty - uy * tx)];
        };
        const lq = [body[10], body[11], body[12], body[13]];
        for (const sx of [-1, 1]) for (const sy of [-1, 1]) for (const sz of [-1, 1]) {
            const c = rotate(lq, [sx * SCENE.linkHalf[0], sy * SCENE.linkHalf[1], sz * SCENE.linkHalf[2]]);
            const local = rotate([-fq[0], -fq[1], -fq[2], fq[3]], [body[7] + c[0] - fl.pos[0], body[8] + c[1] - fl.pos[1], body[9] + c[2] - fl.pos[2]]);
            if ([0, 1, 2].every((k) => Math.abs(local[k]) <= fl.half[k] + 0.01)) return true;
        }
        return false;
    };
    for (let t = 1; t <= TICKS; t++) {
        // The CPU wakes a body on every motor command, so a world meant to sleep gets none.
        if (SCENE.motorMax > 0) set.worlds.forEach((w, i) => w.setJointMotor(ctx[i].joint, motor(i, t), SCENE.motorMax));
        // A sleep scene commands nothing on either side: the CPU would wake the body on every motor command, the GPU ignores commands to a sleeper.
        gpu.setMotors(Float32Array.from({ length: WORLDS }, (_, i) => (SCENE.motorMax > 0 ? motor(i, t) : 0)));
        set.step();
        gpu.step();
        if (SCENE.floor) set.worlds.forEach((w, i) => { if (!touched[i] && touchesFloor(w.readTransforms())) touched[i] = true; });
        if (SCENE.motorMax === 0) {
            const { state: now } = await gpu.read();
            set.worlds.forEach((w, i) => {
                const pose = w.readTransforms().slice(7, 10);
                if ([0, 1, 2].some((k) => pose[k] !== lastPose[i][k])) lastMoved[i] = t;
                lastPose[i] = pose;
                if (asleepAt[i] < 0 && now[i * STATE_FLOATS + 23] === 2) asleepAt[i] = t;
            });
        }
        if (CHECKS.includes(t + 1)) {
            const { state: s0 } = await gpu.read();
            before = { cpu: set.worlds.map((w) => w.readTransforms().slice(7, 10)), phase: set.worlds.map((_, i) => s0[i * STATE_FLOATS + 23]) };
        }
        if (!CHECKS.includes(t)) continue;
        const { state, obs } = await gpu.read();
        let asleep = 0, gpuAsleepCpuMoving = 0, cpuFrozenGpuAwake = 0;
        let angle = 0, velocity = 0, position = 0, rotation = 0;
        set.worlds.forEach((w, i) => {
            const joint = w.readJointStates()[0];
            const body = w.readTransforms();
            const at = i * STATE_FLOATS;
            angle = Math.max(angle, Math.abs(joint.position - obs[i * 2]));
            velocity = Math.max(velocity, Math.abs(joint.velocity - obs[i * 2 + 1]));
            for (let k = 0; k < 3; k++) position = Math.max(position, Math.abs(body[7 + k] - state[at + k]));
            for (let k = 0; k < 4; k++) rotation = Math.max(rotation, Math.abs(body[7 + 3 + k] - state[at + 3 + k]));
            if (state[at + 23] === 2) asleep++;
            if (before) {
                const frozen = [0, 1, 2].every((k) => body[7 + k] === before!.cpu[i][k]);
                if (before.phase[i] === 2 && !frozen) gpuAsleepCpuMoving++;
                if (frozen && before.phase[i] !== 2) cpuFrozenGpuAwake++;
            }
            widest = Math.max(widest, Math.abs(joint.position));
            if (SCENE.limits?.some((l) => Math.abs(joint.position - l) < 0.05)) atLimit++;
        });
        checks.push({ tick: t, angle, velocity, position, rotation, asleep, gpuAsleepCpuMoving, cpuFrozenGpuAwake });
        before = null;
    }
    const guards = await probe(device);
    const info = adapter.info;
    // A link parked against a limit stands exactly still awake until its quiet time is up, so its pose says nothing about when it should sleep: those are left out.
    const parked = set.worlds.map((w) => (SCENE.limits ?? []).some((l) => Math.abs(w.readJointStates()[0].position - l) < 1e-3));
    const sleepTickMisses = SCENE.motorMax === 0 ? asleepAt.filter((g, i) => !parked[i] && (g < 0 || ![0, 1].includes(g - lastMoved[i]))).length : 0;
    return { guards, sleepTickMisses, touched: touched.filter(Boolean).length, worlds: WORLDS, checks, widest, atLimit, adapter: `${info.vendor} ${info.architecture}`, userAgent: navigator.userAgent };
}

run().then(
    (r) => { (window as any).__result = r; document.getElementById('out')!.textContent = JSON.stringify(r); },
    (e) => { (window as any).__result = { error: String(e) }; document.getElementById('out')!.textContent = String(e); },
);
