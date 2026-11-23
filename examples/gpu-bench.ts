import { WorldBatch, JOINT_HINGE } from '@dynova/sdk';
import { GpuReacher, REACHER } from '@dynova/gpu';

const config = { capacity: 8, ringCapacity: 128 };
const params = new URLSearchParams(location.search);
const WORLDS = Number(params.get('n') ?? 4096);
const TICKS = Number(params.get('ticks') ?? 100);
const PIPELINE = 10;
const AGREE = 20;
const motor = (i: number, t: number) => 3 * Math.sin(0.05 * t + i * 0.3);

// Both sides do the same work each tick of a control loop: command every motor, step, read every joint angle and velocity. The CPU side also pays the SDK's per-world
// call and allocation cost and the GPU side the host fill and validation of the motor array: those are part of a control loop, so they stay in. The CPU worlds are not
// reset (they have run the agreement and warm-up ticks first), which does not change their cost per tick.
async function run() {
    const adapter = await navigator.gpu?.requestAdapter();
    if (!adapter) return { skipped: 'no WebGPU adapter', userAgent: navigator.userAgent };
    const device = await adapter.requestDevice();
    const glue = await import('../sdk/typescript/wasm-st/dynova_core.js');
    const exports = await glue.default() as unknown as { memory: WebAssembly.Memory };
    const batch = new glue.Batch(WORLDS, config.capacity, config.ringCapacity);
    batch.enable_physics();
    const set = new WorldBatch(config, exports.memory, 'fallback', batch);
    const joints = set.worlds.map((w) => {
        w.setGravity(...REACHER.gravity);
        const pivot = w.spawn({ x: REACHER.pivot[0], y: REACHER.pivot[1], z: REACHER.pivot[2], hx: 0.1, hy: 0.1, hz: 0.1, invMass: 0 });
        const link = w.spawn({ x: REACHER.link[0], y: REACHER.link[1], z: REACHER.link[2], hx: REACHER.linkHalf[0], hy: REACHER.linkHalf[1], hz: REACHER.linkHalf[2], invMass: REACHER.invMass, friction: 0 });
        return w.createJoint({ kind: JOINT_HINGE, a: pivot, b: link, anchor: REACHER.anchor, axis: REACHER.axis, limits: REACHER.limits ?? undefined, motor: { velocity: 0, maxForce: REACHER.motorMax } });
    });
    const gpu = await GpuReacher.create(device, WORLDS);
    const speeds = new Float32Array(WORLDS);
    const fill = (t: number) => { for (let i = 0; i < WORLDS; i++) speeds[i] = motor(i, t); return speeds; };

    const cpuTick = (t: number) => {
        set.worlds.forEach((w, i) => w.setJointMotor(joints[i], motor(i, t), 20));
        set.step();
        let sum = 0;
        for (const w of set.worlds) sum += w.readJointStates()[0].position;
        return sum;
    };
    const gpuTick = async (t: number) => { gpu.setMotors(fill(t)); gpu.step(); return gpu.readObs(); };

    // Open-loop agreement before any timing: `step(AGREE)` runs several passes in one submission, which the single-tick tolerance suite does not exercise.
    gpu.setMotors(fill(7));
    gpu.step(AGREE);
    for (let t = 0; t < AGREE; t++) { set.worlds.forEach((w, i) => w.setJointMotor(joints[i], speeds[i], 20)); set.step(); }
    const obs = await gpu.readObs();
    let angleError = 0, velocityError = 0, moved = 0;
    set.worlds.forEach((w, i) => {
        const j = w.readJointStates()[0];
        angleError = Math.max(angleError, Math.abs(j.position - obs[i * 2]));
        velocityError = Math.max(velocityError, Math.abs(j.velocity - obs[i * 2 + 1]));
        moved = Math.max(moved, Math.abs(obs[i * 2]));
    });

    // Warm up both: the first dispatch compiles and uploads, the first CPU ticks grow the wasm heap.
    for (let t = 1; t <= 3; t++) { cpuTick(t); await gpuTick(t); }
    gpu.reset();
    await gpu.readObs();

    const start = performance.now();
    for (let t = 1; t <= TICKS; t++) cpuTick(t);
    const cpuMs = (performance.now() - start) / TICKS;

    const g0 = performance.now();
    let last = new Float32Array(0);
    for (let t = 1; t <= TICKS; t++) last = await gpuTick(t);
    const gpuMs = (performance.now() - g0) / TICKS;

    // Open loop, an upper bound and not comparable with the CPU loop above: the motor command holds for PIPELINE ticks, so the queue stays full and the read happens once per PIPELINE ticks.
    const p0 = performance.now();
    for (let t = 0; t < TICKS; t += PIPELINE) { gpu.setMotors(fill(t)); gpu.step(PIPELINE); await gpu.readObs(); }
    const pipelinedMs = (performance.now() - p0) / TICKS;

    const info = adapter.info;
    return {
        worlds: WORLDS, ticks: TICKS, pipeline: PIPELINE,
        cpuMsPerTick: cpuMs, gpuMsPerTick: gpuMs, gpuOpenLoopMsPerTick: pipelinedMs,
        speedup: cpuMs / gpuMs,         agreeTicks: AGREE, angleError, velocityError, moved, hardware: !adapter.info.isFallbackAdapter,
        finiteObs: last.every(Number.isFinite),
        adapter: `${info.vendor} ${info.architecture}`, userAgent: navigator.userAgent,
    };
}

run().then(
    (r) => { (window as any).__result = r; document.getElementById('out')!.textContent = JSON.stringify(r, null, 1); },
    (e) => { (window as any).__result = { error: String(e) }; document.getElementById('out')!.textContent = String(e); },
);
