import { REACHER_WGSL } from './reacher.wgsl.js';

/** Floats per world in the state buffer: position 3, rotation 4, velocity 3, angular velocity 3, nine warm-start impulses, two spare. */
export const STATE_FLOATS = 24;
export const OBS_FLOATS = 2;
export const MAX_WORLDS = 1 << 20;

type V3 = readonly [number, number, number];

/**
 * The arm scene: a static pivot box and a dynamic link on a hinge, as built by `examples/reacherEnv.ts`. Bodies start unrotated. `reacherParams` refuses what the CPU core
 * refuses for such a joint. The kernel has no swept test against the pivot: a link that starts within a substep's travel of the pivot's box is not what the CPU runs.
 */
export interface ReacherScene {
    pivot: V3;
    link: V3;
    linkHalf: V3;
    invMass: number;
    gravity: V3;
    anchor: V3;
    axis: V3;
    limits: readonly [number, number];
    motorMax: number;
    /** Ticks are `dt` seconds; the CPU core runs `substeps` steps inside each. */
    dt: number;
}

export const REACHER: ReacherScene = {
    pivot: [0, 0, 0], link: [0.5, 0, 0], linkHalf: [0.5, 0.05, 0.1], invMass: 1, gravity: [0, -9.81, 0], anchor: [0, 0, 0], axis: [0, 0, 1],
    limits: [-2.5, 2.5], motorMax: 20, dt: 1 / 60,
};

const SUBSTEPS = 4;
const SOLVER_ITERS = 32;
const f = Math.fround;

// The simulation core takes a root per halving of the step count.
function perStep(factor: number, k: number): number {
    let x = f(factor);
    for (let n = k; n > 1; n = Math.floor(n / 2)) x = f(Math.sqrt(x));
    return x;
}

function unit(v: V3): V3 {
    const inv = f(1 / f(Math.sqrt(f(f(v[0] * v[0]) + f(v[1] * v[1]) + f(v[2] * v[2])))));
    return [f(v[0] * inv), f(v[1] * inv), f(v[2] * inv)];
}

function cross(a: V3, b: V3): V3 {
    return [f(f(a[1] * b[2]) - f(a[2] * b[1])), f(f(a[2] * b[0]) - f(a[0] * b[2])), f(f(a[0] * b[1]) - f(a[1] * b[0]))];
}

/** The reference direction of a hinge, perpendicular to its axis: the first tangent the core builds. */
function perpendicular(axis: V3): V3 {
    const pick: V3 = Math.abs(axis[0]) < 0.57 ? [1, 0, 0] : [0, 1, 0];
    return unit(cross(axis, pick));
}

const MAX_COORD = 1e6;
const MAX_ANGLE_LIMIT = 3.1;
export const MAX_MOTOR_SPEED = 1e3;
const MAX_MOTOR_FORCE = 1e6;

function check(ok: boolean, what: string): void {
    if (!ok) throw new RangeError(what);
}

function validate(s: ReacherScene): void {
    const finite3 = (v: V3, bound: number) => v.length === 3 && v.every((c) => Number.isFinite(c) && Math.abs(c) <= bound);
    check(finite3(s.pivot, MAX_COORD) && finite3(s.link, MAX_COORD) && finite3(s.anchor, MAX_COORD), 'pivot, link and anchor must be finite 3-vectors within range');
    check(finite3(s.gravity, 1e3), 'gravity must be a finite 3-vector within 1000');
    check(s.linkHalf.length === 3 && s.linkHalf.every((h) => h >= 1e-3 && h <= 1e4), 'link half sizes must be within 0.001..10000');
    check(s.invMass > 0 && s.invMass <= 1e6, `invMass must be within (0, 1e6], got ${s.invMass}: a link with no inverse mass is static`);
    check(Number.isFinite(s.dt) && s.dt > 0 && s.dt <= 1, `dt must be within (0, 1], got ${s.dt}`);
    check(s.motorMax >= 0 && s.motorMax <= MAX_MOTOR_FORCE, `motorMax must be within 0..${MAX_MOTOR_FORCE}, got ${s.motorMax}`);
    check(finite3(s.axis, MAX_COORD) && f(f(s.axis[0] * s.axis[0]) + f(s.axis[1] * s.axis[1]) + f(s.axis[2] * s.axis[2])) > 1e-12 && Number.isFinite(f(s.axis[0] * s.axis[0]) + f(s.axis[1] * s.axis[1]) + f(s.axis[2] * s.axis[2])), 'the hinge axis must be a finite vector of usable length');
    const [lo, hi] = s.limits;
    check(Number.isFinite(lo) && Number.isFinite(hi) && lo <= 0 && hi >= 0 && Math.abs(lo) <= MAX_ANGLE_LIMIT && Math.abs(hi) <= MAX_ANGLE_LIMIT, `limits must bracket zero within +-${MAX_ANGLE_LIMIT} rad, got [${lo}, ${hi}]`);
}

export function reacherParams(s: ReacherScene): Float32Array {
    validate(s);
    const [hx, hy, hz] = s.linkHalf;
    const inertia = f(f(1 / s.invMass) * f(f(2 / 9) * f(f(hx * hx) + f(hy * hy) + f(hz * hz))));
    const axis = unit(s.axis);
    const ref = perpendicular(axis);
    const sub = (a: V3, b: V3): V3 => [f(a[0] - b[0]), f(a[1] - b[1]), f(a[2] - b[2])];
    const bias = f(1 - perStep(0.8, SUBSTEPS));
    const p = new Float32Array(48);
    p.set([s.dt, s.invMass, f(1 / inertia), s.motorMax], 0);
    p.set([...s.gravity, 0], 4);
    p.set([s.limits[0], s.limits[1], perStep(0.99, SUBSTEPS), perStep(0.97, SUBSTEPS)], 8);
    p.set([bias, SOLVER_ITERS, SUBSTEPS, 1], 12);
    p.set([...sub(s.anchor, s.pivot), 0], 16);
    p.set([...sub(s.anchor, s.link), 0], 20);
    p.set([...axis, 0, ...axis, 0, ...ref, 0, ...ref, 0], 24);
    p.set([0, 0, 0, 1], 40);
    p.set([...s.pivot, 0], 44);
    return p;
}

/**
 * `worlds` copies of the arm scene stepped by one compute dispatch a tick. One invocation owns one world: the state is a row of `STATE_FLOATS` in one storage
 * buffer, the motor command a float in another, the observation `[angle, angular velocity]` a pair in a third.
 */
export class GpuReacher {
    private destroyed = false;
    private reading: Promise<unknown> = Promise.resolve();

    private constructor(
        readonly worlds: number,
        readonly scene: ReacherScene,
        private readonly device: GPUDevice,
        private readonly pipeline: GPUComputePipeline,
        private readonly bind: GPUBindGroup,
        private readonly state: GPUBuffer,
        private readonly motors: GPUBuffer,
        private readonly obs: GPUBuffer,
        private readonly staging: GPUBuffer,
    ) {}

    static async create(device: GPUDevice, worlds: number, scene: ReacherScene = REACHER): Promise<GpuReacher> {
        if (!Number.isInteger(worlds) || worlds < 1 || worlds > MAX_WORLDS) throw new RangeError(`worlds must be an integer within 1..${MAX_WORLDS}, got ${worlds}`);
        reacherParams(scene);
        const module = device.createShaderModule({ code: REACHER_WGSL });
        const info = await module.getCompilationInfo();
        const errors = info.messages.filter((m) => m.type === 'error');
        if (errors.length) throw new Error(`the reacher shader did not compile: ${errors.map((m) => `${m.lineNum}: ${m.message}`).join('; ')}`);
        const pipeline = await device.createComputePipelineAsync({ layout: 'auto', compute: { module, entryPoint: 'step_world' } });
        const params = device.createBuffer({ size: 48 * 4, usage: GPUBufferUsage.UNIFORM | GPUBufferUsage.COPY_DST });
        device.queue.writeBuffer(params, 0, reacherParams(scene));
        const state = device.createBuffer({ size: worlds * STATE_FLOATS * 4, usage: GPUBufferUsage.STORAGE | GPUBufferUsage.COPY_SRC | GPUBufferUsage.COPY_DST });
        const motors = device.createBuffer({ size: worlds * 4, usage: GPUBufferUsage.STORAGE | GPUBufferUsage.COPY_DST });
        const obs = device.createBuffer({ size: worlds * OBS_FLOATS * 4, usage: GPUBufferUsage.STORAGE | GPUBufferUsage.COPY_SRC });
        const staging = device.createBuffer({ size: worlds * STATE_FLOATS * 4 + worlds * OBS_FLOATS * 4, usage: GPUBufferUsage.MAP_READ | GPUBufferUsage.COPY_DST });
        const bind = device.createBindGroup({
            layout: pipeline.getBindGroupLayout(0),
            entries: [params, state, motors, obs].map((buffer, binding) => ({ binding, resource: { buffer } })),
        });
        const reacher = new GpuReacher(worlds, scene, device, pipeline, bind, state, motors, obs, staging);
        reacher.reset();
        return reacher;
    }

    /** Every world back at the scene's start. `kick` holds an impulse `[x, y, z]` per world for the link, as `world.force(link, x, y, z)` gives it; none means none. */
    reset(kick?: Float32Array): void {
        this.assertAlive();
        if (kick && !kick.every(Number.isFinite)) throw new RangeError('kick components must be finite');
        if (kick && kick.length !== this.worlds * 3) throw new RangeError(`three kick components per world: expected ${this.worlds * 3}, got ${kick.length}`);
        const s = this.scene;
        const rows = new Float32Array(this.worlds * STATE_FLOATS);
        for (let i = 0; i < this.worlds; i++) {
            rows.set([...s.link, 0, 0, 0, 1, ...[0, 1, 2].map((k) => f(f(kick?.[i * 3 + k] ?? 0) * s.invMass)), 0, 0, 0, 0], i * STATE_FLOATS);
        }
        this.device.queue.writeBuffer(this.state, 0, rows);
        this.device.queue.writeBuffer(this.motors, 0, new Float32Array(this.worlds));
    }

    /** The motor speed of each world for the ticks that follow. Each tick it is treated as a fresh command, so its warm start restarts, as a `setJointMotor` every tick does on the CPU. */
    setMotors(speeds: Float32Array): void {
        this.assertAlive();
        if (speeds.length !== this.worlds) throw new RangeError(`one motor speed per world: expected ${this.worlds}, got ${speeds.length}`);
        if (!speeds.every((v) => Number.isFinite(v) && Math.abs(v) <= MAX_MOTOR_SPEED)) throw new RangeError(`motor speeds must be finite and within +-${MAX_MOTOR_SPEED}`);
        this.device.queue.writeBuffer(this.motors, 0, speeds);
    }

    /** Runs `ticks` ticks with the motor commands as they stand. */
    step(ticks = 1): void {
        this.assertAlive();
        if (!Number.isInteger(ticks) || ticks < 1) throw new RangeError(`ticks must be a positive integer, got ${ticks}`);
        const enc = this.device.createCommandEncoder();
        for (let t = 0; t < ticks; t++) {
            const pass = enc.beginComputePass();
            pass.setPipeline(this.pipeline);
            pass.setBindGroup(0, this.bind);
            pass.dispatchWorkgroups(Math.ceil(this.worlds / 64));
            pass.end();
        }
        this.device.queue.submit([enc.finish()]);
    }

    /** Waits for the queued ticks and returns `[state rows, observation pairs]` as of the last one. One copy each way, never one call per world. */
    read(): Promise<{ state: Float32Array; obs: Float32Array }> {
        this.assertAlive();
        // One staging buffer: a second read waits for the first to unmap.
        const next = this.reading.then(() => this.copyOut());
        this.reading = next.catch(() => undefined);
        return next;
    }

    private async copyOut(): Promise<{ state: Float32Array; obs: Float32Array }> {
        const enc = this.device.createCommandEncoder();
        const stateBytes = this.worlds * STATE_FLOATS * 4;
        enc.copyBufferToBuffer(this.state, 0, this.staging, 0, stateBytes);
        enc.copyBufferToBuffer(this.obs, 0, this.staging, stateBytes, this.worlds * OBS_FLOATS * 4);
        this.device.queue.submit([enc.finish()]);
        await this.staging.mapAsync(GPUMapMode.READ);
        const all = new Float32Array(this.staging.getMappedRange().slice(0));
        this.staging.unmap();
        return { state: all.subarray(0, this.worlds * STATE_FLOATS), obs: all.subarray(this.worlds * STATE_FLOATS) };
    }

    private assertAlive(): void {
        if (this.destroyed) throw new Error('this GpuReacher was destroyed');
    }

    destroy(): void {
        this.destroyed = true;
        for (const b of [this.state, this.motors, this.obs, this.staging]) b.destroy();
    }
}
