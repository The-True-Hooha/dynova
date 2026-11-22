import { REACHER_WGSL } from './reacher.wgsl.js';

/** Floats per world in the state buffer: position 3, rotation 4, velocity 3, angular velocity 3, nine warm-start impulses, the sleep timer in seconds, the phase (0 fresh, 1 awake, 2 asleep), whether a deep contact held the link in its last tick, and four contact slots of nine (normal impulse, two friction impulses, point, normal). */
export const STATE_FLOATS = 61;
export const OBS_FLOATS = 2;
export const MAX_WORLDS = 1 << 20;

type V3 = readonly [number, number, number];

/**
 * The arm scene: a static pivot box and a dynamic link on a hinge, as built by `examples/reacherEnv.ts`. Bodies start unrotated, which the slider relies on (its orientation rows hold the start rotation, taken as the identity). A link sleeps by the core's rules for a lone jointed body: a motor force above 0 keeps it awake, a motor force of 0 lets it sleep after 0.2 s of rest, and only `reset` wakes it. The CPU also wakes a body on every `setJointMotor`, even with a force of 0; the GPU ignores motor commands to a sleeper, so a loop that commands every tick keeps the CPU link awake and not the GPU one. The nine warm-start slots follow the core's: a hinge uses 0-2 for the anchor and 3-4 for the axis, a slider 0-1 and 3-5, both 6 for the motor and 7-8 for the limits. `reacherParams` refuses what the CPU core
 * refuses for such a joint. The kernel has no swept test against the pivot: a link that starts within a substep's travel of the pivot's box is not what the CPU runs.
 */
export interface ReacherScene {
    /** What joins the link to the pivot; a hinge when absent. A slider limits and drives the offset along `axis` in metres, a hinge the angle about it in radians. */
    kind?: 'hinge' | 'slider';
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
    /** A static box the link collides with in every world; none when absent. */
    floor?: FloorBox;
    /** Friction and restitution of the link as `World.spawn` takes them (0.5 and 0 by default, as the core's). They only matter with a `floor`. */
    linkFriction?: number;
    linkRestitution?: number;
}

/** A static box, unrotated unless `rot` (`[x, y, z, w]`, unit length) says otherwise. */
export interface FloorBox {
    pos: V3;
    half: V3;
    rot?: readonly [number, number, number, number];
    friction?: number;
    restitution?: number;
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

const PARAM_FLOATS = 68;
const MAX_COORD = 1e6;
const MAX_ANGLE_LIMIT = 3.1;
const MAX_LINEAR_LIMIT = 1e4;
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
    if (s.floor) {
        const fl = s.floor, rotQ = fl.rot ?? [0, 0, 0, 1];
        check(finite3(fl.pos, MAX_COORD), 'the floor position must be a finite 3-vector within range');
        check(fl.half.length === 3 && fl.half.every((h) => h >= 1e-3 && h <= 1e4), 'floor half sizes must be within 0.001..10000');
        const len = rotQ.reduce((a, c) => a + c * c, 0);
        check(rotQ.length === 4 && rotQ.every(Number.isFinite) && Math.abs(len - 1) < 1e-3, 'the floor rotation must be a unit quaternion');
        for (const [name, v] of [['friction', fl.friction ?? 0.5], ['restitution', fl.restitution ?? 0]] as const) {
            check(Number.isFinite(v) && v >= 0 && v <= (name === 'friction' ? 2 : 1), `floor ${name} out of range: ${v}`);
        }
    }
    for (const [name, v] of [['linkFriction', s.linkFriction ?? 0.5], ['linkRestitution', s.linkRestitution ?? 0]] as const) {
        check(Number.isFinite(v) && v >= 0 && v <= (name === 'linkFriction' ? 2 : 1), `${name} out of range: ${v}`);
    }
    check(finite3(s.axis, MAX_COORD) && f(f(s.axis[0] * s.axis[0]) + f(s.axis[1] * s.axis[1]) + f(s.axis[2] * s.axis[2])) > 1e-12 && Number.isFinite(f(s.axis[0] * s.axis[0]) + f(s.axis[1] * s.axis[1]) + f(s.axis[2] * s.axis[2])), 'the joint axis must be a finite vector of usable length');
    check(s.kind === undefined || s.kind === 'hinge' || s.kind === 'slider', `kind must be 'hinge' or 'slider', got ${s.kind}`);
    const [lo, hi] = s.limits;
    const bound = s.kind === 'slider' ? MAX_LINEAR_LIMIT : MAX_ANGLE_LIMIT;
    check(Number.isFinite(lo) && Number.isFinite(hi) && lo <= 0 && hi >= 0 && Math.abs(lo) <= bound && Math.abs(hi) <= bound, `limits must bracket zero within +-${bound}, got [${lo}, ${hi}]`);
}

export function reacherParams(s: ReacherScene): Float32Array {
    validate(s);
    const [hx, hy, hz] = s.linkHalf;
    const inertia = f(f(1 / s.invMass) * f(f(2 / 9) * f(f(hx * hx) + f(hy * hy) + f(hz * hz))));
    const axis = unit(s.axis);
    const ref = perpendicular(axis);
    const sub = (a: V3, b: V3): V3 => [f(a[0] - b[0]), f(a[1] - b[1]), f(a[2] - b[2])];
    const bias = f(1 - perStep(0.8, SUBSTEPS));
    const p = new Float32Array(PARAM_FLOATS);
    p.set([s.dt, s.invMass, f(1 / inertia), s.motorMax], 0);
    p.set([...s.gravity, 0], 4);
    p.set([s.limits[0], s.limits[1], perStep(0.99, SUBSTEPS), perStep(0.97, SUBSTEPS)], 8);
    p.set([bias, SOLVER_ITERS, SUBSTEPS, 1], 12);
    p.set([...sub(s.anchor, s.pivot), 0], 16);
    p.set([...sub(s.anchor, s.link), 0], 20);
    p.set([...axis, 0, ...axis, 0, ...ref, 0, ...ref, 0], 24);
    p.set([0, 0, 0, 1], 40);
    p.set([...s.pivot, s.kind === 'slider' ? 1 : 0], 44);
    if (s.floor) {
        const fl = s.floor, q = fl.rot ?? [0, 0, 0, 1];
        const ql = f(Math.sqrt(f(f(q[0] * q[0]) + f(q[1] * q[1]) + f(q[2] * q[2]) + f(q[3] * q[3]))));
        p.set([...fl.pos, 1], 48);
        p.set([...fl.half, fl.friction ?? 0.5], 52);
        p.set([f(q[0] / ql), f(q[1] / ql), f(q[2] / ql), f(q[3] / ql)], 56);
    } else {
        p.set([0, 0, 0, 0, 0.5, 0.5, 0.5, 0.5, 0, 0, 0, 1], 48);
    }
    p.set([...s.linkHalf, s.linkFriction ?? 0.5], 60);
    p.set([s.linkRestitution ?? 0, s.floor?.restitution ?? 0, 0, 0], 64);
    return p;
}

/**
 * `worlds` copies of the arm scene stepped by one compute dispatch a tick. One invocation owns one world: the state is a row of `STATE_FLOATS` in one storage
 * buffer, the motor command a float in another, the observation a pair in a third: `[angle, angular velocity]` of a hinge, `[offset, speed]` along the axis of a slider.
 */
export class GpuReacher {
    private destroyed = false;

    private constructor(
        readonly worlds: number,
        readonly scene: ReacherScene,
        private readonly device: GPUDevice,
        private readonly pipeline: GPUComputePipeline,
        private readonly bind: GPUBindGroup,
        private readonly state: GPUBuffer,
        private readonly motors: GPUBuffer,
        private readonly obs: GPUBuffer,
    ) {}

    static async create(device: GPUDevice, worlds: number, scene: ReacherScene = REACHER): Promise<GpuReacher> {
        if (!Number.isInteger(worlds) || worlds < 1 || worlds > MAX_WORLDS) throw new RangeError(`worlds must be an integer within 1..${MAX_WORLDS}, got ${worlds}`);
        reacherParams(scene);
        const module = device.createShaderModule({ code: REACHER_WGSL });
        const info = await module.getCompilationInfo();
        const errors = info.messages.filter((m) => m.type === 'error');
        if (errors.length) throw new Error(`the reacher shader did not compile: ${errors.map((m) => `${m.lineNum}: ${m.message}`).join('; ')}`);
        const pipeline = await device.createComputePipelineAsync({ layout: 'auto', compute: { module, entryPoint: 'step_world' } });
        const params = device.createBuffer({ size: PARAM_FLOATS * 4, usage: GPUBufferUsage.UNIFORM | GPUBufferUsage.COPY_DST });
        device.queue.writeBuffer(params, 0, reacherParams(scene));
        const state = device.createBuffer({ size: worlds * STATE_FLOATS * 4, usage: GPUBufferUsage.STORAGE | GPUBufferUsage.COPY_SRC | GPUBufferUsage.COPY_DST });
        const motors = device.createBuffer({ size: worlds * 4, usage: GPUBufferUsage.STORAGE | GPUBufferUsage.COPY_DST });
        const obs = device.createBuffer({ size: worlds * OBS_FLOATS * 4, usage: GPUBufferUsage.STORAGE | GPUBufferUsage.COPY_SRC | GPUBufferUsage.COPY_DST });
        const bind = device.createBindGroup({
            layout: pipeline.getBindGroupLayout(0),
            entries: [params, state, motors, obs].map((buffer, binding) => ({ binding, resource: { buffer } })),
        });
        const reacher = new GpuReacher(worlds, scene, device, pipeline, bind, state, motors, obs);
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
        // The observation of the start pose, so a read before the next step does not return the pair from before the reset (a sleeper never rewrites it).
        const axis = unit(s.axis);
        const observed = new Float32Array(this.worlds * OBS_FLOATS);
        if (s.kind === 'slider') {
            for (let i = 0; i < this.worlds; i++) observed[i * OBS_FLOATS + 1] = f(f(rows[i * STATE_FLOATS + 7] * axis[0]) + f(rows[i * STATE_FLOATS + 8] * axis[1]) + f(rows[i * STATE_FLOATS + 9] * axis[2]));
        }
        this.device.queue.writeBuffer(this.obs, 0, observed);
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

    /** The state rows and observation pairs as of the ticks queued before this call. One copy each way, never one call per world. */
    async read(): Promise<{ state: Float32Array; obs: Float32Array }> {
        this.assertAlive();
        const all = await this.copyOut(true);
        return { state: all.subarray(0, this.worlds * STATE_FLOATS), obs: all.subarray(this.worlds * STATE_FLOATS) };
    }

    /** Only the observation pairs (`[coordinate, rate]`, see the class): what a control loop reads each tick, 2 floats a world instead of the state's 61 more. */
    readObs(): Promise<Float32Array> {
        this.assertAlive();
        return this.copyOut(false);
    }

    // The copy is submitted before this returns, so later step/setMotors/reset calls cannot leak into it; each read owns its staging buffer, so reads need no ordering.
    private async copyOut(withState: boolean): Promise<Float32Array> {
        const stateBytes = this.worlds * STATE_FLOATS * 4;
        const obsBytes = this.worlds * OBS_FLOATS * 4;
        const size = withState ? stateBytes + obsBytes : obsBytes;
        const staging = this.device.createBuffer({ size, usage: GPUBufferUsage.MAP_READ | GPUBufferUsage.COPY_DST });
        const enc = this.device.createCommandEncoder();
        if (withState) enc.copyBufferToBuffer(this.state, 0, staging, 0, stateBytes);
        enc.copyBufferToBuffer(this.obs, 0, staging, withState ? stateBytes : 0, obsBytes);
        this.device.queue.submit([enc.finish()]);
        try {
            await staging.mapAsync(GPUMapMode.READ);
            return new Float32Array(staging.getMappedRange().slice(0));
        } finally {
            staging.destroy();
        }
    }

    private assertAlive(): void {
        if (this.destroyed) throw new Error('this GpuReacher was destroyed');
    }

    destroy(): void {
        this.destroyed = true;
        for (const b of [this.state, this.motors, this.obs]) b.destroy();
    }
}
