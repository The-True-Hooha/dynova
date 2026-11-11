import { InspectorView, World, layoutFromRuntime, SHAPE_CAPSULE, SHAPE_SPHERE, JOINT_HINGE, armJoints, pursue, solveArm } from '@dynova/sdk';
import type { ArmSpec, Pose, PursuitOptions, V2 } from '@dynova/sdk';
import type { EntityHandle, JointHandle } from '@dynova/sdk';
import { canvas2dAdapter } from '@dynova/canvas2d';
import { mountInspector, type InspectorControls } from '@dynova/inspector';

const params = new URLSearchParams(location.search);
const scene = params.get('scene') === 'car' ? 'car' : 'arm';
const paused = params.has('paused');
const withPanel = params.get('panel') === '1';

const PPU = scene === 'arm' ? 150 : 38;
const ARM: ArmSpec = { lengths: [1.0, 0.8, 0.5], limit: 2.0 };
// Links stop short of each pivot and are thin, so adjacent links only touch past about 115 degrees of bend, beyond the joint limit.
const LINK_GAP = 0.06;
const LINK_HALF_THICKNESS = 0.03;
const ARM_GAINS = { kp: 8, kd: 0.2, maxSpeed: 3, maxForce: 400 };
// A gentle arc: in the scenes tried, a differential drive stayed upright and on course only while the two wheel speeds stayed close, which
// means a turning radius of several metres.
const CAR_PATH: V2[] = [[2, 0.1], [4, 0.4], [6, 0.95], [8, 1.7], [10, 2.7]];
// The widest wheel-speed split this path asks for is about 25 percent (docs/robotics.md).
const CAR: PursuitOptions = { lookahead: 1.6, maxSpeed: 0.7, maxTurn: 0.12, arrive: 0.5, trackHalfWidth: 0.8, wheelRadius: 0.3 };
const WHEEL_TORQUE = 15;
const WHEEL_OFFSET = 0.8;

const yawOf = (t: Float32Array, id: number) => {
    const [qx, qy, qz, qw] = [t[id * 7 + 3], t[id * 7 + 4], t[id * 7 + 5], t[id * 7 + 6]];
    return Math.atan2(2 * (qx * qy + qw * qz), 1 - 2 * (qy * qy + qz * qz));
};

async function run() {
    const glue = await import('../sdk/typescript/wasm-st/dynova_core.js');
    const exports = await glue.default() as unknown as { memory: WebAssembly.Memory };
    const config = { capacity: 64, ringCapacity: 512 };
    const rt = new glue.Runtime(config.capacity, config.ringCapacity);
    rt.enable_physics();
    const world = World.attachSync(config, exports.memory, 'fallback', layoutFromRuntime(rt), rt);
    const scales = new Float32Array(config.capacity * 3).fill(1);
    const styles = new Map<number, { shape?: 'rect' | 'circle'; color: string }>();
    const add = (h: EntityHandle, size: [number, number], color: string, shape: 'rect' | 'circle' = 'rect') => {
        scales.set([size[0], size[1], 1], h.id * 3);
        styles.set(h.id, { shape, color });
        return h;
    };

    const controllers: Array<() => void> = [];
    const resets: Array<() => void> = [];
    const api: Record<string, unknown> = { scene };
    let origin: V2 = [0, 0];

    if (scene === 'arm') {
        world.setGravity(0, -9.81, 0);
        origin = [-0.3, -0.2];
        const base = add(world.spawn({ x: -0.12, y: 0, z: 0, hx: 0.05, hy: 0.2, hz: 0.1, invMass: 0 }), [0.1, 0.4], '#555d6e');
        const links: EntityHandle[] = [];
        const joints: JointHandle[] = [];
        let reach = 0;
        ARM.lengths.forEach((len, i) => {
            const link = add(world.spawn({ x: reach + len / 2, y: 0, z: 0, hx: len / 2 - LINK_GAP, hy: LINK_HALF_THICKNESS, hz: 0.1, invMass: 1, friction: 0 }), [len - 2 * LINK_GAP, 2 * LINK_HALF_THICKNESS], ['#44aaff', '#ffaa44', '#7fe0a0'][i]);
            joints.push(world.createJoint({
                kind: JOINT_HINGE, a: i === 0 ? base : links[i - 1], b: link, anchor: [reach, 0, 0], axis: [0, 0, 1],
                limits: [-ARM.limit, ARM.limit], motor: { velocity: 0, maxForce: ARM_GAINS.maxForce },
            }));
            links.push(link);
            reach += len;
        });
        const marker = add(world.spawn({ x: 1.2, y: 1.0, z: -0.6, shape: SHAPE_SPHERE, hx: 0.08, invMass: 0 }), [0.16, 0.16], '#ff5577', 'circle');
        let target: V2 = [1.2, 1.0];
        let goal = [0, 0, 0];
        const trim = [0, 0, 0];
        const setTarget = (x: number, y: number) => {
            target = [x, y];
            goal = solveArm(ARM, target, goal).angles;
            trim.fill(0);
            world.teleport(marker, x, y, -0.6);
        };
        setTarget(1.2, 1.0);
        // Gravity leaves a small steady error under a velocity motor, so each joint integrates its remaining error into a trim.
        controllers.push(() => {
            const states = new Map(world.readJointStates().map((st) => [st.joint, st]));
            joints.forEach((joint, i) => {
                const st = states.get(joint.id);
                if (st) trim[i] = Math.max(-0.3, Math.min(0.3, trim[i] + 0.04 * (goal[i] - st.position)));
            });
            world.driveJoints(joints.map((joint, i) => ({ joint, target: goal[i] + trim[i], gains: ARM_GAINS })));
        });
        const tip = (): V2 => {
            const t = world.readTransforms();
            const last = links[2].id;
            const c = yawOf(t, last);
            return [t[last * 7] + Math.cos(c) * (ARM.lengths[2] / 2), t[last * 7 + 1] + Math.sin(c) * (ARM.lengths[2] / 2)];
        };
        Object.assign(api, {
            setTarget,
            tip,
            target: () => target,
            goal: () => [...goal],
            error: () => { const p = tip(); return Math.hypot(p[0] - target[0], p[1] - target[1]); },
            solvedTip: () => armJoints(ARM.lengths, goal).pop(),
        });
    } else {
        world.setGravity(0, 0, -9.81);
        origin = [-4, -3];
        add(world.spawn({ x: 0, y: 0, z: -0.5, hx: 40, hy: 40, hz: 0.5, invMass: 0, friction: 1 }), [0, 0]);
        // The chassis slides on the floor (no friction) and the two capsule wheels, axes along y, carry all the traction.
        const chassis = add(world.spawn({ x: 0, y: 0, z: 0.08, hx: 0.6, hy: 0.35, hz: 0.08, invMass: 1, friction: 0 }), [1.2, 0.7], '#44aaff');
        const wheels: JointHandle[] = [];
        for (const side of [1, -1]) {
            const wheel = add(world.spawn({ x: 0, y: side * WHEEL_OFFSET, z: 0.3, shape: SHAPE_CAPSULE, hx: 0.3, hy: 0.1, invMass: 1, friction: 0.6 }), [0.6, 0.2], '#ffaa44');
            wheels.push(world.createJoint({ kind: JOINT_HINGE, a: chassis, b: wheel, anchor: [0, side * WHEEL_OFFSET, 0.3], axis: [0, 1, 0], motor: { velocity: 0, maxForce: WHEEL_TORQUE } }));
        }
        let index = 0;
        let done = false;
        for (const [i, p] of CAR_PATH.entries()) add(world.spawn({ x: p[0], y: p[1], z: -0.4, shape: SHAPE_SPHERE, hx: 0.1, invMass: 0 }), [0.2, 0.2], i === CAR_PATH.length - 1 ? '#ff5577' : '#7fe0a0', 'circle');
        const pose = (): Pose => {
            const t = world.readTransforms();
            return { x: t[chassis.id * 7], y: t[chassis.id * 7 + 1], yaw: yawOf(t, chassis.id) };
        };
        // After a seek the world is somewhere else, so the controller picks its waypoint again from where the robot now is.
        resets.push(() => {
            const p = pose();
            index = CAR_PATH.reduce((best, w, i) => (Math.hypot(w[0] - p.x, w[1] - p.y) < Math.hypot(CAR_PATH[best][0] - p.x, CAR_PATH[best][1] - p.y) ? i : best), 0);
            done = false;
        });
        controllers.push(() => {
            const r = pursue(CAR_PATH, index, pose(), CAR);
            index = r.index;
            done = r.done;
            world.setJointMotor(wheels[0], r.left, WHEEL_TORQUE);
            world.setJointMotor(wheels[1], r.right, WHEEL_TORQUE);
        });
        Object.assign(api, {
            pose,
            path: CAR_PATH,
            done: () => done,
            index: () => index,
            distanceToEnd: () => { const p = pose(); const e = CAR_PATH[CAR_PATH.length - 1]; return Math.hypot(e[0] - p.x, e[1] - p.y); },
        });
    }

    const canvas = document.getElementById('view') as HTMLCanvasElement;
    const ctx = canvas.getContext('2d')!;
    const view = { pixelsPerUnit: PPU, originX: canvas.width / 2 + origin[0] * PPU, originY: canvas.height / 2 - origin[1] * PPU };
    const adapter = canvas2dAdapter(world, ctx, { scales, view, style: (id) => styles.get(id) ?? { color: '#333b4a' } });
    const status = document.getElementById('status')!;
    const inspectorView = new InspectorView(exports.memory, rt.header_ptr());
    if (withPanel) {
        const controls: InspectorControls = {
            recording: () => world.recording, record: (on) => world.record(on), pause: (on) => world.pause(on), seek: (tick) => world.seek(tick), teleport: () => false,
        };
        mountInspector(document.getElementById('panel')!, { view: inspectorView, controls, setEnabled: (on) => world.setInspector(on) });
    }
    world.record(true);

    const draw = () => {
        ctx.clearRect(0, 0, canvas.width, canvas.height);
        adapter.update();
        status.textContent = scene === 'arm'
            ? `tip error ${(api.error as () => number)().toFixed(3)} m`
            : `distance to the last waypoint ${(api.distanceToEnd as () => number)().toFixed(2)} m${(api.done as () => boolean)() ? ' (arrived)' : ''}`;
    };
    // A paused world still drains its ring, and a motor command accepted there would be logged at the head tick and cut off the recorded
    // future, so the controllers sit still while paused. A seek rewinds the world but not the controllers, so the one that keeps a position in its own state (the robot's waypoint) restarts from the new state.
    let seenSeeks = world.recording?.seeks ?? 0;
    const controlTick = () => {
        const rec = world.recording;
        if (rec?.paused) return;
        if (rec && rec.seeks !== seenSeeks) {
            seenSeeks = rec.seeks;
            for (const r of resets) r();
        }
        for (const c of controllers) c();
    };
    // One control tick: step, read the joint states the step published, send the motor commands that act on the next step.
    const step = (n = 1) => {
        for (let i = 0; i < n; i++) {
            world.step();
            controlTick();
        }
        draw();
    };
    if (scene === 'arm') canvas.addEventListener('pointerdown', (e) => {
        const r = canvas.getBoundingClientRect();
        const x = ((e.clientX - r.left) * canvas.width / r.width - view.originX) / PPU;
        const y = (view.originY - (e.clientY - r.top) * canvas.height / r.height) / PPU;
        (api.setTarget as (x: number, y: number) => void)(x, y);
    });
    draw();
    const loop = () => {
        step();
        requestAnimationFrame(loop);
    };
    if (!paused) requestAnimationFrame(loop);

    (window as any).__robot = { ...api, world, step, controllers: controllers.length };
    (window as any).__ready = true;
}

run().catch((e) => {
    (window as any).__error = String(e);
    document.body.append(String(e));
});
