import { JOINT_HINGE, type EnvSpec, type EntityHandle, type JointHandle, type World } from '../sdk/typescript/src/index.js';

interface Reacher { link: EntityHandle; joint: JointHandle; target: number }

const MAX_FORCE = 20;
const MAX_SPEED = 3;

/**
 * One hinged arm under gravity. The action is the motor speed, the observation is `[angle, angular velocity, target angle]`, the reward is
 * minus the distance to the target, and an episode ends when the arm is on the target and nearly still. Each episode picks a new target and kicks the arm.
 */
export const reacherEnv: EnvSpec<Reacher> = {
    obsSize: 3,
    actionSize: 1,
    maxSteps: 200,
    build(world: World): Reacher {
        world.setGravity(0, -9.81, 0);
        const pivot = world.spawn({ x: 0, y: 0, z: 0, hx: 0.1, hy: 0.1, hz: 0.1, invMass: 0 });
        const link = world.spawn({ x: 0.5, y: 0, z: 0, hx: 0.5, hy: 0.05, hz: 0.1, invMass: 1, friction: 0 });
        const joint = world.createJoint({ kind: JOINT_HINGE, a: pivot, b: link, anchor: [0, 0, 0], axis: [0, 0, 1], limits: [-2.5, 2.5], motor: { velocity: 0, maxForce: MAX_FORCE } });
        return { link, joint, target: 0 };
    },
    reset(world, _i, ctx, rng) {
        ctx.target = rng.range(-1.2, 1.2);
        world.force(ctx.link, 0, rng.range(-4, 4), 0);
    },
    act(world, _i, ctx, action) {
        world.setJointMotor(ctx.joint, Math.max(-MAX_SPEED, Math.min(MAX_SPEED, action[0])), MAX_FORCE);
    },
    observe(world, _i, ctx, out) {
        const state = world.readJointStates().find((s) => s.joint === ctx.joint.id);
        out[0] = state?.position ?? 0;
        out[1] = state?.velocity ?? 0;
        out[2] = ctx.target;
    },
    reward: (_w, _i, _c, obs) => -Math.abs(obs[0] - obs[2]),
    terminated: (_w, _i, _c, obs) => Math.abs(obs[0] - obs[2]) < 0.05 && Math.abs(obs[1]) < 0.1,
};
