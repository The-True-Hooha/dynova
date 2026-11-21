"""One hinged arm under gravity: the action is the motor speed, the observation is `[angle, angular velocity, target angle]`, the reward is minus the
distance to the target, and an episode ends when the arm is on the target and nearly still. The same task as `examples/reacherEnv.ts`."""
from __future__ import annotations

from dataclasses import dataclass

from .. import JOINT_HINGE
from ..env import EnvSpec
from ..world import Handle, JointHandle

MAX_FORCE = 20.0
MAX_SPEED = 3.0


@dataclass
class _Reacher:
    link: Handle
    joint: JointHandle
    target: float = 0.0


class ReacherEnv(EnvSpec):
    obs_size = 3
    action_size = 1
    max_steps = 200

    def build(self, world, index, count):
        world.set_gravity(0, -9.81, 0)
        pivot = world.spawn(0, 0, 0, half=(0.1, 0.1, 0.1), inv_mass=0)
        link = world.spawn(0.5, 0, 0, half=(0.5, 0.05, 0.1), inv_mass=1, friction=0)
        joint = world.create_joint(JOINT_HINGE, pivot, link, (0, 0, 0), (0, 0, 1), limits=(-2.5, 2.5), motor=(0.0, MAX_FORCE))
        return _Reacher(link, joint)

    def reset(self, world, index, ctx, rng):
        ctx.target = rng.range(-1.2, 1.2)
        world.force(ctx.link, 0, rng.range(-4, 4), 0)

    def act(self, world, index, ctx, action):
        world.set_joint_motor(ctx.joint, max(-MAX_SPEED, min(MAX_SPEED, float(action[0]))), MAX_FORCE)

    def observe(self, world, index, ctx, out):
        rows = world.read_joint_states()
        match = rows[rows["joint"] == ctx.joint.id]
        out[0] = float(match["position"][0]) if len(match) else 0.0
        out[1] = float(match["velocity"][0]) if len(match) else 0.0
        out[2] = ctx.target

    def reward(self, world, index, ctx, obs):
        return -abs(float(obs[0]) - float(obs[2]))

    def terminated(self, world, index, ctx, obs):
        return abs(float(obs[0]) - float(obs[2])) < 0.05 and abs(float(obs[1])) < 0.1
