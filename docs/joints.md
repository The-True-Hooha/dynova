# Joints

Four joint kinds connect two bodies, at least one of them dynamic: ball, hinge, fixed and slider. Hinges and sliders take limits and a motor.

```ts
const door = world.createJoint({
  kind: JOINT_HINGE, a: frame, b: leaf,
  anchor: [0, 1, 0], axis: [0, 1, 0],
  limits: [-1.2, 0],                       // radians, measured from the pose at creation
  motor: { velocity: 0, maxForce: 0 },
});
world.setJointMotor(door, 2, 50);          // rad/s (or m/s for a slider), max torque (or force)
world.destroyJoint(door);
```

## Behaviour

- Anchor and axis are given in world space at creation; the runtime stores them in each body's frame, so the joint follows the bodies.
- The hinge angle and the slider distance are zero at creation. Limits must contain zero. Hinge limits stay within +-3.1 rad because the angle comes from a polynomial `atan2` on (-pi, pi] (no libm, so every build agrees to the bit); a single-tick impulse that pushes past pi wraps the angle.
- A motor with `maxForce > 0` keeps its bodies awake, since a slow motor can move a body below the sleep speed. `maxForce` 0 switches it off.
- Bodies joined by a joint sleep and wake together. A joint disappears when either body is destroyed; a seek restores the joints of the target tick, and joint ids only grow, so a stale handle is dropped and counted as stale.
- Limit rows stay armed while not reached, allowing an approach speed of gap/dt, so a fast body stops on the bound instead of overshooting.
- Up to 16384 live joints.

## Inspecting and replaying

Joints are part of the state hash, the snapshot and the recorded command log, so a seek over a joint scene is bit-identical. `core/src/determinism.rs` runs a 1230-tick joint scene (a six-link chain, a door with limits, a motor-driven wheel, a motor-driven slider whose target changes mid-run, a welded pair, a joint destroyed by command and a body destroyed under a joint) with a record and seek round trip; both seeks replay joint, motor and force commands. The pinned hashes match native aarch64, native x86_64 (Rosetta), Chromium and WebKit; see `docs/decisions/determinism.md`. A second test steps the same scene for 10000 ticks and checks, every tick, the worst anchor gap of each joint (under 0.2 m for the chain, 0.25 m for the door, under 0.01 m for the wheel and the weld), that the door and the slider reach both of their limits without passing them by more than 0.1, that the slider stays on its rail and that the destroyed joint and the destroyed body's joint are gone. The inspector shows each joint's anchor distance: the error for ball, hinge and fixed joints, the travel for a slider (`docs/inspector.md`).

## Limits of the solver

Joints use the same 12-iteration sequential-impulse loop as contacts, with Baumgarte correction. A world with joints runs 4 sub-steps per tick (`JOINT_SUBSTEPS`), each with its own solver passes, so the joint integration error stays small: a 10-link chain with an end mass 50x the link mass stays within 5 cm of closed. A 2 m door hinged at one end holds within 1 cm at rest. A light body hung from a moving (kinematic) anchor and swung over its pivot (a 0.3 m sphere on a 0.9 m lever, anchor gliding at 2 to 5 m/s) can still stretch its joint by up to a metre. Joint-free worlds do not sub-step, so their step time and results are unchanged. A world with any joint runs the whole step four times, so it costs more per tick: measured natively at release speed with one joint far from a field of resting spheres, 1000 spheres take 1.3 ms without and 2.0 ms with, and 5000 take 9.6 ms without and 14.9 ms with. Busy worlds (nothing asleep) approach 4 times. Baumgarte correction runs on the sub-step, so contact and joint position correction is stronger per tick in a world with joints than in one without.
