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

## Limits of the solver

Joints use the same 12-iteration sequential-impulse loop as contacts, with Baumgarte correction. Chains of equal masses hold: 10 links stay within 0.15 m of closed over 2000 ticks, 20 within about 0.1 m, 30 drift by about 0.5 m. A heavy end mass breaks long chains (an end mass 50x the link mass pulls a 10-link chain apart). Keep mass ratios under about 10 across a chain, or shorten it.
