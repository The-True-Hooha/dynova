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

Joints use the same 12-iteration sequential-impulse loop as contacts. A world with joints runs 4 sub-steps per tick (`JOINT_SUBSTEPS`), each with its own solver passes, so the joint integration error stays small: a 10-link chain with an end mass 50x the link mass stays within 5 cm of closed, a 2 m door hinged at one end holds within 1 cm at rest, and a 0.3 m sphere on a 0.9 m lever hung from an anchor that glides 1 to 8 m in 5 to 100 ticks (up to 16 m/s) keeps its joint within 5 cm. Joint-free worlds do not sub-step, so their step time and results are unchanged.

- **Position correction.** Contacts and the anchor rows of a joint correct position with Baumgarte; the fraction removed per tick is about the same with or without sub-steps (the per-step fraction is the root that compounds to it; tests check it to within 25 percent). The angular rows that hold orientation (a hinge's axis alignment, the orientation of a fixed joint or slider) correct into pseudo-velocities that move the body and are then discarded. A limit past its bound keeps its correction in the velocity: as a pseudo-rotation it turns the body about its centre instead of its pivot, which opens the anchor and throws the body past the other stop. Putting the axis correction in the real velocity fed energy back through the links of a chain, and a hinge chain whose anchors sat off the links' plane tilted out of it within a few hundred ticks; `core/src/physics/accuracy.rs` holds an 8-link chain with anchors 0, 0.3 and 1 m off the plane and checks it stays planar.
- **Jointed pairs do not collide.** Two bodies held together by a ball, hinge, fixed or slider joint make no contact with each other (they overlap near the anchor by design and a contact there only fights the joint). This includes a static anchor, so a body hinged to a floor or wall is not stopped by it; use a limit. A distance joint (rod or rope) does not switch contacts off, and every other pair, including each jointed body against a third body, collides as usual.
- **Hinge rows.** A hinge's perpendicular rows are fixed in body a, so the warm-started impulses stay in the same directions as the axis turns through any orientation.
- **Flags.** `touching` and `penetrating` report whether a body had any contact (or a deep one) in any sub-step of the tick, as it would in a world stepped once.
- **Cost.** The broad phase is run once per tick and its candidate pairs are reused by the other sub-steps while no body has moved more than 5 cm from where they were found and none has fallen asleep or woken; the contacts themselves are tested every sub-step, so it finds the same contacts as running the broad phase each time (`reusing_the_broad_phase_across_sub_steps_matches_a_fresh_one_in_a_scene_with_fast_and_resting_bodies` checks one mixed scene to the bit). The order of contacts can differ when a body crosses a grid cell between sub-steps, so the results are deterministic but not bit-equal to an uncached run. A fast body outruns the candidates and the broad phase runs again. Measured natively at release speed with one joint far from a field of spheres that are falling or settling (the ignored `jointed_step_cost_report`), 1000 spheres take 0.4 to 0.5 ms without a joint and 0.5 ms with, and 5000 take 1.4 ms without and 2.1 to 2.6 ms with. A world with many fast bodies, or many waking and sleeping each tick, pays for the broad phase more often, up to the full four times.
