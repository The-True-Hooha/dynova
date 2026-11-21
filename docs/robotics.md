# Robotics: controllers on the joint motors

A robot controller is user code that runs once per tick: read where the joints and bodies are, decide, send motor commands. The runtime stays the owner of the world; the controller only reads shared memory and writes commands to the ring, so a recorded run replays exactly.

## The loop

```ts
world.step();                                   // the tick the controller reacts to
const states = world.readJointStates();         // position and rate of every hinge, slider and distance joint
world.driveJoints(drives);                      // PD position control, one motor command per joint
// or world.setJointMotor(joint, speed, maxTorque) for a wheel
```

- `readJointStates()` returns `{ joint, kind, position, velocity }` for every hinge (angle in radians, in (-pi, pi]), slider (offset in metres) and distance joint (stretch in metres; it has no motor, so `driveJoints` refuses it), with the sign that limits and the motor use: a positive motor speed raises `position`. They are the values after the last step; a seek republishes the values of the sought tick. The runtime writes the table after every step inside a sequence word and the reader copies it whole, so it never mixes two ticks.
- `driveJoints(drives)` reads the states once, runs `positionControl` for each `{ joint, target, gains }` and queues the commands. Every command is validated before any is queued, so a bad value queues nothing; a full command ring can still stop it part way. It returns the ids of joints it had no state for.
- `positionControl(state, target, gains)` is the pure part: speed = `kp * error - kd * velocity`, clamped to `maxSpeed`, with `maxForce` as the motor's torque or force limit. A velocity motor holds exactly the speed it is told while the load stays under `maxForce`, so a joint reaches its target without a steady error as the speed goes to zero, up to the solver's own softness (below).
- `solveArm`, `armJoints`, `pursue` (`robotics.ts`) are the arm and path maths the demos use: cyclic coordinate descent for a planar arm within joint limits, and pure pursuit for a differential drive. All pure and deterministic.
- A controller's own state (a trim, the waypoint it is chasing) lives in its code. A seek rewinds the world, not the controller, so after a seek the controller should be re-initialised before it drives live again. The commands it already sent are in the log and replay bit-identically without it.

## The demos

`make dev`, then `/robot.html?scene=arm` (a 3-joint arm; click to move the target) and `/robot.html?scene=car` (a differential-drive robot following a path). `&panel=1` mounts the inspector timeline. Both record from the first step. `e2e/robot.spec.ts` checks in Chromium and WebKit that the arm reaches two targets within 6 and 8 cm, that the robot reaches the end of its path without leaving the floor, that every scrub reproduces the live hash, and that both browsers produce the same final hash (pinned in the spec).

## What the solver needs from a robot model

The joint solver is a sequential-impulse loop with a fixed iteration count (`docs/joints.md`). These are observations from building the two demo scenes and a handful of parameter sets tried in throwaway native tests, not a measured envelope:

- **An arm held in a plane.** Hinges with parallel axes, links in one plane and gravity in that plane held still with anchor gaps near 1 cm. Anchors off the links' plane (links stacked in layers along the hinge axis) used to tilt the arm out of its plane until it came apart. The solver now corrects the axis error without feeding energy back and keeps an 8-link chain planar with anchors up to 1 m off the plane (`core/src/physics/accuracy.rs`), in the one case tested; the demo still keeps its anchors in the plane.
- **Links that touch at a bend fight the joint.** The demo's links stop short of each pivot and are thin, so they only touch past about 115 degrees; its joint limit (2 rad) stays inside that.
- **Wheels:** each wheel is a hinge motor and the floor contact is friction. A 50 percent wheel-speed split drives a circle of the radius the speeds give, and turning in place keeps the chassis on the floor (`core/src/physics/accuracy.rs`). Earlier versions skidded and tumbled above a 25 percent split. The cause was the orientation step, which applied the world-frame angular velocity in the body frame and so turned a spinning wheel on a yawed chassis about the wrong axis; it now applies it in the world frame. The demo path asks for wheel speeds that differ by at least half (checked in `e2e/robot.spec.ts`).
- **The chassis rests on the floor** with no friction; the wheels carry the traction. Frictionless caster spheres fixed to the chassis did not help.

These are limits of the solver, not of the controller API. They are parked in `PLAN.md`.

## Pausing and seeking

The demo's controllers do nothing while the world is paused: a paused runtime still accepts commands, and a motor command accepted there is logged at the head tick, which discards the recorded future after it. A seek rewinds the world but not the controller, so the demo restarts the robot's waypoint from the nearest one when the seek count changes. The arm's controller keeps only a small clamped trim and is left alone. `e2e/robot.spec.ts` scrubs with the controllers running and checks that the recording keeps its end.

## Determinism

`e2e/robot.spec.ts` pins the final state hash of each demo run in the spec and runs in Chromium and WebKit, so those two engines agree with each other and with the pin. Firefox is in `playwright.config.ts` but could not be launched in the development sandbox and is unverified.
