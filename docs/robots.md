# Robot descriptions

`parseUrdf(text)` reads a URDF into a `RobotDesc`; `spawnRobot(world, desc, options)` puts it into a world as bodies and joints; `forwardKinematics(desc, values)` gives every link's frame for joint values, on the host and without a world.

## What is read

- Links with one collision shape each: `box`, `sphere`, `cylinder` (a capsule of the same radius and overall length, ends rounded; a cylinder shorter than twice its radius becomes a sphere-like capsule) and `capsule`. Meshes are dropped with a warning; only the first of several shapes is used.
- Joints: `revolute`, `continuous` (a hinge with no limits), `prismatic` (a slider) and `fixed`, with origin, axis and limits. `floating`, `planar` and `mimic` are not supported; an unsupported type is an error.
- Masses from `<inertial><mass>`. The engine uses one scalar inertia per body, derived from its shape, so inertia tensors and the inertial origin are ignored.
- The tree rules: one root, one parent per link, no loops. Everything else is an error that names the link or joint.

## What spawning does

- Bodies sit at the frame of their collision shape; joint anchors and axes come from the joint frames, at joint value 0.
- A link with no shape is a frame: no body, its mass dropped, and a moving joint into it loses its motion. Each is a warning in `SpawnedRobot.warnings`.
- Bodies with no body above them are the base: static with `fixedBase`, free otherwise.
- Sizes below 1 mm or above 10 km and masses outside 1e-6 to 1e30 kg are clamped to the engine's bounds with a warning. Limits are clamped to include zero and to 3.1 rad.
- `upAxis` (default `'y'`) turns a Z-up description upright for the engine, whose gravity runs along -Y. Pass `'z'` to keep the frames as written.
- If a command cannot be queued part way (ring full, no capacity), the bodies made so far are destroyed and the error is rethrown.

## Checked against

Driven to joint values with position control and gravity off, an imported arm's link frames match `forwardKinematics` within 1 cm and its joint angles within 0.005 rad (`robot.test.ts`, on the real wasm).

## Python

`from dynova import parse_urdf, forward_kinematics, spawn_robot` give the same results as the TypeScript functions for URDF (forward-kinematics frames of the shipped assets are pinned from both languages in `golden_robot_ts.json`; the Python tests also stand the quadruped and the humanoid and drive the arm to forward kinematics). MJCF is not ported.

## Limits

URDF and an MJCF subset, an asset set and a quadruped gait so far (Python and browser loaders for URDF are done; MJCF is TypeScript only). The XML readers (TypeScript's own, Python's ElementTree plus the same limits) refuse DTDs and custom entities and cap the document at 16 MiB, nesting at 64 and elements at 200,000. Python refuses XML namespaces; its warning texts format numbers the Python way (`1e-07`), so compare warnings by meaning, not text; its `up_axis` refuses anything but `'y'` and `'z'`. Joint limits are measured from the spawn pose, which is the zero pose.

## MJCF subset

`parseMjcf(text)` gives the same `RobotDesc`. Read: `compiler` (angle, eulerseq xyz, autolimits), nested `body` with `pos` and `quat`, `axisangle` or `euler`, one `joint` per body (`hinge` and `slide`, with `axis`, `pos`, `range`, `limited`; `freejoint` on a top-level body), `geom` of `sphere`, `box`, `capsule` (size or `fromto`), `cylinder` (a capsule with rounded ends, with the cylinder's own mass) and `plane` (a finite slab: MuJoCo's plane is infinite), `inertial mass`, and worldbody geoms as static scenery.

- Angles are degrees unless `compiler angle="radian"`; slide ranges are metres and are never converted.
- Mass is the `inertial` mass, else the sum of the geoms' masses (`mass`, or `density` x volume, default 1000). Inertia tensors and inertial positions are ignored with a warning.
- A top-level body with no joint is welded to the world; with a `freejoint` it is free.
- A hinge or slide whose parent has no body (the world, or an empty body) hangs on a tiny static anchor made at the joint (`SpawnedRobot.anchors`), with a warning.
- Not supported, as errors: several joints on one body, ball and free-nested joints, meshes (dropped with a warning), tendons, equality constraints and actuators (ignored with a warning). `class` and `childclass` are ignored with a warning, so `<default>` values are not applied.

## Legged robots

`assets/robots/quadruped.urdf` is a 10.6 kg robot with four legs of three joints (`<leg>_abduct`, `<leg>_hip_pitch`, `<leg>_knee`, legs `fl fr hl hr`). Its zero pose is the standing pose, so a controller that holds every joint at 0 stands. `legged.ts` has `standing` and `trot(joints, time, options)`, pure functions from time to joint targets, used with `driveJoints`.

Measured on the real wasm (`legged.test.ts`): holding all joints at 0 it stands 10,000 ticks within 5 cm of where it landed, at its standing height and less than 3 degrees from upright; trotting at 1.5 Hz with a 0.3 rad stride it covers more than 4 m in 25 s with the trunk never more than about 17 degrees from upright. Gains: `kp` 10, `kd` 0.3 (`kd` was seen to chatter between the motor limits at 2, because the derivative term acts on a one-tick-old velocity; keep it under 1), `maxSpeed` 6, `maxForce` 40.

Getting here needed a solver change. With 12 sweeps per sub-step the legs crept under load and the robot sagged and tipped within 10,000 ticks (20 sweeps was not enough either; 28 held). A world with joints now uses up to 32 sweeps (`JOINT_ITERS`, the sweep still stops early when converged); a world without joints is unchanged. The switch is per world, not per island: one joint anywhere gives every contact up to 32 sweeps. A measured worst case, a 2,000-box pile settling in a world with one distant joint, cost about 45 ms per tick against 36 ms; once the pile sleeps the cost is the same. Every determinism hash of a jointed scene moved and was re-pinned.

`assets/robots/humanoid.urdf` is a 37 kg biped: pelvis, torso and head, and per side a hip roll, hip pitch, knee, ankle pitch, shoulder and elbow (12 driven joints), with flat box feet. Holding every joint at 0 it stands 10,000 ticks with the pelvis within 8.6 degrees of upright, at 0.9 to 0.97 m, within 10 cm of where it landed (measured: 0.94 m, about 4 degrees and 4 cm). There is no humanoid walking gait: a biped needs balance feedback, which is not built (parked in PLAN.md).
