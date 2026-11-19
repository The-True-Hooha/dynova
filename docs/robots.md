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

## Limits

URDF and an MJCF subset so far (the asset set, legged controllers and the browser and Python loaders are the rest of phase 15). The XML reader refuses DTDs and custom entities, caps the document at 16 MiB, nesting at 64 and elements at 200,000. Joint limits are measured from the spawn pose, which is the zero pose.

## MJCF subset

`parseMjcf(text)` gives the same `RobotDesc`. Read: `compiler` (angle, eulerseq xyz, autolimits), nested `body` with `pos` and `quat`, `axisangle` or `euler`, one `joint` per body (`hinge` and `slide`, with `axis`, `pos`, `range`, `limited`; `freejoint` on a top-level body), `geom` of `sphere`, `box`, `capsule` (size or `fromto`), `cylinder` (a capsule with rounded ends, with the cylinder's own mass) and `plane` (a finite slab: MuJoCo's plane is infinite), `inertial mass`, and worldbody geoms as static scenery.

- Angles are degrees unless `compiler angle="radian"`; slide ranges are metres and are never converted.
- Mass is the `inertial` mass, else the sum of the geoms' masses (`mass`, or `density` x volume, default 1000). Inertia tensors and inertial positions are ignored with a warning.
- A top-level body with no joint is welded to the world; with a `freejoint` it is free.
- A hinge or slide whose parent has no body (the world, or an empty body) hangs on a tiny static anchor made at the joint (`SpawnedRobot.anchors`), with a warning.
- Not supported, as errors: several joints on one body, ball and free-nested joints, meshes (dropped with a warning), tendons, equality constraints and actuators (ignored with a warning). `class` and `childclass` are ignored with a warning, so `<default>` values are not applied.
