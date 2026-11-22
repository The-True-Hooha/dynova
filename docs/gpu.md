# GPU backend (optional)

Phase 18 adds `@dynova/gpu`: a WebGPU compute path that steps many copies of one scene, the arm (a static pivot, a dynamic link on a hinge or a slider, with limits and a motor). It is opt-in and separate from the core. The CPU path stays the deterministic reference and none of its hashes or pins changed; the GPU path is not bit-identical to it, across GPUs or to the CPU, and is held to a tolerance instead.

## What it runs

One compute invocation steps one world. Per tick, in 4 substeps: the sleep check, gravity, damping, the hinge rows (three for the anchor point, two for the axis, a motor row, a lower and an upper limit row), warm start, up to 32 solver sweeps with an early exit when the residual is small, a position correction for the axis, then integration. It is a port of the CPU solver for exactly this scene, so it is not a general engine.

`ReacherScene.kind` picks the joint: `'hinge'` (the default; angle in radians) or `'slider'` (offset along the axis in metres; two rows across the axis and three holding the orientation). The slider assumes the bodies start unrotated.

Sleep follows the core's rules for a lone jointed body, judged at the start of each substep: a link that stays under 0.3 m/s (and 0.3 rad/s) and is not still gaining speed for 0.2 s falls asleep, and a motor force above 0 keeps it awake. A sleeper is left untouched and costs almost nothing; only `reset` wakes it. The CPU also wakes a body on every `setJointMotor`, even with a force of 0, so a loop that commands a motor every tick keeps a CPU link awake and not a GPU one; in a scene with `motorMax` 0 send no motor commands to the CPU if you compare them.

Contacts: a scene may carry one static box (`ReacherScene.floor`, with `linkFriction` and `linkRestitution`) that the link collides with in every world. The kernel is a port of the core's contact path for that pair: the box-box manifold (face clipping to at most four points, or one edge-edge point), friction on a disc, restitution above 2 m/s, split-impulse recovery of penetration past the 0.01 m slop, the solve order (lowest point first, alternating), warm starting by where a contact point is, the swept test against the box, and the rule that a deep contact (past 0.02 m) keeps the link awake. The floor can be rotated. It must not overlap the pivot or the link's start: a body spawned inside the floor is not what the CPU runs either.

Not in the GPU path: more than one contact body, contacts between links, spheres, capsules and hulls, particles, ball, fixed and distance joints, more than one link, and the CPU's swept test against the pivot (a link that starts within one substep's travel of the pivot's box is not what the CPU runs).

## API

```ts
import { GpuReacher, REACHER } from '@dynova/gpu';

const adapter = await navigator.gpu?.requestAdapter();
const device = await adapter.requestDevice();
const gpu = await GpuReacher.create(device, 4096);   // REACHER, or a ReacherScene
gpu.reset(kick);        // optional Float32Array, 3 floats a world: the impulse world.force(link, ...) gives
gpu.setMotors(speeds);  // Float32Array, one speed a world, finite and within +-1000
gpu.step(1);            // queues that many ticks
const obs = await gpu.readObs();         // [angle, angular velocity] per world
const { state, obs } = await gpu.read(); // plus 61 floats a world: position, rotation, velocities, warm start, sleep, contact cache
gpu.destroy();
```

- `create` refuses what the CPU core refuses for such a joint (zero inverse mass, negative motor force, zero axis, bad limits, NaN) and world counts outside 1 to 1,048,576, with a `RangeError`.
- A read copies the state as of the ticks queued before the call; steps queued after it do not change the result. Several reads may be in flight.
- A motor speed is a fresh command each tick, so its warm start restarts, as `setJointMotor` every tick does on the CPU.
- WebGPU needs a secure context (`https` or `localhost`). Node has no WebGPU, so the tests run in a browser.

## Tolerance

The GPU is held to these bounds against the CPU worlds (`e2e/gpu.spec.ts`: a driven arm, an arm driven into its limits, an arm kicked at 100 times and driven at 30 rad/s, and a skewed arm with an off-origin pivot, tilted axis, bent gravity, heavier link and asymmetric limits; 64 worlds, 200 ticks, checked at ticks 1, 10, 50, 100 and 200):

| Quantity | Bound | Measured at tick 200 |
| --- | --- | --- |
| Joint angle | 1e-4 rad | 2.9e-6 (Apple M3 Pro), 1.3e-6 (SwiftShader) |
| Joint velocity | 1e-3 rad/s | within bound |
| Link position | 1e-4 m | 1.2e-6 |
| Link rotation | 1e-4 | within bound |

The bounds are absolute. They hold for scenes within a few metres of the origin; a pivot 50 m away reached 5e-4 m in 400 ticks because f32 cancellation in the anchor error grows with distance.

## Speed

`e2e/gpu-bench.spec.ts` runs 4096 worlds for 100 ticks, each tick being what a control loop does: command every motor, step, read every angle and velocity. On an Apple M3 Pro in WebKit the CPU batch took 100 to 125 ms a tick and the GPU 2.4 to 5.7 ms (20 to 50 times faster; it varies from run to run). Most of the GPU time is the round trip of the read, not the compute. Queuing 10 ticks before one read took 0.08 to 0.3 ms a tick, but that holds the command for 10 ticks and is an upper bound, not a comparison.

Chromium here reaches WebGPU only through SwiftShader, a CPU rasteriser (it also needs `--enable-unsafe-webgpu`): it ran 4 to 10 times faster than the CPU batch, which says nothing about a real GPU. The speedup is asserted only when the adapter is not a fallback.

## Limits and what is not verified

- Firefox, x86_64 and GPUs other than the M3 Pro were not run.
- The slider's orientation correction (position correction of its angular rows) and its quaternion sign flip have effects below the tolerance in every scene tried. The slider scenes (`joint=slider`, a skewed one, one with the anchor off the link centre) match the CPU to about 1e-6.
- Sleep: the spin the previous substep's position correction adds (and ignoring it in a tick's first substep) is below the tolerance in every scene tried. A sleep decision can flip a substep for a world within 1e-6 of a threshold (0.3 m/s, 0.2 s, 0.5 m/s squared a second), which moves its pose by up to 1e-3.
- Four solver behaviours have effects below the tolerance in every scene tried, so a change to them would not fail the suite: the hinge-axis position correction (largest effect 4.8e-5 rad), the motor warm-start reset, the solver early exit and the bias on a penetrated limit. The 4 m/s joint bias clamp is checked by the kicked scene (`kick=100`), whose anchor errors reach it.
- Results are not bit-identical across GPUs; do not put a GPU state in a hash, a recording or a snapshot.
- Contacts: the floor scenes (`floor=tip`, `flat`, `slope`, a post, with bounce, sleep and kicks to 40 m/s) match the CPU to about 1e-5. Without the contact warm start a resting slider differed by 7e-4 m, so the warm start is what keeps it inside the bound. Impacts at 120 m/s (`kick=30`) differ by 3e-3 in velocity on the first tick: past about 40 m/s the sweep and the solver sit on discrete decisions that f32 resolves differently. A link resting on a narrow post with penetration near the slop sleeps a substep apart from the CPU in SwiftShader (angle 3e-4 after sleep), so no such scene is in the suite. Mutants caught: no swept test, no restitution, no friction, no penetration recovery, no warm start, the solve order, the deep-contact sleep gate. A mutant that leaves the pseudo-velocity out of the sleep speed survives (it stays below the tolerance). The edge-edge manifold and the more-than-four-points reduction are ported but no scene reaches them (the hinge axis and every floor tilt in the hinge scenes keep the cross axes on face axes). The swept test covers the floor only; the core also sweeps the pivot box. Not checked: more than one floor, a floor the link starts inside.
