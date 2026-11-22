# GPU backend (optional)

Phase 18 adds `@dynova/gpu`: a WebGPU compute path that steps many copies of one scene, the arm (a static pivot, a dynamic link on a hinge with limits and a motor). It is opt-in and separate from the core. The CPU path stays the deterministic reference and none of its hashes or pins changed; the GPU path is not bit-identical to it, across GPUs or to the CPU, and is held to a tolerance instead.

## What it runs

One compute invocation steps one world. Per tick, in 4 substeps: gravity, damping, the hinge rows (three for the anchor point, two for the axis, a motor row, a lower and an upper limit row), warm start, up to 32 solver sweeps with an early exit when the residual is small, a position correction for the axis, then integration. It is a port of the CPU solver for exactly this scene, so it is not a general engine.

Not in the GPU path: contacts and collision, sleep, particles, other joint kinds, more than one link, and the CPU's swept test against the pivot (a link that starts within one substep's travel of the pivot's box is not what the CPU runs).

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
const { state, obs } = await gpu.read(); // plus 24 floats a world: position, rotation, velocities, warm start
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
- Four solver behaviours have effects below the tolerance in every scene tried, so a change to them would not fail the suite: the hinge-axis position correction (largest effect 4.8e-5 rad), the motor warm-start reset, the solver early exit and the bias on a penetrated limit. The 4 m/s joint bias clamp is checked by the kicked scene (`kick=100`), whose anchor errors reach it.
- Results are not bit-identical across GPUs; do not put a GPU state in a hash, a recording or a snapshot.
