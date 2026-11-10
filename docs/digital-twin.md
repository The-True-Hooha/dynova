# Digital twin: kinematic bodies driven by telemetry

A digital twin mirrors tracked things (vehicles, machines, people) from an external data stream. Dynova models a tracked thing as a static body that glides to each telemetry sample. Dynamic bodies around it react to it: a moving platform carries what rests on it, a moving wall pushes what is in front of it, and a body hung from it by a joint follows.

## API

```ts
const handles = tracked.map((t) => world.spawn({ x: t.x, y: t.y, z: t.z, hx: 0.5, hy: 0.5, hz: 0.5, invMass: 0 }));

// Each telemetry frame, for every entity that reported:
world.moveKinematic(frame.map((s, i) => ({ handle: handles[i], x: s.x, y: s.y, z: s.z, ticks: 12 })));
```

- A body is kinematic exactly while it has an unfinished target. It must be static (`invMass: 0`); a target on a dynamic body is rejected and counted in `runtimeCounters.malformedDropped`.
- `ticks` (an integer, 1 to 65535) is how long the body takes to arrive. It moves at constant speed and lands exactly on the sample, so rounding never builds up over a long stream. Pick it from the telemetry interval: 12 ticks for a 5 Hz stream at 60 Hz simulation. A body whose next sample arrives before it has arrived is redirected from where it is.
- Orientation is not driven in this version.
- `teleport`, `destroy` and a respawn of the same id end the glide and zero the velocity.
- A batch is checked before anything is queued, and it is queued whole or refused (`world.backpressureDrops` counts the slots refused). The runtime applies at most 4096 slots per step, so a batch larger than that takes effect over more than one tick. A stale handle or a bad value throws. The runtime counts a sample for a body that died in the meantime as stale and skips it, and the other samples in the same slot still apply.

## How it works

- Wire: command tag 15 (`KinematicBatch`). A 64-byte slot carries 1 to 3 samples of 20 bytes (`id`, `generation`, `ticks`, `x`, `y`, `z`); a bulk update is a run of slots. `docs/` and the cross-language fixture in both test suites pin the layout.
- State: one row per moving body (`id`, ticks left, per-tick step, target), sorted by id. A scene with no moving body skips the kinematic work (one empty-table check per step) and hashes as it did before. The rows are captured in keyframes, restored on a seek and included in the state hash.
- Each step sets the body's velocity from the step, so contacts and joints see a real velocity, then advances its position after the solver. A moving body wakes sleeping bodies it touches or is jointed to, because a static body never generates contacts of its own. The check runs before the solver (so contacts apply that tick) and again after the move, so a body that lands on a sleeper in one tick wakes it instead of embedding in it.
- Recording: the batch is logged as one command. If some of its samples were rejected, replay rejects the same ones; replays do not re-count them.

## Cost

Measured on Apple silicon: 5000 moving bodies, one sample each every 12 ticks, mean step 1.8 ms in Chromium and WebKit (`e2e/telemetry.spec.ts` prints it and asserts only that it stays under 16 ms; release wasm); about 1.0 ms natively in a one-off run that is not kept as a test. The JavaScript cost of building 1667 slot writes per sample frame is on top of that and is not in the step time.

## Limits

- A moving body wider than the grid cell scans every body slot each tick it moves, so many wide tracked bodies in a large world are costly.
- A very short `ticks` over a long distance gives a very high speed; there is no clamp.
- Joints to a kinematic anchor inherit the solver's limits (`docs/joints.md`): a light body on a long lever swung hard by a moving anchor stretches its joint.
- No interpolation between samples beyond the constant-speed glide, no extrapolation when samples stop, and no orientation.
