# Environments for learning

`BatchEnv` runs many copies of one task, one per world of a batch (`WorldBatch` or `ParallelBatch`), behind `reset` and `step` on flat typed arrays. It is the reinforcement-learning surface of the runtime; the Python bindings (Phase 12C) will wrap the same calls.

```ts
const env = await BatchEnv.create(batch, spec, { seed: 1 });   // builds every world, checkpoints it, resets it
let r = env.result;                                            // first observations
for (;;) {
    r = await env.step(actions);                               // Float32Array, actionSize numbers per world
    // r.obs, r.reward, r.terminated, r.truncated, r.resetting: one row per world
}
```

## What a task is (`EnvSpec<C>`)

| hook | job |
|---|---|
| `build(world, index, count)` | spawn the scene once; what it returns (`C`: handles, parameters) is passed to every other hook of that world |
| `reset(world, index, ctx, rng)` | randomise the start of an episode with commands |
| `act(world, index, ctx, action)` | turn an action into commands |
| `observe(world, index, ctx, out)` | write the observation into `out` |
| `reward(...)`, `terminated(...)` | computed from the observation |

`examples/reacherEnv.ts` is a complete task: one hinged arm, motor speed as the action, `[angle, velocity, target]` as the observation. Hooks read through the bulk calls (`readTransforms`, `readJointStates`) and write commands; none adds a per-entity wasm call to the read path.

## Semantics

- **Reset** is a ring command (`Checkpoint`, `Reset`, tags 18 and 19). `create` pauses every world, runs `build`, takes the checkpoint and unpauses, so the checkpoint is the scene exactly as `build` left it, unmoved, however many commands that took (`build` may queue more than a ring holds). `reset` returns a world to its checkpoint without advancing it, as a seek does. What is restored is the state in the snapshot: bodies, velocities, sleeping, joints and their warm starts, entity ids and generations. What is not: the tick counter, a larger capacity after a `Grow`, the hull table, the pause flag, the malformed and stale counters, published query results. The effect on the world of commands queued before a reset in the same step is undone; commands after it, your `reset` hook, act on the restored world. A reset is refused, and counted in `malformedDropped`, with no checkpoint or while a recording runs; `create` and `reset` throw when that happens.
- **Hooks.** A `reset` hook may only use what `build` returned (entity, joint handles): it may force, teleport, set motors and gravity, but not spawn, destroy or create joints, because the SDK re-syncs its entity tables after the step that applies the reset and the hook runs before it. Hooks may queue as many commands per world per step as the ring takes at once (`ringCapacity`); more throws.
- **Autoreset is NEXT_STEP** (Gymnasium's mode). The step on which an episode ends reports its `terminated` or `truncated` flag and its last observation; the following step resets that world, ignores its action, and reports `resetting = 1`, `reward = 0` and the first observation of the new episode. The other worlds carry on.
- `truncated` means `maxSteps` steps passed without `terminated`. The two flags are never both set.
- **Seeds.** `mulberry32` (integer arithmetic only, so every engine draws the same numbers) seeded by `episodeSeed(seed, world, episode)`. `env.reset(seed)` restarts the episode counters, so a seed replays. World `i` sees the same history whatever the number of worlds beside it and however they are spread over workers; this is tested in Node on the real wasm and, with four real workers against one thread, in Chromium and WebKit.
- **Buffers** are the environment's own and are overwritten by the next `reset` or `step`. Copy what must outlive it.
- **Errors.** A wrong action length, a non-finite action, a seed that is not an integer in [0, 2^32) or a step started while another runs throws and changes nothing. A non-finite observation or reward, a hook that throws, a reset the runtime refused, commands beyond the ring, or a failed world (a worker trap, for example) throws with the world index in the message, and the environment is then unusable: every later call throws, because the worlds hold half of a step. Nothing is silently clamped.

## Measured

256 worlds of the reacher, 60 steps (includes the hooks and one command write per world per step), headless on one machine: Chromium 262 ms on one worker and 88 ms on four (about 58 000 and 175 000 world-steps per second); WebKit 294 ms and 143 ms. Other machines are not measured.

## Not here

Spaces or metadata objects (a `Box` or `Discrete` description), vectorised wrappers, a Python API (12C), rendering of observations (Phase 14).
