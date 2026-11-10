# Recording, seek and replay

The runtime can record itself and jump to any recorded tick. Everything goes through the command ring: the SDK, the inspector and your own code never write simulation memory.

```ts
world.record(true);        // start keeping keyframes and a command log (needs the physics module)
// ... run, spawn, force, teleport ...
world.pause(true);         // stop the clock; commands still apply
const r = world.recording; // { on, paused, tick, start, end, keyframes, hash, seeks }
world.seek(r.start + 120); // the world becomes the recorded state at that tick
world.pause(false);        // resume: live steps from here fork the timeline
```

## Commands

| Command | Tag | Effect |
|---|---|---|
| `Teleport` | 8 | Moves one body, clears its velocity, wakes sleepers it leaves or lands on. |
| `Record` | 9 | On: recording starts at the end of the step that read it. Off: keyframes and log are freed. |
| `Seek` | 10 | Restores the nearest earlier keyframe and replays the log up to the tick. If several `Seek` commands arrive in one step only the newest replays (a scrubber can send many); the earlier ones are superseded without being counted. |
| `Pause` | 11 | A paused runtime drains and applies commands but runs no physics and publishes no new tick. |

## What a seek reproduces

Seeking to tick T gives a state bit-identical (same build, same browser) to the live run at T: the state hash and the published transform frame match, tick for tick. `e2e/replay.spec.ts` records 600 ticks that include a mouse drag, then scrubs backward and forward through the panel and compares every landing with the live run.

- The recorded state at T is the state **before** the commands applied at T. A paused edit at T is discarded by `Seek(T)`; a live step commits it.
- A live step taken after a seek **forks**: the recorded future from that tick on is replaced. A rejected seek while recording (outside the range) changes nothing and does not advance the world; a seek without a recording is counted as malformed and the step runs as usual.
- Only commands the runtime accepted are logged, so a replay never accepts something live rejected (a spawn of a hull that was not uploaded yet, or an id past the capacity).
- Hull uploads, `Grow` and the inspector switch are not logged: hulls persist outside a snapshot and are never removed, and capacity never shrinks. The state hash skips never-used slots, so it does not depend on the capacity.

## Keyframes and memory

A keyframe is taken every 60 ticks, after 4096 logged commands, and when a paused world has piled up 4096 commands (a long drag). A keyframe holds every column `step` reads from an earlier step: transforms, velocities, sleep timers, island ids, the previous contacts and the warm-start cache, plus the entity generation and liveness tables. The store is capped at 64 MiB including the log; the oldest keyframes go first and `start` moves forward. A keyframe costs about 100 bytes per entity slot plus about 70 bytes per contact of the previous step (a settled, sleeping pile has none), so the cap bounds the recording length rather than guaranteeing a duration.

## Header words

`word 24` recording on, `25` first and `26` last seekable tick, `27`/`28` state hash (low/high), `29` keyframe count, `30` paused, `31` seek count, `32` address of the per-entity live bytes, `33` seqlock around words 24–29. `World.recording` reads them under the seqlock and returns the hash as a `bigint`.

## Handles after a seek

A seek can remove entities that existed (spawned after the tick) and bring back ones that did not. Once the runtime has drained every queued command, `World` rebuilds its id allocator from the runtime's generation and liveness tables: handles for entities that no longer exist go stale and their ids come back. Pause before seeking; a client whose command queue never drains never resyncs.

## Limits

- Bit-identical means the same build in the same browser. The runtime makes no cross-browser claim.
- Recording needs the physics module; without it `Record` is counted as malformed.
- There is no byte-format export or import of a snapshot (save to file). Keyframes live in memory inside the runtime.
- The tick counter is 32-bit and recording assumes it does not wrap (about 2.2 years at 60 Hz).
