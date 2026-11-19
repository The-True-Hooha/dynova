# Batches of worlds

A `Batch` holds N independent worlds in one wasm memory and steps them with one call (`step_all`). Each world is a whole `Runtime`: its own ring, transform buffers, physics, recorder and hull table. No simulation state is shared, so a world ends in exactly the state it would reach alone. This is tested in Rust (mixed and joint scenes, 0 to 64 threads, the pinned 700-tick hash), in Node against the real wasm, and in Chromium and WebKit.

## Use

```ts
const batch = new glue.Batch(256, 64, 128);   // worlds, capacity per world, ring capacity (power of two)
batch.enable_physics();
const set = new WorldBatch({ capacity: 64, ringCapacity: 128 }, memory, 'fallback', batch);
set.worlds[i].spawn({ ... });                 // each world is an ordinary World
set.step();                                   // never call step() on one world of a batch
```

`config.ringCapacity` must equal the batch's ring capacity; `WorldBatch` throws otherwise, because a larger ring would write into the next world.

## Limits

- Construction is rejected with an error (not a trap) when `worlds * (64 KiB + capacity * 1536 B)` exceeds 512 MiB, when one world exceeds the runtime maximum, or when the ring slots of all worlds exceed 2^20. The figures come from measuring the single-threaded build (about 52 KiB of tables per world, 250 B of columns per body, up to 16 contacts of 68 B per body in a dense scene). They are an estimate, not an allocator limit.
- A `Grow` command in one world is capped at that world's share of the same budget, and a recording gets an equal share of 128 MiB. A rejected grow counts in that world's `malformed` counter.
- The native build can step worlds on several threads (`Batch::step_all_threads`); the wasm export steps them in order on one thread. Browser parallelism means several batches on several workers, which is not built yet.
- The step time each world reports is the batch time divided by the world count.
- Worlds are not isolated from a wasm trap: one trap stops every world, and `WorldBatch` marks all of them failed.
