# Batches of worlds

A `Batch` holds N independent worlds and steps them with one call. Each world is a whole `Runtime`: its own ring, transform buffers, physics, recorder and hull table. No simulation state is shared, so a world ends in exactly the state it would reach alone. This is tested in Rust (mixed and joint scenes, 0 to 64 threads, the pinned 700-tick hash), in Node against the real wasm, and in Chromium and WebKit with real workers.

There are two ways to run one in a page.

## `WorldBatch`: one wasm instance, one thread

```ts
const batch = new glue.Batch(256, 64, 128);   // worlds, capacity per world, ring capacity (power of two)
batch.enable_physics();
const set = new WorldBatch({ capacity: 64, ringCapacity: 128 }, memory, 'fallback', batch);
set.worlds[i].spawn({ ... });                 // each world is an ordinary World
set.step();                                   // never call step() on one world of a batch
```

`step_all` runs the worlds in order on the calling thread. Works without shared memory. A wasm trap stops every world, and `WorldBatch` marks all of them failed. The step time each world records is the batch time divided by the world count.

## `ParallelBatch`: worlds spread over workers

```ts
import { launchBatch } from '@dynova/sdk/web';
const batch = await launchBatch({ config: { capacity: 64, ringCapacity: 128 }, worlds: 256, workers: 4, physics: true });
batch.worlds[i].spawn({ ... });
const failed = await batch.step();            // indices of worlds whose worker has failed; [] when all is well
batch.dispose();
```

- Worlds are split into contiguous shares, as even as possible (`partition`). Each worker owns its own wasm instance and its own shared memory, and steps its share one world at a time, so each world records its own measured step time.
- Needs a cross-origin isolated page. Without `SharedArrayBuffer` it throws; use `WorldBatch` then.
- A trap, an error message or a worker error event fails only the worlds that worker holds, whether it arrives during a step or between steps. Their `failure` is set and any command on them throws; the other workers keep stepping. `step()` rejects once no worker is left. A step reply with the wrong number of times counts as an error. `dispose()` during a step resolves that step, with every world listed as failed.
- Measured in headless Chromium and WebKit on one machine (11 and 8 cores), 64 worlds of 300 falling and colliding bodies, 40 ticks: 1 worker 521 ms and 458 ms, 4 workers 155 ms and 209 ms. Timings on other machines are not measured.
- A worker that hangs or dies without sending any event hangs `step()`; there is no timeout.
- Both the page and the worker use the world's ring through shared memory: the page writes commands at any time, the worker drains them at the start of that world's step.

Both need `config.ringCapacity` to equal the ring the batch built; a mismatch throws, because a larger ring would write into the next world.

## Memory limits

- A batch is rejected with an error (not a trap) when `worlds * (64 KiB + capacity * 1536 B)` exceeds 512 MiB, when one world exceeds the runtime maximum, or when the ring slots of all worlds exceed 2^20. With `ParallelBatch` each worker checks its own share, so the whole batch can be larger.
- The figures come from measuring the single-threaded build (about 52 KiB of tables per world, 250 B of columns per body, up to 16 contacts of 68 B per body in a dense scene). The budget is an estimate; what is enforced is a probe: the batch asks the allocator, in one block, for its whole estimated size before building any world, and a `Grow` command asks for 500 B per slot of the new capacity (new transform buffers, the older generations kept, columns copied while resizing). A refusal becomes an error, or a rejected command counted as malformed, instead of a wasm trap. A probe checks that one block of that size exists now: it does not reserve it, the estimate is not validated against real use, and a heap fragmented into small blocks can refuse a batch that would fit. Whether a `Grow` near the memory limit is accepted therefore depends on the host's memory state, not only on the command stream; Grow is not logged or replayed, so seeking is not affected.
- A `Grow` in one world is capped at that world's share of the budget. A recording gets an equal share of 128 MiB.
- Native only: `Batch::step_all_threads` steps worlds on several threads with the same result for any thread count.

## Pages that start several workers

WebKit refuses a module worker whose revalidated script response (a 304) lacks the cross-origin-embedder-policy header. A dev server that adds the headers only to full responses breaks a page that starts several workers from one script. Send the isolation headers on every response; the example and consumer apps do this in their Vite config.
