# Benchmark: step time at 1k, 5k and 10k bodies

`examples/bench.html?n=<bodies>` runs the single-threaded release build (`wasm-st`) with no rendering. `e2e/bench.spec.ts` runs it for 1000, 5000 and 10000 bodies in Chromium and prints one JSON line per run. It asserts correctness only (finite transforms, no malformed commands, all spawns drained); it does not assert timings, because they depend on the machine.

## Scene

A cube-shaped pile of `n` bodies (alternating spheres and unit boxes, with a small deterministic jitter) drops from 6 m into a walled arena sized to the pile. The run has 120 "active" ticks and 240 "late" ticks. Setup ticks that drain the spawn commands are not timed.

## Results (Chromium on the development laptop, one run each)

| Bodies | Active avg / p99 (ms) | Late avg / p99 (ms) | Peak contacts | Dropped contacts | Fullest bucket |
|---|---|---|---|---|---|
| 1000 | 4.8 / 7.6 | 6.9 / 9.8 | 3,289 | 0 | 15 |
| 5000 | 28.2 / 53.8 | 41.8 / 60.6 | 17,156 | 0 | 20 |
| 10000 | 56.4 / 118.5 | 90.1 / 129.3 | 35,396 | 0 | 24 |

WebKit gives the same counters (the solver is deterministic) and times within 3%.

Before Step 17 the contact cap was a fixed 16,384 and the bucket table a fixed 4,096. From about 5000 bodies the cap dropped contacts every tick (57 million dropped at 10000) and the fullest bucket held 159 bodies at 10000; the late-phase step at 10000 bodies averaged 150 ms. Both limits now scale with capacity (`bucket_count_for`, `max_contacts_for` in `core/src/physics.rs`).

## What it shows

1. **60 fps budget (16 ms):** met at 1000 bodies only. 5000 and 10000 bodies in this dense pile still miss it by a wide margin; the solver cost is now the limit, not the broadphase.
2. **Contacts:** no contact is dropped at any size. The cap is 16 contact points per body of capacity (at least 16,384) and the buffer grows on demand. An aligned box lattice makes about 46 points per body, so such a scene can still hit the cap; the counter `droppedContacts` shows it when it does.
3. **Sleeping:** few bodies fall asleep within 360 ticks (74 of 10000), so the island-sleep saving does not help a freshly dropped mixed pile.

Run it again with `make build-st && pnpm exec playwright test --project=chromium e2e/bench.spec.ts`.
