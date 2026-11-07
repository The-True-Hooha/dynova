# Benchmark: step time at 1k, 5k and 10k bodies

`examples/bench.html?n=<bodies>` runs the single-threaded release build (`wasm-st`) with no rendering. `e2e/bench.spec.ts` runs it for 1000, 5000 and 10000 bodies in Chromium and prints one JSON line per run. It asserts correctness only (finite transforms, no malformed commands, all spawns drained); it does not assert timings, because they depend on the machine.

## Scene

A cube-shaped pile of `n` bodies (alternating spheres and unit boxes, with a small deterministic jitter) drops from 6 m into a walled arena sized to the pile. The run has 120 "active" ticks and 240 "late" ticks. Setup ticks that drain the spawn commands are not timed.

## Results (Chromium on the development laptop, one run each)

| Bodies | Active avg / p99 (ms) | Late avg / p99 (ms) | Peak contacts | Dropped contacts | Asleep after 360 ticks |
|---|---|---|---|---|---|
| 1000 | 4.9 / 9.3 | 7.0 / 9.9 | 3289 | 0 | 24 |
| 5000 | 28.8 / 59.0 | 43.1 / 62.7 | 16384 | 61773 | 51 |
| 10000 | 57.2 / 119 | 149.5 / 289 | 16384 | 57040387 | 47 |

WebKit gives the same counters (the solver is deterministic) and times within 3%.

## What it shows

1. **60 fps budget (16 ms):** met at 1000 bodies, missed at 5000 and 10000 in this dense-pile scene.
2. **Contact cap:** `MAX_CONTACTS` is 16384 (`core/src/physics.rs`). At 5000 bodies the pile already reaches the cap and drops contacts every tick, so bodies in a dense pile can interpenetrate. Past the cap the results are not trustworthy.
3. **Broadphase hash:** the bucket table has 4096 buckets. At 10000 bodies the fullest bucket holds 159 bodies (817 buckets used), so the 27-cell query degrades toward quadratic work.
4. **Sleeping:** very few bodies fall asleep within 360 ticks (47 of 10000), so the island-sleep saving does not help a freshly dropped mixed pile.

These are findings, not fixes. Changing the contact cap, the bucket count or the solver is outside the current plan and is logged in `PLAN.md`.

Run it again with `make build-st && pnpm exec playwright test --project=chromium e2e/bench.spec.ts`.
