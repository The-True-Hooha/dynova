# Benchmark: step time at 1k, 5k and 10k bodies

`examples/bench.html?n=<bodies>` runs the single-threaded release build (`wasm-st`) with no rendering. `e2e/bench.spec.ts` runs it for 1000, 5000 and 10000 bodies in Chromium and prints one JSON line per run. It asserts correctness only (finite transforms, no malformed commands, all spawns drained); it does not assert timings, because they depend on the machine.

## Scene

A cube-shaped pile of `n` bodies (alternating spheres and unit boxes, with a small deterministic jitter) drops from 6 m into a walled arena sized to the pile. The run has 120 "active" ticks and 240 "late" ticks. Setup ticks that drain the spawn commands are not timed.

## Results (Chromium on the development laptop, one run each)

| Bodies | Active avg / p99 (ms) | Late avg / p99 (ms) | Peak contacts | Dropped contacts | Fullest bucket |
|---|---|---|---|---|---|
| 1000 | 3.1 / 6.7 | 6.0 / 7.7 | 3,289 | 0 | 15 |
| 5000 | 17.3 / 42.2 | 35.2 / 44.8 | 17,237 | 0 | 20 |
| 10000 | 36.7 / 89.3 | 73.9 / 91.3 | 35,238 | 0 | 24 |

WebKit gives the same counters (the solver is deterministic) and times within 3%.

## History

| Change | 5000 bodies late avg (ms) | 10000 bodies late avg (ms) |
|---|---|---|
| Baseline | 43.1 | 149.5 |
| Step 17: buckets and contact cap scale with capacity | 41.8 | 90.1 |
| Step 18: bounding-sphere reject before the narrow phase, solver early exit | 35.2 | 73.9 |

Step 18 cut narrow-phase calls 20 times in a dense pile (876k to 41k per step at 10000 bodies, native build) and stops the 12 solver iterations once no contact update in a sweep changes a velocity by more than 1e-4 m/s (a NaN counts as not converged). The bounding-sphere reject leaves the contact set identical. The golden hash changed because the early exit changes the converged velocities slightly, and native, Chromium and WebKit agree on the new value.

## What it shows

1. **60 fps budget (16 ms):** met at 1000 bodies only. 5000 and 10000 bodies in this dense pile still miss it. A native profile at 10000 bodies splits the remaining time roughly into broadphase traversal (27 cells, about 140 candidates per body), solver sweeps, and the rest. Further gains need structural work (a body-array layout that keeps one contact's data together, or running islands in parallel workers), which is not planned yet.
2. **Contacts:** no contact is dropped at any size. The cap is 16 contact points per body of capacity (at least 16,384) and the buffer grows on demand. An aligned box lattice makes about 46 points per body, so such a scene can still hit the cap; `droppedContacts` shows it when it does.
3. **Sleeping:** few bodies fall asleep within 360 ticks (88 of 10000), so the island-sleep saving does not help a freshly dropped mixed pile.

Run it again with `make build-st && pnpm exec playwright test --project=chromium e2e/bench.spec.ts`.
