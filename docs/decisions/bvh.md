# Decision: broad-phase, grid or BVH (Stage 5A)

**Decision: no BVH now. Keep the spatial hash grid and fix the two grid defects the data exposes (Step 23). Revisit when a scene has far more than 1600 static bodies or dynamic bodies of many sizes at once.**

VISION principle 4 says to keep the grid until data shows a problem. The data shows two problems, but both come from specific choices in the grid code, not from the grid itself, and each has a small fix. A BVH is a larger change that the data does not yet require.

## How the data was taken

Four scenes, each run on the native build with optimisation on, sampled four times (state about 20 ticks after the tick shown; the row is the median of 20 steps and of 20 contact-generation passes):

```
CARGO_PROFILE_DEV_OPT_LEVEL=3 CARGO_PROFILE_DEV_DEBUG_ASSERTIONS=false CARGO_PROFILE_DEV_OVERFLOW_CHECKS=false \
  cargo test --lib -- --ignored --nocapture broadphase_report
```

This takes a few minutes. The scenes and the metric live in `core/src/physics/scene_data.rs` and are unit tested; the report itself is `#[ignore]`d, so it does not run in the normal suite. The metric is the one the roadmap names, the share of same-cell body pairs in the busiest 2% of occupied cells. Also reported: narrow-phase calls per awake body (the last contact-generation pass), the time of a step, and the time of a contact-generation pass alone (broad and narrow phase together, run on the state a step leaves behind). Times come from one development laptop; run-to-run differences of a few tenths of a millisecond on the small rows are noise, and wasm is slower. The terrain scene's tiles are about 3 m across against a 2 m cell, and its debris falls into one corner: both choices are deliberate stress, so read that scene as a worst case, not a typical one.

| scene | tick | awake | cell | occupied cells | fullest cell | pairs in top 2% | narrow calls per awake body | step ms | contact generation ms |
|---|---|---|---|---|---|---|---|---|---|
| pile of 5000 | 60 | 5000 | 2.0 | 2655 | 7 | 16% | 2.5 | 2.3 | 2.1 |
| | 200 | 4996 | 2.0 | 2504 | 9 | 19% | 2.5 | 9.0 | 3.3 |
| | 400 | 4998 | 2.0 | 1961 | 16 | 31% | 3.1 | 15.6 | 5.1 |
| | 700 | 4993 | 2.0 | 1000 | 17 | 13% | 4.4 | 24.8 | 8.6 |
| 1600 static terrain tiles + 2000 debris | 60 | 2000 | 2.0 | 153 | 40 | 15% | **1601** | 146 | **147** |
| | 200 | 1999 | 2.0 | 109 | 53 | 13% | **1602** | 171 | **148** |
| | 400 | 1954 | 2.0 | 116 | 44 | 11% | **1603** | 148 | **138** |
| | 700 | 1805 | 2.0 | 143 | 44 | 11% | **1603** | 137 | **129** |
| wall and pyramid (223 boxes) | 60 | 36 | 2.0 | 95 | 4 | 6% | 6.6 | 0.8 | 0.04 |
| | 200 to 700 | 0 | 2.0 | 95 | 4 | 6% | 0 | 0.01 | 0.00 |
| 3000 small spheres + one large dynamic box | 60 | 3001 | **28.4** | 7 | **960** | **39%** | 1.0 | 23 | 22 |
| | 200 | 3001 | 28.4 | 6 | **1190** | **48%** | 1.3 | 30 | 23 |
| | 400 | 3001 | 28.4 | 4 | 801 | 28% | 2.3 | 28 | 23 |
| | 700 | 2615 | 28.4 | 4 | 793 | 28% | 3.8 | 49 | 22 |

## What it shows

1. **Uniform pile, wall and pyramid: the grid is fine.** Two to seven narrow-phase calls per awake body, a fullest cell of 4 to 17, and 6% to 31% of pairs in the busiest cells. Contact generation takes 2 to 9 ms for 5000 bodies; it is most of a step while the pile is still falling (little to solve) and about a third of a dense pile's step, where the solver dominates. The wall and pyramid are asleep by tick 200.
2. **Terrain plus debris: the histogram looks flat and the scene is the slowest.** Only 11% to 15% of pairs sit in the busiest cells, so the roadmap's metric says "no clustering". The cost is elsewhere. Every terrain tile is wider than a cell, so each goes into a list of "big statics", and the contact pass calls the narrow phase for **every awake body against every tile with no distance check first** (`generate_contacts`, the loop over `big_statics`): 1600 calls per body, contact generation 129 to 148 ms of a 137 to 171 ms step. The grid histogram cannot see this, which is why the narrow-call count is reported beside it.
3. **One large dynamic body: the cell size follows the widest dynamic body.** A single 20 m box raises the cell to 28 m, so 3000 small spheres fall into 4 to 7 cells (fullest 790 to 1190) and 28% to 48% of pairs sit in the busiest 2% of cells, the roadmap's own example of a BVH trigger. Narrow-phase calls stay low because the bounding-sphere test rejects early; the cost is walking the chain of about a thousand bodies in each crowded cell: 22 to 23 ms of a 28 to 49 ms step.

## Why not a BVH yet

Both defects are properties of the current grid code:

- the big-static loop has no cheap reject, although the ordinary cell loop does (the bounding-sphere test with `reach`);
- the cell size is set by the widest dynamic body, so one outlier sets it for everyone.

A scratch experiment by the reviewer of this decision (a bounding-sphere reject in the big-static loop, a cap on the dynamic cell width, and wide dynamic bodies tested against everything instead of widening the grid) took terrain contact generation from about 130 ms to 12 to 15 ms and mixed-scale from 22 ms to 1 to 4 ms, with the pile and the wall unchanged. A BVH would also work, but it adds a second broad phase to build, keep in step with moving bodies and test, for scenes the grid can serve with about ten lines. Step 23 makes those changes and re-measures.

## Acceptance for Step 23

- Terrain plus debris: contact generation under 20% of the figures above.
- Mixed scale: contact generation within 2x of the pile of the same body count.
- Pile, wall and pyramid: step time within noise of the figures above, golden hash `4599892428385582693` unchanged, and the pair set still equal to a brute-force search on random scenes, including scenes with wide dynamic bodies and many large statics.

## Revisit the BVH when

- a scene has far more than 1600 static bodies (the big-static loop is linear in them even with a reject); or
- dynamic bodies of many sizes coexist in large numbers, so no single cell size and "wide body" threshold suits them; or
- the browser benchmark, not only native, shows contact generation dominating a step.

Not decided here: the `sweep_statics` continuous-collision scan, which also visits every static for each fast body. It is linear in the static count too and shows up only for fast bodies; it is parked in `PLAN.md`.
