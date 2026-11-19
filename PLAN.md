# Dynova Plan: rules, status and backlog

Source of truth for work order. ROADMAP.md defines the stages; this file defines how and in what order the remaining work gets done.

---

## Rules (apply to every step)

1. **Always use the `dod-design` skill.** Phase 0 data-shape rationale before code, Phase 1 implementation, Phase 2 fresh-subagent adversarial review, one fixer pass, then the verification gate. Existing local patterns win over skill defaults.
2. **Write tests for everything.** No behavior change without a test that fails when the behavior is removed. Write the failing test first where practical.
3. **Think outside the box for edge cases.** For each unit of work, list hostile and degenerate inputs before coding and test them. Minimum set to consider:
   - ids: out of range, `u32::MAX`, negative via `Int32Array`, id reused after destroy, generation wrap at 255→0
   - floats: `NaN`, `±Infinity`, `-0.0`, subnormals, huge magnitudes, zero-size shapes, zero or negative `inv_mass`
   - geometry: coincident centers, exact touching, sphere center inside box, deep penetration, bodies exactly on cell borders, negative coordinates
   - ring: full, empty, wraparound at `u32` overflow, unknown tag, truncated slot, producer faster than consumer
   - memory: growth mid-frame, stale views, misaligned offsets, capacity 0 and 1, non-power-of-two ring capacity
   - time: step with no commands, 10k ticks, spawn and destroy in the same tick, destroy then force on the same handle
   - concurrency: torn reads across swap, reader holding a stale buffer, both threads touching the same word
   - determinism: same inputs give bit-identical outputs across runs and (where tested) browsers
4. **Never go out of plan.** Only do what the current step lists. If something outside the plan looks wrong, write it under "Parking lot" at the bottom and keep going. Do not fix it, refactor it, or add features. Changes to the plan need the user's explicit approval and a line in the "Plan changes" log.
5. **Commits are signed by the user alone.** Author and committer are the configured git user (`Davidhero`). Never add `Co-Authored-By`, "Generated with Claude Code", or any Claude attribution to commit messages or PR descriptions.
6. **Commit hygiene.** One logical change per commit. Conventional Commits (`feat:`, `fix:`, `refactor:`, `test:`, `docs:`, `chore:`). Commit only when the step's tests and gate are green. Commit only when the user asks.
7. **Verification gate before declaring any step done:**
   - `cargo test` (host target, see Step 0), `cargo clippy -- -D warnings`
   - `pnpm --filter @dynova/sdk test` and `pnpm --filter @dynova/sdk typecheck`
   - Browser/Playwright checks where the step lists them
   Report failures with the output. Never mark a step done on a red or skipped gate.
8. **Cross-phase invariants (from ROADMAP.md, never regress):** no per-entity WASM call on the read path; the command queue is the only write path; every buffer follows double-buffer + atomic swap.
9. **No decorative comments.** Comment only a non-obvious WHY.
10. **Steps run in order.** No step starts until the previous step's exit check passes.

## Where things stand (2026-11-11)

Every ROADMAP phase is done and committed on `nightly`: 1 to 5 (runtime, physics, Three.js and Canvas2D adapters, inspector and replay, profiling decisions), 6 (joints), 7 (digital twin), 8 (robotics), 9 (CAD) and 10 (consumable packages). ROADMAP.md marks each stage with what was delivered and what was not; the per-feature docs are in `docs/`; the full history of what each step changed is `git log` (commit bodies carry the review findings). The step-by-step plan that used to live here is gone: it described work that is finished.

Open work is the parking lot below plus `NOT_VERIFIED.txt` (local, git-ignored: Firefox, real x86_64 browsers, timings on other machines, bundlers other than Vite and webpack).

## Parking lot

Out-of-plan observations. Do not act on these without approval.

- (Step 4) `TransformBuffers::swap` copies new-front into new-back right after the flip; a reader still holding the old front can see a torn frame. Needs seqlock/epoch or triple buffer. Also O(capacity) memcpy per step; copy dirty ranges only.
- (Step 3) Rust accepts Spawn on a live id with matching generation (silently re-initialises) and on never-allocated ids with gen 0. Needs live-id tracking tied to the TS allocator.
- (Step 3) TS `CommandRing`/`World.spawn` do no range/NaN validation and return a handle even if Rust later drops the command as malformed. TS tail bytes of a reused slot are not zeroed.
- (Step 3) TransformView contract: `qw == 0` (zero quaternion) marks an empty or destroyed slot; renderer must skip it. Document in the unified World API.
- (Step 4) `drain_commands` bound counts accepted commands only; a ring full of malformed slots makes one drain walk the whole ring. Count iterations instead.
- (Step 6.1) `warm_cache` is keyed by body id pair; reusing an id after destroy can inherit the old pair's impulse.
- (low) `Runtime::new` panics on non-power-of-two ring capacity and can overflow `capacity * 7` on wasm32; `TransformBuffers::write_transform` still asserts on out-of-range id (all callers guarded).

- (Step 4) World no longer has `grow()`; Step 4 redesigns growth + `onBufferGrowth` for worker mode (layout is valid today only because all pointers are allocated before `ready`; add a test that every layout offset+length lies inside the initial buffer).
- (Step 4) `maximumPages` 16384 is duplicated between launch.ts and `--max-memory` in .cargo/config.toml; derive from one constant and validate the module minimum.
- (Step 4) e2e gaps: tiny-ring backpressure + destroy/respawn end to end; the "no main-thread wasm" test only patches `WebAssembly.instantiate*`, add `Module`/`Instance`/`compile` and assert the worker ran.
- (Step 4) No worker step/overrun counter in shared memory (needed for the inspector counters).
- (low) Firefox cannot launch under this sandbox; run `pnpm exec playwright test --project=firefox` in a normal terminal.
- (low) `allocatedEntities` is the monotonic id counter, not the live count; document.

- (Step 5/6) Grow allocation failure (huge capacity near the 1 GiB wasm max) traps mid-`grow_to` after the state reset; use `try_reserve` for every vector before mutating and count failure as a rejected command.
- (Step 5/6) Fuzz weakness: all live slots share one pattern, so a stale-but-whole slot or wrong-epoch frame is indistinguishable from a correct one. Use per-frame unique values (tick in x, id^tick in y/z), assert a single tick per frame and monotonic ticks, and model-check every read.
- (low) CommandRing.grow slot zeroing and the JS acquire CAS are not killed by a test (needs a JS-side racing publish/acquire test or a Worker+SAB test); post-read `applyEpoch` late path is untested.
- (low) Header counters (ticks/stale/malformed) are separate stores, so a reader can see tick N with counters from tick N-1.
- (low) "cross-thread view-refresh broadcast" is a pull of the shared epoch by the single main-thread World; a second thread must poll `peekEpoch`. Single consumer only: an array from `readTransforms()` is valid until the next read and until at most two growths.
- (Step 7) Per-browser results to record: Chromium + WebKit verified (fuzz x3 seeds, determinism, plumbing, physics); Firefox unverified in this sandbox.

- (Phase 5 BVH decision) Broadphase cost cliffs: one large dynamic body widens the grid cell for everyone (1000 boxes + one half-50 box = ~500k narrow calls/step); `big_statics` is scanned for every awake body (cost awake x big statics). Neither is needed for the roadmap exit scenes; decide with the bucket histogram (Step 6.6) data.
- (Step 6) Island sleeping replaces the interim "wake sleepers next to an embedded body" rule; speed-only sleep threshold 0.3 m/s is very high; micro-velocity clamp still zeroes all velocity axes of a shallow resting contact; contacts past MAX_CONTACTS (16384) drop silently (add a counter); NaN positions are not quarantined (skip non-finite bodies in step); `eff_mass` clone and `targets`/`contacts` regrow each step.
- (low) Wake timer asymmetry (`-SLEEP_TIME` on one side) was left as is.

- (Step 6 review, LOW) No fallback contact when every clipped box point is dropped (uses body-centre midpoint); parallel capsules get one contact point; island ids are not relabelled when a new contact merges sleeping islands; CCD scales the whole displacement (loses tangential travel near a surface) and sweeps boxes as their thinnest-axis sphere; stored `lt2` has the wrong sign for one frame when a pair's a/b order flips; `bucket_counts` pointer in the header is stale if `enable_physics` runs twice; the edge-edge segment-length mutant (2x) survives (needs a contact at an edge end).
- (Step 6) Single unit-box column above ~20 buckles under sequential impulses; a block solver would be a separate project.

- (Remaining after cleanup) spawn is O(warm_cache) and deactivate O(capacity); `publish` copies the whole buffer each tick; the 8-bit generation aliases after 256 respawns of one id; dynamic-vs-dynamic CCD absent; roadmap determinism hash on the shared (threaded) build not run; Firefox unrun; `enable_physics`/`contact_count`/`sleeping_count` remain wasm exports (setup/read-only); Grow near memory limit relies on the MAX_CAPACITY (1<<18) budget rather than `try_reserve`.

- (Step 9 review, LOW/parked) Hull inertia still uses the AABB box formula (long thin hulls spin too easily); reference/incident tie rule (`+1e-5`) can flip between frames and a 91-body hull pyramid drifts more in z than the box pyramid (0.095 vs 0.0009); hull statics are not swept by CCD; `HullBuild` runs the O(n^3 * n) plane search per commit (about 0.1 ms release each, bounded by the 4096-command drain); a failed hull on the Rust side still consumes its JS id; the decoder cannot report which chunk was dropped, only the commit count mismatch.

- (Step 10, parked) No capsule fit; an oriented box is only produced for exactly planar meshes (other boxes are axis aligned in mesh space); a mirrored instance of one geometry uploads a second identical hull (key uses signed scale); `sample` is O(n k^2); `fitShape` runs `ConvexHull` on all vertices (fine for level props, slow for very dense meshes); `spawnColliders` leaves the shape centre of boxes and spheres at the AABB centre (not the mass centre).

- (Phase 3 closeout, parked) A shared hull cache across `spawnColliders` calls; `ThreeAdapterOptions` is an alias of `AdapterOptions`; skinned and instanced meshes (collider generation) and folder drops; the demo drops its prop set at a scale chosen from the scene bounds, so very non-flat or very large scenes may place props outside any container.

- (Step 20a, parked) A snapshot as a serialized byte format (save-to-file, import via chunked ring commands) is not built; keyframes are in-memory structs inside Rust. After `restore` the published `dropped_contacts` header word and the histogram stay stale until the next step (seek will run inside a step in 20b). `PhysicsWorld::spawn` does not reset `island_of`, so a reused slot can carry a stale island number into `wake` (suspected, not reproduced). Replayed commands will re-count `stale`/`malformed` and `HullCommit` on a registered hull counts malformed after a restore; the 20b state hash must not cover those counters and replay must skip hull commands. The wasm `Teleport` tag has no TS mirror until 20c.
- (Step 20b, parked) The tick counter is `u32` and recording assumes it does not wrap (about 2.2 years at 60 Hz). Only the newest `Seek` in a step replays (earlier ones are superseded, uncounted) to bound replay work; keyframe memory is bounded by a 64 MiB budget that includes the log.
- (Step 21, parked) `sweep_statics` (continuous collision) scans every static for each fast body, so it is linear in the static count like the big-static loop was; it shows only for fast bodies.
- (Step 24, parked) PGS limits: heavy end masses (fixed in Phase 11 by sub-stepping); the warm-start basis of a hinge's and slider's perpendicular rows jumped when the axis crossed |x|=0.57 (fixed in the Phase 11 cleanup: the rows are fixed in body a); the hinge angle wraps at pi; the inspector joint view landed in step 25. The contact friction tangents still switch basis at |nx|=0.57.
- (Step 25, resolved in Phase 11) The 2 m door "sag" came from the test scene, whose post overlapped the door; the solver held it. The post moved (a scene edit, not a solver gain; the limits test still passes with the post removed) and the 10k-tick test now allows 0.01 m. The chain result is from sub-stepping.
- (Step 26, parked) No orientation targets (a light body on a long lever hung from a moving anchor no longer stretches its joint: 5 cm at most over the cases in `accuracy.rs`); `moveKinematic` building cost for 5000 bodies is not measured in the step time.
- (Step 29, resolved in Phase 11 and its cleanup) The differential drive that skidded and tipped above a 15-20 percent split was an orientation-step bug, not the solver rows: `integrate_quaternion` applied the world-frame angular velocity in the body frame. Fixed for every body, which re-pinned every determinism hash (golden, 700-tick, sensor, joint, kinematic, arm, car, assembly). The cleanup then fixed the rest of what the step left open: hinge chains with off-plane anchors tilted out of plane (angular position correction now goes into pseudo-velocities), jointed pairs no longer collide with each other, `touching` and `penetrating` cover every sub-step, Baumgarte removes the same fraction per tick with or without sub-steps, and the broad phase runs once per tick instead of once per sub-step. Not fixed: one more cost, a world with many fast or waking bodies still pays for the broad phase up to four times a tick.
- (Step 30, parked) No command re-bases a running world (shift every body, keep velocities); mates are made from the current pose and ranges are relative to it.
- (Step 31, parked) No import of CAD-native constraints (glTF has no place for them); no solver that assembles overlapping parts.
- (Step 32, parked) Not published to npm; Rollup, esbuild-only, Next.js and yarn untested (Vite production and dev, webpack 5 by hand, npm and pnpm installs tested); no release process.

- (Phase 12A, parked) A `ParallelBatch` has no step timeout, so a worker that hangs or dies without sending an event hangs `step()`. The memory budget and the Grow probe are estimates (a one-block probe, not a reservation); the threaded wasm build was not re-measured. Browser parallelism is one wasm instance per worker, not WASM threads inside one instance. WebKit needs the isolation headers on 304 responses too (dev servers and static hosts that omit them break a page that starts several workers from one script).

- (Phase 12B, parked) A `reset` hook cannot spawn, destroy or create joints (the SDK's entity tables re-sync after the step that applies the reset). `restore` builds a fresh `PhysicsWorld` per reset, so a reset costs about a world's allocation, not just a copy; not measured at large capacities. No observation or action space description, no Python API (12C). Reset leaves the tick counter, a grown capacity and the counters alone.

- (Phase 12C, parked) The Python package has no hull bodies, kinematic batches, raycasts, overlaps, inspector or recording reads yet; wasmtime memory is assumed not to move (the base is re-checked on every access, but a view must not be used after a call that grows memory); worlds of one instance are not thread-safe against a second user thread while `step()` runs (the call raises instead of racing); a PyO3 backend is parked (docs/decisions/python.md). Tested with Python 3.12 only; the code parses under 3.9.
- (Phase 14, parked) A pixel tests every body in the view pyramid, so a scene with hundreds of bodies in view is slow (64 worlds x 64x64 of a 26-body mixed scene took about 280 ms native on one thread); a tile or grid cull is the first remedy. Bodies only, no normals, colour or lens distortion. Depth is measured once per browser on one machine. Python and the SDK render through a batch (a single world is a batch of one).
- (wasmtime-py) Host callbacks that hold a reference to their instance keep its store and memory alive forever (wasmtime keeps them in a global registry); the host object the callbacks use must not point back at the instance.

## Plan changes

Approved changes to the plan and to the wire and header layouts. Delivery notes for each step are in the commit messages.

- 2026-11-04: worker topology = a Web Worker on shared memory with a synchronous fallback behind the same `World` API; Playwright cross-browser (Chromium, Firefox, WebKit) from Step 4; convex hull moved from Stage 2B to Phase 3 Stage 3A, capsule and OBB SAT stay in 2B. Post-Phase-2 cleanup approved (a `Gravity` ring command replaced the Rust-only spawn and gravity exports).
- 2026-11-06 to 2026-11-07: `loadGltfScene` and the `@dynova/three/loaders` entry point (more model formats), the benchmark harness, and the broad-phase and solver performance fixes it led to (Steps 14 to 18) were added on the user's request.
- 2026-11-09: Phase 4B approved with adjustments: the command queue stays the only write path, so snapshots, `Teleport`, `Record`, `Seek` and `Pause` (tags 8 to 11) are ring commands; the header grew to 34 words (paused flag, seek count, live-flag pointer, recording seqlock).
- 2026-11-10: autonomous execution to the end of the roadmap approved (implement, review, fix, commit, next step). Stage 5A decided no BVH (two grid fixes instead) and no fixed-point migration. All three verticals approved in order instead of picking one.
- Phases 6 to 9 changed the wire and header layouts: ring tags 12 to 14 (joints), 15 (kinematic batch) and 16 to 17 (raycast, overlap sphere); the distance joint rides the joint descriptor's axis words; the header grew to 36 words (inspector joint table), 38 (query results) and 41 (joint states). A kinematic sample names the ticks to arrive in rather than a per-tick target. Queries are read-only: they answer into a result buffer and are not logged or replayed.
- 2026-11-11: Phase 10 (consumable packages) added with the user's approval; publishing to npm is out of scope until a separate confirmation, and the licence is MIT (chosen 2026-11-11: anyone may fork, reproduce and contribute).
- 2026-11-11: Phase 11, solver accuracy, added with the user's approval (11A inertia tensors, 11B sub-stepping or more passes, 11C better joint position correction, 11D only if needed). Every step re-pins the golden and determinism hashes and re-measures step time.
- 2026-11-12: Phase 11 closed. 11A, 11C and 11D were not needed: sub-stepping fixed the chain, and the wheeled-robot tumble came from the orientation step applying world angular velocity in the body frame (fixed for all bodies, so every hash was re-pinned on purpose). Order was 11B first because a spike showed inertia tensors did not help the chain.
- 2026-11-12: after Phase 11 the user asked for every open solver item to be fixed before the next phase: the hinge-row basis, off-plane tilt, jointed-pair contacts, the sub-step flags, Baumgarte per tick and the jointed step cost. Hashes of every world with a joint were re-pinned; worlds without joints are bit-identical to before.
- 2026-11-19: Phases 12 to 18 added with the user's approval, to match robot-learning engines such as Alloy: 12 parallel worlds, the TypeScript RL environment API and Python bindings; 13 contact accuracy; 14 camera sensors; 15 URDF/MJCF assets and legged robots; 16 particles, cloth, cables and soft bodies; 17 fluids and sand; 18 an opt-in GPU backend (non-deterministic, the CPU path stays the reference). Order chosen by the user (foundations first). Python is in scope. VISION.md already lists deformables and fluids as real scope and its non-goal on WebGPU covers renderer adapters, not a compute backend. The Phase 13 contact change will re-pin hashes.
- 2026-11-19: Phase 12A closed with the four limits found at its first review resolved: worlds can spread over workers (one wasm instance and one shared memory per worker, so a trap fails one worker's worlds), each world records its own step time (`Batch.step_world`), and construction and Grow ask the allocator before reallocating. Not a wire or header layout change.
- 2026-11-19: Phase 12B added two ring commands, `Checkpoint` (tag 18) and `Reset` (tag 19), both with no payload, so that an environment can return a world to its start through the command queue (the only write path). No header change. Neither command is logged or replayed; a reset is refused while a recording runs.
- 2026-11-19: Phase 12C starts with the spike's decision (docs/decisions/python.md): the Python package hosts the shipped wasm with `wasmtime` behind a small backend interface; a native PyO3 backend is parked until a measured need. The package lives in `python/`.
- 2026-11-19: Phase 12C starts with the spike's decision (docs/decisions/python.md): the Python package hosts the shipped wasm with `wasmtime` behind a small backend interface; a native PyO3 backend is parked until a measured need. The package lives in `python/`.
- 2026-11-19: Phase 12 closed. 12C delivered the Python package (python/, docs/python.md): a wasmtime host of the shipped wasm, `Batch`, `World` and `BatchEnv` with the TypeScript semantics and numbers, a wheel that carries the wasm and a clean-virtualenv install test. No ring, header or core change. Python goldens are pinned from both languages.
- 2026-11-19: Phase 13 changed the contact solver for every world (sub-steps, circular friction cone, position-matched warm starts, bottom-up order, selective damping, rest by acceleration), so every determinism hash moved, jointless worlds included. `WarmEntry` grew (position, normal, first body), which changes the snapshot hash and size. Not a wire or header change.
- 2026-11-19: Phase 14 added camera sensors with no ring, header, snapshot or state change: a read-only render that takes intrinsics (no `tan`), written as `Batch.render_all` and an image buffer per batch. Two readings of the roadmap, for the user to confirm: there is no spatial grid in the raycast, so a pixel tests the bodies whose bounding spheres are in the view pyramid, not a grid; and "colour-id and segmentation" are one body-id image plus a host-side colouring helper, not a second image.
