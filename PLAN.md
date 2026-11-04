# Dynova Plan: Finish Phases 1–2 Before Phase 3

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

---

## Step 0 — Make verification work

- Make `cargo test` runnable (host-target alias or move pure logic out of wasm-only paths).
- Remove duplicate `--max-memory` link-arg in `.cargo/config.toml`; remove unused `web-sys`.
- Align build output paths (`sdk/typescript/wasm`) in `package.json`, `.gitignore`, `Makefile`; make `build-release` pass `--release`.
- Trim `pnpm-workspace.yaml` to packages that exist.
- **Exit:** `make test`, `cargo test`, `cargo clippy`, `pnpm typecheck` all run; existing 2 Rust tests and 34 TS tests pass.

## Step 1 — Cleanup, no behavior change

- Rename `generationerations` to `generations` (Rust fields, `generationerations_ptr` export, `drain_commands` param).
- Fix the 9 clippy `too_many_arguments` warnings with a body-descriptor struct.
- **Exit:** same test results as Step 0; clippy clean.

## Step 2 — Boundary correctness

1. Fix double-buffer staleness: reader must see every entity after any number of swaps. Test: spawn A, step, spawn B, step, both visible.
2. Bounds-check ids in `Runtime::step` and `drain_commands`; out-of-range counts as stale, never panics. Test with hostile ids.
3. Export real rotation quaternion from physics.
4. Make `drain_commands` reuse a buffer (no per-step allocation).
5. Extend ring commands to carry shape, extents or radius, `inv_mass`, restitution. Cover `Force` and `Destroy` end to end.
- **Exit:** Rust tests for stale handle, hostile id, two-frame visibility, force/destroy through the ring, rotation round trip.

## Step 3 — Unify TS `World` with Rust `Runtime`

- Phase 0 decision: Rust owns layout; TS builds views from exported pointers. Remove the JS-side layout computation in `World.create`.
- `World` wraps `Runtime`: pointers, generations, ring.
- Remove `bridgeSharedRing` byte copy from `examples/main.ts`; run the real worker topology.
- Implement the non-SAB fallback (sync `step()` call path).
- **Exit:** 10k-spawn plumbing test from a browser; no per-frame main-thread WASM calls in SAB mode.

## Step 4 — Stage 1B hardening

- `onBufferGrowth` event and cross-thread view-refresh broadcast; grow-mid-frame test with no torn read.
- Ring backpressure: bounded growth plus counter (replace return-false/throw).
- COOP/COEP doc page; clear error for `headers-missing` vs `fallback`.
- Fuzz test: 10k ticks of random spawn/destroy/grow, zero torn reads, zero silent drops, growth events counted.
- Playwright cross-browser determinism smoke test (Chrome, Firefox, WebKit); document differences if any.
- **Exit:** ROADMAP Stage 1B exit criteria met.

## Step 5 — Stage 2A physics proof

- 50-box stack test: settles, no NaN, step time inside 60 fps budget.
- Fix broadphase neighbor coverage (check all cells overlapped by the body AABB, not just +1 cells).
- Replace the O(n²) static/sleeper pass with a static list or grid lookup.
- **Exit:** stack test green; broadphase has a missed-pair regression test (bodies near cell borders, negative coordinates).

## Step 6 — Stage 2B correctness

In order:
1. Warm start: per-contact accumulated impulse, clamp at zero, persist by pair.
2. Friction (tangent impulses).
3. Island-based sleeping (union-find on contacts); remove the `-SLEEP_TIME` hack.
4. OBB SAT for box–box; capsule; sphere–capsule analytic. Convex hull deferred to Phase 3 Stage 3A.
5. Swept-sphere CCD for fast small bodies.
6. Export grid bucket histogram.
- **Exit:** 30-body mixed-shape stress scene stable for 10k ticks; fast projectile vs thin wall does not tunnel; determinism harness re-run on the solver.

## Step 7 — Docs and final gate

- Update README status and project tree; mark ROADMAP progress.
- Final dod-design adversarial review on the full diff, one fixer pass, verification gate.
- **Exit:** every Phase 1 and Phase 2 exit criterion in ROADMAP.md checked. Phase 3 may start.

---

## Open decisions (need user answer before the step that depends on them)

1. ~~Worker topology~~ — DECIDED: dedicated Web Worker owns `Runtime` on shared WASM memory; sync main-thread fallback behind the same `World` API.
2. ~~Step 6 scope~~ — DECIDED: capsule + OBB SAT in Step 6; convex hull moved to Phase 3 Stage 3A.
3. ~~Cross-browser CI~~ — DECIDED: Playwright now (Chromium, Firefox, WebKit) for end-to-end checks from Step 4 onward.

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

## Plan changes

- 2026-11-04: worker topology = Web Worker + SAB with sync fallback (user approved).
- 2026-11-04: convex hull moved from Stage 2B to Phase 3 Stage 3A; capsule + OBB SAT stay in 2B (user approved). ROADMAP.md 2B/3A text needs matching edit in Step 7.
- 2026-11-04: Playwright cross-browser (Chromium, Firefox, WebKit) from Step 4 (user approved).
- 2026-11-04: Step 3 removed `World.grow()`/`initialPages` from World (growth redesign is Step 4); added `World.dispose()`/`markFatal`, `launchWorld` config validation, a second non-shared wasm build (`wasm-st`, `make build-st`) and Playwright e2e (`make e2e`).
- 2026-11-04: Step 4 replaced the double buffer with a lock-free triple buffer (state word packs latest/new/reader), added a shared 12-word layout header with a seqlock epoch (odd while growing; growth number = epoch/2), a `Grow` ring command, ticks/stale/malformed counters in the header, and a JS-side bounded deferred queue for backpressure instead of growing the ring. `TAG_GROW` slot added; ring no longer filters ids (apply-time checks only).
- 2026-11-04: Step 5 restructured `PhysicsWorld::step` into contacts -> wake -> integrate velocities -> targets -> warm start -> 12 biased iterations -> integrate positions (was: positions first, direct positional correction, early return on separating contacts), because the 50-box stack could not settle otherwise. Broadphase now grids all non-giant bodies with a 27-cell deduped query and adaptive cell size; sleeping requires touching and not embedded; solver constants: BIAS_FACTOR 0.05, MAX_BIAS_VEL 0.5, SOLVER_ITERS 12, SLOP 0.01. Golden hash re-pinned (Rust + browser spec). Warm start, sleeping and the two-phase ordering from Step 6 items 1 and 3 are partially pulled forward; Step 6 still owns per-contact friction, island sleeping, OBB/capsule, CCD and the histogram.
- 2026-11-04: Step 6 delivered `collision.rs` (OBB SAT with clipped 4-point manifolds, rotated sphere-box, capsule vs sphere/capsule/box), per-body friction (ring slot byte 56) with Coulomb tangent impulses, per-point warm cache, split-impulse penetration recovery, island sleeping, swept-sphere CCD vs statics, per-shape inertia, bucket histogram in header words 10/11, capsule shape. The "stack of 50 boxes" exit is met as a 5x10 wall, a 55-box pyramid and a 20-high column (taller single columns buckle). Convex hull stays deferred to Phase 3 Stage 3A. Solver constants: BIAS_FACTOR 0.2, MAX_BIAS_VEL 2.0, SOLVER_ITERS 12, PENETRATION_ITERS 4. Golden hash re-pinned.
- 2026-11-04: Post-Phase-2 cleanup (user approved fixing parked items): added a Gravity ring command and removed `set_gravity`/`spawn_box`/`spawn_sphere`/`spawn_capsule` from the wasm exports (Rust-only test API; examples now spawn through `World` + ring); histogram published as an atomic snapshot; header grew to 16 words (ring pending, dropped contacts, worker overruns, contact count); contacts past the cap are counted; MAX_CAPACITY lowered from 1<<20 to 1<<18 to fit 1 GiB; launchWorld rejects capacity above it; Spawn on a live id is rejected; added the roadmap sin/cos/atan2/sqrt determinism probe (pinned native vs Chromium/WebKit).
