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

## Plan changes

- 2026-11-04: worker topology = Web Worker + SAB with sync fallback (user approved).
- 2026-11-04: convex hull moved from Stage 2B to Phase 3 Stage 3A; capsule + OBB SAT stay in 2B (user approved). ROADMAP.md 2B/3A text needs matching edit in Step 7.
- 2026-11-04: Playwright cross-browser (Chromium, Firefox, WebKit) from Step 4 (user approved).
