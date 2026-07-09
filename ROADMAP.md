# Dynova Roadmap

Each phase split into 2 stages: A = make it work, B = make it not fall over. No stage starts before the prior stage's exit criteria are met.

---

## Phase 0 — Toolchain & Known Pitfalls

Not a build phase, a setup checklist. Do this before Stage 1A, not during.

- **Nightly Rust required.** `std` w/ atomics isn't on stable yet.
  ```toml
  # .cargo/config.toml
  [build]
  target = "wasm32-unknown-unknown"
  rustflags = ["-C", "target-feature=+atomics,+bulk-memory,+mutable-globals"]
  [unstable]
  build-std = ["panic_abort", "std"]
  ```
  ```bash
  rustup toolchain install nightly
  rustup component add rust-src --toolchain nightly
  cargo +nightly build --target wasm32-unknown-unknown -Z build-std=panic_abort,std
  ```
  Don't budget on this landing on stable mid-project. It's been "soon" for ~3 years.

- **Shared WASM memory needs a hard `maximum` up front**, and growth invalidates every thread's existing typed array views (not detached like regular ArrayBuffer — stale length, silent unless you check). Needs a cross-thread broadcast (atomic generation counter or `Atomics.notify`) so every consumer — render loop, inspector — recreates its view. Stronger requirement than "re-acquire after growth" in VISION.md; it's a real notification mechanism, not a JS-side self-check.
  ```js
  const memory = new WebAssembly.Memory({ initial: 256, maximum: 16384, shared: true });
  ```

- **Atomics alignment**: `Int32Array` offsets for `Atomics.load/store` must be multiples of 4. Matters once you're hand-computing offsets across transform/velocity/contact/flag buffers in one shared region — misalignment throws `RangeError` at least, so it's loud, but check it in Stage 1A.

- **`console_error_panic_hook` on day 1**, not later — without it a Rust panic in the worker is just `unreachable executed`, no trace.
  ```rust
  #[wasm_bindgen(start)]
  pub fn init() { console_error_panic_hook::set_once(); }
  ```

- **Dev server COOP/COEP**, Vite doesn't set these by default:
  ```ts
  // vite.config.ts
  server: {
    headers: {
      'Cross-Origin-Opener-Policy': 'same-origin',
      'Cross-Origin-Embedder-Policy': 'require-corp',
    },
  },
  ```

- **Two boundaries, not one**: wasm-bindgen glue for the command API (spawn/destroy/force — low freq, marshalling cost fine). Raw `WebAssembly.Memory.buffer` + manual typed array construction for the shared buffer reads. Don't let wasm-bindgen anywhere near the hot read path — it'll sneak in copies/allocations you didn't ask for.

---

## Phase 1 — Runtime Substrate

### Stage 1A — Boundary mechanics
- `TransformBuffers` struct (buf_a/buf_b/write_index/capacity), `swap()`, `current_write_ptr()`
- `TransformView` TS class, `Atomics.load` read path
- SPSC command ring (spawn/force/destroy), fixed capacity, wraparound handling
- JS-owned ID counter (`Atomics.add`), freelist, no generation byte yet
- SAB feature detect + non-SAB fallback (sync WASM call path)
- **Exit:** spawn 10k entities from JS, worker writes garbage transforms, main thread reads them via swap, no main-thread WASM calls per frame. Correctness of physics doesn't matter yet — only the plumbing.

### Stage 1B — Boundary hardening
- Generation byte on entity slots, stale-handle detection (`spawn` returns `{id, gen}`, `destroy` bumps gen, any command against a stale gen is silently dropped + counted, not panicked)
- Buffer growth path: realloc, detach event fired to JS (`onBufferGrowth` callback), SDK auto-reacquires views — write a test that grows mid-frame and asserts no torn read
- COOP/COEP doc page + a runtime check that throws a clear error if SAB is expected but unavailable due to missing headers (distinguish "no SAB support" from "SAB supported but headers missing" — different error messages, first is a browser limitation, second is a config bug on the host site)
- Command ring backpressure: what happens when JS pushes faster than worker drains? Either the ring blocks (bad, defeats purpose) or overwrites oldest (bad, silently drops commands) or grows (bounded growth + counter). Pick bounded growth + inspector counter, not silent drop.
- **Determinism smoke test, moved up from Phase 5.** Before any physics is built: run a fixed-seed no-physics integration step (`pos += vel * dt` a few thousand iterations, mixed with `sin`/`cos`) in Chrome/Firefox/Safari headless via Playwright, diff bit patterns. An afternoon here beats discovering in Phase 5 that `atan2` isn't bit-identical across browsers after the solver's built on that assumption.
- **Exit:** fuzz test — random spawn/destroy/grow churn for 10k ticks, zero torn reads, zero silent command drops, growth events counted and visible. Determinism smoke test passing (or failure mode documented) before Phase 2 starts.

---

## Phase 2 — Physics Core

### Stage 2A — Rigid bodies, single shape type
- Box only, no sphere/capsule/hull yet — prove the pipeline end to end on the simplest shape
- Spatial hash grid (as specced), narrow phase = AABB-only (no exact box-box SAT yet, just broad phase correctness)
- Sequential-impulse solver, fixed 4 iterations, no warm starting
- Integration (semi-implicit Euler), no sleeping yet
- **Exit:** a stack of 50 boxes settles under gravity without exploding or NaN-ing, at 60fps in the worker.

### Stage 2B — Full shape set + solver correctness
- Sphere, capsule, convex hull narrow phase (SAT for convex, sphere-sphere/sphere-capsule analytic)
- Warm starting on the solver (carry impulses frame-to-frame for contact persistence — real perf/stability win, don't skip it)
- Sleeping (velocity + angular velocity threshold, island-based — sleeping is per-connected-component, not per-body, or a sleeping body touching an awake one causes tunneling)
- Continuous collision detection for fast-moving small bodies (swept sphere at minimum — full CCD for arbitrary shapes is a stretch goal, not a v1 requirement)
- Grid bucket occupancy histogram wired into inspector
- **Exit:** ragdoll-equivalent stress test (30 constrained bodies, mixed shapes) stable for 10k ticks, no tunneling on fast projectiles vs thin walls.

---

## Phase 3 — Rendering + Assets

### Stage 3A — Three.js adapter, GLTF in
- `threeAdapter(transforms, instancedMesh)` — instance matrix update from buffer, no per-entity Three.js objects
- GLTF load -> mesh extraction -> convex hull / primitive-fit collider generation -> spawn command
- Camera/scene ownership stays fully in user code — Dynova never creates a `THREE.Scene`
- **Exit:** load a GLTF scene, spawn colliders matching visual meshes, drop them, watch them collide and render correctly.

### Stage 3B — Adapter contract hardening + second renderer as proof
- Formalize the adapter interface (buffer layout doc + TS type) so it's actually renderer-agnostic and not just "whatever Three.js needed"
- Build a second adapter — raw WebGL2 or Canvas2D, doesn't matter which, the point is proving the contract holds without a second full renderer's worth of scope
- **Exit:** same buffer, same World, swap Three.js adapter for the second one at runtime with zero core changes. If this requires touching `core/`, the Stage 3A contract was wrong — fix it here, not later.

---

## Phase 4 — Tooling

### Stage 4A — Inspector read-only
- Entity list, transform/velocity readout, contact points, sleep state — all read from the same shared buffers the renderer uses, zero extra IPC
- Profiler: FPS, frame time, swap rate, command queue depth, memory usage
- **Exit:** inspector runs as a separate panel reading the same SAB the game does, live, no perf hit on the sim (verify: sim FPS unchanged with inspector open vs closed).

### Stage 4B — Inspector read-write + serialization
- Click an entity in inspector, drag to reposition -> issues a command, doesn't mutate buffer directly (respect the command-queue-is-the-only-write-path rule even for tooling)
- Snapshot/restore: serialize full buffer set + command log since last snapshot, binary format
- Replay scrubbing UI (timeline, jump to tick N by replaying from nearest snapshot + command log)
- **Exit:** record 10s of gameplay, scrub timeline backward/forward, snapshot+restore round-trips bit-identical (within the single-build determinism guarantee).

---

## Phase 5 — Re-evaluate (gated on data, not calendar)

### Stage 5A — Profiling-driven decisions
- Pull grid bucket histogram data from real usage (Phase 2B onward) — decide BVH switch only if it shows the clustering pathology, not preemptively
- Decide fixed-point math commitment for lockstep based on the cross-browser f32 determinism results from the Stage 1B smoke test — extend that same harness to cover the actual solver now that it exists, don't write it fresh here
- **Exit:** written decision doc, per item, backed by data — "BVH: no, grid histogram flat across 3 test scenes" or "BVH: yes, terrain+debris scene shows 40% of pairs in 2% of cells"

### Stage 5B — Scope expansion based on 5A
- If lockstep verified feasible: fixed-point math migration for solver
- If BVH warranted: implement as a swappable broad-phase behind the same `query_pairs()` interface, not a rewrite of narrow phase
- Second vertical target selection (robotics/CAD/digital-twin) — pick one, not all three, based on which non-goal from VISION.md is getting the most inbound interest
- **Exit:** whichever expansion was greenlit ships with its own Stage A/B breakdown, spun into Phase 6+.

---

## Cross-phase invariants (never regress)

- No per-entity WASM call on the read path, at any phase
- Command queue is the only write path — inspector, replay, and user code all go through it
- Every new buffer type follows the double-buffer + atomic swap pattern from Phase 1, no exceptions for "just this one small thing"
