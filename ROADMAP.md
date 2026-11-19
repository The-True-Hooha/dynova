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

### Stage 1A — Boundary mechanics  ✅ done
- `TransformBuffers` struct (buf_a/buf_b/write_index/capacity), `swap()`, `current_write_ptr()`
- `TransformView` TS class, `Atomics.load` read path
- SPSC command ring (spawn/force/destroy), fixed capacity, wraparound handling
- JS-owned ID counter (`Atomics.add`), freelist, no generation byte yet
- SAB feature detect + non-SAB fallback (sync WASM call path)
- **Exit:** spawn 10k entities from JS, worker writes garbage transforms, main thread reads them via swap, no main-thread WASM calls per frame. Correctness of physics doesn't matter yet — only the plumbing.

### Stage 1B — Boundary hardening  ✅ done (Firefox unverified in dev sandbox)
- Generation byte on entity slots, stale-handle detection (`spawn` returns `{id, gen}`, `destroy` bumps gen, any command against a stale gen is silently dropped + counted, not panicked)
- Buffer growth path: realloc, detach event fired to JS (`onBufferGrowth` callback), SDK auto-reacquires views — write a test that grows mid-frame and asserts no torn read
- COOP/COEP doc page + a runtime check that throws a clear error if SAB is expected but unavailable due to missing headers (distinguish "no SAB support" from "SAB supported but headers missing" — different error messages, first is a browser limitation, second is a config bug on the host site)
- Command ring backpressure: what happens when JS pushes faster than worker drains? Either the ring blocks (bad, defeats purpose) or overwrites oldest (bad, silently drops commands) or grows (bounded growth + counter). Pick bounded growth + inspector counter, not silent drop.
- **Determinism smoke test, moved up from Phase 5.** Before any physics is built: run a fixed-seed no-physics integration step (`pos += vel * dt` a few thousand iterations, mixed with `sin`/`cos`) in Chrome/Firefox/Safari headless via Playwright, diff bit patterns. An afternoon here beats discovering in Phase 5 that `atan2` isn't bit-identical across browsers after the solver's built on that assumption.
- **Exit:** fuzz test — random spawn/destroy/grow churn for 10k ticks, zero torn reads, zero silent command drops, growth events counted and visible. Determinism smoke test passing (or failure mode documented) before Phase 2 starts.

---

## Phase 2 — Physics Core

### Stage 2A — Rigid bodies, single shape type  ✅ done
- Box only, no sphere/capsule/hull yet — prove the pipeline end to end on the simplest shape
- Spatial hash grid (as specced), narrow phase = AABB-only (no exact box-box SAT yet, just broad phase correctness)
- Sequential-impulse solver, fixed 4 iterations, no warm starting
- Integration (semi-implicit Euler), no sleeping yet
- **Exit:** a stack of 50 boxes settles under gravity without exploding or NaN-ing, at 60fps in the worker.

### Stage 2B — Full shape set + solver correctness  ✅ done with deviations: convex hull moved to Stage 3A; exit 'stack of 50' delivered as wall/pyramid/20-column; exit 'constrained bodies' delivered as 30 unconstrained mixed bodies in a container (no joints exist)
- Sphere, capsule narrow phase (OBB SAT for boxes, sphere-sphere/sphere-capsule analytic); convex hull deferred to Stage 3A
- Warm starting on the solver (carry impulses frame-to-frame for contact persistence — real perf/stability win, don't skip it)
- Sleeping (velocity + angular velocity threshold, island-based — sleeping is per-connected-component, not per-body, or a sleeping body touching an awake one causes tunneling)
- Continuous collision detection for fast-moving small bodies (swept sphere at minimum — full CCD for arbitrary shapes is a stretch goal, not a v1 requirement)
- Grid bucket occupancy histogram wired into inspector
- **Exit:** ragdoll-equivalent stress test (30 constrained bodies, mixed shapes) stable for 10k ticks, no tunneling on fast projectiles vs thin walls.

---

## Phase 3 — Rendering + Assets

### Stage 3A — Three.js adapter, GLTF in  ✅ done (`examples/gltf.html` loads a GLB through `GLTFLoader` from a URL, a file picker or drag-and-drop, with a built-in generated level by default; `examples/stage3a.html` is the procedural-scene variant)
- `threeAdapter(transforms, instancedMesh)` — instance matrix update from buffer, no per-entity Three.js objects
- Convex hull narrow phase (SAT, moved here from Stage 2B) so mesh-derived colliders work
- GLTF load -> mesh extraction -> convex hull / primitive-fit collider generation -> spawn command
- Camera/scene ownership stays fully in user code — Dynova never creates a `THREE.Scene`
- **Exit:** load a GLTF scene, spawn colliders matching visual meshes, drop them, watch them collide and render correctly.

### Stage 3B — Adapter contract hardening + second renderer as proof  ✅ done (`docs/adapter-contract.md`, `@dynova/canvas2d`, `examples/swap.html`; Step 12 changed nothing under `core/`, the hull work in `core/` landed earlier in 3A)
- Formalize the adapter interface (buffer layout doc + TS type) so it's actually renderer-agnostic and not just "whatever Three.js needed"
- Build a second adapter — raw WebGL2 or Canvas2D, doesn't matter which, the point is proving the contract holds without a second full renderer's worth of scope
- **Exit:** same buffer, same World, swap Three.js adapter for the second one at runtime with zero core changes. If this requires touching `core/`, the Stage 3A contract was wrong — fix it here, not later.

---

## Phase 4 — Tooling

### Stage 4A — Inspector read-only (done; see `docs/inspector.md`)
- Entity list, transform/velocity readout, contact points, sleep state — all read from the same shared buffers the renderer uses, zero extra IPC
- Profiler: FPS, frame time, swap rate, command queue depth, memory usage
- **Exit:** inspector runs as a separate panel reading the same SAB the game does, live, no perf hit on the sim (verify: sim FPS unchanged with inspector open vs closed).

### Stage 4B — Inspector read-write + serialization  ✅ done with deviations (`docs/replay.md`): snapshots are in-memory keyframes inside the runtime driven by `Record`/`Seek`/`Pause`/`Teleport` ring commands, with no binary file format (export/import is parked); bit-identical means same build, same browser
- Click an entity in inspector, drag to reposition -> issues a command, doesn't mutate buffer directly (respect the command-queue-is-the-only-write-path rule even for tooling)
- Snapshot/restore: serialize full buffer set + command log since last snapshot, binary format
- Replay scrubbing UI (timeline, jump to tick N by replaying from nearest snapshot + command log)
- **Exit:** record 10s of gameplay, scrub timeline backward/forward, snapshot+restore round-trips bit-identical (within the single-build determinism guarantee).

---

## Phase 5 — Re-evaluate (gated on data, not calendar)

### Stage 5A — Profiling-driven decisions  ✅ done (`docs/decisions/bvh.md`: no BVH, two grid fixes instead; `docs/decisions/determinism.md`: no fixed-point migration, f32 is bit-identical on every build tested)
- Pull grid bucket histogram data from real usage (Phase 2B onward) — decide BVH switch only if it shows the clustering pathology, not preemptively
- Decide fixed-point math commitment for lockstep based on the cross-browser f32 determinism results from the Stage 1B smoke test — extend that same harness to cover the actual solver now that it exists, don't write it fresh here
- **Exit:** written decision doc, per item, backed by data — "BVH: no, grid histogram flat across 3 test scenes" or "BVH: yes, terrain+debris scene shows 40% of pairs in 2% of cells"

### Stage 5B — Scope expansion based on 5A  ✅ resolved 2026-11-10
- No fixed-point migration and no BVH (5A decisions).
- Second vertical: user chose all three, in order: digital twin, robotics, CAD, preceded by joints because robotics and CAD need them. Phases 6–9 below.

---

## Phase 6 — Joints and constraints  ✅ done (docs/joints.md)
- **6A** Joint table and solver: ball, hinge, fixed, slider with limits and motors, solved in the same iteration loop as contacts, warm started, sleeping-island aware. Commands `JointCreate`/`JointDestroy`/`JointMotor` through the ring.
- **6B** Joints in snapshot, replay, state hash and determinism scene; SDK `World.joint*`; inspector shows joints.
- **Exit:** a chain, a hinged door and a motor-driven wheel stay stable for 10k ticks; seek round trip over joint scenes is bit-identical.

## Phase 7 — Digital twin vertical  ✅ done; 60 fps at 5000 entities with drawing and overlay measured in headless Chromium and WebKit (docs/digital-twin.md)
- **7A** Data ingest: kinematic bodies driven by external samples (telemetry to commands, batched), bulk teleport/kinematic-target command, large-N scale pass.
- **7B** Demo and docs: replayed telemetry scene with overlays through the inspector.
- **Exit:** 5k moving tracked entities at 60 fps in the browser fed from a recorded telemetry stream; scrub and replay are bit-identical.

## Phase 8 — Robotics vertical  ✅ done with a limit: a wheeled robot only follows a gentle arc (docs/robotics.md, docs/sensors.md)
- **8A** Sensors: raycast and overlap queries on the read path via the command ring plus a result buffer.
- **8B** Controller loop: joint motor targets from user code each tick, demo arm and wheeled robot.
- **Exit:** a 3-joint arm reaches a target and a wheeled robot follows a path, deterministic under replay.

## Phase 9 — CAD vertical  ✅ done (docs/assemblies.md, docs/assembly-import.md)
- **9A** Assembly mates (fixed, revolute, slider, distance) built on joints, large-coordinate handling.
- **9B** Import of assemblies from GLTF node hierarchy, demo.
- **Exit:** an imported assembly with mates moves through its range without drift.

## Phase 10 — Consumable packages (added 2026-11-11 with the user's approval)
- **10A** Build the four packages (`@dynova/sdk`, `@dynova/three`, `@dynova/canvas2d`, `@dynova/inspector`) to ESM plus type declarations, ship the release wasm (threaded and single-threaded) and the worker inside the SDK, a one-call `launch` for the web, package metadata, per-package READMEs.
- **10B** Prove it: pack the tarballs, install them into a clean Vite app outside the workspace, typecheck and bundle it, and run it in Chromium and WebKit in shared-memory, missing-headers and no-SharedArrayBuffer modes. Packaging docs. Not published to npm.
- **Exit:** a clean app built from the packed tarballs runs a world with physics, the Three.js and Canvas2D adapters and the inspector in two browsers.

## Phase 11 — Solver accuracy (added 2026-11-11 with the user's approval)
Goal: remove the solver limits found while building Phases 6 to 9 (heavy end masses pulling long chains apart, a 2 m hinged door sagging, wheeled robots skidding and tipping, out-of-plane tilt of hinge chains). Each step starts with tests that fail today and ends with every pin re-derived on native aarch64, native x86_64, Chromium and WebKit.
- **11A (not needed; a spike did not help the robot)** Per-body inertia tensors (principal moments applied in world space) instead of one scalar.
- **11B (done)** Sub-stepping for jointed worlds, and a fix to the orientation step, which read world angular velocity in the body frame (the cause of the wheeled-robot tumble). Measured against step time at 1k and 5k bodies.
- **11C (not needed)** Better joint position correction than Baumgarte.
- **11D (not needed)** joint-aware friction ordering; then a direct solve for joint chains (articulated bodies), which is a separate large project.
- **Exit:** a 10-link chain with a 50x end mass stays within 5 cm, a 2 m hinged door sags under 1 cm, the demo robot follows a path needing a 50 percent wheel-speed split without tipping, and the 5000-entity frame budget and every determinism pin hold.

## Phases 12 to 18 — Robot learning and materials (added 2026-11-19 with the user's approval)
Goal: close the gap with robot-learning engines such as Alloy (many parallel worlds, camera sensors, an RL environment API, an asset library, legged robots, deformables, fluids and an optional GPU backend) while the CPU path stays the deterministic reference. Targets everyone: browser users first, Python users who want to push throughput second. Each phase starts with tests that fail today and ends with every determinism pin re-derived on native aarch64, native x86_64, Chromium and WebKit. Order is fixed; a later phase does not start before the earlier one's exit passes.

- **Phase 12 — Parallel worlds and the RL environment API**
  - **12A (done, docs/batch.md; single-threaded wasm step, native threads)** A batch of N independent worlds with their own state, stepped together (workers or WASM threads in the browser, native threads headless). Each world stays bit-identical to the same world run alone.
  - **12B** TypeScript environment API: `reset`, `step`, observation and reward buffers, per-world done flags, seeds. No per-entity calls on the read path.
  - **12C** Python bindings over the same core. Spike first: native wheels (PyO3) against the WASM build under a Python runtime; pick by measured step rate and by whether the results stay bit-identical to the browser. Gym-style `reset`/`step` on batched numpy arrays.
  - **Exit:** 256 worlds of the arm scene step in parallel in Node and from Python; each world's state hash equals the single-world run; step rate is measured and recorded in `docs/`.
- **Phase 13 — Contact accuracy** (largest risk: moves every pinned hash again)
  - Goal: contact-rich manipulation and legged contact that holds up against MuJoCo on a fixed scene set. Candidates, chosen by measurement: friction cone and torsional friction, soft or compliant contacts, a stronger solver for contacts (block or Newton-style), grasp stability, contact friction tangents that no longer switch basis. A scene suite compares against recorded reference numbers; no MuJoCo dependency in the build.
  - **Exit:** a two-finger grasp holds a box under gravity and a lift; a box on an incline slides at the analytic rate; a stack of 20 is stable; step cost at 1k and 5k bodies is re-measured and the 5000-entity frame budget holds.
- **Phase 14 — Camera sensors**
  - Depth, colour-id and segmentation images per world from CPU raycasting on the existing grid, batched through a result buffer; a lens model; images for all worlds in one call.
  - **Exit:** a depth image of a known scene matches analytic distances within 1 mm; segmentation ids match entity ids; 64 worlds at 64x64 are rendered in a measured time.
- **Phase 15 — Assets, robot descriptions and legged robots**
  - URDF and MJCF import (links, joints, collision shapes) into the joint table, an asset set with licences recorded, a quadruped and a humanoid, standing and walking controllers.
  - **Exit:** an imported arm matches its description's pose by forward kinematics; a quadruped stands for 10k ticks and takes steps without tipping; the assets load in the browser and from Python.
- **Phase 16 — Particles, cloth, cables and soft bodies**
  - A position-based particle solver with distance and bending constraints, collisions with rigid bodies, attachment to rigid bodies and joints, in the snapshot, replay and state hash.
  - **Exit:** a cloth drapes over a box and settles; a cable swings from a hinge; a soft block is dropped and rests; a robot pushes a cable; seek round trip is bit-identical.
- **Phase 17 — Fluids and granular materials**
  - Particle fluids and sand (position-based density constraints and friction), coupled to rigid bodies, with a particle budget measured in the browser frame budget.
  - **Exit:** a column of fluid collapses into a basin and the level settles to the analytic volume within 5 percent; a heap of sand holds an angle of repose; a rigid body floats at its analytic depth.
- **Phase 18 — Optional GPU backend**
  - A separate WebGPU compute path for batched worlds, opt-in only. Results are not bit-identical across GPUs; the CPU path stays the deterministic reference and its pins do not move. The GPU path is checked against the CPU by a tolerance suite, not by hash.
  - **Exit:** the arm scene at 4096 worlds runs faster than the CPU batch on a measured GPU and stays within the stated tolerance of the CPU state; the CPU hashes are untouched.

---

## Cross-phase invariants (never regress)

- No per-entity WASM call on the read path, at any phase
- Command queue is the only write path — inspector, replay, and user code all go through it
- Every new buffer type follows the double-buffer + atomic swap pattern from Phase 1, no exceptions for "just this one small thing"
