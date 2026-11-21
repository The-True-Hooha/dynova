# Dynova

> A simulation runtime for the web: Rust and WebAssembly own the world, JavaScript reads it from shared memory, and the only things crossing the boundary are a command queue in and a lock-free buffer out.

This file is the why and the principles. What is built, and how, is in `ROADMAP.md`, `PLAN.md` and `docs/`.

---

## Thesis

Browsers have good rendering APIs and no real simulation runtime. Existing options pay a JS and WASM cost on every read (wasm-bindgen marshalling, wrapper objects, sync layers). Dynova removes reads from the boundary: Rust computes into shared memory, JavaScript renders shared memory, and commands are the only round trip.

The target user is a web developer, not a Rust developer. If the answer to "why does this exist" is only "physics", use Rapier; only "ECS", use Bevy. Dynova's answer is a zero-copy shared-memory presentation layer that works from any JS framework, needs no Rust knowledge to consume, and is one runtime that serves several simulation domains instead of being rebuilt for each.

---

## Core principles

1. **Data over objects.** Structure-of-arrays buffers, flat arrays, index handles. No pointer chasing on the hot path.
2. **Simulation over rendering.** A renderer reads a buffer; it does not know entities exist.
3. **Commands in, buffers out.** Nothing else crosses the WASM boundary. No per-entity calls on the read path, ever.
4. **Boring and correct over clever.** A spatial hash grid, not a BVH, and a sequential-impulse solver, not a full LCP, until profiling says otherwise (Stage 5A measured it and kept the grid: `docs/decisions/bvh.md`).
5. **Observable by default.** Buffers, queue depth, growth, step time and contacts are inspectable without manual instrumentation (`docs/inspector.md`).
6. **Renderer agnostic.** Adapters are functions over the buffer layout and hold no scene state (`docs/adapter-contract.md`; the Three.js usage is in the `@dynova/three` README).
7. **Determinism is a claim you verify, not one you assert.** The state hash and the pinned scenes are the evidence (`docs/decisions/determinism.md`).

---

## Architecture

```
JS host (React, Vue, Svelte, vanilla)
   |  read:  Atomics.load of the state word, then a typed-array view (no call)
   |  write: push to the single-producer command ring
   v
Dynova runtime (a Web Worker on shared memory, or the calling thread as a fallback)
   |- command drain (JS owns the id space; Rust trusts the id in the command)
   |- fixed-timestep step: contacts, solver, joints, integration, sleeping
   |- triple-buffered transform output, published by one atomic state word
   |- read-only side channels: inspector snapshot, joint states, query results
   v
Renderer adapter (Three.js, Canvas2D; any other can implement the same contract)
```

- **World versus scene.** The world is simulation state and can run headless. A scene (cameras, lights, hierarchy) is presentation and optional.
- **Cross-origin isolation is required** for the worker mode: `Cross-Origin-Opener-Policy: same-origin` and `Cross-Origin-Embedder-Policy: require-corp`. Without `SharedArrayBuffer` the same API runs on the calling thread. See `docs/cross-origin-isolation.md`.
- **Growth** reallocates buffers and detaches JS views; the runtime publishes a layout epoch so a reader re-acquires them, and the inspector counts the growth events.
- **Entity ids** are allocated by JavaScript from a shared counter with a generation byte, so a handle is valid the moment `spawn` returns.

---

## Target applications

The shared-buffer, command-queue, renderer-agnostic contract is meant to generalise: games and sandboxes, scientific computing, engineering and CAD, robotics, digital twins, architecture and education. The roadmap sequences them and gates each on evidence; joints, a telemetry-fed digital twin, sensors and controllers, and assemblies with a floating origin are built (Phases 6 to 9). Particles, fluids, deformables, animation, navigation, terrain and AI are real scope that is not built.

---

## Determinism and replay

Within one wasm build and the platforms tested, a run is bit-identical: the same commands give the same state hash on native aarch64, native x86_64, Chromium and WebKit, and a seek over a recording reproduces the live run. That is enough for save, replay and scrubbing, and it is the precondition for lockstep networking, which is not built (no input exchange, rollback or desync detection). Firefox and real x86_64 browsers are untested.

---

## Not built, on purpose or not yet

Networking, a playground (live editor and gallery), WebGPU and Babylon adapters, byte-format snapshot export, deformables, and the non-rigid simulation modules. Adapters and modules beyond the shipping ones are not scaffolded as empty folders.

---

## The Dynova manifesto

Memory layout matters more than object hierarchy. Simulation is independent of rendering. The JS and WASM boundary is a command queue and a buffer, nothing else. Determinism is a claim you verify, not one you assert. Every subsystem is observable by default. The runtime generalises across simulation domains because the contract, buffers out and commands in, does not care what is being simulated.

## What is actually novel versus Bevy-wasm and Rapier.js

Rapier.js already reads typed-array views over WASM memory: not new, credited. Dynova's delta is that the lock-free buffer publication is part of the runtime contract from the start, JS-owned id allocation is public API, and the modules (physics, joints, kinematics, sensors) share one buffer and command contract instead of each getting its own boundary. None of this is a new algorithm; it is committing to one clean contract and making every domain use it.
