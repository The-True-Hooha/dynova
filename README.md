# Dynova

> A Shared-Memory Simulation Runtime for the Web.

High-performance, modular simulation engine built in Rust and WebAssembly, with first-class TypeScript APIs, renderer-agnostic architecture, and deep observability.

**Rust owns the world. JavaScript reads it.** The only thing crossing the boundary is a command queue in, an atomic-swapped buffer out.

---

## What is this?

Browsers have good rendering APIs and no real simulation runtime. Existing options pay JS↔WASM cost on every read — wasm-bindgen marshalling, wrapper objects, sync layers. Dynova removes reads from the boundary entirely: Rust computes into shared memory, JS renders shared memory, commands are the only round-trip.

**Target user:** web developers, not Rust developers. No Rust or ECS knowledge required to consume the runtime.

---

## Architecture

```
JS host (React/Vue/Svelte/vanilla)
        |  read: Atomics.load flag -> typed array view, no call
        |  write: push to SPSC command ring
        v
Dynova Runtime (worker thread, WASM)
  |- Command drain
  |- Fixed-timestep scheduler
  |- ECS (sparse-set, SoA)
  |- Simulation Modules (physics, particles, animation, navigation, ...)
  \- Double-buffered output buffers, atomic swap/frame
        |
        v
Renderer Adapter (pure function: buffer -> draw calls, no entity knowledge)
  \- Three.js / WebGPU / WebGL / Babylon / Canvas2D
```

---

## Core Principles

1. **Data over objects** — SoA buffers, flat arrays, index handles. No pointer chasing on the hot path.
2. **Simulation over rendering** — renderer reads a buffer, doesn't know entities exist.
3. **Commands in, buffers out** — nothing else crosses the WASM boundary. No per-entity calls, ever.
4. **Boring and correct over clever** — pick the simple thing, commit, revisit with profiling data.
5. **Observable by default** — every buffer, queue depth, and swap rate inspectable without manual instrumentation.
6. **Renderer agnostic** — Three.js is v1's adapter, not a dependency.

---

## Project Structure

```
dynova/
  core/              # Rust: runtime, command ring, triple buffer, broadphase, solver, collision
  sdk/typescript/    # World, handles, command ring, buffer views, launchWorld, sim worker
  renderer/three/    # Three.js instanced adapter and scene-to-collider helpers
  renderer/canvas2d/ # Canvas2D adapter (second renderer, proves the adapter contract)
  examples/          # Runnable demos and e2e pages
  e2e/               # Playwright cross-browser tests
  inspector/         # Inspector panel with optional editing and timeline (@dynova/inspector)
  docs/              # Hosting notes, adapter contract, Three.js adapter, scene colliders, inspector, replay, benchmark
```

Planned, not yet built: `inspector/` (Phase 4).

---

## Getting Started

**Prerequisites**

- [Rust nightly](https://rustup.rs/) + `wasm32-unknown-unknown` target
- [wasm-pack](https://rustwasm.github.io/wasm-pack/)
- [pnpm](https://pnpm.io/)

**Install**

```bash
make install
```

**Build WASM + run examples**

```bash
make build-release
make dev
```

Open `http://localhost:5173`.

---

## Available Commands

| Command | Description |
|---|---|
| `make build` | Build WASM (dev) |
| `make build-release` | Build WASM (release, optimized) |
| `make install` | Install all workspace dependencies |
| `make dev` | Start examples dev server |
| `make test` | Run SDK unit tests |
| `make check` | TypeScript typecheck |
| `make build-st` | Build the single-threaded WASM used by the non-SharedArrayBuffer fallback |
| `make e2e` | Cross-browser Playwright tests (builds both WASM variants first) |
| `make all` | Release WASM build + install |
| `make clean` | Wipe build artifacts |

---

## Target Applications

- **Games** — physics sandboxes, character controllers, vehicles
- **Scientific computing** — particle systems, N-body, fluids, orbital mechanics
- **Engineering** — mechanical simulation, stress visualization
- **Robotics** — motion planning, collision testing, sensor visualization
- **Digital twins** — factories, warehouses, cities
- **Education** — interactive physics and math labs

---

## Status

Phases 1, 2 and 3 are complete: shared-memory worker runtime with a sync fallback, lock-free triple-buffered transforms with growth, bounded command backpressure, and a physics core (spheres, boxes, capsules, convex hulls, friction, warm starting, island sleeping, swept CCD). Phase 3 Stage 3A is done (Three.js instanced adapter, scene-graph-to-collider fitting, model loading (glTF with Draco/Meshopt, multi-file `.gltf`, OBJ, STL, PLY), convex hulls); Stage 3B is done (shared decode contract in the SDK, Canvas2D adapter, runtime swap demo at `/swap.html`; see [docs/adapter-contract.md](./docs/adapter-contract.md)). Phase 5 Stage 5A is done (decision docs in [docs/decisions](./docs/decisions): no BVH, grid fixes instead; f32 determinism holds on native aarch64 and x86_64, Chromium and WebKit, so no fixed-point migration). Phase 4 is done: Stage 4A (read-only inspector, `/inspector.html`; see [docs/inspector.md](./docs/inspector.md)) and Stage 4B (drag to reposition, recording, seek and replay scrubbing; see [docs/replay.md](./docs/replay.md)). Phase 6 (joints, see [docs/joints.md](./docs/joints.md)) is done. Phase 9A (CAD assemblies: mates, a distance joint and a floating origin for far-from-origin models, [docs/assemblies.md](./docs/assemblies.md), `/assembly.html`) is done. Phase 8 (robotics: sensor queries in [docs/sensors.md](./docs/sensors.md), controllers and the demos at `/robot.html` in [docs/robotics.md](./docs/robotics.md)) is done with the solver limits listed there. Phase 7 (digital twin: kinematic bodies fed by telemetry, `/telemetry.html`; see [docs/digital-twin.md](./docs/digital-twin.md)) is done except that the 60 fps frame time at 5000 entities with drawing is not measured. Run `make dev` and open `/gltf.html` (drop your own model files on it) or `/stage3a.html?live=1`. See [ROADMAP.md](./ROADMAP.md) and [PLAN.md](./PLAN.md).

Known limits: a single column of unit boxes buckles above about 20 high; dynamic-vs-dynamic CCD is not implemented; Firefox is configured in Playwright but was not verified in the development sandbox. A solver golden hash (single-threaded build) matches the native build in Chromium and WebKit; a sin/cos/atan2/sqrt probe also matches; a run on the shared (threaded) build is not done. The 30-body stress scene has no joints (bodies are contained by static walls).
