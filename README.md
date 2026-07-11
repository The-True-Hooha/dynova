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
  core/              # Rust: ECS, scheduler, broadphase, solver, buffers
  sdk/typescript/    # World, EntityId handles, command queue, buffer views
  renderer/three/    # v1 Three.js adapter
  inspector/         # Developer tooling
  examples/          # Runnable demos
```

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

Currently in **Phase 1 — Stage 1A** (boundary mechanics). The command ring → Rust drain → transform buffer → JS read path is working. See [ROADMAP.md](./ROADMAP.md) for the full implementation plan.
