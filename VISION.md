# Dynova

> A simulation runtime for the web.
> High-performance, modular simulation engine built in Rust and WebAssembly, with first-class TypeScript APIs, renderer-agnostic architecture, powerful developer tooling, and deep observability.

Rust owns the world. JavaScript reads it. The only thing crossing the boundary is a command queue in, an atomic-swapped buffer out.

---

## Thesis

Browsers have good rendering APIs and no real simulation runtime. Existing options pay JS<->WASM cost on every read — wasm-bindgen marshalling, wrapper objects, sync layers. Dynova removes reads from the boundary entirely: Rust computes into shared memory, JS renders shared memory, commands are the only round-trip.

Target user: web developers, not Rust developers. If the answer to "why does this exist" is just "physics" -> use Rapier. Just "ECS" -> use Bevy. Dynova's answer: zero-copy shared-memory presentation layer, usable from any JS framework, no Rust/ECS knowledge required to consume it, and a single runtime that scales across every simulation domain below instead of being rebuilt per-vertical.

---

## Core Principles

1. **Data over objects** — SoA buffers, flat arrays, index handles. No pointer chasing on the hot path.
2. **Simulation over rendering** — renderer reads a buffer, doesn't know entities exist.
3. **Commands in, buffers out** — nothing else crosses the WASM boundary. No per-entity calls, ever.
4. **Boring and correct over clever** — spatial hash grid not BVH until data says otherwise, sequential-impulse solver not full LCP. Pick the simple thing, commit, revisit with profiling data.
5. **Observable by default** — every buffer, queue depth, and swap rate inspectable without manual instrumentation.
6. **Renderer agnostic** — Three.js is v1's adapter, not a dependency. Adapters are pure functions over buffer layouts, no engine-owned scene state.

---

## Target Applications

Full scope of what the runtime is built to eventually serve — not all v1, sequencing is in the separate roadmap doc.

- **Games** — 2D/3D, physics sandboxes, character controllers, vehicles, multiplayer (post-determinism-verification)
- **Scientific computing** — particle systems, N-body, fluids, orbital mechanics, molecular visualization
- **Engineering** — mechanical simulation, stress visualization, assembly/construction sequencing
- **Robotics** — robot sim, motion planning, collision testing, sensor visualization
- **Digital twins** — factories, warehouses, cities, IoT/infrastructure visualization
- **Architecture** — walkthroughs, structural simulation
- **Education** — physics/math/engineering/graphics interactive labs

These are real targets, not decoration — the runtime contract (shared buffers, command queue, renderer-agnostic adapters) is designed to generalize across them. Sequencing and gating criteria per vertical live in the roadmap doc, not here.

---

## Use Cases (concrete, cross-vertical)

- Real-time rigid body sandbox (games, education)
- Large-N particle/point-cloud visualization (scientific computing, digital twins)
- Sensor/collision testing against static + dynamic geometry (robotics)
- Structural stress overlay on a static mesh, no dynamics (engineering, architecture)
- Replay/scrub through a recorded simulation (all verticals — snapshot/restore serialization makes this free)

---

## Architecture

```pwd
JS host (React/Vue/Svelte/vanilla)
        |  read: Atomics.load flag -> typed array view, no call
        |  write: push to SPSC command ring
        v
Dynova Runtime (worker thread, WASM)
  |- Command drain (ID-trusting, no ID generation in Rust)
  |- Fixed-timestep scheduler
  |- ECS (sparse-set, SoA)
  |- Simulation Modules (physics, particles, animation, navigation, ...)
  \- Double-buffered output buffers, atomic swap/frame
        |
        v
Renderer Adapter (pure function: buffer -> draw calls, no entity knowledge)
  \- WebGPU / WebGL / Three.js / Babylon / Canvas2D (v1 ships one, contract supports all)
```

World vs Scene: World is simulation state (physics, entities, transforms) and can exist headless. Scene is presentation (cameras, lights, visual hierarchy) and is optional — a World can run with zero renderers attached (e.g. server-side or worker-only sim for validation/replay generation).

---

## The Boundary — mechanism

## Cross-origin isolation (mandatory)

`SharedArrayBuffer` is undefined without:

```pwd
Cross-Origin-Opener-Policy: same-origin
Cross-Origin-Embedder-Policy: require-corp
```

Third-party iframes on the host page need `Cross-Origin-Resource-Policy: cross-origin` on their own responses or they break under COEP. This is a real adoption blocker for embedding into existing sites, not a footnote.

Fallback (no SAB): plain `ArrayBuffer`, single-threaded, sync WASM step call per frame. Detect via `typeof SharedArrayBuffer !== 'undefined'`, switch backend transparently, same API surface either way.

## Double-buffered swap

```rust
use std::sync::atomic::{AtomicU32, Ordering};

#[repr(C)]
pub struct TransformBuffers {
    pub buf_a: *mut f32,
    pub buf_b: *mut f32,
    pub write_index: AtomicU32,
    pub capacity: u32,
}

impl TransformBuffers {
    pub fn current_write_ptr(&self) -> *mut f32 {
        if self.write_index.load(Ordering::Acquire) == 0 { self.buf_a } else { self.buf_b }
    }
    pub fn swap(&self) {
        let cur = self.write_index.load(Ordering::Acquire);
        self.write_index.store(1 - cur, Ordering::Release);
    }
}
```

```ts
export class TransformView {
  private a: Float32Array;
  private b: Float32Array;
  private flagPtr: Int32Array;

  read(): Float32Array {
    const writing = Atomics.load(this.flagPtr, 0);
    return writing === 0 ? this.b : this.a;
  }
}
```

Worker split — main thread reads via `Atomics.load`, worker writes via `Atomics.store` after each fixed-timestep tick. No lock. Command ring (spawn/force/destroy) is a separate SPSC structure, doesn't need double-buffering since it's a queue not a snapshot.

## Entity ID allocation

JS owns the ID space via a shared atomic counter. Rust never mints IDs, only trusts the one in the command:

```ts
export class World {
  private idCounter: Uint32Array;
  private freeList: number[] = [];

  spawn(desc: EntityDesc): EntityId {
    const id = this.freeList.pop() ?? Atomics.add(this.idCounter, 0, 1);
    this.commandQueue.push({ type: 'Spawn', id, desc });
    return id; // valid immediately, no round-trip
  }

  destroy(id: EntityId): void {
    this.commandQueue.push({ type: 'Destroy', id });
    this.freeList.push(id); // pair with a generation byte for stale-handle safety
  }
}
```

---

## Broad Phase

Flat-array spatial hash grid, not BVH, not SAP, as default across modules where a broad phase is needed. Rebuild every frame, O(N), no sort, no tree — matches roughly-uniform entity distributions (most games/education/particle use cases). Switch to BVH only for verticals with high size/density variance (large static terrain + tiny debris — likely relevant for digital twins/CAD later), gated on the grid bucket occupancy histogram in the inspector showing that pathology.

```rust
const GRID_CELLS: usize = 4096;

pub struct SpatialGrid {
    cell_size: f32,
    buckets: Vec<Vec<u32>>,
}

impl SpatialGrid {
    #[inline]
    fn hash(&self, x: i32, y: i32, z: i32) -> usize {
        let h = (x.wrapping_mul(73856093)) ^ (y.wrapping_mul(19349663)) ^ (z.wrapping_mul(83492791));
        (h as usize) & (GRID_CELLS - 1)
    }

    pub fn clear(&mut self) {
        for b in self.buckets.iter_mut() { b.clear(); }
    }

    pub fn insert(&mut self, entity: u32, min: [f32;3], max: [f32;3]) {
        let cs = self.cell_size;
        let (min_c, max_c) = (
            [(min[0]/cs) as i32, (min[1]/cs) as i32, (min[2]/cs) as i32],
            [(max[0]/cs) as i32, (max[1]/cs) as i32, (max[2]/cs) as i32],
        );
        for x in min_c[0]..=max_c[0] {
            for y in min_c[1]..=max_c[1] {
                for z in min_c[2]..=max_c[2] {
                    self.buckets[self.hash(x,y,z)].push(entity);
                }
            }
        }
    }

    pub fn query_pairs(&self, out: &mut Vec<(u32,u32)>) {
        for bucket in &self.buckets {
            for i in 0..bucket.len() {
                for j in (i+1)..bucket.len() {
                    out.push((bucket[i], bucket[j])); // dedupe downstream
                }
            }
        }
    }
}
```

---

## Simulation Modules

**Physics** — rigid bodies, collision, constraints, sleeping, continuous collision detection
**Deformables** — rope, cloth, springs, soft bodies (post-rigid-body-stability)
**Vehicles** — wheel/suspension constraint stacks on top of rigid body core
**Particles/Fluids** — reuse transform/velocity buffer contract, different integration rules
**Animation** — skeleton buffer, separate from transform buffer, same double-buffer contract
**Navigation** — pathfinding/steering, reads static collision geometry, writes velocity commands
**Terrain** — heightmap collider generation, feeds broad phase same as any static mesh
**AI** — behavior layer issuing commands through the same command queue as user code — no special-cased API

Every module is a plugin over the same buffer/command contract. None get a bespoke boundary mechanism — that's the actual point of the runtime, not a slogan.

---

## Renderer Adapters

WebGPU, WebGL2, Three.js, Babylon.js, Canvas2D, WebXR (future) — all consume the same transform/skeleton/particle buffers via `Atomics.load` + typed array view. An adapter is a pure function: buffer in, draw calls out. It never holds entity state, never gets a "sync" step — if it needs data, it reads the current buffer, every frame, full stop.

```ts
function threeAdapter(transforms: Float32Array, meshes: THREE.InstancedMesh) {
  for (let i = 0; i < meshes.count; i++) {
    const o = i * 10;
    meshes.setMatrixAt(i, matrixFromTRS(transforms, o));
  }
  meshes.instanceMatrix.needsUpdate = true;
}
```

---

## Asset System

Formats: GLTF (primary), OBJ, STL, PLY, heightmaps, voxels, custom importers.
Features: asset registry, streaming, compression, runtime loading, caching.

GLTF + collider generation (convex hull or primitive fit) is the only import path that matters for early modules; the rest of the format list is real scope, sequenced later — not cut, just not gating Phase 1.

---

## Simulation Pipeline (per fixed-timestep tick)

```md
Drain command ring (spawn/force/destroy, trust incoming IDs)
        v
Integrate velocities (forces, gravity)
        v
Broad phase: rebuild spatial grid, query_pairs()
        v
Narrow phase: exact collision test, dedupe
        v
Contact generation
        v
Constraint building
        v
Sequential-impulse solver (fixed iteration count)
        v
Integration (positions)
        v
Animation
        v
Sleeping check
        v
Write output buffers (transform/velocity/contact/skeleton/particle)
        v
Atomic swap
```

Every stage is a plain function over buffers, independently replaceable. No stage owns global state beyond its own input/output buffers. This is also the Frame Graph — each stage is a dependency node, execution order derived from data dependencies not hardcoded sequence, which is what makes future multithreading across stages (not just sim-vs-render) possible without a rewrite.

---

## Memory System

Zero allocations during steady-state simulation. Arena allocators, object pools, dense buffers, bitsets, scratch allocators, index handles. Growth (buffer capacity exceeded) triggers a realloc — and detaches any live `ArrayBuffer` views on the JS side, which must be re-acquired after any command flush that could've caused growth. This is the sharpest edge in the whole system; surfaced in the inspector as a "buffer growth event" counter so it's never silent.

---

## Determinism

Guaranteed only within a single WASM build, single-threaded step, same replay -> same result. Enough for save-state/replay/scrubbing. NOT enough for cross-client lockstep multiplayer — that needs fixed-point math or verified f32-only (no native SIMD divergence) cross-browser testing, neither done yet. Real scope, gated on that verification landing first.

---

## Serialization

JSON, binary (bincode-style), snapshot/restore, replay, save/load, state diffing. Falls out of the buffer-based world model almost for free — a snapshot is just a copy of the current buffer set plus the command log since last snapshot.

---

## Networking (future)

Deterministic sim where verified-practical: multiplayer, rollback, replay, lockstep sync, state replication. Explicitly gated on the Determinism section above landing first — not promised on the current architecture.

---

## Developer Tooling

**Inspector** — panels for World, Scene, Physics, Renderer, Assets, Memory, Plugins, Events. Selecting an entity shows position, velocity, angular velocity, contacts, constraints, sleep state.

**Visualization** — collision shapes, AABB, grid bucket occupancy (surfaces broad-phase pathology directly), contact points/normals, sleeping bodies, forces, memory layout.

**Profiler** — FPS, frame time, physics/solver/render time, buffer swap rate, command queue depth (backpressure signal), memory usage, allocation count, active body count.

**Playground** — live code editor, inspector, timeline, asset browser, example gallery, one-click share/export. Real deliverable, sequenced after core runtime + one renderer adapter are solid.

---

## Project Structure

```pwd
dynova/
  core/              # Rust: ECS, scheduler, broadphase, solver, buffers, frame graph
  sdk-typescript/     # World, EntityId handles, command queue, buffer views
  renderer-three/     # v1 adapter
  renderer-webgpu/     # sequenced, not scaffolded yet
  renderer-babylon/
  inspector/
  playground/
  examples/
  documentation/
```

Adapters beyond the shipping one are real future directories, not committed scaffolding — don't create empty folders for renderers with zero implementation, that's just noise in the tree.

---

## The Dynova Manifesto

Memory layout matters more than object hierarchy. Simulation is independent of rendering. The JS/WASM boundary is a command queue and a buffer, nothing else. Determinism is a claim you verify, not one you assert. Every subsystem is observable by default. The runtime generalizes across simulation domains because the contract — buffers in, commands out — doesn't care what's being simulated.

---

## What's actually novel vs Bevy-wasm / Rapier.js

Rapier.js already does typed-array-view reads over WASM memory — not new, credited. Dynova's delta: the worker-thread double-buffer swap is part of the runtime contract from day one (not bolted on), JS-owned atomic ID allocation is public API contract (not internal detail), and the module system (physics, particles, animation, navigation, AI) all share one buffer/command contract instead of each getting its own boundary mechanism. None of this is a new algorithm — it's committing to one clean contract and making every domain use it.
