import { HEADER_WORDS, TransformView } from './TransformView.js';
import { CommandRing, SHAPE_HULL, HULL_VERTS_PER_SLOT, MAX_CAPACITY, MAX_HULLS, MAX_HULL_COORD, MAX_HULL_VERTS, SpawnDesc, ForceDesc, DestroyDesc } from './CommandRing.js';
import { EntityIdAllocator, EntityHandle } from './EntityIdAllocator.js';
import type { MemoryMode } from './memory.js';

export interface WorldConfig {
    capacity: number;
    ringCapacity: number;
    /** Commands held in JS when the ring is full before spawn/destroy/force throw. Default 65536. */
    maxDeferredCommands?: number;
}

export interface WorldLayout {
    headerByteOffset: number;
    ringSlotsByteOffset: number;
    ringHeadByteOffset: number;
    ringTailByteOffset: number;
    idCounterByteOffset: number;
}

export interface RuntimePointers {
    header_ptr(): number;
    ring_slots_ptr(): number;
    ring_head_ptr(): number;
    ring_tail_ptr(): number;
    id_counter_ptr(): number;
    /** Header size the wasm build was compiled with; compared with this package's `HEADER_WORDS`. */
    header_words?(): number;
}

export function layoutFromRuntime(rt: RuntimePointers): WorldLayout {
    const words = rt.header_words?.();
    if (words !== undefined && words !== HEADER_WORDS) {
        throw new Error(`wasm header has ${words} words but the SDK expects ${HEADER_WORDS}: rebuild the wasm package (make build)`);
    }
    return {
        headerByteOffset: rt.header_ptr(),
        ringSlotsByteOffset: rt.ring_slots_ptr(),
        ringHeadByteOffset: rt.ring_head_ptr(),
        ringTailByteOffset: rt.ring_tail_ptr(),
        idCounterByteOffset: rt.id_counter_ptr(),
    };
}

export interface SpawnOptions {
    x?: number; y?: number; z?: number;
    qx?: number; qy?: number; qz?: number; qw?: number;
    shape?: number;
    hx?: number; hy?: number; hz?: number;
    invMass?: number;
    restitution?: number;
    friction?: number;
    /** Hull id from `registerHull`; required with `shape: SHAPE_HULL`. */
    hull?: number;
}

export interface StepDriver {
    step(): void;
}

export interface GrowthEvent {
    epoch: number;
    capacity: number;
}

type Queued =
    | { k: 'spawn'; d: SpawnDesc }
    | { k: 'destroy'; d: DestroyDesc }
    | { k: 'force'; d: ForceDesc }
    | { k: 'grow'; capacity: number }
    | { k: 'gravity'; v: [number, number, number] }
    | { k: 'inspect'; on: boolean }
    | { k: 'hullVerts'; hull: number; coords: number[] }
    | { k: 'hullCommit'; hull: number; count: number };

const DEFAULT_MAX_DEFERRED = 1 << 16;

export class World {
    readonly memory: WebAssembly.Memory;
    readonly mode: MemoryMode;
    readonly layout: WorldLayout;
    onBufferGrowth: ((e: GrowthEvent) => void) | null = null;

    private transforms: TransformView;
    private ring: CommandRing;
    private ids: EntityIdAllocator;
    private ringCapacity: number;
    private driver: StepDriver | null;
    private lastBuffer: ArrayBufferLike;
    private knownEpoch: number;
    private fatal: string | null = null;
    private onDispose: (() => void) | null = null;
    private deferred: Queued[] = [];
    private deferredHead = 0;
    private maxDeferred: number;
    private deferredTotal = 0;
    private rejected = 0;
    private growths = 0;
    private pendingGrow = 0;
    private nextHull = 0;

    private constructor(
        config: WorldConfig,
        memory: WebAssembly.Memory,
        mode: MemoryMode,
        layout: WorldLayout,
        driver: StepDriver | null,
    ) {
        this.memory = memory;
        this.mode = mode;
        this.layout = layout;
        this.driver = driver;
        this.ringCapacity = config.ringCapacity;
        this.maxDeferred = config.maxDeferredCommands ?? DEFAULT_MAX_DEFERRED;
        this.lastBuffer = memory.buffer;

        this.transforms = new TransformView(memory, layout.headerByteOffset);
        this.knownEpoch = this.transforms.epoch;
        this.ring = new CommandRing(
            memory,
            layout.ringSlotsByteOffset,
            layout.ringHeadByteOffset,
            layout.ringTailByteOffset,
            config.ringCapacity,
        );
        this.ids = new EntityIdAllocator(memory, layout.idCounterByteOffset, this.transforms.capacity);
    }

    /** Worker mode: another thread owns the runtime and steps it. */
    static attach(config: WorldConfig, memory: WebAssembly.Memory, mode: MemoryMode, layout: WorldLayout): World {
        return new World(config, memory, mode, layout, null);
    }

    /** Fallback mode: this thread owns the runtime; call `step()` yourself. */
    static attachSync(
        config: WorldConfig,
        memory: WebAssembly.Memory,
        mode: MemoryMode,
        layout: WorldLayout,
        driver: StepDriver,
    ): World {
        return new World(config, memory, mode, layout, driver);
    }

    /** @internal set by launchWorld so dispose() can stop the simulation worker. */
    setDisposer(fn: () => void): void { this.onDispose = fn; }

    /** @internal called when the simulation worker dies; further commands throw. */
    markFatal(message: string): void { this.fatal ??= message; }

    get failure(): string | null { return this.fatal; }

    dispose(): void {
        this.fatal ??= 'world disposed';
        this.onDispose?.();
        this.onDispose = null;
    }

    private assertAlive(): void {
        if (this.fatal) throw new Error(`world unavailable: ${this.fatal}`);
        this.syncLayout();
    }

    syncLayout(): boolean {
        let changed = false;
        if (this.memory.buffer !== this.lastBuffer) {
            this.lastBuffer = this.memory.buffer;
            const { layout } = this;
            this.transforms.reacquire();
            this.ring.reacquire(this.memory, layout.ringSlotsByteOffset, layout.ringHeadByteOffset, layout.ringTailByteOffset);
            this.ids.reacquire(this.memory, layout.idCounterByteOffset);
            changed = true;
        }
        const peek = this.transforms.peekEpoch();
        if ((peek & 1) === 0 && peek !== this.knownEpoch) {
            this.transforms.reacquire();
            this.applyEpoch();
            changed = true;
        }
        return changed;
    }

    private applyEpoch(): void {
        this.knownEpoch = this.transforms.epoch;
        this.ids.setCapacity(this.transforms.capacity);
        this.growths++;
        if (this.pendingGrow <= this.transforms.capacity) this.pendingGrow = 0;
        try {
            this.onBufferGrowth?.({ epoch: this.knownEpoch >> 1, capacity: this.transforms.capacity });
        } catch (err) {
            queueMicrotask(() => { throw err; });
        }
    }

    reacquireIfGrown(): boolean { return this.syncLayout(); }

    step(): void {
        if (!this.driver) throw new Error('step() is only available in fallback mode; the worker steps the runtime');
        const t0 = performance.now();
        this.driver.step();
        this.transforms.recordStep((performance.now() - t0) * 1000);
        this.syncLayout();
        this.flush();
    }

    private tryPush(c: Queued): boolean {
        switch (c.k) {
            case 'spawn': return this.ring.spawn(c.d);
            case 'destroy': return this.ring.destroy(c.d);
            case 'force': return this.ring.force(c.d);
            case 'grow': return this.ring.grow(c.capacity);
            case 'gravity': return this.ring.gravity(...c.v);
            case 'inspect': return this.ring.inspect(c.on);
            case 'hullVerts': return this.ring.hullVerts(c.hull, c.coords);
            case 'hullCommit': return this.ring.hullCommit(c.hull, c.count);
        }
    }

    flush(): number {
        while (this.deferredHead < this.deferred.length) {
            if (!this.tryPush(this.deferred[this.deferredHead])) break;
            this.deferredHead++;
        }
        if (this.deferredHead === this.deferred.length) {
            this.deferred.length = 0;
            this.deferredHead = 0;
        } else if (this.deferredHead > 1024 && this.deferredHead * 2 > this.deferred.length) {
            this.deferred = this.deferred.slice(this.deferredHead);
            this.deferredHead = 0;
        }
        return this.deferred.length - this.deferredHead;
    }

    private enqueue(c: Queued): boolean {
        if (this.flush() === 0 && this.tryPush(c)) return true;
        if (this.deferred.length - this.deferredHead >= this.maxDeferred) {
            this.rejected++;
            return false;
        }
        this.deferred.push(c);
        this.deferredTotal++;
        return true;
    }

    spawn(opts: SpawnOptions = {}): EntityHandle {
        this.assertAlive();
        if (opts.shape === SHAPE_HULL && !(Number.isInteger(opts.hull) && (opts.hull as number) >= 0 && (opts.hull as number) < this.nextHull)) {
            throw new RangeError(`spawn with SHAPE_HULL needs a hull id returned by registerHull, got ${opts.hull}`);
        }
        const handle = this.ids.alloc();
        const desc: SpawnDesc = {
            id: handle.id, gen: handle.gen,
            x: opts.x ?? 0, y: opts.y ?? 0, z: opts.z ?? 0,
            qx: opts.qx ?? 0, qy: opts.qy ?? 0, qz: opts.qz ?? 0, qw: opts.qw ?? 1,
            shape: opts.shape, hx: opts.hx, hy: opts.hy, hz: opts.hz,
            invMass: opts.invMass, restitution: opts.restitution, friction: opts.friction, hull: opts.hull,
        };
        if (!this.enqueue({ k: 'spawn', d: desc })) {
            this.ids.release(handle);
            throw new Error(`CommandRing full — ${this.deferred.length - this.deferredHead} commands deferred; increase ringCapacity or maxDeferredCommands`);
        }
        return handle;
    }

    destroy(handle: EntityHandle): void {
        this.assertAlive();
        if (!this.ids.isLive(handle)) throw new Error(`stale handle {id: ${handle.id}, gen: ${handle.gen}}`);
        if (!this.enqueue({ k: 'destroy', d: { id: handle.id, gen: handle.gen } })) throw new Error('CommandRing full');
        this.ids.free(handle);
    }

    force(handle: EntityHandle, x: number, y: number, z: number): void {
        this.assertAlive();
        if (!this.ids.isLive(handle)) throw new Error(`stale handle {id: ${handle.id}, gen: ${handle.gen}}`);
        if (!this.enqueue({ k: 'force', d: { id: handle.id, gen: handle.gen, x, y, z } })) throw new Error('CommandRing full');
    }

    /**
     * Ask the runtime to reallocate its buffers for `capacity` entities. Takes effect when the runtime
     * processes the command; `onBufferGrowth` fires and `capacity` rises once this thread observes it.
     */
    requestGrow(capacity: number): void {
        this.assertAlive();
        const floor = Math.max(this.capacity, this.pendingGrow);
        if (!Number.isInteger(capacity) || capacity <= floor || capacity > MAX_CAPACITY) {
            throw new RangeError(`capacity must be an integer in (${floor}, ${MAX_CAPACITY}], got ${capacity}`);
        }
        if (!this.enqueue({ k: 'grow', capacity })) throw new Error('CommandRing full');
        this.pendingGrow = capacity;
    }

    /**
     * Upload a convex hull from flat x,y,z points (4 to 32 vertices, |coordinate| <= 1e4) and return its id for
     * `spawn({ shape: SHAPE_HULL, hull })`. The runtime recentres the hull on its volume centroid, so the body
     * origin is that centroid. Invalid geometry (for example coplanar points) is rejected by the runtime and counted
     * in `runtimeCounters.malformedDropped`; spawns that reference it are dropped the same way.
     */
    registerHull(points: ArrayLike<number>): number {
        this.assertAlive();
        const count = points.length / 3;
        if (!Number.isInteger(count) || count < 4 || count > MAX_HULL_VERTS) {
            throw new RangeError(`a hull needs 4 to ${MAX_HULL_VERTS} vertices as x,y,z triples, got ${points.length} numbers`);
        }
        for (let i = 0; i < points.length; i++) {
            if (!Number.isFinite(points[i]) || Math.abs(points[i]) > MAX_HULL_COORD) throw new RangeError(`hull coordinate ${i} is not finite or exceeds ${MAX_HULL_COORD}`);
        }
        if (this.nextHull >= MAX_HULLS) throw new RangeError(`hull limit reached (${MAX_HULLS})`);
        const chunks = Math.ceil(count / HULL_VERTS_PER_SLOT);
        const room = this.ringCapacity - this.ring.pendingCount + this.maxDeferred - this.deferredCommands;
        if (chunks + 1 > room) throw new Error('CommandRing full: not enough room to upload this hull');
        const hull = this.nextHull++;
        for (let c = 0; c < chunks; c++) {
            const coords = Array.from(points).slice(c * HULL_VERTS_PER_SLOT * 3, (c + 1) * HULL_VERTS_PER_SLOT * 3);
            this.enqueue({ k: 'hullVerts', hull, coords });
        }
        this.enqueue({ k: 'hullCommit', hull, count });
        return hull;
    }

    /** Set world gravity through the command ring (values must be finite and within +-1000). */
    setGravity(x: number, y: number, z: number): void {
        this.assertAlive();
        if (![x, y, z].every((v) => Number.isFinite(v) && Math.abs(v) <= 1000)) throw new RangeError('gravity must be finite and within +-1000');
        if (!this.enqueue({ k: 'gravity', v: [x, y, z] })) throw new Error('CommandRing full');
    }

    /**
     * Turn the debug snapshot on or off through the command ring. While off the runtime copies nothing; read it with
     * `InspectorView`. Safe to call repeatedly.
     */
    setInspector(on: boolean): void {
        this.assertAlive();
        if (!this.enqueue({ k: 'inspect', on })) throw new Error('CommandRing full');
    }

    /** Valid until the next call to `readTransforms()` (and, in fallback mode, the next `step()`). */
    readTransforms(): Float32Array {
        if (this.fatal) throw new Error(`world unavailable: ${this.fatal}`);
        this.syncLayout();
        this.flush();
        const buf = this.transforms.read();
        if (this.transforms.epoch !== this.knownEpoch) this.applyEpoch();
        return buf;
    }

    get runtimeCounters(): { ticks: number; staleDropped: number; malformedDropped: number } {
        return this.transforms.counters();
    }

    bucketHistogram(): Uint32Array {
        return this.transforms.bucketCounts();
    }

    get runtimeStats(): { ringPending: number; droppedContacts: number; workerOverruns: number; contacts: number } {
        return this.transforms.stats();
    }

    get capacity(): number { return this.ids.capacityNow; }
    get ringSize(): number { return this.ringCapacity; }
    get pendingCommands(): number { return this.ring.pendingCount + this.deferred.length - this.deferredHead; }
    get deferredCommands(): number { return this.deferred.length - this.deferredHead; }
    get totalDeferred(): number { return this.deferredTotal; }
    get backpressureDrops(): number { return this.rejected; }
    get growthEvents(): number { return this.growths; }
    get allocatedEntities(): number { return this.ids.allocated; }
}
