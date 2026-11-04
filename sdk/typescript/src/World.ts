import { TransformView } from './TransformView.js';
import { CommandRing, MAX_CAPACITY, SpawnDesc, ForceDesc, DestroyDesc } from './CommandRing.js';
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
}

export function layoutFromRuntime(rt: RuntimePointers): WorldLayout {
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
    | { k: 'gravity'; v: [number, number, number] };

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

    /** Re-acquire views after memory growth and apply capacity growth announced by the runtime. */
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

    /** Alias kept for callers that only care about non-shared memory growth. */
    reacquireIfGrown(): boolean { return this.syncLayout(); }

    step(): void {
        if (!this.driver) throw new Error('step() is only available in fallback mode; the worker steps the runtime');
        this.driver.step();
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
        }
    }

    /** Move deferred commands into the ring in order; returns how many remain deferred. */
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
        const handle = this.ids.alloc();
        const desc: SpawnDesc = {
            id: handle.id, gen: handle.gen,
            x: opts.x ?? 0, y: opts.y ?? 0, z: opts.z ?? 0,
            qx: opts.qx ?? 0, qy: opts.qy ?? 0, qz: opts.qz ?? 0, qw: opts.qw ?? 1,
            shape: opts.shape, hx: opts.hx, hy: opts.hy, hz: opts.hz,
            invMass: opts.invMass, restitution: opts.restitution, friction: opts.friction,
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

    /** Set world gravity through the command ring (values must be finite and within +-1000). */
    setGravity(x: number, y: number, z: number): void {
        this.assertAlive();
        if (![x, y, z].every((v) => Number.isFinite(v) && Math.abs(v) <= 1000)) throw new RangeError('gravity must be finite and within +-1000');
        if (!this.enqueue({ k: 'gravity', v: [x, y, z] })) throw new Error('CommandRing full');
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

    /** Counters the runtime publishes into shared memory; no WASM call involved. */
    get runtimeCounters(): { ticks: number; staleDropped: number; malformedDropped: number } {
        return this.transforms.counters();
    }

    /** Occupancy of each broadphase hash bucket; for the inspector. Valid until the layout is re-acquired. */
    bucketHistogram(): Uint32Array {
        return this.transforms.bucketCounts();
    }

    /** Ring depth, dropped contacts, worker overruns and contact count, read from shared memory. */
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
