import { TransformView } from './TransformView.js';
import { CommandRing, SpawnDesc, ForceDesc, DestroyDesc } from './CommandRing.js';
import { EntityIdAllocator, EntityHandle } from './EntityIdAllocator.js';
import type { MemoryMode } from './memory.js';

export interface WorldConfig {
    capacity: number;
    ringCapacity: number;
}

export interface WorldLayout {
    transformAByteOffset: number;
    transformBByteOffset: number;
    writeIndexByteOffset: number;
    ringSlotsByteOffset: number;
    ringHeadByteOffset: number;
    ringTailByteOffset: number;
    idCounterByteOffset: number;
    generationsByteOffset: number;
}

export interface RuntimePointers {
    transforms_buf_a_ptr(): number;
    transforms_buf_b_ptr(): number;
    transforms_write_index_ptr(): number;
    ring_slots_ptr(): number;
    ring_head_ptr(): number;
    ring_tail_ptr(): number;
    id_counter_ptr(): number;
    generations_ptr(): number;
}

export function layoutFromRuntime(rt: RuntimePointers): WorldLayout {
    return {
        transformAByteOffset: rt.transforms_buf_a_ptr(),
        transformBByteOffset: rt.transforms_buf_b_ptr(),
        writeIndexByteOffset: rt.transforms_write_index_ptr(),
        ringSlotsByteOffset: rt.ring_slots_ptr(),
        ringHeadByteOffset: rt.ring_head_ptr(),
        ringTailByteOffset: rt.ring_tail_ptr(),
        idCounterByteOffset: rt.id_counter_ptr(),
        generationsByteOffset: rt.generations_ptr(),
    };
}

export interface SpawnOptions {
    x?: number; y?: number; z?: number;
    qx?: number; qy?: number; qz?: number; qw?: number;
    shape?: number;
    hx?: number; hy?: number; hz?: number;
    invMass?: number;
    restitution?: number;
}

export interface StepDriver {
    step(): void;
}

export class World {
    readonly memory: WebAssembly.Memory;
    readonly mode: MemoryMode;
    readonly layout: WorldLayout;

    private transforms: TransformView;
    private ring: CommandRing;
    private ids: EntityIdAllocator;
    private capacity: number;
    private driver: StepDriver | null;
    private lastBuffer: ArrayBufferLike;
    private fatal: string | null = null;
    private onDispose: (() => void) | null = null;

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
        this.capacity = config.capacity;
        this.driver = driver;
        this.lastBuffer = memory.buffer;

        this.transforms = new TransformView(
            memory,
            layout.transformAByteOffset,
            layout.transformBByteOffset,
            layout.writeIndexByteOffset,
            config.capacity,
        );
        this.ring = new CommandRing(
            memory,
            layout.ringSlotsByteOffset,
            layout.ringHeadByteOffset,
            layout.ringTailByteOffset,
            config.ringCapacity,
        );
        this.ids = new EntityIdAllocator(memory, layout.idCounterByteOffset, config.capacity);
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
        this.reacquireIfGrown();
    }

    step(): void {
        if (!this.driver) throw new Error('step() is only available in fallback mode; the worker steps the runtime');
        this.driver.step();
        this.reacquireIfGrown();
    }

    reacquireIfGrown(): boolean {
        if (this.memory.buffer === this.lastBuffer) return false;
        this.lastBuffer = this.memory.buffer;
        const { layout, capacity } = this;
        this.transforms.reacquire(this.memory, layout.transformAByteOffset, layout.transformBByteOffset, layout.writeIndexByteOffset, capacity);
        this.ring.reacquire(this.memory, layout.ringSlotsByteOffset, layout.ringHeadByteOffset, layout.ringTailByteOffset);
        this.ids.reacquire(this.memory, layout.idCounterByteOffset);
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
            invMass: opts.invMass, restitution: opts.restitution,
        };
        if (!this.ring.spawn(desc)) {
            this.ids.release(handle);
            throw new Error(`CommandRing full — increase ringCapacity (current: ${this.ring.pendingCount})`);
        }
        return handle;
    }

    destroy(handle: EntityHandle): void {
        this.assertAlive();
        if (!this.ids.isLive(handle)) throw new Error(`stale handle {id: ${handle.id}, gen: ${handle.gen}}`);
        const desc: DestroyDesc = { id: handle.id, gen: handle.gen };
        if (!this.ring.destroy(desc)) throw new Error('CommandRing full');
        this.ids.free(handle);
    }

    force(handle: EntityHandle, x: number, y: number, z: number): void {
        this.assertAlive();
        if (!this.ids.isLive(handle)) throw new Error(`stale handle {id: ${handle.id}, gen: ${handle.gen}}`);
        const desc: ForceDesc = { id: handle.id, gen: handle.gen, x, y, z };
        if (!this.ring.force(desc)) throw new Error('CommandRing full');
    }

    /** The returned array is valid until the next `step()` in fallback mode. */
    readTransforms(): Float32Array {
        this.reacquireIfGrown();
        return this.transforms.read();
    }

    get pendingCommands(): number { return this.ring.pendingCount; }
    get backpressureDrops(): number { return this.ring.backpressureDrops; }
    get allocatedEntities(): number { return this.ids.allocated; }
}
