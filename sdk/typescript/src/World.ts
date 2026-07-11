import { TransformView } from './TransformView.js';
import { CommandRing, SpawnDesc, ForceDesc, DestroyDesc } from './CommandRing.js';
import { EntityIdAllocator, EntityHandle } from './EntityIdAllocator.js';
import { createMemory, MemoryMode } from './memory.js';

export interface WorldConfig {
    capacity: number;
    ringCapacity: number;
    initialPages?: number;
    maximumPages?: number;
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

export interface SpawnOptions {
    x?: number; y?: number; z?: number;
    qx?: number; qy?: number; qz?: number; qw?: number;
}

export class World {
    readonly memory: WebAssembly.Memory;
    readonly mode: MemoryMode;
    readonly layout: WorldLayout;

    private transforms: TransformView;
    private ring: CommandRing;
    private ids: EntityIdAllocator;
    private capacity: number;

    constructor(config: WorldConfig, memory: WebAssembly.Memory, mode: MemoryMode, layout: WorldLayout) {
        this.memory = memory;
        this.mode = mode;
        this.layout = layout;
        this.capacity = config.capacity;

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
        this.ids = new EntityIdAllocator(
            memory,
            layout.idCounterByteOffset,
            layout.generationsByteOffset,
            config.capacity,
        );
    }

    static create(config: WorldConfig): World {
        const initialPages = config.initialPages ?? 256;
        const maximumPages = config.maximumPages ?? 16384;
        const { memory, mode } = createMemory(initialPages, maximumPages);

        const floatsPerEntity = 7;
        const bytesPerFloat = 4;
        const transformBufBytes = config.capacity * floatsPerEntity * bytesPerFloat;
        const ringBytes = config.ringCapacity * 40;

        let offset = 0;
        const align = (n: number) => { offset = Math.ceil(offset / n) * n; };

        align(4); const transformAByteOffset    = offset; offset += transformBufBytes;
        align(4); const transformBByteOffset    = offset; offset += transformBufBytes;
        align(4); const writeIndexByteOffset    = offset; offset += 4;
        align(4); const ringSlotsByteOffset     = offset; offset += ringBytes;
        align(4); const ringHeadByteOffset      = offset; offset += 4;
        align(4); const ringTailByteOffset      = offset; offset += 4;
        align(4); const idCounterByteOffset     = offset; offset += 4;
        const generationsByteOffset             = offset;

        const layout: WorldLayout = {
            transformAByteOffset,
            transformBByteOffset,
            writeIndexByteOffset,
            ringSlotsByteOffset,
            ringHeadByteOffset,
            ringTailByteOffset,
            idCounterByteOffset,
            generationsByteOffset,
        };

        return new World(config, memory, mode, layout);
    }

    spawn(opts: SpawnOptions = {}): EntityHandle {
        const handle = this.ids.alloc();
        const desc: SpawnDesc = {
            id: handle.id, gen: handle.gen,
            x: opts.x ?? 0, y: opts.y ?? 0, z: opts.z ?? 0,
            qx: opts.qx ?? 0, qy: opts.qy ?? 0, qz: opts.qz ?? 0, qw: opts.qw ?? 1,
        };
        if (!this.ring.spawn(desc)) {
            this.ids.free(handle);
            throw new Error(`CommandRing full — increase ringCapacity (current: ${this.ring.pendingCount})`);
        }
        return handle;
    }

    destroy(handle: EntityHandle): void {
        const desc: DestroyDesc = { id: handle.id, gen: handle.gen };
        if (!this.ring.destroy(desc)) throw new Error('CommandRing full');
        this.ids.free(handle);
    }

    force(handle: EntityHandle, x: number, y: number, z: number): void {
        const desc: ForceDesc = { id: handle.id, gen: handle.gen, x, y, z };
        if (!this.ring.force(desc)) throw new Error('CommandRing full');
    }

    grow(pages: number): number {
        let prev: number;
        try { prev = this.memory.grow(pages); } catch (e) {
            if (e instanceof RangeError) return -1;
            throw e;
        }
        if (prev === -1) return -1;
        const { layout, capacity } = this;
        this.transforms.reacquire(this.memory, layout.transformAByteOffset, layout.transformBByteOffset, layout.writeIndexByteOffset, capacity);
        this.ring.reacquire(this.memory, layout.ringSlotsByteOffset, layout.ringHeadByteOffset, layout.ringTailByteOffset);
        this.ids.reacquire(this.memory, layout.idCounterByteOffset, layout.generationsByteOffset);
        return prev;
    }

    readTransforms(): Float32Array {
        return this.transforms.read();
    }

    get pendingCommands(): number { return this.ring.pendingCount; }
    get backpressureDrops(): number { return this.ring.backpressureDrops; }
    get allocatedEntities(): number { return this.ids.allocated; }
}
