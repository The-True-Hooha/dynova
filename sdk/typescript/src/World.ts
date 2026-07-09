import { TransformView } from './TransformView.js';
import { CommandRing, SpawnDesc, ForceDesc } from './CommandRing.js';
import { EntityIdAllocator } from './EntityIdAllocator.js';
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
        this.ids = new EntityIdAllocator(memory, layout.idCounterByteOffset);
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

        align(4); const transformAByteOffset = offset; offset += transformBufBytes;
        align(4); const transformBByteOffset = offset; offset += transformBufBytes;
        align(4); const writeIndexByteOffset = offset; offset += 4;
        align(4); const ringSlotsByteOffset  = offset; offset += ringBytes;
        align(4); const ringHeadByteOffset   = offset; offset += 4;
        align(4); const ringTailByteOffset   = offset; offset += 4;
        align(4); const idCounterByteOffset  = offset;

        const layout: WorldLayout = {
            transformAByteOffset,
            transformBByteOffset,
            writeIndexByteOffset,
            ringSlotsByteOffset,
            ringHeadByteOffset,
            ringTailByteOffset,
            idCounterByteOffset,
        };

        return new World(config, memory, mode, layout);
    }

    spawn(opts: SpawnOptions = {}): number {
        const id = this.ids.alloc();
        const desc: SpawnDesc = {
            id,
            x: opts.x ?? 0, y: opts.y ?? 0, z: opts.z ?? 0,
            qx: opts.qx ?? 0, qy: opts.qy ?? 0, qz: opts.qz ?? 0, qw: opts.qw ?? 1,
        };
        if (!this.ring.spawn(desc)) {
            this.ids.free(id);
            throw new Error(`CommandRing full — increase ringCapacity (current: ${this.ring.pendingCount})`);
        }
        return id;
    }

    destroy(id: number): void {
        if (!this.ring.destroy(id)) throw new Error('CommandRing full');
        this.ids.free(id);
    }

    force(id: number, x: number, y: number, z: number): void {
        const desc: ForceDesc = { id, x, y, z };
        if (!this.ring.force(desc)) throw new Error('CommandRing full');
    }

    readTransforms(): Float32Array {
        return this.transforms.read();
    }

    writeTransformDirect(entityId: number, x: number, y: number, z: number, qx = 0, qy = 0, qz = 0, qw = 1): void {
        const view = new DataView(this.memory.buffer);
        const writeIndex = view.getUint32(this.layout.writeIndexByteOffset, true);
        const base = writeIndex === 0 ? this.layout.transformBByteOffset : this.layout.transformAByteOffset;
        const off = base + entityId * 7 * 4;
        view.setFloat32(off,      x,  true);
        view.setFloat32(off + 4,  y,  true);
        view.setFloat32(off + 8,  z,  true);
        view.setFloat32(off + 12, qx, true);
        view.setFloat32(off + 16, qy, true);
        view.setFloat32(off + 20, qz, true);
        view.setFloat32(off + 24, qw, true);
    }

    get pendingCommands(): number { return this.ring.pendingCount; }
    get allocatedEntities(): number { return this.ids.allocated; }
}
