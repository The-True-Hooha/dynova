export const SLOT_BYTES = 40;

export const TAG_SPAWN   = 0;
export const TAG_DESTROY = 1;
export const TAG_FORCE   = 2;

export interface SpawnDesc {
    id: number;
    gen: number;
    x: number; y: number; z: number;
    qx: number; qy: number; qz: number; qw: number;
}

export interface DestroyDesc {
    id: number;
    gen: number;
}

export interface ForceDesc {
    id: number;
    gen: number;
    x: number; y: number; z: number;
}

export class CommandRing {
    private head: Int32Array;
    private tail: Int32Array;
    private capacity: number;
    private mask: number;
    private view: DataView;
    private drops = 0;

    constructor(
        memory: WebAssembly.Memory,
        slotsByteOffset: number,
        headByteOffset: number,
        tailByteOffset: number,
        capacity: number,
    ) {
        this.capacity = capacity;
        this.mask = capacity - 1;
        this.head = new Int32Array(memory.buffer, headByteOffset, 1);
        this.tail = new Int32Array(memory.buffer, tailByteOffset, 1);
        this.view = new DataView(memory.buffer, slotsByteOffset, capacity * SLOT_BYTES);
    }

    private isFull(): boolean {
        const tail = Atomics.load(this.tail, 0);
        const head = Atomics.load(this.head, 0);
        return (tail - head) >>> 0 >= this.capacity;
    }

    private slotOffset(tail: number): number {
        return (tail & this.mask) * SLOT_BYTES;
    }

    spawn(desc: SpawnDesc): boolean {
        if (this.isFull()) { this.drops++; return false; }
        const tail = Atomics.load(this.tail, 0);
        const off = this.slotOffset(tail);
        this.view.setUint8(off,      TAG_SPAWN);
        this.view.setUint32(off + 1, desc.id,  true);
        this.view.setUint8(off + 5,  desc.gen);
        this.view.setFloat32(off + 6,  desc.x,  true);
        this.view.setFloat32(off + 10, desc.y,  true);
        this.view.setFloat32(off + 14, desc.z,  true);
        this.view.setFloat32(off + 18, desc.qx, true);
        this.view.setFloat32(off + 22, desc.qy, true);
        this.view.setFloat32(off + 26, desc.qz, true);
        this.view.setFloat32(off + 30, desc.qw, true);
        Atomics.add(this.tail, 0, 1);
        return true;
    }

    destroy(desc: DestroyDesc): boolean {
        if (this.isFull()) { this.drops++; return false; }
        const tail = Atomics.load(this.tail, 0);
        const off = this.slotOffset(tail);
        this.view.setUint8(off,      TAG_DESTROY);
        this.view.setUint32(off + 1, desc.id,  true);
        this.view.setUint8(off + 5,  desc.gen);
        Atomics.add(this.tail, 0, 1);
        return true;
    }

    force(desc: ForceDesc): boolean {
        if (this.isFull()) { this.drops++; return false; }
        const tail = Atomics.load(this.tail, 0);
        const off = this.slotOffset(tail);
        this.view.setUint8(off,      TAG_FORCE);
        this.view.setUint32(off + 1, desc.id,  true);
        this.view.setUint8(off + 5,  desc.gen);
        this.view.setFloat32(off + 6,  desc.x,  true);
        this.view.setFloat32(off + 10, desc.y,  true);
        this.view.setFloat32(off + 14, desc.z,  true);
        Atomics.add(this.tail, 0, 1);
        return true;
    }

    get backpressureDrops(): number { return this.drops; }

    get pendingCount(): number {
        const tail = Atomics.load(this.tail, 0);
        const head = Atomics.load(this.head, 0);
        return (tail - head) >>> 0;
    }

    reacquire(memory: WebAssembly.Memory, slotsByteOffset: number, headByteOffset: number, tailByteOffset: number): void {
        this.head = new Int32Array(memory.buffer, headByteOffset, 1);
        this.tail = new Int32Array(memory.buffer, tailByteOffset, 1);
        this.view = new DataView(memory.buffer, slotsByteOffset, this.capacity * SLOT_BYTES);
    }
}
