export const SLOT_BYTES = 40;

export const TAG_SPAWN   = 0;
export const TAG_DESTROY = 1;
export const TAG_FORCE   = 2;

export interface SpawnDesc {
    id: number;
    x: number; y: number; z: number;
    qx: number; qy: number; qz: number; qw: number;
}

export interface ForceDesc {
    id: number;
    x: number; y: number; z: number;
}

export class CommandRing {
    private slots: Uint8Array;
    private head: Int32Array;
    private tail: Int32Array;
    private capacity: number;
    private mask: number;
    private view: DataView;

    constructor(
        memory: WebAssembly.Memory,
        slotsByteOffset: number,
        headByteOffset: number,
        tailByteOffset: number,
        capacity: number,
    ) {
        this.capacity = capacity;
        this.mask = capacity - 1;
        this.slots = new Uint8Array(memory.buffer, slotsByteOffset, capacity * SLOT_BYTES);
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
        if (this.isFull()) return false;
        const tail = Atomics.load(this.tail, 0);
        const off = this.slotOffset(tail);
        this.view.setUint8(off, TAG_SPAWN);
        this.view.setUint32(off + 1,  desc.id, true);
        this.view.setFloat32(off + 5,  desc.x,  true);
        this.view.setFloat32(off + 9,  desc.y,  true);
        this.view.setFloat32(off + 13, desc.z,  true);
        this.view.setFloat32(off + 17, desc.qx, true);
        this.view.setFloat32(off + 21, desc.qy, true);
        this.view.setFloat32(off + 25, desc.qz, true);
        this.view.setFloat32(off + 29, desc.qw, true);
        Atomics.add(this.tail, 0, 1);
        return true;
    }

    destroy(id: number): boolean {
        if (this.isFull()) return false;
        const tail = Atomics.load(this.tail, 0);
        const off = this.slotOffset(tail);
        this.view.setUint8(off, TAG_DESTROY);
        this.view.setUint32(off + 1, id, true);
        Atomics.add(this.tail, 0, 1);
        return true;
    }

    force(desc: ForceDesc): boolean {
        if (this.isFull()) return false;
        const tail = Atomics.load(this.tail, 0);
        const off = this.slotOffset(tail);
        this.view.setUint8(off, TAG_FORCE);
        this.view.setUint32(off + 1,  desc.id, true);
        this.view.setFloat32(off + 5,  desc.x,  true);
        this.view.setFloat32(off + 9,  desc.y,  true);
        this.view.setFloat32(off + 13, desc.z,  true);
        Atomics.add(this.tail, 0, 1);
        return true;
    }

    get pendingCount(): number {
        const tail = Atomics.load(this.tail, 0);
        const head = Atomics.load(this.head, 0);
        return (tail - head) >>> 0;
    }
}
