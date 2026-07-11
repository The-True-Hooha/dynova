export interface EntityHandle {
    id: number;
    gen: number;
}

export class EntityIdAllocator {
    private counter: Uint32Array;
    private generations: Uint8Array;
    private freeList: EntityHandle[] = [];
    private capacity: number;

    constructor(memory: WebAssembly.Memory, counterByteOffset: number, generationsByteOffset: number, capacity: number) {
        this.counter = new Uint32Array(memory.buffer, counterByteOffset, 1);
        this.generations = new Uint8Array(memory.buffer, generationsByteOffset, capacity);
        this.capacity = capacity;
    }

    alloc(): EntityHandle {
        if (this.freeList.length > 0) {
            const handle = this.freeList.pop()!;
            return handle;
        }
        const id = Atomics.add(this.counter, 0, 1);
        return { id, gen: 0 };
    }

    free(handle: EntityHandle): void {
        const nextGen = (this.generations[handle.id] + 1) & 0xff;
        this.generations[handle.id] = nextGen;
        this.freeList.push({ id: handle.id, gen: nextGen });
    }

    currentGen(id: number): number {
        return this.generations[id];
    }

    get allocated(): number {
        return Atomics.load(this.counter, 0);
    }

    reacquire(memory: WebAssembly.Memory, counterByteOffset: number, generationsByteOffset: number): void {
        this.counter = new Uint32Array(memory.buffer, counterByteOffset, 1);
        this.generations = new Uint8Array(memory.buffer, generationsByteOffset, this.capacity);
    }
}
