export class EntityIdAllocator {
    private counter: Uint32Array;
    private freeList: number[] = [];

    constructor(memory: WebAssembly.Memory, byteOffset: number) {
        this.counter = new Uint32Array(memory.buffer, byteOffset, 1);
    }

    alloc(): number {
        return this.freeList.pop() ?? Atomics.add(this.counter, 0, 1);
    }

    free(id: number): void {
        this.freeList.push(id);
    }

    get allocated(): number {
        return Atomics.load(this.counter, 0);
    }

    reacquire(memory: WebAssembly.Memory, byteOffset: number): void {
        this.counter = new Uint32Array(memory.buffer, byteOffset, 1);
    }
}
