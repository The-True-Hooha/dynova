export interface EntityHandle {
    id: number;
    gen: number;
}

const NOT_LIVE = -1;

export class EntityIdAllocator {
    private counter: Uint32Array;
    private freeList: EntityHandle[] = [];
    private liveGen: Int16Array;
    private capacity: number;

    constructor(memory: WebAssembly.Memory, counterByteOffset: number, capacity: number) {
        this.counter = new Uint32Array(memory.buffer, counterByteOffset, 1);
        this.capacity = capacity;
        this.liveGen = new Int16Array(capacity).fill(NOT_LIVE);
    }

    alloc(): EntityHandle {
        const reused = this.freeList.pop();
        if (reused) {
            this.liveGen[reused.id] = reused.gen;
            return reused;
        }
        if (Atomics.load(this.counter, 0) >= this.capacity) {
            throw new RangeError(`entity capacity exhausted (${this.capacity})`);
        }
        const id = Atomics.add(this.counter, 0, 1);
        if (id >= this.capacity) throw new RangeError(`entity capacity exhausted (${this.capacity})`);
        this.liveGen[id] = 0;
        return { id, gen: 0 };
    }

    isLive(handle: EntityHandle): boolean {
        return handle.id >= 0 && handle.id < this.capacity && this.liveGen[handle.id] === handle.gen;
    }

    /** The live handle for `id`, or null when nothing live holds that id. */
    handleOf(id: number): EntityHandle | null {
        if (!Number.isInteger(id) || id < 0 || id >= this.capacity || this.liveGen[id] === NOT_LIVE) return null;
        return { id, gen: this.liveGen[id] };
    }

    free(handle: EntityHandle): boolean {
        if (!this.isLive(handle)) return false;
        this.liveGen[handle.id] = NOT_LIVE;
        this.freeList.push({ id: handle.id, gen: (handle.gen + 1) & 0xff });
        return true;
    }

    /** Undo an alloc whose Spawn never reached the ring: Rust never saw the generation, so do not bump it. */
    release(handle: EntityHandle): boolean {
        if (!this.isLive(handle)) return false;
        this.liveGen[handle.id] = NOT_LIVE;
        this.freeList.push({ id: handle.id, gen: handle.gen });
        return true;
    }

    /**
     * Rebuild from the runtime's tables after a seek rewrote which entities exist. Every id below the allocation counter
     * that is not live becomes free again with the generation the runtime expects next; handles issued before the seek
     * for entities that no longer exist stop being live.
     */
    resync(generations: Uint8Array, live: Uint8Array): void {
        this.liveGen.fill(NOT_LIVE);
        this.freeList = [];
        const allocated = Math.min(Atomics.load(this.counter, 0), generations.length, live.length);
        for (let id = allocated - 1; id >= 0; id--) {
            if (live[id]) this.liveGen[id] = generations[id];
            else this.freeList.push({ id, gen: generations[id] });
        }
    }

    setCapacity(capacity: number): void {
        if (capacity <= this.capacity) return;
        const live = new Int16Array(capacity).fill(NOT_LIVE);
        live.set(this.liveGen);
        this.liveGen = live;
        this.capacity = capacity;
    }

    get capacityNow(): number { return this.capacity; }

    get allocated(): number {
        return Atomics.load(this.counter, 0);
    }

    reacquire(memory: WebAssembly.Memory, counterByteOffset: number): void {
        this.counter = new Uint32Array(memory.buffer, counterByteOffset, 1);
    }
}
