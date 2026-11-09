export const FLOATS_PER_TRANSFORM = 7;

export const H_EPOCH = 0;
export const H_CAPACITY = 1;
export const H_BUF0 = 2;
export const H_STATE = 5;
export const H_TICKS = 7;
export const H_STALE = 8;
export const H_MALFORMED = 9;
export const H_BUCKETS = 10;
export const H_BUCKET_LEN = 11;
export const H_RING_PENDING = 12;
export const H_DROPPED_CONTACTS = 13;
export const H_OVERRUNS = 14;
export const H_CONTACTS = 15;
export const H_INSPECT_SEQ = 16;
export const H_INSPECT_BODIES = 17;
export const H_INSPECT_CONTACTS = 18;
export const H_INSPECT_CONTACT_N = 19;
export const H_STEP_LAST_US = 20;
export const H_STEP_TOTAL_US = 21;
export const H_INSPECT_TICK = 22;
export const H_INSPECT_ON = 23;
export const H_REC_ON = 24;
export const H_REC_START = 25;
export const H_REC_END = 26;
export const H_HASH_LO = 27;
export const H_HASH_HI = 28;
export const H_KEYFRAMES = 29;
export const HEADER_WORDS = 30;

const MAX_SPINS = 5_000_000;
const NEW_FLAG = 0b100;
const READER_SHIFT = 3;

export class TransformView {
    private header: Int32Array;
    private bufs: Float32Array[] = [];
    private memory: WebAssembly.Memory;
    private headerByteOffset: number;
    private _epoch = -1;
    private _capacity = 0;

    constructor(memory: WebAssembly.Memory, headerByteOffset: number) {
        this.memory = memory;
        this.headerByteOffset = headerByteOffset;
        this.header = new Int32Array(memory.buffer, headerByteOffset, HEADER_WORDS);
        this.reacquire();
    }

    get epoch(): number { return this._epoch; }
    get capacity(): number { return this._capacity; }

    // Any wasm allocation can grow memory and detach every view onto the old buffer, not only an explicit Grow command.
    private live(): void {
        if (this.header.buffer !== this.memory.buffer) this.reacquire();
    }

    /** Called by whoever owns the clock and drives `step()`; the runtime itself cannot time anything on wasm. */
    recordStep(micros: number): void {
        this.live();
        const us = Number.isFinite(micros) ? Math.max(0, Math.min(0x7fffffff, Math.round(micros))) : 0;
        Atomics.store(this.header, H_STEP_LAST_US, us);
        Atomics.add(this.header, H_STEP_TOTAL_US, us);
    }

    stats(): { ringPending: number; droppedContacts: number; workerOverruns: number; contacts: number } {
        this.live();
        return {
            ringPending: Atomics.load(this.header, H_RING_PENDING) >>> 0,
            droppedContacts: Atomics.load(this.header, H_DROPPED_CONTACTS) >>> 0,
            workerOverruns: Atomics.load(this.header, H_OVERRUNS) >>> 0,
            contacts: Atomics.load(this.header, H_CONTACTS) >>> 0,
        };
    }

    /** Copy of the occupancy histogram; the runtime frees and reallocates it when capacity grows, so a live view could go stale. */
    bucketCounts(): Uint32Array {
        this.live();
        for (let spins = 0; ; spins++) {
            if (spins > MAX_SPINS) throw new Error('layout update stuck: the runtime never finished growing its buffers');
            const e = Atomics.load(this.header, H_EPOCH);
            if (e & 1) continue;
            const view = new Uint32Array(this.memory.buffer, Atomics.load(this.header, H_BUCKETS), Atomics.load(this.header, H_BUCKET_LEN));
            const copy = view.slice();
            if (Atomics.load(this.header, H_EPOCH) === e) return copy;
        }
    }

    counters(): { ticks: number; staleDropped: number; malformedDropped: number } {
        this.live();
        return {
            ticks: Atomics.load(this.header, H_TICKS) >>> 0,
            staleDropped: Atomics.load(this.header, H_STALE) >>> 0,
            malformedDropped: Atomics.load(this.header, H_MALFORMED) >>> 0,
        };
    }

    /**
     * Epoch the writer has published; differs from `epoch` when buffers were reallocated. The epoch is a
     * seqlock: odd while the runtime rewrites the layout, even when stable. Growth number = epoch / 2.
     */
    peekEpoch(): number {
        this.live();
        return Atomics.load(this.header, H_EPOCH);
    }

    reacquire(): void {
        for (let spins = 0; ; spins++) {
            if (spins > MAX_SPINS) throw new Error('layout update stuck: the runtime never finished growing its buffers');
            this.header = new Int32Array(this.memory.buffer, this.headerByteOffset, HEADER_WORDS);
            const epoch = Atomics.load(this.header, H_EPOCH);
            if (epoch & 1) continue;
            const capacity = Atomics.load(this.header, H_CAPACITY);
            const len = capacity * FLOATS_PER_TRANSFORM;
            const bufs = [0, 1, 2].map(
                (i) => new Float32Array(this.memory.buffer, Atomics.load(this.header, H_BUF0 + i), len),
            );
            if (Atomics.load(this.header, H_EPOCH) !== epoch) continue;
            this.bufs = bufs;
            this._epoch = epoch;
            this._capacity = capacity;
            return;
        }
    }

    private acquire(): number {
        for (;;) {
            const cur = Atomics.load(this.header, H_STATE);
            if ((cur & NEW_FLAG) === 0) return (cur >> READER_SHIFT) & 3;
            const next = ((cur >> READER_SHIFT) & 3) | ((cur & 3) << READER_SHIFT);
            if (Atomics.compareExchange(this.header, H_STATE, cur, next) === cur) return (next >> READER_SHIFT) & 3;
        }
    }

    /**
     * Latest published frame. Single consumer: the returned array is valid until the next `read()`,
     * after which the writer may reuse it, and until at most two buffer growths have happened.
     * Never returns a half-written frame to that consumer.
     */
    read(): Float32Array {
        this.live();
        for (let spins = 0; ; spins++) {
            if (spins > MAX_SPINS) throw new Error('layout update stuck: the runtime never finished growing its buffers');
            const e = Atomics.load(this.header, H_EPOCH);
            if (e & 1) continue;
            if (e !== this._epoch) this.reacquire();
            const idx = this.acquire();
            if (Atomics.load(this.header, H_EPOCH) === this._epoch) return this.bufs[idx];
        }
    }

    readEntity(entityId: number): { x: number; y: number; z: number; qx: number; qy: number; qz: number; qw: number } {
        const buf = this.read();
        const o = entityId * FLOATS_PER_TRANSFORM;
        return { x: buf[o], y: buf[o + 1], z: buf[o + 2], qx: buf[o + 3], qy: buf[o + 4], qz: buf[o + 5], qw: buf[o + 6] };
    }
}
