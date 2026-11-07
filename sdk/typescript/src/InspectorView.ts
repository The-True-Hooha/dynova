import {
    H_CAPACITY, H_CONTACTS, H_DROPPED_CONTACTS, H_EPOCH, H_INSPECT_BODIES, H_INSPECT_CONTACTS, H_INSPECT_CONTACT_N, H_INSPECT_SEQ,
    H_INSPECT_ON, H_INSPECT_TICK, H_OVERRUNS, H_RING_PENDING, H_STEP_LAST_US, H_STEP_TOTAL_US, H_TICKS, HEADER_WORDS,
} from './TransformView.js';

export const BODY_STATE_INACTIVE = 0;
export const BODY_STATE_AWAKE = 1;
export const BODY_STATE_SLEEPING = 2;
export const BODY_STATE_STATIC = 3;

const BODY_WORDS = 14;
const CONTACT_WORDS = 9;
const STATE_WORD = BODY_WORDS - 1;
const COPY_ATTEMPTS = 8;

export interface InspectedBody {
    id: number;
    state: number;
    position: [number, number, number];
    rotation: [number, number, number, number];
    velocity: [number, number, number];
    angularVelocity: [number, number, number];
}

export interface InspectedContact {
    point: [number, number, number];
    normal: [number, number, number];
    depth: number;
    a: number;
    b: number;
}

/** A whole frame copied out of shared memory, so it stays valid while the simulation keeps running. */
export interface InspectorSnapshot {
    /** Simulation tick this frame was written at. */
    tick: number;
    capacity: number;
    contactCount: number;
    body(id: number): InspectedBody | null;
    contact(index: number): InspectedContact;
    /** Ids of entities in the given state, ascending. */
    idsInState(state: number): number[];
}

export interface ProfilerSample {
    ticks: number;
    /** Duration of the most recent step, from the thread that drives `step()`. */
    stepLastMicros: number;
    /** Wrapping sum of step durations; subtract two samples (mod 2^32) and divide by the tick delta for an average. */
    stepTotalMicros: number;
    ringPending: number;
    contacts: number;
    droppedContacts: number;
    workerOverruns: number;
    memoryBytes: number;
    /** Buffer reallocations so far (epoch / 2). */
    growths: number;
    /** Ticks since the snapshot was written; `null` while the inspector is off or has not written a frame. */
    snapshotAgeTicks: number | null;
}

function makeSnapshot(tick: number, capacity: number, bodies: ArrayBuffer, contactCount: number, contacts: ArrayBuffer): InspectorSnapshot {
    const f = new Float32Array(bodies);
    const u = new Uint32Array(bodies);
    const cf = new Float32Array(contacts);
    const cu = new Uint32Array(contacts);
    return {
        tick,
        capacity,
        contactCount,
        body(id) {
            if (!Number.isInteger(id) || id < 0 || id >= capacity) return null;
            const o = id * BODY_WORDS;
            return {
                id,
                state: u[o + STATE_WORD],
                position: [f[o], f[o + 1], f[o + 2]],
                rotation: [f[o + 3], f[o + 4], f[o + 5], f[o + 6]],
                velocity: [f[o + 7], f[o + 8], f[o + 9]],
                angularVelocity: [f[o + 10], f[o + 11], f[o + 12]],
            };
        },
        contact(index) {
            if (!Number.isInteger(index) || index < 0 || index >= contactCount) throw new RangeError(`contact ${index} out of range 0..${contactCount}`);
            const o = index * CONTACT_WORDS;
            return { point: [cf[o], cf[o + 1], cf[o + 2]], normal: [cf[o + 3], cf[o + 4], cf[o + 5]], depth: cf[o + 6], a: cu[o + 7], b: cu[o + 8] };
        },
        idsInState(state) {
            const ids: number[] = [];
            for (let id = 0; id < capacity; id++) if (u[id * BODY_WORDS + STATE_WORD] === state) ids.push(id);
            return ids;
        },
    };
}

/**
 * Read-only reader of the runtime's debug snapshot. It touches only shared memory (never the command ring or the
 * transform triple buffer), so it can sit beside the game without affecting it. Turn the snapshot on with
 * `world.setInspector(true)` first.
 */
export class InspectorView {
    private header: Int32Array;
    /** Frames that kept changing while being copied; the caller got `null` for these. */
    missedReads = 0;

    constructor(private readonly memory: WebAssembly.Memory, private readonly headerByteOffset: number) {
        this.header = new Int32Array(memory.buffer, headerByteOffset, HEADER_WORDS);
    }

    // Any wasm allocation can grow memory and detach every view onto the old buffer.
    private live(): void {
        if (this.header.buffer !== this.memory.buffer) this.header = new Int32Array(this.memory.buffer, this.headerByteOffset, HEADER_WORDS);
    }

    /** `null` while the inspector is off, has not written a frame yet, or a consistent copy could not be taken. */
    snapshot(): InspectorSnapshot | null {
        for (let attempt = 0; attempt < COPY_ATTEMPTS; attempt++) {
            this.live();
            const h = this.header;
            const epoch = Atomics.load(h, H_EPOCH);
            const seq = Atomics.load(h, H_INSPECT_SEQ);
            if ((epoch & 1) || (seq & 1)) continue;
            const bodiesPtr = Atomics.load(h, H_INSPECT_BODIES);
            const contactsPtr = Atomics.load(h, H_INSPECT_CONTACTS);
            if (Atomics.load(h, H_INSPECT_ON) === 0 || bodiesPtr === 0 || seq === 0) return null;
            const capacity = Atomics.load(h, H_CAPACITY);
            const count = Atomics.load(h, H_INSPECT_CONTACT_N);
            const tick = Atomics.load(h, H_INSPECT_TICK) >>> 0;
            const bodies = this.memory.buffer.slice(bodiesPtr, bodiesPtr + capacity * BODY_WORDS * 4);
            const contacts = this.memory.buffer.slice(contactsPtr, contactsPtr + count * CONTACT_WORDS * 4);
            // A read-modify-write is a full barrier: the plain copies above cannot be reordered after this check.
            if (Atomics.add(h, H_INSPECT_SEQ, 0) === seq && Atomics.load(h, H_EPOCH) === epoch && Atomics.load(h, H_INSPECT_BODIES) === bodiesPtr) {
                return makeSnapshot(tick, capacity, bodies, count, contacts);
            }
        }
        this.missedReads++;
        return null;
    }

    profile(): ProfilerSample {
        this.live();
        const h = this.header;
        const ticks = Atomics.load(h, H_TICKS) >>> 0;
        const stamp = Atomics.load(h, H_INSPECT_TICK) >>> 0;
        const enabled = Atomics.load(h, H_INSPECT_ON) !== 0 && Atomics.load(h, H_INSPECT_BODIES) !== 0 && Atomics.load(h, H_INSPECT_SEQ) !== 0;
        return {
            ticks,
            stepLastMicros: Atomics.load(h, H_STEP_LAST_US) >>> 0,
            stepTotalMicros: Atomics.load(h, H_STEP_TOTAL_US) >>> 0,
            ringPending: Atomics.load(h, H_RING_PENDING) >>> 0,
            contacts: Atomics.load(h, H_CONTACTS) >>> 0,
            droppedContacts: Atomics.load(h, H_DROPPED_CONTACTS) >>> 0,
            workerOverruns: Atomics.load(h, H_OVERRUNS) >>> 0,
            memoryBytes: this.memory.buffer.byteLength,
            growths: Atomics.load(h, H_EPOCH) >>> 1,
            snapshotAgeTicks: enabled ? ((ticks - stamp) >>> 0) : null,
        };
    }
}
