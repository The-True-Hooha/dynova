import { describe, expect, it, vi } from 'vitest';
import {
    H_CAPACITY, H_CONTACTS, H_DROPPED_CONTACTS, H_EPOCH, H_INSPECT_BODIES, H_INSPECT_CONTACTS, H_INSPECT_CONTACT_N, H_INSPECT_SEQ,
    H_INSPECT_ON, H_INSPECT_TICK, H_OVERRUNS, H_RING_PENDING, H_STEP_LAST_US, H_STEP_TOTAL_US, H_TICKS, HEADER_WORDS, TransformView,
} from '../TransformView.js';
import { BODY_STATE_AWAKE, BODY_STATE_INACTIVE, BODY_STATE_SLEEPING, BODY_STATE_STATIC, InspectorView } from '../InspectorView.js';
import { TAG_INSPECT } from '../CommandRing.js';
import { layoutFromRuntime } from '../World.js';
import { FakeRuntime, makeSyncWorld, makeWorld } from './helpers.js';

const BODY_WORDS = 14;
const CONTACT_WORDS = 9;

function setup(capacity = 4, shared = true) {
    const memory = new WebAssembly.Memory({ initial: 8, maximum: 32, shared });
    const headerByteOffset = 0;
    const header = new Int32Array(memory.buffer, headerByteOffset, HEADER_WORDS);
    const bodiesAt = 1024;
    const contactsAt = bodiesAt + capacity * BODY_WORDS * 4 + 64;
    const bodies = new Float32Array(memory.buffer, bodiesAt, capacity * BODY_WORDS);
    const bodyBits = new Uint32Array(memory.buffer, bodiesAt, capacity * BODY_WORDS);
    const contacts = new Float32Array(memory.buffer, contactsAt, 16 * CONTACT_WORDS);
    const contactBits = new Uint32Array(memory.buffer, contactsAt, 16 * CONTACT_WORDS);
    Atomics.store(header, H_CAPACITY, capacity);
    Atomics.store(header, H_INSPECT_BODIES, bodiesAt);
    Atomics.store(header, H_INSPECT_CONTACTS, contactsAt);
    Atomics.store(header, H_INSPECT_ON, 1);
    const body = (id: number, vals: number[], state: number) => {
        bodies.set(vals, id * BODY_WORDS);
        bodyBits[id * BODY_WORDS + BODY_WORDS - 1] = state;
    };
    const contact = (k: number, vals: number[], a: number, b: number) => {
        contacts.set(vals, k * CONTACT_WORDS);
        contactBits[k * CONTACT_WORDS + 7] = a;
        contactBits[k * CONTACT_WORDS + 8] = b;
    };
    const publish = (tick: number, contactCount: number) => {
        Atomics.store(header, H_INSPECT_CONTACT_N, contactCount);
        Atomics.store(header, H_INSPECT_TICK, tick);
        Atomics.add(header, H_INSPECT_SEQ, 1);
        Atomics.add(header, H_INSPECT_SEQ, 1);
    };
    return { memory, header, view: new InspectorView(memory, headerByteOffset), body, contact, publish };
}

describe('InspectorView', () => {
    it('returns null before the inspector was ever enabled and reports no snapshot age', () => {
        const memory = new WebAssembly.Memory({ initial: 1, maximum: 4, shared: true });
        const view = new InspectorView(memory, 0);
        expect(view.snapshot()).toBeNull();
        expect(view.missedReads).toBe(0);
        expect(view.profile().snapshotAgeTicks).toBeNull();
    });

    it('returns null while the snapshot is allocated but no frame has been written yet', () => {
        const { view, header } = setup();
        expect(Atomics.load(header, H_INSPECT_SEQ)).toBe(0);
        expect(view.snapshot()).toBeNull();
        expect(view.profile().snapshotAgeTicks).toBeNull();
    });

    it('returns null and no snapshot age while the inspector is switched off, even with an old frame in memory', () => {
        const { view, header, publish } = setup();
        publish(5, 0);
        expect(view.snapshot()).not.toBeNull();
        Atomics.store(header, H_INSPECT_ON, 0);
        expect(view.snapshot()).toBeNull();
        expect(view.profile().snapshotAgeTicks).toBeNull();
        expect(view.missedReads).toBe(0);
    });

    it('decodes bodies, states and contacts from one published frame', () => {
        const { view, body, contact, publish } = setup();
        body(0, [0, -0.5, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0], BODY_STATE_STATIC);
        body(1, [1, 2, 3, 0.1, 0.2, 0.3, 0.9, 4, 5, 6, 7, 8, 9], BODY_STATE_AWAKE);
        body(2, [0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0], BODY_STATE_SLEEPING);
        contact(0, [1, 0, 3, 0, 1, 0, 0.25], 1, 0);
        contact(1, [2, 0, 3, 1, 0, 0, 0.5], 2, 1);
        publish(7, 2);
        const snap = view.snapshot()!;
        expect(snap.tick).toBe(7);
        expect(snap.capacity).toBe(4);
        expect(snap.contactCount).toBe(2);
        const b = snap.body(1)!;
        expect(b.position).toEqual([1, 2, 3]);
        expect(b.rotation[3]).toBeCloseTo(0.9, 6);
        expect(b.velocity).toEqual([4, 5, 6]);
        expect(b.angularVelocity).toEqual([7, 8, 9]);
        expect(snap.body(0)!.state).toBe(BODY_STATE_STATIC);
        expect(snap.body(3)!.state).toBe(BODY_STATE_INACTIVE);
        expect(snap.idsInState(BODY_STATE_AWAKE)).toEqual([1]);
        expect(snap.idsInState(BODY_STATE_INACTIVE)).toEqual([3]);
        expect(snap.contact(1)).toEqual({ point: [2, 0, 3], normal: [1, 0, 0], depth: 0.5, a: 2, b: 1 });
    });

    it('rejects out-of-range ids and contact indexes', () => {
        const { view, publish } = setup();
        publish(1, 0);
        const snap = view.snapshot()!;
        for (const id of [-1, 4, 1.5, NaN, Infinity]) expect(snap.body(id)).toBeNull();
        expect(() => snap.contact(0)).toThrow(RangeError);
        expect(() => snap.contact(-1)).toThrow(RangeError);
    });

    it('returns a copy that does not change when the simulation writes the next frame', () => {
        const { view, body, publish } = setup();
        body(1, [1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0], BODY_STATE_AWAKE);
        publish(1, 0);
        const snap = view.snapshot()!;
        body(1, [9, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0], BODY_STATE_AWAKE);
        expect(snap.body(1)!.position[0]).toBe(1);
        publish(2, 0);
        expect(view.snapshot()!.body(1)!.position[0]).toBe(9);
    });

    it('gives up with null and counts a missed read while the writer is mid-frame', () => {
        const { view, header, publish } = setup();
        publish(1, 0);
        Atomics.add(header, H_INSPECT_SEQ, 1);
        expect(view.snapshot()).toBeNull();
        expect(view.missedReads).toBe(1);
        Atomics.add(header, H_INSPECT_SEQ, 1);
        expect(view.snapshot()).not.toBeNull();
    });

    it.each([['the frame sequence', H_INSPECT_SEQ], ['the layout epoch', H_EPOCH]] as const)(
        'retries when %s changes while it is copying and never returns a half-old, half-new frame',
        (_label, word) => {
            const { memory, header, body, contact, publish } = setup();
            body(1, [1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0], BODY_STATE_AWAKE);
            contact(0, [1, 0, 0, 0, 1, 0, 0.1], 1, 0);
            publish(1, 1);
            let reads = 0;
            let tampered = false;
            const spy = {
                get buffer() {
                    reads++;
                    // Third read is the bodies copy of the first attempt, fourth is its contacts copy.
                    if (reads === 4 && !tampered) {
                        tampered = true;
                        body(1, [2, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0], BODY_STATE_AWAKE);
                        contact(0, [2, 0, 0, 0, 1, 0, 0.1], 1, 0);
                        Atomics.add(header, word, 2);
                    }
                    return memory.buffer;
                },
            } as unknown as WebAssembly.Memory;
            const view = new InspectorView(spy, 0);
            const snap = view.snapshot()!;
            expect(tampered).toBe(true);
            expect(snap.body(1)!.position[0]).toBe(2);
            expect(snap.contact(0).point[0]).toBe(2);
            expect(view.missedReads).toBe(0);
        },
    );

    it('keeps working after memory growth detaches the old buffer, shared or not', () => {
        for (const shared of [false, true]) {
            const { memory, view, body, publish } = setup(4, shared);
            body(1, [5, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0], BODY_STATE_AWAKE);
            publish(3, 0);
            memory.grow(1);
            expect(view.snapshot()!.body(1)!.position[0]).toBe(5);
            expect(view.profile().memoryBytes).toBe(memory.buffer.byteLength);
        }
    });

    it('profile reads every counter and derives growths and snapshot age', () => {
        const { header, view, publish } = setup();
        Atomics.store(header, H_TICKS, 120);
        Atomics.store(header, H_STEP_LAST_US, 1500);
        Atomics.store(header, H_STEP_TOTAL_US, 90000);
        Atomics.store(header, H_RING_PENDING, 3);
        Atomics.store(header, H_CONTACTS, 40);
        Atomics.store(header, H_DROPPED_CONTACTS, 2);
        Atomics.store(header, H_OVERRUNS, 1);
        Atomics.store(header, H_EPOCH, 6);
        publish(118, 0);
        expect(view.profile()).toMatchObject({
            ticks: 120, stepLastMicros: 1500, stepTotalMicros: 90000, ringPending: 3, contacts: 40, droppedContacts: 2,
            workerOverruns: 1, growths: 3, snapshotAgeTicks: 2,
        });
    });

    it('profile counters above 2^31 stay unsigned and the snapshot age wraps', () => {
        const { header, view, publish } = setup();
        Atomics.store(header, H_TICKS, 3);
        Atomics.store(header, H_STEP_TOTAL_US, -2);
        publish(0xfffffffe | 0, 0);
        const p = view.profile();
        expect(p.stepTotalMicros).toBe(0xfffffffe);
        expect(p.snapshotAgeTicks).toBe(5);
    });
});

describe('header size handshake', () => {
    const ptrs = (words?: number) => ({
        header_ptr: () => 1, ring_slots_ptr: () => 2, ring_head_ptr: () => 3, ring_tail_ptr: () => 4, id_counter_ptr: () => 5,
        ...(words === undefined ? {} : { header_words: () => words }),
    });

    it('accepts a matching wasm build and a runtime that does not report its size', () => {
        expect(layoutFromRuntime(ptrs(HEADER_WORDS)).headerByteOffset).toBe(1);
        expect(layoutFromRuntime(ptrs()).headerByteOffset).toBe(1);
    });

    it('rejects a stale wasm build with a message that says how to fix it', () => {
        expect(() => layoutFromRuntime(ptrs(16))).toThrow(/rebuild the wasm package/);
        expect(() => layoutFromRuntime(ptrs(HEADER_WORDS + 1))).toThrow(/header has \d+ words/);
    });
});

describe('step timing and the Inspect command', () => {
    it('recordStep stores the last duration and a wrapping total, and ignores nonsense', () => {
        const memory = new WebAssembly.Memory({ initial: 8, maximum: 32, shared: true });
        const rt = new FakeRuntime(memory, { capacity: 2, ringCapacity: 4 });
        const view = new TransformView(memory, rt.layout.headerByteOffset);
        view.recordStep(1500.4);
        view.recordStep(500);
        expect(Atomics.load(rt.header, H_STEP_LAST_US)).toBe(500);
        expect(Atomics.load(rt.header, H_STEP_TOTAL_US)).toBe(2000);
        for (const bad of [NaN, -5, Infinity]) {
            view.recordStep(bad);
            expect(Atomics.load(rt.header, H_STEP_LAST_US)).toBe(0);
        }
        expect(Atomics.load(rt.header, H_STEP_TOTAL_US)).toBe(2000);
    });

    it('World.step times the driver and records microseconds', () => {
        const { world, memory, rt } = makeSyncWorld({ capacity: 2, ringCapacity: 4 }, () => {});
        const header = new Int32Array(memory.buffer, rt.layout.headerByteOffset, HEADER_WORDS);
        const now = vi.spyOn(performance, 'now');
        now.mockReturnValueOnce(1000).mockReturnValueOnce(1002.5);
        world.step();
        now.mockRestore();
        expect(Atomics.load(header, H_STEP_LAST_US)).toBe(2500);
        expect(Atomics.load(header, H_STEP_TOTAL_US)).toBe(2500);
    });

    it('setInspector writes tag 7 with the on flag through the command ring', () => {
        const { world, memory, layout } = makeWorld({ capacity: 2, ringCapacity: 4 });
        world.setInspector(true);
        world.setInspector(false);
        const bytes = new Uint8Array(memory.buffer, layout.ringSlotsByteOffset, 128);
        expect([bytes[0], bytes[6], bytes[64], bytes[70]]).toEqual([TAG_INSPECT, 1, TAG_INSPECT, 0]);
    });
});
