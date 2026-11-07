import { describe, it, expect } from 'vitest';
import { TransformView, H_EPOCH, H_STATE, H_TICKS, H_STALE, H_MALFORMED, H_BUCKETS, H_BUCKET_LEN, FLOATS_PER_TRANSFORM } from '../TransformView.js';
import { FakeRuntime } from './helpers.js';

function setup(capacity = 4) {
    const memory = new WebAssembly.Memory({ initial: 8, maximum: 32, shared: true });
    const rt = new FakeRuntime(memory, { capacity, ringCapacity: 4 });
    const view = new TransformView(memory, rt.layout.headerByteOffset);
    return { memory, rt, view };
}

const t = (x: number) => [x, 0, 0, 0, 0, 0, 1];

describe('TransformView (triple buffer reader)', () => {
    it('reads zeros before anything is published', () => {
        const { view } = setup();
        expect(Array.from(view.read())).toEqual(new Array(4 * FLOATS_PER_TRANSFORM).fill(0));
    });

    it('returns the latest published frame and keeps returning it when nothing new arrives', () => {
        const { rt, view } = setup();
        rt.writeTransform(0, t(5));
        rt.publish();
        expect(view.read()[0]).toBe(5);
        expect(view.read()[0]).toBe(5);
        rt.writeTransform(0, t(6));
        rt.publish();
        expect(view.read()[0]).toBe(6);
    });

    it('writes before publish are invisible', () => {
        const { rt, view } = setup();
        rt.writeTransform(0, t(9));
        expect(view.read()[0]).toBe(0);
    });

    it('the array a reader holds is never mutated by later publishes (until its next read)', () => {
        const { rt, view } = setup();
        rt.writeTransform(0, t(1));
        rt.publish();
        const held = view.read();
        const snapshot = Array.from(held);
        for (let i = 2; i < 50; i++) {
            rt.writeTransform(0, t(i));
            rt.publish();
        }
        expect(Array.from(held)).toEqual(snapshot);
    });

    it('roles stay a permutation under random interleavings of publish and read', () => {
        const { rt, view } = setup();
        let seed = 12345;
        for (let i = 0; i < 5000; i++) {
            seed = (Math.imul(seed, 1664525) + 1013904223) >>> 0;
            if (seed >>> 29 < 4) { rt.writeTransform(1, t(i)); rt.publish(); } else { view.read(); }
            const s = Atomics.load(rt.header, H_STATE);
            expect([rt.write, s & 3, (s >> 3) & 3].sort()).toEqual([0, 1, 2]);
        }
    });

    it('no read ever observes a mixed frame while the writer republishes constantly', () => {
        const { rt, view } = setup(64);
        for (let frame = 1; frame <= 400; frame++) {
            for (let id = 0; id < 64; id++) rt.writeTransform(id, t(frame));
            rt.publish();
            if (frame % 3 === 0) {
                const buf = view.read();
                const first = buf[0];
                for (let id = 0; id < 64; id++) expect(buf[id * FLOATS_PER_TRANSFORM]).toBe(first);
            }
        }
    });

    it('growth: epoch and capacity update, data survives, new slots are empty', () => {
        const { rt, view } = setup(2);
        rt.writeTransform(1, t(7));
        rt.publish();
        expect(view.peekEpoch()).toBe(0);
        rt.grow(5);
        expect(view.peekEpoch()).toBe(2);
        const buf = view.read();
        expect(view.epoch).toBe(2);
        expect(view.capacity).toBe(5);
        expect(buf.length).toBe(5 * FLOATS_PER_TRANSFORM);
        expect(buf[1 * FLOATS_PER_TRANSFORM]).toBe(7);
        expect(buf[4 * FLOATS_PER_TRANSFORM + 6]).toBe(0);
    });

    it('growth between acquiring a buffer and validating the epoch forces a retry on the new buffers', () => {
        const { rt, view } = setup(2);
        rt.writeTransform(0, t(1));
        rt.publish();
        const realLoad = Atomics.load;
        let armed = true;
        let loads = 0;
        Atomics.load = ((a: Int32Array, i: number) => {
            const v = realLoad(a, i);
            if (armed && i === 0 && ++loads === 1) { armed = false; rt.writeTransform(0, t(2)); rt.publish(); rt.grow(6); }
            return v;
        }) as typeof Atomics.load;
        try {
            const buf = view.read();
            expect(buf.length).toBe(6 * FLOATS_PER_TRANSFORM);
            expect(view.epoch).toBe(2);
            expect(buf[0]).toBe(2);
        } finally {
            Atomics.load = realLoad;
        }
    });

    it('reacquire retries when the writer grows again while views are being built', () => {
        const { rt, view } = setup(2);
        rt.grow(3);
        const realLoad = Atomics.load;
        let fired = false;
        Atomics.load = ((a: Int32Array, i: number) => {
            const v = realLoad(a, i);
            if (!fired && i === 1) { fired = true; rt.grow(4); }
            return v;
        }) as typeof Atomics.load;
        try {
            view.reacquire();
            expect(view.epoch).toBe(4);
            expect(view.capacity).toBe(4);
        } finally {
            Atomics.load = realLoad;
        }
    });

    it('readEntity decodes position and rotation', () => {
        const { rt, view } = setup();
        rt.writeTransform(2, [1, 2, 3, 0.1, 0.2, 0.3, 0.9]);
        rt.publish();
        expect(view.readEntity(2)).toMatchObject({ x: 1, y: 2, z: 3, qw: expect.closeTo(0.9) });
    });

    it('counters read the ticks and drop counters from the header', () => {
        const { rt, view } = setup();
        expect(view.counters()).toEqual({ ticks: 0, staleDropped: 0, malformedDropped: 0 });
        Atomics.store(rt.header, H_TICKS, 7);
        Atomics.store(rt.header, H_STALE, 2);
        Atomics.store(rt.header, H_MALFORMED, 3);
        expect(view.counters()).toEqual({ ticks: 7, staleDropped: 2, malformedDropped: 3 });
    });

    it('counters treat a wrapped (negative int32) tick count as unsigned', () => {
        const { rt, view } = setup();
        Atomics.store(rt.header, H_TICKS, -1);
        expect(view.counters().ticks).toBe(0xffffffff);
    });

    it('an odd epoch (grow in progress) is never accepted: read() waits for it to turn even', () => {
        const { rt, view } = setup(2);
        rt.writeTransform(0, t(3));
        rt.publish();
        Atomics.store(rt.header, H_EPOCH, 1);
        const realLoad = Atomics.load;
        let polls = 0;
        Atomics.load = ((a: Int32Array, i: number) => {
            if (i === 0 && ++polls === 20) Atomics.store(rt.header, H_EPOCH, 2);
            return realLoad(a, i);
        }) as typeof Atomics.load;
        try {
            expect(view.read()[0]).toBe(3);
            expect(polls).toBeGreaterThanOrEqual(20);
            expect(view.epoch).toBe(2);
        } finally {
            Atomics.load = realLoad;
        }
    });

    it('a grow that never completes makes read() and reacquire() throw instead of hanging', () => {
        const { rt, view } = setup(2);
        Atomics.store(rt.header, H_EPOCH, 1);
        expect(() => view.read()).toThrow('layout update stuck');
        expect(() => view.reacquire()).toThrow('layout update stuck');
    });

    it('bucketCounts is a snapshot of the shared occupancy histogram, not a live view', () => {
        const { memory, rt, view } = setup();
        const at = 4096;
        new Uint32Array(memory.buffer, at, 4).set([0, 3, 0, 7]);
        Atomics.store(rt.header, H_BUCKETS, at);
        Atomics.store(rt.header, H_BUCKET_LEN, 4);
        const counts = view.bucketCounts();
        expect(Array.from(counts)).toEqual([0, 3, 0, 7]);
        new Uint32Array(memory.buffer, at, 4)[2] = 9;
        expect(counts[2]).toBe(0);
        expect(view.bucketCounts()[2]).toBe(9);
    });

    it('bucketCounts throws instead of reading a half-published layout', () => {
        const { rt, view } = setup(2);
        Atomics.store(rt.header, H_EPOCH, 1);
        expect(() => view.bucketCounts()).toThrow('layout update stuck');
    });
});
